#include "mss_ros2/mss_node.hpp"

#include <cmath>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mss/common/logger.hpp"
#include "mss/pipeline/config.hpp"

namespace mss_ros2 {

namespace {

constexpr const char* kTopicNames[] = {"lidar", "imu", "gnss", "odom", "map"};
constexpr double kMaxAbsLatitudeDeg = 90.0;    // WGS-84 domain
constexpr double kMaxAbsLongitudeDeg = 180.0;  // WGS-84 domain

std::chrono::nanoseconds to_period(double seconds) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
}

void require(bool ok, const std::string& what) {
  if (!ok) {
    throw std::invalid_argument("mss_node parameter " + what);
  }
}

}  // namespace

MssNode::MssNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mss_node", options), core_logger_(get_logger()) {
  mss::common::set_logger(&core_logger_);
  params_ = read_params();

  std::optional<std::filesystem::path> overlay;
  if (!params_.config_overlay.empty()) {
    overlay = std::filesystem::path(params_.config_overlay);
  }
  const auto cfg = mss::pipeline::load_config(params_.config_path, overlay);
  if (!cfg) {
    throw std::invalid_argument("config error (" + params_.config_path + "):\n" + cfg.reason());
  }
  if (params_.frames.body.empty()) {
    params_.frames.body = cfg.value().frames.body;  // the rig's b frame (ADR-024 section 3)
  }
  require(!params_.frames.body.empty() && params_.frames.body != params_.frames.world,
          "tf.body_frame: empty in the parameter and in frames.body, or equal to tf.world_frame");
  est_ = std::make_unique<mss::pipeline::Estimator>(cfg.value());
  scan_.reserve(cfg.value().realtime.max_points_per_scan);
  gnss_sigma_.sigma_h_m = cfg.value().gnss.sigma_h_m;
  gnss_sigma_.sigma_v_m = cfg.value().gnss.sigma_v_m;
  if (!params_.origin_from_first_fix) {
    origin_.lat_deg = params_.origin_latitude_deg;
    origin_.lon_deg = params_.origin_longitude_deg;
    origin_.alt_m = params_.origin_altitude_m;
    has_origin_ = true;
  }
  path_.header.frame_id = params_.frames.world;

  using std::placeholders::_1;
  const auto sensor_qos = [](int depth) {
    return rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(depth))).best_effort();
  };
  sub_lidar_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(params_.topic_lidar,
                                                         sensor_qos(params_.qos_depth_lidar),
                                                         std::bind(&MssNode::on_lidar, this, _1));
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.topic_imu, sensor_qos(params_.qos_depth_imu), std::bind(&MssNode::on_imu, this, _1));
  sub_gnss_ =
      create_subscription<sensor_msgs::msg::NavSatFix>(params_.topic_gnss,
                                                       sensor_qos(params_.qos_depth_gnss),
                                                       std::bind(&MssNode::on_gnss, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(params_.topic_odom,
                                                           sensor_qos(params_.qos_depth_odom),
                                                           std::bind(&MssNode::on_odom, this, _1));

  constexpr std::size_t kOdomDepth = 10;    // volatile odometry stream
  constexpr std::size_t kLatchedDepth = 1;  // latest path / map only
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/mss/odometry", rclcpp::QoS(kOdomDepth));
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
      "/mss/path", rclcpp::QoS(kLatchedDepth).reliable().transient_local());
  pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mss/map", rclcpp::QoS(kLatchedDepth).reliable().transient_local());
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  process_timer_ = create_wall_timer(to_period(params_.process_period_s),
                                     std::bind(&MssNode::on_process_timer, this));
  if (params_.map_rate_hz > 0.0) {
    map_timer_ = create_wall_timer(to_period(1.0 / params_.map_rate_hz),
                                   std::bind(&MssNode::on_map_timer, this));
  }

  RCLCPP_INFO(get_logger(),
              "mss_node up; config_path=%s overlay=%s; inputs %s %s %s %s; TF %s -> %s; "
              "map %.3g Hz; ENU origin %s",
              params_.config_path.c_str(),
              params_.config_overlay.empty() ? "(none)" : params_.config_overlay.c_str(),
              params_.topic_lidar.c_str(),
              params_.topic_imu.c_str(),
              params_.topic_gnss.c_str(),
              params_.topic_odom.c_str(),
              params_.frames.world.c_str(),
              params_.frames.body.c_str(),
              params_.map_rate_hz,
              params_.origin_from_first_fix ? "from the first valid fix" : "from gnss_origin.*");
}

MssNode::~MssNode() {
  mss::common::set_logger(nullptr);
}

NodeParams MssNode::read_params() {
  NodeParams p;
  std::string default_config;
  try {
    default_config =
        ament_index_cpp::get_package_share_directory("mss_ros2") + "/config/default.yaml";
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    // Not installed through the ament index: config_path must then be set explicitly.
  }
  p.config_path = declare_parameter<std::string>("config_path", default_config);
  p.config_overlay = declare_parameter<std::string>("config_overlay", "");
  p.topic_lidar = declare_parameter<std::string>("topics.lidar", p.topic_lidar);
  p.topic_imu = declare_parameter<std::string>("topics.imu", p.topic_imu);
  p.topic_gnss = declare_parameter<std::string>("topics.gnss", p.topic_gnss);
  p.topic_odom = declare_parameter<std::string>("topics.odom", p.topic_odom);
  p.frames.world = declare_parameter<std::string>("tf.world_frame", p.frames.world);
  p.frames.body = declare_parameter<std::string>("tf.body_frame", p.frames.body);
  p.qos_depth_lidar = declare_parameter<int>("qos_depth.lidar", p.qos_depth_lidar);
  p.qos_depth_imu = declare_parameter<int>("qos_depth.imu", p.qos_depth_imu);
  p.qos_depth_gnss = declare_parameter<int>("qos_depth.gnss", p.qos_depth_gnss);
  p.qos_depth_odom = declare_parameter<int>("qos_depth.odom", p.qos_depth_odom);
  p.process_period_s = declare_parameter<double>("process_period_s", p.process_period_s);
  p.map_rate_hz = declare_parameter<double>("map_rate_hz", p.map_rate_hz);
  p.origin_from_first_fix =
      declare_parameter<bool>("gnss_origin.from_first_fix", p.origin_from_first_fix);
  p.origin_latitude_deg =
      declare_parameter<double>("gnss_origin.latitude_deg", p.origin_latitude_deg);
  p.origin_longitude_deg =
      declare_parameter<double>("gnss_origin.longitude_deg", p.origin_longitude_deg);
  p.origin_altitude_m = declare_parameter<double>("gnss_origin.altitude_m", p.origin_altitude_m);
  p.time_jump_back_warn_s =
      declare_parameter<double>("time_jump_back_warn_s", p.time_jump_back_warn_s);
  p.error_log_period_s = declare_parameter<double>("error_log_period_s", p.error_log_period_s);

  require(!p.config_path.empty(),
          "config_path: not set and the mss_ros2 share directory was not found");
  require(!p.topic_lidar.empty() && !p.topic_imu.empty() && !p.topic_gnss.empty() &&
              !p.topic_odom.empty(),
          "topics.*: must not be empty");
  require(!p.frames.world.empty(), "tf.world_frame: must be non-empty");
  require(
      p.qos_depth_lidar > 0 && p.qos_depth_imu > 0 && p.qos_depth_gnss > 0 && p.qos_depth_odom > 0,
      "qos_depth.*: must be > 0");
  require(std::isfinite(p.process_period_s) && p.process_period_s > 0.0,
          "process_period_s: must be > 0");
  require(std::isfinite(p.map_rate_hz) && p.map_rate_hz >= 0.0, "map_rate_hz: must be >= 0");
  require(std::isfinite(p.time_jump_back_warn_s) && p.time_jump_back_warn_s > 0.0,
          "time_jump_back_warn_s: must be > 0");
  require(std::isfinite(p.error_log_period_s) && p.error_log_period_s > 0.0,
          "error_log_period_s: must be > 0");
  if (!p.origin_from_first_fix) {
    require(std::isfinite(p.origin_latitude_deg) &&
                std::abs(p.origin_latitude_deg) <= kMaxAbsLatitudeDeg,
            "gnss_origin.latitude_deg: must be in [-90, 90]");
    require(std::isfinite(p.origin_longitude_deg) &&
                std::abs(p.origin_longitude_deg) <= kMaxAbsLongitudeDeg,
            "gnss_origin.longitude_deg: must be in [-180, 180]");
    require(std::isfinite(p.origin_altitude_m), "gnss_origin.altitude_m: must be finite");
  }
  return p;
}

bool MssNode::check_stamp(Topic topic, mss::common::Timestamp t) {
  TopicCounters& c = counters_[topic];
  if (!c.stamp_gate.is_new(t)) {
    ++c.time_skipped;
    const double back_s = mss::common::seconds_between(t, c.stamp_gate.last());
    const char* what = back_s > params_.time_jump_back_warn_s
                           ? "time jumped back (bag loop or clock reset), skipped"
                           : "stamp not increasing, skipped";
    report(topic, what, [back_s] { return std::to_string(back_s) + " s back"; });
    return false;
  }
  return true;
}

bool MssNode::accept_stamp(Topic topic, mss::common::Timestamp t) {
  if (!check_stamp(topic, t)) {
    return false;
  }
  commit_stamp(topic, t);
  return true;
}

bool MssNode::report_due(Topic topic) {
  TopicCounters& c = counters_[topic];
  ++c.pending_report;
  const auto now = std::chrono::steady_clock::now();
  return c.last_report == std::chrono::steady_clock::time_point{} ||
         now - c.last_report >= to_period(params_.error_log_period_s);
}

void MssNode::log_report(Topic topic, const char* what, const std::string& reason) {
  TopicCounters& c = counters_[topic];
  const auto now = std::chrono::steady_clock::now();
  RCLCPP_WARN(get_logger(),
              "%s: %s: %s (%llu such events since the last report; totals: time %llu, "
              "convert %llu, push %llu)",
              kTopicNames[topic],
              what,
              reason.c_str(),
              static_cast<unsigned long long>(c.pending_report),
              static_cast<unsigned long long>(c.time_skipped),
              static_cast<unsigned long long>(c.convert_errors),
              static_cast<unsigned long long>(c.push_errors));
  c.pending_report = 0;
  c.last_report = now;
}

void MssNode::on_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  TopicCounters& c = counters_[kLidar];
  ++c.received;
  const mss::common::Timestamp stamp = to_timestamp(msg->header.stamp);
  if (!check_stamp(kLidar, stamp)) {
    return;
  }
  const mss::common::Status conv = scan_from_cloud(*msg, scan_);
  if (!conv) {
    ++c.convert_errors;
    report(kLidar, "conversion failed, scan dropped", [&conv] { return conv.reason(); });
    return;
  }
  // Only a cloud that converted advances the stamp gate (R4).
  commit_stamp(kLidar, stamp);
  const mss::common::Status st = est_->push_scan(scan_);
  if (!st) {
    ++c.push_errors;
    report(kLidar, "push_scan", [&st] { return st.reason(); });
  }
}

void MssNode::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  TopicCounters& c = counters_[kImu];
  ++c.received;
  const mss::common::ImuSample s = imu_from_msg(*msg);
  if (!accept_stamp(kImu, s.t)) {
    return;
  }
  const mss::common::Status st = est_->push_imu(s);
  if (!st) {
    ++c.push_errors;
    report(kImu, "push_imu", [&st] { return st.reason(); });
  }
}

void MssNode::on_gnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  TopicCounters& c = counters_[kGnss];
  ++c.received;
  if (!accept_stamp(kGnss, to_timestamp(msg->header.stamp))) {
    return;
  }
  if (!has_origin_) {
    if (!origin_from_fix(*msg, origin_)) {
      ++n_gnss_before_origin_;
      return;
    }
    has_origin_ = true;
    RCLCPP_INFO(get_logger(),
                "GNSS ENU origin from the first valid fix: %.9f deg, %.9f deg, %.3f m",
                origin_.lat_deg,
                origin_.lon_deg,
                origin_.alt_m);
  }
  const mss::common::Status st = est_->push_gnss(gnss_from_navsat(*msg, origin_, gnss_sigma_));
  if (!st) {
    ++c.push_errors;
    report(kGnss, "push_gnss", [&st] { return st.reason(); });
  }
}

void MssNode::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  TopicCounters& c = counters_[kOdom];
  ++c.received;
  const mss::common::WheelSample s = wheel_from_odometry(*msg);
  if (!accept_stamp(kOdom, s.t)) {
    return;
  }
  const mss::common::Status st = est_->push_odom(s);
  if (!st) {
    ++c.push_errors;
    report(kOdom, "push_odom", [&st] { return st.reason(); });
  }
}

void MssNode::on_process_timer() {
  est_->run_once();
  const mss::common::Estimate e = est_->latest_state();
  if (e.state.t == mss::common::Timestamp{} || e.state.t == last_published_) {
    return;
  }
  last_published_ = e.state.t;
  ++n_published_;
  pub_odom_->publish(odometry_from_estimate(e, params_.frames));
  tf_->sendTransform(transform_from_pose(e.state.T_w_b, e.state.t, params_.frames));
  path_.header.stamp = to_msg_time(e.state.t);
  path_.poses.push_back(pose_stamped(e.state.T_w_b, e.state.t, params_.frames.world));
  pub_path_->publish(path_);
}

void MssNode::on_map_timer() {
  const auto info = est_->export_map(map_points_);
  if (!info) {
    ++counters_[kMap].push_errors;
    report(kMap, "export_map", [&info] { return info.reason(); });
    return;
  }
  if (info.value().n_keyframes == 0) {
    return;
  }
  cloud_from_points(map_points_, last_published_, params_.frames.world, map_msg_);
  pub_map_->publish(map_msg_);
  ++n_map_published_;
}

void MssNode::finish() {
  est_->flush();
  const mss::pipeline::Estimator::Stats s = est_->stats();
  const auto& c = counters_;
  RCLCPP_INFO(
      get_logger(),
      "mss_node done: received lidar %llu imu %llu gnss %llu odom %llu; skipped by time "
      "lidar %llu imu %llu gnss %llu odom %llu; convert errors lidar %llu; gnss before origin "
      "%llu; published %llu poses, %llu maps; estimator scans %llu keyframes %llu dropped "
      "scans %llu rejected imu %llu match failures %llu",
      static_cast<unsigned long long>(c[kLidar].received),
      static_cast<unsigned long long>(c[kImu].received),
      static_cast<unsigned long long>(c[kGnss].received),
      static_cast<unsigned long long>(c[kOdom].received),
      static_cast<unsigned long long>(c[kLidar].time_skipped),
      static_cast<unsigned long long>(c[kImu].time_skipped),
      static_cast<unsigned long long>(c[kGnss].time_skipped),
      static_cast<unsigned long long>(c[kOdom].time_skipped),
      static_cast<unsigned long long>(c[kLidar].convert_errors),
      static_cast<unsigned long long>(n_gnss_before_origin_),
      static_cast<unsigned long long>(n_published_),
      static_cast<unsigned long long>(n_map_published_),
      static_cast<unsigned long long>(s.n_scans),
      static_cast<unsigned long long>(s.n_keyframes),
      static_cast<unsigned long long>(s.n_dropped_scans),
      static_cast<unsigned long long>(s.n_rejected_imu),
      static_cast<unsigned long long>(s.n_match_failures));
}

}  // namespace mss_ros2
