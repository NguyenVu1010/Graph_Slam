#include "mss_ros2/mss_node.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mss/common/logger.hpp"
#include "mss/pipeline/config.hpp"

#include "mss_ros2/run_saver.hpp"

namespace mss_ros2 {

namespace {

constexpr const char* kTopicNames[] = {"lidar", "imu", "gnss", "odom", "map", "viz"};
constexpr double kMaxAbsLatitudeDeg = 90.0;    // WGS-84 domain
constexpr double kMaxAbsLongitudeDeg = 180.0;  // WGS-84 domain
// viz (ADR-031): the world timer samples the sim clock and polls gz at this wall period (s),
// independent of viz.paused_after_s (review N2-7), which must hold at least kMinClockSamples of
// them and is bounded above so the "paused" answer does not lag by seconds.
constexpr double kWorldTimerS = 0.05;
constexpr double kMinClockSamples = 4.0;
constexpr double kMinPausedAfterS = kMinClockSamples * kWorldTimerS;  // 0.2 s
constexpr double kMaxPausedAfterS = 5.0;
// With a Gazebo world configured and no /clock seen for this long (wall s), warn (again every
// period): the world state cannot be read (review N2-2).
constexpr double kNoClockWarnS = 10.0;

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
  if (params_.viz_enabled) {
    setup_viz();
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
  p.viz_enabled = declare_parameter<bool>("viz.enabled", p.viz_enabled);
  if (p.viz_enabled) {  // off: no viz.* parameter beyond the flag (behaviour as before)
    p.viz_rate_hz = declare_parameter<double>("viz.rate_hz", p.viz_rate_hz);
    p.viz_save_dir = declare_parameter<std::string>("viz.save_dir", p.viz_save_dir);
    p.viz_run_id = declare_parameter<std::string>("viz.run_id", p.viz_run_id);
    p.viz_gz_world = declare_parameter<std::string>("viz.gz_world", p.viz_gz_world);
    p.viz_gz_timeout_ms = declare_parameter<int>("viz.gz_timeout_ms", p.viz_gz_timeout_ms);
    require(std::isfinite(p.viz_rate_hz) && p.viz_rate_hz > 0.0, "viz.rate_hz: must be > 0");
    require(!p.viz_save_dir.empty(), "viz.save_dir: must be non-empty");
    require(
        p.viz_run_id.find('/') == std::string::npos && p.viz_run_id != "." && p.viz_run_id != "..",
        "viz.run_id: must be a single directory name");
    require(p.viz_gz_timeout_ms > 0, "viz.gz_timeout_ms: must be > 0");
    p.viz_paused_after_s = declare_parameter<double>("viz.paused_after_s", p.viz_paused_after_s);
    p.viz_running_min_rtf = declare_parameter<double>("viz.running_min_rtf", p.viz_running_min_rtf);
    // Bounds (review N2-7): at least kMinClockSamples samples of the world timer per
    // paused_after_s, and at most kMaxPausedAfterS of lag; an RTF above 1 would make a world
    // running in real time read as paused.
    require(std::isfinite(p.viz_paused_after_s) && p.viz_paused_after_s >= kMinPausedAfterS &&
                p.viz_paused_after_s <= kMaxPausedAfterS,
            "viz.paused_after_s: must be in [" + std::to_string(kMinPausedAfterS) + ", " +
                std::to_string(kMaxPausedAfterS) + "] s");
    require(std::isfinite(p.viz_running_min_rtf) && p.viz_running_min_rtf > 0.0 &&
                p.viz_running_min_rtf <= 1.0,
            "viz.running_min_rtf: must be in (0, 1]");
    // The world state is read from the sim clock (review N2-1): without use_sim_time the node's
    // clock is system time, every world would read as running, and /mss/save would resume a
    // world paused from the GUI or CLI.
    require(p.viz_gz_world.empty() || get_parameter("use_sim_time").as_bool(),
            "viz.gz_world: set ('" + p.viz_gz_world +
                "') but use_sim_time is false; the Gazebo world state comes from /clock, so "
                "set use_sim_time:=true (or leave viz.gz_world empty)");
  }

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
  const mss::common::Timestamp stamp = to_timestamp(msg->header.stamp);
  if (!check_stamp(kGnss, stamp)) {
    return;
  }
  // A fix that is skipped below must not advance the stamp gate (as R4 for the lidar): a garbage
  // fix with a late stamp would otherwise block the real publisher's fixes.
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
  if (!fix_in_range(*msg)) {  // R21: every fix, not only the origin
    ++c.convert_errors;
    report(kGnss, "fix outside the WGS-84 range or not finite, skipped", [msg] {
      return std::to_string(msg->latitude) + " deg, " + std::to_string(msg->longitude) + " deg";
    });
    return;
  }
  commit_stamp(kGnss, stamp);
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

void MssNode::setup_viz() {
  std::string run_id = params_.viz_run_id;
  if (run_id.empty()) {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char buf[32];
    std::strftime(buf, sizeof(buf), "mss-%Y%m%d-%H%M%S", &local);
    run_id = buf;
  }
  save_dir_ = (std::filesystem::path(params_.viz_save_dir) / run_id).string();
  world_ = std::make_unique<WorldControl>(params_.viz_gz_world, params_.viz_gz_timeout_ms);
  clock_watch_ =
      std::make_unique<SimClockWatch>(params_.viz_paused_after_s, params_.viz_running_min_rtf);

  constexpr std::size_t kLatchedDepth = 1;  // latest graph only, for RViz joining late
  const rclcpp::QoS latched = rclcpp::QoS(kLatchedDepth).reliable().transient_local();
  pub_kf_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/mss/graph/keyframes", latched);
  pub_loop_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/mss/graph/loops", latched);
  pub_graph_path_ = create_publisher<nav_msgs::msg::Path>("/mss/graph/path", latched);
  pub_counts_ = create_publisher<visualization_msgs::msg::Marker>("/mss/graph/counts", latched);
  pub_paused_ = create_publisher<std_msgs::msg::Bool>("/mss/paused", latched);
  using std::placeholders::_1;
  using std::placeholders::_2;
  // Deferred-response services (header + request): answered from on_world_timer() once gz is
  // done, through send_response (review M-2).
  srv_save_ = create_service<std_srvs::srv::Trigger>(
      "/mss/save",
      [this](const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<std_srvs::srv::Trigger::Request> req) { on_save(header, req); });
  srv_pause_ = create_service<std_srvs::srv::SetBool>(
      "/mss/pause",
      [this](const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<std_srvs::srv::SetBool::Request> req) {
        on_pause(header, req);
      });
  // Wall timers: with use_sim_time a ROS timer stops with /clock, i.e. while the world is paused,
  // and RViz must still be redrawn then (ADR-031). The world timer samples the sim clock and
  // polls gz every kWorldTimerS.
  viz_timer_ = create_wall_timer(to_period(1.0 / params_.viz_rate_hz),
                                 std::bind(&MssNode::on_viz_timer, this));
  world_timer_ =
      create_wall_timer(to_period(kWorldTimerS), std::bind(&MssNode::on_world_timer, this));
  RCLCPP_INFO(get_logger(),
              "viz on: /mss/graph/{keyframes,loops,path,counts} at %.3g Hz; /mss/save -> %s; "
              "/mss/pause -> Gazebo world '%s'; /mss/paused after %.3g s without sim-clock change",
              params_.viz_rate_hz,
              save_dir_.c_str(),
              params_.viz_gz_world.empty() ? "(none)" : params_.viz_gz_world.c_str(),
              params_.viz_paused_after_s);
}

void MssNode::set_world_command_starter(CommandStarter starter) {
  if (world_) {
    world_ = std::make_unique<WorldControl>(
        params_.viz_gz_world, params_.viz_gz_timeout_ms, std::move(starter));
  }
}

void MssNode::on_viz_timer() {
  const mss::common::Status st = est_->graph_snapshot(snap_);
  if (!st) {
    ++counters_[kViz].convert_errors;  // the graph could not be turned into a display (N-9)
    report(kViz, "graph_snapshot", [&st] { return st.reason(); });
    return;
  }
  const mss::pipeline::Estimator::Stats s = est_->stats();
  GraphCounts counts;
  counts.keyframes = s.n_keyframes;
  counts.loops_accepted = s.n_loops_accepted;
  counts.loops_rejected = s.n_loops_rejected;
  counts.loops_pcm_rejected = s.n_loops_pcm_rejected;
  counts.loops_removed = s.n_loops_removed;
  const mss::common::Timestamp t = last_published_;
  const std::string& world = params_.frames.world;
  keyframe_markers(snap_, t, world, viz_style_, kf_markers_);
  loop_markers(snap_, t, world, viz_style_, loop_markers_);
  path_from_snapshot(snap_, t, world, graph_path_);
  pub_kf_markers_->publish(kf_markers_);
  pub_loop_markers_->publish(loop_markers_);
  pub_graph_path_->publish(graph_path_);
  pub_counts_->publish(counts_marker(snap_, counts, t, world, viz_style_));
}

namespace {

double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

bool MssNode::world_paused_now() const {
  return clock_watch_->paused(steady_s());
}

void MssNode::publish_paused() {
  const bool paused = world_paused_now();
  if (!paused_published_ || *paused_published_ != paused) {
    std_msgs::msg::Bool m;
    m.data = paused;
    pub_paused_->publish(m);
    paused_published_ = paused;
  }
}

void MssNode::note_world(bool paused) {
  if (paused) {
    clock_watch_->note_paused(steady_s());
  } else {
    clock_watch_->note_running(steady_s());
  }
  publish_paused();  // at once, not after the next clock samples (review M-3)
}

void MssNode::on_world_timer() {
  // 1. The world state from the sim clock (zero until the first /clock with use_sim_time).
  const double now_s = steady_s();
  const std::int64_t sim_ns = get_clock()->now().nanoseconds();
  if (sim_ns != 0) {
    clock_watch_->update(sim_ns, now_s);
  } else if (world_->configured() && now_s - no_clock_warned_s_ >= kNoClockWarnS) {
    if (no_clock_warned_s_ == 0.0) {
      no_clock_warned_s_ = now_s;  // start the count at the first sample
    } else {
      RCLCPP_WARN(get_logger(),
                  "viz: no /clock for %.0f s: the Gazebo world state cannot be read, so /mss/save "
                  "will not pause it and /mss/paused stays true (is the clock bridged?)",
                  kNoClockWarnS);
      no_clock_warned_s_ = now_s;
    }
  }
  publish_paused();
  // 2. The world request in flight, if it is done.
  if (!pending_) {
    return;
  }
  const std::optional<mss::common::Status> st = world_->poll();
  if (!st) {
    return;
  }
  PendingOp op = std::move(*pending_);
  pending_.reset();
  switch (op.op) {
    case WorldOp::kPause:
      if (st->is_ok()) {
        note_world(op.paused);
      }
      answer_pause(op.header, st->is_ok(), st->is_ok() ? op.text : st->reason());
      return;
    case WorldOp::kSavePausing: {
      const bool paused_here = st->is_ok();
      if (paused_here) {
        note_world(true);
      }
      const std::string note = paused_here ? "" : "; world not paused: " + st->reason();
      const auto [ok, msg] = save_now(note);
      if (!paused_here) {
        answer_save(op.header, ok, msg);
        return;
      }
      // Resume only the world this save paused itself (review M-1).
      const mss::common::Status b = world_->begin(false);
      if (!b.is_ok()) {
        answer_save(op.header, ok, msg + "; resume failed, world left paused: " + b.reason());
        return;
      }
      pending_ = PendingOp{WorldOp::kSaveResuming, op.header, msg, ok, false};
      return;
    }
    case WorldOp::kSaveResuming:
      if (st->is_ok()) {
        note_world(false);
      }
      answer_save(
          op.header,
          op.ok,
          st->is_ok() ? op.text : op.text + "; resume failed, world left paused: " + st->reason());
      return;
  }
}

void MssNode::answer_pause(const std::shared_ptr<rmw_request_id_t>& header,
                           bool ok,
                           const std::string& m) {
  std_srvs::srv::SetBool::Response res;
  res.success = ok;
  res.message = m;
  if (ok) {
    RCLCPP_INFO(get_logger(), "/mss/pause: %s", m.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "/mss/pause: %s", m.c_str());
  }
  srv_pause_->send_response(*header, res);
}

void MssNode::answer_save(const std::shared_ptr<rmw_request_id_t>& header,
                          bool ok,
                          const std::string& m) {
  std_srvs::srv::Trigger::Response res;
  res.success = ok;
  res.message = m;
  if (ok) {
    RCLCPP_INFO(get_logger(), "/mss/save: %s", m.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "/mss/save failed: %s", m.c_str());
  }
  srv_save_->send_response(*header, res);
}

void MssNode::on_pause(const std::shared_ptr<rmw_request_id_t>& header,
                       const std_srvs::srv::SetBool::Request::SharedPtr& req) {
  if (pending_) {
    answer_pause(header, false, "busy: a world request is still in flight");
    return;
  }
  const mss::common::Status b = world_->begin(req->data);
  if (!b.is_ok()) {
    answer_pause(header, false, b.reason());
    return;
  }
  pending_ = PendingOp{
      WorldOp::kPause, header, req->data ? "world paused" : "world running", false, req->data};
}

void MssNode::on_save(const std::shared_ptr<rmw_request_id_t>& header,
                      const std_srvs::srv::Trigger::Request::SharedPtr& /*req*/) {
  if (pending_) {
    answer_save(header, false, "busy: a world request is still in flight");
    return;
  }
  // Stop a running world while writing so no sensor data is lost meanwhile (ADR-031). The state
  // comes from the sim clock, not from what this node last asked (review M-1).
  if (!world_->configured()) {
    const auto [ok, msg] = save_now("; no Gazebo world to pause");
    answer_save(header, ok, msg);
    return;
  }
  if (world_paused_now()) {
    // Paused by anyone (this node, the GUI, the gz CLI) or not advancing (stalled, no /clock):
    // do not touch it (review M-1, N2-2).
    const auto [ok, msg] =
        save_now("; sim clock not advancing (world paused or stalled), world left as it is");
    answer_save(header, ok, msg);
    return;
  }
  const mss::common::Status b = world_->begin(true);
  if (!b.is_ok()) {
    const auto [ok, msg] = save_now("; world not paused: " + b.reason());
    answer_save(header, ok, msg);
    return;
  }
  pending_ = PendingOp{WorldOp::kSavePausing, header, "", false, true};
}

std::pair<bool, std::string> MssNode::save_now(const std::string& note) {
  std::vector<Eigen::Vector3f> points;
  mss::common::Status st = est_->graph_snapshot(snap_);
  if (st.is_ok()) {
    const auto m = est_->export_map(points);
    if (!m.is_ok() && !snap_.keyframes.empty()) {
      st = mss::common::Status::error("export_map: " + m.reason());
    }
  }
  if (!st.is_ok()) {
    return {false, "graph_snapshot: " + st.reason() + note};
  }
  // A new directory per save, never an existing one, also after a restart with the same run id
  // (review N-8, N2-3).
  const auto dir = new_save_dir(save_dir_, n_saves_ + 1);
  if (!dir.is_ok()) {
    return {false, dir.reason() + note};
  }
  n_saves_ = dir.value().second;
  const mss::common::Result<SaveSummary> saved = save_run(dir.value().first, snap_, points);
  if (!saved.is_ok()) {
    return {false, saved.reason() + note};
  }
  const SaveSummary& s = saved.value();
  return {true,
          s.dir.string() + " (kf_opt.tum " + std::to_string(s.n_keyframes) +
              " keyframes, loops.csv " + std::to_string(s.n_loops) + " loops, map.pcd " +
              std::to_string(s.n_points) + " points)" + note};
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
