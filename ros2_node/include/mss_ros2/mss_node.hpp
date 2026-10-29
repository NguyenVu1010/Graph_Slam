#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/pipeline/estimator.hpp"

#include "mss_ros2/conversions.hpp"
#include "mss_ros2/rclcpp_logger.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace mss_ros2 {

/// ROS wiring parameters of mss_node (ADR-024 section 7). Estimator tunables are not here:
/// they come from config_path + config_overlay (ADR-008). Defaults are the values below;
/// MssNode validates them at start-up and throws std::invalid_argument naming the parameter.
struct NodeParams {
  std::string config_path;     ///< config/default.yaml (required)
  std::string config_overlay;  ///< optional overlay YAML ("" = none)
  std::string topic_lidar = "/lidar/points";
  std::string topic_imu = "/imu/data";
  std::string topic_gnss = "/gnss/fix";
  std::string topic_odom = "/wheel/odom";
  /// tf.world_frame, tf.body_frame ("" = frames.body of the loaded config).
  OutputFrames frames{"map", ""};
  int qos_depth_lidar = 10;        ///< messages (> 0)
  int qos_depth_imu = 400;         ///< messages (> 0): 2 s at 200 Hz, as realtime.imu_queue_depth
  int qos_depth_gnss = 50;         ///< messages (> 0)
  int qos_depth_odom = 50;         ///< messages (> 0)
  double process_period_s = 0.01;  ///< s, wall period of the run_once() timer (> 0)
  double map_rate_hz = 1.0;        ///< Hz, /mss/map export rate (>= 0; 0 = off)
  bool origin_from_first_fix = true;   ///< ENU origin = first valid fix (else gnss_origin.*)
  double origin_latitude_deg = 0.0;    ///< deg, ENU origin when origin_from_first_fix is false
  double origin_longitude_deg = 0.0;   ///< deg
  double origin_altitude_m = 0.0;      ///< m, WGS-84 ellipsoid height
  double time_jump_back_warn_s = 1.0;  ///< s, a stamp this far back is logged as a jump (> 0)
  double error_log_period_s = 5.0;     ///< s, throttle of the drop/error warnings (> 0)
};

/// mss_node: subscribes to LiDAR, IMU, GNSS and wheel odometry, feeds pipeline::Estimator,
/// publishes /mss/odometry, /mss/path, TF world -> body and /mss/map (ADR-024).
/// Threading: everything (callbacks, both timers) runs on the single-threaded executor of
/// main(), which is the one thread the Estimator contract requires (ADR-017); no lock.
class MssNode : public rclcpp::Node {
 public:
  /// Reads and validates the parameters, loads the config and builds the Estimator.
  /// Errors: throws std::invalid_argument (bad parameter or config; message names the key)
  /// or what the Estimator constructor throws. Initialisation only.
  explicit MssNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MssNode() override;

  /// End of input: Estimator::flush() and one log line with the node and estimator counters.
  /// Call once after the executor stops (publishers are not used).
  void finish();

 private:
  enum Topic : std::size_t { kLidar = 0, kImu, kGnss, kOdom, kMap, kNumTopics };
  struct TopicCounters {
    std::uint64_t received = 0;        ///< messages received
    std::uint64_t time_skipped = 0;    ///< dropped: stamp not increasing / jumped back
    std::uint64_t convert_errors = 0;  ///< dropped: message could not be converted
    std::uint64_t push_errors = 0;     ///< push_* returned an error (see the Estimator counters)
    std::uint64_t pending_report = 0;  ///< problems since the last throttled warning
    std::chrono::steady_clock::time_point last_report{};
    bool has_last_stamp = false;
    mss::common::Timestamp last_stamp{};
  };

  NodeParams read_params();
  bool accept_stamp(Topic topic, mss::common::Timestamp t);
  /// Counts one problem on `topic` and logs one throttled warning (error_log_period_s) with
  /// `reason()`; the reason string is only built when the warning is actually logged, so the
  /// per-message paths stay allocation-free (CLAUDE.md hot path; review I-3).
  template <typename ReasonFn>
  void report(Topic topic, const char* what, ReasonFn&& reason) {
    if (report_due(topic)) {
      log_report(topic, what, reason());
    }
  }
  bool report_due(Topic topic);
  void log_report(Topic topic, const char* what, const std::string& reason);

  void on_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void on_gnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void on_process_timer();
  void on_map_timer();

  RclcppLogger core_logger_;
  NodeParams params_;
  std::unique_ptr<mss::pipeline::Estimator> est_;
  GnssFallbackSigma gnss_sigma_;
  bool has_origin_ = false;
  mss::gnss::Lla origin_;
  std::uint64_t n_gnss_before_origin_ = 0;

  mss::common::ScanFrame scan_;  ///< reused conversion buffer, reserved once
  std::array<TopicCounters, kNumTopics> counters_{};
  mss::common::Timestamp last_published_{};
  std::uint64_t n_published_ = 0;
  std::uint64_t n_map_published_ = 0;
  nav_msgs::msg::Path path_;
  std::vector<Eigen::Vector3f> map_points_;
  sensor_msgs::msg::PointCloud2 map_msg_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace mss_ros2
