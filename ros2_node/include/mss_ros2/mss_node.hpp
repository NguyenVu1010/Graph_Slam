#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
#include "mss_ros2/stamp_gate.hpp"
#include "mss_ros2/world_control.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  /// Live graph display and the save / pause services (ADR-031). Off: the node publishes and
  /// serves exactly what it did before, and the viz.* parameters below are not even declared.
  bool viz_enabled = false;
  double viz_rate_hz = 1.0;              ///< Hz, wall rate of the graph display (> 0)
  std::string viz_save_dir = "results";  ///< /mss/save writes <save_dir>/<run_id>/ (non-empty)
  std::string viz_run_id;                ///< "" = mss-<local start time YYYYmmdd-HHMMSS>
  std::string viz_gz_world;              ///< Gazebo world to pause ("" = none: pause fails)
  int viz_gz_timeout_ms = 5000;          ///< ms, gz service request timeout (> 0)
  /// s of wall time without a sim-clock change after which the world counts as paused, in
  /// [0.2, 5] (at least 4 world-timer samples, at most 5 s of lag; review N2-7);
  /// /mss/save and /mss/paused use it, joy_commands reads /mss/paused (review M-1, N-7).
  double viz_paused_after_s = 0.5;
  /// Real-time factor below which an advancing sim clock still counts as paused, in (0, 1]:
  /// Gazebo's start-up single steps (live_start, about 0.01 x) are not "running".
  double viz_running_min_rtf = 0.05;
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

  /// Per-input message counters (for tests and diagnostics).
  struct InputStats {
    std::uint64_t received = 0;        ///< messages received
    std::uint64_t time_skipped = 0;    ///< dropped: stamp not increasing / jumped back
    std::uint64_t convert_errors = 0;  ///< dropped: not convertible or out of range
    std::uint64_t push_errors = 0;     ///< push_* returned an error
  };
  InputStats lidar_stats() const { return stats_of(kLidar); }
  InputStats gnss_stats() const { return stats_of(kGnss); }
  bool has_origin() const { return has_origin_; }
  /// GNSS fixes skipped while no ENU origin was taken yet (no fix, or an impossible one).
  std::uint64_t n_gnss_before_origin() const { return n_gnss_before_origin_; }

  /// Subscription callbacks, public so tests can feed messages without DDS. Executor thread only.
  void on_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void on_gnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  /// viz.enabled (ADR-031). Services answer with a deferred response: the gz process runs
  /// asynchronously and a wall timer polls it, so the executor never waits on gz (review M-2).
  /// One world request at a time; a request while one is in flight is answered at once with
  /// success false ("busy").
  /// - /mss/pause (std_srvs/SetBool): data true pauses the Gazebo world, false resumes it.
  /// - /mss/save (std_srvs/Trigger): when the world runs (sim clock advancing, SimClockWatch),
  ///   pauses it, writes kf_opt.tum, loops.csv and map.pcd into
  ///   <viz.save_dir>/<run id>/save-<n>/ (run_saver.hpp), then resumes it; a world that is
  ///   already paused (by anyone) stays paused and is never resumed by a save (review M-1).
  ///   Without a world, or when the pause fails, it saves anyway and says so in the message.
  /// - /mss/paused (std_msgs/Bool, latched): the world state the node infers from the sim clock.
  /// The run directory /mss/save writes under.
  const std::string& save_dir() const { return save_dir_; }
  /// Test seam: replaces the gz command starter of the world control (viz.enabled only).
  void set_world_command_starter(CommandStarter starter);

 private:
  enum Topic : std::size_t { kLidar = 0, kImu, kGnss, kOdom, kMap, kViz, kNumTopics };
  struct TopicCounters {
    std::uint64_t received = 0;        ///< messages received
    std::uint64_t time_skipped = 0;    ///< dropped: stamp not increasing / jumped back
    std::uint64_t convert_errors = 0;  ///< dropped: message could not be converted
    std::uint64_t push_errors = 0;     ///< push_* returned an error (see the Estimator counters)
    std::uint64_t pending_report = 0;  ///< problems since the last throttled warning
    std::chrono::steady_clock::time_point last_report{};
    StampGate stamp_gate;  ///< last accepted stamp (strictly increasing per topic)
  };

  NodeParams read_params();
  InputStats stats_of(std::size_t topic) const {
    const TopicCounters& c = counters_[topic];
    return {c.received, c.time_skipped, c.convert_errors, c.push_errors};
  }
  /// Stamp must be later than the last committed one; counts and reports a skip. Does not
  /// advance the gate: call commit_stamp() once the message is going to be used.
  bool check_stamp(Topic topic, mss::common::Timestamp t);
  void commit_stamp(Topic topic, mss::common::Timestamp t) {
    counters_[topic].stamp_gate.commit(t);
  }
  /// check_stamp() then commit_stamp(), for topics with no conversion step that can fail.
  bool accept_stamp(Topic topic, mss::common::Timestamp t);
  /// Counts one problem on `topic` and logs one throttled warning (error_log_period_s) with
  /// `reason()`; the reason string is only built when the warning is actually logged, so the
  /// per-message paths stay allocation-free (CONTRIBUTING.md hot path; review I-3).
  template <typename ReasonFn>
  void report(Topic topic, const char* what, ReasonFn&& reason) {
    if (report_due(topic)) {
      log_report(topic, what, reason());
    }
  }
  bool report_due(Topic topic);
  void log_report(Topic topic, const char* what, const std::string& reason);

  void on_process_timer();
  void on_map_timer();
  void setup_viz();     ///< publishers, services and the wall timers of viz.enabled
  void on_viz_timer();  ///< graph snapshot -> keyframes, loops, path, counts
  /// Wall timer of viz.enabled: samples the sim clock (SimClockWatch, /mss/paused) and polls the
  /// world request in flight, finishing the deferred service call it belongs to.
  void on_world_timer();
  void on_pause(const std::shared_ptr<rmw_request_id_t>& header,
                const std_srvs::srv::SetBool::Request::SharedPtr& req);
  void on_save(const std::shared_ptr<rmw_request_id_t>& header,
               const std_srvs::srv::Trigger::Request::SharedPtr& req);
  bool world_paused_now() const;
  /// Publishes /mss/paused when the inferred state changed.
  void publish_paused();
  /// The node's own gz request succeeded: update the clock watch and /mss/paused at once (M-3).
  void note_world(bool paused);
  /// Writes one save directory and returns the response text (appending `note`).
  std::pair<bool, std::string> save_now(const std::string& note);
  void answer_save(const std::shared_ptr<rmw_request_id_t>& header, bool ok, const std::string& m);
  void answer_pause(const std::shared_ptr<rmw_request_id_t>& header, bool ok, const std::string& m);

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

  // viz.enabled only (ADR-031); all null / empty when it is off.
  mss::pipeline::GraphSnapshot snap_;  ///< reused graph snapshot
  visualization_msgs::msg::MarkerArray kf_markers_;
  visualization_msgs::msg::MarkerArray loop_markers_;
  nav_msgs::msg::Path graph_path_;
  VizStyle viz_style_;
  std::unique_ptr<WorldControl> world_;
  std::unique_ptr<SimClockWatch> clock_watch_;
  std::optional<bool> paused_published_;  ///< last /mss/paused value sent
  /// The deferred service call the world request in flight belongs to.
  enum class WorldOp { kPause, kSavePausing, kSaveResuming };
  struct PendingOp {
    WorldOp op = WorldOp::kPause;
    std::shared_ptr<rmw_request_id_t> header;
    std::string text;     ///< kPause: the state asked; save: the note / the save message so far
    bool ok = false;      ///< kSaveResuming: whether the save itself succeeded
    bool paused = false;  ///< kPause: the state asked (true: pause)
  };
  std::optional<PendingOp> pending_;
  std::string save_dir_;  ///< <viz.save_dir>/<run id>
  int n_saves_ = 0;       ///< highest save-<n> this node created (the next search starts above)
  double no_clock_warned_s_ = 0.0;  ///< steady s of the last "no /clock" warning (0: none yet)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_kf_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_graph_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_counts_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_paused_;
  rclcpp::TimerBase::SharedPtr world_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pause_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
};

}  // namespace mss_ros2
