#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"
#include "mss/pipeline/graph_snapshot.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Message <-> core struct conversions of mss_node (ADR-024 sections 1, 2, 5). Same semantics
// as tools/bag_to_mssdata.py, so the node and mss_bench read a bag the same way. No ROS
// state: pure functions, tested in test/test_conversions.cpp.
namespace mss_ros2 {

/// header.stamp -> Timestamp: sec * 1e9 + nanosec in int64 (never through double).
mss::common::Timestamp to_timestamp(const builtin_interfaces::msg::Time& stamp);

/// Timestamp -> builtin_interfaces Time: sec = floor(ns / 1e9), nanosec in [0, 1e9).
builtin_interfaces::msg::Time to_msg_time(mss::common::Timestamp t);

/// PointCloud2 as the noise bridge writes it (sim/mss_sim/bag_io.py CLOUD_FIELDS) -> ScanFrame.
/// t_start = header.stamp; per point x y z (FLOAT32, m, frame l), intensity (FLOAT32; absent:
/// 0), t (UINT32, ns since header.stamp) and ring (UINT16), read at the message's own field
/// offsets and point_step. Non-finite points are kept (preprocess drops them, as in mss_bench).
/// `out` must have been reserved once (capacity = realtime.max_points_per_scan); it is cleared
/// and refilled without allocation.
/// Errors (literal reasons, `out` left empty): big-endian data; x, y or z missing or not
///   FLOAT32; t or ring missing (not a bridged cloud) or of another type; intensity not
///   FLOAT32; data shorter than width * height * point_step or a field past point_step; more
///   points than out.capacity().
/// Hot path: yes (per scan): no allocation, no logging.
mss::common::Status scan_from_cloud(const sensor_msgs::msg::PointCloud2& msg,
                                    mss::common::ScanFrame& out);

/// Imu -> ImuSample in b: acc_b = linear_acceleration (m/s^2), gyro_b = angular_velocity
/// (rad/s), t = header.stamp. Orientation and covariances are ignored.
mss::common::ImuSample imu_from_msg(const sensor_msgs::msg::Imu& msg);

/// Odometry -> WheelSample: v_mps = twist.twist.linear.x, omega_radps = twist.twist.angular.z,
/// t = header.stamp. The pose part is ignored.
mss::common::WheelSample wheel_from_odometry(const nav_msgs::msg::Odometry& msg);

/// True when latitude, longitude and altitude are finite, |lat| <= 90 and |lon| <= 180 deg (the
/// lla_to_enu precondition). The node skips and counts every fix that fails this, not only the
/// origin (backlog R21).
bool fix_in_range(const sensor_msgs::msg::NavSatFix& msg);

/// Geodetic position of a fix. Returns false when the fix cannot serve as the ENU origin:
/// status.status < 0 (NO_FIX), a non-finite latitude, longitude or altitude, or a latitude
/// outside [-90, 90] or a longitude outside [-180, 180] deg (the lla_to_enu precondition; the
/// node skips and counts such a fix instead of taking it as the origin). Status and finiteness: as
/// bag_to_mssdata._gnss_records.
bool origin_from_fix(const sensor_msgs::msg::NavSatFix& msg, mss::gnss::Lla& out);

/// Sigmas used when a fix declares COVARIANCE_TYPE_UNKNOWN (config gnss.sigma_h_m, sigma_v_m).
struct GnssFallbackSigma {
  double sigma_h_m = 0.0;  ///< m, horizontal 1-sigma
  double sigma_v_m = 0.0;  ///< m, vertical 1-sigma
};

/// NavSatFix -> GnssMeasurement in e: p_e_g = gnss::lla_to_enu(fix, origin); cov =
/// position_covariance (ENU, row-major) unless the type is UNKNOWN, then
/// diag(sigma_h^2, sigma_h^2, sigma_v^2); status = status.status. Non-finite fixes pass
/// through (the GNSS gate rejects them), as in bag_to_mssdata.
mss::common::GnssMeasurement gnss_from_navsat(const sensor_msgs::msg::NavSatFix& msg,
                                              const mss::gnss::Lla& origin,
                                              const GnssFallbackSigma& fallback);

/// Pose T_w_b -> geometry_msgs Pose: position p_w_b, orientation q(R_w_b) (Hamilton w x y z,
/// written to the ROS fields x y z w).
geometry_msgs::msg::Pose pose_to_msg(const mss::common::Pose& T_w_b);

/// Pose covariance of T_w_b, core [rho, phi] right perturbation (T <- T * Exp(delta), delta
/// in b) -> ROS [x y z rotx roty rotz] about the axes of w: J cov J^T with
/// J = blkdiag(R_w_b, R_w_b) (first order: p' = p + R rho, R' = Exp(R phi) R). ADR-024 sec. 2.
mss::common::Mat6 cov_to_ros(const mss::common::Pose& T_w_b, const mss::common::Mat6& cov);

/// Frame names of the outputs (ROS parameters tf.world_frame, tf.body_frame).
struct OutputFrames {
  std::string world;  ///< frame of w, e.g. "map"
  std::string body;   ///< frame of b, e.g. "base_link"
};

/// Estimate -> Odometry: header (state.t, world), child body; pose T_w_b with cov_to_ros;
/// twist.linear = R_w_b^T v_w_b (in body), angular 0 and twist covariance 0 (not estimated).
nav_msgs::msg::Odometry odometry_from_estimate(const mss::common::Estimate& e,
                                               const OutputFrames& frames);

/// T_w_b at t -> PoseStamped in world (one entry of /mss/path).
geometry_msgs::msg::PoseStamped pose_stamped(const mss::common::Pose& T_w_b,
                                             mss::common::Timestamp t,
                                             const std::string& world_frame);

/// T_w_b at t -> TF world -> body (parent world, child body; T_w_b maps body to world, which
/// is what a tf2 transform parent <- child means).
geometry_msgs::msg::TransformStamped transform_from_pose(const mss::common::Pose& T_w_b,
                                                         mss::common::Timestamp t,
                                                         const OutputFrames& frames);

/// Points in w (Estimator::export_map) -> PointCloud2 x y z FLOAT32, point_step 12,
/// height 1, width n, host byte order. `out.data` is resized (it allocates only when the
/// map grows past its previous capacity). Off the hot path (map rate, ADR-024 sec. 2).
void cloud_from_points(const std::vector<Eigen::Vector3f>& points_w,
                       mss::common::Timestamp t,
                       const std::string& world_frame,
                       sensor_msgs::msg::PointCloud2& out);

// --- Live graph display (viz.enabled, ADR-031) ----------------------------------------------
// Off the hot path (viz.rate_hz, about 1 Hz); every marker array starts with one DELETEALL
// marker and redraws the whole graph, so RViz shows the optimised poses of the moment.

/// Marker sizes (display only, no estimation effect). Like the marker colours (loop_color, the
/// keyframe arrow), they are display constants, not YAML tunables: ADR-031 section 2 names this
/// exemption from the CONTRIBUTING.md config rule (review N-2).
struct VizStyle {
  /// m, keyframe arrow length. The arrow points along x of b (the IMU), which is the forward
  /// direction only when the IMU x axis is the robot's forward axis (true for the sim robot,
  /// review N-13).
  double arrow_length_m = 0.6;
  double arrow_shaft_m = 0.06;  ///< m, keyframe arrow shaft diameter
  double arrow_head_m = 0.15;   ///< m, keyframe arrow head diameter
  double loop_width_m = 0.06;   ///< m, loop line width
  double text_height_m = 0.6;   ///< m, height of the counter text
  double text_above_m = 2.0;    ///< m, counter text height above the newest keyframe
};

/// Counters shown in the counter text (from Estimator::Stats).
struct GraphCounts {
  std::uint64_t keyframes = 0;           ///< n_keyframes
  std::uint64_t loops_accepted = 0;      ///< n_loops_accepted (entries into the graph)
  std::uint64_t loops_rejected = 0;      ///< n_loops_rejected (ICP / ambiguity)
  std::uint64_t loops_pcm_rejected = 0;  ///< n_loops_pcm_rejected
  std::uint64_t loops_removed = 0;       ///< n_loops_removed (by PCM)
};

/// Colour of a loop line: green kActive, cyan kReleased (no kernel), red kRemoved.
std_msgs::msg::ColorRGBA loop_color(mss::pipeline::LoopState state);

/// One ARROW marker per keyframe at its optimised T_w_b (namespace "keyframes", id = KeyId),
/// after one DELETEALL marker: 1 + keyframes markers, all with frame_id `world_frame` and stamp
/// `t`. `out.markers` keeps its capacity between calls.
void keyframe_markers(const mss::pipeline::GraphSnapshot& snap,
                      mss::common::Timestamp t,
                      const std::string& world_frame,
                      const VizStyle& style,
                      visualization_msgs::msg::MarkerArray& out);

/// One LINE_LIST marker per loop from keyframe i to keyframe j at their optimised positions,
/// coloured by state (loop_color; namespace "loops", id = index in snap.loops), after one
/// DELETEALL marker. A loop whose keyframe has no pose in the snapshot is skipped.
void loop_markers(const mss::pipeline::GraphSnapshot& snap,
                  mss::common::Timestamp t,
                  const std::string& world_frame,
                  const VizStyle& style,
                  visualization_msgs::msg::MarkerArray& out);

/// The keyframes as a path (one PoseStamped per keyframe, stamped with its own time) in
/// `world_frame`, header stamp `t`. Redrawn whole each time, so it shows the loop corrections
/// that /mss/path (front-end poses as published) does not (backlog R9).
void path_from_snapshot(const mss::pipeline::GraphSnapshot& snap,
                        mss::common::Timestamp t,
                        const std::string& world_frame,
                        nav_msgs::msg::Path& out);

/// TEXT_VIEW_FACING marker with the counters and the loops by state, `text_above_m` above the
/// newest keyframe (the origin of `world_frame` before the first). Namespace "counts", id 0.
visualization_msgs::msg::Marker counts_marker(const mss::pipeline::GraphSnapshot& snap,
                                              const GraphCounts& counts,
                                              mss::common::Timestamp t,
                                              const std::string& world_frame,
                                              const VizStyle& style);

}  // namespace mss_ros2
