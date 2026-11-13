#include "mss2d/config.hpp"

#include <cmath>

#include "mss2d/messages.hpp"

namespace mss2d {

Status Config::validate() const {
  if (scan_ring_depth < 1 || scan_ring_depth > kScanRingCapacity) {
    return Status::error("scan_ring_depth: must be in [1, kScanRingCapacity]");
  }
  if (imu_ring_depth < 1 || imu_ring_depth > kImuRingCapacity) {
    return Status::error("imu_ring_depth: must be in [1, kImuRingCapacity]");
  }
  if (wheel_ring_depth < 1 || wheel_ring_depth > kWheelRingCapacity) {
    return Status::error("wheel_ring_depth: must be in [1, kWheelRingCapacity]");
  }
  // Negated comparisons so NaN fails too.
  if (!(imu_rate_hz > 0.0F && imu_rate_hz <= 2000.0F)) {
    return Status::error("imu_rate_hz: must be in (0, 2000] Hz");
  }
  if (!(scan_rate_hz > 0.0F && scan_rate_hz <= 50.0F)) {
    return Status::error("scan_rate_hz: must be in (0, 50] Hz");
  }
  if (!(imu_buffer_s > 0.0F)) {
    return Status::error("imu_buffer_s: must be > 0 s");
  }
  if (static_cast<float>(imu_ring_depth) < imu_rate_hz * imu_buffer_s) {
    return Status::error("imu_ring_depth: must hold imu_rate_hz * imu_buffer_s samples");
  }
  if (max_scan_points < 1 || max_scan_points > kMaxScanPoints) {
    return Status::error("max_scan_points: must be in [1, 1024]");
  }
  if (static_cast<float>(kMotionImuHistory) < 2.0F * imu_rate_hz / scan_rate_hz) {
    return Status::error("imu_rate_hz: kMotionImuHistory must hold two scan periods");
  }
  if (const Status s = grid.validate(); !s.is_ok()) {
    return s;
  }
  if (const Status s = matcher.validate(); !s.is_ok()) {
    return s;
  }
  if (const Status s = motion.validate(); !s.is_ok()) {
    return s;
  }
  if (const Status s = submap.validate(); !s.is_ok()) {
    return s;
  }
  // The robot must stay inside the grid with the distance field margin around it.
  const float half_side_m = 0.5F * static_cast<float>(grid.side_cells) * grid.resolution_m;
  if (!(submap.max_distance_m < half_side_m - grid.max_distance_m)) {
    return Status::error(
        "submap.max_distance_m: must be < half the grid side minus grid.max_distance_m");
  }
  return lidar.validate();
}

Status MotionConfig::validate() const {
  if (!(integration_step_s > 0.0F && integration_step_s <= 0.1F)) {
    return Status::error("motion.integration_step_s: must be in (0, 0.1] s");
  }
  if (!(wheel_timeout_s > 0.0F && wheel_timeout_s <= 1.0F)) {
    return Status::error("motion.wheel_timeout_s: must be in (0, 1] s");
  }
  if (!(max_gyro_bias_rad_s > 0.0F && max_gyro_bias_rad_s <= 1.0F)) {
    return Status::error("motion.max_gyro_bias_rad_s: must be in (0, 1] rad/s");
  }
  if (!(std::abs(gyro_bias_rad_s) <= max_gyro_bias_rad_s)) {
    return Status::error("motion.gyro_bias_rad_s: must be within +-max_gyro_bias_rad_s");
  }
  if (!(gyro_bias_gain >= 0.0F && gyro_bias_gain <= 1.0F)) {
    return Status::error("motion.gyro_bias_gain: must be in [0, 1]");
  }
  if (deskew_knots < 1 || deskew_knots > kMaxDeskewKnots) {
    return Status::error("motion.deskew_knots: must be in [1, kMaxDeskewKnots]");
  }
  return Status::ok();
}

Status SubmapConfig::validate() const {
  if (scans_per_submap < 2 || scans_per_submap > 10000) {
    return Status::error("submap.scans_per_submap: must be in [2, 10000]");
  }
  if (!(max_distance_m > 0.0F)) {
    return Status::error("submap.max_distance_m: must be > 0 m");
  }
  return Status::ok();
}

Status ExtrinsicConfig::validate() const {
  if (!(std::abs(x_m) <= 2.0F && std::abs(y_m) <= 2.0F)) {
    return Status::error("lidar.x_m, lidar.y_m: must be within +-2 m");
  }
  if (!(std::abs(yaw_rad) <= 3.1415927F)) {
    return Status::error("lidar.yaw_rad: must be in [-pi, pi] rad");
  }
  return Status::ok();
}

Status GridConfig::validate() const {
  if (!(resolution_m >= 0.01F && resolution_m <= 0.5F)) {
    return Status::error("grid.resolution_m: must be in [0.01, 0.5] m");
  }
  if (side_cells < 16 || side_cells > kMaxGridSide) {
    return Status::error("grid.side_cells: must be in [16, kMaxGridSide]");
  }
  if (!(max_log_odds >= 1.0F && max_log_odds <= 10.0F)) {
    return Status::error("grid.max_log_odds: must be in [1, 10]");
  }
  // Each increment must survive the int8 quantisation (step max_log_odds / 127).
  const float q = max_log_odds / 127.0F;
  if (!(hit_log_odds >= q && hit_log_odds <= max_log_odds)) {
    return Status::error("grid.hit_log_odds: must be in [max_log_odds / 127, max_log_odds]");
  }
  if (!(miss_log_odds <= -q && miss_log_odds >= -max_log_odds)) {
    return Status::error("grid.miss_log_odds: must be in [-max_log_odds, -max_log_odds / 127]");
  }
  if (!(occupied_log_odds > -max_log_odds && occupied_log_odds < max_log_odds)) {
    return Status::error("grid.occupied_log_odds: must be in (-max_log_odds, max_log_odds)");
  }
  if (!(max_distance_m >= 2.0F * resolution_m && max_distance_m <= 250.0F * resolution_m)) {
    return Status::error("grid.max_distance_m: must be in [2, 250] cells");
  }
  return Status::ok();
}

Status MatcherConfig::validate() const {
  if (!(min_range_m >= 0.0F)) {
    return Status::error("matcher.min_range_m: must be >= 0 m");
  }
  if (!(max_range_m > min_range_m)) {
    return Status::error("matcher.max_range_m: must be > min_range_m");
  }
  if (!(search_linear_m >= 0.0F && search_linear_m <= 2.0F)) {
    return Status::error("matcher.search_linear_m: must be in [0, 2] m");
  }
  if (!(search_angle_rad >= 0.0F && search_angle_rad <= 1.5707964F)) {
    return Status::error("matcher.search_angle_rad: must be in [0, pi/2] rad");
  }
  if (!(search_angle_step_rad > 0.0F && search_angle_step_rad <= 0.2F)) {
    return Status::error("matcher.search_angle_step_rad: must be in (0, 0.2] rad");
  }
  if (max_iterations < 1 || max_iterations > 50) {
    return Status::error("matcher.max_iterations: must be in [1, 50]");
  }
  if (!(converged_translation_m > 0.0F)) {
    return Status::error("matcher.converged_translation_m: must be > 0 m");
  }
  if (!(converged_rotation_rad > 0.0F)) {
    return Status::error("matcher.converged_rotation_rad: must be > 0 rad");
  }
  if (!(huber_m > 0.0F)) {
    return Status::error("matcher.huber_m: must be > 0 m");
  }
  if (min_points < 3) {
    return Status::error("matcher.min_points: must be >= 3");
  }
  if (!(degeneracy_eig_threshold >= 0.0F)) {
    return Status::error("matcher.degeneracy_eig_threshold: must be >= 0");
  }
  return Status::ok();
}

}  // namespace mss2d
