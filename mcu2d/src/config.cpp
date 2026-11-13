#include "mss2d/config.hpp"

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
  if (const Status s = grid.validate(); !s.is_ok()) {
    return s;
  }
  return matcher.validate();
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
