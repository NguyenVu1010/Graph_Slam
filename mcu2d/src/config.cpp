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
  return Status::ok();
}

}  // namespace mss2d
