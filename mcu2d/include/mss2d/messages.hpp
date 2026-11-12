#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mss2d/timestamp.hpp"

namespace mss2d {

// Frames and units follow docs/conventions.md: SI units, radians, frame `b` = body = IMU.
// The 2D problem is SE(2): gyro about body z, translation in the body x/y plane.

/// Most points one scan can hold (RPLIDAR A1/A2 class: about 360 to 1000 per turn).
constexpr std::size_t kMaxScanPoints = 1024;

struct ImuSample2d {
  Timestamp t{};
  float gyro_z_rad_s = 0.0F;  ///< angular rate about body z, rad/s
  bool has_accel = false;     ///< accel_x/y valid
  float accel_x_m_s2 = 0.0F;  ///< specific force, body x, m/s^2
  float accel_y_m_s2 = 0.0F;  ///< specific force, body y, m/s^2
};

struct WheelSample {
  Timestamp t{};
  float v_m_s = 0.0F;         ///< forward speed along body x, m/s
  bool has_yaw_rate = false;  ///< yaw_rate_rad_s valid
  float yaw_rate_rad_s = 0.0F;
};

struct ScanPoint2d {
  float range_m = 0.0F;
  float angle_rad = 0.0F;        ///< in the LiDAR frame, counter-clockwise from +x
  std::int32_t t_offset_ns = 0;  ///< sample time minus Scan2d::t_start, >= 0
};

/// One LiDAR turn. Fixed storage, no allocation; `count` points are valid.
struct Scan2d {
  Timestamp t_start{};
  std::uint32_t count = 0;
  std::array<ScanPoint2d, kMaxScanPoints> points{};

  /// Scan key = scan end time: t_start + max t_offset_ns over the valid points.
  Timestamp key() const {
    std::int32_t max_off = 0;
    for (std::uint32_t i = 0; i < count && i < kMaxScanPoints; ++i) {
      if (points[i].t_offset_ns > max_off) {
        max_off = points[i].t_offset_ns;
      }
    }
    return Timestamp{t_start.ns + max_off};
  }
};

}  // namespace mss2d
