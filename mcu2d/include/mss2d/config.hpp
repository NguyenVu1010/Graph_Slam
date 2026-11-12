#pragma once

#include <cstddef>

#include "mss2d/status.hpp"

namespace mss2d {

// Compile-time ring capacities (power of two, static storage inside Merger).
constexpr std::size_t kScanRingCapacity = 2;   ///< scan slots (12 KB each)
constexpr std::size_t kImuRingCapacity = 512;  ///< >= 2 s at 200 Hz
constexpr std::size_t kWheelRingCapacity = 64;

struct Config {
  // Ring depths used at runtime, each in [1, compile-time capacity above].
  std::size_t scan_ring_depth = kScanRingCapacity;
  std::size_t imu_ring_depth = kImuRingCapacity;
  std::size_t wheel_ring_depth = kWheelRingCapacity;

  float imu_rate_hz = 200.0F;          ///< nominal IMU rate, Hz, in (0, 2000]
  float scan_rate_hz = 10.0F;          ///< nominal LiDAR turn rate, Hz, in (0, 50]
  float imu_buffer_s = 2.0F;           ///< IMU history the ring must hold, s, > 0
  std::size_t max_scan_points = 1024;  ///< points accepted per scan, in [1, 1024]

  /// Ok, or an error whose reason names the bad key.
  Status validate() const;
};

}  // namespace mss2d
