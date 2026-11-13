#pragma once

#include <cstddef>

#include "mss2d/status.hpp"

namespace mss2d {

// Compile-time ring capacities (power of two, static storage inside Merger).
constexpr std::size_t kScanRingCapacity = 2;   ///< scan slots (12 KB each)
constexpr std::size_t kImuRingCapacity = 512;  ///< >= 2 s at 200 Hz
constexpr std::size_t kWheelRingCapacity = 64;

/// Largest submap side in cells: the grid (int8 log-odds) and the distance field (uint8)
/// reserve kMaxGridSide^2 bytes each, 40 KB + 40 KB at 200.
constexpr int kMaxGridSide = 200;

/// Occupancy submap and its distance field.
struct GridConfig {
  float resolution_m = 0.05F;  ///< cell size, m, in [0.01, 0.5]
  int side_cells = 200;        ///< cells per side (square submap), in [16, kMaxGridSide]
  // Log-odds model. Stored as int8 in steps of max_log_odds / 127.
  float hit_log_odds = 0.85F;      ///< added on a beam end, > 0 (p_hit about 0.70)
  float miss_log_odds = -0.40F;    ///< added on cells a beam crosses, < 0 (p_miss about 0.40)
  float max_log_odds = 3.5F;       ///< clamp, |log-odds| <= this, in [1, 10]
  float occupied_log_odds = 0.0F;  ///< cell is occupied for the distance field if above, in
                                   ///< (-max_log_odds, max_log_odds)
  float max_distance_m = 0.5F;     ///< distance field clamp, m, in [2 * resolution, 250 cells]

  Status validate() const;
};

/// Scan filter, correlative search, Gauss-Newton refinement and degeneracy flags.
struct MatcherConfig {
  float min_range_m = 0.15F;  ///< beams shorter than this are dropped, m, >= 0
  float max_range_m = 12.0F;  ///< beams longer than this are dropped, m, > min_range_m
  // Correlative search window around the guess. The linear step is one grid cell.
  float search_linear_m = 0.30F;              ///< half width in x and y, m, in [0, 2]
  float search_angle_rad = 0.17453293F;       ///< half width in yaw, rad (10 deg), in [0, pi/2]
  float search_angle_step_rad = 0.01745329F;  ///< yaw step, rad (1 deg), in (0, 0.2]
  // Gauss-Newton.
  int max_iterations = 5;                 ///< in [1, 50]
  float converged_translation_m = 1e-3F;  ///< step below this (and rotation) = converged, > 0
  float converged_rotation_rad = 1e-3F;   ///< rad (0.06 deg), > 0
  int min_points = 50;                    ///< fewer usable points = TooFewPoints, >= 3
  float huber_m = 0.10F;                  ///< Huber threshold on the distance residual, m, > 0
  /// Hessian eigenvalue below this flags the direction degenerate (absolute, like
  /// core/degeneracy). Translation entries are sums of squared unitless gradients, so
  /// about "points constraining that direction"; >= 0, 0 disables. 50 sits between a
  /// corridor's axis (about 20) and a room's weakest direction (about 280) for 720 beams.
  float degeneracy_eig_threshold = 50.0F;

  Status validate() const;
};

struct Config {
  // Ring depths used at runtime, each in [1, compile-time capacity above].
  std::size_t scan_ring_depth = kScanRingCapacity;
  std::size_t imu_ring_depth = kImuRingCapacity;
  std::size_t wheel_ring_depth = kWheelRingCapacity;

  float imu_rate_hz = 200.0F;          ///< nominal IMU rate, Hz, in (0, 2000]
  float scan_rate_hz = 10.0F;          ///< nominal LiDAR turn rate, Hz, in (0, 50]
  float imu_buffer_s = 2.0F;           ///< IMU history the ring must hold, s, > 0
  std::size_t max_scan_points = 1024;  ///< points accepted per scan, in [1, 1024]

  GridConfig grid;
  MatcherConfig matcher;

  /// Ok, or an error whose reason names the bad key.
  Status validate() const;
};

}  // namespace mss2d
