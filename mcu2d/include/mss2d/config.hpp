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

/// Occupancy grids owned by the front end: two active submaps (the older one is matched
/// against, both are inserted into) plus finished ones kept for the backend. 4 x 46 KB.
constexpr int kSubmapPoolSize = 4;

/// Motion history kept by the front end for prediction and deskew (power of two not needed).
constexpr int kMotionImuHistory = 256;   ///< gyro samples, >= 2 scan periods at the IMU rate
constexpr int kMotionWheelHistory = 64;  ///< wheel samples
constexpr int kMaxDeskewKnots = 64;      ///< poses tabulated over one LiDAR turn

/// Pose graph (docs/backend.md): one node per finished submap. Node 0 is fixed (gauge).
constexpr int kMaxGraphNodes = 128;
constexpr int kMaxGraphEdges = 384;  ///< odometry (nodes - 1) plus loop edges
/// Doubles in the envelope (skyline) of the Cholesky factor after reverse Cuthill-McKee
/// ordering; a solve whose envelope does not fit is refused. 96 KB.
constexpr int kMaxFactorEntries = 12288;

/// Finished submaps are kept for loop closing as 1-bit occupancy at kLoopGridDownsample times
/// the submap cell size: (200 / 2)^2 bits = 1250 bytes per node.
constexpr int kLoopGridDownsample = 2;
constexpr int kLoopGridSide = kMaxGridSide / kLoopGridDownsample;
constexpr int kLoopGridBytes = (kLoopGridSide * kLoopGridSide + 7) / 8;
constexpr int kMaxLoopCandidates = 4;  ///< old nodes verified per new node, at most

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

/// Motion prediction between scans and inside one scan (deskew).
struct MotionConfig {
  float integration_step_s = 0.005F;  ///< longest integration sub-step, s, in (0, 0.1]
  /// Use the wheel speed when a sample lies within wheel_timeout_s of the query time;
  /// otherwise (or when false) the body velocity of the last two scan poses is used.
  bool use_wheel = true;
  float wheel_timeout_s = 0.1F;  ///< s, in (0, 1]
  float gyro_bias_rad_s = 0.0F;  ///< initial gyro z bias, rad/s, |.| <= 0.2
  /// Per scan, bias += gain * (gyro yaw change - matched yaw change) / dt. In [0, 1],
  /// 0 keeps the initial bias.
  float gyro_bias_gain = 0.1F;
  float max_gyro_bias_rad_s = 0.2F;  ///< online estimate clamped to +-this, rad/s, in (0, 1]
  bool deskew = true;                ///< false: every point is taken at the scan end
  int deskew_knots = 16;             ///< poses tabulated over a turn, in [1, kMaxDeskewKnots]

  Status validate() const;
};

/// Submap rotation. A new submap starts when the newest one reaches half of either limit,
/// the oldest active one is finished at either limit (so consecutive submaps overlap).
struct SubmapConfig {
  int scans_per_submap = 50;  ///< scans inserted before the oldest is finished, in [2, 10000]
  /// Body distance from the submap origin that finishes it, m, > 0 and below half the grid
  /// side minus max_distance_m so the robot stays well inside the grid.
  float max_distance_m = 2.0F;

  Status validate() const;
};

/// LiDAR extrinsic T_b_l (LiDAR frame in the body = IMU frame).
struct ExtrinsicConfig {
  float x_m = 0.0F;      ///< p_b_l x, m, |.| <= 2
  float y_m = 0.0F;      ///< p_b_l y, m, |.| <= 2
  float yaw_rad = 0.0F;  ///< rad, in [-pi, pi]

  Status validate() const;
};

/// Pose graph: odometry edge noise and the Levenberg-Marquardt solver.
struct GraphConfig {
  // Odometry edge between consecutive submap origins, travelled path d (m) and yaw change
  // |dyaw| (rad): sigma_xy = odom_sigma_xy_m + odom_sigma_xy_per_m d,
  // sigma_yaw = odom_sigma_yaw_rad + odom_sigma_yaw_per_m d + odom_sigma_yaw_per_rad |dyaw|.
  float odom_sigma_xy_m = 0.005F;         ///< m, > 0
  float odom_sigma_xy_per_m = 0.003F;     ///< m per m, >= 0
  float odom_sigma_yaw_rad = 0.001F;      ///< rad, > 0
  float odom_sigma_yaw_per_m = 0.0005F;   ///< rad per m, >= 0
  float odom_sigma_yaw_per_rad = 0.005F;  ///< rad per rad, >= 0
  /// Travel along directions the matcher flagged degenerate follows the prediction (wheel):
  /// the edge gets this fraction of that travel as extra sigma along those directions,
  /// unitless (about the wheel scale error), >= 0.
  float odom_degenerate_fraction = 0.05F;
  int max_iterations = 20;           ///< LM iterations per solve, in [1, 200]
  float initial_lambda = 1e-4F;      ///< LM damping start, relative to diag(H), in (0, 1]
  float converged_step = 1e-5F;      ///< largest |delta| entry (m, rad) that ends a solve, > 0
  float converged_rel_cost = 1e-6F;  ///< relative cost decrease that ends a solve, >= 0

  Status validate() const;
};

/// Loop closing against finished submaps (docs/backend.md).
struct LoopConfig {
  bool enabled = true;
  /// Candidates: nodes at least min_path_m back along the travelled path whose origin lies
  /// within search_radius_m + drift_per_m * path of the query scan (graph estimate).
  float min_path_m = 6.0F;         ///< m, >= 0
  float search_radius_m = 3.0F;    ///< m, > 0
  float drift_per_m = 0.03F;       ///< m per m of path, >= 0 (also widens the window)
  float drift_rad_per_m = 0.005F;  ///< rad per m of path, >= 0
  int max_candidates = 2;  ///< nearest candidates verified per node, [1, kMaxLoopCandidates]
  /// Correlative window half widths: min + drift * path, clamped to max.
  float min_window_m = 0.3F;       ///< m, > 0
  float max_window_m = 1.5F;       ///< m, in [min_window_m, 2]
  float min_window_rad = 0.05F;    ///< rad, > 0
  float max_window_rad = 0.35F;    ///< rad, in [min_window_rad, pi/2]
  float angle_step_rad = 0.0175F;  ///< rad (1 deg), in (0, 0.1]
  int max_query_points = 360;      ///< scan points used (even stride), in [16, kMaxScanPoints]
  int refine_iterations = 10;      ///< Gauss-Newton on the smoothed map, in [1, 50]
  /// Accept: mean smoothed occupancy at the scan points >= min_score, in (0, 1), and the
  /// smallest / largest eigenvalue of the translation Hessian >= min_eig_ratio, in [0, 1).
  float min_score = 0.55F;
  float min_eig_ratio = 0.15F;
  float sigma_xy_m = 0.05F;     ///< loop edge sigma, m, > 0
  float sigma_yaw_rad = 0.01F;  ///< rad, > 0
  /// Cauchy kernel on loop edges, rho(s) = c^2 ln(1 + s / c^2), s = e^T Info e; c > 0.
  float cauchy_c = 3.0F;
  /// After a solve, a new loop edge with e^T Info e above this is dropped and the graph
  /// solved again, > 0.
  float max_chi2 = 25.0F;

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
  MotionConfig motion;
  SubmapConfig submap;
  ExtrinsicConfig lidar;
  GraphConfig graph;
  LoopConfig loop;

  /// Ok, or an error whose reason names the bad key.
  Status validate() const;
};

}  // namespace mss2d
