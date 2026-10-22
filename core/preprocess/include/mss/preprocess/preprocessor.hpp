#pragma once

#include <cstddef>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/preprocess/config.hpp"

namespace mss::preprocess {

/// Per-scan cleaning of a raw LiDAR scan: range filter, deskew, voxel downsample.
///
/// Ownership/threading: single-threaded; one instance owned by pipeline::Estimator and
/// used only on the front-end thread. Holds internal scratch buffers sized once in the
/// constructor; it keeps no reference to its inputs after process() returns.
class Preprocessor {
 public:
  /// Validates `cfg` and reserves every scratch buffer for `max_points` input points.
  /// Frames: `T_b_l` = pose of the LiDAR frame l in the body frame b (frames.T_b_l),
  ///   stored and used by the deskew step of process().
  /// Units: `max_points` in points (normally realtime.max_points_per_scan); T_b_l.p in m.
  /// Preconditions: `max_points > 0`.
  /// Errors: throws std::invalid_argument (initialisation may throw) when
  ///   `cfg.min_range_m < 0`, `cfg.max_range_m <= cfg.min_range_m`,
  ///   `cfg.downsample_voxel_m < 0` or `max_points == 0`.
  ///   `cfg.downsample_voxel_m == 0` is valid and disables downsampling.
  /// Hot path: no (allocates).
  /// Tests: P-01..P-04 construct it.
  Preprocessor(const PreprocessConfig& cfg, const common::Pose& T_b_l, std::size_t max_points);

  /// Filters, deskews and downsamples `in` into `out`. Steps, in this order:
  ///   1. drop points with any non-finite x/y/z, or with range |p_l| (Euclidean norm in
  ///      l, metres) outside the closed interval [min_range_m, max_range_m];
  ///   2. if `deskew_enabled`: move every point to the LiDAR frame at `in.t_start`:
  ///      `p_l(t_start) = T_l_b * T_b0_b(t) * T_b_l * p_l(t)` with
  ///      `t = in.t_start + t_offset_ns`, `T_b0_b(t)` interpolated from `traj` (see
  ///      imu::ImuTrajectory); point times outside [samples[0].t, samples[size-1].t] use
  ///      the nearest end sample (clamp, no extrapolation);
  ///   3. if `downsample_voxel_m > 0`: voxel grid in l anchored at the origin
  ///      (index = floor(p_l / downsample_voxel_m) per axis); keep the FIRST surviving
  ///      point (input order) of each voxel, unchanged, with all its attributes.
  /// Kept points stay in input order. `out.t_start = in.t_start`; every kept point keeps
  /// its intensity, ring and original `t_offset_ns` (after deskew it no longer describes
  /// the point's frame time, it is kept for diagnostics only).
  /// Deskew composes the constructor's extrinsic `T_b_l` with ImuTrajectory::T_b0_b:
  ///   `T_l0_l(t) = T_b_l^-1 * T_b0_b(t) * T_b_l` (l0 = LiDAR frame at in.t_start).
  /// Frames: `in` in l with per-point times; `traj` in b0 (body at in.t_start);
  ///   `out` in l at `in.t_start`.
  /// Units: metres, nanoseconds.
  /// Preconditions (NOT checked): `&in != &out`; `in.size() <= max_points` given to the
  ///   constructor; when deskewing, `traj.t_start == in.t_start` and traj satisfies the
  ///   invariants in imu::ImuTrajectory.
  /// Errors: returns Status::error, and leaves `out` cleared (size 0), ONLY when
  ///   - `out.capacity() < in.size()` ("output capacity too small"), or
  ///   - `deskew_enabled` and `traj.size == 0` ("empty imu trajectory").
  ///   Filtering every point away is NOT an error: returns ok with `out.size() == 0`;
  ///   the scan matcher then fails with "too few inliers".
  /// Hot path: yes, per scan. No heap allocation, no logging, no exceptions; `in` is
  ///   read through spans, never copied.
  /// Tests: P-01 (deskew, constant motion), P-02 (static: out == in after filtering),
  ///   P-03 (voxel downsample), P-04 (NaN and range filter).
  common::Status process(const common::ScanFrame& in,
                         const imu::ImuTrajectory& traj,
                         common::ScanFrame& out);
};

}  // namespace mss::preprocess
