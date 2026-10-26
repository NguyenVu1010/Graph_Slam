#pragma once

#include <array>
#include <cstddef>

#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"

namespace mss::imu {

/// Body motion during one LiDAR scan, used by preprocess::Preprocessor to deskew.
///
/// Frames: `b0` is the body frame at the scan's `t_start` (== ScanFrame::t_start, the
/// LiDAR `header.stamp`). `T_b0_b` is the pose of the body at time `t` expressed in
/// `b0` (a relative pose, NOT `T_w_b`): `p_b0 = T_b0_b * p_b`. Deskew outputs points in
/// the LiDAR frame at `t_start`, so a point with `t_offset_ns == 0` is unchanged.
/// Units: `t` sensor-clock nanoseconds; `T_b0_b.p` in metres.
/// Invariants the producer guarantees (the consumer checks only `size > 0`):
///   - `size <= kMaxImuPerScan`; only `samples[0, size)` are valid;
///   - `samples[0].t == t_start` and `samples[0].T_b0_b` is exactly the identity: the
///     producer inserts this first sample with the IMU state interpolated (linear in
///     time between the bracketing IMU samples) to `t_start`;
///   - `samples[k].t` strictly increasing;
///   - `samples[size-1].t >= t_start + max(t_offset_ns)` (the scan's last point time),
///     so every point time lies inside [samples[0].t, samples[size-1].t].
/// Between two samples the pose is interpolated on SE(3) with
/// `T_a * Exp(s * Log(T_a^-1 * T_b))`, `s = (t - t_a) / (t_b - t_a)` (lie.hpp).
/// The producer is the pipeline (front-end thread), which integrates IMU samples from the
/// scan's start with imu::Preintegrator; when more than kMaxImuPerScan samples fall in a
/// scan it keeps a subset evenly spaced by index, including both ends.
/// Ownership/threading: plain value type, no heap memory (fixed array, ~13 KB); the
/// pipeline owns one instance per scan slot and reuses it. Not shared between threads.
/// Tests: P-01, P-02 (through preprocess).
struct ImuTrajectory {
  /// Capacity: 128 samples covers 0.1 s scans at up to 1.28 kHz IMU rate.
  static constexpr std::size_t kMaxImuPerScan = 128;

  struct Sample {
    common::Timestamp t{};  ///< sensor clock, ns
    common::Pose T_b0_b;    ///< body at `t` expressed in body at `t_start`
  };

  std::array<Sample, kMaxImuPerScan> samples{};
  std::size_t size = 0;         ///< number of valid entries in `samples`
  common::Timestamp t_start{};  ///< == ScanFrame::t_start of the scan; defines `b0`
};

}  // namespace mss::imu
