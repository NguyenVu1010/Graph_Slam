#pragma once

#include "mss/common/result.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/imu/trajectory.hpp"

#include "imu_buffer.hpp"

namespace mss::pipeline::detail {

/// Builds the deskew trajectory of one scan (docs/derivations/pipeline.md section 2).
/// `x0` is the predicted state at the scan start (`x0.t` = scan `t_start`); `t_end` the
/// scan end time. `scratch` is reset and used for the integration. On success `traj`
/// satisfies the invariants of imu::ImuTrajectory: samples[0] = {t_start, identity},
/// t strictly increasing, last t >= t_end, at most kMaxImuPerScan entries (evenly spaced by
/// index when subsampled, both ends kept).
/// Frames: x0 in w, g_w gravity in w (m/s^2); traj in b0 (body at t_start).
/// Errors (literals): "imu does not cover scan start", "imu does not cover scan end",
///   "imu sample rejected" (non-finite sample); `traj` is then left with size 0.
/// Hot path: yes (per scan); no allocation, logging or exceptions.
common::Status build_imu_trajectory(const ImuBuffer& imu,
                                    const common::NavState& x0,
                                    common::Timestamp t_end,
                                    const common::Vec3& g_w,
                                    imu::Preintegrator& scratch,
                                    imu::ImuTrajectory& traj);

}  // namespace mss::pipeline::detail
