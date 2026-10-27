#pragma once

// Study-only seams of the backend (no GTSAM types, so the no-GTSAM test build can include
// it). Never called by production code.

namespace mss::backend::detail {

/// IMU study (docs/derivations/imu_only_observability.md section 10, test_imu_only_study.cpp):
/// when true, Graph::add_static_gravity_factor adds the round-1 full 3x1 residual instead of
/// the shipped direction-only form. Process-wide; the caller restores false.
void set_static_gravity_full_residual(bool full);

/// PR 4d fix-round-2 study (imu_only_observability.md section 12): when true,
/// Graph::static_gravity_chi2 returns the fix-round-1 posterior statistic |W r|^2 at the
/// estimate instead of the innovation statistic, to reproduce and explain the fix-round-1
/// numbers. Process-wide; the caller restores false.
void set_static_gravity_posterior_statistic(bool on);

}  // namespace mss::backend::detail
