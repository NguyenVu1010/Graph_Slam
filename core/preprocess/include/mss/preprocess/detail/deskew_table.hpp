#pragma once

#include <array>
#include <cstddef>

#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/trajectory.hpp"

namespace mss::preprocess::detail {

/// Per-scan table for deskewing: T_l0_l(t) = T_b_l^-1 * T_b0_b(t) * T_b_l, interpolated
/// on SE(3) in the LiDAR frame (docs/derivations/preprocess.md sections 2-4).
/// Ownership/threading: plain value, fixed arrays (no heap); one per Preprocessor, used
/// on the front-end thread only.
class DeskewTable {
 public:
  /// Precomputes A_k = T_l_b * T_b0_b(t_k) * T_b_l and xi_k = Log(A_k^-1 * A_k+1)
  /// ([rho, phi]). Precondition: 1 <= traj.size <= ImuTrajectory::kMaxImuPerScan.
  /// Hot path: per scan, no allocation.
  void build(const imu::ImuTrajectory& traj, const common::Pose& T_b_l);

  /// T_l0_l at absolute sensor time `t`, clamped to [samples[0].t, samples[size-1].t].
  /// Returns the identity when the table is empty. Hot path: per point, no allocation.
  common::Pose T_l0_l(common::Timestamp t) const;

 private:
  static constexpr std::size_t kCapacity = imu::ImuTrajectory::kMaxImuPerScan;
  std::array<common::Timestamp, kCapacity> t_{};
  std::array<common::Pose, kCapacity> A_{};   // T_l0_l at t_k
  std::array<common::Vec6, kCapacity> xi_{};  // Log(A_k^-1 A_k+1), [rho, phi]
  std::size_t size_ = 0;
};

}  // namespace mss::preprocess::detail
