#include "mss/preprocess/detail/deskew_table.hpp"

#include <algorithm>
#include <cstdint>

#include "mss/common/lie.hpp"

namespace mss::preprocess::detail {

namespace {
// Bitwise identity test: no tolerance, so only an exact identity sample qualifies.
bool is_exact_identity(const common::Pose& pose) {
  return (pose.R.array() == common::Mat3::Identity().array()).all() &&
         (pose.p.array() == 0.0).all();
}
}  // namespace

void DeskewTable::build(const imu::ImuTrajectory& traj, const common::Pose& T_b_l) {
  size_ = std::min(traj.size, kCapacity);
  const common::Pose T_l_b = T_b_l.inverse();
  for (std::size_t k = 0; k < size_; ++k) {
    const imu::ImuTrajectory::Sample& sample = traj.samples[k];
    t_[k] = sample.t;
    // Exactness rule (derivation section 4): conjugating the identity would leave residue.
    A_[k] = is_exact_identity(sample.T_b0_b) ? common::Pose{} : T_l_b * sample.T_b0_b * T_b_l;
  }
  for (std::size_t k = 0; k + 1 < size_; ++k) {
    xi_[k] = common::se3_log(A_[k].inverse() * A_[k + 1]);
  }
  if (size_ > 0) {
    xi_[size_ - 1] = common::Vec6::Zero();
  }
}

common::Pose DeskewTable::T_l0_l(common::Timestamp t) const {
  if (size_ == 0) {
    return common::Pose{};
  }
  if (t <= t_[0]) {
    return A_[0];
  }
  if (t >= t_[size_ - 1]) {
    return A_[size_ - 1];
  }
  // t_[0] < t < t_[size_-1]: upper_bound finds k + 1 with t_k <= t < t_k+1.
  const auto* const first = t_.data();
  const auto* const upper = std::upper_bound(first, first + size_, t);
  const std::size_t k = static_cast<std::size_t>(upper - first) - 1;
  if (t == t_[k] || (xi_[k].array() == 0.0).all()) {
    return A_[k];
  }
  const std::int64_t dt_ns = t_[k + 1].ns - t_[k].ns;
  const double s = static_cast<double>(t.ns - t_[k].ns) / static_cast<double>(dt_ns);
  const common::Vec6 xi_s = s * xi_[k];
  return A_[k] * common::se3_exp(xi_s);
}

}  // namespace mss::preprocess::detail
