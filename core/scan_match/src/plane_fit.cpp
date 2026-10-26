#include "plane_fit.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>

namespace mss::scan_match::detail {

namespace {
constexpr int kMinPlanePoints = 3;  // a plane needs 3 points (geometry, not a tunable)
}  // namespace

std::optional<PlaneFit> fit_plane(const map::NeighborBuffer& buf) {
  if (buf.size < kMinPlanePoints) {
    return std::nullopt;
  }
  const std::size_t n = static_cast<std::size_t>(buf.size);
  const double inv_n = 1.0 / static_cast<double>(n);
  common::Vec3 q = common::Vec3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    q += buf.p_w[i];
  }
  q *= inv_n;
  common::Mat3 S = common::Mat3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const common::Vec3 d = buf.p_w[i] - q;
    S.noalias() += d * d.transpose();
  }
  // compute() rather than computeDirect(): more accurate on near-flat neighbourhoods.
  Eigen::SelfAdjointEigenSolver<common::Mat3> es;
  es.compute(S);
  PlaneFit fit;
  fit.q_w = q;
  fit.n_w = es.eigenvectors().col(0).normalized();
  fit.rms_m = std::sqrt(std::max(es.eigenvalues()(0), 0.0) * inv_n);
  return fit;
}

}  // namespace mss::scan_match::detail
