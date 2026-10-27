#include "plane_fit.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>

namespace mss::scan_match::detail {

std::optional<PlaneFit> fit_plane(const map::NeighborBuffer& buf) {
  if (buf.size < kMinPlanePoints) {
    return std::nullopt;
  }
  const auto n = static_cast<std::size_t>(buf.size);
  const double inv_n = 1.0 / static_cast<double>(n);
  common::Vec3 q = common::Vec3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    q += buf.p_w[i];
  }
  q *= inv_n;
  common::Mat3 scatter = common::Mat3::Zero();  // S in the derivation
  for (std::size_t i = 0; i < n; ++i) {
    const common::Vec3 d = buf.p_w[i] - q;
    scatter.noalias() += d * d.transpose();
  }
  // compute() rather than computeDirect(): more accurate on near-flat neighbourhoods.
  Eigen::SelfAdjointEigenSolver<common::Mat3> es;
  es.compute(scatter);
  PlaneFit fit;
  fit.q_w = q;
  fit.n_w = es.eigenvectors().col(0).normalized();
  fit.rms_m = std::sqrt(std::max(es.eigenvalues()(0), 0.0) * inv_n);
  fit.lambda_min_m2 = es.eigenvalues()(0);
  fit.lambda_mid_m2 = es.eigenvalues()(1);
  fit.lambda_max_m2 = es.eigenvalues()(2);
  return fit;
}

bool passes_planarity(const PlaneFit& fit, double min_eig_ratio) {
  return !(min_eig_ratio > 0.0) || fit.lambda_min_m2 <= min_eig_ratio * fit.lambda_mid_m2;
}

bool passes_collinearity(const PlaneFit& fit, double min_mid_ratio) {
  return !(min_mid_ratio > 0.0) || fit.lambda_mid_m2 > min_mid_ratio * fit.lambda_max_m2;
}

}  // namespace mss::scan_match::detail
