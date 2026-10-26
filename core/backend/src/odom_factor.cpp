#include "odom_factor.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>
#include <gtsam/linear/NoiseModel.h>

#include "mss/odom/odom_residual.hpp"

#include "conversions.hpp"

namespace mss::backend::detail {

using common::Mat6;
using common::Vec6;

std::optional<OdomBetweenFactor> OdomBetweenFactor::create(gtsam::Key x_i,
                                                           gtsam::Key x_j,
                                                           const common::OdomDelta& odom) {
  if (!odom.cov.allFinite() || !odom.T_b0_b1.R.allFinite() || !odom.T_b0_b1.p.allFinite()) {
    return std::nullopt;
  }
  const Mat6 cov = 0.5 * (odom.cov + odom.cov.transpose());
  const Eigen::SelfAdjointEigenSolver<Mat6> es(cov);
  if (es.info() != Eigen::Success) {
    return std::nullopt;
  }
  const Vec6& lambda = es.eigenvalues();  // ascending
  const Mat6& U = es.eigenvectors();
  const double lambda_max = lambda(5);
  if (!(lambda_max > 0.0) || lambda(0) < -kOdomRankRelTol * lambda_max) {
    return std::nullopt;  // zero or not positive semi-definite
  }
  // Kept eigenvalues are the largest ones (rows of W in descending order).
  Whitening W = Whitening::Zero();
  int rank = 0;
  for (int k = 5; k >= 0; --k) {
    if (lambda(k) > kOdomRankRelTol * lambda_max) {
      W.row(rank) = U.col(k).transpose() / std::sqrt(lambda(k));
      ++rank;
    }
  }
  return OdomBetweenFactor(x_i, x_j, odom.T_b0_b1, W, rank);
}

OdomBetweenFactor::OdomBetweenFactor(
    gtsam::Key x_i, gtsam::Key x_j, const common::Pose& T_bi_bj, const Whitening& W, int rank)
    : Base(gtsam::noiseModel::Unit::Create(static_cast<std::size_t>(rank)), x_i, x_j),
      T_bi_bj_(T_bi_bj),
      W_(W),
      rank_(rank) {}

// Residual W r_m with r_m = Log(Z^-1 T_w_bi^-1 T_w_bj) ([rho, phi]); variables T_w_bi,
// T_w_bj with the GTSAM right perturbation [dphi, drho] = P^T xi_m, so H = W J_m P.
gtsam::Vector OdomBetweenFactor::evaluateError(const gtsam::Pose3& T_w_bi,
                                               const gtsam::Pose3& T_w_bj,
                                               boost::optional<gtsam::Matrix&> H_x_i,
                                               boost::optional<gtsam::Matrix&> H_x_j) const {
  const odom::RelativePoseResidual res =
      odom::relative_pose_residual(from_gtsam(T_w_bi), from_gtsam(T_w_bj), T_bi_bj_);
  const Whitening P = permutation_matrix();
  if (H_x_i) {
    *H_x_i = W_.topRows(rank_) * res.J_r_xi_i * P;
  }
  if (H_x_j) {
    *H_x_j = W_.topRows(rank_) * res.J_r_xi_j * P;
  }
  return W_.topRows(rank_) * res.r;
}

gtsam::NonlinearFactor::shared_ptr OdomBetweenFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new OdomBetweenFactor(*this));
}

}  // namespace mss::backend::detail
