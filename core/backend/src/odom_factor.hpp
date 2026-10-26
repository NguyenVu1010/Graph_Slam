#pragma once

#include <optional>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

// Numerical rank tolerance of the odometry covariance: eigenvalues below
// kOdomRankRelTol * lambda_max count as zero (not a tunable; backend_factors.md section 7).
inline constexpr double kOdomRankRelTol = 1e-9;

/// Wheel-odometry relative-pose factor on (X_i, X_j) with a possibly singular covariance,
/// docs/derivations/backend_factors.md section 7.
/// Residual (rank x 1): W r_m, r_m = odom::relative_pose_residual(T_i, T_j, T_b0_b1).r
///   ([rho, phi], m / rad), W = diag(lambda_kept)^-1/2 U_kept^T from the eigen split of
///   odom.cov. Noise: Unit(rank). Charts: Pose3 right perturbation [dphi, drho]; the mss
///   Jacobians are mapped with J_g = J_m P.
class OdomBetweenFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;
  using Base::evaluateError;
  using Whitening = Eigen::Matrix<double, 6, 6>;

  /// Returns nullopt when odom.cov or odom.T_b0_b1 is non-finite, odom.cov is not positive
  /// semi-definite, or its numerical rank is 0 (e.g. t0 == t1).
  static std::optional<OdomBetweenFactor> create(gtsam::Key x_i,
                                                 gtsam::Key x_j,
                                                 const common::OdomDelta& odom);

  /// Number of whitened rows (constrained directions), 1..6.
  int rank() const { return rank_; }

  gtsam::Vector evaluateError(const gtsam::Pose3& T_w_bi,
                              const gtsam::Pose3& T_w_bj,
                              boost::optional<gtsam::Matrix&> H_x_i = boost::none,
                              boost::optional<gtsam::Matrix&> H_x_j = boost::none) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  OdomBetweenFactor(
      gtsam::Key x_i, gtsam::Key x_j, const common::Pose& T_bi_bj, const Whitening& W, int rank);

  common::Pose T_bi_bj_;
  Whitening W_ = Whitening::Zero();  // first rank_ rows used
  int rank_ = 0;
};

}  // namespace mss::backend::detail
