#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

// Relative-pose factors with exact Jacobians on (X_i, X_j) = (T_w_bi, T_w_bj) for the LiDAR
// and loop edges, selected by backend.relative_factor (docs/derivations/backend_factors.md
// section 5; ADR-015 PR 4e note). Measurement Z = T_bi_bj. Both return a GTSAM-ordered
// residual [rotation, translation], so the noise is the one of gtsam::BetweenFactor<Pose3>:
// Gaussian::Covariance(P cov P^T), cov 6x6 [rho, phi] of the right perturbation of Z,
// optionally inside the loop's Robust (Cauchy) wrapper. Variables: Pose3 right perturbation
// [dphi, drho] (GTSAM_POSE3_EXPMAP: R <- R Exp(dphi), p <- p + R drho to first order).

/// "se3_exact": the BetweenFactor<Pose3> residual with exact Jacobians.
/// Residual (6x1, [phi, rho], rad / m): e = P r_m,
///   r_m = odom::relative_pose_residual(T_w_bi, T_w_bj, Z).r = Log(Z^-1 T_w_bi^-1 T_w_bj)
/// ([rho, phi]), P = [0 I; I 0]. Jacobians (mss order, right perturbation xi_m = P xi_g):
///   J_r_xi_j = Jr^-1(r_m),  J_r_xi_i = -Jr^-1(r_m) Ad(T_w_bj^-1 T_w_bi),
///   H_x_i = P J_r_xi_i P,  H_x_j = P J_r_xi_j P.
class Se3RelativePoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;
  using Base::evaluateError;

  /// `noise` in GTSAM order [phi, rho] (6-dimensional); `T_bi_bj` finite.
  Se3RelativePoseFactor(gtsam::Key x_i,
                        gtsam::Key x_j,
                        const common::Pose& T_bi_bj,
                        const gtsam::SharedNoiseModel& noise);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& T_w_bi,
      const gtsam::Pose3& T_w_bj,
      boost::optional<gtsam::Matrix&> H_x_i = boost::none,
      boost::optional<gtsam::Matrix&> H_x_j = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Pose T_bi_bj_;
};

/// "decoupled": translation and rotation of E = Z^-1 T_w_bi^-1 T_w_bj as separate residuals
/// (the SO(3) x R3 chart of E instead of the SE(3) Log).
/// Residual (6x1, [phi, rho] order, rad / m), with t_rel = R_i^T (p_j - p_i):
///   r_phi = Log(R_Z^T R_i^T R_j),   r_p = R_Z^T (t_rel - p_Z)
/// Jacobians, columns [dphi, drho] of T_w_bi and of T_w_bj (Jri = Jr^-1(r_phi)):
///   X_i:  d r_phi = [ -Jri R_j^T R_i ,  0       ]
///         d r_p   = [ R_Z^T skew(t_rel), -R_Z^T  ]
///   X_j:  d r_phi = [ Jri ,  0                   ]
///         d r_p   = [ 0   ,  R_Z^T R_i^T R_j     ]
class DecoupledRelativePoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;
  using Base::evaluateError;

  /// `noise` in GTSAM order [phi, rho] (6-dimensional); `T_bi_bj` finite.
  DecoupledRelativePoseFactor(gtsam::Key x_i,
                              gtsam::Key x_j,
                              const common::Pose& T_bi_bj,
                              const gtsam::SharedNoiseModel& noise);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& T_w_bi,
      const gtsam::Pose3& T_w_bj,
      boost::optional<gtsam::Matrix&> H_x_i = boost::none,
      boost::optional<gtsam::Matrix&> H_x_j = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Pose T_bi_bj_;
};

}  // namespace mss::backend::detail
