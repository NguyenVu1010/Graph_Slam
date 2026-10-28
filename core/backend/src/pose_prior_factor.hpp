#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

/// Pose prior with an exact Jacobian (docs/derivations/backend_factors.md section 2b). It
/// replaces gtsam::PriorFactor<Pose3>, which has the same residual but returns an identity
/// Jacobian, exact only at zero residual (docs/lessons.md section 1.9).
/// Variable: X (a Pose3: T_w_b of a keyframe, or the GNSS frame variable T_e_w), right
/// perturbation [dphi, drho] (GTSAM_POSE3_EXPMAP: R <- R Exp(dphi), p <- p + R drho to
/// first order). Prior: X0, a fixed pose of the same frames as X.
/// Residual (6x1, GTSAM order [phi, rho], rad / m): e = P r_m,
///   r_m = odom::relative_pose_residual(X0, X, Identity).r = Log(X0^-1 X)  ([rho, phi]),
/// P = [0 I; I 0]. This is PriorFactor<Pose3>'s -Local(X, X0) = Logmap(X0^-1 X), so the
/// cost and the noise model ([phi, rho] sigmas) are unchanged; only the Jacobian differs:
///   J_r_xi = Jr^-1(r_m)  (mss order),  H = P Jr^-1(r_m) P.
class PosePriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  using Base::evaluateError;

  /// `noise` in GTSAM order [phi, rho] (6-dimensional); `prior` finite.
  PosePriorFactor(gtsam::Key x, const common::Pose& prior, const gtsam::SharedNoiseModel& noise);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& X, boost::optional<gtsam::Matrix&> H_x = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Pose prior_;
};

}  // namespace mss::backend::detail
