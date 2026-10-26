#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

/// GNSS antenna position factor on X_k = T_w_b, docs/derivations/backend_factors.md
/// section 6. Residual (3x1, m), the Graph header sign:
///   r = R_e_w (R_w_b p_b_g + p_w_b) + p_e_w - p_e_g
/// Constants: T_e_w (w -> e, from gnss::GnssAligner), lever arm p_b_g (m, in b).
/// Chart: Pose3 right perturbation [dphi, drho]. Noise: Gaussian::Covariance(cov_m2).
class GnssPositionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  using Base::evaluateError;

  /// Precondition: cov_m2 symmetric positive definite.
  GnssPositionFactor(gtsam::Key x,
                     const common::Vec3& p_e_g,
                     const common::Mat3& cov_m2,
                     const common::Pose& T_e_w,
                     const common::Vec3& p_b_g);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& T_w_b, boost::optional<gtsam::Matrix&> H_x = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Vec3 p_e_g_;
  common::Pose T_e_w_;
  common::Vec3 p_b_g_;
};

}  // namespace mss::backend::detail
