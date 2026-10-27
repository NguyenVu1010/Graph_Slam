#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

/// Static-window specific-force factor on X_0 = T_w_b and B_0 (keyframe 0),
/// docs/derivations/backend_factors.md section 6c (imu_only_observability.md section 9).
/// Measurement: f_b, the mean accelerometer reading of the static initialisation window
/// (m/s^2, in b), with the covariance of that mean. At rest the specific force is
/// f = ba - R_w_b^T g_w, so the residual (3x1, m/s^2) is
///   r = f_b - (ba - R_w_b^T g_w)
/// Variables: T_w_b with the Pose3 right perturbation [dphi, drho]; B_0 additive [dba, dbg]:
///   d r / d[dphi, drho] = [skew(R_w_b^T g_w), 0],   d r / d[dba, dbg] = [-I, 0]
/// It ties the tilt and the horizontal accelerometer bias together (what a static window
/// measures), unlike independent priors on each. Constant: g_w (m/s^2, in w).
class StaticGravityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::imuBias::ConstantBias>;
  using Base::evaluateError;

  /// Precondition: cov_m2ps4 symmetric positive definite ((m/s^2)^2).
  StaticGravityFactor(gtsam::Key x,
                      gtsam::Key b,
                      const common::Vec3& f_b,
                      const common::Mat3& cov_m2ps4,
                      const common::Vec3& g_w);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& T_w_b,
      const gtsam::imuBias::ConstantBias& bias,
      boost::optional<gtsam::Matrix&> H_x = boost::none,
      boost::optional<gtsam::Matrix&> H_b = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Vec3 f_b_;
  common::Vec3 g_w_;
};

}  // namespace mss::backend::detail
