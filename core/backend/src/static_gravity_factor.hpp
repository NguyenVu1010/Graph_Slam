#pragma once

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

/// Static-window gravity-direction factor on X_0 = T_w_b and B_0 (keyframe 0),
/// docs/derivations/backend_factors.md section 6c (imu_only_observability.md sections 9, 10).
/// Measurement: f_b, the mean accelerometer reading of the static initialisation window
/// (m/s^2, in b), with the covariance C of that mean. At rest the specific force is
/// f = ba - R_w_b^T g_w. The full residual e = f_b - (ba - R_w_b^T g_w) is projected on the
/// plane normal to f_b, which keeps its direction (tilt against horizontal bias) and drops
/// its magnitude (|g| against ba_z; section 10b):
///   r = P e  (2x1, m/s^2),  P = [e1^T; e2^T], (e1, e2, f_b / |f_b|) orthonormal
/// Variables: T_w_b with the Pose3 right perturbation [dphi, drho]; B_0 additive [dba, dbg]:
///   d r / d[dphi, drho] = P [skew(R_w_b^T g_w), 0],   d r / d[dba, dbg] = P [-I, 0]
/// Noise: Gaussian::Covariance(P C P^T). Constant: g_w (m/s^2, in w).
class StaticGravityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::imuBias::ConstantBias>;
  using Base::evaluateError;
  using Projection = Eigen::Matrix<double, 2, 3>;

  /// Preconditions: f_b non-zero; cov_m2ps4 symmetric positive definite ((m/s^2)^2).
  /// `full_residual` (study only, StudySeams): the 3x1 residual e with noise cov, the round-1
  /// form of imu_only_observability.md section 9; never set in production.
  StaticGravityFactor(gtsam::Key x,
                      gtsam::Key b,
                      const common::Vec3& f_b,
                      const common::Mat3& cov_m2ps4,
                      const common::Vec3& g_w,
                      bool full_residual = false);

  /// The rows of P: an orthonormal basis of the plane normal to f_b (unit-length f_b).
  static Projection projection(const common::Vec3& f_b);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& T_w_b,
      const gtsam::imuBias::ConstantBias& bias,
      boost::optional<gtsam::Matrix&> H_x = boost::none,
      boost::optional<gtsam::Matrix&> H_b = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::Vec3 f_b_;
  common::Vec3 g_w_;
  Projection P_;
  bool full_ = false;
};

}  // namespace mss::backend::detail
