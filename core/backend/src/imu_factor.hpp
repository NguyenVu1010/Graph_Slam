#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mss/common/types.hpp"

namespace mss::backend::detail {

/// IMU preintegration factor on (X_i, V_i, X_j, V_j, B_i), docs/derivations/
/// backend_factors.md section 3 (ADR-015; NOT gtsam::CombinedImuFactor).
/// Residual (9x1, order [phi, p, v] = order of delta.cov, rad / m / m/s):
///   r_phi = Log(dR'^T R_i^T R_j),  r_p = dR'^T (A - dp'),  r_v = dR'^T (C - dv'),
///   A = R_i^T (p_j - p_i - v_i T - 0.5 g_w T^2),  C = R_i^T (v_j - v_i - g_w T),
///   dR', dv', dp' = delta corrected to first order for B_i - delta.bias_lin.
/// Charts (GTSAM): Pose3 right perturbation [dphi, drho] (GTSAM_POSE3_EXPMAP), Vector3 and
/// ConstantBias [dba, dbg] additive. Noise: Gaussian::Covariance(delta.cov).
class ImuPreintFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3,
                                                        gtsam::Vector3,
                                                        gtsam::Pose3,
                                                        gtsam::Vector3,
                                                        gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3,
                                        gtsam::Vector3,
                                        gtsam::Pose3,
                                        gtsam::Vector3,
                                        gtsam::imuBias::ConstantBias>;
  using Base::evaluateError;

  /// `g_w`: gravity in w, m/s^2. Precondition: delta.cov symmetric positive definite.
  ImuPreintFactor(gtsam::Key x_i,
                  gtsam::Key v_i,
                  gtsam::Key x_j,
                  gtsam::Key v_j,
                  gtsam::Key b_i,
                  const common::PreintegratedDelta& delta,
                  const common::Vec3& g_w);

  [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3& X_i,
      const gtsam::Vector3& v_i,
      const gtsam::Pose3& X_j,
      const gtsam::Vector3& v_j,
      const gtsam::imuBias::ConstantBias& b_i,
      boost::optional<gtsam::Matrix&> H_x_i = boost::none,
      boost::optional<gtsam::Matrix&> H_v_i = boost::none,
      boost::optional<gtsam::Matrix&> H_x_j = boost::none,
      boost::optional<gtsam::Matrix&> H_v_j = boost::none,
      boost::optional<gtsam::Matrix&> H_b_i = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

 private:
  common::PreintegratedDelta delta_;
  common::Vec3 g_w_;
};

}  // namespace mss::backend::detail
