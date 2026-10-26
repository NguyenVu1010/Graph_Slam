#include "imu_factor.hpp"

#include <gtsam/linear/NoiseModel.h>

#include "mss/common/lie.hpp"

namespace mss::backend::detail {

using common::Mat3;
using common::Vec3;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat96 = Eigen::Matrix<double, 9, 6>;
using Mat93 = Eigen::Matrix<double, 9, 3>;

ImuPreintFactor::ImuPreintFactor(gtsam::Key x_i,
                                 gtsam::Key v_i,
                                 gtsam::Key x_j,
                                 gtsam::Key v_j,
                                 gtsam::Key b_i,
                                 const common::PreintegratedDelta& delta,
                                 const common::Vec3& g_w)
    : Base(gtsam::noiseModel::Gaussian::Covariance(delta.cov), x_i, v_i, x_j, v_j, b_i),
      delta_(delta),
      g_w_(g_w) {}

// Residual, variables and charts: see the class comment and backend_factors.md section 3.
// Jacobian rows [phi, p, v]; pose columns [dphi, drho]; bias columns [dba, dbg].
gtsam::Vector ImuPreintFactor::evaluateError(const gtsam::Pose3& X_i,
                                             const gtsam::Vector3& v_i,
                                             const gtsam::Pose3& X_j,
                                             const gtsam::Vector3& v_j,
                                             const gtsam::imuBias::ConstantBias& b_i,
                                             boost::optional<gtsam::Matrix&> H_x_i,
                                             boost::optional<gtsam::Matrix&> H_v_i,
                                             boost::optional<gtsam::Matrix&> H_x_j,
                                             boost::optional<gtsam::Matrix&> H_v_j,
                                             boost::optional<gtsam::Matrix&> H_b_i) const {
  const Mat3 R_i = X_i.rotation().matrix();
  const Mat3 R_j = X_j.rotation().matrix();
  const Vec3& p_i = X_i.translation();
  const Vec3& p_j = X_j.translation();
  const double T = delta_.dt_s;

  // First-order bias correction (imu_preintegration.md section 5).
  const Vec3 dba = b_i.accelerometer() - delta_.bias_lin.ba;
  const Vec3 dbg = b_i.gyroscope() - delta_.bias_lin.bg;
  const Vec3 theta = delta_.J_dR_bg * dbg;
  const Mat3 dR_c = delta_.dR * common::so3_exp(theta);
  const Vec3 dv_c = delta_.dv + delta_.J_dv_ba * dba + delta_.J_dv_bg * dbg;
  const Vec3 dp_c = delta_.dp + delta_.J_dp_ba * dba + delta_.J_dp_bg * dbg;
  const Mat3 dR_cT = dR_c.transpose();
  const Mat3 R_iT = R_i.transpose();

  const Vec3 A = R_iT * (p_j - p_i - v_i * T - 0.5 * g_w_ * T * T);
  const Vec3 C = R_iT * (v_j - v_i - g_w_ * T);
  const Vec3 r_phi = common::so3_log(dR_cT * R_iT * R_j);
  const Vec3 r_p = dR_cT * (A - dp_c);
  const Vec3 r_v = dR_cT * (C - dv_c);

  Vec9 r;
  r << r_phi, r_p, r_v;

  const bool any_h = H_x_i || H_v_i || H_x_j || H_v_j || H_b_i;
  if (!any_h) {
    return r;
  }
  const Mat3 J_ri = common::so3_jr_inv(r_phi);
  const Mat3 dR_cT_R_iT = dR_cT * R_iT;

  if (H_x_i) {
    Mat96 J = Mat96::Zero();
    J.block<3, 3>(0, 0) = -J_ri * R_j.transpose() * R_i;
    J.block<3, 3>(3, 0) = dR_cT * common::skew(A);
    J.block<3, 3>(3, 3) = -dR_cT;
    J.block<3, 3>(6, 0) = dR_cT * common::skew(C);
    *H_x_i = J;
  }
  if (H_v_i) {
    Mat93 J = Mat93::Zero();
    J.block<3, 3>(3, 0) = -dR_cT_R_iT * T;
    J.block<3, 3>(6, 0) = -dR_cT_R_iT;
    *H_v_i = J;
  }
  if (H_x_j) {
    Mat96 J = Mat96::Zero();
    J.block<3, 3>(0, 0) = J_ri;
    J.block<3, 3>(3, 3) = dR_cT_R_iT * R_j;
    *H_x_j = J;
  }
  if (H_v_j) {
    Mat93 J = Mat93::Zero();
    J.block<3, 3>(6, 0) = dR_cT_R_iT;
    *H_v_j = J;
  }
  if (H_b_i) {
    // u = Jr(theta) J_dR_bg dbg is the right increment of dR'.
    const Mat3 J_u_bg = common::so3_jr(theta) * delta_.J_dR_bg;
    Mat96 J = Mat96::Zero();
    J.block<3, 3>(0, 3) = -J_ri * common::so3_exp(r_phi).transpose() * J_u_bg;
    J.block<3, 3>(3, 0) = -dR_cT * delta_.J_dp_ba;
    J.block<3, 3>(3, 3) = common::skew(r_p) * J_u_bg - dR_cT * delta_.J_dp_bg;
    J.block<3, 3>(6, 0) = -dR_cT * delta_.J_dv_ba;
    J.block<3, 3>(6, 3) = common::skew(r_v) * J_u_bg - dR_cT * delta_.J_dv_bg;
    *H_b_i = J;
  }
  return r;
}

gtsam::NonlinearFactor::shared_ptr ImuPreintFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new ImuPreintFactor(*this));
}

}  // namespace mss::backend::detail
