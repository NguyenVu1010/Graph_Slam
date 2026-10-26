#include "mss/odom/odom_residual.hpp"

#include <cmath>

#include <Eigen/Core>

#include "mss/common/lie.hpp"

// Derivation: docs/derivations/wheel_odom.md sections 4-5.
namespace mss::odom {
namespace {

using common::Mat3;
using common::Mat6;
using common::Pose;
using common::Vec3;
using common::Vec6;

// rad. Below this rotation angle the coefficients of Q(rho, phi) use their Taylor series
// (next terms O(theta^4), < 1e-12 relative), avoiding the cancellation in the closed form.
constexpr double kSmallAngleQRad = 1e-3;

// Barfoot, State Estimation for Robotics, eq. 7.86: the top-right block of the SE(3) left
// Jacobian Jl([rho, phi]) = [Jl(phi), Q; 0, Jl(phi)].
Mat3 se3_q(const Vec3& rho, const Vec3& phi) {
  const double theta = phi.norm();
  const double t2 = theta * theta;
  double c1 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;
  if (theta < kSmallAngleQRad) {
    c1 = 1.0 / 6.0 - t2 / 120.0;
    c2 = 1.0 / 24.0 - t2 / 720.0;
    c3 = 1.0 / 120.0 - t2 / 2520.0;
  } else {
    const double st = std::sin(theta);
    const double ct = std::cos(theta);
    const double t3 = t2 * theta;
    const double t4 = t2 * t2;
    const double t5 = t4 * theta;
    c1 = (theta - st) / t3;
    c2 = (t2 + 2.0 * ct - 2.0) / (2.0 * t4);
    c3 = (2.0 * theta - 3.0 * st + theta * ct) / (2.0 * t5);
  }
  const Mat3 phi_x = common::skew(phi);
  const Mat3 rho_x = common::skew(rho);
  const Mat3 px = phi_x * rho_x;
  const Mat3 xp = rho_x * phi_x;
  const Mat3 pxp = px * phi_x;
  const Mat3 q = 0.5 * rho_x + c1 * (px + xp + pxp) + c2 * (phi_x * px + xp * phi_x - 3.0 * pxp) +
                 c3 * (pxp * phi_x + phi_x * pxp);
  return q;
}

// SE(3) right-Jacobian inverse, xi = [rho, phi]: Jr^-1(xi) = Jl^-1(-xi), with
// Jl^-1(xi) = [Jl^-1(phi), -Jl^-1(phi) Q Jl^-1(phi); 0, Jl^-1(phi)].
// TODO(common): move to mss::common::lie (se3_jr_inv) once common offers it (plan
// contract issue 3); odom then calls the common version.
Mat6 se3_jr_inv(const Vec6& xi) {
  const Vec3 rho = -xi.head<3>();
  const Vec3 phi = -xi.tail<3>();
  const Mat3 jl_inv = common::so3_jl_inv(phi);
  const Mat3 q = se3_q(rho, phi);
  Mat6 out = Mat6::Zero();
  out.topLeftCorner<3, 3>() = jl_inv;
  out.topRightCorner<3, 3>() = -jl_inv * q * jl_inv;
  out.bottomRightCorner<3, 3>() = jl_inv;
  return out;
}

}  // namespace

// Residual r = Log(Z^-1 T_w_bi^-1 T_w_bj); variables T_w_bi, T_w_bj (right perturbation,
// [rho, phi]). J_r_xi_j = Jr^-1(r), J_r_xi_i = -Jr^-1(r) Ad(T_w_bj^-1 T_w_bi).
RelativePoseResidual relative_pose_residual(const Pose& T_w_bi,
                                            const Pose& T_w_bj,
                                            const Pose& T_bi_bj_meas) {
  RelativePoseResidual res;
  const Pose T_bi_bj = T_w_bi.inverse() * T_w_bj;
  res.r = common::se3_log(T_bi_bj_meas.inverse() * T_bi_bj);
  const Mat6 jr_inv = se3_jr_inv(res.r);
  res.J_r_xi_j = jr_inv;
  res.J_r_xi_i = -jr_inv * common::se3_adj(T_bi_bj.inverse());
  return res;
}

// Residual r = S v_o, v_o = R_b_o^T (R_w_b^T v_w_b + omega_b x p_b_o); variables T_w_b
// (right perturbation, [rho, phi]) and v_w_b (additive). With u = R_w_b^T v_w_b,
// Exp(phi)^T u ~ u + skew(u) phi:
//   J_r_xi = S R_b_o^T [0, skew(u)],  J_r_v = S R_b_o^T R_w_b^T.
NonholonomicResidual nonholonomic_residual(const Pose& T_w_b,
                                           const Vec3& v_w_b,
                                           const Vec3& omega_b_radps,
                                           const Pose& T_b_o) {
  NonholonomicResidual res;
  const Mat3 R_o_b = T_b_o.R.transpose();
  const Mat3 R_b_w = T_w_b.R.transpose();
  const Vec3 v_b = R_b_w * v_w_b;  // body velocity in b
  const Vec3 v_o = R_o_b * (v_b + omega_b_radps.cross(T_b_o.p));
  res.r = v_o.tail<2>();  // S picks [y, z] of o
  const Mat3 J_vo_phi = R_o_b * common::skew(v_b);
  const Mat3 J_vo_v = R_o_b * R_b_w;
  res.J_r_xi.rightCols<3>() = J_vo_phi.bottomRows<2>();
  res.J_r_v = J_vo_v.bottomRows<2>();
  return res;
}

}  // namespace mss::odom
