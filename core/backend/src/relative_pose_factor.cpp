#include "relative_pose_factor.hpp"

#include <utility>

#include "mss/common/lie.hpp"
#include "mss/odom/odom_residual.hpp"

#include "conversions.hpp"

namespace mss::backend::detail {

using common::Mat3;
using common::Mat6;
using common::Vec3;
using common::Vec6;

Se3RelativePoseFactor::Se3RelativePoseFactor(gtsam::Key x_i,
                                             gtsam::Key x_j,
                                             const common::Pose& T_bi_bj,
                                             gtsam::SharedNoiseModel noise)
    : Base(std::move(noise), x_i, x_j), T_bi_bj_(T_bi_bj) {}

// Residual e = P r_m, r_m = Log(Z^-1 T_w_bi^-1 T_w_bj) ([rho, phi], exact, odom O-01).
// Variables T_w_bi, T_w_bj, right perturbation; GTSAM tangent xi_g = [dphi, drho] = P xi_m.
// Rows: e = P r_m, so d e = P d r_m. Columns: xi_m = P xi_g (P = P^-1), so
//   H = d e / d xi_g = P J_m P   (backend_factors.md sections 1 and 5).
gtsam::Vector Se3RelativePoseFactor::evaluateError(const gtsam::Pose3& T_w_bi,
                                                   const gtsam::Pose3& T_w_bj,
                                                   boost::optional<gtsam::Matrix&> H_x_i,
                                                   boost::optional<gtsam::Matrix&> H_x_j) const {
  const odom::RelativePoseResidual res =
      odom::relative_pose_residual(from_gtsam(T_w_bi), from_gtsam(T_w_bj), T_bi_bj_);
  // P S P^T with P = P^T: the same block swap as a covariance (conversions.hpp).
  if (H_x_i) {
    const Mat6 H = permute_cov(res.J_r_xi_i);
    *H_x_i = H;
  }
  if (H_x_j) {
    const Mat6 H = permute_cov(res.J_r_xi_j);
    *H_x_j = H;
  }
  Vec6 e;
  e << res.r.tail<3>(), res.r.head<3>();  // [phi, rho]
  return e;
}

gtsam::NonlinearFactor::shared_ptr Se3RelativePoseFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new Se3RelativePoseFactor(*this));
}

DecoupledRelativePoseFactor::DecoupledRelativePoseFactor(gtsam::Key x_i,
                                                         gtsam::Key x_j,
                                                         const common::Pose& T_bi_bj,
                                                         gtsam::SharedNoiseModel noise)
    : Base(std::move(noise), x_i, x_j), T_bi_bj_(T_bi_bj) {}

// Residual r_phi = Log(R_Z^T R_i^T R_j), r_p = R_Z^T (R_i^T (p_j - p_i) - p_Z); variables
// T_w_bi, T_w_bj with the right perturbation [dphi, drho] (R <- R Exp(dphi),
// p <- p + R drho). Derivation (backend_factors.md section 5):
//   X_i rotation: R_i^T <- Exp(-dphi) R_i^T, so t_rel <- t_rel + skew(t_rel) dphi and
//     R_Z^T R_i^T R_j <- R_Z^T R_i^T R_j Exp(-R_j^T R_i dphi): d r_phi = -Jri R_j^T R_i.
//   X_i translation: p_i <- p_i + R_i drho, so t_rel <- t_rel - drho.
//   X_j rotation: R_j <- R_j Exp(dphi): d r_phi = Jri; r_p does not depend on R_j.
//   X_j translation: p_j <- p_j + R_j drho, so t_rel <- t_rel + R_i^T R_j drho.
gtsam::Vector DecoupledRelativePoseFactor::evaluateError(
    const gtsam::Pose3& T_w_bi,
    const gtsam::Pose3& T_w_bj,
    boost::optional<gtsam::Matrix&> H_x_i,
    boost::optional<gtsam::Matrix&> H_x_j) const {
  const Mat3 R_i = T_w_bi.rotation().matrix();
  const Mat3 R_j = T_w_bj.rotation().matrix();
  const Mat3 R_Z_T = T_bi_bj_.R.transpose();
  const Mat3 R_ij = R_i.transpose() * R_j;
  const Vec3 t_rel = R_i.transpose() * (T_w_bj.translation() - T_w_bi.translation());
  const Vec3 r_p = R_Z_T * (t_rel - T_bi_bj_.p);
  const Vec3 r_phi = common::so3_log(R_Z_T * R_ij);
  const Mat3 Jri = common::so3_jr_inv(r_phi);
  if (H_x_i) {
    Mat6 H = Mat6::Zero();  // rows [r_phi, r_p], columns [dphi, drho]
    H.block<3, 3>(0, 0) = -Jri * R_ij.transpose();
    H.block<3, 3>(3, 0) = R_Z_T * common::skew(t_rel);
    H.block<3, 3>(3, 3) = -R_Z_T;
    *H_x_i = H;
  }
  if (H_x_j) {
    Mat6 H = Mat6::Zero();
    H.block<3, 3>(0, 0) = Jri;
    H.block<3, 3>(3, 3) = R_Z_T * R_ij;
    *H_x_j = H;
  }
  Vec6 e;
  e << r_phi, r_p;
  return e;
}

gtsam::NonlinearFactor::shared_ptr DecoupledRelativePoseFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new DecoupledRelativePoseFactor(*this));
}

}  // namespace mss::backend::detail
