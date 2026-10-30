#include "mss/gnss/gnss.hpp"  // own header first (CLAUDE.md)
// The file name differs from the header name, so clang-format cannot detect the main
// include; this comment keeps it in its own block.

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "mss/common/lie.hpp"

namespace mss::gnss {

GnssAligner::GnssAligner(const GnssConfig& cfg)
    : min_pairs_(cfg.align_min_pairs),
      min_spread_m_(cfg.align_min_spread_m),
      check_yaw_sigma_(cfg.align_check_yaw_sigma),
      frontend_cov_(cfg.align_yaw_sigma_frontend_cov),
      max_yaw_sigma_rad_(cfg.align_yaw_max_sigma_rad),
      min_var_m2_(cfg.align_yaw_min_var_m2) {}

common::Result<common::Pose> GnssAligner::update(const common::Vec3& p_e_g,
                                                 const common::Pose& T_w_b,
                                                 const common::Vec3& p_b_g) {
  // No declared covariance: every fix counts with the variance floor.
  return update(p_e_g, common::Mat3::Zero(), T_w_b, p_b_g);
}

common::Result<common::Pose> GnssAligner::update(const common::Vec3& p_e_g,
                                                 const common::Mat3& cov_e,
                                                 const common::Pose& T_w_b,
                                                 const common::Vec3& p_b_g) {
  return update(p_e_g, cov_e, T_w_b, p_b_g, common::PoseCovariance::Zero());
}

namespace {

// Larger eigenvalue of the symmetric part of the horizontal 2x2 block of `cov` (m^2).
double horizontal_var_max_m2(const common::Mat3& cov) {
  const double h_mean_m2 = 0.5 * (cov(0, 0) + cov(1, 1));
  const double h_half_diff_m2 = 0.5 * (cov(0, 0) - cov(1, 1));
  const double h_off_m2 = 0.5 * (cov(0, 1) + cov(1, 0));
  return h_mean_m2 + std::hypot(h_half_diff_m2, h_off_m2);
}

}  // namespace

// Closed-form 4-DoF fit over running sums; docs/derivations/gnss.md section 4.
common::Result<common::Pose> GnssAligner::update(const common::Vec3& p_e_g,
                                                 const common::Mat3& cov_e,
                                                 const common::Pose& T_w_b,
                                                 const common::Vec3& p_b_g,
                                                 const common::PoseCovariance& P_w_b) {
  using ResultPose = common::Result<common::Pose>;
  if (!p_e_g.allFinite() || !cov_e.allFinite() || !T_w_b.R.allFinite() || !T_w_b.p.allFinite() ||
      !p_b_g.allFinite()) {
    return ResultPose::error("non-finite input");
  }
  // Larger eigenvalue of the symmetric part of the horizontal block, floored (gnss.md s. 4.1).
  const double var_m2 = std::max(horizontal_var_max_m2(cov_e), min_var_m2_);
  if (frontend_cov_ && P_w_b.allFinite()) {
    // Backlog G15: the front-end antenna position error in w, J P J^T with
    // J = d(T_w_b p_b_g)/d[rho, phi] = [R_w_b, -R_w_b skew(p_b_g)] (right perturbation): its
    // larger horizontal eigenvalue (gnss.md section 4.1).
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = T_w_b.R;
    J.rightCols<3>() = -T_w_b.R * common::skew(p_b_g);
    const common::Mat3 cov_w = J * P_w_b * J.transpose();
    sum_front_var_m2_ += std::max(horizontal_var_max_m2(cov_w), 0.0);
  }

  const common::Vec3 p_w_g = T_w_b * p_b_g;  // lever arm applied (G-04)
  if (n_pairs_ == 0) {
    ref_w_ = p_w_g;
    ref_e_ = p_e_g;
  }
  const common::Vec3 a = p_w_g - ref_w_;
  const common::Vec3 b = p_e_g - ref_e_;
  const Eigen::Vector2d a_h = a.head<2>();
  const Eigen::Vector2d b_h = b.head<2>();
  sum_w_ += a;
  sum_e_ += b;
  sum_we_ += a_h * b_h.transpose();
  sum_w_h_sq_m2_ += a_h.squaredNorm();
  sum_e_h_sq_m2_ += b_h.squaredNorm();
  sum_var_m2_ += var_m2;
  sum_var_w_ += var_m2 * a_h;
  sum_var_w_sq_m4_ += var_m2 * a_h.squaredNorm();
  ++n_pairs_;

  if (n_pairs_ < min_pairs_) {
    return ResultPose::error("not enough pairs");
  }

  const auto n = static_cast<double>(n_pairs_);
  // Yaw needs horizontal motion on BOTH sides: a frozen or repeated GNSS fix while the
  // body moves gives cross_c ~ 0 and an arbitrary atan2 (docs/derivations/gnss.md s. 4).
  const Eigen::Vector2d mean_a_h = sum_w_.head<2>() / n;
  const Eigen::Vector2d mean_b_h = sum_e_.head<2>() / n;
  const double spread_w_sq_m2 = sum_w_h_sq_m2_ / n - mean_a_h.squaredNorm();
  const double spread_e_sq_m2 = sum_e_h_sq_m2_ / n - mean_b_h.squaredNorm();
  const double spread_w_m = std::sqrt(spread_w_sq_m2 > 0.0 ? spread_w_sq_m2 : 0.0);
  const double spread_e_m = std::sqrt(spread_e_sq_m2 > 0.0 ? spread_e_sq_m2 : 0.0);
  if (spread_w_m < min_spread_m_ || spread_e_m < min_spread_m_) {
    return ResultPose::error("insufficient horizontal spread");
  }
  // Yaw sigma with the w side exact: delta_yaw = sum (J a_c,i) . n_i / sum |a_c,i|^2, so
  // var = sum var_i |a_c,i|^2 / (sum |a_c,i|^2)^2, with a_c = a_h - mean_a_h expanded over the
  // running sums (gnss.md section 4.1).
  const double sum_ac_sq_m2 = n * spread_w_sq_m2;  // > 0: spread_w_m >= min_spread_m_ > 0
  const double num_m4 =
      sum_var_w_sq_m4_ - 2.0 * mean_a_h.dot(sum_var_w_) + mean_a_h.squaredNorm() * sum_var_m2_;
  // Backlog G15: the front-end errors e_i are of unknown correlation (dead reckoning along a
  // degenerate axis is one correlated error), so their part of the variance is bounded for any
  // correlation: Var(sum u_i . e_i) <= (sum |u_i| s_i)^2 <= sum |a_c,i|^2 sum s_i^2, which adds
  // sum s_i^2 / sum |a_c,i|^2 (gnss.md section 4.1); 0 without the flag.
  // Without the term the previous expression stays, bit for bit (m-4 of the H4 review).
  const double num_clamped_m4 = num_m4 > 0.0 ? num_m4 : 0.0;
  yaw_sigma_rad_ = sum_front_var_m2_ > 0.0
                       ? std::sqrt(num_clamped_m4 / (sum_ac_sq_m2 * sum_ac_sq_m2) +
                                   sum_front_var_m2_ / sum_ac_sq_m2)
                       : std::sqrt(num_clamped_m4) / sum_ac_sq_m2;
  if (check_yaw_sigma_ && !(yaw_sigma_rad_ <= max_yaw_sigma_rad_)) {
    return ResultPose::error("yaw sigma above threshold");
  }

  const Eigen::Matrix2d cross_c =
      sum_we_ - sum_w_.head<2>() * sum_e_.head<2>().transpose() / n;  // m^2
  const double yaw_rad = std::atan2(cross_c(0, 1) - cross_c(1, 0), cross_c(0, 0) + cross_c(1, 1));
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);

  common::Pose T_e_w;
  T_e_w.R << c, -s, 0.0,  //
      s, c, 0.0,          //
      0.0, 0.0, 1.0;
  // p_e_w = mean(p_e_g) - R_e_w mean(p_w_g), with the means un-shifted.
  const common::Vec3 mean_w = ref_w_ + sum_w_ / n;
  const common::Vec3 mean_e = ref_e_ + sum_e_ / n;
  T_e_w.p = mean_e - T_e_w.R * mean_w;
  return ResultPose::ok(T_e_w);
}

}  // namespace mss::gnss
