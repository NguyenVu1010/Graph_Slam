#include <cmath>

#include <Eigen/Core>

#include "mss/gnss/gnss.hpp"

namespace mss::gnss {

// The thresholds are the header constants kMinPairsForAlignment / kMinHorizontalSpreadM
// until the pipeline adds them to GnssConfig (plan Contract issue 2), so cfg is unused.
GnssAligner::GnssAligner(const GnssConfig& /*cfg*/) {}

// Closed-form 4-DoF fit over running sums; docs/derivations/gnss.md section 4.
common::Result<common::Pose> GnssAligner::update(const common::Vec3& p_e_g,
                                                 const common::Pose& T_w_b,
                                                 const common::Vec3& p_b_g) {
  using ResultPose = common::Result<common::Pose>;
  if (!p_e_g.allFinite() || !T_w_b.R.allFinite() || !T_w_b.p.allFinite() || !p_b_g.allFinite()) {
    return ResultPose::error("non-finite input");
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
  ++n_pairs_;

  if (n_pairs_ < kMinPairsForAlignment) {
    return ResultPose::error("not enough pairs");
  }

  const auto n = static_cast<double>(n_pairs_);
  const Eigen::Vector2d mean_a_h = sum_w_.head<2>() / n;
  const double spread_sq_m2 = sum_w_h_sq_m2_ / n - mean_a_h.squaredNorm();
  const double spread_m = std::sqrt(spread_sq_m2 > 0.0 ? spread_sq_m2 : 0.0);
  if (spread_m < kMinHorizontalSpreadM) {
    return ResultPose::error("insufficient horizontal spread");
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
