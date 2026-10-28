#include "mss/gnss/gnss.hpp"  // own header first (CLAUDE.md)
// The file name differs from the header name, so clang-format cannot detect the main
// include; this comment keeps it in its own block.

#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace mss::gnss {

namespace {

// Below this horizontal norm the body x axis is treated as vertical (no heading).
constexpr double kMinHeadingNorm = 1e-6;
// Fixed-point iterations of the lever-arm correction (gnss.md section 5); each one shrinks the
// remaining error by about |lever displacement| / |segment| per segment (< 0.2 in practice).
constexpr int kLeverIterations = 4;

double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

}  // namespace

GnssTrackYawAligner::GnssTrackYawAligner(const GnssConfig& cfg)
    : chunk_fixes_(cfg.align_yaw_chunk_fixes),
      max_sigma_rad_(cfg.align_yaw_max_sigma_rad),
      reverse_nsigma_(cfg.align_yaw_reverse_nsigma),
      yaw_sigma_rad_(std::numeric_limits<double>::infinity()) {}

// docs/derivations/gnss.md section 5.
void GnssTrackYawAligner::close_chunk() {
  const auto n = static_cast<double>(n_chunk_);
  const Eigen::Vector2d c = chunk_sum_e_ / n;
  const Eigen::Vector2d m = chunk_sum_l_ / n;  // mean horizontal lever arm in w
  const double h_norm = chunk_sum_h_.norm();
  // Opposite headings within one chunk (a U-turn in place) cancel: keep the previous chunk's
  // heading direction rather than an arbitrary one.
  const Eigen::Vector2d h =
      h_norm > kMinHeadingNorm ? Eigen::Vector2d(chunk_sum_h_ / h_norm) : Eigen::Vector2d(prev_h_);
  const double var_m2 = chunk_sum_var_m2_ / (n * n);  // per-axis variance of the mean
  n_chunk_ = 0;
  chunk_sum_e_.setZero();
  chunk_sum_h_.setZero();
  chunk_sum_l_.setZero();
  chunk_sum_var_m2_ = 0.0;
  if (!have_prev_) {
    have_prev_ = true;
    prev_c_ = c;
    prev_m_ = m;
    prev_h_ = h;
    prev_var_m2_ = var_m2;
    return;
  }
  // Antenna displacement in e minus the lever arm's rotation, rotated with the current yaw.
  const Eigen::Vector2d l = m - prev_m_;
  const double cy = std::cos(yaw_rad_);
  const double sy = std::sin(yaw_rad_);
  const Eigen::Vector2d b =
      c - prev_c_ -
      (have_yaw_ ? Eigen::Vector2d(cy * l.x() - sy * l.y(), sy * l.x() + cy * l.y())
                 : Eigen::Vector2d(Eigen::Vector2d::Zero()));
  const Eigen::Vector2d hs = prev_h_ + h;
  Eigen::Vector2d a = hs.norm() > kMinHeadingNorm ? Eigen::Vector2d(hs / hs.norm()) : prev_h_;
  // Reversal: only against a confident estimate (sigma_yaw within the alignment threshold).
  if (have_yaw_ && yaw_sigma_rad_ <= max_sigma_rad_) {
    const Eigen::Vector2d a_e(cy * a.x() - sy * a.y(), sy * a.x() + cy * a.y());
    const double sigma_seg_m = std::sqrt(prev_var_m2_ + var_m2);
    if (a_e.dot(b) < -reverse_nsigma_ * sigma_seg_m) {
      a = -a;
      ++n_reversals_;
    }
  }
  // Chunk prev_c_ now has both its coefficients: prev_a_ (segment before) and -a (this one).
  noise_q_m2_ += prev_var_m2_ * (prev_a_ - a).squaredNorm();
  const Eigen::Vector2d b_raw = c - prev_c_;  // uncorrected: the correction is redone below
  sum_dot_m_ += a.dot(b_raw);
  sum_cross_m_ += cross2(a, b_raw);
  sum_al_dot_m_ += a.dot(l);
  sum_al_cross_m_ += cross2(a, l);
  prev_a_ = a;
  prev_c_ = c;
  prev_m_ = m;
  prev_h_ = h;
  prev_var_m2_ = var_m2;

  // yaw = atan2(sum a x b', sum a . b') with b' = b - Rz(yaw) l: with Rz(y) l = cos l + sin J l,
  // a x (Rz l) = cos (a x l) + sin (a . l) and a . (Rz l) = cos (a . l) - sin (a x l).
  double yaw = std::atan2(sum_cross_m_, sum_dot_m_);
  double cross_m = sum_cross_m_;
  double dot_m = sum_dot_m_;
  for (int it = 0; it < kLeverIterations; ++it) {
    const double cw = std::cos(yaw);
    const double sw = std::sin(yaw);
    cross_m = sum_cross_m_ - (cw * sum_al_cross_m_ + sw * sum_al_dot_m_);
    dot_m = sum_dot_m_ - (cw * sum_al_dot_m_ - sw * sum_al_cross_m_);
    yaw = std::atan2(cross_m, dot_m);
  }
  const double s_m = std::hypot(dot_m, cross_m);
  if (!(s_m > 0.0)) {
    yaw_sigma_rad_ = std::numeric_limits<double>::infinity();
    return;
  }
  // The last chunk's coefficient is a_K = 0, so it adds var_K |a_{K-1}|^2 = var_K.
  const double var_cross_m2 = noise_q_m2_ + prev_var_m2_ * prev_a_.squaredNorm();
  yaw_rad_ = yaw;
  yaw_sigma_rad_ = std::sqrt(var_cross_m2) / s_m;
  have_yaw_ = true;
}

common::Result<common::Pose> GnssTrackYawAligner::update(const common::Vec3& p_e_g,
                                                         const common::Mat3& cov_e,
                                                         const common::Pose& T_w_b,
                                                         const common::Vec3& p_b_g) {
  using ResultPose = common::Result<common::Pose>;
  if (!p_e_g.allFinite() || !cov_e.allFinite() || !T_w_b.R.allFinite() || !T_w_b.p.allFinite() ||
      !p_b_g.allFinite()) {
    return ResultPose::error("non-finite input");
  }
  const Eigen::Vector2d fwd = T_w_b.R.col(0).head<2>();  // body x axis, horizontal part, in w
  const double fwd_norm = fwd.norm();
  if (!(fwd_norm > kMinHeadingNorm)) {
    return ResultPose::error("heading undefined");
  }

  const common::Vec3 p_w_g = T_w_b * p_b_g;
  if (n_fixes_ == 0) {
    ref_w_ = p_w_g;
    ref_e_ = p_e_g;
  }
  sum_w_ += p_w_g - ref_w_;
  sum_e_ += p_e_g - ref_e_;
  ++n_fixes_;

  chunk_sum_e_ += (p_e_g - ref_e_).head<2>();
  chunk_sum_h_ += fwd / fwd_norm;
  chunk_sum_l_ += (T_w_b.R * p_b_g).head<2>();
  chunk_sum_var_m2_ += 0.5 * (cov_e(0, 0) + cov_e(1, 1));
  ++n_chunk_;
  if (n_chunk_ >= chunk_fixes_) {
    close_chunk();
  }

  if (!have_yaw_) {
    return ResultPose::error("not enough track");
  }
  if (!(yaw_sigma_rad_ <= max_sigma_rad_)) {
    return ResultPose::error("yaw sigma too large");
  }
  const double c = std::cos(yaw_rad_);
  const double s = std::sin(yaw_rad_);
  common::Pose T_e_w;
  T_e_w.R << c, -s, 0.0,  //
      s, c, 0.0,          //
      0.0, 0.0, 1.0;
  const auto n = static_cast<double>(n_fixes_);
  const common::Vec3 mean_w = ref_w_ + sum_w_ / n;
  const common::Vec3 mean_e = ref_e_ + sum_e_ / n;
  T_e_w.p = mean_e - T_e_w.R * mean_w;
  return ResultPose::ok(T_e_w);
}

}  // namespace mss::gnss
