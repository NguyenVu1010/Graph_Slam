#include "mss/gnss/gnss.hpp"  // own header first (CLAUDE.md)
// The file name differs from the header name, so clang-format cannot detect the main
// include; this comment keeps it in its own block.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Core>

namespace mss::gnss {

namespace {

// Below this horizontal norm the body x axis is treated as vertical (no heading).
constexpr double kMinHeadingNorm = 1e-6;

double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

Eigen::Vector2d rot2(double c, double s, const Eigen::Vector2d& v) {
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

}  // namespace

GnssTrackYawAligner::GnssTrackYawAligner(const GnssConfig& cfg)
    : chunk_fixes_(cfg.align_yaw_chunk_fixes),
      max_sigma_rad_(cfg.align_yaw_max_sigma_rad),
      min_track_m_(cfg.align_yaw_min_track_m),
      min_coherence_(cfg.align_yaw_min_coherence),
      reverse_nsigma_(cfg.align_yaw_reverse_nsigma),
      max_turn_rad_(cfg.align_yaw_max_turn_rad),
      min_var_m2_(cfg.align_yaw_min_var_m2),
      min_wheel_travel_m_(cfg.align_yaw_min_wheel_travel_m),
      yaw_sigma_rad_(std::numeric_limits<double>::infinity()) {}

// docs/derivations/gnss.md section 5.
void GnssTrackYawAligner::close_chunk() {
  const auto n = static_cast<double>(n_chunk_);
  Chunk k;
  k.c = chunk_sum_e_ / n;
  k.m = chunk_sum_l_ / n;          // mean horizontal lever arm in w
  k.psi = chunk_sum_psi_rad_ / n;  // mean unwrapped heading
  k.span = chunk_max_psi_rad_ - chunk_min_psi_rad_;
  k.var = chunk_sum_var_m2_ / (n * n);  // per-axis variance of the mean
  k.wheel_first_m = chunk_wheel_m_[0];
  k.wheel_second_m = chunk_wheel_m_[1];
  k.wheel_first_ok = chunk_wheel_ok_[0];
  k.wheel_second_ok = chunk_wheel_ok_[1];
  n_chunk_ = 0;
  chunk_sum_e_.setZero();
  chunk_sum_psi_rad_ = 0.0;
  chunk_sum_l_.setZero();
  chunk_sum_var_m2_ = 0.0;
  chunk_wheel_m_[0] = 0.0;
  chunk_wheel_m_[1] = 0.0;
  chunk_wheel_ok_[0] = true;
  chunk_wheel_ok_[1] = true;
  if (!have_prev_) {
    have_prev_ = true;
    prev_ = k;
    return;
  }
  const Eigen::Vector2d b = k.c - prev_.c;
  const Eigen::Vector2d l = k.m - prev_.m;
  const double psi_mid = 0.5 * (prev_.psi + k.psi);
  Eigen::Vector2d a(std::cos(psi_mid), std::sin(psi_mid));
  if (std::abs(k.psi - prev_.psi) > max_turn_rad_ || prev_.span > max_turn_rad_ ||
      k.span > max_turn_rad_) {
    // Turn gate: course over ground is only used on near-straight segments.
    a.setZero();  // skipped: contributes nothing; both chunks keep only their other coefficient
    ++n_skipped_turns_;
  } else if (prev_.wheel_second_ok && k.wheel_first_ok) {
    // Direction of travel from the wheel speed over the motion between the two centroids.
    const double travel_m = prev_.wheel_second_m + k.wheel_first_m;
    if (std::abs(travel_m) < min_wheel_travel_m_) {
      // Stationary: no direction, and the sign of wheel noise at rest would alternate a_k
      // and break the telescoping of the chunk noise.
      a.setZero();
      ++n_stationary_;
    } else if (travel_m < 0.0) {
      a = -a;
      ++n_reversals_;
    }
  } else {
    // No wheel data: forward-driving assumption. Once confident, a segment clearly against the
    // estimate (its lever-corrected GNSS displacement against the heading rotated into e) is
    // taken as reverse motion.
    ++n_no_wheel_segments_;
    if (confident()) {
      const double cy = std::cos(yaw_rad_);
      const double sy = std::sin(yaw_rad_);
      const double sigma_seg_m = std::sqrt(prev_.var + k.var);
      if (rot2(cy, sy, a).dot(b - rot2(cy, sy, l)) < -reverse_nsigma_ * sigma_seg_m) {
        a = -a;
        ++n_reversals_;
      }
    }
  }
  // Chunk prev_ now has both its coefficients: prev_a_ (segment before) and -a (this one).
  noise_q_m2_ += prev_.var * (prev_a_ - a).squaredNorm();
  sum_abs_b_m_ += a.squaredNorm() > 0.0 ? b.norm() : 0.0;
  sum_dot_m_ += a.dot(b);
  sum_cross_m_ += cross2(a, b);
  sum_al_dot_m_ += a.dot(l);
  sum_al_cross_m_ += cross2(a, l);
  prev_a_ = a;
  prev_ = k;

  // Closed form (gnss.md section 5): Z = e^{i yaw} (r + L), r > 0, so
  // r = -Re(L) + sqrt(|Z|^2 - Im(L)^2) and e^{i yaw} = Z / (r + L).
  const double z2_m2 = sum_dot_m_ * sum_dot_m_ + sum_cross_m_ * sum_cross_m_;
  const double disc_m2 = z2_m2 - sum_al_cross_m_ * sum_al_cross_m_;
  const double r_m = disc_m2 >= 0.0 ? -sum_al_dot_m_ + std::sqrt(disc_m2) : -1.0;
  if (!(r_m > 0.0)) {
    have_yaw_ = false;  // no solution with forward net motion: "not enough track"
    track_m_ = 0.0;
    yaw_sigma_rad_ = std::numeric_limits<double>::infinity();
    return;
  }
  // arg(Z / (r + L)) = arg(Z) - arg(r + L).
  const double yaw =
      std::atan2(sum_cross_m_, sum_dot_m_) - std::atan2(sum_al_cross_m_, r_m + sum_al_dot_m_);
  // The last chunk's coefficient is a_K = 0, so it adds var_K |a_{K-1}|^2.
  const double var_cross_m2 = noise_q_m2_ + prev_.var * prev_a_.squaredNorm();
  yaw_rad_ = std::atan2(std::sin(yaw), std::cos(yaw));
  track_m_ = r_m;
  yaw_sigma_rad_ = std::sqrt(var_cross_m2) / r_m;
  have_yaw_ = true;
}

common::Result<common::Pose> GnssTrackYawAligner::update(const common::Vec3& p_e_g,
                                                         const common::Mat3& cov_e,
                                                         const common::Pose& T_w_b,
                                                         const common::Vec3& p_b_g,
                                                         double wheel_travel_m) {
  using ResultPose = common::Result<common::Pose>;
  if (!p_e_g.allFinite() || !cov_e.allFinite() || !T_w_b.R.allFinite() || !T_w_b.p.allFinite() ||
      !p_b_g.allFinite() || std::isinf(wheel_travel_m)) {
    return ResultPose::error("non-finite input");
  }
  const double declared_m2 = 0.5 * (cov_e(0, 0) + cov_e(1, 1));
  if (!(declared_m2 > 0.0)) {
    return ResultPose::error("non-positive covariance");
  }
  const Eigen::Vector2d fwd = T_w_b.R.col(0).head<2>();  // body x axis, horizontal part, in w
  const double fwd_norm = fwd.norm();
  if (!(fwd_norm > kMinHeadingNorm)) {
    return ResultPose::error("heading undefined");
  }
  const Eigen::Vector2d u = fwd / fwd_norm;
  // Unwrapped heading: the turn since the last fix, in (-pi, pi].
  psi_rad_ = have_psi_ ? psi_rad_ + std::atan2(cross2(last_fwd_, u), last_fwd_.dot(u))
                       : std::atan2(u.y(), u.x());
  have_psi_ = true;
  last_fwd_ = u;

  const common::Vec3 p_w_g = T_w_b * p_b_g;
  if (n_fixes_ == 0) {
    ref_w_ = p_w_g;
    ref_e_ = p_e_g;
  }
  sum_w_ += p_w_g - ref_w_;
  sum_e_ += p_e_g - ref_e_;
  ++n_fixes_;

  // First or second half of the chunk, for the wheel travel between the chunk centroids.
  const std::size_t half = 2 * n_chunk_ < chunk_fixes_ ? 0U : 1U;
  if (std::isnan(wheel_travel_m)) {
    chunk_wheel_ok_[half] = false;
  } else {
    chunk_wheel_m_[half] += wheel_travel_m;
  }
  chunk_sum_e_ += (p_e_g - ref_e_).head<2>();
  chunk_sum_psi_rad_ += psi_rad_;
  chunk_min_psi_rad_ = n_chunk_ == 0 ? psi_rad_ : std::min(chunk_min_psi_rad_, psi_rad_);
  chunk_max_psi_rad_ = n_chunk_ == 0 ? psi_rad_ : std::max(chunk_max_psi_rad_, psi_rad_);
  chunk_sum_l_ += (T_w_b.R * p_b_g).head<2>();
  chunk_sum_var_m2_ += std::max(declared_m2, min_var_m2_);  // variance floor
  ++n_chunk_;
  if (n_chunk_ >= chunk_fixes_) {
    close_chunk();
  }

  if (!have_yaw_) {
    return ResultPose::error("not enough track");
  }
  if (!(track_m_ >= min_track_m_)) {
    return ResultPose::error("track too short");
  }
  if (!(track_m_ >= min_coherence_ * sum_abs_b_m_)) {
    return ResultPose::error("track not coherent");
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
