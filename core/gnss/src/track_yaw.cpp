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

// Below this horizontal norm the travel axis is treated as vertical (no heading).
constexpr double kMinHeadingNorm = 1e-6;
// Below this half heading change (rad) sin(h) / h is taken as 1 (its relative error is h^2 / 6).
constexpr double kSincSmallRad = 1e-6;

double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

Eigen::Vector2d rot2(double c, double s, const Eigen::Vector2d& v) {
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

}  // namespace

GnssTrackYawAligner::GnssTrackYawAligner(const GnssConfig& cfg)
    : GnssTrackYawAligner(cfg, common::Pose{}) {}

GnssTrackYawAligner::GnssTrackYawAligner(const GnssConfig& cfg, const common::Pose& T_b_o)
    : chunk_fixes_(cfg.align_yaw_chunk_fixes),
      max_gap_fixes_(cfg.align_yaw_wheel_max_gap_fixes),
      merge_max_chunks_(cfg.align_yaw_merge_max_chunks),
      frontend_travel_(cfg.align_yaw_frontend_travel),
      max_sigma_rad_(cfg.align_yaw_max_sigma_rad),
      min_track_m_(cfg.align_yaw_min_track_m),
      min_coherence_(cfg.align_yaw_min_coherence),
      reverse_nsigma_(cfg.align_yaw_reverse_nsigma),
      max_turn_rad_(cfg.align_yaw_max_turn_rad),
      min_var_m2_(cfg.align_yaw_min_var_m2),
      min_wheel_travel_m_(cfg.align_yaw_min_wheel_travel_m),
      wheel_turns_(cfg.align_yaw_wheel_turns),
      slip_frac_(cfg.align_yaw_wheel_slip_frac),
      min_track_wheel_m_(cfg.align_yaw_min_track_wheel_m),
      wheel_restart_m_(cfg.align_yaw_wheel_restart_m),
      T_b_o_(T_b_o),
      yaw_sigma_rad_(std::numeric_limits<double>::infinity()) {}

// gnss.md section 5.1: the wheel-path direction error that the steps into chunk `ch` add to Z,
// across the motion, given the coefficients of the segment before the chunk (`c_before`, the
// chunk is its later one, weights (n - i) / n) and after it (`c_after`, weights i / n); zero
// for a segment whose direction is not from the wheel path. Slip: fully correlated inside the
// chunk, bounded by the triangle inequality across the two segments. Speed profile: an error
// across the step direction u, independent per step, moves Z across by e (w_f c_before . u +
// w_r c_after . u), so the sums of var w w u u^T give it exactly.
double GnssTrackYawAligner::chunk_dir_var_m2(const Chunk& ch,
                                             const Eigen::Vector2d& c_before,
                                             const Eigen::Vector2d& c_after) const {
  const double slip_m =
      slip_frac_ * (ch.path_fall_m * c_before.norm() + ch.path_rise_m * c_after.norm());
  return slip_m * slip_m + c_before.dot(ch.prof_fall_m2 * c_before) +
         c_after.dot(ch.prof_rise_m2 * c_after) + 2.0 * c_before.dot(ch.prof_cross_m2 * c_after);
}

// Adds segment (b, l, a) to `s` (gnss.md section 5): prev_ is still the earlier chunk, which
// now has both its coefficients.
void GnssTrackYawAligner::add_segment(Segments& s,
                                      const Eigen::Vector2d& b,
                                      const Eigen::Vector2d& l,
                                      const Eigen::Vector2d& a,
                                      const Eigen::Vector2d& a_wheel_path,
                                      double wheel_disp_m,
                                      bool no_wheel) const {
  s.noise_q_m2 += prev_.var * (s.prev_a - a).squaredNorm();
  s.wheel_dir_var_m2 += chunk_dir_var_m2(prev_, s.prev_a_wheel_path, a_wheel_path);
  s.prev_a_wheel_path = a_wheel_path;
  s.abs_b_m += a.squaredNorm() > 0.0 ? b.norm() : 0.0;
  s.wheel_disp_m += wheel_disp_m;
  s.dot_m += a.dot(b);
  s.cross_m += cross2(a, b);
  s.al_dot_m += a.dot(l);
  s.al_cross_m += cross2(a, l);
  s.prev_a = a;
  s.n_no_wheel += no_wheel ? 1 : 0;
}

// Closed form (gnss.md section 5): Z = e^{i yaw} (r + L), r > 0, so
// r = -Re(L) + sqrt(|Z|^2 - Im(L)^2) and e^{i yaw} = Z / (r + L); r < 0: no solution. prev_ is
// the last chunk of `s`.
GnssTrackYawAligner::Solution GnssTrackYawAligner::solve(const Segments& s) const {
  Solution out;
  out.z2_m2 = s.dot_m * s.dot_m + s.cross_m * s.cross_m;
  const double disc_m2 = out.z2_m2 - s.al_cross_m * s.al_cross_m;
  out.r_m = disc_m2 >= 0.0 ? -s.al_dot_m + std::sqrt(disc_m2) : -1.0;
  // arg(Z / (r + L)) = arg(Z) - arg(r + L).
  const double yaw =
      std::atan2(s.cross_m, s.dot_m) - std::atan2(s.al_cross_m, out.r_m + s.al_dot_m);
  out.yaw_rad = std::atan2(std::sin(yaw), std::cos(yaw));
  // The last chunk's coefficient is a_K = 0, so it adds var_K |a_{K-1}|^2. The wheel-path
  // segments add their direction error across the motion (gnss.md section 5.1).
  out.var_m2 = s.noise_q_m2 + prev_.var * s.prev_a.squaredNorm() + s.wheel_dir_var_m2 +
               chunk_dir_var_m2(prev_, s.prev_a_wheel_path, Eigen::Vector2d::Zero());
  return out;
}

// docs/derivations/gnss.md section 5.
void GnssTrackYawAligner::close_chunk() {
  const auto n = static_cast<double>(n_chunk_);
  Chunk k;
  k.c = chunk_sum_e_ / n;
  k.mo = chunk_sum_lo_ / n;        // mean horizontal lever arm from o in w
  k.psi = chunk_sum_psi_rad_ / n;  // mean unwrapped heading of the travel axis
  // Heading variation inside the chunk (at least its span), its second half, and the step into
  // its first fix (gnss.md section 5.1).
  k.span =
      std::max(chunk_max_psi_rad_ - chunk_min_psi_rad_, chunk_turn_rad_[0] + chunk_turn_rad_[1]);
  k.turn_first_rad = chunk_turn_rad_[0];
  k.turn_second_rad = chunk_turn_rad_[1];
  k.turn_into_rad = chunk_turn_into_rad_;
  k.var = chunk_sum_var_m2_ / (n * n);  // per-axis variance of the mean
  k.wheel_first_m = chunk_wheel_m_[0];
  k.wheel_second_m = chunk_wheel_m_[1];
  k.wheel_first_ok = chunk_wheel_ok_[0];
  k.wheel_second_ok = chunk_wheel_ok_[1];
  k.q = chunk_sum_q_ / n;
  k.prof_cross_m2 = chunk_prof_cross_m2_;
  k.path_rise_m = chunk_path_rise_m_;
  k.path_fall_m = chunk_path_fall_m_;
  k.prof_rise_m2 = chunk_prof_rise_m2_;
  k.prof_fall_m2 = chunk_prof_fall_m2_;
  k.path_rise_ok = chunk_path_rise_ok_;
  k.path_fall_ok = chunk_path_fall_ok_;
  chunk_sum_q_.setZero();
  chunk_sum_lo_.setZero();
  chunk_prof_cross_m2_.setZero();
  chunk_path_rise_m_ = 0.0;
  chunk_path_fall_m_ = 0.0;
  chunk_prof_rise_m2_.setZero();
  chunk_prof_fall_m2_.setZero();
  k.wtravel_rise_m = chunk_wtravel_rise_m_;
  k.wtravel_fall_m = chunk_wtravel_fall_m_;
  chunk_wtravel_rise_m_ = 0.0;
  chunk_wtravel_fall_m_ = 0.0;
  chunk_path_rise_ok_ = true;
  chunk_path_fall_ok_ = true;
  k.po = chunk_sum_po_ / n;
  k.fe_ok = chunk_fe_ok_;
  chunk_sum_po_.setZero();
  chunk_fe_ok_ = true;
  ++chunk_seq_;
  chunk_turn_rad_[0] = 0.0;
  chunk_turn_rad_[1] = 0.0;
  chunk_turn_into_rad_ = 0.0;
  n_chunk_ = 0;
  chunk_sum_e_.setZero();
  chunk_sum_psi_rad_ = 0.0;
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
  // The chunk just before k: prev_, or the last chunk a stationary segment from prev_ was
  // extended over (align_yaw_merge_max_chunks), whose totals are in carry_.
  const Chunk& near = carry_.chunks > 0 ? near_ : prev_;
  const Eigen::Vector2d b = k.c - prev_.c;
  Eigen::Vector2d l = k.mo - prev_.mo;
  const double psi_mid = 0.5 * (prev_.psi + k.psi);
  Eigen::Vector2d a(std::cos(psi_mid), std::sin(psi_mid));
  Eigen::Vector2d a_wheel_path = Eigen::Vector2d::Zero();  // a when it comes from the wheel path
  double wheel_disp_m = 0.0;  // wheel-predicted displacement of a segment used with wheel data
  bool no_wheel = false;
  // Heading variation between the two chunk centres: the net change, and the total absolute
  // variation of the steps between them (a turn and its opposite hidden in a GNSS outage
  // cancel in the net change only).
  const double turn_between_rad = std::max(
      std::abs(k.psi - prev_.psi), prev_.turn_second_rad + k.turn_into_rad + k.turn_first_rad);
  // Wheel travel between the centres, and the front end (study section 16.13): when every step
  // from the first fix of prev_ to the last of k observed the travel axis, a displacement of o
  // below align_yaw_min_wheel_travel_m is a still body whatever the wheels say.
  const double travel_m = carry_.travel_m + near.wheel_second_m + k.wheel_first_m;
  const double wtravel_m = prev_.wtravel_rise_m + k.wtravel_fall_m;  // used when not extended
  const bool wheel_ok = carry_.wheel_ok && near.wheel_second_ok && k.wheel_first_ok;
  const bool fe_steps_ok = carry_.fe_ok && prev_.fe_ok && k.fe_ok;
  const bool fe_ok = frontend_travel_ && fe_steps_ok;
  const Eigen::Vector2d fe_disp = k.po - prev_.po;
  const bool fe_still = fe_ok && fe_disp.norm() < min_wheel_travel_m_;
  bool stationary = false;

  if (carry_.chunks > 0) {
    // Extended segment (study section 16.13): the mean heading of its two end chunks does not
    // represent its span (a turn at one end biases it), so its direction is the path of o
    // between the chunk means: the front end where it observed the travel axis, else the wheel
    // path. Without either it is skipped. The path follows the turns, so the turn gate does not
    // apply. The slip and profile terms of the chunks it was extended over are left out: their
    // wheel travel is below align_yaw_min_wheel_travel_m in total (else it would not have been
    // extended), so their slip error is below slip x 0.2 m (1 cm at 0.05) against the 0.16 m
    // noise of a chunk mean (study section 16.13: NEES 0.95 with them, 0.94 without, at a 10 deg
    // skid and slip 0.15).
    const Eigen::Vector2d g_fe = fe_disp + l;
    const Eigen::Vector2d g = k.q - prev_.q + l;
    if (fe_still || (wheel_ok && std::abs(travel_m) < min_wheel_travel_m_)) {
      stationary = true;
    } else if (fe_ok && g_fe.squaredNorm() > 0.0) {
      a = g_fe / g_fe.norm();
      l.setZero();
      wheel_disp_m = g_fe.norm();
      ++n_wheel_turns_;
    } else if (wheel_turns_ && carry_.path_ok && prev_.path_rise_ok && k.path_fall_ok &&
               g.squaredNorm() > 0.0) {
      a = g / g.norm();
      a_wheel_path = a;
      l.setZero();
      wheel_disp_m = g.norm();

      ++n_wheel_turns_;
      if (travel_m < 0.0) {
        ++n_reversals_;
      }
    } else {
      a.setZero();
      ++n_skipped_turns_;
    }
  } else if (turn_between_rad > max_turn_rad_ || prev_.span > max_turn_rad_ ||
             k.span > max_turn_rad_) {
    // Turn gate: course over ground along the mean heading is only used on near-straight
    // segments. With the wheel path the direction is the antenna displacement in w instead
    // (gnss.md section 5.1): the odometry-frame path plus R_w_b (p_b_g - p_b_o).
    const Eigen::Vector2d g = k.q - prev_.q + l;
    const double path_m = prev_.path_rise_m + k.path_fall_m;
    const Eigen::Vector2d g_fe = fe_disp + l;  // the same from the front-end path of o
    if (fe_ok) {
      // The LiDAR observed the travel axis (study section 16.13): the front-end path is the true
      // one (a skid included) up to the scan-matching error, so it adds no direction error.
      if (fe_still || !(g_fe.squaredNorm() > 0.0)) {
        a.setZero();
        ++n_stationary_;
      } else {
        a = g_fe / g_fe.norm();
        l.setZero();
        wheel_disp_m = g_fe.norm();
        ++n_wheel_turns_;
      }
    } else if (!(wheel_turns_ && prev_.path_rise_ok && k.path_fall_ok)) {
      a.setZero();  // skipped: contributes nothing; both chunks keep only their other coefficient
      ++n_skipped_turns_;
    } else if (path_m < min_wheel_travel_m_ || !(g.squaredNorm() > 0.0)) {
      a.setZero();  // stationary (turning in place): no direction of travel
      ++n_stationary_;
    } else {
      a = g / g.norm();
      a_wheel_path = a;
      l.setZero();  // the lever arm is inside g: nothing for L
      wheel_disp_m = g.norm();
      ++n_wheel_turns_;
      if (prev_.wheel_second_m + k.wheel_first_m < 0.0) {
        ++n_reversals_;
      }
    }
  } else if (wheel_ok) {
    // Direction of travel from the wheel speed over the motion between the two centroids.
    if (fe_still || std::abs(travel_m) < min_wheel_travel_m_) {
      // Stationary: no direction, and the sign of wheel noise at rest would alternate a_k
      // and break the telescoping of the chunk noise.
      stationary = true;
    } else {
      wheel_disp_m = std::abs(wtravel_m);
      if (travel_m < 0.0) {
        a = -a;
        ++n_reversals_;
      }
    }
  } else if (fe_ok) {
    // No wheel data, the LiDAR observed the travel axis: the front end gives the direction.
    if (fe_still) {
      stationary = true;
    } else if (fe_disp.dot(a) < 0.0) {
      a = -a;
      ++n_reversals_;
    }
    no_wheel = true;
  } else {
    // No wheel data: forward-driving assumption. Once confident, a segment clearly against the
    // estimate (its lever-corrected GNSS displacement against the heading rotated into e) is
    // taken as reverse motion.
    ++n_no_wheel_segments_;
    no_wheel = true;
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
  if (stationary) {
    ++n_stationary_;
    // Extended only while a path of o will be there to give its direction: the front end, or the
    // wheel path over every chunk so far (else it is added as a stationary segment, as before).
    const bool path_possible = fe_ok || (wheel_turns_ && carry_.path_ok && prev_.path_rise_ok &&
                                         k.path_rise_ok && k.path_fall_ok);
    if (carry_.chunks < merge_max_chunks_ && path_possible) {
      // Extend the segment over k (study section 16.13): prev_ stays its earlier chunk, so a slow
      // robot's travel adds up; nothing is added until it has travel or reaches the limit.
      carry_.chunks += 1;
      carry_.travel_m = travel_m;
      carry_.wheel_ok = wheel_ok;
      carry_.fe_ok = fe_steps_ok;
      carry_.path_ok = carry_.path_ok && k.path_rise_ok && k.path_fall_ok;
      near_ = k;
      return;
    }
    a.setZero();
  }
  add_segment(all_, b, l, a, a_wheel_path, wheel_disp_m, no_wheel);
  add_segment(recent_, b, l, a, a_wheel_path, wheel_disp_m, no_wheel);
  if (no_wheel) {
    // A segment without wheel data cannot be checked against the wheels: it breaks the window,
    // which starts again at the next segment (a pending restart stays pending). Keeping it would
    // switch the check off for good (review R2-1 of fix round 2).
    recent_ = Segments{};
  }
  prev_ = k;
  carry_ = Carry{};

  Solution sol = solve(all_);
  // Wheel/GNSS consistency (gnss.md section 5.1). The segments since the last mismatch must agree
  // with the wheel travel within their noise (align_yaw_reverse_nsigma sigmas) plus the slip
  // fraction; otherwise they start again from here (wheels turning on a still body, or an
  // odometer with a wrong scale), and every older segment is suspect: the alignment waits until
  // the recent segments agree with the wheels over align_yaw_wheel_restart_m of wheel travel, and
  // then they replace all segments. So the time the wheels turned on a still body neither delays
  // the alignment by the spun distance nor stays in Z.
  if (recent_.wheel_disp_m > 0.0 && recent_.n_no_wheel == 0) {
    const Solution rec = solve(recent_);
    const double tol_m =
        reverse_nsigma_ * std::sqrt(rec.var_m2) + slip_frac_ * recent_.wheel_disp_m;
    if (std::abs(std::max(rec.r_m, 0.0) - recent_.wheel_disp_m) > tol_m) {
      ++n_wheel_mismatches_;
      recent_ = Segments{};
      restart_pending_ = true;
    } else if (restart_pending_ && recent_.wheel_disp_m >= wheel_restart_m_) {
      ++n_wheel_restarts_;
      all_ = recent_;
      sol = rec;
      restart_pending_ = false;
    }
  }
  if (!(sol.r_m > 0.0)) {
    have_yaw_ = false;  // no solution with forward net motion: "not enough track"
    track_m_ = 0.0;
    yaw_sigma_rad_ = std::numeric_limits<double>::infinity();
    return;
  }
  yaw_rad_ = sol.yaw_rad;
  track_m_ = sol.r_m;
  // To first order delta_yaw = Im(conj(Z) delta_Z) / |Z|^2 (r and arg(r + L) depend only on
  // |Z|), so the sigma divides by |Z| = |r + L|, not by r (they differ when the lever arm
  // turns).
  yaw_sigma_rad_ = std::sqrt(sol.var_m2 / sol.z2_m2);
  have_yaw_ = true;
}

common::Result<common::Pose> GnssTrackYawAligner::update(const common::Vec3& p_e_g,
                                                         const common::Mat3& cov_e,
                                                         const common::Pose& T_w_b,
                                                         const common::Vec3& p_b_g,
                                                         double wheel_odometer_m,
                                                         double heading_variation_rad,
                                                         double travel_unobserved_scans) {
  using ResultPose = common::Result<common::Pose>;
  if (!p_e_g.allFinite() || !cov_e.allFinite() || !T_w_b.R.allFinite() || !T_w_b.p.allFinite() ||
      !p_b_g.allFinite() || std::isinf(wheel_odometer_m) || std::isinf(heading_variation_rad) ||
      std::isinf(travel_unobserved_scans)) {
    return ResultPose::error("non-finite input");
  }
  if (!(cov_e(0, 0) > 0.0) || !(cov_e(1, 1) > 0.0)) {
    return ResultPose::error("non-positive covariance");
  }
  const double declared_m2 = 0.5 * (cov_e(0, 0) + cov_e(1, 1));
  // Travel axis: the odometry frame's x axis (R_w_b R_b_o), horizontal part, in w. A wheeled
  // robot moves along it; it equals the body x axis when T_b_o is identity.
  const common::Vec3 x_o_w = T_w_b.R * T_b_o_.R.col(0);
  const Eigen::Vector2d fwd = x_o_w.head<2>();
  const double fwd_norm = fwd.norm();
  if (!(fwd_norm > kMinHeadingNorm)) {
    return ResultPose::error("heading undefined");
  }
  const Eigen::Vector2d u = fwd / fwd_norm;
  // Unwrapped heading: the turn since the last fix, in (-pi, pi].
  const bool had_psi = have_psi_;
  const double psi_prev_rad = psi_rad_;
  psi_rad_ = have_psi_ ? psi_rad_ + std::atan2(cross2(last_fwd_, u), last_fwd_.dot(u))
                       : std::atan2(u.y(), u.x());
  have_psi_ = true;
  last_fwd_ = u;
  const double dpsi_rad = psi_rad_ - psi_prev_rad;
  // Total absolute heading variation of the step (gnss.md section 5.1): the difference of the
  // front-end readings since the last fix added here, at least the net change; the net change
  // alone when a reading is missing.
  const double turn_rad =
      had_psi && !std::isnan(heading_variation_rad) && !std::isnan(last_heading_variation_rad_)
          ? std::max(heading_variation_rad - last_heading_variation_rad_, std::abs(dpsi_rad))
          : std::abs(dpsi_rad);
  last_heading_variation_rad_ = heading_variation_rad;

  const common::Vec3 p_w_g = T_w_b * p_b_g;
  if (n_fixes_ == 0) {
    ref_w_ = p_w_g;
    ref_e_ = p_e_g;
  }
  sum_w_ += p_w_g - ref_w_;
  sum_e_ += p_e_g - ref_e_;
  ++n_fixes_;

  // First or second half of the chunk, for the wheel travel between the chunk centroids. The
  // travel of this fix is the odometer difference since the last fix added here, so travel
  // during fixes rejected before the aligner is carried, not lost.
  const std::size_t half = 2 * n_chunk_ < chunk_fixes_ ? 0U : 1U;
  if (n_chunk_ == 0) {
    chunk_turn_into_rad_ = had_psi ? turn_rad : 0.0;
  } else {
    chunk_turn_rad_[half] += turn_rad;
  }
  // The odometer is absolute, so a run of at most align_yaw_wheel_max_gap_fixes fixes without a
  // reading is bridged by the next reading, whose travel lands on this fix (study section
  // 16.13); a longer run leaves every half it touched without wheel data.
  const bool reading = !std::isnan(wheel_odometer_m);
  const bool bridged = reading && odo_missing_ > 0;
  const bool step_ok = reading && !std::isnan(last_odometer_m_) && odo_missing_ <= max_gap_fixes_;
  const double step_m = step_ok ? wheel_odometer_m - last_odometer_m_ : 0.0;
  if (!reading) {
    if (odo_missing_ == 0) {
      gap_seq_ = chunk_seq_;
      gap_half_ = half;
    }
    ++odo_missing_;
    if (odo_missing_ > max_gap_fixes_) {
      // Not bridged. A run that started in the chunk before (in its second half, the bound being
      // below half a chunk) overflows in this chunk's first half, which the segment using that
      // second half also uses.
      chunk_wheel_ok_[half] = false;
      if (gap_seq_ == chunk_seq_) {
        chunk_wheel_ok_[gap_half_] = false;
      }
    }
  } else if (step_ok) {
    chunk_wheel_m_[half] += step_m;
    // The travel between the chunk centroids, weighted as the step enters them (gnss.md 5.1).
    const double w_r = static_cast<double>(n_chunk_) / static_cast<double>(chunk_fixes_);
    chunk_wtravel_rise_m_ += w_r * step_m;
    chunk_wtravel_fall_m_ += (1.0 - w_r) * step_m;
  } else {
    chunk_wheel_ok_[half] = false;
  }
  if (reading) {
    last_odometer_m_ = wheel_odometer_m;
    odo_missing_ = 0;
  }
  if (wheel_turns_) {
    // Wheel path (gnss.md section 5.1): the odometry frame o (the odometer measures its forward
    // speed) moves along the chord of its heading arc, exact for constant speed and yaw rate
    // between the two fixes. The step into fix i of a chunk enters Q_{k+1} - Q_k with weight
    // i / n when the chunk is the earlier one (rise) and (n - i) / n when it is the later one
    // (fall). A step whose total heading variation exceeds the turn gate (a GNSS outage over a
    // turn, or over two opposite turns) has no known shape: its chunks are not used through the
    // wheel path.
    const double w_rise = static_cast<double>(n_chunk_) / static_cast<double>(chunk_fixes_);
    const double w_fall = 1.0 - w_rise;
    if (step_ok && !bridged && had_psi && turn_rad <= max_turn_rad_) {
      const double h_rad = 0.5 * dpsi_rad;
      const double sinc = std::abs(h_rad) > kSincSmallRad ? std::sin(h_rad) / h_rad : 1.0;
      const double mid_rad = psi_prev_rad + h_rad;
      path_q_ += step_m * sinc * Eigen::Vector2d(std::cos(mid_rad), std::sin(mid_rad));
      // Unknown speed profile between the fixes: the step direction is within +-h of the mid
      // heading, uniform: variance (s 2h)^2 / 12 across.
      const Eigen::Vector2d u_step(std::cos(mid_rad), std::sin(mid_rad));
      const Eigen::Matrix2d prof_m2 =
          (step_m * step_m * 4.0 * h_rad * h_rad / 12.0) * (u_step * u_step.transpose());
      chunk_path_rise_m_ += w_rise * std::abs(step_m);
      chunk_path_fall_m_ += w_fall * std::abs(step_m);
      chunk_prof_rise_m2_ += (w_rise * w_rise) * prof_m2;
      chunk_prof_fall_m2_ += (w_fall * w_fall) * prof_m2;
      chunk_prof_cross_m2_ += (w_rise * w_fall) * prof_m2;
    } else {
      chunk_path_fall_ok_ = false;
      chunk_path_rise_ok_ = chunk_path_rise_ok_ && n_chunk_ == 0;
    }
    chunk_sum_q_ += path_q_;
  }
  chunk_sum_e_ += (p_e_g - ref_e_).head<2>();
  chunk_sum_psi_rad_ += psi_rad_;
  chunk_min_psi_rad_ = n_chunk_ == 0 ? psi_rad_ : std::min(chunk_min_psi_rad_, psi_rad_);
  chunk_max_psi_rad_ = n_chunk_ == 0 ? psi_rad_ : std::max(chunk_max_psi_rad_, psi_rad_);
  chunk_sum_lo_ += (T_w_b.R * (p_b_g - T_b_o_.p)).head<2>();
  // Front end (study section 16.13): the position of o, and whether the step into this fix
  // observed the travel axis (the count of unobserved scans did not change).
  chunk_sum_po_ += (T_w_b.p + T_w_b.R * T_b_o_.p - ref_w_).head<2>();
  chunk_fe_ok_ = chunk_fe_ok_ && !std::isnan(travel_unobserved_scans) &&
                 !std::isnan(last_unobserved_scans_) &&
                 !(travel_unobserved_scans > last_unobserved_scans_);
  last_unobserved_scans_ = travel_unobserved_scans;
  chunk_sum_var_m2_ += std::max(declared_m2, min_var_m2_);  // variance floor
  ++n_chunk_;
  if (n_chunk_ >= chunk_fixes_) {
    close_chunk();
  }

  if (!have_yaw_) {
    return ResultPose::error("not enough track");
  }
  if (!(track_m_ >= min_track())) {
    return ResultPose::error("track too short");
  }
  if (restart_pending_ || !(track_m_ >= min_coherence_ * all_.abs_b_m)) {
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
