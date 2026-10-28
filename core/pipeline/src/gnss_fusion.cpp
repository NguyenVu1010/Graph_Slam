#include "gnss_fusion.hpp"

#include <Eigen/Core>

namespace mss::pipeline::detail {

GnssFusion::GnssFusion(const gnss::GnssConfig& cfg, const common::Vec3& p_b_g)
    : cfg_(cfg),
      max_cov_m2_(cfg.max_cov_m2),
      chi2_gate_(cfg.chi2_gate),
      p_b_g_(p_b_g),
      aligner_(cfg),
      track_aligner_(cfg),
      early_open_(cfg.attach_prealign_fixes) {
  if (cfg.attach_prealign_fixes) {
    early_.reserve(static_cast<std::size_t>(cfg.prealign_max_fixes));
  }
}

common::Status GnssFusion::chi_square(const common::GnssMeasurement& m,
                                      const common::Pose& T_w_b,
                                      const common::PoseCovariance& P_w_b) const {
  // G-02: S = m.cov + J P J^T, P := 0 when not finite (pipeline.md section 5).
  Eigen::Matrix<double, 3, 6> J_r_xi;
  const common::Vec3 r = gnss::gnss_residual(m.p_e_g, T_e_w_, T_w_b, p_b_g_, &J_r_xi);
  common::Mat3 s_e = m.cov;
  if (P_w_b.allFinite()) {
    s_e += J_r_xi * P_w_b * J_r_xi.transpose();
  }
  return gnss::chi_square_gate(r, s_e, chi2_gate_);
}

common::Status GnssFusion::on_fix(const common::GnssMeasurement& m,
                                  const common::Pose& T_w_b_fix,
                                  const common::PoseCovariance& P_w_b,
                                  FixOutcome* outcome,
                                  double wheel_travel_m) {
  FixOutcome dummy = FixOutcome::kAccepted;
  FixOutcome& out = outcome != nullptr ? *outcome : dummy;
  common::Status gate = gnss::gate_fix(m, max_cov_m2_);
  if (!gate.is_ok()) {
    out = FixOutcome::kGateFix;
    return gate;
  }
  if (!T_w_b_fix.R.allFinite() || !T_w_b_fix.p.allFinite()) {
    out = FixOutcome::kNonFinitePose;
    return common::Status::error("non-finite pose");
  }
  if (aligned_) {
    common::Status chi2 = chi_square(m, T_w_b_fix, P_w_b);
    if (!chi2.is_ok()) {
      if (forced_ || ++consecutive_rejects_ < cfg_.max_consecutive_rejects) {
        out = FixOutcome::kChiSquare;
        return chi2;
      }
      // Lock-out guard: the front end drifted past the gate (e.g. during an outage) and no
      // fix can pull it back. Restart the alignment on the current front end; the new
      // factors share the graph's T_e_w with the old ones (ADR-019) and pull the chain back.
      aligner_ = gnss::GnssAligner(cfg_);
      track_aligner_ = gnss::GnssTrackYawAligner(cfg_);
      aligned_ = false;
      consecutive_rejects_ = 0;
      ++n_realign_;
    } else {
      consecutive_rejects_ = 0;
    }
  }
  // ADR-023: the track-yaw aligner replaces GnssAligner when gnss.align_yaw_from_track.
  const common::Result<common::Pose> T =
      forced_ ? common::Result<common::Pose>::error("forced T_e_w")
      : cfg_.align_yaw_from_track
          ? track_aligner_.update(m.p_e_g, m.cov, T_w_b_fix, p_b_g_, wheel_travel_m)
          : aligner_.update(m.p_e_g, T_w_b_fix, p_b_g_);
  if (T.is_ok()) {
    T_e_w_ = T.value();
    if (!aligned_) {
      ++n_alignments_;
    }
    aligned_ = true;
  }
  if (!aligned_) {
    if (early_open_) {  // ADR-022: kept until aligned
      if (early_.size() < early_.capacity()) {
        EarlyFix e;
        e.m = m;
        e.T_w_b_fix = T_w_b_fix;
        e.P_w_b = P_w_b;
        e.shift_w.setZero();
        early_.push_back(e);
      } else {
        ++n_prealign_dropped_;  // full: keep the earlier (static) ones
      }
    }
    out = FixOutcome::kNotAligned;
    return common::Status::error("gnss not aligned yet");
  }
  early_open_ = false;  // first alignment: the buffer closes (one-shot, also after a realign)
  if (!pending_.push(PendingFix{m, T_w_b_fix})) {
    ++n_pending_dropped_;  // the oldest pending fix was overwritten
  }
  out = FixOutcome::kAccepted;
  return common::Status::ok();
}

std::size_t GnssFusion::collect(common::Timestamp t_k,
                                const common::Pose& T_w_bk,
                                std::array<common::GnssMeasurement, kMaxAuxQueue>& out) {
  std::size_t n = 0;
  const common::Vec3 p_w_g_k = T_w_bk * p_b_g_;
  while (!pending_.empty() && pending_.front().m.t <= t_k) {
    PendingFix f;
    pending_.pop(f);
    // p_e_g' = p_e_g + R_e_w (T_w_bk p_b_g - T_w_b(t_fix) p_b_g), cov unchanged.
    const common::Vec3 p_w_g_fix = f.T_w_b_fix * p_b_g_;
    const common::Vec3 shift_w =
        shift_oracle_ != nullptr ? (*shift_oracle_)(f.m.t, t_k) : common::Vec3(p_w_g_k - p_w_g_fix);
    out[n] = f.m;
    out[n].p_e_g = f.m.p_e_g + T_e_w_.R * shift_w;
    ++n;
  }
  return n;
}

common::Status GnssFusion::on_preinit_fix(const common::GnssMeasurement& m) {
  if (!early_open_) {
    return common::Status::ok();
  }
  common::Status gate = gnss::gate_fix(m, max_cov_m2_);
  if (!gate.is_ok()) {
    return gate;
  }
  if (early_.size() == early_.capacity()) {
    // Oldest first: it is the least likely to be static. O(capacity) moves of one entry
    // (about 0.6 kB each, no allocation) per fix, only while waiting for the IMU
    // initialisation with a full buffer: cheap at GNSS rates.
    early_.erase(early_.begin());
    ++n_prealign_dropped_;
  }
  EarlyFix e;
  e.m = m;
  e.P_w_b.setZero();
  e.shift_w.setZero();
  e.preinit = true;
  early_.push_back(e);
  return common::Status::ok();
}

void GnssFusion::on_first_keyframe(common::KeyId id,
                                   common::Timestamp t_k,
                                   const common::Pose& T_w_b0,
                                   const common::PoseCovariance& P0,
                                   common::Timestamp static_from,
                                   bool static_ok) {
  std::size_t kept = 0;
  for (std::size_t i = 0; i < early_.size(); ++i) {
    EarlyFix& e = early_[i];
    if (e.preinit) {
      if (!static_ok || e.m.t < static_from || e.m.t > t_k) {
        ++n_prealign_not_static_;
        continue;
      }
      // At rest at keyframe 0's pose: no shift.
      e.T_w_b_fix = T_w_b0;
      e.P_w_b = P0;
      e.shift_w.setZero();
      e.kf = id;
      e.t_k = t_k;
      e.assigned = true;
      e.preinit = false;
      e.static_kf0 = true;
    }
    early_[kept++] = e;
  }
  early_.resize(kept);  // shrinks: no allocation
  set_keyframe_reference(id, t_k, T_w_b0);
}

void GnssFusion::on_keyframe(common::KeyId id, common::Timestamp t_k, const common::Pose& T_w_bk) {
  const common::Vec3 p_w_g_k = T_w_bk * p_b_g_;
  for (EarlyFix& e : early_) {
    if (e.assigned || e.preinit || e.m.t > t_k) {
      continue;
    }
    const common::Vec3 p_w_g_fix = e.T_w_b_fix * p_b_g_;
    // Test-only: the shift oracle replaces the front-end displacement, as in collect().
    if (have_prev_kf_ && e.m.t.ns - prev_kf_t_.ns <= t_k.ns - e.m.t.ns) {
      e.shift_w = shift_oracle_ != nullptr ? (*shift_oracle_)(e.m.t, prev_kf_t_)
                                           : common::Vec3(prev_kf_T_ * p_b_g_ - p_w_g_fix);
      e.kf = prev_kf_;
      e.t_k = prev_kf_t_;
    } else {
      e.shift_w = shift_oracle_ != nullptr ? (*shift_oracle_)(e.m.t, t_k)
                                           : common::Vec3(p_w_g_k - p_w_g_fix);
      e.kf = id;
      e.t_k = t_k;
    }
    e.assigned = true;
  }
}

void GnssFusion::set_keyframe_reference(common::KeyId id,
                                        common::Timestamp t_k,
                                        const common::Pose& T_w_ref) {
  have_prev_kf_ = true;
  prev_kf_ = id;
  prev_kf_t_ = t_k;
  prev_kf_T_ = T_w_ref;
}

std::size_t GnssFusion::take_prealign(std::array<EarlyFactor, kMaxAuxQueue>& out) {
  if (!aligned_) {
    return 0;
  }
  std::size_t n = 0;
  std::size_t kept = 0;
  for (std::size_t i = 0; i < early_.size(); ++i) {
    const EarlyFix& e = early_[i];
    if (!e.assigned || n == out.size()) {
      early_[kept++] = e;
      continue;
    }
    if (!chi_square(e.m, e.T_w_b_fix, e.P_w_b).is_ok()) {
      ++n_prealign_rejected_;
      continue;
    }
    // Same shift as collect(): p_e_g' = p_e_g + R_e_w shift_w, cov unchanged.
    out[n].m = e.m;
    out[n].m.p_e_g = e.m.p_e_g + T_e_w_.R * e.shift_w;
    out[n].kf = e.kf;
    out[n].t_k = e.t_k;
    n_prealign_static_ += e.static_kf0 ? 1U : 0U;
    ++n;
  }
  early_.resize(kept);
  n_prealign_attached_ += n;
  return n;
}

}  // namespace mss::pipeline::detail
