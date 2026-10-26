#include "gnss_fusion.hpp"

#include <Eigen/Core>

namespace mss::pipeline::detail {

GnssFusion::GnssFusion(const gnss::GnssConfig& cfg, const common::Vec3& p_b_g)
    : max_cov_m2_(cfg.max_cov_m2), chi2_gate_(cfg.chi2_gate), p_b_g_(p_b_g), aligner_(cfg) {}

common::Status GnssFusion::on_fix(const common::GnssMeasurement& m,
                                  const common::Pose& T_w_b_fix,
                                  const common::PoseCovariance& P_w_b) {
  common::Status gate = gnss::gate_fix(m, max_cov_m2_);
  if (!gate.is_ok()) {
    return gate;
  }
  if (!T_w_b_fix.R.allFinite() || !T_w_b_fix.p.allFinite()) {
    return common::Status::error("non-finite pose");
  }
  if (aligned_) {
    // G-02: S = m.cov + J P J^T, P := 0 when not finite (pipeline.md section 5).
    Eigen::Matrix<double, 3, 6> J_r_xi;
    const common::Vec3 r = gnss::gnss_residual(m.p_e_g, T_e_w_, T_w_b_fix, p_b_g_, &J_r_xi);
    common::Mat3 S = m.cov;
    if (P_w_b.allFinite()) {
      S += J_r_xi * P_w_b * J_r_xi.transpose();
    }
    common::Status chi2 = gnss::chi_square_gate(r, S, chi2_gate_);
    if (!chi2.is_ok()) {
      return chi2;
    }
  }
  const common::Result<common::Pose> T = aligner_.update(m.p_e_g, T_w_b_fix, p_b_g_);
  if (T.is_ok()) {
    T_e_w_ = T.value();
    aligned_ = true;
  }
  if (!aligned_) {
    return common::Status::error("gnss not aligned yet");
  }
  if (!pending_.push(PendingFix{m, T_w_b_fix})) {
    ++n_pending_dropped_;  // the oldest pending fix was overwritten
  }
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
    const common::Vec3 shift_w = p_w_g_k - p_w_g_fix;
    out[n] = f.m;
    out[n].p_e_g = f.m.p_e_g + T_e_w_.R * shift_w;
    ++n;
  }
  return n;
}

}  // namespace mss::pipeline::detail
