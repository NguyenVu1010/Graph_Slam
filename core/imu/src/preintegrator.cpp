#include "mss/imu/preintegrator.hpp"

#include <stdexcept>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"

// Formulas and their derivation: docs/derivations/imu_preintegration.md.
namespace mss::imu {

using common::ImuBias;
using common::ImuSample;
using common::Mat3;
using common::NavState;
using common::PreintegratedDelta;
using common::Status;
using common::Vec3;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat93 = Eigen::Matrix<double, 9, 3>;

Preintegrator::Preintegrator(const ImuConfig& cfg) {
  // `!(x >= 0)` also rejects NaN.
  if (!(cfg.acc_noise_density >= 0.0)) {
    throw std::invalid_argument("imu.acc_noise_density must be >= 0");
  }
  if (!(cfg.gyro_noise_density >= 0.0)) {
    throw std::invalid_argument("imu.gyro_noise_density must be >= 0");
  }
  if (!(cfg.rate_hz > 0.0)) {
    throw std::invalid_argument("imu.rate_hz must be > 0");
  }
  acc_var_ = cfg.acc_noise_density * cfg.acc_noise_density;
  gyro_var_ = cfg.gyro_noise_density * cfg.gyro_noise_density;
  reset(ImuBias{});
}

void Preintegrator::reset(const ImuBias& bias) {
  delta_ = PreintegratedDelta{};
  delta_.bias_lin = bias;
  t_first_ = common::Timestamp{};
  t_prev_ = common::Timestamp{};
  has_first_ = false;
}

Status Preintegrator::integrate(const ImuSample& s) {
  // Validate before touching any member, so a rejected sample leaves the delta unchanged.
  if (!s.acc_b.allFinite() || !s.gyro_b.allFinite()) {
    return Status::error("non-finite imu sample");
  }
  if (has_first_ && s.t <= t_prev_) {
    return Status::error("imu timestamp not increasing");
  }
  if (!has_first_) {
    // §2: the first sample only sets the start time.
    has_first_ = true;
    t_first_ = s.t;
    t_prev_ = s.t;
    return Status::ok();
  }

  const double dt = common::seconds_between(t_prev_, s.t);  // s
  const double dt2_half = 0.5 * dt * dt;                    // s^2
  const Vec3 a = s.acc_b - delta_.bias_lin.ba;              // m/s^2, bias-corrected
  const Vec3 w = s.gyro_b - delta_.bias_lin.bg;             // rad/s, bias-corrected
  const Vec3 w_dt = w * dt;                                 // rad
  const Mat3 R_bkm1_bk = common::so3_exp(w_dt);
  const Mat3 J_r_k = common::so3_jr(w_dt);
  const Mat3 R_bi_bkm1 = delta_.dR;  // old rotation dR before this sample
  const Mat3 a_hat = common::skew(a);

  // Bias Jacobians (§3). Residuals: dR (right perturbation, dR(b + db) ~ dR Exp(J_dR_bg dbg)),
  // dv and dp (additive), all in b_i. Variables: ba (m/s^2), bg (rad/s). Each update uses
  // the OLD R and the OLD J_dR_bg / J_dv_*, hence the order p, v, R.
  const Mat3 r_ahat_j = R_bi_bkm1 * a_hat * delta_.J_dR_bg;  // d(R a)/dbg = -R a^ J_dR_bg
  delta_.J_dp_ba += delta_.J_dv_ba * dt - R_bi_bkm1 * dt2_half;
  delta_.J_dp_bg += delta_.J_dv_bg * dt - r_ahat_j * dt2_half;
  delta_.J_dv_ba += -R_bi_bkm1 * dt;
  delta_.J_dv_bg += -r_ahat_j * dt;
  delta_.J_dR_bg = R_bkm1_bk.transpose() * delta_.J_dR_bg - J_r_k * dt;

  // Covariance (§4), chart [phi, p, v] = GTSAM NavState local coordinates:
  //   Sigma <- A Sigma A^T + B (sa^2 / dt) B^T + C (sg^2 / dt) C^T.
  // A = f_state, B = g_acc, C = g_gyro depend only on this step (R_bkm1_bk = dR_k, a, w,
  // dt), not on the old rotation R_bi_bkm1.
  const Mat3 R_bk_bkm1 = R_bkm1_bk.transpose();
  const Mat3 rt_ahat_k = R_bk_bkm1 * a_hat;
  Mat9 f_state = Mat9::Zero();
  f_state.block<3, 3>(0, 0) = R_bk_bkm1;
  f_state.block<3, 3>(3, 0) = -rt_ahat_k * dt2_half;
  f_state.block<3, 3>(3, 3) = R_bk_bkm1;
  f_state.block<3, 3>(3, 6) = R_bk_bkm1 * dt;
  f_state.block<3, 3>(6, 0) = -rt_ahat_k * dt;
  f_state.block<3, 3>(6, 6) = R_bk_bkm1;
  Mat93 g_acc = Mat93::Zero();  // acts on accel noise
  g_acc.block<3, 3>(3, 0) = R_bk_bkm1 * dt2_half;
  g_acc.block<3, 3>(6, 0) = R_bk_bkm1 * dt;
  Mat93 g_gyro = Mat93::Zero();  // acts on gyro noise
  g_gyro.block<3, 3>(0, 0) = J_r_k * dt;
  const Mat9 cov_new = f_state * delta_.cov * f_state.transpose() +
                       g_acc * (acc_var_ / dt) * g_acc.transpose() +
                       g_gyro * (gyro_var_ / dt) * g_gyro.transpose();
  delta_.cov = cov_new;

  // Mean (§2): p and v use the old rotation.
  const Vec3 acc_bi = R_bi_bkm1 * a;
  delta_.dp += delta_.dv * dt + acc_bi * dt2_half;
  delta_.dv += acc_bi * dt;
  delta_.dR = R_bi_bkm1 * R_bkm1_bk;

  t_prev_ = s.t;
  delta_.dt_s = common::seconds_between(t_first_, t_prev_);
  return Status::ok();
}

const PreintegratedDelta& Preintegrator::delta() const {
  return delta_;
}

NavState Preintegrator::predict(const NavState& x0, const Vec3& g_w) const {
  if (!has_first_ || t_prev_ == t_first_) {
    return x0;  // nothing integrated (dt_s == 0)
  }
  // First-order bias correction (§5), db = x0.bias - bias_lin.
  const Vec3 dbg = x0.bias.bg - delta_.bias_lin.bg;
  const Vec3 dba = x0.bias.ba - delta_.bias_lin.ba;
  const Vec3 phi_corr = delta_.J_dR_bg * dbg;
  const Mat3 R_corr = delta_.dR * common::so3_exp(phi_corr);
  const Vec3 dv_corr = delta_.dv + delta_.J_dv_ba * dba + delta_.J_dv_bg * dbg;
  const Vec3 dp_corr = delta_.dp + delta_.J_dp_ba * dba + delta_.J_dp_bg * dbg;

  const double dt = delta_.dt_s;  // s
  const Mat3& R_w_b0 = x0.T_w_b.R;
  NavState x1;
  x1.t = common::Timestamp{x0.t.ns + (t_prev_.ns - t_first_.ns)};
  x1.T_w_b.R = R_w_b0 * R_corr;
  x1.T_w_b.p = x0.T_w_b.p + x0.v_w_b * dt + g_w * (0.5 * dt * dt) + R_w_b0 * dp_corr;
  x1.v_w_b = x0.v_w_b + g_w * dt + R_w_b0 * dv_corr;
  x1.bias = x0.bias;
  return x1;
}

}  // namespace mss::imu
