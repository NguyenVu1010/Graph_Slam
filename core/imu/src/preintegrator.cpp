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
  const Mat3 dR_k = common::so3_exp(w_dt);
  const Mat3 Jr = common::so3_jr(w_dt);
  const Mat3 R = delta_.dR;  // old rotation R_bi_bk-1
  const Mat3 a_hat = common::skew(a);

  // Bias Jacobians (§3). Residuals: dR (right perturbation, dR(b + db) ~ dR Exp(J_dR_bg dbg)),
  // dv and dp (additive), all in b_i. Variables: ba (m/s^2), bg (rad/s). Each update uses
  // the OLD R and the OLD J_dR_bg / J_dv_*, hence the order p, v, R.
  const Mat3 R_ahat_J = R * a_hat * delta_.J_dR_bg;  // d(R a)/dbg = -R a^ J_dR_bg
  delta_.J_dp_ba += delta_.J_dv_ba * dt - R * dt2_half;
  delta_.J_dp_bg += delta_.J_dv_bg * dt - R_ahat_J * dt2_half;
  delta_.J_dv_ba += -R * dt;
  delta_.J_dv_bg += -R_ahat_J * dt;
  delta_.J_dR_bg = dR_k.transpose() * delta_.J_dR_bg - Jr * dt;

  // Covariance (§4), chart [phi, p, v] = GTSAM NavState local coordinates:
  //   Sigma <- A Sigma A^T + B (sa^2 / dt) B^T + C (sg^2 / dt) C^T.
  // A, B, C depend only on this step (dR_k, a, w, dt), not on the old R.
  const Mat3 dR_kT = dR_k.transpose();
  const Mat3 dR_kT_ahat = dR_kT * a_hat;
  Mat9 A = Mat9::Zero();
  A.block<3, 3>(0, 0) = dR_kT;
  A.block<3, 3>(3, 0) = -dR_kT_ahat * dt2_half;
  A.block<3, 3>(3, 3) = dR_kT;
  A.block<3, 3>(3, 6) = dR_kT * dt;
  A.block<3, 3>(6, 0) = -dR_kT_ahat * dt;
  A.block<3, 3>(6, 6) = dR_kT;
  Mat93 B = Mat93::Zero();  // acts on accel noise
  B.block<3, 3>(3, 0) = dR_kT * dt2_half;
  B.block<3, 3>(6, 0) = dR_kT * dt;
  Mat93 C = Mat93::Zero();  // acts on gyro noise
  C.block<3, 3>(0, 0) = Jr * dt;
  const Mat9 cov_new = A * delta_.cov * A.transpose() + B * (acc_var_ / dt) * B.transpose() +
                       C * (gyro_var_ / dt) * C.transpose();
  delta_.cov = cov_new;

  // Mean (§2): p and v use the old rotation.
  const Vec3 Ra = R * a;
  delta_.dp += delta_.dv * dt + Ra * dt2_half;
  delta_.dv += Ra * dt;
  delta_.dR = R * dR_k;

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
  const Vec3 dR_corr_phi = delta_.J_dR_bg * dbg;
  const Mat3 dR_corr = delta_.dR * common::so3_exp(dR_corr_phi);
  const Vec3 dv_corr = delta_.dv + delta_.J_dv_ba * dba + delta_.J_dv_bg * dbg;
  const Vec3 dp_corr = delta_.dp + delta_.J_dp_ba * dba + delta_.J_dp_bg * dbg;

  const double dt = delta_.dt_s;  // s
  const Mat3& R0 = x0.T_w_b.R;
  NavState x1;
  x1.t = common::Timestamp{x0.t.ns + (t_prev_.ns - t_first_.ns)};
  x1.T_w_b.R = R0 * dR_corr;
  x1.T_w_b.p = x0.T_w_b.p + x0.v_w_b * dt + g_w * (0.5 * dt * dt) + R0 * dp_corr;
  x1.v_w_b = x0.v_w_b + g_w * dt + R0 * dv_corr;
  x1.bias = x0.bias;
  return x1;
}

}  // namespace mss::imu
