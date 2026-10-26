#include "mss/odom/wheel_integrator.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"

// Derivation: docs/derivations/wheel_odom.md (sections 1-3).
namespace mss::odom {
namespace {

// rad. Below this |yaw increment| the arc coefficients use their Taylor series
// (truncation < 1e-18 relative), so a = 0 and tiny yaw rates never divide by zero.
constexpr double kSeriesAngleRad = 1e-4;

// SE(3) tangent indices [rho_x, rho_y, phi_z] of the planar motion of o.
constexpr std::array<int, 3> kPlanarIdx = {0, 1, 5};

// Arc of one sample: forward distance s (m) and yaw increment a (rad).
// sin_c = sin(a)/a, cos_c = (1 - cos a)/a; t = s * [sin_c, cos_c] (derivation section 1).
// d_sin_c, d_cos_c are their derivatives w.r.t. a, so dt/da = s * [d_sin_c, d_cos_c]
// (derivation section 3).
struct ArcCoeffs {
  double sin_c = 1.0;
  double cos_c = 0.0;
  double d_sin_c = 0.0;
  double d_cos_c = 0.5;
};

ArcCoeffs arc_coeffs(double a) {
  ArcCoeffs c;
  if (std::abs(a) < kSeriesAngleRad) {
    const double a2 = a * a;
    c.sin_c = 1.0 - a2 / 6.0;
    c.cos_c = a * (0.5 - a2 / 24.0);
    c.d_sin_c = a * (-1.0 / 3.0 + a2 / 30.0);
    c.d_cos_c = 0.5 - a2 / 8.0;
  } else {
    const double sa = std::sin(a);
    const double ca = std::cos(a);
    const double half = std::sin(0.5 * a);
    const double one_minus_cos = 2.0 * half * half;  // 1 - cos a without the cancellation
    const double a2 = a * a;
    c.sin_c = sa / a;
    c.cos_c = one_minus_cos / a;
    c.d_sin_c = (a * ca - sa) / a2;
    c.d_cos_c = (a * sa - one_minus_cos) / a2;
  }
  return c;
}

common::Mat3 rot_z(double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  common::Mat3 rot;
  rot << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return rot;
}

}  // namespace

// NOLINTNEXTLINE(modernize-pass-by-value): signature is frozen in wheel_integrator.hpp.
WheelIntegrator::WheelIntegrator(const WheelOdomConfig& cfg, const common::Pose& T_b_o)
    : cfg_(cfg), T_b_o_(T_b_o) {
  // !(x >= 0) also rejects NaN.
  if (!(cfg.speed_sigma_mps >= 0.0)) {
    throw std::invalid_argument("wheel_odom.speed_sigma_mps must be >= 0 (m/s)");
  }
  if (!(cfg.yaw_rate_sigma_radps >= 0.0)) {
    throw std::invalid_argument("wheel_odom.yaw_rate_sigma_radps must be >= 0 (rad/s)");
  }
}

common::Status WheelIntegrator::integrate(const common::WheelSample& s) {
  if (!std::isfinite(s.v_mps) || !std::isfinite(s.omega_radps)) {
    return common::Status::error("non-finite wheel sample");
  }
  if (!has_sample_) {
    has_sample_ = true;
    t_start_ = s.t;
    t_last_ = s.t;
    return common::Status::ok();
  }
  if (s.t <= t_last_) {
    return common::Status::error("wheel timestamp not increasing");
  }

  const double dt_s = common::seconds_between(t_last_, s.t);
  const double ds_m = s.v_mps * dt_s;
  const double da_rad = s.omega_radps * dt_s;
  const ArcCoeffs c = arc_coeffs(da_rad);
  const double tx_m = ds_m * c.sin_c;  // increment translation in o_{k-1}
  const double ty_m = ds_m * c.cos_c;

  // Covariance of the SE(2) right tangent [rho_x, rho_y, phi_z] (derivation section 3):
  // P_k = A_k P_{k-1} A_k^T + B_k Q_k B_k^T, A_k = Ad_SE2(Delta_k^-1),
  // B_k = d(right tangent of Delta_k)/d(s, a), Q_k = diag(sigma_v^2, sigma_omega^2) dt^2.
  const double ca = std::cos(da_rad);
  const double sa = std::sin(da_rad);
  Eigen::Matrix2d rot_inv;  // R(a)^T
  rot_inv << ca, sa, -sa, ca;
  const Eigen::Vector2d t_inv = -(rot_inv * Eigen::Vector2d(tx_m, ty_m));  // Delta^-1 transl.
  Eigen::Matrix3d adj_inv = Eigen::Matrix3d::Identity();
  adj_inv.topLeftCorner<2, 2>() = rot_inv;
  adj_inv(0, 2) = t_inv.y();
  adj_inv(1, 2) = -t_inv.x();

  Eigen::Matrix2d dt_dn;  // columns dt/ds, dt/da
  dt_dn << c.sin_c, ds_m * c.d_sin_c, c.cos_c, ds_m * c.d_cos_c;
  Eigen::Matrix<double, 3, 2> b_k = Eigen::Matrix<double, 3, 2>::Zero();
  b_k.topRows<2>() = rot_inv * dt_dn;
  b_k(2, 1) = 1.0;

  const double dt2 = dt_s * dt_s;
  Eigen::Matrix2d q_k = Eigen::Matrix2d::Zero();
  q_k(0, 0) = cfg_.speed_sigma_mps * cfg_.speed_sigma_mps * dt2;
  q_k(1, 1) = cfg_.yaw_rate_sigma_radps * cfg_.yaw_rate_sigma_radps * dt2;

  const Eigen::Matrix3d cov_new =
      adj_inv * cov_o_ * adj_inv.transpose() + b_k * q_k * b_k.transpose();
  cov_o_ = 0.5 * (cov_new + cov_new.transpose());

  // Mean: T_o0_ok = T_o0_ok-1 * Delta_k.
  const double cy = std::cos(yaw_rad_);
  const double sy = std::sin(yaw_rad_);
  x_m_ += cy * tx_m - sy * ty_m;
  y_m_ += sy * tx_m + cy * ty_m;
  yaw_rad_ += da_rad;
  t_last_ = s.t;
  return common::Status::ok();
}

common::OdomDelta WheelIntegrator::take_delta() {
  common::OdomDelta d;
  d.t0 = t_start_;
  d.t1 = t_last_;
  if (t_last_ != t_start_) {  // something was integrated (timestamps strictly increase)
    const common::Pose T_o0_o1{rot_z(yaw_rad_), common::Vec3(x_m_, y_m_, 0.0)};
    d.T_b0_b1 = T_b_o_ * T_o0_o1 * T_b_o_.inverse();
    // Embed [rho_x, rho_y, phi_z] at SE(3) tangent indices 0, 1, 5 (z, roll, pitch get
    // zero variance: singular by design), then move from o to b (derivation section 3).
    common::Mat6 cov_o = common::Mat6::Zero();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        cov_o(kPlanarIdx[i], kPlanarIdx[j]) = cov_o_(i, j);
      }
    }
    d.cov = common::transform_covariance(cov_o, T_b_o_);
  }
  x_m_ = 0.0;
  y_m_ = 0.0;
  yaw_rad_ = 0.0;
  cov_o_.setZero();
  t_start_ = t_last_;
  return d;
}

}  // namespace mss::odom
