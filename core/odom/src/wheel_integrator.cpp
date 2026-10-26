#include "mss/odom/wheel_integrator.hpp"

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

// Arc of one sample: forward distance s (m) and yaw increment a (rad).
// sin_c = sin(a)/a, cos_c = (1 - cos a)/a; t = s * [sin_c, cos_c] (derivation section 1).
struct ArcCoeffs {
  double sin_c = 1.0;
  double cos_c = 0.0;
};

ArcCoeffs arc_coeffs(double a) {
  ArcCoeffs c;
  if (std::abs(a) < kSeriesAngleRad) {
    const double a2 = a * a;
    c.sin_c = 1.0 - a2 / 6.0;
    c.cos_c = a * (0.5 - a2 / 24.0);
  } else {
    const double half = std::sin(0.5 * a);
    c.sin_c = std::sin(a) / a;
    c.cos_c = 2.0 * half * half / a;  // (1 - cos a)/a without the cancellation
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
  }
  x_m_ = 0.0;
  y_m_ = 0.0;
  yaw_rad_ = 0.0;
  cov_o_.setZero();
  t_start_ = t_last_;
  return d;
}

}  // namespace mss::odom
