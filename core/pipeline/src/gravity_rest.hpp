#pragma once

#include "mss/imu/config.hpp"
#include "mss/imu/initializer.hpp"

namespace mss::pipeline::detail {

/// Rest gate of the keyframe-0 gravity factor (ADR-020, imu_only_observability.md section 12).
/// The initializer accepts a window on per-axis std only, so a constant turn or a steady
/// acceleration passes; the factor needs the body at rest. Returns nullptr when the window
/// passes, else the reason (a string literal):
/// - |mean gyro| (= s.bias.bg) > imu.init_max_mean_gyro_radps;
/// - `wheel_moved`: a wheel speed above imu.init_max_wheel_speed_mps inside the window
///   (the caller passes false when wheel odometry is off);
/// - ||f| - g| > imu.init_gravity_check_sigma * sqrt(u^T C u + prior_bias_acc_sigma^2),
///   f = s.acc_mean_b, u = f / |f|, C = s.acc_mean_cov, g = imu.gravity_mps2.
/// Units: rad/s, m/s, m/s^2. No allocation. Tests: PipelineInternals.GravityRestGate*.
const char* gravity_rest_failure(const imu::InitState& s,
                                 const imu::ImuConfig& imu,
                                 double prior_bias_acc_sigma_mps2,
                                 bool wheel_moved);

}  // namespace mss::pipeline::detail
