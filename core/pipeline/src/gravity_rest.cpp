#include "gravity_rest.hpp"

#include <cmath>

namespace mss::pipeline::detail {

const char* gravity_rest_failure(const imu::InitState& s,
                                 const imu::ImuConfig& imu,
                                 double prior_bias_acc_sigma_mps2,
                                 bool wheel_moved) {
  if (!(s.bias.bg.norm() <= imu.init_max_mean_gyro_radps)) {
    return "mean gyro above imu.init_max_mean_gyro_radps (turning)";
  }
  if (wheel_moved) {
    return "wheel speed above imu.init_max_wheel_speed_mps in the window";
  }
  const double f_norm = s.acc_mean_b.norm();  // m/s^2
  if (!(f_norm > 0.0)) {
    return "zero specific force";
  }
  const common::Vec3 u = s.acc_mean_b / f_norm;
  const double var_mean = u.dot(s.acc_mean_cov * u);  // (m/s^2)^2
  const double sigma = std::sqrt(var_mean + prior_bias_acc_sigma_mps2 * prior_bias_acc_sigma_mps2);
  if (!(std::abs(f_norm - imu.gravity_mps2) <= imu.init_gravity_check_sigma * sigma)) {
    return "specific-force magnitude off gravity by more than imu.init_gravity_check_sigma";
  }
  return nullptr;
}

}  // namespace mss::pipeline::detail
