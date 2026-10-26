#include "validate.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace mss::backend::detail {
namespace {

// `!(v > 0)` also rejects NaN.
void require_positive(double v, const char* name) {
  if (!(v > 0.0) || !std::isfinite(v)) {
    throw std::invalid_argument(std::string(name) + " must be > 0");
  }
}

}  // namespace

void validate(const BackendConfig& cfg, const imu::ImuConfig& imu_cfg, const common::Vec3& p_b_g) {
  require_positive(imu_cfg.rate_hz, "imu.rate_hz");
  require_positive(imu_cfg.gravity_mps2, "imu.gravity_mps2");
  require_positive(imu_cfg.acc_random_walk, "imu.acc_random_walk");
  require_positive(imu_cfg.gyro_random_walk, "imu.gyro_random_walk");
  require_positive(cfg.keyframe_dist_m, "backend.keyframe_dist_m");
  require_positive(cfg.keyframe_rot_rad, "backend.keyframe_rot_rad");
  require_positive(cfg.relinearize_threshold, "backend.relinearize_threshold");
  if (cfg.relinearize_skip <= 0) {
    throw std::invalid_argument("backend.relinearize_skip must be > 0");
  }
  require_positive(cfg.prior_pos_sigma_m, "backend.prior_pos_sigma_m");
  require_positive(cfg.prior_rot_sigma_rad, "backend.prior_rot_sigma_rad");
  require_positive(cfg.prior_vel_sigma_mps, "backend.prior_vel_sigma_mps");
  require_positive(cfg.prior_bias_acc_sigma_mps2, "backend.prior_bias_acc_sigma_mps2");
  require_positive(cfg.prior_bias_gyro_sigma_radps, "backend.prior_bias_gyro_sigma_radps");
  if (!p_b_g.allFinite()) {
    throw std::invalid_argument("frames.p_b_g must be finite");
  }
}

}  // namespace mss::backend::detail
