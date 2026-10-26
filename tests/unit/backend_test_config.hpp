#pragma once

// Shared helpers for the backend tests (both builds; the GTSAM tests include it too).

#include <string>
#include <string_view>
#include <vector>

#include "mss/backend/config.hpp"
#include "mss/common/logger.hpp"

#include "imu_test_utils.hpp"

namespace mss::backend::test {

// The config/default.yaml backend.* values, angles converted to radians (tests do not
// parse YAML; ADR-008).
inline BackendConfig default_backend_config() {
  BackendConfig cfg;
  cfg.keyframe_dist_m = 1.0;
  cfg.keyframe_rot_rad = 0.17453292519943295;  // 10 deg
  cfg.relinearize_threshold = 0.1;
  cfg.relinearize_skip = 1;
  cfg.prior_pos_sigma_m = 0.1;
  cfg.prior_rot_sigma_rad = 0.017453292519943295;  // 1 deg
  cfg.prior_vel_sigma_mps = 0.1;
  cfg.prior_bias_acc_sigma_mps2 = 0.1;
  cfg.prior_bias_gyro_sigma_radps = 0.01;
  return cfg;
}

using imu::test::default_imu_config;

// Records log lines while alive (installs itself as the process logger, restores the
// default sink on destruction).
class CapturingLogger final : public common::Logger {
 public:
  CapturingLogger() { common::set_logger(this); }
  ~CapturingLogger() override { common::set_logger(nullptr); }
  CapturingLogger(const CapturingLogger&) = delete;
  CapturingLogger& operator=(const CapturingLogger&) = delete;

  void log(common::LogLevel level, std::string_view msg) override {
    if (level == common::LogLevel::kWarn) {
      warnings.emplace_back(msg);
    } else if (level == common::LogLevel::kDebug) {
      debugs.emplace_back(msg);
    }
  }

  std::vector<std::string> warnings;
  std::vector<std::string> debugs;
};

}  // namespace mss::backend::test
