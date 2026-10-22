#pragma once

namespace mss::odom {

// YAML: wheel_odom.*  Loaded and validated by mss::pipeline::load_config.
struct WheelOdomConfig {
  bool enabled = false;          // feature flag
  double wheel_radius_m = 0.0;   // m
  double track_m = 0.0;          // m, wheel separation
  double speed_sigma_mps = 0.0;  // m/s, wheel speed sigma
};

}  // namespace mss::odom
