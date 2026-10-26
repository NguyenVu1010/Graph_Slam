#pragma once

namespace mss::imu {

// YAML: imu.*  Loaded and validated by mss::pipeline::load_config.
struct ImuConfig {
  double rate_hz = 0.0;                  // Hz, nominal sample rate (> 0)
  double acc_noise_density = 0.0;        // m/s^2/sqrt(Hz)
  double gyro_noise_density = 0.0;       // rad/s/sqrt(Hz)
  double acc_random_walk = 0.0;          // m/s^3/sqrt(Hz)
  double gyro_random_walk = 0.0;         // rad/s^2/sqrt(Hz)
  double gravity_mps2 = 0.0;             // m/s^2, gravity magnitude
  double init_window_s = 0.0;            // s, static window for initialisation
  double init_max_acc_std_mps2 = 0.0;    // m/s^2, max accel std to accept the window as static
  double init_max_gyro_std_radps = 0.0;  // rad/s, max gyro std to accept the window as static
};

}  // namespace mss::imu
