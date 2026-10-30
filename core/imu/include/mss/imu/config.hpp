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
  // Rest gate of the keyframe-0 gravity factor (pipeline, ADR-020), all > 0:
  double init_max_mean_gyro_radps = 0.0;  // rad/s, max |mean gyro| of the window (bias + turn)
  double init_max_wheel_speed_mps = 0.0;  // m/s, max |wheel speed| in the window (wheel on)
  double init_gravity_check_sigma = 0.0;  // max ||f| - g| in sigmas (mean noise + acc bias prior)
  // Level start (ADR-027): the body rests on a level floor at initialisation, so roll = pitch = 0
  // and the whole window mean minus gravity is accelerometer bias. Off: roll and pitch from the
  // window mean, bias 0 (previous behaviour).
  bool init_level_start = false;
  // m/s^2, > 0 (checked when init_level_start): largest horizontal norm of the window mean taken
  // as bias. Above it the body is taken as tilted (ramp, mount) and the flag-off initialisation
  // is used instead (InitState::level_start_fallback).
  double init_level_max_bias_mps2 = 0.0;
};

}  // namespace mss::imu
