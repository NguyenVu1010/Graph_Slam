#pragma once

// Shared helpers for the imu tests (unit, covariance and the GTSAM cross-check).

#include <cstdint>
#include <random>
#include <vector>

#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

namespace mss::imu::test {

// The config/default.yaml imu.* values (tests do not parse YAML; ADR-008).
inline ImuConfig default_imu_config() {
  ImuConfig cfg;
  cfg.rate_hz = 200.0;
  cfg.acc_noise_density = 5.8e-3;
  cfg.gyro_noise_density = 3.5e-4;
  cfg.acc_random_walk = 2.0e-4;
  cfg.gyro_random_walk = 2.0e-5;
  cfg.gravity_mps2 = 9.80665;
  cfg.init_window_s = 2.0;
  cfg.init_max_acc_std_mps2 = 0.05;
  cfg.init_max_gyro_std_radps = 0.01;
  return cfg;
}

// Random 200 Hz IMU stream with +/-0.5 ms jitter: t_0 = 1 s, then each step adds
// 5 ms + U{-0.5 ms, 0.5 ms} (integer ns). gyro_b ~ N(0, 0.5) rad/s per axis (or
// N(0, 0.5) * n_hat with n_hat = [1, 2, 3].normalized() when `single_axis_gyro`);
// acc_b ~ N(0, 2.0) m/s^2 per axis plus [0, 0, 9.80665]. One mt19937(seed), draw order
// t, gyro, acc per sample.
inline std::vector<common::ImuSample> random_imu_sequence(std::uint32_t seed,
                                                          int n,
                                                          bool single_axis_gyro = false) {
  constexpr std::int64_t kT0Ns = 1'000'000'000;
  constexpr std::int64_t kStepNs = 5'000'000;
  constexpr std::int64_t kJitterNs = 500'000;
  constexpr double kGyroStdRadps = 0.5;
  constexpr double kAccStdMps2 = 2.0;
  constexpr double kGravityMps2 = 9.80665;

  std::mt19937 rng(seed);
  std::uniform_int_distribution<std::int64_t> jitter(-kJitterNs, kJitterNs);
  std::normal_distribution<double> gyro(0.0, kGyroStdRadps);
  std::normal_distribution<double> acc(0.0, kAccStdMps2);
  const common::Vec3 n_hat = common::Vec3(1.0, 2.0, 3.0).normalized();

  std::vector<common::ImuSample> seq(static_cast<std::size_t>(n));
  std::int64_t t_ns = kT0Ns;
  for (int k = 0; k < n; ++k) {
    if (k > 0) {
      t_ns += kStepNs + jitter(rng);
    }
    common::ImuSample& s = seq[static_cast<std::size_t>(k)];
    s.t = common::Timestamp{t_ns};
    if (single_axis_gyro) {
      s.gyro_b = gyro(rng) * n_hat;
    } else {
      const double gx = gyro(rng);
      const double gy = gyro(rng);
      const double gz = gyro(rng);
      s.gyro_b = common::Vec3(gx, gy, gz);
    }
    const double ax = acc(rng);
    const double ay = acc(rng);
    const double az = acc(rng);
    s.acc_b = common::Vec3(ax, ay, az + kGravityMps2);
  }
  return seq;
}

}  // namespace mss::imu::test
