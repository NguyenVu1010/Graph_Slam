// imu component test with a known answer: static initialisation, then preintegration of a
// tilted body moving on a horizontal circle at constant speed and yaw rate.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/result.hpp"
#include "mss/common/span.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/initializer.hpp"
#include "mss/imu/preintegrator.hpp"

namespace mss::imu {
namespace {

using common::ImuSample;
using common::Mat3;
using common::NavState;
using common::Timestamp;
using common::Vec3;

// config/default.yaml imu.* values.
ImuConfig default_config() {
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

TEST(ImuComponent, TiltedBodyOnCircleKnownAnswer) {
  constexpr double kGravityMps2 = 9.80665;
  constexpr double kYawRateRadps = 0.5;                   // W
  constexpr double kSpeedMps = 1.0;                       // V
  constexpr double kRadiusM = kSpeedMps / kYawRateRadps;  // r = 2 m
  constexpr std::int64_t kPeriodNs = 5'000'000;           // 200 Hz
  constexpr std::int64_t kT0Ns = 1'000'000'000;

  const ImuConfig cfg = default_config();
  const Mat3 R_tilt =
      common::so3_exp(Vec3(0.0, 0.05, 0.0)) * common::so3_exp(Vec3(-0.08, 0.0, 0.0));
  const Vec3 bg_true(0.002, -0.001, 0.003);

  // 1) Static window: 401 samples, 2.0 s.
  std::vector<ImuSample> still(401);
  for (std::size_t k = 0; k < still.size(); ++k) {
    still[k].t = Timestamp{kT0Ns + static_cast<std::int64_t>(k) * kPeriodNs};
    still[k].acc_b = R_tilt.transpose() * Vec3(0.0, 0.0, kGravityMps2);
    still[k].gyro_b = bg_true;
  }
  ImuInitializer initializer(cfg);
  const common::Result<InitState> init_r =
      initializer.try_init(common::Span<const ImuSample>(still));
  ASSERT_TRUE(init_r.is_ok()) << init_r.reason();
  const InitState& init = init_r.value();
  EXPECT_LT((init.R_w_b - R_tilt).norm(), 1e-9);
  EXPECT_LT((init.bias.bg - bg_true).norm(), 1e-12);

  // 2) Motion over 1 s, tau = k * 5 ms since the motion start.
  auto Rz = [](double yaw) { return common::so3_exp(Vec3(0.0, 0.0, yaw)); };
  auto R_w_b = [&](double tau) -> Mat3 { return Rz(kYawRateRadps * tau) * R_tilt; };
  auto p_w = [&](double tau) -> Vec3 {
    const double th = kYawRateRadps * tau;
    return Vec3(kRadiusM * std::sin(th), kRadiusM * (1.0 - std::cos(th)), 0.0);
  };
  auto v_w = [&](double tau) -> Vec3 {
    const double th = kYawRateRadps * tau;
    return Vec3(kSpeedMps * std::cos(th), kSpeedMps * std::sin(th), 0.0);
  };
  auto a_w = [&](double tau) -> Vec3 {
    const double th = kYawRateRadps * tau;
    return Vec3(
        -kSpeedMps * kYawRateRadps * std::sin(th), kSpeedMps * kYawRateRadps * std::cos(th), 0.0);
  };
  const Vec3 gyro_b = R_tilt.transpose() * Vec3(0.0, 0.0, kYawRateRadps) + bg_true;
  const std::int64_t t_motion_ns = still.back().t.ns;
  std::vector<ImuSample> motion(201);
  for (std::size_t k = 0; k < motion.size(); ++k) {
    const std::int64_t dt_ns = static_cast<std::int64_t>(k) * kPeriodNs;
    const double tau = static_cast<double>(dt_ns) * 1e-9;  // s
    motion[k].t = Timestamp{t_motion_ns + dt_ns};
    motion[k].gyro_b = gyro_b;
    motion[k].acc_b = R_w_b(tau).transpose() * (a_w(tau) - init.g_w);
  }

  Preintegrator pre(cfg);
  pre.reset(init.bias);
  for (const ImuSample& s : motion) {
    ASSERT_TRUE(pre.integrate(s).is_ok());
  }
  NavState x0;
  x0.t = motion.front().t;
  x0.T_w_b.R = init.R_w_b;
  x0.T_w_b.p = Vec3::Zero();
  x0.v_w_b = Vec3(kSpeedMps, 0.0, 0.0);
  x0.bias = init.bias;
  const NavState x1 = pre.predict(x0, init.g_w);

  const Mat3 R_err = x1.T_w_b.R.transpose() * R_w_b(1.0);
  EXPECT_LT(common::so3_log(R_err).norm(), 1e-9);  // constant body rate: Euler is exact
  // Euler bound 0.5 * W * |a_h| * dt * T = 6.25e-4, rounded up.
  EXPECT_LT((x1.T_w_b.p - p_w(1.0)).norm(), 1e-3);
  EXPECT_LT((x1.v_w_b - v_w(1.0)).norm(), 1e-3);
  EXPECT_EQ(x1.t.ns - x0.t.ns, 1'000'000'000);
}

}  // namespace
}  // namespace mss::imu
