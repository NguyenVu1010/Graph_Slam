// Graph API checks that hold in both builds (with and without GTSAM).
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "mss/backend/config.hpp"
#include "mss/backend/graph.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"

#include "backend_test_config.hpp"

namespace mss::backend {
namespace {

using common::Vec3;
using test::default_backend_config;
using test::default_imu_config;

TEST(BackendApi, ConstructsWithDefaults) {
  EXPECT_NO_THROW(Graph(default_backend_config(), default_imu_config(), Vec3(0.3, 0.0, 0.8)));
}

TEST(BackendApi, ConstructorRejectsInvalidConfig) {
  const Vec3 p_b_g = Vec3::Zero();
  const auto expect_throw = [&](BackendConfig cfg, imu::ImuConfig imu_cfg, const Vec3& p) {
    EXPECT_THROW(Graph(cfg, imu_cfg, p), std::invalid_argument);
  };
  {
    imu::ImuConfig c = default_imu_config();
    c.rate_hz = 0.0;
    expect_throw(default_backend_config(), c, p_b_g);
  }
  {
    imu::ImuConfig c = default_imu_config();
    c.gravity_mps2 = 0.0;
    expect_throw(default_backend_config(), c, p_b_g);
  }
  BackendConfig b = default_backend_config();
  b.keyframe_dist_m = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.keyframe_rot_rad = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.relinearize_threshold = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.relinearize_skip = 0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.prior_pos_sigma_m = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.prior_rot_sigma_rad = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.prior_vel_sigma_mps = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.prior_bias_acc_sigma_mps2 = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.prior_bias_gyro_sigma_radps = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.static_gravity_factor = true;  // validated only when enabled (M-1)
  b.static_gravity_sigma_scale = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  b = default_backend_config();
  b.gnss_frame_variable = true;  // ADR-019: its prior sigmas are then required (> 0)
  b.gnss_frame_tilt_sigma_rad = 1e-4;
  b.gnss_frame_yaw_sigma_rad = 0.5;
  expect_throw(b, default_imu_config(), p_b_g);  // gnss_frame_pos_sigma_m still 0
  b.gnss_frame_pos_sigma_m = 10.0;
  EXPECT_NO_THROW(Graph(b, default_imu_config(), p_b_g));
  b.gnss_frame_yaw_sigma_rad = 0.0;
  expect_throw(b, default_imu_config(), p_b_g);
  {
    imu::ImuConfig c = default_imu_config();
    c.acc_random_walk = -1e-4;
    expect_throw(default_backend_config(), c, p_b_g);
  }
  {
    imu::ImuConfig c = default_imu_config();
    c.gyro_random_walk = -1e-5;
    expect_throw(default_backend_config(), c, p_b_g);
  }
  expect_throw(default_backend_config(),
               default_imu_config(),
               Vec3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
}

// Controller ruling (plan contract issue 2): a zero random walk cannot be a sigma.
TEST(BackendApi, ConstructorRejectsZeroRandomWalk) {
  imu::ImuConfig c = default_imu_config();
  c.acc_random_walk = 0.0;
  EXPECT_THROW(Graph(default_backend_config(), c, Vec3::Zero()), std::invalid_argument);
  c = default_imu_config();
  c.gyro_random_walk = 0.0;
  EXPECT_THROW(Graph(default_backend_config(), c, Vec3::Zero()), std::invalid_argument);
}

TEST(BackendApi, KeyIdsAreDense) {
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  common::NavState x0;
  for (common::KeyId k = 0; k < 3; ++k) {
    x0.t = common::Timestamp{static_cast<std::int64_t>(k + 1) * 1'000'000'000};
    EXPECT_EQ(g.add_keyframe(x0.t, x0), k);
  }
}

TEST(BackendApi, MarginalCovUnknownKeyIsNan) {
  const Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  const common::PoseCovariance cov = g.marginal_cov(7);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      EXPECT_TRUE(std::isnan(cov(r, c)));
    }
  }
  EXPECT_FALSE(g.keyframe_state(7).is_ok());
}

TEST(BackendApi, IsMoveOnly) {
  static_assert(!std::is_copy_constructible_v<Graph>);
  static_assert(!std::is_copy_assignable_v<Graph>);
  static_assert(std::is_nothrow_move_constructible_v<Graph>);
  static_assert(std::is_nothrow_move_assignable_v<Graph>);
  Graph a(default_backend_config(), default_imu_config(), Vec3::Zero());
  EXPECT_EQ(a.add_keyframe(common::Timestamp{1}, common::NavState{}), 0U);
  Graph b = std::move(a);
  EXPECT_EQ(b.add_keyframe(common::Timestamp{2}, common::NavState{}), 1U);
}

}  // namespace
}  // namespace mss::backend
