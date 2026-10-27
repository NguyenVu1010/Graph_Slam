// The no-GTSAM stub (MSS_WITH_GTSAM=OFF, core/backend/src/graph_stub.cpp). Built only then.
#include <cmath>

#include <gtest/gtest.h>

#include "mss/backend/graph.hpp"
#include "mss/common/types.hpp"

#include "backend_test_config.hpp"

namespace mss::backend {
namespace {

using common::Vec3;

TEST(BackendStub, OptimizeReturnsBuiltWithoutGtsam) {
  Graph g(test::default_backend_config(), test::default_imu_config(), Vec3::Zero());
  const common::Result<common::Estimate> empty = g.optimize();
  ASSERT_FALSE(empty.is_ok());
  EXPECT_EQ(empty.reason(), "built without GTSAM");

  common::NavState x0;
  x0.t = common::Timestamp{1'000'000'000};
  EXPECT_EQ(g.add_keyframe(x0.t, x0), 0U);
  x0.t = common::Timestamp{2'000'000'000};
  EXPECT_EQ(g.add_keyframe(x0.t, x0), 1U);

  common::PreintegratedDelta delta;
  delta.dt_s = 1.0;
  delta.cov = Eigen::Matrix<double, 9, 9>::Identity() * 1e-4;
  g.add_imu_factor(0, 1, delta);
  g.add_lidar_factor(0, 1, common::Pose{}, common::PoseCovariance::Identity() * 1e-4, nullptr);
  common::GnssMeasurement m;
  g.add_gnss_factor(1, m, common::Pose{});
  common::OdomDelta od;
  od.cov = common::PoseCovariance::Identity() * 1e-4;
  g.add_odom_factor(0, 1, od);
  common::LoopConstraint lc;
  lc.i = 0;
  lc.j = 1;
  lc.cov = common::PoseCovariance::Identity() * 1e-4;
  g.add_loop_factor(lc);

  const common::Result<common::Estimate> r = g.optimize();
  ASSERT_FALSE(r.is_ok());
  EXPECT_EQ(r.reason(), "built without GTSAM");
  const common::Result<common::NavState> s = g.keyframe_state(0);
  ASSERT_FALSE(s.is_ok());
  EXPECT_EQ(s.reason(), "built without GTSAM");
  const common::PoseCovariance cov = g.marginal_cov(0);
  EXPECT_TRUE(cov.array().isNaN().all());
  // PR 4d additions: the frame variable and the gravity factor are absent in the stub.
  g.add_static_gravity_factor(0, common::Vec3(0.0, 0.0, 9.8), common::Mat3::Identity());
  EXPECT_EQ(g.gnss_frame_estimate().reason(), "built without GTSAM");
  EXPECT_EQ(g.static_gravity_chi2().reason(), "built without GTSAM");
  EXPECT_FALSE(g.remove_static_gravity_factor());
}

}  // namespace
}  // namespace mss::backend
