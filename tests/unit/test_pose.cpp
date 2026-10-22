#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"

namespace mss::common {

// T_w_l = T_w_b * T_b_l must move a LiDAR point the same way as the two steps.
TEST(Pose, CompositionCancelsFrames) {
  Pose T_w_b;
  T_w_b.R = Eigen::AngleAxisd(0.3, Vec3::UnitZ()).toRotationMatrix();
  T_w_b.p = Vec3(1.0, 2.0, 3.0);
  Pose T_b_l;
  T_b_l.R = Eigen::AngleAxisd(-0.1, Vec3::UnitX()).toRotationMatrix();
  T_b_l.p = Vec3(0.1, 0.0, 0.2);

  const Vec3 p_l(0.5, -0.4, 1.2);
  const Vec3 p_w_two_steps = T_w_b * (T_b_l * p_l);
  const Pose T_w_l = T_w_b * T_b_l;
  const Vec3 p_w_composed = T_w_l * p_l;
  EXPECT_TRUE(p_w_two_steps.isApprox(p_w_composed, 1e-12));
}

TEST(Pose, InverseRoundTrips) {
  Pose T;
  T.R = Eigen::AngleAxisd(1.1, Vec3(1.0, 1.0, 0.0).normalized()).toRotationMatrix();
  T.p = Vec3(-2.0, 0.5, 4.0);
  const Pose I = T * T.inverse();
  EXPECT_TRUE(I.R.isApprox(Mat3::Identity(), 1e-12));
  EXPECT_TRUE(I.p.isZero(1e-12));
}

}  // namespace mss::common
