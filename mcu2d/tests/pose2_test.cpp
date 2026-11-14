#include "mss2d/pose2.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace mss2d {
namespace {

using P = Pose2T<double>;
constexpr double kPi = 3.14159265358979323846;

void expect_pose_near(const P& a, const P& b, double tol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(normalize_angle(a.yaw - b.yaw), 0.0, tol);
}

TEST(Pose2, NormalizeAngleWrapsToMinusPiPi) {
  EXPECT_NEAR(normalize_angle(3 * kPi / 2), -kPi / 2, 1e-12);
  EXPECT_NEAR(normalize_angle(-3 * kPi / 2), kPi / 2, 1e-12);
  EXPECT_NEAR(normalize_angle(7 * kPi + 0.1), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(normalize_angle(kPi), -kPi, 1e-12);
  EXPECT_NEAR(normalize_angle(0.3), 0.3, 1e-15);
}

TEST(Pose2, ComposeTransformAndInverse) {
  const P T_a_b(1.0, 2.0, kPi / 2);
  const P T_b_c(0.5, 0.0, kPi / 4);
  const P T_a_c = T_a_b * T_b_c;
  expect_pose_near(T_a_c, P(1.0, 2.5, 3 * kPi / 4), 1e-12);

  // Point composition cancels: T_a_c * p == T_a_b * (T_b_c * p).
  const Vec2T<double> p{0.3, -0.7};
  const Vec2T<double> q1 = T_a_c * p;
  const Vec2T<double> q2 = T_a_b * (T_b_c * p);
  EXPECT_NEAR(q1.x, q2.x, 1e-12);
  EXPECT_NEAR(q1.y, q2.y, 1e-12);

  expect_pose_near(T_a_b * T_a_b.inverse(), P(), 1e-12);
  expect_pose_near(T_a_b.inverse() * T_a_b, P(), 1e-12);
}

TEST(Pose2, ExpLogRoundTrip) {
  const double cases[][3] = {{0.1, -0.2, 0.3},
                             {1.0, 2.0, -2.5},
                             {0.0, 0.0, 0.0},
                             {0.5, 0.5, 1e-7},
                             {-3.0, 1.0, 3.1},
                             {0.2, 0.0, -1e-5}};
  for (const auto& c : cases) {
    Vec3T<double> d;
    d[0] = c[0];
    d[1] = c[1];
    d[2] = c[2];
    const Vec3T<double> back = P::exp(d).log();
    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(back[k], d[k], 1e-10) << "case " << c[0] << " " << c[1] << " " << c[2];
    }
    const P T(c[0], c[1], c[2]);
    expect_pose_near(P::exp(T.log()), T, 1e-10);
  }
}

TEST(Pose2, FloatExpLogStayFiniteForSmallAngles) {
  // Float has about 7 digits: 1 - cos(th) cancels to 0 for th below about 3e-4, just above
  // the Taylor branch. Log and Exp must stay finite and round-trip there.
  for (const float th : {1.0e-4F, 1.2e-4F, 1.7e-4F, 3.0e-4F, 7.0e-4F, -2.0e-4F}) {
    Vec3f d;
    d[0] = 0.09F;
    d[1] = -0.02F;
    d[2] = th;
    const Vec3f back = Pose2::exp(d).log();
    for (int k = 0; k < 3; ++k) {
      ASSERT_TRUE(std::isfinite(back[k])) << "th " << th;
      EXPECT_NEAR(back[k], d[k], 1e-6F) << "th " << th << " k " << k;
    }
  }
}

TEST(Pose2, ExpIsTheIntegratedConstantTwist) {
  // Exp(delta) = the pose reached by moving with constant body twist delta for 1 s:
  // integrate in many small body steps and compare.
  Vec3T<double> d;
  d[0] = 1.2;
  d[1] = -0.4;
  d[2] = 1.7;
  P T;
  const int n = 100000;
  for (int i = 0; i < n; ++i) {
    T = T * P(d[0] / n, d[1] / n, d[2] / n);
  }
  expect_pose_near(P::exp(d), T, 1e-4);
}

TEST(Pose2, RightPerturbationIsInTheBodyFrame) {
  // T * Exp([dx, 0, 0]) moves along the body x axis, which is world y at yaw = pi/2.
  const P T(1.0, 1.0, kPi / 2);
  Vec3T<double> d;
  d[0] = 0.2;
  expect_pose_near(T * P::exp(d), P(1.0, 1.2, kPi / 2), 1e-12);
}

}  // namespace
}  // namespace mss2d
