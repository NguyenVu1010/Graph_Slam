// Cross-check of mss::common Lie wrappers against GTSAM Pose3 (C-02).
// See docs/derivations/lie_conventions.md, section "Library orders".
#include <cmath>
#include <random>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

namespace mss::common {
namespace {

constexpr double kPi = 3.14159265358979323846;
// GTSAM 4.2 SO3::Logmap switches to a first-order approximation when trace(R) + 1 < 1e-3,
// i.e. within ~0.032 rad of pi, with errors up to ~5e-4 rad there. Sophus is exact
// (see Lie.So3ExpLogRoundTrip), so the Logmap comparison stays below this margin.
constexpr double kGtsamLogmapNearPiMargin = 0.05;  // rad

Pose random_pose(std::mt19937& rng, double max_angle) {
  std::normal_distribution<double> n(0.0, 1.0);
  std::uniform_real_distribution<double> angle(0.0, max_angle);
  std::uniform_real_distribution<double> u(-5.0, 5.0);  // m
  Vec3 axis(n(rng), n(rng), n(rng));
  while (axis.norm() < 1e-6) {
    axis = Vec3(n(rng), n(rng), n(rng));
  }
  Pose T;
  T.R = so3_exp(Vec3(angle(rng) * axis.normalized()));
  T.p = Vec3(u(rng), u(rng), u(rng));
  return T;
}

// GTSAM Pose3::Logmap / Expmap use [phi, rho] (rotation first); mss uses [rho, phi].
// core/backend relies on exactly this permutation at its boundary:
//   xi_gtsam = [xi_mss.tail<3>(), xi_mss.head<3>()].
TEST(LieGtsam, LogmapIsPermutedSe3Log) {
  std::mt19937 rng(42);
  for (int i = 0; i < 1000; ++i) {
    const Pose T = random_pose(rng, kPi - kGtsamLogmapNearPiMargin);
    const gtsam::Pose3 T_gtsam(gtsam::Rot3(T.R), gtsam::Point3(T.p));
    const Vec6 xi = se3_log(T);                                   // [rho, phi]
    const Vec6 xi_gtsam = gtsam::Pose3::Logmap(T_gtsam);          // [phi, rho]
    EXPECT_LT((xi.head<3>() - xi_gtsam.tail<3>()).norm(), 1e-9);  // rho
    EXPECT_LT((xi.tail<3>() - xi_gtsam.head<3>()).norm(), 1e-9);  // phi
  }
}

// GTSAM Expmap of the permuted vector is the same pose as se3_exp, up to pi.
TEST(LieGtsam, ExpmapOfPermutedXiEqualsSe3Exp) {
  std::mt19937 rng(42);
  for (int i = 0; i < 1000; ++i) {
    const Pose T = random_pose(rng, kPi - 1e-6);
    const Vec6 xi = se3_log(T);  // [rho, phi]
    Vec6 xi_gtsam;
    xi_gtsam << xi.tail<3>(), xi.head<3>();  // [phi, rho]
    const gtsam::Pose3 T_gtsam = gtsam::Pose3::Expmap(xi_gtsam);
    const Pose T_mss = se3_exp(xi);
    EXPECT_LT((T_gtsam.rotation().matrix() - T_mss.R).norm(), 1e-9);
    EXPECT_LT((T_gtsam.translation() - T_mss.p).norm(), 1e-9);
  }
}

}  // namespace
}  // namespace mss::common
