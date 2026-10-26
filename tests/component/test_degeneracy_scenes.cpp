// Component tests D-01..D-03 (docs/Test_CI_CD.md, ### degeneracy): degeneracy analysis of
// point-to-plane Hessians built from synthetic corridor, room and floor scenes.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "degeneracy_scenes.hpp"

namespace mss::degeneracy {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kYawRad = 30.0 * kDegToRad;
constexpr double kAxisTolRad = 5.0 * kDegToRad;  // D-01 criterion
constexpr double kPointSigmaM = 0.01;
constexpr double kNormalSigma = 0.01;
constexpr std::uint32_t kSeed = 42;
// config/default.yaml values with the feature flag on.
constexpr DegeneracyConfig kCfg{true, 100.0, 10.0};

common::Vec6 corridor_axis6() {
  common::Vec6 axis = common::Vec6::Zero();
  axis.head<3>() = Eigen::AngleAxisd(kYawRad, common::Vec3::UnitZ()) * common::Vec3::UnitX();
  return axis;
}

common::Mat6 corridor_hessian() {
  return test::point_to_plane_hessian(
      test::make_corridor_scene(kYawRad), kPointSigmaM, kNormalSigma, kSeed);
}

common::Vec6 test_delta() {
  common::Vec6 delta;
  delta << 0.3, -0.2, 0.1, 0.01, -0.02, 0.03;
  return delta;
}

}  // namespace

TEST(DegeneracyScenes, D01_CorridorFlaggedAlongAxis) {
  const common::DegeneracyReport r = analyze(corridor_hessian(), kCfg);
  const std::array<bool, 6> expected{true, false, false, false, false, false};
  EXPECT_EQ(r.degenerate, expected);
  const common::Vec6 v0 = r.eigvecs.col(0);
  const double angle_rad = std::acos(std::min(1.0, std::abs(v0.dot(corridor_axis6()))));
  EXPECT_LT(angle_rad, kAxisTolRad);
}

TEST(DegeneracyScenes, D02_RoomNotFlagged) {
  const common::DegeneracyReport r = analyze(
      test::point_to_plane_hessian(test::make_room_scene(), kPointSigmaM, kNormalSigma, kSeed),
      kCfg);
  EXPECT_EQ(r.degenerate, (std::array<bool, 6>{}));
  EXPECT_GT(r.eigvals(0), kCfg.eig_threshold);
}

TEST(DegeneracyScenes, D03_ProjectedUpdateAndRelaxedCovariance) {
  const common::Mat6 H = corridor_hessian();
  const common::DegeneracyReport r = analyze(H, kCfg);
  ASSERT_TRUE(r.degenerate[0]);
  const common::Vec6 v0 = r.eigvecs.col(0);
  const common::Vec6 delta = test_delta();
  const common::Vec6 out = project_update(delta, r);
  EXPECT_LT(std::abs(v0.dot(out)), 1e-12 * delta.norm());  // D-03: update along it is 0
  const common::PoseCovariance cov = H.inverse();
  const common::PoseCovariance relaxed = relax_covariance(cov, r, kCfg);
  // D-03: covariance along the degenerate direction grows >= relax_factor times.
  EXPECT_GE(v0.dot(relaxed * v0), kCfg.relax_factor * (1.0 - 1e-9) * v0.dot(cov * v0));
}

TEST(DegeneracyScenes, D03_DisabledIsIdentity) {
  const common::Mat6 H = corridor_hessian();
  DegeneracyConfig cfg = kCfg;
  cfg.enabled = false;
  const common::DegeneracyReport r = analyze(H, cfg);
  EXPECT_EQ(r.degenerate, (std::array<bool, 6>{}));
  const common::Vec6 delta = test_delta();
  const common::Vec6 out = project_update(delta, r);
  EXPECT_TRUE((out.array() == delta.array()).all());
  const common::PoseCovariance cov = H.inverse();
  const common::PoseCovariance relaxed = relax_covariance(cov, r, cfg);
  EXPECT_TRUE((relaxed.array() == cov.array()).all());
}

TEST(DegeneracyScenes, SinglePlaneThreeDirections) {
  // Noise-free floor: rho_x, rho_y and phi_z are unconstrained (repeated zero eigenvalue);
  // the projection must remove the whole subspace whatever basis the solver picked.
  const common::DegeneracyReport r =
      analyze(test::point_to_plane_hessian(test::make_floor_scene(), 0.0, 0.0, kSeed), kCfg);
  const std::array<bool, 6> expected{true, true, true, false, false, false};
  EXPECT_EQ(r.degenerate, expected);
  const common::Vec6 delta = test_delta();
  const common::Vec6 out = project_update(delta, r);
  const double tol = 1e-9 * delta.norm();
  EXPECT_LT(std::abs(out(0)), tol);
  EXPECT_LT(std::abs(out(1)), tol);
  EXPECT_LT(std::abs(out(5)), tol);
  for (const int i : {2, 3, 4}) {
    EXPECT_NEAR(out(i), delta(i), tol) << "i=" << i;
  }
}

}  // namespace mss::degeneracy
