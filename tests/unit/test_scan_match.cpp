// Unit tests of mss::scan_match (docs/Test_CI_CD.md S-01; docs/derivations/scan_match.md).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "plane_fit.hpp"

namespace mss::scan_match {
namespace {

using common::Pose;
using common::Vec3;
using common::Vec6;

constexpr double kPi = 3.14159265358979323846;

map::NeighborBuffer make_buffer(const std::initializer_list<Vec3>& pts) {
  map::NeighborBuffer buf;
  for (const Vec3& p : pts) {
    buf.p_w[static_cast<std::size_t>(buf.size)] = p;
    buf.dist2[static_cast<std::size_t>(buf.size)] = 0.0F;
    ++buf.size;
  }
  return buf;
}

// Residual r = n_w . (T_w_l p_l - q_w); variable xi = [rho, phi], right perturbation
// T_w_l <- T_w_l * Exp(xi). S-01: analytic J_r_xi vs central differences, rel. error < 1e-6.
TEST(PointToPlane, S01_JacobianMatchesNumerical) {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u10(-10.0, 10.0);
  std::uniform_real_distribution<double> u5(-5.0, 5.0);
  std::normal_distribution<double> n01(0.0, 1.0);
  constexpr double kH = 1e-6;
  for (int trial = 0; trial < 100; ++trial) {
    const Vec3 p_l(u10(rng), u10(rng), u10(rng));
    const Vec3 q_w(u10(rng), u10(rng), u10(rng));
    const Vec3 n_raw(n01(rng), n01(rng), n01(rng));
    const Vec3 n_w = n_raw.normalized();
    const Vec3 phi(n01(rng), n01(rng), n01(rng));
    const Vec3 t(u5(rng), u5(rng), u5(rng));
    const Pose T_w_l{common::so3_exp(phi), t};
    Vec6 J = Vec6::Zero();
    point_to_plane_residual(p_l, q_w, n_w, T_w_l, &J);
    Vec6 J_num = Vec6::Zero();
    for (int k = 0; k < 6; ++k) {
      const Vec6 e = Vec6::Unit(k) * kH;
      const double rp = point_to_plane_residual(p_l, q_w, n_w, common::boxplus(T_w_l, e), nullptr);
      const double rm = point_to_plane_residual(p_l, q_w, n_w, common::boxplus(T_w_l, -e), nullptr);
      J_num(k) = (rp - rm) / (2.0 * kH);
    }
    const double rel = (J - J_num).norm() / std::max(J_num.norm(), 1e-12);
    EXPECT_LT(rel, 1e-6) << "trial " << trial;
  }
}

TEST(PointToPlane, ResidualIsSignedDistanceAndNullJacobianOk) {
  const Vec3 p_l(1.0, 2.0, 3.0);
  const Pose T_w_l{common::so3_exp(Vec3(0.0, 0.0, kPi / 2.0)), Vec3(0.5, 0.0, 0.0)};
  const Vec3 q_w = Vec3::Zero();
  const Vec3 n_w = Vec3::UnitX();
  Vec6 J = Vec6::Zero();
  const double r = point_to_plane_residual(p_l, q_w, n_w, T_w_l, &J);
  const Vec3 p_w = T_w_l * p_l;
  EXPECT_NEAR(r, p_w.x(), 1e-12);
  EXPECT_EQ(r, point_to_plane_residual(p_l, q_w, n_w, T_w_l, nullptr));
}

TEST(PlaneFit, ExactPlaneRecoversNormalAndZeroRms) {
  const map::NeighborBuffer buf = make_buffer({Vec3(0.0, 0.0, 2.0),
                                               Vec3(0.1, 0.0, 2.0),
                                               Vec3(0.0, 0.1, 2.0),
                                               Vec3(0.1, 0.1, 2.0),
                                               Vec3(0.05, 0.05, 2.0)});
  const std::optional<detail::PlaneFit> fit = detail::fit_plane(buf);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(std::abs(fit->n_w.z()), 1.0, 1e-12);
  const Vec3 centroid(0.05, 0.05, 2.0);
  EXPECT_LT((fit->q_w - centroid).norm(), 1e-12);
  EXPECT_LT(fit->rms_m, 1e-12);
}

TEST(PlaneFit, RmsMatchesOffPlaneSpread) {
  const map::NeighborBuffer buf = make_buffer({Vec3(0.0, 0.0, 0.03),
                                               Vec3(0.2, 0.0, 0.03),
                                               Vec3(0.0, 0.2, 0.03),
                                               Vec3(0.2, 0.2, 0.03),
                                               Vec3(0.0, 0.0, -0.03),
                                               Vec3(0.2, 0.0, -0.03),
                                               Vec3(0.0, 0.2, -0.03),
                                               Vec3(0.2, 0.2, -0.03)});
  const std::optional<detail::PlaneFit> fit = detail::fit_plane(buf);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->rms_m, 0.03, 1e-9);
}

TEST(PlaneFit, FewerThanThreeIsNullopt) {
  EXPECT_FALSE(detail::fit_plane(make_buffer({})).has_value());
  EXPECT_FALSE(detail::fit_plane(make_buffer({Vec3(0.0, 0.0, 0.0)})).has_value());
  EXPECT_FALSE(
      detail::fit_plane(make_buffer({Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)})).has_value());
  EXPECT_TRUE(detail::fit_plane(
                  make_buffer({Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)}))
                  .has_value());
}

// Planarity gate (derivation section 3): two faces at 90 deg (z = 0 and x = 0) pass the
// 0.05 m RMS gate (rms 0.038 m) but have lambda_min / lambda_mid = 0.46; a noisy plane has
// a ratio of 3e-4. Gate off (ratio 0) accepts both.
TEST(PlaneFit, PlanarityGateRejectsEdgeAcceptsPlane) {
  const map::NeighborBuffer edge = make_buffer({Vec3(0.0, 0.0, 0.0),
                                                Vec3(0.125, 0.0, 0.0),
                                                Vec3(0.0, 0.125, 0.0),
                                                Vec3(0.0, 0.0, 0.125),
                                                Vec3(0.0, 0.125, 0.125)});
  const map::NeighborBuffer plane = make_buffer({Vec3(0.0, 0.0, 0.0),
                                                 Vec3(0.125, 0.0, 0.003),
                                                 Vec3(0.0, 0.125, -0.002),
                                                 Vec3(0.125, 0.125, 0.001),
                                                 Vec3(0.06, 0.06, -0.002)});
  const std::optional<detail::PlaneFit> fe = detail::fit_plane(edge);
  const std::optional<detail::PlaneFit> fp = detail::fit_plane(plane);
  ASSERT_TRUE(fe.has_value() && fp.has_value());
  EXPECT_LT(fe->rms_m, 0.05);
  EXPECT_NEAR(fe->lambda_min_m2 / fe->lambda_mid_m2, 0.4597, 1e-3);
  EXPECT_LT(fp->lambda_min_m2 / fp->lambda_mid_m2, 1e-3);
  for (const double ratio : {0.05, 0.1, 0.2}) {
    EXPECT_FALSE(detail::passes_planarity(*fe, ratio)) << ratio;
    EXPECT_TRUE(detail::passes_planarity(*fp, ratio)) << ratio;
  }
  EXPECT_TRUE(detail::passes_planarity(*fe, 0.0));
  EXPECT_TRUE(detail::passes_planarity(*fp, 0.0));
  // Collinear: lambda_min = lambda_mid = 0 passes (no division).
  const std::optional<detail::PlaneFit> fl = detail::fit_plane(
      make_buffer({Vec3(0.0, 0.0, 0.0), Vec3(0.1, 0.0, 0.0), Vec3(0.2, 0.0, 0.0)}));
  ASSERT_TRUE(fl.has_value());
  EXPECT_TRUE(detail::passes_planarity(*fl, 0.1));
}

// Collinearity gate (derivation section 3): five points on one line (a single ring) have
// lambda_mid = 0 and are rejected at any ratio > 0; a 2-D patch passes; points 1 cm off a
// line (two rings 1 cm apart over 0.5 m) have lambda_mid / lambda_max ~ 4e-4. Off accepts all.
TEST(PlaneFit, CollinearityGateRejectsLineAcceptsPatch) {
  const map::NeighborBuffer line = make_buffer({Vec3(0.0, 0.0, 0.0),
                                                Vec3(0.1, 0.0, 0.0),
                                                Vec3(0.2, 0.0, 0.0),
                                                Vec3(0.3, 0.0, 0.0),
                                                Vec3(0.4, 0.0, 0.0)});
  const map::NeighborBuffer thin = make_buffer({Vec3(0.0, 0.0, 0.0),
                                                Vec3(0.125, 0.01, 0.0),
                                                Vec3(0.25, 0.0, 0.0),
                                                Vec3(0.375, 0.01, 0.0),
                                                Vec3(0.5, 0.0, 0.0)});
  const map::NeighborBuffer patch = make_buffer({Vec3(0.0, 0.0, 0.0),
                                                 Vec3(0.125, 0.0, 0.0),
                                                 Vec3(0.0, 0.125, 0.0),
                                                 Vec3(0.125, 0.125, 0.0),
                                                 Vec3(0.06, 0.06, 0.0)});
  const std::optional<detail::PlaneFit> fl = detail::fit_plane(line);
  const std::optional<detail::PlaneFit> ft = detail::fit_plane(thin);
  const std::optional<detail::PlaneFit> fp = detail::fit_plane(patch);
  ASSERT_TRUE(fl.has_value() && ft.has_value() && fp.has_value());
  EXPECT_NEAR(fl->lambda_mid_m2, 0.0, 1e-12);
  EXPECT_LT(ft->lambda_mid_m2 / ft->lambda_max_m2, 1e-3);
  EXPECT_GT(fp->lambda_mid_m2 / fp->lambda_max_m2, 0.5);
  for (const double ratio : {0.01, 0.05, 0.1}) {
    EXPECT_FALSE(detail::passes_collinearity(*fl, ratio)) << ratio;
    EXPECT_FALSE(detail::passes_collinearity(*ft, ratio)) << ratio;
    EXPECT_TRUE(detail::passes_collinearity(*fp, ratio)) << ratio;
  }
  EXPECT_TRUE(detail::passes_collinearity(*fl, 0.0));
  EXPECT_TRUE(detail::passes_collinearity(*ft, 0.0));
  EXPECT_TRUE(detail::passes_collinearity(*fp, 0.0));
}

pipeline::Config default_config() {
  const common::Result<pipeline::Config> cfg =
      pipeline::load_config(std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml");
  if (!cfg.is_ok()) {
    throw std::runtime_error("default config: " + cfg.reason());
  }
  return cfg.value();
}

// Pushes an `nx` x `ny` grid with `spacing_m` on the plane z = `z_m` (frame of the scan).
void push_grid(common::ScanFrame& scan, int nx, int ny, double spacing_m, double z_m) {
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      scan.push(static_cast<float>(i * spacing_m),
                static_cast<float>(j * spacing_m),
                static_cast<float>(z_m),
                0.0F,
                0U,
                0U);
    }
  }
}

TEST(ScanMatcher, ConstructorRejectsInvalidConfig) {
  const pipeline::Config c = default_config();
  const std::size_t kMax = c.realtime.max_points_per_scan;
  EXPECT_NO_THROW(ScanMatcher(c.scan_match, c.degeneracy, kMax));
  const auto expect_throw = [&](void (*mutate)(ScanMatchConfig&)) {
    ScanMatchConfig bad = c.scan_match;
    mutate(bad);
    EXPECT_THROW(ScanMatcher(bad, c.degeneracy, kMax), std::invalid_argument);
  };
  expect_throw([](ScanMatchConfig& s) { s.max_iterations = 0; });
  expect_throw([](ScanMatchConfig& s) { s.rot_tol_rad = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.trans_tol_m = -1.0; });
  expect_throw([](ScanMatchConfig& s) { s.max_corr_dist_m = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.plane_rms_max_m = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.plane_min_eig_ratio = -0.1; });
  expect_throw([](ScanMatchConfig& s) { s.plane_min_eig_ratio = 1.0; });
  expect_throw([](ScanMatchConfig& s) { s.min_mid_eig_ratio = -0.1; });
  expect_throw([](ScanMatchConfig& s) { s.min_mid_eig_ratio = 1.0; });
  expect_throw([](ScanMatchConfig& s) { s.robust_huber_delta_m = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.min_inliers = 5; });
  expect_throw([](ScanMatchConfig& s) { s.cov_scale = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.min_residual_sigma_m = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.unobserved_var_m2 = 0.0; });
  expect_throw([](ScanMatchConfig& s) { s.unobserved_var_rad2 = -1.0; });
  expect_throw(
      [](ScanMatchConfig& s) { s.rot_tol_rad = std::numeric_limits<double>::quiet_NaN(); });
  EXPECT_THROW(ScanMatcher(c.scan_match, c.degeneracy, 0), std::invalid_argument);
}

TEST(ScanMatcher, ScanSizeBoundary) {
  const pipeline::Config c = default_config();
  ScanMatcher matcher(c.scan_match, c.degeneracy, 4);
  const map::VoxelMap empty_map(c.map);
  common::ScanFrame scan(5);
  push_grid(scan, 5, 1, 0.1, 0.0);
  const common::Result<common::MatchResult> r5 = matcher.align(scan, empty_map, Pose{});
  ASSERT_FALSE(r5.is_ok());
  EXPECT_EQ(r5.reason(), "scan larger than reserved max_points");
  scan.clear();
  push_grid(scan, 4, 1, 0.1, 0.0);
  const common::Result<common::MatchResult> r4 = matcher.align(scan, empty_map, Pose{});
  ASSERT_FALSE(r4.is_ok());
  EXPECT_EQ(r4.reason(), "too few inliers");
}

TEST(ScanMatcher, NonFiniteInitialPose) {
  const pipeline::Config c = default_config();
  ScanMatcher matcher(c.scan_match, c.degeneracy, 16);
  const map::VoxelMap m(c.map);
  common::ScanFrame scan(9);
  push_grid(scan, 3, 3, 0.1, 0.0);
  Pose T;
  T.p.x() = std::numeric_limits<double>::quiet_NaN();
  const common::Result<common::MatchResult> r1 = matcher.align(scan, m, T);
  ASSERT_FALSE(r1.is_ok());
  EXPECT_EQ(r1.reason(), "non-finite initial pose");
  Pose T2;
  T2.R(1, 2) = std::numeric_limits<double>::infinity();
  const common::Result<common::MatchResult> r2 = matcher.align(scan, m, T2);
  ASSERT_FALSE(r2.is_ok());
  EXPECT_EQ(r2.reason(), "non-finite initial pose");
}

TEST(ScanMatcher, EmptyScanOrMapIsTooFewInliers) {
  const pipeline::Config c = default_config();
  ScanMatcher matcher(c.scan_match, c.degeneracy, 16);
  map::VoxelMap grid_map(c.map);
  common::ScanFrame grid(9);
  push_grid(grid, 3, 3, 0.1, 0.0);
  grid_map.insert(grid, Pose{});
  const common::ScanFrame empty_scan(9);
  const common::Result<common::MatchResult> r1 = matcher.align(empty_scan, grid_map, Pose{});
  ASSERT_FALSE(r1.is_ok());
  EXPECT_EQ(r1.reason(), "too few inliers");
  const map::VoxelMap empty_map(c.map);
  const common::Result<common::MatchResult> r2 = matcher.align(grid, empty_map, Pose{});
  ASSERT_FALSE(r2.is_ok());
  EXPECT_EQ(r2.reason(), "too few inliers");
}

}  // namespace
}  // namespace mss::scan_match
