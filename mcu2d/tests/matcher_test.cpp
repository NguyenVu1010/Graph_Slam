#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "mss2d/scan_matcher.hpp"
#include "sim_world.hpp"

namespace mss2d {
namespace {

constexpr double kDeg = sim::kPi / 180.0;

// --- Jacobian vs numerical difference (double) ---

struct SmoothField {
  // D(q) = sin(0.7 qx) + 0.3 qy^2 + 0.2 qx qy, a smooth stand-in for the distance field.
  static double value(double x, double y) { return std::sin(0.7 * x) + 0.3 * y * y + 0.2 * x * y; }
  static double gx(double x, double y) { return 0.7 * std::cos(0.7 * x) + 0.2 * y; }
  static double gy(double x, double y) { return 0.6 * y + 0.2 * x; }
};

TEST(ScanMatcherJacobian, MatchesCentralDifferenceOfRightPerturbation) {
  using P = Pose2T<double>;
  const P poses[] = {P(0.3, -1.2, 0.4), P(-2.0, 0.5, 2.9), P(1.0, 1.0, -1.7)};
  const Vec2T<double> pts[] = {{2.5, 0.3}, {-1.0, 4.0}, {0.2, -3.3}};
  const double h = 1e-6;
  for (const P& T : poses) {
    for (const Vec2T<double>& p : pts) {
      const Vec2T<double> q = T * p;
      const Vec3T<double> j =
          residual_jacobian(T, p, SmoothField::gx(q.x, q.y), SmoothField::gy(q.x, q.y));
      for (int k = 0; k < 3; ++k) {
        Vec3T<double> d;
        d[k] = h;
        const Vec2T<double> qp = (T * P::exp(d)) * p;
        d[k] = -h;
        const Vec2T<double> qm = (T * P::exp(d)) * p;
        const double num =
            (SmoothField::value(qp.x, qp.y) - SmoothField::value(qm.x, qm.y)) / (2 * h);
        EXPECT_LT(std::abs(j[k] - num), 1e-6 * std::max(1.0, std::abs(num)))
            << "k=" << k << " analytic " << j[k] << " numeric " << num;
      }
    }
  }
}

// --- known-answer matching on synthetic worlds ---

struct Errors {
  double pos_m;
  double yaw_deg;
};

Errors errors(const Pose2& got, const Pose2T<double>& truth) {
  const double dx = got.x - truth.x;
  const double dy = got.y - truth.y;
  return {std::sqrt(dx * dx + dy * dy),
          std::abs(normalize_angle(static_cast<double>(got.yaw) - truth.yaw)) / kDeg};
}

class MatcherTest : public ::testing::Test {
 protected:
  MatcherTest()
      : grid(std::make_unique<OccupancyGrid>(cfg.grid)),
        df(std::make_unique<DistanceField>(cfg.grid)),
        matcher(std::make_unique<ScanMatcher>(cfg.matcher)),
        scan(std::make_unique<Scan2d>()),
        pts(std::make_unique<PointCloud2d>()) {}

  /// Submap (frame s = world) from scans at T and at T shifted by extra_dx along world x.
  void build_map(const sim::World& w,
                 const Pose2T<double>& T,
                 double sigma,
                 std::uint32_t seed,
                 std::initializer_list<double> extra_dx = {}) {
    grid->reset(Pose2());
    insert_at(w, T, sigma, seed);
    for (const double dx : extra_dx) {
      insert_at(w, Pose2T<double>(T.x + dx, T.y, T.yaw), sigma, ++seed);
    }
    df = std::make_unique<DistanceField>(cfg.grid);
    df->compute(*grid);
  }

  void insert_at(const sim::World& w, const Pose2T<double>& T, double sigma, std::uint32_t seed) {
    sim::make_scan(w, T, 720, cfg.matcher.max_range_m, sigma, seed, *scan);
    scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
    grid->insert(
        *pts, Pose2(static_cast<float>(T.x), static_cast<float>(T.y), static_cast<float>(T.yaw)));
  }

  MatchResult match_from(const sim::World& w,
                         const Pose2T<double>& truth,
                         const Pose2& guess,
                         double sigma,
                         std::uint32_t seed) {
    sim::make_scan(w, truth, 720, cfg.matcher.max_range_m, sigma, seed, *scan);
    scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
    matcher = std::make_unique<ScanMatcher>(cfg.matcher);
    return matcher->match(*grid, *df, *pts, guess);
  }

  Config cfg;
  std::unique_ptr<OccupancyGrid> grid;
  std::unique_ptr<DistanceField> df;
  std::unique_ptr<ScanMatcher> matcher;
  std::unique_ptr<Scan2d> scan;
  std::unique_ptr<PointCloud2d> pts;
};

const Pose2T<double> kT1(0.2, -0.1, 0.3);
const Pose2T<double> kT2(0.2 + 0.15, -0.1 - 0.10, 0.3 + 5 * kDeg);
const Pose2 kGuess(0.2F, -0.1F, 0.3F);  // T1: the offset to T2 is unknown to the matcher

TEST_F(MatcherTest, RecoversKnownOffsetNoiseFree) {
  build_map(sim::room_world(), kT1, 0.0, 1);
  const MatchResult r = match_from(sim::room_world(), kT2, kGuess, 0.0, 2);
  const Errors e = errors(r.T_s_l, kT2);
  std::printf(
      "[ noise-free ] pos err %.4f m, yaw err %.4f deg, iters %d, cost %.2e m^2, "
      "score %.3f, points %d\n",
      e.pos_m,
      e.yaw_deg,
      r.iterations,
      r.cost_m2,
      r.correlative_score,
      r.n_points);
  EXPECT_EQ(r.status, MatchStatus::kOk);
  EXPECT_LT(e.pos_m, 0.01);
  EXPECT_LT(e.yaw_deg, 0.2);
  std::printf("[ room eig   ] %.3g %.3g %.3g\n",
              r.degeneracy.eig.values[0],
              r.degeneracy.eig.values[1],
              r.degeneracy.eig.values[2]);
  EXPECT_EQ(r.degeneracy.n_degenerate, 0);
  EXPECT_GT(r.n_points, 600);
}

TEST_F(MatcherTest, RecoversKnownOffsetWithRangeNoise) {
  build_map(sim::room_world(), kT1, 0.02, 11);
  const MatchResult r = match_from(sim::room_world(), kT2, kGuess, 0.02, 12);
  const Errors e = errors(r.T_s_l, kT2);
  std::printf("[ sigma 2 cm ] pos err %.4f m, yaw err %.4f deg, iters %d, cost %.2e m^2\n",
              e.pos_m,
              e.yaw_deg,
              r.iterations,
              r.cost_m2);
  EXPECT_NE(r.status, MatchStatus::kTooFewPoints);
  EXPECT_LT(e.pos_m, 0.03);
  EXPECT_LT(e.yaw_deg, 0.5);
}

TEST_F(MatcherTest, CorrelativeSearchRescuesAStartOutsideTheGaussNewtonBasin) {
  // With the default 0.5 m field clamp Gauss-Newton alone converged from every corner of the
  // default window in this room. A 0.3 m clamp narrows its basin enough to show the coarse
  // stage at work; the start is inside the default correlative window.
  cfg.grid.max_distance_m = 0.3F;
  const Pose2T<double> truth(0.2 - 0.28, -0.1 - 0.27, 0.3 + 9.5 * kDeg);
  build_map(sim::room_world(), kT1, 0.0, 1);

  // Gauss-Newton alone (zero search window) does not reach the answer...
  cfg.matcher.search_linear_m = 0.0F;
  cfg.matcher.search_angle_rad = 0.0F;
  const MatchResult gn_only = match_from(sim::room_world(), truth, kGuess, 0.0, 3);
  const Errors e_gn = errors(gn_only.T_s_l, truth);
  // ...with the default window it does.
  cfg.matcher = MatcherConfig{};
  const MatchResult full = match_from(sim::room_world(), truth, kGuess, 0.0, 3);
  const Errors e = errors(full.T_s_l, truth);
  std::printf("[ far start  ] GN only: %.4f m %.3f deg; correlative + GN: %.4f m %.4f deg\n",
              e_gn.pos_m,
              e_gn.yaw_deg,
              e.pos_m,
              e.yaw_deg);
  EXPECT_TRUE(e_gn.pos_m > 0.05 || e_gn.yaw_deg > 1.0);
  EXPECT_EQ(full.status, MatchStatus::kOk);
  EXPECT_LT(e.pos_m, 0.01);
  EXPECT_LT(e.yaw_deg, 0.2);
}

TEST_F(MatcherTest, CorridorFlagsTheAlongAxisDirection) {
  const Pose2T<double> t1(0.0, 0.0, 0.0);
  const Pose2T<double> t2(0.4, 0.05, 2 * kDeg);
  // Five scans along the corridor, as a submap accumulates them: one scan leaves holes in the
  // far wall cells (beam spacing > cell size at grazing angles), which observe x weakly.
  build_map(sim::corridor_world(), t1, 0.0, 1, {-1.0, -0.5, 0.5, 1.0});
  const MatchResult r = match_from(sim::corridor_world(), t2, Pose2(), 0.0, 2);
  const DegeneracyReport& d = r.degeneracy;
  std::printf("[ corridor   ] eigenvalues %.3g %.3g %.3g, v_min (%.3f %.3f %.3f)\n",
              d.eig.values[0],
              d.eig.values[1],
              d.eig.values[2],
              d.eig.vectors(0, 0),
              d.eig.vectors(1, 0),
              d.eig.vectors(2, 0));
  ASSERT_EQ(d.n_degenerate, 1);
  EXPECT_TRUE(d.degenerate[0]);
  EXPECT_GT(std::abs(d.eig.vectors(0, 0)), 0.99F);  // the corridor axis (x)
  // The observable directions are still right. y within res/2: both walls lie 1.5 cm above
  // their occupied cell centres (0.99 vs 0.975, -1.01 vs -1.025), so the map is shifted by
  // -1.5 cm and the cell-centre bias does not cancel here (docs/scan_matching.md section 5).
  EXPECT_NEAR(r.T_s_l.y, t2.y, 0.025);
  EXPECT_NEAR(normalize_angle(static_cast<double>(r.T_s_l.yaw) - t2.yaw) / kDeg, 0.0, 0.2);
}

TEST_F(MatcherTest, TooFewPoints) {
  build_map(sim::room_world(), kT1, 0.0, 1);
  pts->count = 10;
  const MatchResult r = matcher->match(*grid, *df, *pts, kGuess);
  EXPECT_EQ(r.status, MatchStatus::kTooFewPoints);
  EXPECT_FLOAT_EQ(r.T_s_l.x, kGuess.x);
}

TEST_F(MatcherTest, HostTimingOf720BeamMatch) {
  build_map(sim::room_world(), kT1, 0.0, 1);
  sim::make_scan(sim::room_world(), kT2, 720, cfg.matcher.max_range_m, 0.0, 2, *scan);
  scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
  std::vector<double> ms;
  ms.reserve(50);
  for (int i = 0; i < 50; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    const MatchResult r = matcher->match(*grid, *df, *pts, kGuess);
    const auto t1 = std::chrono::steady_clock::now();
    ASSERT_EQ(r.status, MatchStatus::kOk);
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  std::printf(
      "[ timing     ] match 720 beams: median %.3f ms, min %.3f ms (50 runs)\n", ms[25], ms[0]);
}

}  // namespace
}  // namespace mss2d
