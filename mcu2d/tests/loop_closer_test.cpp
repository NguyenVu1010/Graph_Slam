#include "mss2d/loop_closer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "sim_world.hpp"

namespace mss2d {
namespace {

using P = Pose2T<double>;

Pose2 to_float(const P& p) {
  return Pose2(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.yaw));
}

/// Submap at the world origin with noise-free scans from `poses` (LiDAR poses in w = s).
std::unique_ptr<OccupancyGrid> build_map(const sim::World& w,
                                         const Config& cfg,
                                         std::initializer_list<P> poses) {
  auto grid = std::make_unique<OccupancyGrid>(cfg.grid);
  grid->reset(Pose2());
  auto scan = std::make_unique<Scan2d>();
  auto pts = std::make_unique<PointCloud2d>();
  for (const P& T : poses) {
    sim::make_scan(w, T, 720, 12.0, 0.0, 1, *scan);
    scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
    grid->insert(*pts, to_float(T));
  }
  return grid;
}

std::unique_ptr<PointCloud2d> query(const sim::World& w, const Config& cfg, const P& T_w_l) {
  auto scan = std::make_unique<Scan2d>();
  auto pts = std::make_unique<PointCloud2d>();
  sim::make_scan(w, T_w_l, 720, 12.0, 0.01, 7, *scan);
  scan_to_points(*scan, cfg.matcher.min_range_m, cfg.matcher.max_range_m, *pts);
  return pts;
}

TEST(CompactSubmap, MarksTheCoarseCellOfEveryOccupiedCell) {
  const Config cfg;
  auto grid = build_map(sim::room_world(), cfg, {P(0.2, -0.1, 0.3)});
  auto c = std::make_unique<CompactSubmap>();
  compact_submap(*grid, *c);
  const int d = kLoopGridDownsample;
  const int side = grid->side() / d;
  int n_fine = 0;
  int n_coarse = 0;
  for (int y = 0; y < grid->side(); ++y) {
    for (int x = 0; x < grid->side(); ++x) {
      if (grid->occupied(x, y)) {
        ++n_fine;
        EXPECT_TRUE(c->occupied(x / d, y / d, side)) << x << " " << y;
      }
    }
  }
  for (int y = 0; y < side; ++y) {
    for (int x = 0; x < side; ++x) {
      if (!c->occupied(x, y, side)) {
        continue;
      }
      ++n_coarse;
      bool any = false;
      for (int k = 0; k < d * d; ++k) {
        any = any || grid->occupied(d * x + k % d, d * y + k / d);
      }
      EXPECT_TRUE(any) << x << " " << y;
    }
  }
  EXPECT_GT(n_fine, 300);
  EXPECT_LT(n_coarse, n_fine);
  EXPECT_EQ(sizeof(CompactSubmap), 1250U);
}

TEST(LoopMatcher, RecoversAKnownPoseFromAWrongGuessInsideTheWindow) {
  const Config cfg;
  const sim::World w = sim::room_world();
  auto grid = build_map(w, cfg, {P(0.0, 0.0, 0.0), P(0.5, 0.3, 0.2)});
  auto map = std::make_unique<CompactSubmap>();
  compact_submap(*grid, *map);
  auto matcher = std::make_unique<LoopMatcher>(cfg);

  const P truth(0.8, -0.4, 0.5);
  auto pts = query(w, cfg, truth);
  const Pose2 guess = to_float(truth * P(0.6, -0.5, 0.2));  // 0.78 m and 11 deg off
  std::vector<double> ms;
  LoopMatch m;
  for (int rep = 0; rep < 10; ++rep) {
    const auto t0 = std::chrono::steady_clock::now();
    m = matcher->match(*map, *pts, guess, 1.0F, 0.35F);
    const auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  const double err = std::hypot(m.T_s_l.x - truth.x, m.T_s_l.y - truth.y);
  const double err_yaw = std::abs(normalize_angle(m.T_s_l.yaw - truth.yaw));
  std::printf(
      "[ loop match ] status %d score %.3f (coarse %.3f) ratio %.2f error %.4f m %.3f deg"
      ", %.3f ms median (window 1 m, 20 deg, %d points)\n",
      static_cast<int>(m.status),
      static_cast<double>(m.score),
      static_cast<double>(m.coarse_score),
      static_cast<double>(m.eig_ratio),
      err,
      err_yaw * 180.0 / sim::kPi,
      ms[ms.size() / 2],
      m.n_points);
  EXPECT_EQ(m.status, LoopStatus::kAccepted);
  EXPECT_LT(err, 0.03);  // a quarter of a coarse cell; measured below 1 cm
  EXPECT_LT(err_yaw, 0.01);
}

TEST(LoopMatcher, RejectsAScanFromAnotherPlace) {
  const Config cfg;
  auto grid = build_map(sim::room_world(), cfg, {P(0.0, 0.0, 0.0)});
  auto map = std::make_unique<CompactSubmap>();
  compact_submap(*grid, *map);
  auto matcher = std::make_unique<LoopMatcher>(cfg);
  // A scan of a 2 m wide corridor against the 8 x 6 m room.
  auto pts = query(sim::corridor_world(), cfg, P(0.0, 0.0, 0.0));
  const LoopMatch m = matcher->match(*map, *pts, Pose2(), 1.5F, 0.35F);
  EXPECT_EQ(m.status, LoopStatus::kLowScore);
}

TEST(LoopMatcher, RejectsTheSymmetricCorridorAsDegenerate) {
  // Every shift along a featureless corridor fits equally well: a high score but no
  // information along the axis, so the candidate must not become a loop edge.
  const Config cfg;
  const sim::World w = sim::corridor_world();
  auto grid = build_map(w, cfg, {P(-1.0, 0.0, 0.0), P(0.0, 0.0, 0.0), P(1.0, 0.0, 0.0)});
  auto map = std::make_unique<CompactSubmap>();
  compact_submap(*grid, *map);
  auto matcher = std::make_unique<LoopMatcher>(cfg);
  auto pts = query(w, cfg, P(0.5, 0.05, 0.02));
  const LoopMatch m = matcher->match(*map, *pts, Pose2(1.2F, 0.0F, 0.0F), 1.5F, 0.35F);
  std::printf("[ corridor   ] score %.3f ratio %.4f\n",
              static_cast<double>(m.score),
              static_cast<double>(m.eig_ratio));
  EXPECT_GT(m.score, cfg.loop.min_score);
  EXPECT_EQ(m.status, LoopStatus::kDegenerate);
}

}  // namespace
}  // namespace mss2d
