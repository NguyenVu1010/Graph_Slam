#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "mss2d/config.hpp"
#include "mss2d/distance_field.hpp"
#include "mss2d/occupancy_grid.hpp"

namespace mss2d {
namespace {

bool names(const Status& s, const char* key) {
  return !s.is_ok() && std::strstr(s.reason(), key) != nullptr;
}

TEST(GridConfig, DefaultsValidAndBadKeysNamed) {
  EXPECT_TRUE(GridConfig{}.validate().is_ok());
  EXPECT_TRUE(Config{}.validate().is_ok());
  GridConfig c;
  c.side_cells = kMaxGridSide + 1;
  EXPECT_TRUE(names(c.validate(), "side_cells"));
  c = GridConfig{};
  c.resolution_m = 0.0F;
  EXPECT_TRUE(names(c.validate(), "resolution_m"));
  c = GridConfig{};
  c.hit_log_odds = -1.0F;
  EXPECT_TRUE(names(c.validate(), "hit_log_odds"));
  c = GridConfig{};
  c.miss_log_odds = 0.0F;
  EXPECT_TRUE(names(c.validate(), "miss_log_odds"));
  c = GridConfig{};
  c.max_distance_m = 0.06F;  // < 2 cells
  EXPECT_TRUE(names(c.validate(), "max_distance_m"));
  Config full;
  full.grid.side_cells = 0;
  EXPECT_TRUE(names(full.validate(), "side_cells"));
}

TEST(MatcherConfig, DefaultsValidAndBadKeysNamed) {
  EXPECT_TRUE(MatcherConfig{}.validate().is_ok());
  MatcherConfig c;
  c.max_range_m = 0.1F;
  EXPECT_TRUE(names(c.validate(), "max_range_m"));
  c = MatcherConfig{};
  c.search_angle_step_rad = 0.0F;
  EXPECT_TRUE(names(c.validate(), "search_angle_step_rad"));
  c = MatcherConfig{};
  c.max_iterations = 0;
  EXPECT_TRUE(names(c.validate(), "max_iterations"));
  c = MatcherConfig{};
  c.min_points = 2;
  EXPECT_TRUE(names(c.validate(), "min_points"));
  Config full;
  full.matcher.degeneracy_eig_threshold = -1.0F;
  EXPECT_TRUE(names(full.validate(), "degeneracy_eig_threshold"));
}

class GridTest : public ::testing::Test {
 protected:
  GridTest() : grid(std::make_unique<OccupancyGrid>(cfg)) { grid->reset(Pose2()); }
  GridConfig cfg;
  std::unique_ptr<OccupancyGrid> grid;
};

TEST_F(GridTest, CellGeometryCentresTheGrid) {
  int ix = 0;
  int iy = 0;
  ASSERT_TRUE(grid->cell_of(Vec2f{0.01F, -0.01F}, ix, iy));
  EXPECT_EQ(ix, 100);
  EXPECT_EQ(iy, 99);
  const Vec2f c = grid->cell_center(100, 99);
  EXPECT_NEAR(c.x, 0.025F, 1e-6F);
  EXPECT_NEAR(c.y, -0.025F, 1e-6F);
  EXPECT_FALSE(grid->cell_of(Vec2f{5.01F, 0.0F}, ix, iy));
  EXPECT_FALSE(grid->cell_of(Vec2f{0.0F, -5.01F}, ix, iy));
  EXPECT_EQ(grid->probability(3, 4), 0.5F);  // unknown
}

TEST_F(GridTest, BeamFreesCrossedCellsAndHitsTheEnd) {
  PointCloud2d pts;
  pts.count = 1;
  pts.pts[0] = Vec2f{1.01F, 0.0F};               // along +x from the sensor
  grid->insert(pts, Pose2(0.01F, 0.01F, 0.0F));  // sensor in cell (100, 100)
  // End at x = 1.02: cell 100 + 20 = 120.
  EXPECT_EQ(grid->log_odds(120, 100), grid->hit_step());
  EXPECT_GT(grid->probability(120, 100), 0.69F);
  for (int ix = 100; ix < 120; ++ix) {
    EXPECT_EQ(grid->log_odds(ix, 100), grid->miss_step()) << ix;
  }
  EXPECT_EQ(grid->log_odds(121, 100), 0);
  EXPECT_EQ(grid->log_odds(110, 101), 0);
  EXPECT_LT(grid->probability(110, 100), 0.41F);
}

TEST_F(GridTest, DiagonalBeamIsConnected) {
  PointCloud2d pts;
  pts.count = 1;
  pts.pts[0] = Vec2f{0.5F, 0.3F};
  grid->insert(pts, Pose2(0.01F, 0.01F, 0.0F));
  int n_miss = 0;
  for (int iy = 0; iy < grid->side(); ++iy) {
    for (int ix = 0; ix < grid->side(); ++ix) {
      n_miss += grid->log_odds(ix, iy) < 0 ? 1 : 0;
    }
  }
  // Bresenham visits max(|dx|, |dy|) = 10 cells before the end cell (110, 106).
  EXPECT_EQ(n_miss, 10);
  EXPECT_EQ(grid->log_odds(110, 106), grid->hit_step());
}

TEST_F(GridTest, HitWinsOverMissInTheSameScanAndLogOddsClamp) {
  PointCloud2d pts;
  pts.count = 2;
  pts.pts[0] = Vec2f{0.5F, 0.0F};  // ends in cell 110
  pts.pts[1] = Vec2f{1.0F, 0.0F};  // crosses cell 110
  for (int i = 0; i < 50; ++i) {
    grid->insert(pts, Pose2(0.01F, 0.01F, 0.0F));
  }
  EXPECT_EQ(grid->log_odds(110, 100), 127);
  EXPECT_EQ(grid->log_odds(105, 100), -127);
  EXPECT_NEAR(grid->probability(110, 100), 1.0F / (1.0F + std::exp(-3.5F)), 1e-6F);
  EXPECT_TRUE(grid->occupied(110, 100));
  EXPECT_FALSE(grid->occupied(105, 100));
}

TEST_F(GridTest, SensorOutsideTheGridInsertsNothingBeamLeavingIsClipped) {
  PointCloud2d pts;
  pts.count = 1;
  pts.pts[0] = Vec2f{1.0F, 0.0F};
  grid->insert(pts, Pose2(-6.0F, 0.0F, 0.0F));
  EXPECT_EQ(grid->log_odds(0, 100), 0);
  pts.pts[0] = Vec2f{8.0F, 0.0F};
  grid->insert(pts, Pose2(4.51F, 0.01F, 0.0F));  // sensor cell 190, end far outside
  EXPECT_EQ(grid->log_odds(199, 100), grid->miss_step());
  EXPECT_EQ(grid->log_odds(190, 100), grid->miss_step());
}

// --- distance field ---

/// Brute force: min over occupied centres, in m, clamped.
float brute_distance(const OccupancyGrid& g, int ix, int iy, float max_d) {
  float best2 = 1e30F;
  for (int y = 0; y < g.side(); ++y) {
    for (int x = 0; x < g.side(); ++x) {
      if (g.occupied(x, y)) {
        const float dx = static_cast<float>(x - ix);
        const float dy = static_cast<float>(y - iy);
        best2 = std::min(best2, dx * dx + dy * dy);
      }
    }
  }
  return std::min(std::sqrt(best2) * g.resolution(), max_d);
}

TEST(DistanceField, MatchesBruteForceOnRandomSmallGrids) {
  GridConfig cfg;
  cfg.side_cells = 40;
  cfg.resolution_m = 0.1F;
  cfg.max_distance_m = 1.2F;
  auto grid = std::make_unique<OccupancyGrid>(cfg);
  auto df = std::make_unique<DistanceField>(cfg);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-1.95F, 1.95F);
  for (int trial = 0; trial < 5; ++trial) {
    grid->reset(Pose2());
    PointCloud2d pts;
    pts.count = 3 + 4 * trial;
    for (std::uint32_t i = 0; i < pts.count; ++i) {
      pts.pts[i] = Vec2f{u(rng), u(rng)};
    }
    grid->insert(pts, Pose2(0.0F, 0.0F, 0.0F));
    df->compute(*grid);
    float worst = 0.0F;
    for (int iy = 0; iy < cfg.side_cells; ++iy) {
      for (int ix = 0; ix < cfg.side_cells; ++ix) {
        const float want = brute_distance(*grid, ix, iy, cfg.max_distance_m);
        worst = std::max(worst, std::abs(df->at_cell(ix, iy) - want));
      }
    }
    // Only the uint8 quantisation: half a step.
    EXPECT_LE(worst, 0.5F * cfg.max_distance_m / 255.0F + 1e-6F) << "trial " << trial;
  }
}

TEST(DistanceField, EmptyGridIsMaxDistanceEverywhere) {
  GridConfig cfg;
  auto grid = std::make_unique<OccupancyGrid>(cfg);
  auto df = std::make_unique<DistanceField>(cfg);
  grid->reset(Pose2());
  df->compute(*grid);
  EXPECT_FLOAT_EQ(df->at_cell(0, 0), cfg.max_distance_m);
  EXPECT_FLOAT_EQ(df->at_cell(123, 77), cfg.max_distance_m);
}

TEST(DistanceField, BilinearGradientMatchesNumericalDifference) {
  // The formula in double.
  const double v[4] = {0.3, 0.7, -0.2, 1.1};
  const double h = 1e-6;
  for (const double fx : {0.1, 0.5, 0.83}) {
    for (const double fy : {0.05, 0.4, 0.99}) {
      double gx = 0;
      double gy = 0;
      double dummy = 0;
      detail::bilinear(v[0], v[1], v[2], v[3], fx, fy, gx, gy);
      const double nx = (detail::bilinear(v[0], v[1], v[2], v[3], fx + h, fy, dummy, dummy) -
                         detail::bilinear(v[0], v[1], v[2], v[3], fx - h, fy, dummy, dummy)) /
                        (2 * h);
      const double ny = (detail::bilinear(v[0], v[1], v[2], v[3], fx, fy + h, dummy, dummy) -
                         detail::bilinear(v[0], v[1], v[2], v[3], fx, fy - h, dummy, dummy)) /
                        (2 * h);
      EXPECT_NEAR(gx, nx, 1e-6 * std::max(1.0, std::abs(nx)));
      EXPECT_NEAR(gy, ny, 1e-6 * std::max(1.0, std::abs(ny)));
    }
  }
}

TEST(DistanceField, SampleInterpolatesCellCentresWithMetricGradient) {
  GridConfig cfg;
  auto grid = std::make_unique<OccupancyGrid>(cfg);
  auto df = std::make_unique<DistanceField>(cfg);
  grid->reset(Pose2());
  // A wall of hits along x = 1.025 (cell column 120).
  PointCloud2d pts;
  pts.count = 0;
  for (int i = -20; i <= 20; ++i) {
    pts.pts[pts.count++] = Vec2f{1.02F, 0.05F * static_cast<float>(i) + 0.01F};
  }
  grid->insert(pts, Pose2());
  df->compute(*grid);
  float d = 0;
  float gx = 0;
  float gy = 0;
  // At cell centres the sample equals the stored value.
  const Vec2f c = grid->cell_center(116, 100);
  ASSERT_TRUE(df->sample(c, d, gx, gy));
  EXPECT_NEAR(d, df->at_cell(116, 100), 1e-6F);
  // Between centres, left of the wall: d = 1.025 - x, gradient (-1, 0).
  ASSERT_TRUE(df->sample(Vec2f{0.87F, 0.013F}, d, gx, gy));
  EXPECT_NEAR(d, 1.025F - 0.87F, 0.003F);
  EXPECT_NEAR(gx, -1.0F, 0.02F);
  EXPECT_NEAR(gy, 0.0F, 0.02F);
  // Numerical difference of sample() itself.
  const float h = 0.004F;
  float dp = 0;
  float dm = 0;
  float u = 0;
  ASSERT_TRUE(df->sample(Vec2f{0.87F + h, 0.013F}, dp, u, u));
  ASSERT_TRUE(df->sample(Vec2f{0.87F - h, 0.013F}, dm, u, u));
  EXPECT_NEAR(gx, (dp - dm) / (2 * h), 1e-3F);
  // Outside the interpolation domain.
  EXPECT_FALSE(df->sample(Vec2f{4.99F, 0.0F}, d, gx, gy));
  EXPECT_FALSE(df->sample(Vec2f{0.0F, -7.0F}, d, gx, gy));
}

}  // namespace
}  // namespace mss2d
