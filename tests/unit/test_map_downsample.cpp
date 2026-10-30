// Unit tests of detail::voxel_centroids (core/pipeline/src/map_downsample.hpp), the
// map_export.voxel_m downsampling of Estimator::export_map.
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "map_downsample.hpp"

namespace mss::pipeline::detail {
namespace {

TEST(MapDownsample, ZeroVoxelLeavesPointsUnchanged) {
  const std::vector<Eigen::Vector3f> in = {{0.1F, 0.2F, 0.3F}, {0.1F, 0.2F, 0.3F}, {5.F, 6.F, 7.F}};
  std::vector<Eigen::Vector3f> pts = in;
  voxel_centroids(pts, 0.0);
  EXPECT_EQ(pts, in);
}

TEST(MapDownsample, CentroidPerVoxelInFirstMetOrder) {
  // Voxel 1 m: (0.1, 0.1, 0.1) and (0.3, 0.5, 0.9) share voxel (0,0,0); (-0.5, 0, 0) is in
  // (-1,0,0) (floor, not truncation); (2.2, 0, 0) and (2.8, 0, 0) share (2,0,0).
  std::vector<Eigen::Vector3f> pts = {{2.2F, 0.F, 0.F},
                                      {0.1F, 0.1F, 0.1F},
                                      {-0.5F, 0.F, 0.F},
                                      {0.3F, 0.5F, 0.9F},
                                      {2.8F, 0.F, 0.F}};
  voxel_centroids(pts, 1.0);
  ASSERT_EQ(pts.size(), 3U);
  EXPECT_TRUE(pts[0].isApprox(Eigen::Vector3f(2.5F, 0.F, 0.F), 1e-6F)) << pts[0].transpose();
  EXPECT_TRUE(pts[1].isApprox(Eigen::Vector3f(0.2F, 0.3F, 0.5F), 1e-6F)) << pts[1].transpose();
  EXPECT_TRUE(pts[2].isApprox(Eigen::Vector3f(-0.5F, 0.F, 0.F), 1e-6F)) << pts[2].transpose();
}

TEST(MapDownsample, EmptyStaysEmpty) {
  std::vector<Eigen::Vector3f> pts;
  voxel_centroids(pts, 0.5);
  EXPECT_TRUE(pts.empty());
}

// Review H1 finding 3: a Config built in code can carry a tiny edge; non-finite or huge points
// would overflow the int64 voxel index. No undefined cast: tiny edge = unchanged, bad points
// are dropped.
TEST(MapDownsample, TinyOrNanEdgeLeavesPointsUnchanged) {
  const std::vector<Eigen::Vector3f> in = {{0.1F, 0.2F, 0.3F}, {5.F, 6.F, 7.F}};
  for (const double v : {1.0e-20, 1.0e-4, std::nan("")}) {
    std::vector<Eigen::Vector3f> pts = in;
    voxel_centroids(pts, v);
    EXPECT_EQ(pts, in) << v;
  }
}

TEST(MapDownsample, NonFiniteAndHugePointsAreDropped) {
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<Eigen::Vector3f> pts = {{0.1F, 0.1F, 0.1F},
                                      {std::nanf(""), 0.F, 0.F},
                                      {inf, 0.F, 0.F},
                                      {1.0e30F, 0.F, 0.F},  // index 1e30 > int64
                                      {0.3F, 0.5F, 0.9F}};
  voxel_centroids(pts, 1.0);
  ASSERT_EQ(pts.size(), 1U);  // the two finite points share voxel (0,0,0)
  EXPECT_TRUE(pts[0].isApprox(Eigen::Vector3f(0.2F, 0.3F, 0.5F), 1e-6F)) << pts[0].transpose();
}

}  // namespace
}  // namespace mss::pipeline::detail
