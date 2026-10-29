// Unit tests of detail::voxel_centroids (core/pipeline/src/map_downsample.hpp), the
// map_export.voxel_m downsampling of Estimator::export_map.
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

}  // namespace
}  // namespace mss::pipeline::detail
