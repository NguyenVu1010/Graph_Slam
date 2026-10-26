#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/config.hpp"

#include "alloc_counter.hpp"
#include "map_reference_model.hpp"

namespace mss::map {
namespace {

using common::Pose;
using common::ScanFrame;
using common::Vec3;
using testing::default_map_config;
using testing::push_points;
using testing::RefMap;

constexpr double kPi = 3.14159265358979323846;

void construct(const MapConfig& c) {
  const VoxelMap m(c);
  (void)m.num_voxels();
}

ScanFrame make_scan(const std::vector<Vec3>& pts) {
  ScanFrame scan(pts.size());
  push_points(scan, pts);
  return scan;
}

std::vector<Vec3> uniform_points(std::mt19937& rng, std::size_t n, double lo, double hi) {
  std::uniform_real_distribution<double> u(lo, hi);
  std::vector<Vec3> pts;
  pts.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double x = u(rng);
    const double y = u(rng);
    const double z = u(rng);
    pts.emplace_back(x, y, z);
  }
  return pts;
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

// Checks VoxelMap::knn against RefMap::knn: count, dist2 sequence and point multiset.
void expect_knn_matches(const VoxelMap& m, const RefMap& r, const Vec3& q, int k) {
  NeighborBuffer buf;
  const int n = m.knn(q, k, buf);
  const std::vector<testing::RefNeighbor> ref = r.knn(q, k);
  ASSERT_EQ(n, static_cast<int>(ref.size())) << "k " << k;
  ASSERT_EQ(buf.size, n);
  using P = std::array<float, 3>;
  std::vector<P> got;
  std::vector<P> want;
  for (int i = 0; i < n; ++i) {
    const std::size_t u = static_cast<std::size_t>(i);
    EXPECT_FLOAT_EQ(buf.dist2[u], ref[u].dist2) << "i " << i;
    if (i > 0) {
      EXPECT_LE(buf.dist2[u - 1], buf.dist2[u]);
    }
    got.push_back(P{static_cast<float>(buf.p_w[u].x()),
                    static_cast<float>(buf.p_w[u].y()),
                    static_cast<float>(buf.p_w[u].z())});
    want.push_back(ref[u].p);
  }
  std::sort(got.begin(), got.end());
  std::sort(want.begin(), want.end());
  EXPECT_EQ(got, want);
}

// True when one of the first `n` neighbours equals `p` (float coordinates).
bool contains(const NeighborBuffer& buf, int n, const Vec3& p) {
  for (int i = 0; i < n; ++i) {
    const Vec3& q = buf.p_w[static_cast<std::size_t>(i)];
    if (static_cast<float>(q.x()) == static_cast<float>(p.x()) &&
        static_cast<float>(q.y()) == static_cast<float>(p.y()) &&
        static_cast<float>(q.z()) == static_cast<float>(p.z())) {
      return true;
    }
  }
  return false;
}

TEST(VoxelMap, ConstructorRejectsBadConfig) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double v : {0.0, -0.5, nan, inf}) {
    MapConfig c = default_map_config();
    c.voxel_m = v;
    EXPECT_THROW(construct(c), std::invalid_argument) << "voxel_m " << v;
  }
  {
    MapConfig c = default_map_config();
    c.max_points_per_voxel = 0;
    EXPECT_THROW(construct(c), std::invalid_argument);
  }
  for (const int k : {0, 33}) {
    MapConfig c = default_map_config();
    c.knn_k = k;
    EXPECT_THROW(construct(c), std::invalid_argument) << "knn_k " << k;
  }
  for (const std::size_t n : {std::size_t{0}, (std::size_t{1} << 30) + 1}) {
    MapConfig c = default_map_config();
    c.max_voxels = n;
    EXPECT_THROW(construct(c), std::invalid_argument) << "max_voxels " << n;
  }
  for (const double r : {0.0, nan}) {
    MapConfig c = default_map_config();
    c.prune_radius_m = r;
    EXPECT_THROW(construct(c), std::invalid_argument) << "prune_radius_m " << r;
  }
  EXPECT_NO_THROW(construct(default_map_config()));
  MapConfig c32 = default_map_config();
  c32.knn_k = NeighborBuffer::kMaxNeighbors;
  EXPECT_NO_THROW(construct(c32));
}

TEST(VoxelMap, DefaultYamlConfigConstructs) {
  const common::Result<pipeline::Config> cfg =
      pipeline::load_config(std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml");
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const VoxelMap m(cfg.value().map);
  EXPECT_EQ(m.num_voxels(), 0u);
  EXPECT_EQ(m.num_points(), 0u);
  EXPECT_EQ(m.num_dropped(), 0u);
}

TEST(VoxelMap, InsertCountsVoxelsAndPoints) {
  VoxelMap m(default_map_config());
  const ScanFrame scan = make_scan(
      {Vec3(0.1, 0.1, 0.1), Vec3(0.2, 0.2, 0.2), Vec3(0.6, 0.1, 0.1), Vec3(-0.1, 0.1, 0.1)});
  m.insert(scan, Pose{});
  EXPECT_EQ(m.num_voxels(), 3u);
  EXPECT_EQ(m.num_points(), 4u);
  EXPECT_EQ(m.num_dropped(), 0u);
}

TEST(VoxelMap, M02_RingKeepsAtMostNPointsPerVoxel) {
  VoxelMap m(default_map_config());
  std::vector<Vec3> pts;
  for (int i = 0; i < 50; ++i) {
    pts.emplace_back(0.005 + 0.009 * i, 0.25, 0.25);
  }
  m.insert(make_scan(pts), Pose{});
  EXPECT_EQ(m.num_points(), 20u);
  EXPECT_EQ(m.num_voxels(), 1u);
}

TEST(VoxelMap, SkipsNonFiniteAndOutOfRangePoints) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<Vec3> pts;
  for (const double bad : {nan, inf, -inf}) {
    pts.emplace_back(bad, 0.1, 0.1);
    pts.emplace_back(0.1, bad, 0.1);
    pts.emplace_back(0.1, 0.1, bad);
  }
  pts.emplace_back(3.0e9, 0.1, 0.1);
  VoxelMap m(default_map_config());
  m.insert(make_scan(pts), Pose{});
  EXPECT_EQ(m.num_points(), 0u);
  EXPECT_EQ(m.num_voxels(), 0u);
  EXPECT_EQ(m.num_dropped(), 0u);
}

TEST(VoxelMap, M02_TableFullDropsNewVoxelsOnly) {
  MapConfig c = default_map_config();
  c.max_voxels = 4;
  VoxelMap m(c);
  std::vector<Vec3> pts;
  for (int i = 0; i < 6; ++i) {
    pts.emplace_back(0.25 + i, 0.25, 0.25);
  }
  m.insert(make_scan(pts), Pose{});
  EXPECT_EQ(m.num_voxels(), 4u);
  EXPECT_EQ(m.num_points(), 4u);
  EXPECT_EQ(m.num_dropped(), 2u);
  m.insert(make_scan({Vec3(0.1, 0.1, 0.1), Vec3(0.2, 0.2, 0.2), Vec3(0.3, 0.3, 0.3)}), Pose{});
  EXPECT_EQ(m.num_voxels(), 4u);
  EXPECT_EQ(m.num_points(), 7u);
  EXPECT_EQ(m.num_dropped(), 2u);
}

TEST(VoxelMap, InsertMatchesReferenceCounts) {
  std::mt19937 rng(1);
  MapConfig c = default_map_config();
  c.max_voxels = 3000;
  VoxelMap m(c);
  RefMap r(c);
  std::uniform_real_distribution<double> yaw(-kPi, kPi);
  std::uniform_real_distribution<double> pos(-2.0, 2.0);
  for (int s = 0; s < 5; ++s) {
    const ScanFrame scan = make_scan(uniform_points(rng, 2000, -5.0, 5.0));
    const double a = yaw(rng);
    const double x = pos(rng);
    const double y = pos(rng);
    const double z = pos(rng);
    const Pose T = yaw_pose(a, Vec3(x, y, z));
    m.insert(scan, T);
    r.insert(scan, T);
    EXPECT_EQ(m.num_voxels(), r.num_voxels()) << "scan " << s;
    EXPECT_EQ(m.num_points(), r.num_points()) << "scan " << s;
    EXPECT_EQ(m.num_dropped(), r.num_dropped()) << "scan " << s;
  }
  EXPECT_GT(m.num_dropped(), 0u);
}

TEST(VoxelMap, InsertDoesNotAllocate) {
  std::mt19937 rng(4);
  const ScanFrame scan = make_scan(uniform_points(rng, 10000, -20.0, 20.0));
  const Pose T = yaw_pose(0.3, Vec3(1.0, 2.0, 0.5));
  MapConfig small = default_map_config();
  small.max_voxels = 4;
  for (const MapConfig& c : {default_map_config(), small}) {
    VoxelMap m(c);
    m.insert(scan, T);
    AllocCounter::reset();
    {
      EigenMallocGuard g;
      for (int i = 0; i < 10; ++i) {
        m.insert(scan, T);
      }
    }
    EXPECT_EQ(AllocCounter::count(), 0u) << "max_voxels " << c.max_voxels;
  }
}

TEST(VoxelMap, M01_KnnMatchesBruteForceSmall) {
  std::mt19937 rng(2);
  const MapConfig c = default_map_config();
  VoxelMap m(c);
  RefMap r(c);
  const ScanFrame scan = make_scan(uniform_points(rng, 5000, -3.0, 3.0));
  m.insert(scan, Pose{});
  r.insert(scan, Pose{});
  const std::vector<Vec3> queries = uniform_points(rng, 200, -3.5, 3.5);
  for (const Vec3& q : queries) {
    for (const int k : {0, 1, 5, 32, 100}) {
      expect_knn_matches(m, r, q, k);
    }
  }
}

TEST(VoxelMap, KnnNeverLeavesThe27Voxels) {
  const Vec3 q(0.1, 0.1, 0.1);  // key x = 0
  NeighborBuffer buf;
  {
    VoxelMap m(default_map_config());
    m.insert(make_scan({Vec3(1.6, 0.1, 0.1)}), Pose{});  // key x = 3, the only point
    EXPECT_EQ(m.knn(q, 32, buf), 0);
    EXPECT_EQ(buf.size, 0);
  }
  {
    VoxelMap m(default_map_config());
    m.insert(make_scan({Vec3(1.4, 0.1, 0.1)}), Pose{});  // key x = 2
    EXPECT_EQ(m.knn(q, 32, buf), 0);
  }
  VoxelMap m(default_map_config());
  const Vec3 a(0.9, 0.1, 0.1);  // key x = 1
  const Vec3 b(0.5, 0.1, 0.1);  // exactly on the boundary: key x = 1
  m.insert(make_scan({a, b}), Pose{});
  int n = m.knn(q, 32, buf);
  EXPECT_EQ(n, 2);
  EXPECT_TRUE(contains(buf, n, a));
  EXPECT_TRUE(contains(buf, n, b));
  n = m.knn(Vec3(1.1, 0.1, 0.1), 32, buf);  // key x = 2
  EXPECT_TRUE(contains(buf, n, b));
}

TEST(VoxelMap, KnnTieBreakFollowsVisitOrder) {
  // All coordinates are exact in binary, so both distances are exactly 0.5 m.
  const Vec3 q(0.25, 0.25, 0.25);
  const Vec3 below(0.25, 0.25, -0.25);  // dz = -1
  const Vec3 above(0.25, 0.25, 0.75);   // dz = +1
  NeighborBuffer buf;
  {
    VoxelMap m(default_map_config());
    m.insert(make_scan({above, below}), Pose{});
    ASSERT_EQ(m.knn(q, 1, buf), 1);
    EXPECT_TRUE(contains(buf, 1, below));
  }
  // Same voxel, distance 0.125 m each: the lower slot (inserted first) wins.
  const Vec3 first(0.375, 0.25, 0.25);
  const Vec3 second(0.125, 0.25, 0.25);
  VoxelMap m(default_map_config());
  m.insert(make_scan({first, second}), Pose{});
  ASSERT_EQ(m.knn(q, 1, buf), 1);
  EXPECT_TRUE(contains(buf, 1, first));
}

TEST(VoxelMap, M02_RingKeepsLastNPoints) {
  VoxelMap m(default_map_config());
  std::vector<Vec3> pts;
  for (int i = 0; i < 50; ++i) {
    pts.emplace_back(0.005 + 0.009 * i, 0.25, 0.25);
  }
  m.insert(make_scan(pts), Pose{});
  NeighborBuffer buf;
  const int n = m.knn(Vec3(0.25, 0.25, 0.25), 32, buf);
  ASSERT_EQ(n, 20);
  for (int i = 30; i < 50; ++i) {
    EXPECT_TRUE(contains(buf, n, pts[static_cast<std::size_t>(i)])) << "point " << i;
  }
}

TEST(VoxelMap, KnnNonFiniteOrOutOfRangeQueryReturnsZero) {
  VoxelMap m(default_map_config());
  m.insert(make_scan({Vec3(0.1, 0.1, 0.1)}), Pose{});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const Vec3& q :
       {Vec3(nan, 0.0, 0.0), Vec3(0.0, inf, 0.0), Vec3(0.0, 0.0, -inf), Vec3(3.0e9, 0.0, 0.0)}) {
    NeighborBuffer buf;
    buf.size = 7;
    EXPECT_EQ(m.knn(q, 0, buf), 0);
    EXPECT_EQ(buf.size, 0);
  }
}

TEST(VoxelMap, KnnEmptyMapReturnsZero) {
  const VoxelMap m(default_map_config());
  NeighborBuffer buf;
  buf.size = 3;
  EXPECT_EQ(m.knn(Vec3(0.1, 0.2, 0.3), 0, buf), 0);
  EXPECT_EQ(buf.size, 0);
}

TEST(VoxelMap, InsertAppliesPose) {
  VoxelMap m(default_map_config());
  const Pose T = yaw_pose(kPi / 2.0, Vec3(10.0, 0.0, 0.0));
  m.insert(make_scan({Vec3(1.0, 0.0, 0.0)}), T);
  NeighborBuffer buf;
  ASSERT_EQ(m.knn(Vec3(10.0, 1.0, 0.0), 1, buf), 1);
  EXPECT_LT(buf.dist2[0], 1e-10F);
  EXPECT_LT((buf.p_w[0] - Vec3(10.0, 1.0, 0.0)).norm(), 1e-6);
}

TEST(VoxelMap, KnnDoesNotAllocate) {
  std::mt19937 rng(5);
  VoxelMap m(default_map_config());
  for (int s = 0; s < 10; ++s) {
    const ScanFrame scan = make_scan(uniform_points(rng, 10000, -10.0, 10.0));
    m.insert(scan, Pose{});
  }
  const std::vector<Vec3> queries = uniform_points(rng, 10000, -10.0, 10.0);
  NeighborBuffer buf;
  long hits = 0;
  AllocCounter::reset();
  {
    EigenMallocGuard g;
    for (const Vec3& q : queries) {
      hits += m.knn(q, 0, buf);
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_GT(hits, 0);
}

}  // namespace
}  // namespace mss::map
