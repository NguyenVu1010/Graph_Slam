// Component tests of mss::map::VoxelMap on synthetic data (docs/Test_CI_CD.md M-01..M-03).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"

#include "../unit/map_reference_model.hpp"

namespace mss::map {
namespace {

using common::Pose;
using common::ScanFrame;
using common::Vec3;
using testing::default_map_config;
using testing::push_points;
using testing::RefMap;

constexpr double kPi = 3.14159265358979323846;

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

// `n` points uniform in a horizontal disc of radius `radius_m` around the origin, z uniform
// in [z_lo_m, z_hi_m].
std::vector<Vec3> disc_points(
    std::mt19937& rng, std::size_t n, double radius_m, double z_lo_m, double z_hi_m) {
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  std::uniform_real_distribution<double> uz(z_lo_m, z_hi_m);
  std::vector<Vec3> pts;
  pts.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double r = radius_m * std::sqrt(u01(rng));
    const double a = ang(rng);
    const double z = uz(rng);
    pts.emplace_back(r * std::cos(a), r * std::sin(a), z);
  }
  return pts;
}

void fill_scan(ScanFrame& scan, const std::vector<Vec3>& pts) {
  scan.clear();
  push_points(scan, pts);
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

// Nearest-rank percentile, p in [0, 100]. Sorts `v`, which must not be empty.
double percentile(std::vector<double>& v, double p) {
  std::sort(v.begin(), v.end());
  const double rank = std::ceil(p / 100.0 * static_cast<double>(v.size()));
  const std::size_t idx = rank < 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
  return v[std::min(v.size() - 1, idx)];
}

// Test_CI_CD M-01: 100000 points, 1000 queries, equal to brute force over the 27 voxels.
TEST(VoxelMapComponent, M01_KnnMatchesBruteForce100k) {
  std::mt19937 rng(11);
  const MapConfig c = default_map_config();
  VoxelMap m(c);
  RefMap r(c);
  ScanFrame scan(10000);
  for (int s = 0; s < 10; ++s) {
    fill_scan(scan, uniform_points(rng, 10000, -10.0, 10.0));
    m.insert(scan, Pose{});
    r.insert(scan, Pose{});
  }
  ASSERT_EQ(m.num_points(), r.num_points());
  const std::vector<Vec3> queries = uniform_points(rng, 1000, -10.5, 10.5);
  for (const Vec3& q : queries) {
    expect_knn_matches(m, r, q, 0);
    expect_knn_matches(m, r, q, 32);
  }
}

// Test_CI_CD M-02: <= N points per voxel, voxels outside the radius are removed, and the
// voxel count does not grow with the distance driven.
TEST(VoxelMapComponent, M02_MemoryBoundedOnLongDrive) {
  std::mt19937 rng(12);
  const MapConfig c = default_map_config();
  VoxelMap m(c);
  RefMap r(c);
  constexpr int kScans = 1000;     // 10 m/s at 10 Hz: 1 m per scan, 1 km in total
  constexpr int kPruneEvery = 50;  // prune_period_s 5.0 / 0.1 s scan period
  constexpr double kDiscRadiusM = 20.0;
  constexpr double kBehindM = 150.0;  // query this far behind the robot, beyond the radius
  ScanFrame scan(2000);
  std::size_t voxels_after_150 = 0;
  for (int s = 1; s <= kScans; ++s) {
    const double x_robot = static_cast<double>(s);
    fill_scan(scan, disc_points(rng, 2000, kDiscRadiusM, 0.0, 2.0));
    Pose T_w_l;
    T_w_l.p = Vec3(x_robot, 0.0, 0.0);
    m.insert(scan, T_w_l);
    r.insert(scan, T_w_l);
    ASSERT_LE(m.num_points(), m.num_voxels() * static_cast<std::size_t>(c.max_points_per_voxel));
    ASSERT_EQ(m.num_dropped(), 0u);
    if (s % kPruneEvery == 0) {
      const Vec3 p_w_b(x_robot, 0.0, 0.0);
      m.prune(p_w_b);
      r.prune(p_w_b);
      ASSERT_EQ(m.num_voxels(), r.num_voxels()) << "scan " << s;
      ASSERT_EQ(m.num_points(), r.num_points()) << "scan " << s;
      NeighborBuffer buf;
      EXPECT_EQ(m.knn(Vec3(x_robot - kBehindM, 0.0, 1.0), 0, buf), 0) << "scan " << s;
    }
    if (s == 150) {
      voxels_after_150 = m.num_voxels();
    }
  }
  ASSERT_GT(voxels_after_150, 0u);
  EXPECT_LE(static_cast<double>(m.num_voxels()), 1.1 * static_cast<double>(voxels_after_150));
}

// Test_CI_CD M-03: insert and kNN timings, recorded only; never fails on time.
TEST(VoxelMapComponent, M03_InsertAndKnnTimingRecorded) {
  using Clock = std::chrono::steady_clock;
  std::mt19937 rng(13);
  VoxelMap m(default_map_config());
  constexpr int kScans = 100;
  constexpr std::size_t kScanPoints = 10000;  // ADR-007 scan size
  constexpr std::size_t kQueries = 10000;
  constexpr double kRangeM = 30.0;
  std::vector<std::vector<Vec3>> scans;
  scans.reserve(kScans);
  for (int s = 0; s < kScans; ++s) {
    scans.push_back(disc_points(rng, kScanPoints, kRangeM, -1.0, 3.0));
  }
  std::uniform_real_distribution<double> along(0.0, static_cast<double>(kScans));
  std::vector<Vec3> queries = disc_points(rng, kQueries, kRangeM, -1.0, 3.0);
  for (Vec3& q : queries) {
    q.x() += along(rng);
  }
  std::vector<double> insert_us;
  insert_us.reserve(kScans);
  std::vector<double> knn_ns;
  knn_ns.reserve(kQueries);

  ScanFrame scan(kScanPoints);
  for (int s = 0; s < kScans; ++s) {
    fill_scan(scan, scans[static_cast<std::size_t>(s)]);
    Pose T_w_l;
    T_w_l.p = Vec3(static_cast<double>(s), 0.0, 0.0);
    const Clock::time_point t0 = Clock::now();
    m.insert(scan, T_w_l);
    const Clock::time_point t1 = Clock::now();
    insert_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  NeighborBuffer buf;
  long hits = 0;
  for (const Vec3& q : queries) {
    const Clock::time_point t0 = Clock::now();
    hits += m.knn(q, 0, buf);
    const Clock::time_point t1 = Clock::now();
    knn_ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
  }
  const double ins_p50 = percentile(insert_us, 50.0);
  const double ins_p95 = percentile(insert_us, 95.0);
  const double knn_p50 = percentile(knn_ns, 50.0);
  const double knn_p95 = percentile(knn_ns, 95.0);
  std::printf(
      "M-03 insert_scan_us p50=%.1f p95=%.1f | knn_query_ns p50=%.0f p95=%.0f | voxels=%zu "
      "points=%zu\n",
      ins_p50,
      ins_p95,
      knn_p50,
      knn_p95,
      m.num_voxels(),
      m.num_points());
  RecordProperty("insert_scan_p50_us", static_cast<int>(std::lround(ins_p50)));
  RecordProperty("insert_scan_p95_us", static_cast<int>(std::lround(ins_p95)));
  RecordProperty("knn_query_p50_ns", static_cast<int>(std::lround(knn_p50)));
  RecordProperty("knn_query_p95_ns", static_cast<int>(std::lround(knn_p95)));
  EXPECT_GT(hits, 0);
}

}  // namespace
}  // namespace mss::map
