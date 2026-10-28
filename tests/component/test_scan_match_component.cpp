// Component tests of mss::scan_match::ScanMatcher on synthetic scenes with a known pose
// (docs/Test_CI_CD.md S-02..S-05, D-03; docs/derivations/scan_match.md).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/degeneracy/degeneracy.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/preprocess/preprocessor.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "alloc_counter.hpp"
#include "box_world.hpp"
#include "scan_match_scenes.hpp"

namespace mss::scan_match {
namespace {

using common::Mat3;
using common::Mat6;
using common::MatchResult;
using common::Pose;
using common::ScanFrame;
using common::Vec3;
using common::Vec6;
using test::corridor_points_w;
using test::fill_scan;
using test::insert_points;
using test::line_points_w;
using test::perturb;
using test::pose_error;
using test::PoseError;
using test::room_points_w;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMapSpacingM = 0.125;
constexpr double kRoomScanSpacingM = 0.5;
constexpr double kDenseRoomScanSpacingM = 0.25;
const Pose kTruthRoom{common::so3_exp(Vec3(0.02, -0.03, 0.2)), Vec3(0.4, -0.3, 0.2)};

// Points per map voxel for the synthetic scenes: a 0.5 m voxel on a box edge or corner holds
// up to 3 faces x 16 points of the 0.125 m map grid. With the default 20 the voxel ring
// overwrites part of one face there, so 5-NN neighbourhoods mix two faces, pass the 0.05 m
// RMS gate and bias the known answer by about 1 cm (measured); 48 keeps every map point.
constexpr int kSceneMaxPointsPerVoxel = 48;

// config/default.yaml exactly as shipped.
pipeline::Config shipped_config() {
  const common::Result<pipeline::Config> cfg =
      pipeline::load_config(std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml");
  if (!cfg.is_ok()) {
    throw std::runtime_error("default config: " + cfg.reason());
  }
  return cfg.value();
}

// config/default.yaml with map.max_points_per_voxel = kSceneMaxPointsPerVoxel.
pipeline::Config default_config() {
  pipeline::Config c = shipped_config();
  c.map.max_points_per_voxel = kSceneMaxPointsPerVoxel;
  return c;
}

bool bitwise_equal(const MatchResult& a, const MatchResult& b) {
  return std::memcmp(a.T_w_l.R.data(), b.T_w_l.R.data(), sizeof(double) * 9) == 0 &&
         std::memcmp(a.T_w_l.p.data(), b.T_w_l.p.data(), sizeof(double) * 3) == 0 &&
         std::memcmp(a.H.data(), b.H.data(), sizeof(double) * 36) == 0 &&
         std::memcmp(a.cov.data(), b.cov.data(), sizeof(double) * 36) == 0 &&
         a.iterations == b.iterations && a.inliers == b.inliers && a.converged == b.converged;
}

// Room map (0.125 m grid) with the default config.
struct RoomFixture {
  pipeline::Config cfg = default_config();
  map::VoxelMap map{cfg.map};
  RoomFixture() { insert_points(map, room_points_w(kMapSpacingM)); }
};

// Scan of `p_w` seen from `T_w_l`.
void make_scan(ScanFrame& scan, const std::vector<Vec3>& p_w, const Pose& T_w_l) {
  scan.clear();
  fill_scan(scan, p_w, T_w_l);
}

// Test_CI_CD S-02: 6-sided room, initial offset <= 0.5 m and 10 deg: error < 1 cm and
// 0.1 deg within <= 10 iterations.
TEST(ScanMatchComponent, S02_KnownTransformRecovered) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  std::mt19937 rng(2);
  for (int trial = 0; trial < 10; ++trial) {
    const Pose init = perturb(kTruthRoom, 0.5, 10.0 * kDegToRad, rng);
    const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
    ASSERT_TRUE(r.is_ok()) << "trial " << trial << ": " << r.reason();
    const MatchResult& m = r.value();
    const PoseError e = pose_error(m.T_w_l, kTruthRoom);
    std::printf("S-02 trial %d: iters=%d inliers=%d err_m=%.2e err_deg=%.2e\n",
                trial,
                m.iterations,
                m.inliers,
                e.trans_m,
                e.rot_rad / kDegToRad);
    EXPECT_TRUE(m.converged) << "trial " << trial;
    EXPECT_LE(m.iterations, 10) << "trial " << trial;
    EXPECT_LT(e.trans_m, 0.01) << "trial " << trial;
    EXPECT_LT(e.rot_rad, 0.1 * kDegToRad) << "trial " << trial;
    EXPECT_GE(m.inliers, f.cfg.scan_match.min_inliers);
    EXPECT_TRUE(m.cov == m.cov.transpose());
    const Eigen::LLT<Mat6> llt(m.cov);
    EXPECT_EQ(llt.info(), Eigen::Success) << "trial " << trial;
  }
}

// Worst S-02 error over 10 trials on the 0.125 m room map stored with `cfg.map`, plus the
// smallest final inlier count. Same per-trial limits as S-02.
struct EdgeBias {
  double worst_m = 0.0;
  double worst_deg = 0.0;
  int min_inliers = std::numeric_limits<int>::max();
};

EdgeBias s02_edge_bias(const pipeline::Config& cfg) {
  map::VoxelMap m(cfg.map);
  insert_points(m, room_points_w(kMapSpacingM));
  ScanMatcher matcher(cfg.scan_match, cfg.degeneracy, cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  std::mt19937 rng(2);
  EdgeBias b;
  for (int trial = 0; trial < 10; ++trial) {
    const Pose init = perturb(kTruthRoom, 0.5, 10.0 * kDegToRad, rng);
    const common::Result<MatchResult> r = matcher.align(scan, m, init);
    EXPECT_TRUE(r.is_ok()) << "trial " << trial << ": " << r.reason();
    if (!r.is_ok()) {
      continue;
    }
    const MatchResult& res = r.value();
    const PoseError e = pose_error(res.T_w_l, kTruthRoom);
    b.worst_m = std::max(b.worst_m, e.trans_m);
    b.worst_deg = std::max(b.worst_deg, e.rot_rad / kDegToRad);
    b.min_inliers = std::min(b.min_inliers, res.inliers);
    EXPECT_TRUE(res.converged) << "trial " << trial;
    EXPECT_LE(res.iterations, 10) << "trial " << trial;
    EXPECT_LT(e.trans_m, 0.01) << "trial " << trial;
    EXPECT_LT(e.rot_rad, 0.1 * kDegToRad) << "trial " << trial;
  }
  std::printf(
      "S-02 map N=%d min_point_dist_m %.3f plane %.3f mid %.3f: worst err_m=%.2e err_deg=%.2e "
      "min inliers %d, map points %zu, spacing replaced %zu\n",
      cfg.map.max_points_per_voxel,
      cfg.map.min_point_dist_m,
      cfg.scan_match.plane_min_eig_ratio,
      cfg.scan_match.min_mid_eig_ratio,
      b.worst_m,
      b.worst_deg,
      b.min_inliers,
      m.num_points(),
      m.num_spacing_replaced());
  return b;
}

// The shipped config with spacing and both plane-fit gates off: the configuration before
// the ADR-007 amendment.
pipeline::Config ungated_config() {
  pipeline::Config c = shipped_config();
  c.map.min_point_dist_m = 0.0;
  c.scan_match.plane_min_eig_ratio = 0.0;
  c.scan_match.min_mid_eig_ratio = 0.0;
  return c;
}

// Without the gates (pins the weakness behind the ADR-007 amendment): in the voxels on the
// box edges at x = -4, y = -3 and z = -1 two faces share a voxel (32 points of the 0.125 m
// grid for 20 slots), the ring keeps the last 20 and 5-NN neighbourhoods become two parallel
// point columns on two faces (exactly coplanar), which pass plane_rms_max_m and bias the
// answer by about 9.5 mm / 0.08 deg. Same limits as S-02.
TEST(ScanMatchComponent, S02_UngatedEdgeBias) {
  const EdgeBias b = s02_edge_bias(ungated_config());
  EXPECT_GT(b.worst_m, 5e-3);
  RecordProperty("edge_bias_worst_um", static_cast<int>(std::lround(b.worst_m * 1e6)));
}

// S-02 at the SHIPPED config (spacing and both plane-fit gates off, the behaviour of main):
// the S-02 bounds (< 1 cm, < 0.1 deg) hold with the box-edge bias of the two-column
// neighbourhoods (9.5 mm / 0.083 deg measured; 7.2 mm / 0.092 deg with planarity 0.01).
// The next test shows the settings that remove it and why they are not shipped.
TEST(ScanMatchComponent, S02_DefaultMapConfigEdgeBias) {
  const pipeline::Config cfg = shipped_config();
  const EdgeBias b = s02_edge_bias(cfg);
  EXPECT_LT(b.worst_m, 0.01);
  EXPECT_LT(b.worst_deg, 0.1);
  EXPECT_GE(b.min_inliers, cfg.scan_match.min_inliers);
  RecordProperty("edge_bias_shipped_um", static_cast<int>(std::lround(b.worst_m * 1e6)));
}

// Stronger settings remove the edge bias, each from the ungated config:
// - map.min_point_dist_m = 0.1 m: the face point 8.8 cm from the other face's point across
//   each such edge replaces it and the ring keeps different points (0.38 mm / 0.016 deg);
// - scan_match.min_mid_eig_ratio = 0.2: the two-column neighbourhoods are long and thin
//   (lambda_mid / lambda_max < 0.2) and are rejected (0.28 mm / 0.017 deg).
// Neither is shipped (docs/derivations/scan_match.md section 3, map.md "Đánh giá"): spacing
// is a memory and speed option whose E-01 effect depends on the gates, and the collinearity
// gate rejects the thin wall strips of a 5 m wide corridor (LiDAR-only drift 11.5 -> 241 m
// with planarity 0.05) and fails the no-GTSAM LiDAR NEES bound (2.83 < 3) and
// CorridorDriftCorrectedByGnss. Eigenvalue-ratio gates also reject many genuinely planar
// neighbourhoods once the map carries range noise (scan_match.md section 3).
TEST(ScanMatchComponent, S02_StrongerSettingsRemoveEdgeBias) {
  pipeline::Config spacing = ungated_config();
  spacing.map.min_point_dist_m = 0.1;
  pipeline::Config collinearity = ungated_config();
  collinearity.scan_match.min_mid_eig_ratio = 0.2;
  for (const pipeline::Config& cfg : {spacing, collinearity}) {
    const EdgeBias b = s02_edge_bias(cfg);
    EXPECT_LT(b.worst_m, 1e-3);
    EXPECT_LT(b.worst_deg, 0.03);
    EXPECT_GE(b.min_inliers, cfg.scan_match.min_inliers);
  }
}

// Gate sweep (scan_match.md section 3 table): planarity ratio 0 / 0.01 / 0.05 / 0.1 / 0.2
// and collinearity ratio 0 / 0.01 / 0.05 / 0.1 / 0.2, each with the other gate and spacing
// off. Same per-trial limits as S-02 for every value; the inlier count stays above
// min_inliers.
TEST(ScanMatchComponent, S02_GateSweep) {
  for (const double ratio : {0.0, 0.01, 0.05, 0.1, 0.2}) {
    pipeline::Config pl = ungated_config();
    pl.scan_match.plane_min_eig_ratio = ratio;
    std::printf("plane_min_eig_ratio %.2f: ", ratio);
    EXPECT_GE(s02_edge_bias(pl).min_inliers, pl.scan_match.min_inliers) << ratio;
    pipeline::Config mid = ungated_config();
    mid.scan_match.min_mid_eig_ratio = ratio;
    std::printf("min_mid_eig_ratio %.2f: ", ratio);
    EXPECT_GE(s02_edge_bias(mid).min_inliers, mid.scan_match.min_inliers) << ratio;
  }
}

// Noise-free scan at the true pose: sigma_r^2 is ~0 (float rounding), so the covariance sits
// at the floor cov_scale * min_residual_sigma_m^2 * H^-1, not at zero.
TEST(ScanMatchComponent, NoiseFreeCovarianceAtFloor) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  const common::Result<MatchResult> r = matcher.align(scan, f.map, kTruthRoom);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const MatchResult& m = r.value();
  const double floor_m2 =
      f.cfg.scan_match.min_residual_sigma_m * f.cfg.scan_match.min_residual_sigma_m;
  const double c = f.cfg.scan_match.cov_scale * floor_m2;
  const Mat6 cov_H = m.cov * m.H;
  EXPECT_LT((cov_H - c * Mat6::Identity()).norm(), 1e-6 * c);
  const double pos_trace_m2 = m.cov.topLeftCorner<3, 3>().trace();
  std::printf("noise-free cov position trace %.3e m^2 (floor sigma %.3f m)\n",
              pos_trace_m2,
              f.cfg.scan_match.min_residual_sigma_m);
  EXPECT_GT(pos_trace_m2, 0.0);
  const Eigen::LLT<Mat6> llt(m.cov);
  EXPECT_EQ(llt.info(), Eigen::Success);
}

// cov = cov_scale * sigma_r^2 * H^-1: 3x the scan noise gives about 9x the position variance.
TEST(ScanMatchComponent, CovarianceScalesWithNoise) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> clean_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(clean_w.size());
  const auto position_variance = [&](double sigma_m) {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, sigma_m);
    std::vector<Vec3> noisy_w = clean_w;
    for (Vec3& p : noisy_w) {
      const double dx = noise(rng);
      const double dy = noise(rng);
      const double dz = noise(rng);
      p += Vec3(dx, dy, dz);
    }
    make_scan(scan, noisy_w, kTruthRoom);
    const common::Result<MatchResult> r = matcher.align(scan, f.map, kTruthRoom);
    EXPECT_TRUE(r.is_ok()) << r.reason();
    return r.is_ok() ? r.value().cov.topLeftCorner<3, 3>().trace() : 0.0;
  };
  const double var_1cm = position_variance(0.01);
  const double var_3cm = position_variance(0.03);
  const double ratio = var_3cm / var_1cm;
  std::printf(
      "cov position trace: 1 cm %.3e m^2, 3 cm %.3e m^2, ratio %.2f\n", var_1cm, var_3cm, ratio);
  EXPECT_GT(var_1cm, 0.0);
  EXPECT_GT(ratio, 7.0);
  EXPECT_LT(ratio, 11.0);
}

TEST(ScanMatchComponent, NoAllocationInAlign) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  std::mt19937 rng(2);
  const Pose init = perturb(kTruthRoom, 0.3, 5.0 * kDegToRad, rng);
  ASSERT_TRUE(matcher.align(scan, f.map, init).is_ok());  // warm-up
  bool ok = false;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
    ok = r.is_ok();
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(ok);
  const ScanFrame empty_scan(4);
  bool err = false;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    const common::Result<MatchResult> r = matcher.align(empty_scan, f.map, init);
    err = !r.is_ok();
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(err);
}

TEST(ScanMatchComponent, DeterministicBitIdentical) {
  RoomFixture f;
  const std::size_t max_points = f.cfg.realtime.max_points_per_scan;
  ScanMatcher a(f.cfg.scan_match, f.cfg.degeneracy, max_points);
  ScanMatcher b(f.cfg.scan_match, f.cfg.degeneracy, max_points);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  std::mt19937 rng(2);
  const Pose init = perturb(kTruthRoom, 0.5, 10.0 * kDegToRad, rng);
  const common::Result<MatchResult> ra = a.align(scan, f.map, init);
  const common::Result<MatchResult> rb = b.align(scan, f.map, init);
  ASSERT_TRUE(ra.is_ok() && rb.is_ok());
  EXPECT_TRUE(bitwise_equal(ra.value(), rb.value()));
}

TEST(ScanMatchComponent, ReuseMatchesFreshMatcher) {
  RoomFixture f;
  const std::size_t max_points = f.cfg.realtime.max_points_per_scan;
  ScanMatcher a(f.cfg.scan_match, f.cfg.degeneracy, max_points);
  ScanMatcher b(f.cfg.scan_match, f.cfg.degeneracy, max_points);
  const std::vector<Vec3> big_w = room_points_w(kMapSpacingM);
  const std::vector<Vec3> small_w = room_points_w(kRoomScanSpacingM);
  ScanFrame big(big_w.size());
  make_scan(big, big_w, kTruthRoom);
  ScanFrame small(small_w.size());
  make_scan(small, small_w, kTruthRoom);
  std::mt19937 rng(2);
  const Pose init = perturb(kTruthRoom, 0.3, 5.0 * kDegToRad, rng);
  ASSERT_TRUE(a.align(big, f.map, init).is_ok());
  const common::Result<MatchResult> ra = a.align(small, f.map, init);
  const common::Result<MatchResult> rb = b.align(small, f.map, init);
  ASSERT_TRUE(ra.is_ok() && rb.is_ok());
  EXPECT_TRUE(bitwise_equal(ra.value(), rb.value()));
}

TEST(ScanMatchComponent, NonFinitePointsSkipped) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame clean(scan_w.size());
  make_scan(clean, scan_w, kTruthRoom);
  // Same points with NaN at index 0, +Inf in the middle and -Inf at the end.
  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
  constexpr float kInf = std::numeric_limits<float>::infinity();
  ScanFrame dirty(scan_w.size() + 3);
  const common::Span<const float> xyz = clean.xyz();
  dirty.push(kNan, 0.0F, 0.0F, 0.0F, 0U, 0U);
  const std::size_t half = clean.size() / 2;
  for (std::size_t i = 0; i < clean.size(); ++i) {
    if (i == half) {
      dirty.push(1.0F, kInf, 0.0F, 0.0F, 0U, 0U);
    }
    dirty.push(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2], 0.0F, 0U, 0U);
  }
  dirty.push(0.0F, 0.0F, -kInf, 0.0F, 0U, 0U);
  ASSERT_EQ(dirty.size(), clean.size() + 3);
  std::mt19937 rng(2);
  const Pose init = perturb(kTruthRoom, 0.3, 5.0 * kDegToRad, rng);
  const common::Result<MatchResult> rc = matcher.align(clean, f.map, init);
  const common::Result<MatchResult> rd = matcher.align(dirty, f.map, init);
  ASSERT_TRUE(rc.is_ok() && rd.is_ok());
  EXPECT_EQ(rc.value().inliers, rd.value().inliers);
  EXPECT_TRUE(bitwise_equal(rc.value(), rd.value()));
}

// Test_CI_CD S-03: convergence basin over initial offsets 0..1 m x 0..30 deg; >= 95 % at
// 0.3 m and 5 deg; CSV for the plot under <build-dir>/test_output/scan_match/.
TEST(ScanMatchComponent, S03_ConvergenceBasin) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  const std::vector<double> trans_offsets_m = {0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0};
  const std::vector<double> rot_offsets_deg = {0.0, 5.0, 10.0, 15.0, 20.0, 30.0};
  constexpr int kTrials = 20;
  const std::filesystem::path csv_path =
      std::filesystem::path{MSS_TEST_OUTPUT_DIR} / "scan_match" / "s03_convergence_basin.csv";
  std::filesystem::create_directories(csv_path.parent_path());
  std::ofstream csv(csv_path);
  ASSERT_TRUE(csv.is_open()) << csv_path;
  csv << "trans_offset_m,rot_offset_deg,trials,successes,success_rate,mean_iterations\n";
  std::mt19937 rng(3);
  double rate_at_03m_5deg = -1.0;
  for (std::size_t ti = 0; ti < trans_offsets_m.size(); ++ti) {
    for (std::size_t ri = 0; ri < rot_offsets_deg.size(); ++ri) {
      const double t_m = trans_offsets_m[ti];
      const double r_deg = rot_offsets_deg[ri];
      int successes = 0;
      int iterations = 0;
      for (int trial = 0; trial < kTrials; ++trial) {
        const Pose init = perturb(kTruthRoom, t_m, r_deg * kDegToRad, rng);
        const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
        if (!r.is_ok()) {
          continue;
        }
        iterations += r.value().iterations;
        const PoseError e = pose_error(r.value().T_w_l, kTruthRoom);
        if (e.trans_m < 0.01 && e.rot_rad < 0.1 * kDegToRad) {
          ++successes;
        }
      }
      const double rate = static_cast<double>(successes) / kTrials;
      csv << t_m << ',' << r_deg << ',' << kTrials << ',' << successes << ',' << rate << ','
          << static_cast<double>(iterations) / kTrials << '\n';
      if (ti == 3 && ri == 1) {  // 0.3 m, 5 deg
        rate_at_03m_5deg = rate;
      }
    }
  }
  csv.close();
  std::printf("S-03 CSV: %s (success rate at 0.3 m, 5 deg: %.2f)\n",
              csv_path.string().c_str(),
              rate_at_03m_5deg);
  RecordProperty("s03_csv", csv_path.string());
  EXPECT_GE(rate_at_03m_5deg, 0.95);
}

// Test_CI_CD S-04: scan noise N(0, 2 cm) per axis -> position error < 3 cm.
TEST(ScanMatchComponent, S04_GaussianNoise) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> clean_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(clean_w.size());
  std::mt19937 rng(4);
  std::normal_distribution<double> noise(0.0, 0.02);
  for (int trial = 0; trial < 10; ++trial) {
    std::vector<Vec3> noisy_w = clean_w;
    for (Vec3& p : noisy_w) {
      const double dx = noise(rng);
      const double dy = noise(rng);
      const double dz = noise(rng);
      p += Vec3(dx, dy, dz);
    }
    make_scan(scan, noisy_w, kTruthRoom);
    const Pose init = perturb(kTruthRoom, 0.3, 5.0 * kDegToRad, rng);
    const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
    ASSERT_TRUE(r.is_ok()) << "trial " << trial << ": " << r.reason();
    const PoseError e = pose_error(r.value().T_w_l, kTruthRoom);
    std::printf(
        "S-04 trial %d: err_m=%.2e err_deg=%.2e\n", trial, e.trans_m, e.rot_rad / kDegToRad);
    EXPECT_LT(e.trans_m, 0.03) << "trial " << trial;
  }
}

// Test_CI_CD S-05: 20 % of the scan replaced by uniform outliers in the room box -> the
// Huber kernel keeps the position error < 5 cm.
TEST(ScanMatchComponent, S05_OutliersRobustKernel) {
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> clean_w = room_points_w(kRoomScanSpacingM);
  ScanFrame scan(clean_w.size());
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> ux(-4.0, 4.0);
  std::uniform_real_distribution<double> uy(-3.0, 3.0);
  std::uniform_real_distribution<double> uz(-1.0, 2.0);
  const std::size_t n_outliers = clean_w.size() / 5;
  for (int trial = 0; trial < 10; ++trial) {
    std::vector<Vec3> dirty_w = clean_w;
    std::vector<std::size_t> idx(dirty_w.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
      idx[i] = i;
    }
    std::shuffle(idx.begin(), idx.end(), rng);
    for (std::size_t k = 0; k < n_outliers; ++k) {
      const double x = ux(rng);
      const double y = uy(rng);
      const double z = uz(rng);
      dirty_w[idx[k]] = Vec3(x, y, z);
    }
    make_scan(scan, dirty_w, kTruthRoom);
    const Pose init = perturb(kTruthRoom, 0.3, 5.0 * kDegToRad, rng);
    const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
    ASSERT_TRUE(r.is_ok()) << "trial " << trial << ": " << r.reason();
    const PoseError e = pose_error(r.value().T_w_l, kTruthRoom);
    std::printf("S-05 trial %d: err_m=%.2e err_deg=%.2e inliers=%d\n",
                trial,
                e.trans_m,
                e.rot_rad / kDegToRad,
                r.value().inliers);
    EXPECT_LT(e.trans_m, 0.05) << "trial " << trial;
  }
}

// Corridor along x (map 0.125 m, scan 0.25 m), truth = (0, 0.1, 0), init 0.3 m along the
// unobservable axis plus 3 deg of yaw.
struct CorridorFixture {
  pipeline::Config cfg = default_config();
  map::VoxelMap map{cfg.map};
  Pose truth{Mat3::Identity(), Vec3(0.0, 0.1, 0.0)};
  Pose init{common::so3_exp(Vec3(0.0, 0.0, 3.0 * kDegToRad)), Vec3(0.3, 0.2, 0.0)};
  std::vector<Vec3> scan_w = corridor_points_w(0.25, 6.0);
  ScanFrame scan{scan_w.size()};
  CorridorFixture() {
    insert_points(map, corridor_points_w(kMapSpacingM, 10.0));
    make_scan(scan, scan_w, truth);
    cfg.degeneracy.enabled = true;
  }
};

// Test_CI_CD D-03 inside the matcher: an exactly degenerate corridor (lambda = 0 along x) is
// solved (not "singular Hessian"), the observable DoF are recovered, the estimate does not
// move along the axis, and the covariance there is "unobserved": exactly unobserved_var_m2
// (known answer, derivation section 6), decoupled, observed directions unchanged.
TEST(ScanMatchComponent, D03_CorridorProjectedNotSingular) {
  CorridorFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const common::Result<MatchResult> r = matcher.align(f.scan, f.map, f.init);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const MatchResult& m = r.value();
  const PoseError e = pose_error(m.T_w_l, f.truth);
  std::printf("D-03 corridor: iters=%d inliers=%d p=(%.4f, %.4f, %.4f) rot_err_deg=%.2e\n",
              m.iterations,
              m.inliers,
              m.T_w_l.p.x(),
              m.T_w_l.p.y(),
              m.T_w_l.p.z(),
              e.rot_rad / kDegToRad);
  EXPECT_LT(std::abs(m.T_w_l.p.y() - f.truth.p.y()), 0.01);
  EXPECT_LT(std::abs(m.T_w_l.p.z() - f.truth.p.z()), 0.01);
  EXPECT_LT(e.rot_rad, 0.1 * kDegToRad);
  EXPECT_LT(std::abs(m.T_w_l.p.x() - f.init.p.x()), 0.01);

  const common::DegeneracyReport report = degeneracy::analyze(m.H, f.cfg.degeneracy);
  ASSERT_TRUE(report.degenerate[0]);
  const Vec6 v0 = report.eigvecs.col(0);
  const Vec3 axis_l = m.T_w_l.R.transpose() * Vec3::UnitX();
  EXPECT_GT(std::abs(v0.head<3>().dot(axis_l)), std::cos(5.0 * kDegToRad));

  // cov = P (c A^-1) P + sum_k flagged (v_k^T D v_k) v_k v_k^T, c = cov_scale * s2 > 0, P the
  // projector onto the unflagged directions. On the observed subspace P cov P = c (Q^T H Q)^-1
  // (Q an orthonormal basis of it), so P cov H P = c P (1e-9 relative); the flagged
  // direction is decoupled; along the corridor axis the variance is unobserved_var_m2.
  const double var_m2 = f.cfg.scan_match.unobserved_var_m2;
  Mat6 proj = Mat6::Identity();
  for (int k = 0; k < 6; ++k) {
    if (report.degenerate[static_cast<std::size_t>(k)]) {
      const Vec6 v_k = report.eigvecs.col(k);
      proj -= v_k * v_k.transpose();
    }
  }
  const Mat6 m_obs = proj * m.cov * m.H * proj;
  const double c = m_obs.trace() / proj.trace();
  ASSERT_GT(c, 0.0);
  EXPECT_LT((m_obs - c * proj).norm(), 1e-9 * c);
  const Mat6 cov_obs = proj * m.cov * proj;
  EXPECT_LT((cov_obs * v0).norm(), 1e-12 * var_m2);
  EXPECT_LT((m.cov * v0 - v0.dot(m.cov * v0) * v0).norm(), 1e-12 * var_m2);
  const Eigen::SelfAdjointEigenSolver<Mat6> es_obs(cov_obs);
  const double max_unflagged_var = es_obs.eigenvalues()(5);
  // Known answer: v0^T D v0 with D = diag(var_m2 I3, var_rad2 I3); v0 is the corridor axis up
  // to a small rotation part from the fit, so this is var_m2 to about 1e-7.
  const double want = var_m2 * v0.head<3>().squaredNorm() +
                      f.cfg.scan_match.unobserved_var_rad2 * v0.tail<3>().squaredNorm();
  const double var_axis = v0.dot(m.cov * v0);
  std::printf(
      "D-03 cov: axis var %.9e m^2 (unobserved_var_m2 %.3e, |v0.phi| %.2e), max "
      "unflagged var %.3e\n",
      var_axis,
      var_m2,
      v0.tail<3>().norm(),
      max_unflagged_var);
  EXPECT_NEAR(var_axis / want, 1.0, 1e-9);
  EXPECT_NEAR(var_axis / var_m2, 1.0, 1e-6);
  EXPECT_GT(var_axis, 1e3 * max_unflagged_var);
}

// Enabled with nothing flagged (6-sided room): bitwise equal to disabled. The scan is the
// 0.25 m room grid: with the 0.5 m grid (720 points) the smallest eigenvalue of H in the
// first iteration is 77 < eig_threshold = 100 (141 at convergence), so a direction is
// legitimately flagged and the paths differ.
TEST(ScanMatchComponent, DegeneracyEnabledRoomEqualsDisabled) {
  RoomFixture f;
  const std::size_t max_points = f.cfg.realtime.max_points_per_scan;
  degeneracy::DegeneracyConfig enabled = f.cfg.degeneracy;
  enabled.enabled = true;
  degeneracy::DegeneracyConfig disabled = f.cfg.degeneracy;
  disabled.enabled = false;
  ScanMatcher a(f.cfg.scan_match, enabled, max_points);
  ScanMatcher b(f.cfg.scan_match, disabled, max_points);
  const std::vector<Vec3> scan_w = room_points_w(kDenseRoomScanSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  std::mt19937 rng(2);
  const Pose init = perturb(kTruthRoom, 0.5, 10.0 * kDegToRad, rng);
  const common::Result<MatchResult> ra = a.align(scan, f.map, init);
  const common::Result<MatchResult> rb = b.align(scan, f.map, init);
  ASSERT_TRUE(ra.is_ok() && rb.is_ok());
  EXPECT_TRUE(bitwise_equal(ra.value(), rb.value()));
}

TEST(ScanMatchComponent, NoAllocationWithDegeneracy) {
  CorridorFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  ASSERT_TRUE(matcher.align(f.scan, f.map, f.init).is_ok());  // warm-up
  bool ok = false;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    const common::Result<MatchResult> r = matcher.align(f.scan, f.map, f.init);
    ok = r.is_ok();
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(ok);
}

// Map is a single line: plane-fit RMS 0 and an arbitrary normal. Never NaN: either an error
// or ok with finite pose, H and cov.
TEST(ScanMatchComponent, CollinearMapNeverNaN) {
  for (const bool deg_enabled : {false, true}) {
    pipeline::Config cfg = default_config();
    cfg.degeneracy.enabled = deg_enabled;
    map::VoxelMap m(cfg.map);
    insert_points(m, line_points_w(0.05, 10.0));
    std::vector<Vec3> scan_w = line_points_w(0.1, 5.0);
    for (Vec3& p : scan_w) {
      p += Vec3(0.0, 0.02, 0.02);
    }
    ScanFrame scan(scan_w.size());
    make_scan(scan, scan_w, Pose{});
    ScanMatcher matcher(cfg.scan_match, cfg.degeneracy, cfg.realtime.max_points_per_scan);
    const common::Result<MatchResult> r = matcher.align(scan, m, Pose{});
    std::printf("collinear (degeneracy %s): %s\n",
                deg_enabled ? "on" : "off",
                r.is_ok() ? "ok" : r.reason().c_str());
    if (r.is_ok()) {
      const MatchResult& res = r.value();
      EXPECT_TRUE(res.T_w_l.R.allFinite() && res.T_w_l.p.allFinite());
      EXPECT_TRUE(res.H.allFinite());
      EXPECT_TRUE(res.cov.allFinite());
    }
  }
}

// Ray-cast LiDAR of box_world.hpp (rings x columns, same elevations) from a static pose:
// ring-major order, all offsets 0, points in l.
void cast_box_static(const test::BoxWorldParams& p, const Pose& T_w_l, ScanFrame& out) {
  out.clear();
  for (int r = 0; r < p.rings; ++r) {
    const double el_deg =
        p.elevation_min_deg + (p.elevation_max_deg - p.elevation_min_deg) * r / (p.rings - 1);
    const double el = el_deg * kDegToRad;
    for (int c = 0; c < p.columns; ++c) {
      const double az = 2.0 * kPi * c / p.columns;
      const Vec3 d_l(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      const Vec3 d = T_w_l.R * d_l;
      double range = std::numeric_limits<double>::infinity();
      for (int k = 0; k < 3; ++k) {
        if (std::abs(d(k)) > 0.0) {
          const double wall = d(k) > 0.0 ? p.box_half_m(k) : -p.box_half_m(k);
          const double hit = (wall - T_w_l.p(k)) / d(k);
          range = hit > 0.0 ? std::min(range, hit) : range;
        }
      }
      const Vec3 p_l = range * d_l;
      (void)out.push(static_cast<float>(p_l.x()),
                     static_cast<float>(p_l.y()),
                     static_cast<float>(p_l.z()),
                     1.0F,
                     0U,
                     static_cast<std::uint16_t>(r));
    }
  }
}

struct CorridorConditioning {
  double h_ratio = 0.0;      // lambda_min / lambda_max of H (6x6, [rho, phi])
  double trans_ratio = 0.0;  // lambda_min / lambda_max of the translation Schur complement S
  double axis_share = 0.0;   // a^T S a / lambda_max(S), a = corridor axis in l
  int inliers = 0;
  std::size_t map_points = 0;
  std::size_t spacing_replaced = 0;
};

// Corridor from the ray-cast box world (pipeline component tests): the x walls lie beyond
// preprocess.max_range_m, so walls y = +-5 m, floor and ceiling remain and the world x axis
// is unobservable (no information along x for exact planes). Keyframe clouds every
// backend.keyframe_dist_m along x (body at y = z = 0, static scans, no deskew) are
// preprocessed and inserted as the pipeline does (ADR-016); a scan from the middle of the
// last stretch is aligned at its true pose. Information along x comes from neighbourhoods
// whose fitted normal has an x component: those across the wall-floor and wall-ceiling
// edges (planarity gate sweep, scan_match.md section 3).
CorridorConditioning corridor_conditioning(const pipeline::Config& cfg) {
  test::BoxWorldParams p;
  p.box_half_m.x() = 1000.0;  // m, beyond preprocess.max_range_m (100 m)
  preprocess::PreprocessConfig pc = cfg.preprocess;
  pc.deskew_enabled = false;  // static scans
  const auto n_rays = static_cast<std::size_t>(p.rings * p.columns);
  preprocess::Preprocessor pre(pc, p.T_b_l, n_rays);
  const imu::ImuTrajectory traj;  // not read without deskew
  ScanFrame raw(n_rays);
  ScanFrame ds(n_rays);
  map::VoxelMap m(cfg.map);
  const double step_m = cfg.backend.keyframe_dist_m;
  constexpr int kKeyframes = 10;
  for (int k = 0; k < kKeyframes; ++k) {
    const Pose T_w_l = Pose{Mat3::Identity(), Vec3(k * step_m, 0.0, 0.0)} * p.T_b_l;
    cast_box_static(p, T_w_l, raw);
    EXPECT_TRUE(pre.process(raw, traj, ds).is_ok());
    m.insert(ds, T_w_l);
  }
  const Pose T_w_l = Pose{Mat3::Identity(), Vec3((kKeyframes - 0.5) * step_m, 0.0, 0.0)} * p.T_b_l;
  cast_box_static(p, T_w_l, raw);
  EXPECT_TRUE(pre.process(raw, traj, ds).is_ok());
  ScanMatcher matcher(cfg.scan_match, cfg.degeneracy, n_rays);
  const common::Result<MatchResult> r = matcher.align(ds, m, T_w_l);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  CorridorConditioning c;
  if (!r.is_ok()) {
    return c;
  }
  const Mat6& H = r.value().H;
  const common::DegeneracyReport rep = degeneracy::analyze(H, cfg.degeneracy);
  c.h_ratio = rep.eigvals(0) / rep.eigvals(5);
  const Mat3 H_tt = H.topLeftCorner<3, 3>();
  const Mat3 H_tr = H.topRightCorner<3, 3>();
  const Mat3 H_rr = H.bottomRightCorner<3, 3>();
  const Mat3 S = H_tt - H_tr * H_rr.ldlt().solve(H_tr.transpose());
  const Eigen::SelfAdjointEigenSolver<Mat3> es(S);
  c.trans_ratio = es.eigenvalues()(0) / es.eigenvalues()(2);
  const Vec3 axis_l = T_w_l.R.transpose() * Vec3::UnitX();
  c.axis_share = axis_l.dot(S * axis_l) / es.eigenvalues()(2);
  c.inliers = r.value().inliers;
  c.map_points = m.num_points();
  c.spacing_replaced = m.num_spacing_replaced();
  std::printf(
      "corridor min_point_dist_m %.3f plane %.3f mid %.3f: H lambda_min/max %.3e, translation "
      "Schur min/max "
      "%.3e, "
      "axis share %.3e, inliers %d, map points %zu, spacing replaced %zu\n",
      cfg.map.min_point_dist_m,
      cfg.scan_match.plane_min_eig_ratio,
      cfg.scan_match.min_mid_eig_ratio,
      c.h_ratio,
      c.trans_ratio,
      c.axis_share,
      c.inliers,
      c.map_points,
      c.spacing_replaced);
  return c;
}

// Spurious information along an unobservable corridor axis (scan_match.md section 3, map.md
// "Đánh giá"). Without the gates about 1.1 % of the largest translational information lies
// along the axis and map spacing does not change that (1.3 / 1.2 / 0.6 % at 0.05 / 0.1 /
// 0.2 m). The source is neighbourhoods across the wall-floor and wall-ceiling edges: the
// planarity gate (off by default) cuts the share to 0.10 % at 0.01 (0.04 % at 0.05). This is
// a conditioning statement only: in the pipeline corridor the fused GNSS ATE gets worse with
// the gate (CorridorDriftCorrectedByGnss 1.8 -> 5.2 m at 0.01).
TEST(ScanMatchComponent, CorridorAxisLeak) {
  for (const double d_m : {0.0, 0.05, 0.1, 0.2}) {
    pipeline::Config cfg = ungated_config();
    cfg.map.min_point_dist_m = d_m;
    const CorridorConditioning c = corridor_conditioning(cfg);
    EXPECT_GT(c.inliers, 0) << d_m;
    EXPECT_GT(c.axis_share, 5e-3) << d_m;
    EXPECT_LT(c.axis_share, 0.05) << d_m;
  }
  pipeline::Config gated = ungated_config();
  gated.scan_match.plane_min_eig_ratio = 0.01;
  const CorridorConditioning c = corridor_conditioning(gated);
  EXPECT_GE(c.inliers, gated.scan_match.min_inliers);
  EXPECT_LT(c.axis_share, 2e-3);
  RecordProperty("axis_share_plane001_ppm", static_cast<int>(std::lround(c.axis_share * 1e6)));
}

// Nearest-rank percentile, p in [0, 100]. Sorts `v`, which must not be empty.
double percentile(std::vector<double>& v, double p) {
  std::sort(v.begin(), v.end());
  const double rank = std::ceil(p / 100.0 * static_cast<double>(v.size()));
  const std::size_t idx = rank < 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
  return v[std::min(v.size() - 1, idx)];
}

// align() latency on the synthetic room with an ADR-007-sized scan (~11.5k points).
// Recorded for the PR (hot-path rule); never fails on time, only on correctness.
TEST(ScanMatchComponent, AlignTimingRecorded) {
  using Clock = std::chrono::steady_clock;
  RoomFixture f;
  ScanMatcher matcher(f.cfg.scan_match, f.cfg.degeneracy, f.cfg.realtime.max_points_per_scan);
  const std::vector<Vec3> scan_w = room_points_w(kMapSpacingM);
  ScanFrame scan(scan_w.size());
  make_scan(scan, scan_w, kTruthRoom);
  constexpr int kRuns = 50;
  std::mt19937 rng(6);
  std::vector<double> align_ms;
  align_ms.reserve(kRuns);
  int iterations = 0;
  for (int run = 0; run < kRuns; ++run) {
    const Pose init = perturb(kTruthRoom, 0.2, 2.0 * kDegToRad, rng);
    const Clock::time_point t0 = Clock::now();
    const common::Result<MatchResult> r = matcher.align(scan, f.map, init);
    const Clock::time_point t1 = Clock::now();
    ASSERT_TRUE(r.is_ok()) << "run " << run << ": " << r.reason();
    iterations += r.value().iterations;
    align_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  const double p50 = percentile(align_ms, 50.0);
  const double p95 = percentile(align_ms, 95.0);
  std::printf("scan_match align_ms p50=%.2f p95=%.2f points=%zu iters_mean=%.1f\n",
              p50,
              p95,
              scan.size(),
              static_cast<double>(iterations) / kRuns);
  RecordProperty("align_p50_us", static_cast<int>(std::lround(p50 * 1000.0)));
  RecordProperty("align_p95_us", static_cast<int>(std::lround(p95 * 1000.0)));
}

}  // namespace
}  // namespace mss::scan_match
