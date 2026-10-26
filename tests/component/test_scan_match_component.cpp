// Component tests of mss::scan_match::ScanMatcher on synthetic scenes with a known pose
// (docs/Test_CI_CD.md S-02..S-05, D-03; docs/derivations/scan_match.md).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
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
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/degeneracy/degeneracy.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "alloc_counter.hpp"
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

// config/default.yaml with map.max_points_per_voxel = kSceneMaxPointsPerVoxel.
pipeline::Config default_config() {
  const common::Result<pipeline::Config> cfg =
      pipeline::load_config(std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml");
  if (!cfg.is_ok()) {
    throw std::runtime_error("default config: " + cfg.reason());
  }
  pipeline::Config c = cfg.value();
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
// move along the axis and the covariance there is cov_scale * sigma_r^2 / eig_threshold.
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

  // cov = c * H_reg^-1 with c = cov_scale * sigma_r^2 > 0, so cov * H_reg = c * I.
  Mat6 H_reg = m.H;
  for (int k = 0; k < 6; ++k) {
    if (report.degenerate[static_cast<std::size_t>(k)]) {
      const Vec6 v_k = report.eigvecs.col(k);
      H_reg.noalias() +=
          (f.cfg.degeneracy.eig_threshold - report.eigvals(k)) * (v_k * v_k.transpose());
    }
  }
  const Mat6 cov_H = m.cov * H_reg;
  const double c = cov_H.trace() / 6.0;
  ASSERT_GT(c, 0.0);
  EXPECT_LT((cov_H - c * Mat6::Identity()).norm(), 1e-6 * c * 6.0);
  const double var_axis = v0.dot(m.cov * v0);
  EXPECT_NEAR(var_axis * f.cfg.degeneracy.eig_threshold / c, 1.0, 1e-6);
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
