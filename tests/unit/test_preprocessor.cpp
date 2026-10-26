#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/preprocess/config.hpp"
#include "mss/preprocess/detail/voxel_set.hpp"
#include "mss/preprocess/preprocessor.hpp"

#include "alloc_counter.hpp"

namespace mss::preprocess {
namespace {

// config/default.yaml preprocess.* and realtime.max_points_per_scan.
constexpr std::size_t kMaxPointsPerScan = 65536;
const PreprocessConfig kDefaultCfg{0.5, 100.0, 0.5, true};

Preprocessor make(const PreprocessConfig& cfg, std::size_t max_points) {
  return Preprocessor(cfg, common::Pose{}, max_points);
}

TEST(Preprocessor, ConstructsWithDefaultYamlValues) {
  EXPECT_NO_THROW(make(kDefaultCfg, kMaxPointsPerScan));
}

TEST(Preprocessor, VoxelZeroIsValid) {
  EXPECT_NO_THROW(make({0.5, 100.0, 0.0, true}, 10));
}

TEST(Preprocessor, RejectsInvalidConfig) {
  EXPECT_THROW(make({-0.1, 100.0, 0.5, true}, 10), std::invalid_argument);  // min < 0
  EXPECT_THROW(make({0.5, 0.5, 0.5, true}, 10), std::invalid_argument);     // max <= min
  EXPECT_THROW(make({0.5, 0.4, 0.5, true}, 10), std::invalid_argument);
  EXPECT_THROW(make({0.5, 100.0, -0.5, true}, 10), std::invalid_argument);   // voxel < 0
  EXPECT_THROW(make({0.5, 100.0, 0.5, true}, 0), std::invalid_argument);     // max_points == 0
  EXPECT_THROW(make({0.5, 100.0, 1e-20, true}, 10), std::invalid_argument);  // index overflow
}

TEST(Preprocessor, RejectsNonFiniteConfig) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {nan, inf}) {
    EXPECT_THROW(make({bad, 100.0, 0.5, true}, 10), std::invalid_argument);
    EXPECT_THROW(make({0.5, bad, 0.5, true}, 10), std::invalid_argument);
    EXPECT_THROW(make({0.5, 100.0, bad, true}, 10), std::invalid_argument);
  }
}

// ---- process() ----------------------------------------------------------------------

constexpr std::int64_t kTStartNs = 2'000'000'000;
constexpr std::int64_t kImuPeriodNs = 5'000'000;  // 200 Hz (imu.rate_hz)
constexpr std::size_t kImuSamples = 21;           // covers a 0.1 s scan
constexpr std::uint32_t kScanLengthNs = 100'000'000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRigPitchRad = 2.0 * kPi / 180.0;  // ADR-011 section 2 rig: R_y(2 deg)

common::Pose rig_T_b_l() {
  common::Pose T;
  T.R = Eigen::AngleAxisd(kRigPitchRad, common::Vec3::UnitY()).toRotationMatrix();
  T.p = common::Vec3(0.10, 0.0, 0.0);
  return T;
}

// Uniform points in [-20, 20]^3 m whose range lies in [min_r, max_r], times in [0, 100 ms].
common::ScanFrame random_scan(std::size_t n,
                              std::uint32_t seed,
                              double min_r = 0.0,
                              double max_r = 1e9) {
  common::ScanFrame f(n);
  f.t_start = common::Timestamp{kTStartNs};
  std::mt19937 rng{seed};
  std::uniform_real_distribution<float> u(-20.0F, 20.0F);
  std::uniform_int_distribution<std::uint32_t> t(0, kScanLengthNs);
  while (f.size() < n) {
    const float x = u(rng);
    const float y = u(rng);
    const float z = u(rng);
    const double xd = x;
    const double yd = y;
    const double zd = z;
    const double r = std::sqrt(xd * xd + yd * yd + zd * zd);
    if (r < min_r || r > max_r) {
      continue;
    }
    const std::size_t i = f.size();
    f.push(x, y, z, static_cast<float>(i), t(rng), static_cast<std::uint16_t>(i % 16));
  }
  return f;
}

imu::ImuTrajectory static_traj() {
  imu::ImuTrajectory traj;
  traj.t_start = common::Timestamp{kTStartNs};
  traj.size = kImuSamples;
  for (std::size_t k = 0; k < kImuSamples; ++k) {
    traj.samples[k] = {common::Timestamp{kTStartNs + static_cast<std::int64_t>(k) * kImuPeriodNs},
                       common::Pose{}};
  }
  return traj;
}

// Constant body twist v = [1, 0, 0] m/s, w = [0, 0, 1] rad/s.
imu::ImuTrajectory moving_traj() {
  imu::ImuTrajectory traj = static_traj();
  common::Vec6 twist;
  twist << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  for (std::size_t k = 1; k < kImuSamples; ++k) {
    const double t_s = common::seconds_between(traj.t_start, traj.samples[k].t);
    const common::Vec6 xi = t_s * twist;
    traj.samples[k].T_b0_b = common::se3_exp(xi);
  }
  return traj;
}

bool same_point(const common::ScanFrame& a,
                std::size_t i,
                const common::ScanFrame& b,
                std::size_t j) {
  return a.xyz()[3 * i] == b.xyz()[3 * j] && a.xyz()[3 * i + 1] == b.xyz()[3 * j + 1] &&
         a.xyz()[3 * i + 2] == b.xyz()[3 * j + 2] && a.intensity()[i] == b.intensity()[j] &&
         a.t_offset_ns()[i] == b.t_offset_ns()[j] && a.ring()[i] == b.ring()[j];
}

::testing::AssertionResult frames_equal(const common::ScanFrame& a, const common::ScanFrame& b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "size " << a.size() << " vs " << b.size();
  }
  if (a.t_start != b.t_start) {
    return ::testing::AssertionFailure() << "t_start differs";
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!same_point(a, i, b, i)) {
      return ::testing::AssertionFailure() << "point " << i << " differs";
    }
  }
  return ::testing::AssertionSuccess();
}

using KeyTuple = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

KeyTuple key_of(const common::ScanFrame& f, std::size_t i, double voxel_m) {
  const detail::VoxelKey k =
      detail::voxel_key(f.xyz()[3 * i], f.xyz()[3 * i + 1], f.xyz()[3 * i + 2], voxel_m);
  return {k.x, k.y, k.z};
}

bool in_range(const common::ScanFrame& f, std::size_t i, double min_r, double max_r) {
  const double x = f.xyz()[3 * i];
  const double y = f.xyz()[3 * i + 1];
  const double z = f.xyz()[3 * i + 2];
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  const double r = std::sqrt(x * x + y * y + z * z);
  return r >= min_r && r <= max_r;
}

::testing::AssertionResult one_point_per_voxel(const common::ScanFrame& out, double voxel_m) {
  std::set<KeyTuple> keys;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (!keys.insert(key_of(out, i, voxel_m)).second) {
      return ::testing::AssertionFailure() << "voxel of point " << i << " already used";
    }
  }
  return ::testing::AssertionSuccess();
}

// P-02 (Test_CI_CD): omega = 0, v = 0 -> output equals input. Downsampling is off so that
// "equal" is meaningful.
TEST(Preprocessor, P02_StaticRobotOutputEqualsInput) {
  const PreprocessConfig cfg{0.5, 100.0, 0.0, true};
  const common::ScanFrame in = random_scan(5000, 42, 1.0, 30.0);
  common::ScanFrame out(5000);
  Preprocessor pre = make(cfg, 5000);
  ASSERT_TRUE(pre.process(in, static_traj(), out).is_ok());
  EXPECT_TRUE(frames_equal(out, in));
}

// P-03 (Test_CI_CD): random cloud, voxel 0.5 m.
TEST(Preprocessor, P03_VoxelDownsample) {
  const PreprocessConfig cfg{0.5, 100.0, 0.5, false};
  const common::ScanFrame in = random_scan(10000, 42);
  common::ScanFrame out(10000);
  Preprocessor pre = make(cfg, 10000);
  ASSERT_TRUE(pre.process(in, imu::ImuTrajectory{}, out).is_ok());
  ASSERT_GT(out.size(), 0U);
  ASSERT_LT(out.size(), in.size());

  // (a) at most one output point per voxel.
  EXPECT_TRUE(one_point_per_voxel(out, cfg.downsample_voxel_m));

  // (b) every output point is bitwise one input point, relative input order kept.
  std::size_t j = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    while (j < in.size() && !same_point(out, i, in, j)) {
      ++j;
    }
    ASSERT_LT(j, in.size()) << "output point " << i << " not found in order";
    ++j;
  }

  // (c) output voxels == voxels of the range-filtered input; (d) first in input order wins.
  std::set<KeyTuple> in_keys;
  std::vector<std::size_t> expected;
  for (std::size_t i = 0; i < in.size(); ++i) {
    if (in_range(in, i, cfg.min_range_m, cfg.max_range_m) &&
        in_keys.insert(key_of(in, i, cfg.downsample_voxel_m)).second) {
      expected.push_back(i);
    }
  }
  std::set<KeyTuple> out_keys;
  for (std::size_t i = 0; i < out.size(); ++i) {
    out_keys.insert(key_of(out, i, cfg.downsample_voxel_m));
  }
  EXPECT_EQ(out_keys, in_keys);
  ASSERT_EQ(out.size(), expected.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    ASSERT_TRUE(same_point(out, i, in, expected[i])) << "point " << i;
  }

  // (e) a second run on the same instance and a run on a fresh instance are identical.
  common::ScanFrame out2(10000);
  ASSERT_TRUE(pre.process(in, imu::ImuTrajectory{}, out2).is_ok());
  EXPECT_TRUE(frames_equal(out2, out));
  common::ScanFrame out3(10000);
  Preprocessor fresh = make(cfg, 10000);
  ASSERT_TRUE(fresh.process(in, imu::ImuTrajectory{}, out3).is_ok());
  EXPECT_TRUE(frames_equal(out3, out));
}

TEST(Preprocessor, P03b_DeskewedVoxelOutputRespectsGrid) {
  const PreprocessConfig cfg{0.5, 100.0, 0.5, true};
  const common::ScanFrame in = random_scan(10000, 42);
  common::ScanFrame out(10000);
  Preprocessor pre(cfg, rig_T_b_l(), 10000);
  ASSERT_TRUE(pre.process(in, moving_traj(), out).is_ok());
  ASSERT_GT(out.size(), 0U);
  EXPECT_TRUE(one_point_per_voxel(out, cfg.downsample_voxel_m));
}

// P-04 (Test_CI_CD): NaN, < 0.5 m and > max range are all removed, no crash.
TEST(Preprocessor, P04_BadPointsRemoved) {
  const PreprocessConfig cfg{0.5, 100.0, 0.0, false};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  common::ScanFrame in(16);
  in.t_start = common::Timestamp{kTStartNs};
  const auto push = [&in](float x, float y, float z) {
    const std::size_t i = in.size();
    in.push(x, y, z, static_cast<float>(i), static_cast<std::uint32_t>(i), 0);
  };
  push(nan, 1.0F, 1.0F);
  push(0.5F, 0.0F, 0.0F);  // kept: min range is inclusive
  push(1.0F, nan, 1.0F);
  push(1.0F, 1.0F, nan);
  push(inf, 1.0F, 1.0F);
  push(100.0F, 0.0F, 0.0F);  // kept: max range is inclusive
  push(1.0F, -inf, 1.0F);
  push(0.49F, 0.0F, 0.0F);
  push(0.0F, 0.0F, 0.0F);
  push(3.0F, 4.0F, 0.0F);  // kept
  push(100.01F, 0.0F, 0.0F);
  push(60.0F, 60.0F, 60.0F);
  common::ScanFrame out(16);
  Preprocessor pre = make(cfg, 16);
  ASSERT_TRUE(pre.process(in, imu::ImuTrajectory{}, out).is_ok());
  ASSERT_EQ(out.size(), 3U);
  EXPECT_TRUE(same_point(out, 0, in, 1));
  EXPECT_TRUE(same_point(out, 1, in, 5));
  EXPECT_TRUE(same_point(out, 2, in, 9));
}

TEST(Preprocessor, P04_AllPointsFilteredIsOkAndEmpty) {
  const PreprocessConfig cfg{0.5, 100.0, 0.0, false};
  common::ScanFrame in(4);
  in.push(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F, 0, 0);
  in.push(0.1F, 0.0F, 0.0F, 0.0F, 0, 0);
  in.push(200.0F, 0.0F, 0.0F, 0.0F, 0, 0);
  common::ScanFrame out(4);
  Preprocessor pre = make(cfg, 4);
  EXPECT_TRUE(pre.process(in, imu::ImuTrajectory{}, out).is_ok());
  EXPECT_EQ(out.size(), 0U);
}

TEST(Preprocessor, EmptyInputIsOk) {
  common::ScanFrame in(4);
  in.t_start = common::Timestamp{kTStartNs};
  common::ScanFrame out(4);
  Preprocessor pre = make(kDefaultCfg, 4);
  ASSERT_TRUE(pre.process(in, static_traj(), out).is_ok());
  EXPECT_EQ(out.size(), 0U);
  EXPECT_EQ(out.t_start, in.t_start);
}

TEST(Preprocessor, OutputCapacityTooSmallIsError) {
  const common::ScanFrame in = random_scan(10, 42);
  common::ScanFrame out(3);
  for (int i = 0; i < 3; ++i) {
    out.push(1.0F, 2.0F, 3.0F, 0.0F, 0, 0);
  }
  Preprocessor pre = make(kDefaultCfg, 10);
  const common::Status s = pre.process(in, static_traj(), out);
  EXPECT_FALSE(s.is_ok());
  EXPECT_EQ(s.reason(), "output capacity too small");
  EXPECT_EQ(out.size(), 0U);
}

TEST(Preprocessor, EmptyTrajectoryIsErrorWhenDeskewing) {
  const common::ScanFrame in = random_scan(10, 42);
  common::ScanFrame out(10);
  out.push(1.0F, 2.0F, 3.0F, 0.0F, 0, 0);
  Preprocessor pre = make(kDefaultCfg, 10);
  const common::Status s = pre.process(in, imu::ImuTrajectory{}, out);
  EXPECT_FALSE(s.is_ok());
  EXPECT_EQ(s.reason(), "empty imu trajectory");
  EXPECT_EQ(out.size(), 0U);
}

TEST(Preprocessor, EmptyTrajectoryIgnoredWhenDeskewOff) {
  const common::ScanFrame in = random_scan(10, 42);
  common::ScanFrame out(10);
  Preprocessor pre = make({0.5, 100.0, 0.5, false}, 10);
  EXPECT_TRUE(pre.process(in, imu::ImuTrajectory{}, out).is_ok());
}

// Review focus 4: t_offset_ns == 0 is unchanged bit for bit under non-zero motion.
TEST(Preprocessor, ZeroOffsetPointUnchangedUnderMotion) {
  const PreprocessConfig cfg{0.5, 100.0, 0.0, true};
  common::ScanFrame in(3);
  in.t_start = common::Timestamp{kTStartNs};
  in.push(5.0F, 0.0F, 0.0F, 1.0F, 0, 0);
  in.push(0.0F, 5.0F, 0.0F, 2.0F, 0, 1);
  in.push(0.0F, 0.0F, 5.0F, 3.0F, 0, 2);
  common::ScanFrame out(3);
  Preprocessor pre(cfg, rig_T_b_l(), 3);
  ASSERT_TRUE(pre.process(in, moving_traj(), out).is_ok());
  EXPECT_TRUE(frames_equal(out, in));
}

// Review focus 3: no state leaks from a larger previous scan.
TEST(Preprocessor, SecondScanMatchesFreshInstance) {
  const common::ScanFrame a = random_scan(10000, 42);
  const common::ScanFrame b = random_scan(2000, 7);
  common::ScanFrame out_a(10000);
  common::ScanFrame out_b(10000);
  common::ScanFrame out_fresh(10000);
  Preprocessor pre(kDefaultCfg, rig_T_b_l(), 10000);
  ASSERT_TRUE(pre.process(a, moving_traj(), out_a).is_ok());
  ASSERT_TRUE(pre.process(b, moving_traj(), out_b).is_ok());
  Preprocessor fresh(kDefaultCfg, rig_T_b_l(), 10000);
  ASSERT_TRUE(fresh.process(b, moving_traj(), out_fresh).is_ok());
  ASSERT_GT(out_b.size(), 0U);
  EXPECT_TRUE(frames_equal(out_b, out_fresh));
}

// Review F1: input larger than max_points (an unchecked precondition) must not write past
// the voxel set; surplus new voxels are dropped and process() still returns ok.
TEST(Preprocessor, InputBeyondMaxPointsDoesNotOverflowVoxelSet) {
  constexpr std::size_t kMaxPoints = 100;
  constexpr std::size_t kInPoints = 1000;
  common::ScanFrame in(kInPoints);
  in.t_start = common::Timestamp{kTStartNs};
  for (std::size_t i = 0; i < kInPoints; ++i) {  // one point per voxel, all in range
    in.push(1.0F + static_cast<float>(i), 0.25F, 0.25F, 0.0F, 0, 0);
  }
  common::ScanFrame out(kInPoints);
  Preprocessor pre = make({0.5, 2000.0, 0.5, false}, kMaxPoints);
  ASSERT_TRUE(pre.process(in, imu::ImuTrajectory{}, out).is_ok());
  ASSERT_EQ(out.size(), kMaxPoints);
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_TRUE(same_point(out, i, in, i)) << "point " << i;
  }
}

TEST(Preprocessor, ProcessDoesNotAllocate) {
  const common::ScanFrame in = random_scan(kMaxPointsPerScan, 42);
  const imu::ImuTrajectory traj = moving_traj();
  common::ScanFrame out(kMaxPointsPerScan);
  Preprocessor pre(kDefaultCfg, rig_T_b_l(), kMaxPointsPerScan);
  bool ok1 = false;
  bool ok2 = false;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    ok1 = pre.process(in, traj, out).is_ok();
    ok2 = pre.process(in, traj, out).is_ok();
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(ok1);
  EXPECT_TRUE(ok2);
  EXPECT_GT(out.size(), 0U);
}

}  // namespace
}  // namespace mss::preprocess
