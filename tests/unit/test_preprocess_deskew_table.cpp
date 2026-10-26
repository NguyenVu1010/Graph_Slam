#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/preprocess/detail/deskew_table.hpp"

#include "alloc_counter.hpp"

namespace mss::preprocess::detail {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRigPitchRad = 2.0 * kPi / 180.0;  // ADR-011 section 2 rig: R_y(2 deg)
constexpr std::int64_t kImuPeriodNs = 5'000'000;    // 200 Hz (imu.rate_hz)
constexpr std::size_t kSamples = 21;                // 0.1 s scan
constexpr std::int64_t kT0Ns = 1'000'000'000;
constexpr double kTol = 1e-12;

common::Pose rig_T_b_l() {
  common::Pose T;
  T.R = Eigen::AngleAxisd(kRigPitchRad, common::Vec3::UnitY()).toRotationMatrix();
  T.p = common::Vec3(0.10, 0.0, 0.0);
  return T;
}

imu::ImuTrajectory random_traj() {
  std::mt19937 rng{42};  // config/default.yaml random_seed
  std::uniform_real_distribution<double> u(-0.2, 0.2);
  imu::ImuTrajectory traj;
  traj.t_start = common::Timestamp{kT0Ns};
  traj.size = kSamples;
  traj.samples[0] = {traj.t_start, common::Pose{}};
  for (std::size_t k = 1; k < kSamples; ++k) {
    common::Vec6 xi;
    for (int i = 0; i < 6; ++i) {
      xi(i) = u(rng);
    }
    traj.samples[k] = {common::Timestamp{kT0Ns + static_cast<std::int64_t>(k) * kImuPeriodNs},
                       common::se3_exp(xi)};
  }
  return traj;
}

common::Pose conj(const common::Pose& T_b0_b, const common::Pose& T_b_l) {
  return T_b_l.inverse() * T_b0_b * T_b_l;
}

double max_diff(const common::Pose& a, const common::Pose& b) {
  return std::max((a.R - b.R).cwiseAbs().maxCoeff(), (a.p - b.p).cwiseAbs().maxCoeff());
}

void expect_bitwise_eq(const common::Pose& a, const common::Pose& b) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_EQ(a.R(r, c), b.R(r, c));
    }
    EXPECT_EQ(a.p(r), b.p(r));
  }
}

TEST(DeskewTable, MatchesSamplesAtSampleTimes) {
  const imu::ImuTrajectory traj = random_traj();
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  for (std::size_t k = 0; k < traj.size; ++k) {
    EXPECT_LT(max_diff(table.T_l0_l(traj.samples[k].t), conj(traj.samples[k].T_b0_b, rig_T_b_l())),
              kTol)
        << "k=" << k;
  }
}

TEST(DeskewTable, MatchesBodyFrameInterpolation) {
  const imu::ImuTrajectory traj = random_traj();
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  std::mt19937 rng{42};
  const std::int64_t span_ns = static_cast<std::int64_t>(kSamples - 1) * kImuPeriodNs;
  std::uniform_int_distribution<std::int64_t> u(0, span_ns - 1);
  for (int i = 0; i < 50; ++i) {
    const std::int64_t off = u(rng);
    const std::size_t k = static_cast<std::size_t>(off / kImuPeriodNs);
    const common::Pose& Tk = traj.samples[k].T_b0_b;
    const common::Pose& Tk1 = traj.samples[k + 1].T_b0_b;
    const double s = static_cast<double>(off - static_cast<std::int64_t>(k) * kImuPeriodNs) /
                     static_cast<double>(kImuPeriodNs);
    const common::Vec6 xi = s * common::se3_log(Tk.inverse() * Tk1);
    const common::Pose expected = conj(Tk * common::se3_exp(xi), rig_T_b_l());
    EXPECT_LT(max_diff(table.T_l0_l(common::Timestamp{kT0Ns + off}), expected), kTol)
        << "off=" << off;
  }
}

TEST(DeskewTable, ClampsBeyondLastSample) {
  const imu::ImuTrajectory traj = random_traj();
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  const common::Timestamp t_last = traj.samples[traj.size - 1].t;
  const common::Pose at_last = table.T_l0_l(t_last);
  expect_bitwise_eq(table.T_l0_l(common::Timestamp{t_last.ns + 1'000'000}), at_last);
  expect_bitwise_eq(table.T_l0_l(common::Timestamp{t_last.ns + 1'000'000'000}), at_last);
}

TEST(DeskewTable, ClampsBeforeFirstSample) {
  const imu::ImuTrajectory traj = random_traj();
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  expect_bitwise_eq(table.T_l0_l(common::Timestamp{kT0Ns - 1}), common::Pose{});
}

TEST(DeskewTable, SingleSampleIsIdentityEverywhere) {
  imu::ImuTrajectory traj = random_traj();
  traj.size = 1;
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  for (const std::int64_t t : {kT0Ns - 1'000, kT0Ns, kT0Ns + 3'000'000, kT0Ns + 1'000'000'000}) {
    expect_bitwise_eq(table.T_l0_l(common::Timestamp{t}), common::Pose{});
  }
}

TEST(DeskewTable, IdentitySampleGivesExactIdentity) {
  const imu::ImuTrajectory traj = random_traj();
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  expect_bitwise_eq(table.T_l0_l(common::Timestamp{kT0Ns}), common::Pose{});
}

TEST(DeskewTable, StaticTrajectoryIsExactIdentity) {
  imu::ImuTrajectory traj = random_traj();
  for (std::size_t k = 0; k < traj.size; ++k) {
    traj.samples[k].T_b0_b = common::Pose{};
  }
  DeskewTable table;
  table.build(traj, rig_T_b_l());
  for (std::int64_t off = -1'000; off < 120'000'000; off += 777'777) {
    expect_bitwise_eq(table.T_l0_l(common::Timestamp{kT0Ns + off}), common::Pose{});
  }
}

TEST(DeskewTable, BuildAndQueryDoNotAllocate) {
  const imu::ImuTrajectory traj = random_traj();
  const common::Pose T_b_l = rig_T_b_l();
  DeskewTable table;
  double sink = 0.0;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    table.build(traj, T_b_l);
    for (std::int64_t i = 0; i < 1000; ++i) {
      sink += table.T_l0_l(common::Timestamp{kT0Ns + i * 100'000}).p.x();
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(std::isfinite(sink));  // keep the queries observable
}

}  // namespace
}  // namespace mss::preprocess::detail
