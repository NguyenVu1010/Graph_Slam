// Unit tests of the pipeline's internal helpers (core/pipeline/src, not installed):
// IMU ring buffer, deskew trajectory builder, l->b conversions, keyframe store and policy.
// Formulas: docs/derivations/pipeline.md.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/pipeline/bounded_queue.hpp"

#include "alloc_counter.hpp"
#include "frames.hpp"
#include "imu_buffer.hpp"
#include "imu_trajectory_builder.hpp"

namespace mss::pipeline::detail {
namespace {

using common::ImuSample;
using common::Mat3;
using common::Mat6;
using common::NavState;
using common::Pose;
using common::Timestamp;
using common::Vec3;
using common::Vec6;

constexpr double kG = 9.80665;                  // m/s^2
constexpr std::int64_t kMs = 1000000;           // ns per ms
constexpr std::int64_t kImuPeriodNs = 5 * kMs;  // 200 Hz
const Vec3 kGw(0.0, 0.0, -kG);

imu::ImuConfig imu_config(double rate_hz = 200.0) {
  imu::ImuConfig c;
  c.rate_hz = rate_hz;
  c.acc_noise_density = 5.8e-3;
  c.gyro_noise_density = 3.5e-4;
  c.acc_random_walk = 2.0e-4;
  c.gyro_random_walk = 2.0e-5;
  c.gravity_mps2 = kG;
  c.init_window_s = 2.0;
  c.init_max_acc_std_mps2 = 0.05;
  c.init_max_gyro_std_radps = 0.01;
  return c;
}

ImuSample sample(std::int64_t t, const Vec3& acc, const Vec3& gyro) {
  ImuSample s;
  s.t = Timestamp{t};
  s.acc_b = acc;
  s.gyro_b = gyro;
  return s;
}

// Fills `buf` with samples every `period_ns` in [t0, t1], skipping times in `skip`.
void fill(ImuBuffer& buf,
          std::int64_t t0,
          std::int64_t t1,
          std::int64_t period_ns,
          const Vec3& acc,
          const Vec3& gyro,
          const std::vector<std::int64_t>& skip = {}) {
  for (std::int64_t t = t0; t <= t1; t += period_ns) {
    bool skipped = false;
    for (const std::int64_t s : skip) {
      skipped = skipped || s == t;
    }
    if (!skipped) {
      ASSERT_TRUE(buf.push(sample(t, acc, gyro)));
    }
  }
}

double rot_err(const Mat3& a, const Mat3& b) {
  const Mat3 d = a.transpose() * b;
  return common::so3_log(d).norm();
}

void expect_trajectory_invariants(const imu::ImuTrajectory& traj,
                                  Timestamp t_start,
                                  Timestamp t_end) {
  ASSERT_GT(traj.size, 0U);
  EXPECT_EQ(traj.t_start, t_start);
  EXPECT_EQ(traj.samples[0].t, t_start);
  EXPECT_TRUE(traj.samples[0].T_b0_b.R == Mat3::Identity());
  EXPECT_TRUE(traj.samples[0].T_b0_b.p == Vec3::Zero());
  EXPECT_GE(traj.samples[traj.size - 1].t, t_end);
  for (std::size_t k = 1; k < traj.size; ++k) {
    EXPECT_LT(traj.samples[k - 1].t, traj.samples[k].t) << k;
  }
}

// ---------------------------------------------------------------- ImuBuffer

TEST(ImuBuffer, PushRejectsNonIncreasing) {
  ImuBuffer buf;
  EXPECT_TRUE(buf.push(sample(10, Vec3::Zero(), Vec3::Zero())));
  EXPECT_FALSE(buf.push(sample(10, Vec3::Zero(), Vec3::Zero())));
  EXPECT_FALSE(buf.push(sample(9, Vec3::Zero(), Vec3::Zero())));
  EXPECT_EQ(buf.size(), 1U);
}

TEST(ImuBuffer, FullDropsOldest) {
  ImuBuffer buf;
  for (std::size_t k = 0; k <= kMaxImuQueue; ++k) {
    ASSERT_TRUE(buf.push(sample(static_cast<std::int64_t>(k), Vec3::Zero(), Vec3::Zero())));
  }
  EXPECT_TRUE(buf.full());
  EXPECT_EQ(buf.size(), kMaxImuQueue);
  EXPECT_EQ(buf.front().t, Timestamp{1});
  EXPECT_EQ(buf.back().t, (Timestamp{static_cast<std::int64_t>(kMaxImuQueue)}));
  buf.pop_front();
  EXPECT_EQ(buf.front().t, Timestamp{2});
}

TEST(ImuBuffer, InterpolateLinear) {
  ImuBuffer buf;
  ASSERT_TRUE(buf.push(sample(0, Vec3::Zero(), Vec3::Zero())));
  ASSERT_TRUE(buf.push(sample(10, Vec3(10.0, 0.0, 0.0), Vec3(0.0, 0.0, 1.0))));
  const auto s4 = buf.interpolate(Timestamp{4});
  ASSERT_TRUE(s4.has_value());
  EXPECT_EQ(s4->acc_b.x(), 4.0);
  EXPECT_EQ(s4->t, Timestamp{4});
  EXPECT_NEAR(s4->gyro_b.z(), 0.4, 1e-15);
  const auto s10 = buf.interpolate(Timestamp{10});
  ASSERT_TRUE(s10.has_value());
  EXPECT_EQ(s10->acc_b.x(), 10.0);
  EXPECT_EQ(s10->t, Timestamp{10});
  EXPECT_FALSE(buf.interpolate(Timestamp{11}).has_value());
  EXPECT_FALSE(buf.interpolate(Timestamp{-1}).has_value());
  EXPECT_FALSE(ImuBuffer{}.interpolate(Timestamp{0}).has_value());
}

TEST(ImuBuffer, CopyRangeInclusive) {
  ImuBuffer buf;
  for (std::int64_t t = 0; t <= 9; ++t) {
    ASSERT_TRUE(buf.push(sample(t, Vec3::Zero(), Vec3::Zero())));
  }
  std::array<ImuSample, kMaxImuQueue> out{};
  const std::size_t n = buf.copy_range(Timestamp{2}, Timestamp{5}, out);
  ASSERT_EQ(n, 4U);
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_EQ(out[i].t, (Timestamp{static_cast<std::int64_t>(2 + i)}));
  }
  EXPECT_EQ(buf.first_after(Timestamp{5}), 6U);
  EXPECT_EQ(buf.first_after(Timestamp{-3}), 0U);
  EXPECT_EQ(buf.first_after(Timestamp{9}), 10U);
}

// ---------------------------------------------------------------- ImuTrajectory

struct TrajFixture {
  imu::Preintegrator scratch{imu_config()};
  imu::ImuTrajectory traj;
  ImuBuffer buf;
};

NavState state_at(std::int64_t t) {
  NavState x;
  x.t = Timestamp{t};
  return x;
}

TEST(ImuTrajectory, StaticIsIdentity) {
  TrajFixture f;
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 0, 300 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3::Zero()));
  const std::int64_t t_start = 102 * kMs + 300;  // between two samples
  const Timestamp t_end{t_start + 99 * kMs};
  const NavState x0 = state_at(t_start);
  ASSERT_TRUE(build_imu_trajectory(f.buf, x0, t_end, kGw, f.scratch, f.traj).is_ok());
  ASSERT_NO_FATAL_FAILURE(expect_trajectory_invariants(f.traj, x0.t, t_end));
  for (std::size_t k = 0; k < f.traj.size; ++k) {
    EXPECT_LT(rot_err(f.traj.samples[k].T_b0_b.R, Mat3::Identity()), 1e-9) << k;
    EXPECT_LT(f.traj.samples[k].T_b0_b.p.norm(), 1e-9) << k;
  }
}

TEST(ImuTrajectory, ConstantYawRateMatchesExp) {
  TrajFixture f;
  const double w = 0.5;  // rad/s
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 0, 300 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3(0.0, 0.0, w)));
  const std::int64_t t_start = 100 * kMs;
  const Timestamp t_end{t_start + 99 * kMs};
  ASSERT_TRUE(
      build_imu_trajectory(f.buf, state_at(t_start), t_end, kGw, f.scratch, f.traj).is_ok());
  ASSERT_NO_FATAL_FAILURE(expect_trajectory_invariants(f.traj, Timestamp{t_start}, t_end));
  for (std::size_t k = 0; k < f.traj.size; ++k) {
    const double dt = common::seconds_between(Timestamp{t_start}, f.traj.samples[k].t);
    const Mat3 expected = common::so3_exp(Vec3(0.0, 0.0, w * dt));
    EXPECT_LT(rot_err(f.traj.samples[k].T_b0_b.R, expected), 1e-9) << k;
  }
}

TEST(ImuTrajectory, ConstantVelocityTranslation) {
  TrajFixture f;
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 0, 300 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3::Zero()));
  const std::int64_t t_start = 100 * kMs + 1234;
  const Timestamp t_end{t_start + 99 * kMs};
  NavState x0 = state_at(t_start);
  x0.v_w_b = Vec3(1.0, 0.0, 0.0);
  ASSERT_TRUE(build_imu_trajectory(f.buf, x0, t_end, kGw, f.scratch, f.traj).is_ok());
  ASSERT_NO_FATAL_FAILURE(expect_trajectory_invariants(f.traj, x0.t, t_end));
  for (std::size_t k = 0; k < f.traj.size; ++k) {
    const double dt = common::seconds_between(x0.t, f.traj.samples[k].t);
    EXPECT_LT((f.traj.samples[k].T_b0_b.p - Vec3(dt, 0.0, 0.0)).norm(), 1e-9) << k;
  }
}

TEST(ImuTrajectory, MoreThanCapacitySubsampledWithEnds) {
  TrajFixture f;
  const std::int64_t period = kMs / 2;  // 2 kHz
  ASSERT_NO_FATAL_FAILURE(fill(f.buf, 0, 300 * kMs, period, Vec3(0.0, 0.0, kG), Vec3::Zero()));
  const std::int64_t t_start = 100 * kMs + 100;
  const Timestamp t_end{t_start + 100 * kMs};
  ASSERT_TRUE(
      build_imu_trajectory(f.buf, state_at(t_start), t_end, kGw, f.scratch, f.traj).is_ok());
  EXPECT_EQ(f.traj.size, imu::ImuTrajectory::kMaxImuPerScan);
  ASSERT_NO_FATAL_FAILURE(expect_trajectory_invariants(f.traj, Timestamp{t_start}, t_end));
  // Last entry = first buffered sample with t >= t_end.
  const std::size_t i_last = f.buf.first_after(Timestamp{t_end.ns - 1});
  EXPECT_EQ(f.traj.samples[f.traj.size - 1].t, f.buf.at(i_last).t);
}

TEST(ImuTrajectory, BridgesMissingSamples) {  // Review Focus 2
  TrajFixture f;
  const double w = 0.5;  // rad/s
  ASSERT_NO_FATAL_FAILURE(fill(f.buf,
                               0,
                               300 * kMs,
                               kImuPeriodNs,
                               Vec3(0.0, 0.0, kG),
                               Vec3(0.0, 0.0, w),
                               {140 * kMs, 145 * kMs, 150 * kMs}));
  const std::int64_t t_start = 100 * kMs;
  const Timestamp t_end{t_start + 99 * kMs};
  ASSERT_TRUE(
      build_imu_trajectory(f.buf, state_at(t_start), t_end, kGw, f.scratch, f.traj).is_ok());
  ASSERT_NO_FATAL_FAILURE(expect_trajectory_invariants(f.traj, Timestamp{t_start}, t_end));
  for (std::size_t k = 0; k < f.traj.size; ++k) {
    const double dt = common::seconds_between(Timestamp{t_start}, f.traj.samples[k].t);
    const Mat3 expected = common::so3_exp(Vec3(0.0, 0.0, w * dt));
    EXPECT_LT(rot_err(f.traj.samples[k].T_b0_b.R, expected), 1e-9) << k;
  }
}

TEST(ImuTrajectory, ErrorsWhenImuMissesScan) {
  TrajFixture f;
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 50 * kMs, 150 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3::Zero()));
  const auto before =
      build_imu_trajectory(f.buf, state_at(40 * kMs), Timestamp{100 * kMs}, kGw, f.scratch, f.traj);
  EXPECT_EQ(before.reason(), "imu does not cover scan start");
  EXPECT_EQ(f.traj.size, 0U);
  const auto after = build_imu_trajectory(
      f.buf, state_at(100 * kMs), Timestamp{151 * kMs}, kGw, f.scratch, f.traj);
  EXPECT_EQ(after.reason(), "imu does not cover scan end");
  EXPECT_EQ(f.traj.size, 0U);
}

TEST(ImuTrajectory, BuildDoesNotAllocate) {
  TrajFixture f;
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 0, 300 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3(0.0, 0.0, 0.3)));
  const NavState x0 = state_at(100 * kMs + 7);
  bool ok = true;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 0; k < 10; ++k) {
      ok = ok &&
           build_imu_trajectory(f.buf, x0, Timestamp{x0.t.ns + 99 * kMs}, kGw, f.scratch, f.traj)
               .is_ok();
    }
  }
  const std::size_t allocs = AllocCounter::count();
  EXPECT_TRUE(ok);
  EXPECT_EQ(allocs, 0U);
}

// ---------------------------------------------------------------- Frames

Pose random_pose(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Pose T;
  T.R = common::so3_exp(Vec3(u(rng), u(rng), u(rng)));
  T.p = Vec3(u(rng), u(rng), u(rng));
  return T;
}

TEST(Frames, LidarCovInBodyMatchesNumericalJacobian) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double h = 1e-6;
  for (int trial = 0; trial < 20; ++trial) {
    const Pose T_b_l = random_pose(rng);
    const Pose T_w_l = random_pose(rng);
    const Pose T_l_b = T_b_l.inverse();
    const Pose T_w_b = T_w_l * T_l_b;
    Mat6 J_num = Mat6::Zero();
    for (int k = 0; k < 6; ++k) {
      Vec6 e = Vec6::Zero();
      e(k) = h;
      const Vec6 plus = common::boxminus(common::boxplus(T_w_l, e) * T_l_b, T_w_b);
      const Vec6 minus = common::boxminus(common::boxplus(T_w_l, -e) * T_l_b, T_w_b);
      J_num.col(k) = (plus - minus) / (2.0 * h);
    }
    const Mat6 adj = common::se3_adj(T_b_l);
    EXPECT_LT((J_num - adj).norm() / adj.norm(), 1e-6) << trial;

    Mat6 A = Mat6::Zero();
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        A(r, c) = u(rng);
      }
    }
    const Mat6 C = A * A.transpose() + 0.1 * Mat6::Identity();  // SPD
    const Mat6 expected = adj * C * adj.transpose();
    const Mat6 got = lidar_cov_in_body(C, T_b_l);
    EXPECT_LT((got - expected).cwiseAbs().maxCoeff(), 1e-12 * (1.0 + expected.norm())) << trial;
  }
}

TEST(Frames, DegeneracyEigvecsRotatedToBody) {
  std::mt19937 rng(11);
  const Pose T_b_l = random_pose(rng);
  common::DegeneracyReport r_l;
  for (int k = 0; k < 6; ++k) {
    r_l.eigvals(k) = 1.0 + k;
  }
  r_l.eigvecs = Mat6::Identity();
  r_l.eigvecs.block<3, 3>(0, 0) = common::so3_exp(Vec3(0.1, 0.2, 0.3));
  r_l.degenerate = {true, false, false, true, false, false};
  const common::DegeneracyReport r_b = degeneracy_in_body(r_l, T_b_l);
  const Mat6 expected = common::se3_adj(T_b_l) * r_l.eigvecs;
  EXPECT_LT((r_b.eigvecs - expected).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_TRUE(r_b.eigvals == r_l.eigvals);
  EXPECT_EQ(r_b.degenerate, r_l.degenerate);
}

}  // namespace
}  // namespace mss::pipeline::detail
