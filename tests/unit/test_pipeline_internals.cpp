// Unit tests of the pipeline's internal helpers (core/pipeline/src, not installed):
// IMU ring buffer, deskew trajectory builder, l->b conversions, keyframe store and policy.
// Formulas: docs/derivations/pipeline.md.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mss/backend/config.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
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
#include "keyframe_policy.hpp"
#include "keyframe_store.hpp"

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

// N-3: the state used for a GNSS fix at t is the prediction interpolated at t, not the
// pose of the previous IMU sample. Constant velocity 1 m/s along x, t mid-interval.
TEST(ImuTrajectory, PredictStateAtInterpolatesMidInterval) {
  TrajFixture f;
  ASSERT_NO_FATAL_FAILURE(
      fill(f.buf, 0, 300 * kMs, kImuPeriodNs, Vec3(0.0, 0.0, kG), Vec3::Zero()));
  NavState x0 = state_at(100 * kMs);
  x0.v_w_b = Vec3(1.0, 0.0, 0.0);
  imu::Preintegrator pred(imu_config());
  pred.reset(x0.bias);
  ASSERT_TRUE(pred.integrate(*f.buf.interpolate(x0.t)).is_ok());
  const Timestamp t{x0.t.ns + 12 * kMs + kMs / 2};  // 12.5 ms: between samples 110 and 115
  const NavState x = predict_state_at(f.buf, pred, x0.t, x0, t, kGw, f.scratch);
  EXPECT_NEAR(x.T_w_b.p.x(), 0.0125, 1e-9);  // the previous sample (110 ms) would give 0.010
  EXPECT_LT(x.T_w_b.p.tail<2>().norm(), 1e-9);
  EXPECT_EQ(x.t, t);
  const NavState same = predict_state_at(f.buf, pred, x0.t, x0, x0.t, kGw, f.scratch);
  EXPECT_LT(same.T_w_b.p.norm(), 1e-12);  // t == x0.t: x0 itself
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

// ---------------------------------------------------------------- KeyframeStore

void make_cloud(common::ScanFrame& cloud, std::size_t n, float base) {
  cloud.reserve(n);
  cloud.clear();
  for (std::size_t i = 0; i < n; ++i) {
    const float f = base + static_cast<float>(i);
    ASSERT_TRUE(cloud.push(f, 2.0F * f, 3.0F, 1.0F, static_cast<std::uint32_t>(i), 0));
  }
}

TEST(KeyframeStore, AddKeepsInsertionOrder) {
  KeyframeStore store(5);
  common::ScanFrame cloud;
  ASSERT_NO_FATAL_FAILURE(make_cloud(cloud, 10, 0.0F));
  for (common::KeyId id = 0; id < 3; ++id) {
    ASSERT_TRUE(store.add(id, Timestamp{static_cast<std::int64_t>(100 + id)}, cloud).is_ok());
  }
  ASSERT_EQ(store.size(), 3U);
  EXPECT_EQ(store.capacity(), 5U);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(store.at(i).id, i);
  }
  const KeyframeEntry* e = store.find(1);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->t, Timestamp{101});
  EXPECT_EQ(e->cloud_l.size(), 10U);
  EXPECT_EQ(store.find(7), nullptr);
}

TEST(KeyframeStore, EvictsOldestBeyondMaxKeyframes) {
  KeyframeStore store(3);
  common::ScanFrame cloud;
  ASSERT_NO_FATAL_FAILURE(make_cloud(cloud, 4, 0.0F));
  for (common::KeyId id = 0; id < 5; ++id) {
    ASSERT_TRUE(store.add(id, Timestamp{static_cast<std::int64_t>(id)}, cloud).is_ok());
  }
  ASSERT_EQ(store.size(), 3U);
  EXPECT_EQ(store.at(0).id, 2U);
  EXPECT_EQ(store.at(1).id, 3U);
  EXPECT_EQ(store.at(2).id, 4U);
  EXPECT_EQ(store.n_evicted(), 2U);
  EXPECT_EQ(store.find(0), nullptr);
  ASSERT_NE(store.find(4), nullptr);
  EXPECT_EQ(store.find(4)->id, 4U);
}

TEST(KeyframeStore, RejectsNonIncreasingId) {
  KeyframeStore store(3);
  common::ScanFrame cloud;
  ASSERT_NO_FATAL_FAILURE(make_cloud(cloud, 4, 0.0F));
  ASSERT_TRUE(store.add(1, Timestamp{1}, cloud).is_ok());
  EXPECT_EQ(store.add(1, Timestamp{2}, cloud).reason(), "keyframe id not increasing");
  EXPECT_EQ(store.add(0, Timestamp{3}, cloud).reason(), "keyframe id not increasing");
  EXPECT_EQ(store.size(), 1U);
  EXPECT_THROW(KeyframeStore{0}, std::invalid_argument);
}

TEST(KeyframeStore, CloudIsCopied) {
  KeyframeStore store(2);
  common::ScanFrame cloud;
  ASSERT_NO_FATAL_FAILURE(make_cloud(cloud, 4, 1.0F));
  ASSERT_TRUE(store.add(0, Timestamp{1}, cloud).is_ok());
  cloud.xyz_mut()[0] = -100.0F;
  EXPECT_EQ(store.at(0).cloud_l.xyz()[0], 1.0F);
}

TEST(KeyframeStore, ReusedSlotDoesNotAllocate) {
  KeyframeStore store(2);
  common::ScanFrame cloud;
  ASSERT_NO_FATAL_FAILURE(make_cloud(cloud, 50, 0.0F));
  ASSERT_TRUE(store.add(0, Timestamp{0}, cloud).is_ok());
  ASSERT_TRUE(store.add(1, Timestamp{1}, cloud).is_ok());
  bool ok = true;
  AllocCounter::reset();
  for (common::KeyId id = 2; id < 6; ++id) {
    ok = ok && store.add(id, Timestamp{static_cast<std::int64_t>(id)}, cloud).is_ok();
  }
  const std::size_t allocs = AllocCounter::count();
  EXPECT_TRUE(ok);
  EXPECT_EQ(allocs, 0U);
}

// ---------------------------------------------------------------- KeyframePolicy

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

backend::BackendConfig keyframe_cfg() {
  backend::BackendConfig c;
  c.keyframe_dist_m = 1.0;
  c.keyframe_rot_rad = 10.0 * kDegToRad;
  return c;
}

Pose moved(const Vec3& dp, double yaw_deg) {
  Pose T;
  T.R = common::so3_exp(Vec3(0.0, 0.0, yaw_deg * kDegToRad));
  T.p = dp;
  return T;
}

TEST(KeyframePolicy, ByDistance) {
  const backend::BackendConfig c = keyframe_cfg();
  EXPECT_FALSE(is_new_keyframe(Pose{}, moved(Vec3(0.999, 0.0, 0.0), 0.0), c));
  EXPECT_TRUE(is_new_keyframe(Pose{}, moved(Vec3(1.0, 0.0, 0.0), 0.0), c));
  EXPECT_TRUE(is_new_keyframe(moved(Vec3(1.0, 1.0, 0.0), 0.0), moved(Vec3(1.0, 2.5, 0.0), 0.0), c));
}

TEST(KeyframePolicy, ByRotation) {
  const backend::BackendConfig c = keyframe_cfg();
  EXPECT_FALSE(is_new_keyframe(Pose{}, moved(Vec3::Zero(), 9.99), c));
  // 10 deg up to the rounding of so3_exp/so3_log: just above it.
  EXPECT_TRUE(is_new_keyframe(Pose{}, moved(Vec3::Zero(), 10.0 + 1e-9), c));
  EXPECT_TRUE(is_new_keyframe(moved(Vec3::Zero(), 30.0), moved(Vec3::Zero(), 15.0), c));
}

TEST(KeyframePolicy, PoseMovedIsStrict) {
  const double trans_m = 0.05;
  const double rot_rad = 0.5 * kDegToRad;
  EXPECT_FALSE(pose_moved(Pose{}, moved(Vec3(0.05, 0.0, 0.0), 0.0), trans_m, rot_rad));
  EXPECT_TRUE(pose_moved(Pose{}, moved(Vec3(0.0501, 0.0, 0.0), 0.0), trans_m, rot_rad));
  EXPECT_FALSE(pose_moved(Pose{}, moved(Vec3::Zero(), 0.49), trans_m, rot_rad));
  EXPECT_TRUE(pose_moved(Pose{}, moved(Vec3::Zero(), 0.51), trans_m, rot_rad));
}

TEST(KeyframePolicy, MapWindowKeepsLastN) {
  MapWindow window(3);
  for (common::KeyId id = 0; id < 5; ++id) {
    window.push(id, moved(Vec3(static_cast<double>(id), 0.0, 0.0), 0.0));
  }
  ASSERT_EQ(window.size(), 3U);
  EXPECT_EQ(window.at(0).id, 2U);
  EXPECT_EQ(window.at(2).id, 4U);
  EXPECT_EQ(window.at(0).T_w_b_inserted.p.x(), 2.0);
  window.set_pose(0, moved(Vec3(7.0, 0.0, 0.0), 0.0));
  EXPECT_EQ(window.at(0).T_w_b_inserted.p.x(), 7.0);
  EXPECT_EQ(window.at(1).T_w_b_inserted.p.x(), 3.0);
  EXPECT_THROW(MapWindow{0}, std::invalid_argument);
}

TEST(KeyframePolicy, OdomDeltaTimeCheck) {
  const std::int64_t period = 20 * kMs;
  const Timestamp t_i{1000 * kMs};
  const Timestamp t_j{2000 * kMs};
  common::OdomDelta d;
  d.t0 = Timestamp{t_i.ns - 30 * kMs};
  d.t1 = Timestamp{t_j.ns + 30 * kMs};
  EXPECT_TRUE(odom_delta_in_time(d, t_i, t_j, period));
  d.t0 = Timestamp{t_i.ns - 30 * kMs - 1};
  EXPECT_FALSE(odom_delta_in_time(d, t_i, t_j, period));
  d.t0 = t_i;
  d.t1 = Timestamp{t_j.ns - 30 * kMs - 1};
  EXPECT_FALSE(odom_delta_in_time(d, t_i, t_j, period));
  d.t1 = t_j;
  EXPECT_TRUE(odom_delta_in_time(d, t_i, t_j, period));
  EXPECT_FALSE(odom_delta_in_time(d, t_i, t_j, 0));
}

}  // namespace
}  // namespace mss::pipeline::detail
