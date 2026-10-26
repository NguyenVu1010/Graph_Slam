// WheelIntegrator: exact SE(2) integration of wheel samples into body-frame deltas
// (docs/derivations/wheel_odom.md sections 1-3). Conventions: T_a_b maps b to a, right
// perturbation T <- T * Exp(delta), tangent [rho, phi].
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/odom/config.hpp"
#include "mss/odom/wheel_integrator.hpp"

#include "alloc_counter.hpp"

namespace mss::odom {
namespace {

using common::Mat6;
using common::OdomDelta;
using common::Pose;
using common::Timestamp;
using common::Vec3;
using common::Vec6;
using common::WheelSample;

constexpr std::int64_t kDtNs = 20'000'000;  // ns, 50 Hz
constexpr double kDtS = 0.02;               // s, kDtNs in seconds
constexpr double kV = 0.8;                  // m/s
constexpr double kOmega = 0.5;              // rad/s
constexpr double kSigmaV = 0.02;            // m/s (config/default.yaml)
constexpr double kSigmaOmega = 0.02;        // rad/s (config/default.yaml)

Pose make_tbo() {
  return Pose{common::so3_exp(Vec3(0.1, -0.2, 0.3)), Vec3(0.15, -0.05, 0.10)};
}

WheelOdomConfig make_cfg(double sigma_v = kSigmaV, double sigma_omega = kSigmaOmega) {
  WheelOdomConfig cfg;
  cfg.speed_sigma_mps = sigma_v;
  cfg.yaw_rate_sigma_radps = sigma_omega;
  return cfg;
}

WheelSample make_sample(std::int64_t t_ns, double v, double omega) {
  WheelSample s;
  s.t = Timestamp{t_ns};
  s.v_mps = v;
  s.omega_radps = omega;
  return s;
}

// Analytic unicycle pose of o after t_s seconds at constant (v, omega), from rest at the
// origin, embedded in SE(3). omega == 0 gives a straight line.
Pose arc_pose(double v_mps, double omega_radps, double t_s) {
  const double yaw = omega_radps * t_s;
  double x = v_mps * t_s;
  double y = 0.0;
  if (omega_radps != 0.0) {
    const double half = std::sin(0.5 * yaw);
    x = v_mps / omega_radps * std::sin(yaw);
    y = v_mps / omega_radps * 2.0 * half * half;  // (1 - cos yaw) without cancellation
  }
  return Pose{common::so3_exp(Vec3(0.0, 0.0, yaw)), Vec3(x, y, 0.0)};
}

// Feeds n samples at t = k * dt_ns (k = 0..n-1) with a constant reading.
OdomDelta run_constant(const WheelOdomConfig& cfg,
                       const Pose& T_b_o,
                       int n,
                       double v,
                       double omega,
                       std::int64_t dt_ns = kDtNs) {
  WheelIntegrator wi(cfg, T_b_o);
  for (int k = 0; k < n; ++k) {
    EXPECT_TRUE(wi.integrate(make_sample(k * dt_ns, v, omega)).is_ok());
  }
  return wi.take_delta();
}

double pose_error(const Pose& a, const Pose& b) {
  return common::boxminus(a, b).norm();
}

bool is_identity(const Pose& T) {
  return T.R == common::Mat3::Identity() && T.p == Vec3::Zero();
}

TEST(WheelIntegrator, ConstructorRejectsNegativeSigma) {
  EXPECT_THROW(WheelIntegrator(make_cfg(-0.01, kSigmaOmega), Pose{}), std::invalid_argument);
  EXPECT_THROW(WheelIntegrator(make_cfg(kSigmaV, -0.01), Pose{}), std::invalid_argument);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(WheelIntegrator(make_cfg(nan, kSigmaOmega), Pose{}), std::invalid_argument);
  EXPECT_THROW(WheelIntegrator(make_cfg(kSigmaV, nan), Pose{}), std::invalid_argument);
  EXPECT_NO_THROW(WheelIntegrator(make_cfg(0.0, 0.0), Pose{}));
}

TEST(WheelIntegrator, FirstSampleOnlySetsStart) {
  WheelIntegrator wi(make_cfg(), make_tbo());
  ASSERT_TRUE(wi.integrate(make_sample(5'000'000'000, kV, kOmega)).is_ok());
  const OdomDelta d = wi.take_delta();
  EXPECT_TRUE(is_identity(d.T_b0_b1));
  EXPECT_EQ(d.t0, Timestamp{5'000'000'000});
  EXPECT_EQ(d.t1, Timestamp{5'000'000'000});
  EXPECT_TRUE(d.cov.isZero(0.0));
}

TEST(WheelIntegrator, EmptyBeforeAnySample) {
  WheelIntegrator wi(make_cfg(), make_tbo());
  const OdomDelta d = wi.take_delta();
  EXPECT_TRUE(is_identity(d.T_b0_b1));
  EXPECT_EQ(d.t0, Timestamp{});
  EXPECT_EQ(d.t1, Timestamp{});
  EXPECT_TRUE(d.cov.isZero(0.0));
}

TEST(WheelIntegrator, ConstantArcMatchesAnalytic) {
  const OdomDelta d = run_constant(make_cfg(), Pose{}, 51, kV, kOmega);
  EXPECT_LT(pose_error(d.T_b0_b1, arc_pose(kV, kOmega, 1.0)), 1e-12);
  EXPECT_EQ(d.t0, Timestamp{0});
  EXPECT_EQ(d.t1, Timestamp{1'000'000'000});
}

TEST(WheelIntegrator, ExtrinsicConjugation) {
  const Pose T_b_o = make_tbo();
  const OdomDelta d = run_constant(make_cfg(), T_b_o, 51, kV, kOmega);
  const Pose expected = T_b_o * arc_pose(kV, kOmega, 1.0) * T_b_o.inverse();
  EXPECT_LT(pose_error(d.T_b0_b1, expected), 1e-12);
}

TEST(WheelIntegrator, ChainsWithoutGaps) {
  const Pose T_b_o = make_tbo();
  WheelIntegrator wi(make_cfg(), T_b_o);
  for (int k = 0; k < 26; ++k) {
    ASSERT_TRUE(wi.integrate(make_sample(k * kDtNs, kV, kOmega)).is_ok());
  }
  const OdomDelta d1 = wi.take_delta();
  for (int k = 26; k < 51; ++k) {
    ASSERT_TRUE(wi.integrate(make_sample(k * kDtNs, kV, kOmega)).is_ok());
  }
  const OdomDelta d2 = wi.take_delta();
  EXPECT_EQ(d1.t0, Timestamp{0});
  EXPECT_EQ(d2.t0, d1.t1);
  EXPECT_EQ(d2.t1, Timestamp{50 * kDtNs});
  const OdomDelta whole = run_constant(make_cfg(), T_b_o, 51, kV, kOmega);
  EXPECT_LT(pose_error(d1.T_b0_b1 * d2.T_b0_b1, whole.T_b0_b1), 1e-12);

  const OdomDelta d3 = wi.take_delta();
  EXPECT_TRUE(is_identity(d3.T_b0_b1));
  EXPECT_EQ(d3.t0, d2.t1);
  EXPECT_EQ(d3.t1, d2.t1);
  EXPECT_TRUE(d3.cov.isZero(0.0));
}

TEST(WheelIntegrator, RejectsNonFinite) {
  const Pose T_b_o = make_tbo();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  WheelIntegrator wi(make_cfg(), T_b_o);
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(wi.integrate(make_sample(k * kDtNs, kV, kOmega)).is_ok());
  }
  const common::Status s_nan = wi.integrate(make_sample(10 * kDtNs, nan, kOmega));
  EXPECT_FALSE(s_nan.is_ok());
  EXPECT_EQ(s_nan.reason(), "non-finite wheel sample");
  const common::Status s_inf = wi.integrate(make_sample(10 * kDtNs, kV, inf));
  EXPECT_FALSE(s_inf.is_ok());
  EXPECT_EQ(s_inf.reason(), "non-finite wheel sample");
  const OdomDelta d = wi.take_delta();
  const OdomDelta ref = run_constant(make_cfg(), T_b_o, 10, kV, kOmega);
  EXPECT_EQ(d.t0, ref.t0);
  EXPECT_EQ(d.t1, ref.t1);
  EXPECT_TRUE(d.T_b0_b1.R == ref.T_b0_b1.R);
  EXPECT_TRUE(d.T_b0_b1.p == ref.T_b0_b1.p);
  EXPECT_TRUE(d.cov == ref.cov);

  // A non-finite FIRST sample does not set the start time.
  WheelIntegrator first(make_cfg(), T_b_o);
  EXPECT_FALSE(first.integrate(make_sample(1'000'000'000, nan, kOmega)).is_ok());
  ASSERT_TRUE(first.integrate(make_sample(2'000'000'000, kV, kOmega)).is_ok());
  ASSERT_TRUE(first.integrate(make_sample(2'000'000'000 + kDtNs, kV, kOmega)).is_ok());
  const OdomDelta df = first.take_delta();
  EXPECT_EQ(df.t0, Timestamp{2'000'000'000});
  EXPECT_EQ(df.t1, Timestamp{2'000'000'000 + kDtNs});
  const Pose expected = T_b_o * arc_pose(kV, kOmega, kDtS) * T_b_o.inverse();
  EXPECT_LT(pose_error(df.T_b0_b1, expected), 1e-12);
}

TEST(WheelIntegrator, RejectsNonIncreasingTime) {
  const Pose T_b_o = make_tbo();
  WheelIntegrator wi(make_cfg(), T_b_o);
  ASSERT_TRUE(wi.integrate(make_sample(0, kV, kOmega)).is_ok());
  ASSERT_TRUE(wi.integrate(make_sample(kDtNs, kV, kOmega)).is_ok());
  const common::Status equal = wi.integrate(make_sample(kDtNs, kV, kOmega));
  EXPECT_FALSE(equal.is_ok());
  EXPECT_EQ(equal.reason(), "wheel timestamp not increasing");
  const common::Status earlier = wi.integrate(make_sample(kDtNs / 2, kV, kOmega));
  EXPECT_FALSE(earlier.is_ok());
  EXPECT_EQ(earlier.reason(), "wheel timestamp not increasing");
  const OdomDelta d = wi.take_delta();
  const OdomDelta ref = run_constant(make_cfg(), T_b_o, 2, kV, kOmega);
  EXPECT_EQ(d.t0, ref.t0);
  EXPECT_EQ(d.t1, ref.t1);
  EXPECT_TRUE(d.T_b0_b1.R == ref.T_b0_b1.R);
  EXPECT_TRUE(d.T_b0_b1.p == ref.T_b0_b1.p);
  EXPECT_TRUE(d.cov == ref.cov);
}

// Review focus 3: yaw increment just below / above kSeriesAngleRad = 1e-4 rad, tiny and
// zero yaw rate.
TEST(WheelIntegrator, SeriesThresholdContinuity) {
  const double omegas[] = {0.99e-4 / kDtS, 1.01e-4 / kDtS, 1e-12, 0.0};  // rad/s
  for (const double omega : omegas) {
    const OdomDelta d = run_constant(make_cfg(), Pose{}, 51, kV, omega);
    EXPECT_TRUE(d.T_b0_b1.R.allFinite() && d.T_b0_b1.p.allFinite()) << omega;
    EXPECT_TRUE(d.cov.allFinite()) << omega;
    EXPECT_LT(pose_error(d.T_b0_b1, arc_pose(kV, omega, 1.0)), 1e-12) << omega;
  }
}

// Review focus 4: more than one full turn between keyframes.
TEST(WheelIntegrator, MultiTurnWraps) {
  const double omega = 1.0;  // rad/s, 8 rad > 2 pi in 8 s
  const OdomDelta d = run_constant(make_cfg(), Pose{}, 401, kV, omega);
  EXPECT_LT(pose_error(d.T_b0_b1, arc_pose(kV, omega, 8.0)), 1e-9);
  EXPECT_EQ(d.t1, Timestamp{8'000'000'000});
}

// ---- Covariance (derivation section 3) ----

TEST(WheelIntegrator, SingleSampleCovKnownAnswer) {
  const OdomDelta d = run_constant(make_cfg(), Pose{}, 2, kV, 0.0);
  const double s = kV * kDtS;  // m
  const double sv2 = kSigmaV * kSigmaV;
  const double so2 = kSigmaOmega * kSigmaOmega;
  const double dt2 = kDtS * kDtS;
  Mat6 expected = Mat6::Zero();
  expected(0, 0) = sv2 * dt2;
  expected(1, 1) = (0.5 * s) * (0.5 * s) * so2 * dt2;
  expected(1, 5) = 0.5 * s * so2 * dt2;
  expected(5, 1) = expected(1, 5);
  expected(5, 5) = so2 * dt2;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (expected(i, j) == 0.0) {
        EXPECT_EQ(d.cov(i, j), 0.0) << i << "," << j;
      } else {
        EXPECT_NEAR(d.cov(i, j), expected(i, j), 1e-18) << i << "," << j;
      }
    }
  }
}

constexpr int kNumDiffSamples = 21;

// Delta for readings v_k, omega_k at t = k * kDtNs; sample 0 only sets the start.
OdomDelta run_readings(const WheelOdomConfig& cfg,
                       const Pose& T_b_o,
                       const double* v,
                       const double* omega) {
  WheelIntegrator wi(cfg, T_b_o);
  for (int k = 0; k < kNumDiffSamples; ++k) {
    EXPECT_TRUE(wi.integrate(make_sample(k * kDtNs, v[k], omega[k])).is_ok());
  }
  return wi.take_delta();
}

// Numerical-difference test of A_k, B_k: cov == sum_k J_k diag(sv^2, so^2) J_k^T, where
// J_k is the derivative of T_b0_b1 (right tangent) w.r.t. the reading (v_k, omega_k).
TEST(WheelIntegrator, CovMatchesNumericalPropagation) {
  constexpr double kH = 1e-6;  // central-difference step (m/s, rad/s)
  const Pose T_b_o = make_tbo();
  const WheelOdomConfig cfg = make_cfg();
  double v[kNumDiffSamples];
  double omega[kNumDiffSamples];
  for (int k = 0; k < kNumDiffSamples; ++k) {
    v[k] = 0.5 + 0.02 * k;
    omega[k] = 0.3 - 0.03 * k;
  }
  const OdomDelta d0 = run_readings(cfg, T_b_o, v, omega);
  Mat6 cov_num = Mat6::Zero();
  for (int k = 1; k < kNumDiffSamples; ++k) {
    for (int which = 0; which < 2; ++which) {
      double* x = which == 0 ? &v[k] : &omega[k];
      const double x0 = *x;
      *x = x0 + kH;
      const Vec6 plus = common::boxminus(run_readings(cfg, T_b_o, v, omega).T_b0_b1, d0.T_b0_b1);
      *x = x0 - kH;
      const Vec6 minus = common::boxminus(run_readings(cfg, T_b_o, v, omega).T_b0_b1, d0.T_b0_b1);
      *x = x0;
      const Vec6 j = (plus - minus) / (2.0 * kH);
      const double sigma = which == 0 ? kSigmaV : kSigmaOmega;
      cov_num += sigma * sigma * j * j.transpose();
    }
  }
  EXPECT_LT((d0.cov - cov_num).norm() / cov_num.norm(), 1e-6);
}

TEST(WheelIntegrator, CovSymmetricPsdSingular) {
  const OdomDelta dt = run_constant(make_cfg(), make_tbo(), 51, kV, kOmega);
  EXPECT_TRUE(dt.cov == dt.cov.transpose());

  const OdomDelta d = run_constant(make_cfg(), Pose{}, 51, kV, kOmega);
  EXPECT_TRUE(d.cov == d.cov.transpose());
  for (const int i : {2, 3, 4}) {
    EXPECT_TRUE(d.cov.row(i).isZero(0.0)) << i;
    EXPECT_TRUE(d.cov.col(i).isZero(0.0)) << i;
  }
  const int idx[3] = {0, 1, 5};
  common::Mat3 block;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      block(i, j) = d.cov(idx[i], idx[j]);
    }
  }
  const Eigen::SelfAdjointEigenSolver<common::Mat3> es(block);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-15);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);  // planar block is full rank after motion
}

TEST(WheelIntegrator, ZeroSigmaGivesZeroCov) {
  const OdomDelta d = run_constant(make_cfg(0.0, 0.0), make_tbo(), 51, kV, kOmega);
  EXPECT_TRUE(d.cov.isZero(0.0));
  const OdomDelta ref = run_constant(make_cfg(), make_tbo(), 51, kV, kOmega);
  EXPECT_TRUE(d.T_b0_b1.R == ref.T_b0_b1.R);
  EXPECT_TRUE(d.T_b0_b1.p == ref.T_b0_b1.p);
}

// Review focus 1: pure rotation in place.
TEST(WheelIntegrator, PureRotationInPlace) {
  const OdomDelta d = run_constant(make_cfg(), Pose{}, 51, 0.0, kOmega);
  EXPECT_TRUE(d.T_b0_b1.p == Vec3::Zero());
  EXPECT_LT(pose_error(d.T_b0_b1, arc_pose(0.0, kOmega, 1.0)), 1e-12);
  const double sv2 = kSigmaV * kSigmaV;
  const double so2 = kSigmaOmega * kSigmaOmega;
  const double dt2 = kDtS * kDtS;
  const double a = kOmega * kDtS;  // rad per sample
  const double yaw_var = 50.0 * so2 * dt2;
  EXPECT_NEAR(d.cov(5, 5), yaw_var, 1e-12 * yaw_var);
  EXPECT_LT(std::abs(d.cov(0, 5)), 1e-20);
  EXPECT_LT(std::abs(d.cov(1, 5)), 1e-20);
  // Speed noise still enters rho; rotations preserve the trace of the rho block.
  const double rho_trace = 50.0 * sv2 * dt2 * (2.0 - 2.0 * std::cos(a)) / (a * a);
  EXPECT_NEAR(d.cov(0, 0) + d.cov(1, 1), rho_trace, 1e-12 * rho_trace);
}

// Review focus 2: reverse driving.
TEST(WheelIntegrator, ReverseDrivingStraight) {
  const OdomDelta back = run_constant(make_cfg(), Pose{}, 51, -kV, 0.0);
  const OdomDelta fwd = run_constant(make_cfg(), Pose{}, 51, kV, 0.0);
  EXPECT_LT((back.T_b0_b1.p - Vec3(-0.8, 0.0, 0.0)).norm(), 1e-12);
  EXPECT_TRUE(back.T_b0_b1.R == common::Mat3::Identity());
  Mat6 expected = fwd.cov;
  expected(1, 5) = -expected(1, 5);
  expected(5, 1) = -expected(5, 1);
  EXPECT_NE(fwd.cov(1, 5), 0.0);
  EXPECT_LT((back.cov - expected).norm() / fwd.cov.norm(), 1e-12);
}

// Review focus 5: 2 s dropout; the next sample's reading is held over the gap.
TEST(WheelIntegrator, LongGapHeldReading) {
  WheelIntegrator wi(make_cfg(), Pose{});
  ASSERT_TRUE(wi.integrate(make_sample(0, kV, 0.0)).is_ok());
  ASSERT_TRUE(wi.integrate(make_sample(kDtNs, kV, 0.0)).is_ok());
  ASSERT_TRUE(wi.integrate(make_sample(kDtNs + 2'000'000'000, kV, 0.0)).is_ok());
  const OdomDelta d = wi.take_delta();
  EXPECT_NEAR(d.T_b0_b1.p.x(), kV * 2.02, 1e-12);
  const double expected = kSigmaV * kSigmaV * (kDtS * kDtS + 2.0 * 2.0);
  EXPECT_NEAR(d.cov(0, 0), expected, 1e-12 * expected);
}

TEST(WheelIntegrator, HotPathNoAllocation) {
  WheelIntegrator wi(make_cfg(), make_tbo());
  ASSERT_TRUE(wi.integrate(make_sample(0, kV, kOmega)).is_ok());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  int ok_count = 0;
  double sum = 0.0;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 1; k <= 1000; ++k) {
      ok_count += wi.integrate(make_sample(k * kDtNs, kV, kOmega)).is_ok() ? 1 : 0;
    }
    ok_count += wi.integrate(make_sample(1001 * kDtNs, nan, kOmega)).is_ok() ? 1 : 0;
    ok_count += wi.integrate(make_sample(0, kV, kOmega)).is_ok() ? 1 : 0;
    for (int i = 0; i < 10; ++i) {
      const OdomDelta d = wi.take_delta();
      sum += d.cov(0, 0) + d.T_b0_b1.p.x();
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_EQ(ok_count, 1000);
  EXPECT_TRUE(std::isfinite(sum));
}

}  // namespace
}  // namespace mss::odom
