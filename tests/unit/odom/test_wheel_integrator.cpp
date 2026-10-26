// WheelIntegrator: exact SE(2) integration of wheel samples into body-frame deltas
// (docs/derivations/wheel_odom.md sections 1-3). Conventions: T_a_b maps b to a, right
// perturbation T <- T * Exp(delta), tangent [rho, phi].
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

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

}  // namespace
}  // namespace mss::odom
