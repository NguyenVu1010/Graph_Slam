// odom component test: WheelIntegrator + residuals on a synthetic ground-vehicle trajectory
// with a known answer (O-01, O-02 end to end). docs/derivations/wheel_odom.md.
// Truth: piecewise-constant (v, omega) unicycle motion of the odometry frame o at 50 Hz,
// body pose T_w_b = T_w_o * T_b_o^-1 with a non-identity extrinsic.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/odom/config.hpp"
#include "mss/odom/odom_residual.hpp"
#include "mss/odom/wheel_integrator.hpp"

namespace mss::odom {
namespace {

using common::Pose;
using common::Timestamp;
using common::Vec3;

constexpr std::int64_t kDtNs = 20'000'000;  // ns, 50 Hz
constexpr double kDtS = 0.02;               // s
constexpr int kKeyframeEvery = 10;          // samples between keyframes
constexpr double kSlipMps = 0.1;            // m/s, injected lateral velocity

struct Segment {
  int samples;   // number of dt intervals
  double v;      // m/s
  double omega;  // rad/s
};

// Straight 2 s; arc 3 s; in-place turn 2 s; reverse 1 s.
const Segment kSegments[] = {{100, 0.8, 0.0}, {150, 0.8, 0.5}, {100, 0.0, -1.0}, {50, -0.5, 0.0}};

Pose make_tbo() {
  return Pose{common::so3_exp(Vec3(0.1, -0.2, 0.3)), Vec3(0.15, -0.05, 0.10)};
}

// Analytic unicycle pose of o after t_s at constant (v, omega), from the origin.
Pose arc_pose(double v_mps, double omega_radps, double t_s) {
  const double yaw = omega_radps * t_s;
  double x = v_mps * t_s;
  double y = 0.0;
  if (omega_radps != 0.0) {
    const double half = std::sin(0.5 * yaw);
    x = v_mps / omega_radps * std::sin(yaw);
    y = v_mps / omega_radps * 2.0 * half * half;
  }
  return Pose{common::so3_exp(Vec3(0.0, 0.0, yaw)), Vec3(x, y, 0.0)};
}

struct Truth {
  std::vector<Pose> T_w_o;  // per sample index
  std::vector<double> v;    // reading of sample k (held over (k-1, k]); k = 0 unused
  std::vector<double> omega;
};

Truth make_truth() {
  Truth truth;
  // Non-trivial start pose of o in w.
  Pose seg_start{common::so3_exp(Vec3(0.0, 0.0, 0.7)), Vec3(10.0, -3.0, 1.0)};
  truth.T_w_o.push_back(seg_start);
  truth.v.push_back(kSegments[0].v);
  truth.omega.push_back(kSegments[0].omega);
  for (const Segment& seg : kSegments) {
    for (int i = 1; i <= seg.samples; ++i) {
      truth.T_w_o.push_back(seg_start * arc_pose(seg.v, seg.omega, i * kDtS));
      truth.v.push_back(seg.v);
      truth.omega.push_back(seg.omega);
    }
    seg_start = truth.T_w_o.back();
  }
  return truth;
}

// Body kinematics at sample k with o-frame velocity v_o (m/s); the angular rate is the
// reading of the interval that follows k (the last interval for the final sample).
struct BodyState {
  Pose T_w_b;
  Vec3 v_w_b;
  Vec3 omega_b;
};

BodyState body_state(const Truth& truth, const Pose& T_b_o, std::size_t k, double lateral) {
  const std::size_t next = k + 1 < truth.v.size() ? k + 1 : k;
  BodyState s;
  s.T_w_b = truth.T_w_o[k] * T_b_o.inverse();
  s.omega_b = T_b_o.R * Vec3(0.0, 0.0, truth.omega[next]);
  const Vec3 v_o(truth.v[next], lateral, 0.0);
  s.v_w_b = s.T_w_b.R * (T_b_o.R * v_o - s.omega_b.cross(T_b_o.p));
  return s;
}

TEST(OdomComponent, KeyframeChainOnSyntheticTrajectory) {
  const Truth truth = make_truth();
  const Pose T_b_o = make_tbo();
  WheelOdomConfig cfg;
  cfg.speed_sigma_mps = 0.02;
  cfg.yaw_rate_sigma_radps = 0.02;
  WheelIntegrator wi(cfg, T_b_o);

  Pose chain;  // product of all deltas
  std::size_t prev_kf = 0;
  int keyframes = 0;
  for (std::size_t k = 0; k < truth.T_w_o.size(); ++k) {
    common::WheelSample s;
    s.t = Timestamp{static_cast<std::int64_t>(k) * kDtNs};
    s.v_mps = truth.v[k];
    s.omega_radps = truth.omega[k];
    ASSERT_TRUE(wi.integrate(s).is_ok());
    if (k % kKeyframeEvery != 0) {
      continue;
    }
    const BodyState bs = body_state(truth, T_b_o, k, 0.0);
    EXPECT_LT(nonholonomic_residual(bs.T_w_b, bs.v_w_b, bs.omega_b, T_b_o).r.norm(), 1e-12) << k;
    if (k == 0) {
      continue;
    }
    const common::OdomDelta d = wi.take_delta();
    const Pose T_w_bi = truth.T_w_o[prev_kf] * T_b_o.inverse();
    EXPECT_LT(relative_pose_residual(T_w_bi, bs.T_w_b, d.T_b0_b1).r.norm(), 1e-9) << k;
    EXPECT_EQ(d.t0, Timestamp{static_cast<std::int64_t>(prev_kf) * kDtNs});
    EXPECT_EQ(d.t1, Timestamp{static_cast<std::int64_t>(k) * kDtNs});
    EXPECT_TRUE(d.cov.allFinite());
    chain = chain * d.T_b0_b1;
    prev_kf = k;
    ++keyframes;
  }
  EXPECT_EQ(keyframes, 40);
  const Pose T_w_b0 = truth.T_w_o.front() * T_b_o.inverse();
  const Pose T_w_bn = truth.T_w_o.back() * T_b_o.inverse();
  EXPECT_LT(common::boxminus(chain, T_w_b0.inverse() * T_w_bn).norm(), 1e-9);
}

TEST(OdomComponent, InjectedLateralSlipShowsInResidual) {
  const Truth truth = make_truth();
  const Pose T_b_o = make_tbo();
  for (std::size_t k = 0; k < truth.T_w_o.size(); k += kKeyframeEvery) {
    const BodyState bs = body_state(truth, T_b_o, k, kSlipMps);
    const NonholonomicResidual res = nonholonomic_residual(bs.T_w_b, bs.v_w_b, bs.omega_b, T_b_o);
    EXPECT_NEAR(res.r(0), kSlipMps, 1e-12) << k;
    EXPECT_NEAR(res.r(1), 0.0, 1e-12) << k;
  }
}

}  // namespace
}  // namespace mss::odom
