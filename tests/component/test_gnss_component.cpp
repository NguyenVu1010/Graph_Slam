#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"

#include "innovation.hpp"

namespace {

using mss::common::GnssMeasurement;
using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Vec3;
using mss::gnss::gate_fix;
using mss::gnss::GnssAligner;
using mss::gnss::GnssConfig;
using mss::gnss::detail::chi_square_gate;
using mss::gnss::detail::gnss_residual;
using mss::gnss::detail::kChi2Gate3Dof95;

constexpr double kMaxCovM2 = 25.0;  // config/default.yaml gnss.max_cov_m2

TEST(GnssComponent, G02_ChiSquareRejectsJump) {
  // Test_CI_CD G-02. 1000 fixes along x at 0.2 m spacing (planner's call), nominal noise
  // sigma_h = 1.0 m, sigma_v = 2.0 m (ADR-011), one 5 m jump at fix 500.
  constexpr int kFixes = 1000;
  constexpr int kJump = 500;
  constexpr int kNoFix = 100;
  constexpr int kBigCov = 200;
  std::mt19937 rng(42);
  std::normal_distribution<double> n01(0.0, 1.0);
  const Vec3 sigma_m(1.0, 1.0, 2.0);
  int accepted = 0;
  int normal = 0;
  for (int k = 0; k < kFixes; ++k) {
    const Vec3 truth(0.2 * k, 0.0, 0.0);
    GnssMeasurement m;
    m.p_e_g = truth + Vec3(sigma_m.x() * n01(rng), sigma_m.y() * n01(rng), sigma_m.z() * n01(rng));
    m.cov = Vec3(1.0, 1.0, 4.0).asDiagonal();
    m.status = 0;
    if (k == kJump) {
      m.p_e_g.x() += 5.0;
    }
    if (k == kNoFix) {
      m.status = -1;
    }
    if (k == kBigCov) {
      m.cov(0, 0) = 36.0;
    }
    const mss::common::Status pre = gate_fix(m, kMaxCovM2);
    if (k == kNoFix) {
      EXPECT_EQ(pre.reason(), "no fix");
      continue;
    }
    if (k == kBigCov) {
      EXPECT_EQ(pre.reason(), "fix covariance too large");
      continue;
    }
    ASSERT_TRUE(pre.is_ok()) << "fix " << k;
    Pose T_w_b;
    T_w_b.p = truth;
    const Vec3 r = gnss_residual(m.p_e_g, Pose{}, T_w_b, Vec3::Zero(), nullptr);
    const Mat3 S = m.cov;  // exact prediction, P = 0
    const mss::common::Status gate = chi_square_gate(r, S, kChi2Gate3Dof95);
    if (k == kJump) {
      EXPECT_EQ(gate.reason(), "chi-square gate exceeded");
      continue;
    }
    ++normal;
    accepted += gate.is_ok() ? 1 : 0;
  }
  ASSERT_EQ(normal, 997);
  const double acceptance = static_cast<double>(accepted) / normal;
  EXPECT_GE(acceptance, 0.90) << "acceptance " << acceptance;
  EXPECT_LE(acceptance, 0.99) << "acceptance " << acceptance;
  RecordProperty("acceptance", std::to_string(acceptance));
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kStepM = 0.2;  // m per fix: 1 m/s at 5 Hz (ADR-011)

double wrap_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

double yaw_of(const Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

GnssConfig aligner_cfg() {
  GnssConfig c;
  c.enabled = true;
  c.sigma_h_m = 1.0;
  c.sigma_v_m = 2.0;
  c.max_cov_m2 = 25.0;
  return c;
}

TEST(GnssComponent, G03_YawAlignedWithin0p5DegAfter30m) {
  // Test_CI_CD G-03: trajectory rotated 30 deg from ENU. Straight path, heading 10 deg in w,
  // 0.2 m per fix, lever arm from ADR-011, 151 fixes (30 m). Noise sigma_h = 0.2 m,
  // sigma_v = 0.4 m (planner's call: at the nominal 1.0 m the 1-sigma yaw error after 30 m
  // is ~0.54 deg, docs/derivations/gnss.md section 4).
  const double yaw_true = 30.0 * kDeg;
  const Vec3 p_true(120.0, -45.0, 3.0);
  const Pose T_e_w = yaw_pose(yaw_true, p_true);
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  const double heading = 10.0 * kDeg;
  const Vec3 dir(std::cos(heading), std::sin(heading), 0.0);
  constexpr int kLastFix = 150;
  double worst_yaw_deg = 0.0;
  for (unsigned seed = 0; seed < 20; ++seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    GnssAligner aligner(aligner_cfg());
    for (int k = 0; k <= kLastFix; ++k) {
      const Pose T_w_b = yaw_pose(heading, Vec3(kStepM * k * dir));
      const Vec3 noise(0.2 * n01(rng), 0.2 * n01(rng), 0.4 * n01(rng));
      const Vec3 p_e_g = T_e_w * (T_w_b * p_b_g) + noise;
      const mss::common::Result<Pose> res = aligner.update(p_e_g, T_w_b, p_b_g);
      if (kStepM * k >= 30.0 - 1e-9) {
        ASSERT_TRUE(res.is_ok()) << "seed " << seed << ": " << res.reason();
        const double yaw_err = std::abs(wrap_pi(yaw_of(res.value().R) - yaw_true));
        worst_yaw_deg = std::max(worst_yaw_deg, yaw_err / kDeg);
        EXPECT_LT(yaw_err, 0.5 * kDeg) << "seed " << seed;
        EXPECT_LT((res.value().p - p_true).norm(), 0.5) << "seed " << seed;
      }
    }
  }
  RecordProperty("worst_yaw_err_deg", std::to_string(worst_yaw_deg));
}

// Runs phase A (straight 20 m) and phase B (rotation in place, two turns) and returns the
// maximum translation change of the T_e_w estimate during phase B. When `check` is set,
// also asserts the noise-free G-04 expectations.
double run_g04(const Vec3& p_b_g_true, const Vec3& p_b_g_used, bool check) {
  const Pose T_e_w = yaw_pose(30.0 * kDeg, Vec3(120.0, -45.0, 3.0));
  GnssAligner aligner(aligner_cfg());
  mss::common::Result<Pose> res = mss::common::Result<Pose>::error("none");
  constexpr int kPhaseAFixes = 101;  // 20 m at 0.2 m per fix
  for (int k = 0; k < kPhaseAFixes; ++k) {
    const Pose T_w_b = yaw_pose(0.0, Vec3(kStepM * k, 0.0, 0.0));
    res = aligner.update(T_e_w * (T_w_b * p_b_g_true), T_w_b, p_b_g_used);
  }
  if (!res.is_ok()) {
    ADD_FAILURE() << "phase A did not align: " << res.reason();
    return 0.0;
  }
  const Pose T_a = res.value();
  const Vec3 p_w_b(kStepM * (kPhaseAFixes - 1), 0.0, 0.0);
  const Vec3 p_e_b_true = T_e_w * p_w_b;
  constexpr int kPhaseBSteps = 80;
  double max_dp_m = 0.0;
  for (int j = 0; j <= kPhaseBSteps; ++j) {
    const double yaw_b = 4.0 * kPi * j / kPhaseBSteps;
    const Pose T_w_b = yaw_pose(yaw_b, p_w_b);
    const Vec3 p_e_g = T_e_w * (T_w_b * p_b_g_true);
    res = aligner.update(p_e_g, T_w_b, p_b_g_used);
    if (!res.is_ok()) {
      ADD_FAILURE() << "phase B step " << j << ": " << res.reason();
      return max_dp_m;
    }
    const Pose& est = res.value();
    max_dp_m = std::max(max_dp_m, (est.p - T_a.p).norm());
    if (check) {
      EXPECT_LT((est.p - T_a.p).norm(), 1e-9) << "step " << j;
      EXPECT_LT((est.R - T_a.R).norm(), 1e-9) << "step " << j;
      const Vec3 p_e_b = p_e_g - est.R * (T_w_b.R * p_b_g_used);
      EXPECT_LT((p_e_b - p_e_b_true).norm(), 1e-9) << "step " << j;
    }
  }
  return max_dp_m;
}

TEST(GnssComponent, G04_LeverArmRotationInPlace) {
  // Test_CI_CD G-04: antenna 0.5 m off the body origin, robot rotates in place. Noise-free.
  const Vec3 p_b_g(0.5, 0.0, 0.0);
  (void)run_g04(p_b_g, p_b_g, true);
  // Negative control: ignoring the lever arm moves the estimate during the rotation.
  const double max_dp_m = run_g04(p_b_g, Vec3::Zero(), false);
  EXPECT_GT(max_dp_m, 0.01);
}

}  // namespace
