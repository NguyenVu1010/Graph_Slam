#include <cstdint>
#include <random>

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
}

}  // namespace
