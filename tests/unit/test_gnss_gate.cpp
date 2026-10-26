#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"

#include "alloc_counter.hpp"
#include "innovation.hpp"

namespace {

using mss::common::GnssMeasurement;
using mss::common::Mat3;
using mss::common::Mat6;
using mss::common::Pose;
using mss::common::Vec3;
using mss::common::Vec6;
using mss::gnss::gate_fix;
using mss::gnss::detail::chi_square_gate;
using mss::gnss::detail::gnss_residual;
using mss::gnss::detail::innovation_nis;
using mss::gnss::detail::kChi2Gate3Dof95;
using J36 = Eigen::Matrix<double, 3, 6>;

constexpr double kMaxCovM2 = 25.0;  // config/default.yaml gnss.max_cov_m2
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

GnssMeasurement good_fix() {
  GnssMeasurement m;
  m.p_e_g = Vec3(10.0, -5.0, 2.0);
  m.cov = Vec3(2.25, 2.25, 9.0).asDiagonal();
  m.status = 0;
  return m;
}

// ---------------------------------------------------------------- GnssGate

TEST(GnssGate, AcceptsGoodFixAllStatuses) {
  for (const int status : {0, 1, 2}) {
    GnssMeasurement m = good_fix();
    m.status = static_cast<std::int8_t>(status);
    EXPECT_TRUE(gate_fix(m, kMaxCovM2).is_ok()) << "status " << status;
  }
}

TEST(GnssGate, NoFix) {
  GnssMeasurement m = good_fix();
  m.status = -1;
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "no fix");
}

TEST(GnssGate, NonFinite) {
  GnssMeasurement m = good_fix();
  m.p_e_g.y() = kNan;
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "non-finite fix");
  m = good_fix();
  m.cov(0, 2) = kInf;  // off-diagonal
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "non-finite fix");
}

TEST(GnssGate, CovTooLarge) {
  GnssMeasurement m = good_fix();
  m.cov(2, 2) = 25.000001;
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "fix covariance too large");
  m.cov(2, 2) = 25.0;  // strict >
  EXPECT_TRUE(gate_fix(m, kMaxCovM2).is_ok());
}

TEST(GnssGate, CheckOrder) {
  GnssMeasurement m = good_fix();
  m.status = -1;
  m.p_e_g.x() = kNan;
  m.cov(1, 1) = 100.0;
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "no fix");
  m.status = 0;
  EXPECT_EQ(gate_fix(m, kMaxCovM2).reason(), "non-finite fix");
}

TEST(GnssGate, NegativeVariancePassesGateButNotChiSquare) {  // Review Focus 2
  GnssMeasurement m = good_fix();
  m.cov = Vec3(-1.0, 1.0, 1.0).asDiagonal();
  EXPECT_TRUE(gate_fix(m, kMaxCovM2).is_ok());  // frozen contract: only diag > max is checked
  EXPECT_EQ(chi_square_gate(Vec3(0.1, 0.0, 0.0), m.cov, kChi2Gate3Dof95).reason(),
            "innovation covariance not positive definite");
  m.cov = Mat3::Zero();
  EXPECT_TRUE(gate_fix(m, kMaxCovM2).is_ok());
  EXPECT_EQ(chi_square_gate(Vec3(0.1, 0.0, 0.0), m.cov, kChi2Gate3Dof95).reason(),
            "innovation covariance not positive definite");
}

// ---------------------------------------------------------------- GnssInnovation

TEST(GnssInnovation, NisKnownValue) {
  const Mat3 S = Vec3(1.0, 4.0, 9.0).asDiagonal();
  const auto nis = innovation_nis(Vec3(1.0, 2.0, 3.0), S);
  ASSERT_TRUE(nis.is_ok());
  EXPECT_NEAR(nis.value(), 3.0, 1e-12);
}

TEST(GnssInnovation, GateBoundary) {
  const Vec3 r(1.0, 2.0, 3.0);
  const Mat3 S = Vec3(1.0, 4.0, 9.0).asDiagonal();
  EXPECT_TRUE(chi_square_gate(r, S, 3.0).is_ok());
  EXPECT_EQ(chi_square_gate(r, S, 3.0 - 1e-9).reason(), "chi-square gate exceeded");
}

TEST(GnssInnovation, NonFiniteOrNonPdIsError) {  // Review Focus 1
  const Mat3 S = Mat3::Identity();
  EXPECT_EQ(chi_square_gate(Vec3(kNan, 0.0, 0.0), S, kChi2Gate3Dof95).reason(),
            "non-finite innovation");
  Mat3 S_nan = S;
  S_nan(1, 2) = kNan;
  EXPECT_EQ(chi_square_gate(Vec3(0.1, 0.0, 0.0), S_nan, kChi2Gate3Dof95).reason(),
            "non-finite innovation");
  Mat3 S_indef;
  S_indef << 1.0, 2.0, 0.0,  //
      2.0, 1.0, 0.0,         //
      0.0, 0.0, 1.0;
  EXPECT_EQ(chi_square_gate(Vec3(0.1, 0.0, 0.0), S_indef, kChi2Gate3Dof95).reason(),
            "innovation covariance not positive definite");
  EXPECT_FALSE(innovation_nis(Vec3(0.1, 0.0, 0.0), S_indef).is_ok());
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

TEST(GnssInnovation, ResidualZeroAtTruth) {
  const Pose T_e_w = yaw_pose(0.7, Vec3(120.0, -45.0, 3.0));
  const Pose T_w_b = mss::common::se3_exp((Vec6() << 1.0, 2.0, 0.3, 0.1, -0.2, 0.9).finished());
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  const Vec3 p_e_g = T_e_w * (T_w_b * p_b_g);
  const Vec3 r = gnss_residual(p_e_g, T_e_w, T_w_b, p_b_g, nullptr);
  EXPECT_LT(r.norm(), 1e-12);
}

TEST(GnssInnovation, JacobianMatchesNumeric) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const Vec3 p_b_g(-0.20, 0.10, 0.60);  // ADR-011 lever arm
  constexpr double kH = 1e-6;
  for (int trial = 0; trial < 20; ++trial) {
    const Pose T_e_w = yaw_pose(3.0 * u(rng), Vec3(50.0 * u(rng), 50.0 * u(rng), u(rng)));
    Vec6 xi0;
    xi0 << 10.0 * u(rng), 10.0 * u(rng), u(rng), u(rng), u(rng), 2.0 * u(rng);
    const Pose T_w_b = mss::common::se3_exp(xi0);
    const Vec3 p_e_g(60.0 * u(rng), 60.0 * u(rng), 2.0 * u(rng));
    J36 J;
    (void)gnss_residual(p_e_g, T_e_w, T_w_b, p_b_g, &J);
    J36 J_num;
    for (int i = 0; i < 6; ++i) {
      const Vec6 d = Vec6::Unit(i) * kH;
      const Vec3 rp = gnss_residual(p_e_g, T_e_w, mss::common::boxplus(T_w_b, d), p_b_g, nullptr);
      const Vec3 rm =
          gnss_residual(p_e_g, T_e_w, mss::common::boxplus(T_w_b, Vec6(-d)), p_b_g, nullptr);
      J_num.col(i) = (rp - rm) / (2.0 * kH);
    }
    EXPECT_LT((J_num - J).norm() / J.norm(), 1e-6) << "trial " << trial;
  }
}

TEST(GnssInnovation, NoAllocation) {
  const GnssMeasurement ok_fix = good_fix();
  GnssMeasurement no_fix = good_fix();
  no_fix.status = -1;
  GnssMeasurement nan_fix = good_fix();
  nan_fix.p_e_g.x() = kNan;
  GnssMeasurement big_fix = good_fix();
  big_fix.cov(0, 0) = 30.0;
  const Pose T_e_w = yaw_pose(0.5, Vec3(1.0, 2.0, 3.0));
  const Pose T_w_b = yaw_pose(-0.3, Vec3(4.0, 5.0, 0.0));
  const Vec3 p_b_g(-0.20, 0.10, 0.60);
  const Mat3 S = Mat3::Identity();
  int n_ok = 0;
  double sink = 0.0;
  J36 J;
  AllocCounter::reset();
  {
    EigenMallocGuard guard;
    for (int i = 0; i < 100; ++i) {
      n_ok += gate_fix(ok_fix, kMaxCovM2).is_ok() ? 1 : 0;
      n_ok += gate_fix(no_fix, kMaxCovM2).is_ok() ? 1 : 0;
      n_ok += gate_fix(nan_fix, kMaxCovM2).is_ok() ? 1 : 0;
      n_ok += gate_fix(big_fix, kMaxCovM2).is_ok() ? 1 : 0;
      const Vec3 r = gnss_residual(ok_fix.p_e_g, T_e_w, T_w_b, p_b_g, &J);
      sink += r.x() + J(0, 0);
      n_ok += chi_square_gate(Vec3(0.1, 0.1, 0.1), S, kChi2Gate3Dof95).is_ok() ? 1 : 0;
      n_ok += chi_square_gate(Vec3(10.0, 0.0, 0.0), S, kChi2Gate3Dof95).is_ok() ? 1 : 0;
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0u);
  EXPECT_EQ(n_ok, 200);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
