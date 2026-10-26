// O-01, O-02: wheel-odometry residuals and their analytic Jacobians against central
// differences (docs/derivations/wheel_odom.md sections 4-5). Right perturbation
// T <- T * Exp(xi), xi = [rho, phi]; v_w_b perturbed additively.
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/odom/odom_residual.hpp"

#include "alloc_counter.hpp"

namespace mss::odom {
namespace {

using common::Mat3;
using common::Mat6;
using common::Pose;
using common::Vec3;
using common::Vec6;

constexpr double kPi = 3.14159265358979323846;
constexpr double kH = 1e-6;           // central-difference step
constexpr double kRelTol = 1e-6;      // relative Frobenius error (CLAUDE.md)
constexpr unsigned kSeed = 20261026;  // fixed seed
constexpr int kRandomCases = 200;
constexpr int kSmallAngleCases = 20;

Pose make_tbo() {
  return Pose{common::so3_exp(Vec3(0.1, -0.2, 0.3)), Vec3(0.15, -0.05, 0.10)};
}

Vec3 random_axis(std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Vec3 axis(n(rng), n(rng), n(rng));
  while (axis.norm() < 1e-6) {
    axis = Vec3(n(rng), n(rng), n(rng));
  }
  return axis.normalized();
}

// |phi| in [0, pi - 0.1] rad, rho components in [-5, 5] m.
Pose random_pose(std::mt19937& rng) {
  std::uniform_real_distribution<double> angle(0.0, kPi - 0.1);
  std::uniform_real_distribution<double> pos(-5.0, 5.0);
  const Vec3 phi = angle(rng) * random_axis(rng);
  return Pose{common::so3_exp(phi), Vec3(pos(rng), pos(rng), pos(rng))};
}

// xi with |rho_i| <= rho_max (m), |phi| <= phi_max (rad).
Vec6 random_xi(std::mt19937& rng, double rho_max, double phi_max) {
  std::uniform_real_distribution<double> rho(-rho_max, rho_max);
  std::uniform_real_distribution<double> angle(0.0, phi_max);
  Vec6 xi;
  xi.head<3>() = Vec3(rho(rng), rho(rng), rho(rng));
  xi.tail<3>() = angle(rng) * random_axis(rng);
  return xi;
}

template <typename A, typename B>
double rel_err(const Eigen::MatrixBase<A>& analytic, const Eigen::MatrixBase<B>& numeric) {
  return (analytic - numeric).norm() / numeric.norm();
}

// Numerical Jacobians of the relative-pose residual w.r.t. right perturbations of T_w_bi
// (which == 0) or T_w_bj (which == 1).
Mat6 num_jac_rel(const Pose& T_w_bi, const Pose& T_w_bj, const Pose& Z, int which) {
  Mat6 J = Mat6::Zero();
  for (int k = 0; k < 6; ++k) {
    Vec6 d = Vec6::Zero();
    d(k) = kH;
    const Pose Ti_p = which == 0 ? common::boxplus(T_w_bi, d) : T_w_bi;
    const Pose Tj_p = which == 1 ? common::boxplus(T_w_bj, d) : T_w_bj;
    const Pose Ti_m = which == 0 ? common::boxplus(T_w_bi, Vec6(-d)) : T_w_bi;
    const Pose Tj_m = which == 1 ? common::boxplus(T_w_bj, Vec6(-d)) : T_w_bj;
    const Vec6 rp = relative_pose_residual(Ti_p, Tj_p, Z).r;
    const Vec6 rm = relative_pose_residual(Ti_m, Tj_m, Z).r;
    J.col(k) = (rp - rm) / (2.0 * kH);
  }
  return J;
}

void check_rel_jacobians(const Pose& T_w_bi, const Pose& T_w_bj, const Pose& Z) {
  const RelativePoseResidual res = relative_pose_residual(T_w_bi, T_w_bj, Z);
  const Mat6 J_i = num_jac_rel(T_w_bi, T_w_bj, Z, 0);
  const Mat6 J_j = num_jac_rel(T_w_bi, T_w_bj, Z, 1);
  EXPECT_LT(rel_err(res.J_r_xi_i, J_i), kRelTol);
  EXPECT_LT(rel_err(res.J_r_xi_j, J_j), kRelTol);
}

TEST(OdomResidual, O01_ConsistentMotionZeroResidual) {
  std::mt19937 rng(kSeed);
  for (int n = 0; n < kRandomCases; ++n) {
    const Pose T_w_bi = random_pose(rng);
    const Pose Z = random_pose(rng);
    const Pose T_w_bj = T_w_bi * Z;
    const RelativePoseResidual res = relative_pose_residual(T_w_bi, T_w_bj, Z);
    EXPECT_LT(res.r.norm(), 1e-12);
    EXPECT_LT((res.J_r_xi_j - Mat6::Identity()).norm(), 1e-12);
    check_rel_jacobians(T_w_bi, T_w_bj, Z);
  }
}

TEST(OdomResidual, O01_JacobiansMatchNumerical) {
  std::mt19937 rng(kSeed);
  for (int n = 0; n < kRandomCases; ++n) {
    const Pose T_w_bi = random_pose(rng);
    const Pose Z0 = random_pose(rng);
    const Pose T_w_bj = T_w_bi * Z0;
    const Vec6 xi = random_xi(rng, 0.2, 0.3);  // |xi| <= 0.5
    const Pose Z = common::boxplus(Z0, xi);
    ASSERT_GT(relative_pose_residual(T_w_bi, T_w_bj, Z).r.norm(), 0.0);
    check_rel_jacobians(T_w_bi, T_w_bj, Z);
  }
  // Residual rotation below kSmallAngleQRad = 1e-3 rad: series branch of Q.
  for (int n = 0; n < kSmallAngleCases; ++n) {
    const Pose T_w_bi = random_pose(rng);
    const Pose Z0 = random_pose(rng);
    const Pose T_w_bj = T_w_bi * Z0;
    const Vec6 xi = random_xi(rng, 0.5, 0.9e-3);
    const Pose Z = common::boxplus(Z0, xi);
    const RelativePoseResidual res = relative_pose_residual(T_w_bi, T_w_bj, Z);
    ASSERT_LT(res.r.tail<3>().norm(), 1e-3);
    check_rel_jacobians(T_w_bi, T_w_bj, Z);
  }
}

TEST(OdomResidual, O02_ResidualProportionalToLateralVelocity) {
  std::mt19937 rng(kSeed);
  const Pose T_b_o = make_tbo();
  const Vec3 omega_b(0.05, -0.02, 0.4);  // rad/s
  for (int n = 0; n < 20; ++n) {
    const Pose T_w_b = random_pose(rng);
    for (const double v_lat : {0.0, 0.05, 0.1, 0.2}) {  // m/s
      const Vec3 v_o(0.8, v_lat, 0.0);
      const Vec3 v_w_b = T_w_b.R * (T_b_o.R * v_o - omega_b.cross(T_b_o.p));
      const NonholonomicResidual res = nonholonomic_residual(T_w_b, v_w_b, omega_b, T_b_o);
      EXPECT_NEAR(res.r(0), v_lat, 1e-12);
      EXPECT_NEAR(res.r(1), 0.0, 1e-12);
      if (v_lat > 0.0) {
        EXPECT_NEAR(res.r(0) / v_lat, 1.0, 1e-12);
      }
    }
  }
}

TEST(OdomResidual, O02_JacobiansMatchNumerical) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> vel(-2.0, 2.0);   // m/s
  std::uniform_real_distribution<double> rate(-1.0, 1.0);  // rad/s
  const Pose T_b_o = make_tbo();
  for (int n = 0; n < kRandomCases; ++n) {
    const Pose T_w_b = random_pose(rng);
    const Vec3 v_w_b(vel(rng), vel(rng), vel(rng));
    const Vec3 omega_b(rate(rng), rate(rng), rate(rng));
    const NonholonomicResidual res = nonholonomic_residual(T_w_b, v_w_b, omega_b, T_b_o);

    Eigen::Matrix<double, 2, 6> J_xi = Eigen::Matrix<double, 2, 6>::Zero();
    for (int k = 0; k < 6; ++k) {
      Vec6 d = Vec6::Zero();
      d(k) = kH;
      const Eigen::Vector2d rp =
          nonholonomic_residual(common::boxplus(T_w_b, d), v_w_b, omega_b, T_b_o).r;
      const Eigen::Vector2d rm =
          nonholonomic_residual(common::boxplus(T_w_b, Vec6(-d)), v_w_b, omega_b, T_b_o).r;
      J_xi.col(k) = (rp - rm) / (2.0 * kH);
    }
    Eigen::Matrix<double, 2, 3> J_v = Eigen::Matrix<double, 2, 3>::Zero();
    for (int k = 0; k < 3; ++k) {
      Vec3 d = Vec3::Zero();
      d(k) = kH;
      const Eigen::Vector2d rp = nonholonomic_residual(T_w_b, Vec3(v_w_b + d), omega_b, T_b_o).r;
      const Eigen::Vector2d rm = nonholonomic_residual(T_w_b, Vec3(v_w_b - d), omega_b, T_b_o).r;
      J_v.col(k) = (rp - rm) / (2.0 * kH);
    }
    EXPECT_LT(rel_err(res.J_r_xi, J_xi), kRelTol);
    EXPECT_LT(rel_err(res.J_r_v, J_v), kRelTol);
    EXPECT_TRUE(res.J_r_xi.leftCols<3>().isZero(0.0));
  }
}

TEST(OdomResidual, NoAllocation) {
  std::mt19937 rng(kSeed);
  const Pose T_w_bi = random_pose(rng);
  const Pose T_w_bj = random_pose(rng);
  const Pose Z = random_pose(rng);
  const Pose T_b_o = make_tbo();
  const Vec3 v_w_b(0.8, 0.1, -0.05);
  const Vec3 omega_b(0.05, -0.02, 0.4);
  double sum = 0.0;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int n = 0; n < 100; ++n) {
      const RelativePoseResidual a = relative_pose_residual(T_w_bi, T_w_bj, Z);
      const NonholonomicResidual b = nonholonomic_residual(T_w_bi, v_w_b, omega_b, T_b_o);
      sum += a.r(0) + a.J_r_xi_i(0, 0) + b.r(0) + b.J_r_v(0, 0);
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(std::isfinite(sum));
}

}  // namespace
}  // namespace mss::odom
