// C-01, C-02: Lie group wrappers. Conventions: docs/derivations/lie_conventions.md
// (right perturbation T <- T * Exp(delta), tangent order [rho, phi]).
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

namespace mss::common {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNumDiffStep = 1e-6;  // h for numerical differentiation
constexpr int kRoundTripSamples = 10000;
constexpr int kEdgeSamples = 100;

Vec3 random_unit_axis(std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Vec3 axis(n(rng), n(rng), n(rng));
  while (axis.norm() < 1e-6) {
    axis = Vec3(n(rng), n(rng), n(rng));
  }
  return axis.normalized();
}

// phi = random unit axis * angle, angle ~ U(lo, hi).
Vec3 random_phi(std::mt19937& rng, double lo, double hi) {
  std::uniform_real_distribution<double> angle(lo, hi);
  const double theta = angle(rng);
  const Vec3 axis = random_unit_axis(rng);
  return Vec3(theta * axis);
}

Vec3 random_rho(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-5.0, 5.0);  // m
  return Vec3(u(rng), u(rng), u(rng));
}

Pose random_pose(std::mt19937& rng) {
  Pose T;
  T.R = so3_exp(random_phi(rng, 0.0, kPi - 1e-6));
  T.p = random_rho(rng);
  return T;
}

// All phi samples for the round-trip tests: bulk, near 0, near pi.
std::vector<Vec3> round_trip_phis(std::mt19937& rng) {
  std::vector<Vec3> phis;
  phis.reserve(kRoundTripSamples + 2 * kEdgeSamples);
  for (int i = 0; i < kRoundTripSamples; ++i) {
    phis.push_back(random_phi(rng, 0.0, kPi - 1e-6));
  }
  for (int i = 0; i < kEdgeSamples; ++i) {
    phis.push_back(random_phi(rng, 0.0, 1e-10));
  }
  for (int i = 0; i < kEdgeSamples; ++i) {
    phis.push_back(random_phi(rng, kPi - 1e-6, kPi - 1e-9));
  }
  return phis;
}

double relative_frobenius_error(const Mat3& actual, const Mat3& expected) {
  return (actual - expected).norm() / expected.norm();
}

TEST(Lie, So3ExpLogRoundTrip) {
  std::mt19937 rng(42);
  for (const Vec3& phi : round_trip_phis(rng)) {
    const Vec3 phi_back = so3_log(so3_exp(phi));
    EXPECT_LT((phi_back - phi).norm(), 1e-9) << "phi = " << phi.transpose();
  }
}

TEST(Lie, Se3ExpLogRoundTrip) {
  std::mt19937 rng(42);
  for (const Vec3& phi : round_trip_phis(rng)) {
    Vec6 xi;
    xi << random_rho(rng), phi;  // [rho, phi]
    const Vec6 xi_back = se3_log(se3_exp(xi));
    EXPECT_LT((xi_back - xi).norm(), 1e-9) << "xi = " << xi.transpose();
  }
}

// Exp(phi + dphi) ~= Exp(phi) * Exp(Jr(phi) * dphi). Column k of Jr is
// Log(Exp(phi)^T * Exp(phi + h e_k)) / h. A central difference is used so that the
// O(h) truncation error of the forward difference does not eat the 1e-6 budget.
TEST(Lie, JrMatchesNumericDiff) {
  std::mt19937 rng(42);
  for (int i = 0; i < 200; ++i) {
    const Vec3 phi = random_phi(rng, 0.0, kPi - 1e-3);
    const Mat3 R_t = so3_exp(phi).transpose();
    Mat3 J_num;
    for (int k = 0; k < 3; ++k) {
      const Vec3 step = kNumDiffStep * Vec3::Unit(k);
      const Vec3 plus = so3_log(R_t * so3_exp(Vec3(phi + step)));
      const Vec3 minus = so3_log(R_t * so3_exp(Vec3(phi - step)));
      J_num.col(k) = (plus - minus) / (2.0 * kNumDiffStep);
    }
    EXPECT_LT(relative_frobenius_error(so3_jr(phi), J_num), 1e-6) << "phi = " << phi.transpose();
  }
}

TEST(Lie, JrInvIsInverseOfJr) {
  std::mt19937 rng(42);
  for (int i = 0; i < 200; ++i) {
    const Vec3 phi = random_phi(rng, 0.0, kPi - 1e-6);
    const Mat3 product = so3_jr(phi) * so3_jr_inv(phi);
    EXPECT_TRUE(product.isApprox(Mat3::Identity(), 1e-9)) << "phi = " << phi.transpose();
    const Mat3 product_l = so3_jl(phi) * so3_jl_inv(phi);
    EXPECT_TRUE(product_l.isApprox(Mat3::Identity(), 1e-9)) << "phi = " << phi.transpose();
  }
}

TEST(Lie, JlEqualsJrOfNegPhi) {
  std::mt19937 rng(42);
  for (int i = 0; i < 200; ++i) {
    const Vec3 phi = random_phi(rng, 0.0, kPi - 1e-6);
    EXPECT_LT((so3_jl(phi) - so3_jr(Vec3(-phi))).norm(), 1e-12);
    EXPECT_LT((so3_jl_inv(phi) - so3_jr_inv(Vec3(-phi))).norm(), 1e-12);
  }
}

TEST(Lie, SkewIsCrossProduct) {
  const Vec3 a(0.3, -1.2, 2.5);
  const Vec3 b(-0.7, 0.4, 1.1);
  EXPECT_LT((skew(a) * b - a.cross(b)).norm(), 1e-15);
}

// T * Exp(xi) * T^-1 = Exp(Ad(T) * xi).
TEST(Lie, AdjointMatchesConjugation) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 200; ++i) {
    const Pose T = random_pose(rng);
    Vec6 xi;
    xi << u(rng), u(rng), u(rng), u(rng), u(rng), u(rng);
    xi *= 1e-3 / xi.norm();
    const Vec6 expected = se3_log(T * se3_exp(xi) * T.inverse());
    const Vec6 actual = se3_adj(T) * xi;
    EXPECT_LT((actual - expected).norm(), 1e-9);
  }
}

TEST(Lie, BoxplusBoxminus) {
  std::mt19937 rng(42);
  for (int i = 0; i < 1000; ++i) {
    const Pose T = random_pose(rng);
    Vec6 d;
    d << random_rho(rng), random_phi(rng, 0.0, kPi - 1e-3);
    EXPECT_LT((boxminus(boxplus(T, d), T) - d).norm(), 1e-9) << "d = " << d.transpose();
    // boxplus is the right perturbation T * Exp(d).
    const Pose expected = T * se3_exp(d);
    const Pose actual = boxplus(T, d);
    EXPECT_TRUE(actual.R.isApprox(expected.R, 1e-12));
    EXPECT_TRUE(actual.p.isApprox(expected.p, 1e-12));
  }
}

TEST(Lie, TransformCovarianceIsCongruence) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 100; ++i) {
    const Pose T_y_x = random_pose(rng);
    Mat6 A;
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        A(r, c) = u(rng);
      }
    }
    const PoseCovariance cov_x = A * A.transpose() + 1e-3 * Mat6::Identity();
    const Mat6 Ad = se3_adj(T_y_x);
    const PoseCovariance expected = Ad * cov_x * Ad.transpose();
    const PoseCovariance cov_y = transform_covariance(cov_x, T_y_x);
    EXPECT_LT((cov_y - expected).norm(), 1e-12 * expected.norm());
    EXPECT_EQ(cov_y, cov_y.transpose());
  }
}

TEST(Lie, SophusRoundTrip) {
  std::mt19937 rng(42);
  for (int i = 0; i < 1000; ++i) {
    const Pose T = random_pose(rng);
    const Pose T_back = from_sophus(to_sophus(T));
    EXPECT_LT((T_back.R - T.R).norm(), 1e-12);
    EXPECT_LT((T_back.p - T.p).norm(), 1e-12);
  }
}

}  // namespace
}  // namespace mss::common
