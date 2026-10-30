#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "alloc_counter.hpp"

namespace mss::degeneracy {

namespace {

// B with entries N(0, 1) from std::mt19937(seed); returns B * B^T + 1e-3 * I.
common::Mat6 random_spd(std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> dist(0.0, 1.0);
  common::Mat6 B;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      B(i, j) = dist(rng);
    }
  }
  const common::Mat6 H = B * B.transpose() + 1e-3 * common::Mat6::Identity();
  return H;
}

common::Mat6 diag6(double a, double b, double c, double d, double e, double f) {
  common::Vec6 v;
  v << a, b, c, d, e, f;
  const common::Mat6 H = v.asDiagonal();
  return H;
}

bool bitwise_equal(const common::DegeneracyReport& a, const common::DegeneracyReport& b) {
  return (a.eigvals.array() == b.eigvals.array()).all() &&
         (a.eigvecs.array() == b.eigvecs.array()).all() && a.degenerate == b.degenerate;
}

constexpr DegeneracyConfig kCfg{true, 100.0, 10.0};

// Report of random_spd(13) with directions 0 and 2 flagged by hand.
common::DegeneracyReport report_flags02() {
  common::DegeneracyReport r = analyze(random_spd(13), DegeneracyConfig{false, 0.0, 10.0});
  r.degenerate[0] = true;
  r.degenerate[2] = true;
  return r;
}

common::Vec6 random_delta() {
  std::mt19937 rng(21);
  std::normal_distribution<double> dist(0.0, 1.0);
  common::Vec6 d;
  for (int i = 0; i < 6; ++i) {
    d(i) = dist(rng);
  }
  return d;
}

common::DegeneracyReport all_flagged() {
  common::DegeneracyReport r;
  r.degenerate.fill(true);
  return r;
}

bool is_flagged_index(int k) {
  return k == 0 || k == 2;
}

}  // namespace

TEST(Analyze, DiagonalSortedAscendingUnitNorm) {
  const common::Mat6 H = diag6(5, 1, 4, 2, 6, 3);
  const common::DegeneracyReport r = analyze(H, kCfg);
  const std::array<int, 6> perm{1, 3, 5, 2, 0, 4};
  for (int k = 0; k < 6; ++k) {
    EXPECT_NEAR(r.eigvals(k), static_cast<double>(k + 1), 1e-12);
    EXPECT_NEAR(std::abs(r.eigvecs(perm[static_cast<std::size_t>(k)], k)), 1.0, 1e-12);
  }
  const common::Mat6 rec = r.eigvecs * r.eigvals.asDiagonal() * r.eigvecs.transpose();
  EXPECT_LT((rec - H).norm(), 1e-12);
}

TEST(Analyze, RandomSpdReconstructs) {
  const common::Mat6 H = random_spd(7);
  const common::DegeneracyReport r = analyze(H, kCfg);
  const common::Mat6 rec = r.eigvecs * r.eigvals.asDiagonal() * r.eigvecs.transpose();
  EXPECT_LT((rec - H).norm(), 1e-9 * H.norm());
  const common::Mat6 vtv = r.eigvecs.transpose() * r.eigvecs;
  EXPECT_LT((vtv - common::Mat6::Identity()).norm(), 1e-12);
  for (int k = 0; k < 5; ++k) {
    EXPECT_LE(r.eigvals(k), r.eigvals(k + 1));
  }
}

TEST(Analyze, FlagsBelowThresholdOnlyWhenEnabled) {
  const common::Mat6 H = diag6(50, 150, 1e3, 1e3, 1e3, 1e3);
  const common::DegeneracyReport on = analyze(H, kCfg);
  const std::array<bool, 6> expected{true, false, false, false, false, false};
  EXPECT_EQ(on.degenerate, expected);
  const common::DegeneracyReport off = analyze(H, DegeneracyConfig{false, 100.0, 10.0});
  EXPECT_EQ(off.degenerate, (std::array<bool, 6>{}));
  EXPECT_TRUE((off.eigvals.array() == on.eigvals.array()).all());
}

TEST(Analyze, ThresholdIsStrict) {
  const common::Mat6 H = random_spd(3);
  const double l0 = analyze(H, DegeneracyConfig{true, 0.0, 10.0}).eigvals(0);
  EXPECT_FALSE(analyze(H, DegeneracyConfig{true, l0, 10.0}).degenerate[0]);
  const double above = std::nextafter(l0, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(analyze(H, DegeneracyConfig{true, above, 10.0}).degenerate[0]);
}

TEST(Analyze, NegativeEigenvalueFlagged) {
  const common::Mat6 H = diag6(-1e-9, 1e3, 1e3, 1e3, 1e3, 1e3);
  const common::DegeneracyReport r = analyze(H, DegeneracyConfig{true, 0.0, 10.0});
  const std::array<bool, 6> expected{true, false, false, false, false, false};
  EXPECT_EQ(r.degenerate, expected);
  EXPECT_TRUE(r.eigvals.allFinite());
}

TEST(Analyze, NonFiniteLowerTriangleFallsBack) {
  common::Mat6 h_nan = random_spd(5);
  h_nan(3, 1) = std::numeric_limits<double>::quiet_NaN();
  common::Mat6 h_inf = random_spd(5);
  h_inf(4, 4) = std::numeric_limits<double>::infinity();
  for (const common::Mat6& H : {h_nan, h_inf}) {
    const common::DegeneracyReport on = analyze(H, kCfg);
    EXPECT_TRUE((on.eigvals.array() == 0.0).all());
    EXPECT_TRUE((on.eigvecs.array() == common::Mat6::Identity().array()).all());
    for (bool d : on.degenerate) {
      EXPECT_TRUE(d);
    }
    const common::DegeneracyReport off = analyze(H, DegeneracyConfig{false, 100.0, 10.0});
    EXPECT_TRUE((off.eigvals.array() == 0.0).all());
    EXPECT_TRUE((off.eigvecs.array() == common::Mat6::Identity().array()).all());
    for (bool d : off.degenerate) {
      EXPECT_FALSE(d);
    }
  }
}

TEST(Analyze, UpperTriangleIsIgnored) {
  const common::Mat6 H = random_spd(9);
  common::Mat6 hu = H;
  hu(1, 3) = std::numeric_limits<double>::quiet_NaN();
  hu(0, 5) = 1e300;
  EXPECT_TRUE(bitwise_equal(analyze(hu, kCfg), analyze(H, kCfg)));
}

TEST(ProjectUpdate, NoFlagsIsExactIdentity) {
  const common::DegeneracyReport r = analyze(random_spd(13), DegeneracyConfig{false, 0.0, 10.0});
  const common::Vec6 delta = random_delta();
  const common::Vec6 out = project_update(delta, r);
  EXPECT_TRUE((out.array() == delta.array()).all());
}

TEST(ProjectUpdate, RemovesFlaggedKeepsOthers) {
  const common::DegeneracyReport r = report_flags02();
  const common::Vec6 delta = random_delta();
  const common::Vec6 out = project_update(delta, r);
  const common::Vec6 diff = out - delta;
  for (int k = 0; k < 6; ++k) {
    const common::Vec6 v = r.eigvecs.col(k);
    if (is_flagged_index(k)) {
      EXPECT_LT(std::abs(v.dot(out)), 1e-12 * delta.norm()) << "k=" << k;
    } else {
      EXPECT_LT(std::abs(v.dot(diff)), 1e-12 * delta.norm()) << "k=" << k;
    }
  }
}

TEST(ProjectUpdate, AllFlaggedIdentityGivesZero) {
  const common::Vec6 out = project_update(random_delta(), all_flagged());
  EXPECT_TRUE((out.array() == 0.0).all());
}

TEST(RelaxCovariance, NoFlagsIsExactIdentity) {
  const common::DegeneracyReport r = analyze(random_spd(13), DegeneracyConfig{false, 0.0, 10.0});
  const common::PoseCovariance cov = random_spd(11);
  const common::PoseCovariance out = relax_covariance(cov, r, kCfg);
  EXPECT_TRUE((out.array() == cov.array()).all());
}

TEST(RelaxCovariance, InflatesFlaggedDirectionsByFactor) {
  const common::DegeneracyReport r = report_flags02();
  const common::PoseCovariance cov = random_spd(11);
  const common::PoseCovariance out = relax_covariance(cov, r, kCfg);
  for (int k = 0; k < 6; ++k) {
    const common::Vec6 v = r.eigvecs.col(k);
    const double ratio = v.dot(out * v) / v.dot(cov * v);
    if (is_flagged_index(k)) {
      EXPECT_NEAR(ratio, 10.0, 1e-10 * 10.0) << "k=" << k;
      EXPECT_GE(ratio, 10.0 * (1.0 - 1e-12)) << "k=" << k;
    } else {
      EXPECT_NEAR(ratio, 1.0, 1e-10) << "k=" << k;
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_EQ(out(i, j), out(j, i));
    }
  }
  EXPECT_EQ(Eigen::LLT<common::Mat6>(out).info(), Eigen::Success);
}

TEST(RelaxCovariance, FactorOneKeepsCov) {
  const common::PoseCovariance cov = random_spd(11);
  const common::PoseCovariance out =
      relax_covariance(cov, report_flags02(), DegeneracyConfig{true, 100.0, 1.0});
  EXPECT_LT((out - cov).norm(), 1e-12 * cov.norm());
  EXPECT_TRUE((out.array() == out.transpose().array()).all());
}

TEST(RelaxCovariance, AllFlaggedIdentityScalesCov) {
  const common::PoseCovariance cov = random_spd(11);
  const common::PoseCovariance out = relax_covariance(cov, all_flagged(), kCfg);
  const common::PoseCovariance expected = 10.0 * cov;
  EXPECT_LT((out - expected).norm(), 1e-12 * 10.0 * cov.norm());
  EXPECT_TRUE(out.allFinite());
}

constexpr double kVarM2 = 1.0;    // m^2, unobserved translation variance in these tests
constexpr double kVarRad2 = 0.1;  // rad^2, unobserved rotation variance in these tests

TEST(UnobservedCovariance, NoFlagsIsExactIdentity) {
  const common::PoseCovariance cov = random_spd(11);
  const common::DegeneracyReport r = analyze(random_spd(13), DegeneracyConfig{false, 0.0, 10.0});
  const common::PoseCovariance out = unobserved_covariance(cov, r, kCfg, kVarM2, kVarRad2, 0.0);
  EXPECT_TRUE((out.array() == cov.array()).all());
}

// cov' = P cov P + P_F D P_F (no floor at c = 0): observed block unchanged, flagged block
// v_j^T D v_k (the projection of D onto the flagged subspace, basis-invariant), no coupling
// between the two, D = diag(var_m2 I3, var_rad2 I3).
TEST(UnobservedCovariance, ReplacesFlaggedKeepsObserved) {
  const common::PoseCovariance cov = random_spd(11);
  const common::DegeneracyReport r = report_flags02();
  const common::PoseCovariance out = unobserved_covariance(cov, r, kCfg, kVarM2, kVarRad2, 0.0);
  EXPECT_TRUE(out == out.transpose());
  for (int j = 0; j < 6; ++j) {
    const common::Vec6 v_j = r.eigvecs.col(j);
    for (int k = 0; k < 6; ++k) {
      const common::Vec6 v_k = r.eigvecs.col(k);
      const double got = v_j.dot(out * v_k);
      double want = 0.0;
      if (!is_flagged_index(j) && !is_flagged_index(k)) {
        want = v_j.dot(cov * v_k);
      } else if (is_flagged_index(j) && is_flagged_index(k)) {
        want =
            kVarM2 * v_j.head<3>().dot(v_k.head<3>()) + kVarRad2 * v_j.tail<3>().dot(v_k.tail<3>());
      }
      EXPECT_NEAR(got, want, 1e-9 * cov.norm()) << j << "," << k;
    }
  }
}

TEST(UnobservedCovariance, AxisDirectionsGetConfiguredVariance) {
  const common::Mat6 c = random_spd(11);
  const common::PoseCovariance cov = 0.5 * (c + c.transpose());  // exactly symmetric
  common::DegeneracyReport r;                                    // eigvecs = I
  r.degenerate[0] = true;                                        // rho_x
  r.degenerate[5] = true;                                        // phi_z
  const common::PoseCovariance out = unobserved_covariance(cov, r, kCfg, kVarM2, kVarRad2, 0.0);
  // Two flagged directions: U = P_F D P_F goes through a 6x6 eigen-solve, so rounding only.
  constexpr double kTol = 1e-15;
  EXPECT_NEAR(out(0, 0), kVarM2, kTol);
  EXPECT_NEAR(out(5, 5), kVarRad2, kTol);
  for (int i = 1; i < 5; ++i) {
    EXPECT_NEAR(out(0, i), 0.0, kTol);
    EXPECT_NEAR(out(5, i), 0.0, kTol);
    for (int j = 1; j < 5; ++j) {
      EXPECT_EQ(out(i, j), cov(i, j));
    }
  }
  const common::PoseCovariance all =
      unobserved_covariance(cov, all_flagged(), kCfg, kVarM2, kVarRad2, 0.0);
  common::Vec6 d;
  d << kVarM2, kVarM2, kVarM2, kVarRad2, kVarRad2, kVarRad2;
  const common::PoseCovariance expected = d.asDiagonal();
  EXPECT_LT((all - expected).norm(), 1e-15);
}

// Floor (review I2): var_k = max(v_k^T D v_k, relax_factor * c / tau_k). Absolute tau = 100,
// relax_factor 10, c = 1: floor 0.1. rho_x has var_m2 = 0.001 < 0.1 -> 0.1; phi_z has 1.0 rad^2
// > 0.1 -> 1.0. c = 0 (or tau <= 0) applies no floor.
TEST(UnobservedCovariance, FloorAppliesWhenConfiguredVarianceIsSmaller) {
  const common::Mat6 c6 = random_spd(11);
  const common::PoseCovariance cov = 0.5 * (c6 + c6.transpose());
  common::DegeneracyReport r;  // eigvecs = I
  r.degenerate[0] = true;
  r.degenerate[5] = true;
  const common::PoseCovariance out = unobserved_covariance(cov, r, kCfg, 0.001, 1.0, 1.0);
  EXPECT_NEAR(out(0, 0), 0.1, 1e-15);
  EXPECT_NEAR(out(5, 5), 1.0, 1e-15);
  const common::PoseCovariance no_floor = unobserved_covariance(cov, r, kCfg, 0.001, 1.0, 0.0);
  EXPECT_NEAR(no_floor(0, 0), 0.001, 1e-15);
  DegeneracyConfig no_tau = kCfg;  // eig_threshold 0: no floor
  no_tau.eig_threshold = 0.0;
  EXPECT_NEAR(unobserved_covariance(cov, r, no_tau, 0.001, 1.0, 1.0)(0, 0), 0.001, 1e-15);
}

// Review M3: with a repeated flagged eigenvalue the eigen-solver's basis inside the eigenspace
// is arbitrary. U = P_F D P_F does not depend on it: two bases of span{rho_x, phi_x} (axes and
// axes rotated by 45 deg) give the same covariance, D restricted to the subspace.
TEST(UnobservedCovariance, RepeatedFlaggedEigenvalueBasisInvariant) {
  const common::Mat6 c6 = random_spd(11);
  const common::PoseCovariance cov = 0.5 * (c6 + c6.transpose());
  common::DegeneracyReport a;  // eigvecs = I
  a.degenerate[0] = true;
  a.degenerate[3] = true;
  common::DegeneracyReport b = a;
  const double h = std::sqrt(0.5);
  b.eigvecs.col(0) = common::Vec6::Zero();
  b.eigvecs.col(3) = common::Vec6::Zero();
  b.eigvecs(0, 0) = h;
  b.eigvecs(3, 0) = h;
  b.eigvecs(0, 3) = -h;
  b.eigvecs(3, 3) = h;
  const common::PoseCovariance out_a = unobserved_covariance(cov, a, kCfg, kVarM2, kVarRad2, 0.0);
  const common::PoseCovariance out_b = unobserved_covariance(cov, b, kCfg, kVarM2, kVarRad2, 0.0);
  EXPECT_LT((out_a - out_b).norm(), 1e-14);
  EXPECT_NEAR(out_b(0, 0), kVarM2, 1e-15);
  EXPECT_NEAR(out_b(3, 3), kVarRad2, 1e-15);
  EXPECT_NEAR(out_b(0, 3), 0.0, 1e-15);
}

// Backlog G3 (degeneracy.md section 8): the marginal information along u is 1 / (u^T H^-1 u),
// against an explicit inverse on random SPD matrices (relative error < 1e-9).
TEST(MarginalInformation, MatchesExplicitInverse) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    const common::Mat6 H = random_spd(100 + seed);
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    common::Vec6 u;
    for (int i = 0; i < 6; ++i) {
      u(i) = n01(rng);
    }
    const double expected = 1.0 / u.dot(H.inverse() * u);
    const double got = marginal_information(analyze(H, DegeneracyConfig{}), u);
    EXPECT_NEAR(got, expected, 1e-9 * expected) << seed;
  }
}

// Along an eigenvector it is the eigenvalue, so the gate ">= eig_threshold" is analyze()'s test;
// a translation coupled to a rotation keeps only its Schur complement, while the raw count
// u^T H u keeps the full diagonal; a direction mixed with a degenerate one inherits its weakness.
TEST(MarginalInformation, EigenvectorCouplingAndMixing) {
  const common::Mat6 D = diag6(3.0, 500.0, 800.0, 40.0, 60.0, 90.0);
  common::Vec6 x = common::Vec6::Zero();
  x(0) = 1.0;
  EXPECT_NEAR(marginal_information(analyze(D, DegeneracyConfig{}), x), 3.0, 1e-9);
  // rho_x coupled to phi_z: H = [[a, c], [c, b]] on (rho_x, phi_z): Schur a - c^2 / b.
  common::Mat6 C = diag6(400.0, 500.0, 800.0, 40.0, 60.0, 90.0);
  C(0, 5) = 150.0;
  C(5, 0) = 150.0;
  const double schur = 400.0 - 150.0 * 150.0 / 90.0;  // 150
  EXPECT_NEAR(marginal_information(analyze(C, DegeneracyConfig{}), x), schur, 1e-9 * schur);
  EXPECT_DOUBLE_EQ(x.dot(C * x), 400.0);  // the raw count would pass a gate of 200
  // 10 deg off the degenerate axis x (3) towards y (5000): far below the threshold of 100.
  const common::Mat6 D2 = diag6(3.0, 5000.0, 800.0, 40.0, 60.0, 90.0);
  common::Vec6 u = common::Vec6::Zero();
  u(0) = std::cos(0.1745);
  u(1) = std::sin(0.1745);
  const double mixed = marginal_information(analyze(D2, DegeneracyConfig{}), u);
  EXPECT_LT(mixed, 3.0 / (u(0) * u(0)) + 1e-9);
  EXPECT_GT(u.dot(D2 * u), 100.0);  // the raw count would call it observed
}

// A component along a non-positive eigenvalue (a singular corridor Hessian), a non-finite H
// (analyze() returns zero eigenvalues) and u = 0 all give 0; a null direction orthogonal to u
// does not.
TEST(MarginalInformation, SingularNonFiniteAndZero) {
  const common::Mat6 singular = diag6(0.0, 500.0, 800.0, 40.0, 60.0, 90.0);
  common::Vec6 x = common::Vec6::Zero();
  x(0) = 1.0;
  common::Vec6 y = common::Vec6::Zero();
  y(1) = 1.0;
  EXPECT_EQ(marginal_information(analyze(singular, DegeneracyConfig{}), x), 0.0);
  EXPECT_NEAR(marginal_information(analyze(singular, DegeneracyConfig{}), y), 500.0, 1e-9);
  common::Mat6 bad = singular;
  bad(2, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(marginal_information(analyze(bad, DegeneracyConfig{}), y), 0.0);
  EXPECT_EQ(marginal_information(analyze(singular, DegeneracyConfig{}), common::Vec6::Zero()), 0.0);
  // An indefinite H (a negative eigenvalue with a component along u) has no information along u,
  // also when the positive part alone would give some.
  const common::Mat6 indefinite = diag6(1.0, -1000.0, 800.0, 40.0, 60.0, 90.0);
  common::Vec6 xy = common::Vec6::Zero();
  xy(0) = std::sqrt(0.5);
  xy(1) = std::sqrt(0.5);
  EXPECT_EQ(marginal_information(analyze(indefinite, DegeneracyConfig{}), xy), 0.0);
  // A report whose eigenvalue is not finite gives 0, also on an eigenvector orthogonal to u
  // (analyze() never returns one; the function does not rely on that).
  common::DegeneracyReport nan_report =
      analyze(diag6(1000.0, 800.0, 600.0, 500.0, 400.0, 300.0), DegeneracyConfig{});
  int k_orthogonal = -1;
  for (int k = 1; k < 6; ++k) {
    if (std::abs(nan_report.eigvecs.col(k).dot(x)) < 1e-12 &&
        nan_report.eigvals(k) < nan_report.eigvals.maxCoeff()) {
      k_orthogonal = k;
    }
  }
  ASSERT_GE(k_orthogonal, 1);
  EXPECT_NEAR(marginal_information(nan_report, x), 1000.0, 1e-9);
  nan_report.eigvals(k_orthogonal) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(marginal_information(nan_report, x), 0.0);
}

// H4 review m-1: a rank-deficient H whose null space is orthogonal to u (walls only: rho_z, roll
// and pitch unobserved), seen in a randomly rotated frame, has eigenvalues of round-off size and
// either sign on the null space; the information along a horizontal u is still finite and equal
// to the unrotated value (1 / (c^2 / 1000 + s^2 / 800)), within 1e-6. A u with a component along
// the null space still gets 0.
TEST(MarginalInformation, RotatedRankDeficient) {
  const common::Mat6 H0 = diag6(1000.0, 800.0, 0.0, 0.0, 0.0, 500.0);
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);
  for (int trial = 0; trial < 200; ++trial) {
    const Eigen::Vector3d axis = Eigen::Vector3d(n01(rng), n01(rng), n01(rng)).normalized();
    const Eigen::Matrix3d R = Eigen::AngleAxisd(3.0 * n01(rng), axis).toRotationMatrix();
    common::Mat6 Q = common::Mat6::Zero();
    Q.topLeftCorner<3, 3>() = R;
    Q.bottomRightCorner<3, 3>() = R;
    const common::Mat6 H = Q * H0 * Q.transpose();
    const double a = 0.3 * trial;
    common::Vec6 u0 = common::Vec6::Zero();
    u0(0) = std::cos(a);
    u0(1) = std::sin(a);
    const common::Vec6 u = Q * u0;
    const double expected = 1.0 / (u0(0) * u0(0) / 1000.0 + u0(1) * u0(1) / 800.0);
    const double got = marginal_information(analyze(H, DegeneracyConfig{}), u);
    EXPECT_NEAR(got, expected, 1e-6 * expected) << trial;
    common::Vec6 z0 = u0;
    z0(2) = 0.5;  // a vertical component: rho_z is unobserved
    EXPECT_EQ(marginal_information(analyze(H, DegeneracyConfig{}), Q * z0), 0.0) << trial;
  }
}

TEST(DegeneracyHotPath, NoAllocation) {
  const common::Mat6 H = random_spd(17);
  const DegeneracyConfig cfg{true, 1e9, 10.0};  // everything flagged: every branch runs
  const common::Vec6 delta = random_delta();
  const common::PoseCovariance cov = random_spd(11);
  double sink = 0.0;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int i = 0; i < 100; ++i) {
      const common::DegeneracyReport r = analyze(H, cfg);
      const common::Vec6 d = project_update(delta, r);
      const common::PoseCovariance c = relax_covariance(cov, r, cfg);
      const common::PoseCovariance u = unobserved_covariance(cov, r, cfg, kVarM2, kVarRad2, 1e-3);
      sink += d.sum() + c.trace() + u.trace() + marginal_information(r, delta);
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace mss::degeneracy
