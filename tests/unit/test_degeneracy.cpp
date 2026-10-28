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
      sink += d.sum() + c.trace() + u.trace();
    }
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace mss::degeneracy
