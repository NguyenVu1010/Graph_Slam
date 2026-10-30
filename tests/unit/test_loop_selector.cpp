// LoopSelector (PCM, ADR-028; docs/derivations/loop_closure.md section 11): odometry prefix
// sums against direct compounding, the pairwise cycle test under drift and aliasing, the
// maximum-clique selection with late removal, ties, eviction, validation and allocation.
// Known answers, fixed seeds only.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/loop/loop_selector.hpp"

#include "alloc_counter.hpp"
#include "loop_scenes.hpp"

namespace mss::loop {
namespace {

using common::KeyId;
using common::LoopConstraint;
using common::Mat6;
using common::Pose;
using common::PoseCovariance;
using common::Vec3;
using common::Vec6;

constexpr double kDeg = test::kLoopPi / 180.0;

LoopConfig pcm_config() {
  const pipeline::Config c = test::shipped_loop_config();
  EXPECT_TRUE(c.loop_closure.enabled);
  EXPECT_TRUE(c.loop_closure.pcm_enabled);
  return c.loop_closure;
}

PoseCovariance diag_cov(double sig_m, double sig_rad) {
  Vec6 d;
  d << sig_m, sig_m, sig_m, sig_rad, sig_rad, sig_rad;
  const PoseCovariance c = d.cwiseAbs2().asDiagonal();
  return c;
}

// Two laps of a 6 m circle in 0.5 m steps (as LoopComponent.TwoLapDrift). The odometry is the
// truth with a yaw bias of 0.1 deg per step (drift) and fixed-seed noise; claimed covariance
// 2 cm / 0.15 deg times `claim` (claim < 1: overconfident).
struct Lap {
  std::vector<Pose> truth;
  LoopSelector sel;
  Lap(const LoopConfig& cfg, double claim, int n) : sel(cfg, 200) {
    std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);
    const auto circle = [](int k) {
      const double a = k * 0.5 / 6.0;
      return test::body_pose(6.0 * std::sin(a), 6.0 * (1.0 - std::cos(a)) - 6.0, a);
    };
    for (int k = 0; k < n; ++k) {
      truth.push_back(circle(k));
      if (k == 0) {
        continue;
      }
      Vec6 e;
      e << 0.02 * n01(rng), 0.02 * n01(rng), 0.0, 0.0, 0.0, 0.1 * kDeg + 0.15 * kDeg * n01(rng);
      const Pose meas = (circle(k - 1).inverse() * circle(k)) * common::se3_exp(e);
      EXPECT_TRUE(sel.add_odometry(
          static_cast<KeyId>(k), meas, diag_cov(0.02 * claim, 0.15 * kDeg * claim)));
    }
  }
  // A loop measured from the truth (exact, floored covariance 5 cm / 1 deg), optionally with
  // the query displaced by `alias_m` along world x (the copy one period away along an aisle:
  // the same world shift for every aliased loop, so aliased loops agree with each other).
  LoopConstraint loop(int i, int j, double alias_m = 0.0) const {
    LoopConstraint lc;
    lc.i = static_cast<KeyId>(i);
    lc.j = static_cast<KeyId>(j);
    Pose T_j = truth[static_cast<std::size_t>(j)];
    T_j.p.x() += alias_m;
    lc.T_bi_bj = truth[static_cast<std::size_t>(i)].inverse() * T_j;
    lc.cov = diag_cov(0.05, 1.0 * kDeg);
    return lc;
  }
};

// O(a -> b) from the prefix sums equals compounding the increments one by one (right
// perturbation: C <- Ad(Z^-1) C Ad(Z^-1)^T + C_k), and O(b -> a) is its inverse.
TEST(LoopSelector, OdometryPrefixSumsMatchDirectCompounding) {
  LoopConfig cfg = pcm_config();
  cfg.pcm_odometry_sigma_scale = 1.0;  // the covariance as given
  cfg.pcm_yaw_drift_rad_per_m =
      0.0;  // no drift term (YawDriftCovarianceMatchesNumericalDifference)
  LoopSelector sel(cfg, 10);
  std::mt19937 rng(3);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::vector<Pose> inc(1);
  std::vector<PoseCovariance> cov(1);
  for (int k = 1; k <= 20; ++k) {
    Vec6 xi;
    for (int a = 0; a < 6; ++a) {
      xi(a) = (a < 3 ? 0.5 : 0.2) * n01(rng);
    }
    Mat6 A;
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        A(r, c) = 0.01 * n01(rng);
      }
    }
    const PoseCovariance C = A * A.transpose() + diag_cov(0.01, 0.002);
    inc.push_back(common::se3_exp(xi));
    cov.push_back(C);
    ASSERT_TRUE(sel.add_odometry(static_cast<KeyId>(k), inc.back(), C));
  }
  EXPECT_FALSE(sel.add_odometry(25, Pose{}, diag_cov(0.01, 0.01)));  // out of order
  for (const auto& ab : {std::pair<int, int>{0, 20}, {3, 11}, {7, 8}, {12, 19}}) {
    Pose T = Pose{};
    PoseCovariance C = PoseCovariance::Zero();
    for (int k = ab.first + 1; k <= ab.second; ++k) {
      T = T * inc[static_cast<std::size_t>(k)];
      C = common::transform_covariance(C, inc[static_cast<std::size_t>(k)].inverse()) +
          cov[static_cast<std::size_t>(k)];
    }
    Pose T_s;
    PoseCovariance C_s;
    ASSERT_TRUE(
        sel.odometry(static_cast<KeyId>(ab.first), static_cast<KeyId>(ab.second), T_s, C_s));
    EXPECT_LT(common::se3_log(T.inverse() * T_s).norm(), 1e-9);
    EXPECT_LT((C_s - C).norm(), 1e-9 * C.norm()) << ab.first << " " << ab.second;
    Pose T_r;
    PoseCovariance C_r;
    ASSERT_TRUE(
        sel.odometry(static_cast<KeyId>(ab.second), static_cast<KeyId>(ab.first), T_r, C_r));
    EXPECT_LT(common::se3_log(T_r * T).norm(), 1e-9);
    const PoseCovariance C_inv = common::transform_covariance(C, T);
    EXPECT_LT((C_r - C_inv).norm(), 1e-9 * C_inv.norm());
  }
}

// The pairwise test does not see the drift common to both loops: true loops of the second lap
// are consistent with each other even with an overconfident odometry claim (x0.5, x0.25)
// after 1.2 m of drift; a loop aliased by one 2 m period is inconsistent with the true ones
// near it;
// two loops aliased by the same period are consistent with each other (aliasing that PCM
// cannot see, handled by the ambiguity check).
TEST(LoopSelector, PairTestIgnoresCommonDriftAndCatchesAliasing) {
  const LoopConfig cfg = pcm_config();
  for (const double claim : {1.0, 0.5, 0.25}) {
    const Lap lap(cfg, claim, 151);
    const LoopConstraint t1 = lap.loop(0, 75);
    const LoopConstraint t2 = lap.loop(23, 98);
    const LoopConstraint t3 = lap.loop(60, 135);
    const LoopConstraint f1 = lap.loop(10, 85, 2.0);
    const LoopConstraint f2 = lap.loop(30, 105, 2.0);
    EXPECT_LE(lap.sel.pair_chi2(t1, t2), cfg.pcm_chi2_gate) << claim;
    EXPECT_LE(lap.sel.pair_chi2(t1, t3), cfg.pcm_chi2_gate) << claim;
    EXPECT_LE(lap.sel.pair_chi2(t2, t3), cfg.pcm_chi2_gate) << claim;
    // An aliased loop is inconsistent with the true loops near it (segments of 7 to 13
    // keyframes). Against a true loop 30 or more keyframes away the inflated odometry
    // covariance (pcm_odometry_sigma_scale) can hide a 2 m alias; the clique still drops it
    // (LargestConsistentSetWinsAndRemovesLaterInconsistentLoops).
    EXPECT_GT(lap.sel.pair_chi2(t1, f1), cfg.pcm_chi2_gate) << claim;
    EXPECT_GT(lap.sel.pair_chi2(t2, f1), cfg.pcm_chi2_gate) << claim;
    EXPECT_GT(lap.sel.pair_chi2(t2, f2), cfg.pcm_chi2_gate) << claim;
    EXPECT_LE(lap.sel.pair_chi2(f1, f2), cfg.pcm_chi2_gate) << claim;
  }
}

// Late removal: two aliased loops arrive first and are selected; three true loops then
// outnumber them, so the aliased ones leave the set (to_remove) and the true ones enter.
// In the other order the aliased loops are never selected.
TEST(LoopSelector, LargestConsistentSetWinsAndRemovesLaterInconsistentLoops) {
  const LoopConfig cfg = pcm_config();
  Lap lap(cfg, 1.0, 151);
  LoopSelector::Decision d = lap.sel.propose(lap.loop(10, 85, 2.0));
  EXPECT_TRUE(d.selected);  // the first loop is taken (no evidence against it yet)
  ASSERT_EQ(lap.sel.to_add().size(), 1U);
  d = lap.sel.propose(lap.loop(30, 105, 2.0));
  EXPECT_TRUE(d.selected);
  EXPECT_EQ(d.n_consistent, 1);
  d = lap.sel.propose(lap.loop(0, 75));
  EXPECT_FALSE(d.selected);  // 1 against 2
  EXPECT_TRUE(lap.sel.to_add().empty());
  EXPECT_TRUE(lap.sel.to_remove().empty());
  d = lap.sel.propose(lap.loop(23, 98));
  EXPECT_FALSE(d.selected);  // 2 against 2: the current selection stays
  d = lap.sel.propose(lap.loop(60, 135));
  EXPECT_TRUE(d.selected);  // 3 against 2
  EXPECT_EQ(d.clique_size, 3);
  ASSERT_EQ(lap.sel.to_remove().size(), 2U);
  ASSERT_EQ(lap.sel.to_add().size(), 3U);  // the two earlier true loops come back, oldest first
  EXPECT_EQ(lap.sel.to_add()[0].i, 0U);
  EXPECT_EQ(lap.sel.to_add()[1].i, 23U);
  EXPECT_EQ(lap.sel.to_add()[2].i, 60U);
  for (const auto& ij : lap.sel.to_remove()) {
    EXPECT_TRUE(ij.first == 10U || ij.first == 30U);
  }
  Lap fresh(cfg, 1.0, 151);
  for (const auto& l : {fresh.loop(0, 75), fresh.loop(23, 98), fresh.loop(60, 135)}) {
    EXPECT_TRUE(fresh.sel.propose(l).selected);
  }
  EXPECT_FALSE(fresh.sel.propose(fresh.loop(10, 85, 2.0)).selected);
  EXPECT_FALSE(fresh.sel.propose(fresh.loop(30, 105, 2.0)).selected);
  EXPECT_EQ(fresh.sel.n_selected(), 3U);
}

// B-03 through PCM: square loop graph (10 m sides, 4 odometry edges), true loops (0, 4) and
// (1, 4), outlier (1, 3) 5 m / 29 deg off. With the outlier first it is selected, then removed
// when the two true loops agree with each other and not with it.
TEST(LoopSelector, B03_OutlierLoopRemovedWhenTwoTrueLoopsAgree) {
  const LoopConfig cfg = pcm_config();
  LoopSelector sel(cfg, 8);
  std::vector<Pose> T;
  for (int k = 0; k < 5; ++k) {
    const double yaw = 0.5 * test::kLoopPi * (k % 4);
    const Vec3 corners[4] = {Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(10, 10, 0), Vec3(0, 10, 0)};
    T.push_back(Pose{test::loop_rz(yaw), corners[k % 4]});
  }
  const PoseCovariance edge = diag_cov(0.02, 0.002);
  for (int k = 1; k < 5; ++k) {
    ASSERT_TRUE(sel.add_odometry(static_cast<KeyId>(k), T[k - 1].inverse() * T[k], edge));
  }
  const auto loop = [&](int i, int j, const Vec6& wrong) {
    LoopConstraint lc;
    lc.i = static_cast<KeyId>(i);
    lc.j = static_cast<KeyId>(j);
    lc.T_bi_bj = T[i].inverse() * T[j] * common::se3_exp(wrong);
    lc.cov = edge;
    return lc;
  };
  Vec6 wrong;
  wrong << 5.0, 0.0, 0.0, 0.0, 0.0, 0.5;
  EXPECT_TRUE(sel.propose(loop(1, 3, wrong)).selected);
  EXPECT_FALSE(sel.propose(loop(0, 4, Vec6::Zero())).selected);  // tie: selection stays
  EXPECT_TRUE(sel.propose(loop(1, 4, Vec6::Zero())).selected);
  ASSERT_EQ(sel.to_remove().size(), 1U);
  EXPECT_EQ(sel.to_remove()[0].first, 1U);
  EXPECT_EQ(sel.to_remove()[0].second, 3U);
  EXPECT_EQ(sel.n_selected(), 2U);
}

// pcm_max_loops = 2: the third loop evicts the oldest; a selected oldest is frozen (it stays in
// the graph, no removal is reported).
TEST(LoopSelector, OldestLoopLeavesWhenFull) {
  LoopConfig cfg = pcm_config();
  cfg.pcm_max_loops = 2;
  Lap lap(cfg, 1.0, 151);
  EXPECT_TRUE(lap.sel.propose(lap.loop(0, 75)).selected);
  EXPECT_TRUE(lap.sel.propose(lap.loop(23, 98)).selected);
  const LoopSelector::Decision d = lap.sel.propose(lap.loop(60, 135));
  EXPECT_TRUE(d.frozen);
  EXPECT_TRUE(d.selected);
  EXPECT_TRUE(lap.sel.to_remove().empty());
  EXPECT_EQ(lap.sel.n_loops(), 2U);
}

TEST(LoopSelector, ValidationAndInvalidLoops) {
  LoopConfig cfg = pcm_config();
  LoopConfig bad = cfg;
  bad.pcm_chi2_gate = 0.0;
  EXPECT_THROW(LoopSelector(bad, 1), std::invalid_argument);
  bad = cfg;
  bad.pcm_max_loops = 65;
  EXPECT_THROW(LoopSelector(bad, 1), std::invalid_argument);
  bad = cfg;
  bad.pcm_max_clique_steps = 0;
  EXPECT_THROW(LoopSelector(bad, 1), std::invalid_argument);
  bad.enabled = false;  // disabled: nothing validated
  EXPECT_NO_THROW(LoopSelector(bad, 1));
  Lap lap(cfg, 1.0, 10);
  EXPECT_FALSE(lap.sel.propose(lap.loop(5, 3)).selected);   // i > j
  EXPECT_FALSE(lap.sel.propose(lap.loop(0, 20)).selected);  // j outside the chain
  LoopConstraint flat = lap.loop(0, 9);
  flat.cov = PoseCovariance::Zero();
  EXPECT_FALSE(lap.sel.propose(flat).selected);  // covariance not positive definite
  EXPECT_EQ(lap.sel.n_loops(), 0U);
}

// propose() allocates nothing once constructed (lists reserved, 64-bit masks).
TEST(LoopSelector, ProposeDoesNotAllocate) {
  const LoopConfig cfg = pcm_config();
  Lap lap(cfg, 1.0, 151);
  const LoopConstraint a = lap.loop(0, 75);
  const LoopConstraint b = lap.loop(23, 98);
  const LoopConstraint f = lap.loop(10, 85, 2.0);
  AllocCounter::reset();
  (void)lap.sel.propose(a);
  (void)lap.sel.propose(f);
  (void)lap.sel.propose(b);
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
}

// Review H3 m-1: forget() drops a held loop without reporting it (the graph lost it); the
// selection is recomputed from what is left at the next propose().
TEST(LoopSelector, ForgetDropsAHeldLoopSilently) {
  const LoopConfig cfg = pcm_config();
  Lap lap(cfg, 1.0, 151);
  EXPECT_TRUE(lap.sel.propose(lap.loop(10, 85, 2.0)).selected);  // aliased, first
  EXPECT_TRUE(lap.sel.forget(10, 85));
  EXPECT_FALSE(lap.sel.forget(10, 85));
  EXPECT_EQ(lap.sel.n_loops(), 0U);
  const LoopSelector::Decision d = lap.sel.propose(lap.loop(0, 75));
  EXPECT_TRUE(d.selected);  // no tie any more: the aliased loop is gone
  EXPECT_TRUE(lap.sel.to_remove().empty());
}

// Review H3 m-2: with pcm_max_clique_steps = 1 the search is cut at once; it starts from the
// current selection, so the selection never shrinks and nothing is removed.
TEST(LoopSelector, TruncatedCliqueSearchNeverMakesTheSelectionWorse) {
  LoopConfig cfg = pcm_config();
  cfg.pcm_max_clique_steps = 1;
  Lap lap(cfg, 1.0, 151);
  std::size_t prev = 0;
  for (const auto& l : {lap.loop(0, 75), lap.loop(23, 98), lap.loop(60, 135)}) {
    (void)lap.sel.propose(l);
    EXPECT_TRUE(lap.sel.to_remove().empty());
    EXPECT_GE(lap.sel.n_selected(), prev);
    prev = lap.sel.n_selected();
  }
}

// --- Review H3 B-1, fixed by the yaw-drift term (ADR-028 amendment C) -----------------------
// Three laps of a circle (radius 6 to 20 m) with the repo's yaw bias (0.1 deg per 0.5 m step, not
// in the claimed covariance) and white noise of the claimed sigmas (x1, x0.5): 4 TRUE loops
// lap 2 -> lap 1, then 5 TRUE loops lap 3 -> lap 1 (exact, 5 cm / 1 deg). Known answer: every
// loop stays selected, none is removed. Without the drift term PCM removed 2 to 4 of the first
// revisit's loops (their cycles with the second revisit's hold a whole lap of biased odometry).
struct Laps3 {
  std::vector<Pose> truth;
  LoopSelector sel;
  int per_lap = 0;
  Laps3(const LoopConfig& cfg, double radius_m, double claim, unsigned seed = 42) : sel(cfg, 4000) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    per_lap = static_cast<int>(std::lround(2.0 * test::kLoopPi * radius_m / 0.5));
    const auto circle = [radius_m](int k) {
      const double a = k * 0.5 / radius_m;
      return test::body_pose(radius_m * std::sin(a), radius_m * (1.0 - std::cos(a)) - radius_m, a);
    };
    const int n = 3 * per_lap + 1;
    for (int k = 0; k < n; ++k) {
      truth.push_back(circle(k));
      if (k == 0) {
        continue;
      }
      Vec6 e;
      e << 0.02 * n01(rng), 0.02 * n01(rng), 0.0, 0.0, 0.0, 0.1 * kDeg + 0.15 * kDeg * n01(rng);
      const Pose meas = (circle(k - 1).inverse() * circle(k)) * common::se3_exp(e);
      EXPECT_TRUE(sel.add_odometry(
          static_cast<KeyId>(k), meas, diag_cov(0.02 * claim, 0.15 * kDeg * claim)));
    }
  }
  LoopConstraint loop(int i, int j) const {
    LoopConstraint lc;
    lc.i = static_cast<KeyId>(i);
    lc.j = static_cast<KeyId>(j);
    lc.T_bi_bj = truth[static_cast<std::size_t>(i)].inverse() * truth[static_cast<std::size_t>(j)];
    lc.cov = diag_cov(0.05, 1.0 * kDeg);
    return lc;
  }
};

// The probe's loops: 4 lap 2 -> lap 1, 5 lap 3 -> lap 1 and 3 lap 3 -> lap 2 (12 true loops).
struct ThreeLapRun {
  int removed = 0;
  int rejected = 0;
  std::size_t selected = 0;
  double max_chi2 = 0.0;
};

ThreeLapRun three_laps(const LoopConfig& cfg, double radius, double claim, unsigned seed) {
  Laps3 L(cfg, radius, claim, seed);
  const int N = L.per_lap;
  std::vector<LoopConstraint> seq;
  for (int q = 0; q < 4; ++q) {
    seq.push_back(L.loop(5 + q * N / 5, N + 5 + q * N / 5));
  }
  for (int q = 0; q < 5; ++q) {
    seq.push_back(L.loop(5 + q * N / 5, 2 * N + 5 + q * N / 5));
  }
  for (int q = 0; q < 3; ++q) {
    seq.push_back(L.loop(N + 10 + q * N / 4, 2 * N + 10 + q * N / 4));
  }
  ThreeLapRun r;
  for (std::size_t a = 0; a < seq.size(); ++a) {
    for (std::size_t b = a + 1; b < seq.size(); ++b) {
      r.max_chi2 = std::max(r.max_chi2, L.sel.pair_chi2(seq[a], seq[b]));
    }
  }
  for (const LoopConstraint& lc : seq) {
    r.rejected += L.sel.propose(lc).selected ? 0 : 1;
    r.removed += static_cast<int>(L.sel.to_remove().size());
  }
  r.selected = L.sel.n_selected();
  return r;
}

constexpr unsigned kThreeLapSeeds[] = {42U, 1U, 7U, 2024U, 3U, 99U, 31337U};

TEST(LoopSelector, ThreeLapTrueLoopsAllStaySelected) {
  const LoopConfig cfg = pcm_config();
  struct Case {
    double radius;
    double claim;
  };
  // Claim x1 on every lap size up to 250 m (radius 40 m); claim x0.5 up to 125 m (radius 20 m):
  // the covered range of pcm_odometry_sigma_scale 1.5 (loop_closure.md 11b; review H3 I-1).
  const Case cases[] = {{6.0, 1.0},
                        {9.0, 1.0},
                        {12.0, 1.0},
                        {20.0, 1.0},
                        {40.0, 1.0},
                        {6.0, 0.5},
                        {9.0, 0.5},
                        {12.0, 0.5},
                        {20.0, 0.5}};
  for (const Case& c : cases) {
    double worst = 0.0;
    for (const unsigned seed : kThreeLapSeeds) {
      const ThreeLapRun r = three_laps(cfg, c.radius, c.claim, seed);
      worst = std::max(worst, r.max_chi2);
      EXPECT_EQ(r.removed, 0) << c.radius << " " << c.claim << " seed " << seed;
      EXPECT_EQ(r.rejected, 0) << c.radius << " " << c.claim << " seed " << seed;
      EXPECT_EQ(r.selected, 12U) << c.radius << " " << c.claim << " seed " << seed;
    }
    std::printf("[ pcm three laps ] R %.0f m claim x%.2f, 7 seeds: largest pair chi2 %.1f\n",
                c.radius,
                c.claim,
                worst);
  }
}

// Known limit (review H3 I-1; the owner keeps pcm_odometry_sigma_scale at 1.5 pending a manual
// test): at claim x0.5 on a 250 m lap (radius 40 m) the white part of a lap, claimed at half its
// size, exceeds what the 1.5 inflation covers. As this test prints them (run with
// --gtest_also_run_disabled_tests): seed 2024 rejects 3 true loops (largest pair chi2 24.5), seed 3
// removes 1 and rejects 1 (33.2), seed 31337 rejects 2 (38.0); the other 4 pass (16.3 to 20.7).
// Known answer once covered: every loop selected.
// TODO(C1-27, owner decision on review H3 I-1): re-enable with pcm_odometry_sigma_scale 2, or
// delete if 1.5 is kept and the limit accepted.
TEST(LoopSelector, DISABLED_ThreeLapRadius40HalfClaimAllStaySelected) {
  const LoopConfig cfg = pcm_config();
  for (const unsigned seed : kThreeLapSeeds) {
    const ThreeLapRun r = three_laps(cfg, 40.0, 0.5, seed);
    std::printf(
        "[ pcm three laps ] R 40 m claim x0.50 seed %u: largest pair chi2 %.1f, %d "
        "removed, %d not selected\n",
        seed,
        r.max_chi2,
        r.removed,
        r.rejected);
    EXPECT_EQ(r.removed, 0) << seed;
    EXPECT_EQ(r.rejected, 0) << seed;
  }
}

// The yaw-drift covariance against numerical differentiation (ADR-028 amendment C,
// loop_closure.md 11b). Residual: the right-perturbation tangent of the segment O(a -> b);
// variable: the drift rate b (rad/m), applied to every step as Z_k Exp(b |p_k| e_yaw). With the
// step covariances zero and no inflation the selector's segment covariance must be
// sigma_b^2 g g^T with g = d tangent / d b, here by central differences (h = 1e-6 rad/m) on the
// re-composed chain. Relative error < 1e-6, both directions (a < b and a > b).
TEST(LoopSelector, YawDriftCovarianceMatchesNumericalDifference) {
  LoopConfig cfg = pcm_config();
  cfg.pcm_odometry_sigma_scale = 1.0;
  cfg.pcm_yaw_drift_rad_per_m = 0.01;
  LoopSelector sel(cfg, 40);
  std::mt19937 rng(5);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::vector<Pose> inc(1);
  for (int k = 1; k <= 30; ++k) {
    Vec6 xi;
    xi << 0.5 + 0.1 * n01(rng), 0.05 * n01(rng), 0.02 * n01(rng), 0.02 * n01(rng), 0.02 * n01(rng),
        0.1 * n01(rng);
    inc.push_back(common::se3_exp(xi));
    ASSERT_TRUE(sel.add_odometry(static_cast<KeyId>(k), inc.back(), PoseCovariance::Zero()));
  }
  const auto segment = [&](int a, int b, double drift) {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    Pose T;
    for (int k = lo + 1; k <= hi; ++k) {
      Vec6 e = Vec6::Zero();
      e(5) = drift * inc[static_cast<std::size_t>(k)].p.norm();
      T = T * inc[static_cast<std::size_t>(k)] * common::se3_exp(e);
    }
    return a < b ? T : T.inverse();
  };
  constexpr double kH = 1e-6;
  for (const auto& ab : {std::pair<int, int>{0, 30}, {4, 21}, {25, 3}, {10, 11}}) {
    const Pose T0 = segment(ab.first, ab.second, 0.0);
    const Vec6 g = (common::boxminus(segment(ab.first, ab.second, kH), T0) -
                    common::boxminus(segment(ab.first, ab.second, -kH), T0)) /
                   (2.0 * kH);
    const PoseCovariance expect =
        cfg.pcm_yaw_drift_rad_per_m * cfg.pcm_yaw_drift_rad_per_m * (g * g.transpose());
    Pose T_s;
    PoseCovariance C_s;
    ASSERT_TRUE(
        sel.odometry(static_cast<KeyId>(ab.first), static_cast<KeyId>(ab.second), T_s, C_s));
    EXPECT_LT(common::se3_log(T_s.inverse() * T0).norm(), 1e-9);
    EXPECT_LT((C_s - expect).norm(), 1e-6 * expect.norm()) << ab.first << " " << ab.second;
    // The dominant cross term: lateral error ~ distance x yaw error (yaw-lateral correlation).
    std::printf("[ yaw drift ] segment %d -> %d: g = [%.3f %.3f %.3f | %.4f %.4f %.4f]\n",
                ab.first,
                ab.second,
                g(0),
                g(1),
                g(2),
                g(3),
                g(4),
                g(5));
  }
}

// How much the drift term weakens the aliasing rejection: chi2 of a loop aliased by one 2 m
// period against true loops 10, 13, 30 and 50 keyframes away, with and without the term (both at
// the shipped pcm_odometry_sigma_scale 1.5). All four stay above the gate.
TEST(LoopSelector, YawDriftTermKeepsNearAliasesInconsistent) {
  LoopConfig with = pcm_config();
  LoopConfig without = with;
  without.pcm_yaw_drift_rad_per_m = 0.0;
  const Lap lw(with, 1.0, 151);
  const Lap lo(without, 1.0, 151);
  const LoopConstraint f = lw.loop(10, 85, 2.0);
  for (const int i : {0, 23, 40, 60}) {
    const LoopConstraint t = lw.loop(i, i + 75);
    const double cw = lw.sel.pair_chi2(t, f);
    const double co = lo.sel.pair_chi2(t, f);
    std::printf(
        "[ yaw drift ] aliased (10, 85) against true (%d, %d): chi2 %.1f with the drift "
        "term, %.1f without\n",
        i,
        i + 75,
        cw,
        co);
    EXPECT_GT(cw, with.pcm_chi2_gate) << i;
    EXPECT_GT(co, with.pcm_chi2_gate) << i;
  }
}

}  // namespace
}  // namespace mss::loop
