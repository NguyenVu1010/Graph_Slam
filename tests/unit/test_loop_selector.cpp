// LoopSelector (PCM, ADR-028; docs/derivations/loop_closure.md section 11): odometry prefix
// sums against direct compounding, the pairwise cycle test under drift and aliasing, the
// maximum-clique selection with late removal, ties, eviction, validation and allocation.
// Known answers, fixed seeds only.
#include <cmath>
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

// --- Review H3 B-1 (held for the owner's decision; prepared known answers) -----------------
// Three laps of a circle (radius 6 to 20 m) with the repo's unmodelled yaw bias (0.1 deg per
// 0.5 m step) and white noise of the claimed sigmas (x1, x0.5): 4 TRUE loops lap 2 -> lap 1,
// then 5 TRUE loops lap 3 -> lap 1 (exact, 5 cm / 1 deg). Known answer: every loop stays
// selected, none is removed. Today PCM removes 2 to 4 of the first revisit's loops (their cycles
// with the second revisit's hold a whole lap of biased odometry). Enabled with the B-1 fix.
struct Laps3 {
  std::vector<Pose> truth;
  LoopSelector sel;
  int per_lap = 0;
  Laps3(const LoopConfig& cfg, double radius_m, double claim) : sel(cfg, 4000) {
    std::mt19937 rng(42);
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

TEST(LoopSelector, DISABLED_ThreeLapTrueLoopsAllStaySelected) {
  const LoopConfig cfg = pcm_config();
  for (const double radius : {6.0, 9.0, 12.0, 20.0}) {
    for (const double claim : {1.0, 0.5}) {
      Laps3 L(cfg, radius, claim);
      const int N = L.per_lap;
      int removed = 0;
      int rejected = 0;
      for (int q = 0; q < 4; ++q) {
        rejected += L.sel.propose(L.loop(5 + q * N / 5, N + 5 + q * N / 5)).selected ? 0 : 1;
        removed += static_cast<int>(L.sel.to_remove().size());
      }
      for (int q = 0; q < 5; ++q) {
        rejected += L.sel.propose(L.loop(5 + q * N / 5, 2 * N + 5 + q * N / 5)).selected ? 0 : 1;
        removed += static_cast<int>(L.sel.to_remove().size());
      }
      std::printf(
          "[ pcm three laps ] R %.0f m (%d kf/lap) claim x%.2f: %d removed, %d not "
          "selected, %zu of 9 selected\n",
          radius,
          N,
          claim,
          removed,
          rejected,
          L.sel.n_selected());
      EXPECT_EQ(removed, 0) << radius << " " << claim;
      EXPECT_EQ(rejected, 0) << radius << " " << claim;
      EXPECT_EQ(L.sel.n_selected(), 9U) << radius << " " << claim;
    }
  }
}

}  // namespace
}  // namespace mss::loop
