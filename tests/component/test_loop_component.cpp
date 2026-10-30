// Loop closure end to end (ADR-026), known answers, no randomness except fixed seeds:
//   - LoopComponent.TwoLapDriftClosedByLoops: backend::Graph + loop::LoopDetector on a two-lap
//     circle in the pillar hall with drifting odometry (relative factors carrying a yaw bias,
//     covariance consistent with it): with loops the end-to-start error drops, every accepted
//     loop matches the truth.
//     The detector gets the graph's relative marginal (correction gate), as in the pipeline.
//   - LoopComponent.BoxWorldEstimatorLoops: the whole Estimator on the box world (two laps):
//     loops are found, each one matches the truth, the trajectory is not degraded and the
//     front end keeps matching after the corrections (map rebuilt).
// GTSAM only (the stub backend has no keyframe poses, so loop closure is a no-op there:
// LoopComponent.StubBuildIsANoOp).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/loop/loop_detector.hpp"
#include "mss/loop/loop_selector.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"

#include "box_world.hpp"
#include "estimator_core.hpp"
#include "loop_scenes.hpp"
#include "pipeline_test_support.hpp"

namespace mss::pipeline {
namespace {

using common::KeyId;
using common::Pose;
using common::Vec3;
using common::Vec6;

constexpr double kDeg = test::kLoopPi / 180.0;
constexpr std::int64_t kNsPerS = 1'000'000'000;
constexpr double kLoopTolM = 0.1;  // accepted loop vs truth: accuracy asked (m, rad)
constexpr double kLoopTolRad = 0.5 * kDeg;
// A loop off by more than this is a false loop (also the E-07 evaluation criterion).
constexpr double kFalseLoopM = 0.3;
constexpr double kFalseLoopRad = 3.0 * kDeg;

// --- Graph + LoopDetector, two laps with drift --------------------------------------------

constexpr double kRadiusM = 6.0;              // circle radius in the hall, m
constexpr double kStepM = 0.5;                // keyframe spacing along the circle, m
constexpr int kKeyframes = 151;               // two laps (75.4 keyframes per lap) and the start
constexpr double kYawBiasRad = 0.1 * kDeg;    // odometry yaw bias per keyframe step
constexpr double kYawSigmaRad = 0.15 * kDeg;  // odometry yaw noise sigma per step
constexpr double kPosSigmaM = 0.02;           // odometry position noise sigma per step

Pose circle_pose(int k) {
  const double a = k * kStepM / kRadiusM;  // arc angle
  return test::body_pose(kRadiusM * std::sin(a), kRadiusM * (1.0 - std::cos(a)) - kRadiusM, a);
}

// The pipeline's source of the correction gate (as in EstimatorCore): the graph's relative
// marginal of (i, j).
class GraphRelativeCovariance : public loop::RelativeCovarianceSource {
 public:
  explicit GraphRelativeCovariance(const backend::Graph& g) : g_(g) {}
  common::Result<common::PoseCovariance> relative_cov(KeyId i, KeyId j) const override {
    return g_.relative_pose_cov(i, j);
  }

 private:
  const backend::Graph& g_;
};

struct LapRun {
  double end_to_start_m = 0.0;
  int accepted = 0;
  int false_loops = 0;
  double max_err_m = 0.0;
  double max_err_rad = 0.0;
  int searches = 0;
  int ambiguous = 0;     // rejected by the ambiguity check
  int correction = 0;    // rejected by a correction gate
  int pcm_rejected = 0;  // verified but not selected by PCM
  int removed = 0;       // selected, then removed from the graph by PCM
};

// How a two-lap run is set up (review H3 I-1: the claimed odometry covariance is scaled by
// `claim` while the noise keeps its sigmas, so claim < 1 is an overconfident graph).
struct LapOptions {
  bool loops_on = true;
  double claim = 1.0;
  bool graph_gate = false;  // loop_closure.graph_covariance_gate
  bool pcm = true;          // loop_closure.pcm_enabled
  unsigned seed = 42;       // odometry noise draw
};

LapRun run_two_laps(const Config& cfg, const LapOptions& o);

LapRun run_two_laps(const Config& cfg, bool loops_on) {
  LapOptions o;
  o.loops_on = loops_on;
  return run_two_laps(cfg, o);
}

LapRun run_two_laps(const Config& cfg, const LapOptions& o) {
  LapRun out;
  const bool loops_on = o.loops_on;
  const std::vector<Vec3> hall = test::hall_points_w(0.4);
  const Pose T_b_l = test::loop_T_b_l();
  constexpr std::size_t kMaxPoints = 20000;
  backend::Graph graph(cfg.backend, cfg.imu, Vec3::Zero());
  loop::LoopConfig lc = cfg.loop_closure;
  lc.enabled = loops_on;
  lc.graph_covariance_gate = o.graph_gate;
  lc.pcm_enabled = o.pcm;
  loop::LoopDetector det(lc, cfg.scan_match, cfg.map, T_b_l, kMaxPoints);
  loop::LoopSelector sel(lc, kKeyframes);
  std::vector<std::unique_ptr<common::ScanFrame>> clouds;
  std::vector<loop::KeyframeView> views;
  std::mt19937 rng(o.seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  common::PoseCovariance cov = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    cov(a, a) = o.claim * o.claim * kPosSigmaM * kPosSigmaM;
    cov(a + 3, a + 3) = o.claim * o.claim * kYawSigmaRad * kYawSigmaRad;
  }
  Pose guess = circle_pose(0);
  int skip = 0;
  for (int k = 0; k < kKeyframes; ++k) {
    const common::Timestamp t{static_cast<std::int64_t>(k) * kNsPerS};
    if (k > 0) {
      // Drifting odometry: truth with a yaw bias and white noise of the claimed sigmas.
      const Pose truth = circle_pose(k - 1).inverse() * circle_pose(k);
      Vec6 e;
      e << kPosSigmaM * n01(rng), kPosSigmaM * n01(rng), 0.0, 0.0, 0.0,
          kYawBiasRad + kYawSigmaRad * n01(rng);
      const Pose meas = truth * common::se3_exp(e);
      guess = guess * meas;
      common::NavState x0;
      x0.t = t;
      x0.T_w_b = guess;
      graph.add_keyframe(t, x0);
      graph.add_lidar_factor(static_cast<KeyId>(k - 1), static_cast<KeyId>(k), meas, cov, nullptr);
      if (loops_on && o.pcm) {
        EXPECT_TRUE(sel.add_odometry(static_cast<KeyId>(k), meas, cov));
      }
    } else {
      common::NavState x0;
      x0.t = t;
      x0.T_w_b = guess;
      graph.add_keyframe(t, x0);
    }
    EXPECT_TRUE(graph.optimize().is_ok());
    clouds.push_back(std::make_unique<common::ScanFrame>(kMaxPoints));
    test::scan_at(hall, circle_pose(k) * T_b_l, 15.0, *clouds.back());
    if (!loops_on) {
      continue;
    }
    if (skip > 0) {
      --skip;
      continue;
    }
    views.clear();
    for (int i = 0; i <= k; ++i) {
      loop::KeyframeView v;
      v.id = static_cast<KeyId>(i);
      v.t = common::Timestamp{static_cast<std::int64_t>(i) * kNsPerS};
      v.T_w_b = graph.keyframe_state(static_cast<KeyId>(i)).value().T_w_b;
      v.cloud_l = clouds[static_cast<std::size_t>(i)].get();
      views.push_back(v);
    }
    const GraphRelativeCovariance rel(graph);
    const loop::LoopResult r =
        det.detect(views.back(), common::Span<const loop::KeyframeView>(views), &rel);
    out.searches += 1;
    out.ambiguous += r.outcome == loop::LoopOutcome::kAmbiguous ? 1 : 0;
    out.correction += r.outcome == loop::LoopOutcome::kCorrection ? 1 : 0;
    if (r.outcome != loop::LoopOutcome::kAccepted) {
      continue;
    }
    // ADR-028: PCM selects (as EstimatorCore::try_loop_closure does).
    bool selected = true;
    if (o.pcm) {
      selected = sel.propose(r.constraint).selected;
      for (const auto& ij : sel.to_remove()) {
        EXPECT_TRUE(graph.remove_loop_factor(ij.first, ij.second));
        ++out.removed;
      }
      for (const common::LoopConstraint& a : sel.to_add()) {
        graph.add_loop_factor(a);
      }
      out.pcm_rejected += selected ? 0 : 1;
      if (sel.to_add().empty() && sel.to_remove().empty()) {
        continue;
      }
    } else {
      graph.add_loop_factor(r.constraint);
    }
    for (int u = 0; u <= lc.isam_extra_updates; ++u) {
      EXPECT_TRUE(graph.optimize().is_ok());
    }
    guess = graph.keyframe_state(static_cast<KeyId>(k)).value().T_w_b;
    if (!selected) {
      continue;
    }
    skip = lc.min_interval_keyframes;
    ++out.accepted;
    const Pose truth = circle_pose(static_cast<int>(r.constraint.i)).inverse() * circle_pose(k);
    const double e_p = (r.constraint.T_bi_bj.p - truth.p).norm();
    const double e_yaw = common::so3_log(truth.R.transpose() * r.constraint.T_bi_bj.R).norm();
    out.false_loops += (e_p > kFalseLoopM || e_yaw > kFalseLoopRad) ? 1 : 0;
    out.max_err_m = std::max(out.max_err_m, e_p);
    out.max_err_rad = std::max(out.max_err_rad, e_yaw);
    std::printf("  loop %d -> %llu: error %.3f m %.3f deg, sc %.3f ratio %.3f rms %.3f\n",
                k,
                static_cast<unsigned long long>(r.constraint.i),
                e_p,
                e_yaw / kDeg,
                r.sc_distance,
                r.inlier_ratio,
                r.rms_m);
  }
  {
    const Pose e0 = graph.keyframe_state(0).value().T_w_b;
    const Pose eN = graph.keyframe_state(static_cast<KeyId>(kKeyframes - 1)).value().T_w_b;
    const Vec3 d_est = e0.inverse() * eN.p;
    const Vec3 d_true = circle_pose(0).inverse() * circle_pose(kKeyframes - 1).p;
    out.end_to_start_m = (d_est - d_true).norm();
  }
  return out;
}

Config shipped_config() {
  const auto r = load_config(ct::kDefault);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  return r.value();
}

TEST(LoopComponent, TwoLapDriftClosedByLoops) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const Config cfg = shipped_config();
  const LapRun off = run_two_laps(cfg, false);
  const LapRun on = run_two_laps(cfg, true);
  std::printf(
      "[ loop two laps ] end-to-start off %.3f m, on %.3f m; %d searches, %d loops accepted, "
      "%d false, %d ambiguous, %d correction-gated\n",
      off.end_to_start_m,
      on.end_to_start_m,
      on.searches,
      on.accepted,
      on.false_loops,
      on.ambiguous,
      on.correction);
  RecordProperty("end_to_start_off_m", std::to_string(off.end_to_start_m));
  RecordProperty("end_to_start_on_m", std::to_string(on.end_to_start_m));
  EXPECT_EQ(off.accepted, 0);
  EXPECT_GT(off.end_to_start_m, 0.5);  // the drift is real
  EXPECT_GE(on.accepted, 3);
  EXPECT_EQ(on.false_loops, 0);
  EXPECT_LT(on.max_err_m, kLoopTolM);
  EXPECT_LT(on.max_err_rad, kLoopTolRad);
  EXPECT_LT(on.end_to_start_m, 0.2 * off.end_to_start_m);
}

// PCM compared with no PCM on the same run (review H3, I-1): the two-lap drift test over a
// fixed list of odometry seeds, claimed covariance as the noise. PCM selects the same loops as
// without it (none rejected, none removed) and the end-to-start error is the same within 1 mm;
// the loops remove at least 20 % of the drift for every seed (loops-off error 0.61 to 1.79 m).
// Measured: seeds 42 / 1 / 7 / 2024 give 0.082 / 0.125 / 1.199 / 0.454 m with and without PCM.
struct SeedRun {
  LapRun off;
  LapRun pcm;
  LapRun no_pcm;
};

SeedRun seed_run(const Config& cfg, unsigned seed, double claim) {
  SeedRun r;
  LapOptions off;
  off.loops_on = false;
  off.seed = seed;
  r.off = run_two_laps(cfg, off);
  LapOptions a;
  a.seed = seed;
  a.claim = claim;
  r.pcm = run_two_laps(cfg, a);
  LapOptions b = a;
  b.pcm = false;
  r.no_pcm = run_two_laps(cfg, b);
  std::printf(
      "[ loop two laps ] seed %u claim x%.2f: end-to-start off %.3f m, PCM %.3f m "
      "(%d loops, %d PCM-rejected, %d removed), no PCM %.3f m (%d loops)\n",
      seed,
      claim,
      r.off.end_to_start_m,
      r.pcm.end_to_start_m,
      r.pcm.accepted,
      r.pcm.pcm_rejected,
      r.pcm.removed,
      r.no_pcm.end_to_start_m,
      r.no_pcm.accepted);
  return r;
}

constexpr unsigned kSeeds[] = {42U, 1U, 7U, 2024U};

TEST(LoopComponent, TwoLapPcmNoWorseThanWithoutAcrossSeeds) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const Config cfg = shipped_config();
  for (const unsigned seed : kSeeds) {
    const SeedRun r = seed_run(cfg, seed, 1.0);
    EXPECT_EQ(r.pcm.accepted, r.no_pcm.accepted) << seed;
    EXPECT_EQ(r.pcm.pcm_rejected, 0) << seed;
    EXPECT_EQ(r.pcm.removed, 0) << seed;
    EXPECT_EQ(r.pcm.false_loops, 0) << seed;
    EXPECT_LE(r.pcm.end_to_start_m, r.no_pcm.end_to_start_m + 1e-3) << seed;
    EXPECT_LT(r.pcm.end_to_start_m, 0.8 * r.off.end_to_start_m) << seed;
  }
}

// Review H3 B-1, fixed by the yaw-drift term (ADR-028 amendment C): the same with the claimed
// covariance halved and quartered (overconfident odometry). PCM is no worse than without it.
// Before the drift term: seed 7 x0.5 rejected one true loop (1.350 m against 1.129 m without
// PCM) and seed 2024 x0.25 one (0.576 against 0.511 m).
TEST(LoopComponent, TwoLapOverconfidentPcmNoWorseThanWithoutAcrossSeeds) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const Config cfg = shipped_config();
  for (const double claim : {0.5, 0.25}) {
    for (const unsigned seed : kSeeds) {
      const SeedRun r = seed_run(cfg, seed, claim);
      EXPECT_EQ(r.pcm.accepted, r.no_pcm.accepted) << seed << " " << claim;
      EXPECT_EQ(r.pcm.removed, 0) << seed << " " << claim;
      EXPECT_LE(r.pcm.end_to_start_m, r.no_pcm.end_to_start_m + 1e-3) << seed << " " << claim;
    }
  }
}

// PCM against false loops that passed verification (the ambiguity check missed them), with the
// loops given directly (no scans, so detection recall does not depend on the graph): the
// two-lap odometry of LoopComponent.TwoLapDrift, 8 true loops one lap apart (truth, floored
// covariance 5 cm / 1 deg) and 3 loops aliased by one 2 m period along world x, one of them
// before any true loop (so it is selected first). PCM selects every true loop, removes the
// early aliased one and never selects the others; end-to-start within 1 cm of the run without
// the aliased loops. Without PCM (Cauchy only) they stay and drag the lap (here little: eight
// true loops outvote them).
struct OracleRun {
  double end_to_start_m = 0.0;
  int removed = 0;
  int rejected = 0;
  int aliased_in_graph = 0;
};

OracleRun run_oracle_laps(const Config& cfg, bool with_aliased, bool pcm) {
  OracleRun out;
  backend::Graph graph(cfg.backend, cfg.imu, Vec3::Zero());
  loop::LoopConfig lc = cfg.loop_closure;
  lc.enabled = true;
  lc.pcm_enabled = pcm;
  loop::LoopSelector sel(lc, kKeyframes);
  std::mt19937 rng(42);
  std::normal_distribution<double> n01(0.0, 1.0);
  common::PoseCovariance cov = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    cov(a, a) = kPosSigmaM * kPosSigmaM;
    cov(a + 3, a + 3) = kYawSigmaRad * kYawSigmaRad;
  }
  common::PoseCovariance lcov = common::PoseCovariance::Zero();
  lcov.diagonal() << 0.0025, 0.0025, 0.0025, 3.05e-4, 3.05e-4, 3.05e-4;
  struct Proposed {
    int i;
    int j;
    bool aliased;
  };
  std::vector<Proposed> loops = {{0, 75, false},
                                 {9, 84, false},
                                 {23, 98, false},
                                 {38, 113, false},
                                 {50, 125, false},
                                 {60, 135, false},
                                 {68, 143, false},
                                 {0, 150, false}};
  if (with_aliased) {
    loops.push_back({1, 70, true});
    loops.push_back({20, 110, true});
    loops.push_back({65, 145, true});
  }
  Pose guess = circle_pose(0);
  for (int k = 0; k < kKeyframes; ++k) {
    const common::Timestamp t{static_cast<std::int64_t>(k) * kNsPerS};
    if (k > 0) {
      const Pose truth = circle_pose(k - 1).inverse() * circle_pose(k);
      Vec6 e;
      e << kPosSigmaM * n01(rng), kPosSigmaM * n01(rng), 0.0, 0.0, 0.0,
          kYawBiasRad + kYawSigmaRad * n01(rng);
      const Pose meas = truth * common::se3_exp(e);
      guess = guess * meas;
      common::NavState x0;
      x0.t = t;
      x0.T_w_b = guess;
      graph.add_keyframe(t, x0);
      graph.add_lidar_factor(static_cast<KeyId>(k - 1), static_cast<KeyId>(k), meas, cov, nullptr);
      if (pcm) {
        EXPECT_TRUE(sel.add_odometry(static_cast<KeyId>(k), meas, cov));
      }
    } else {
      common::NavState x0;
      x0.t = t;
      x0.T_w_b = guess;
      graph.add_keyframe(t, x0);
    }
    EXPECT_TRUE(graph.optimize().is_ok());
    for (const Proposed& l : loops) {
      if (l.j != k) {
        continue;
      }
      common::LoopConstraint c;
      c.i = static_cast<KeyId>(l.i);
      c.j = static_cast<KeyId>(l.j);
      Pose T_j = circle_pose(l.j);
      T_j.p.x() += l.aliased ? 2.0 : 0.0;
      c.T_bi_bj = circle_pose(l.i).inverse() * T_j;
      c.cov = lcov;
      if (!pcm) {
        graph.add_loop_factor(c);
      } else {
        out.rejected += sel.propose(c).selected ? 0 : 1;
        for (const auto& ij : sel.to_remove()) {
          EXPECT_TRUE(graph.remove_loop_factor(ij.first, ij.second));
          ++out.removed;
        }
        for (const common::LoopConstraint& a : sel.to_add()) {
          graph.add_loop_factor(a);
        }
      }
      for (int u = 0; u <= lc.isam_extra_updates; ++u) {
        EXPECT_TRUE(graph.optimize().is_ok());
      }
      guess = graph.keyframe_state(static_cast<KeyId>(k)).value().T_w_b;
    }
  }
  const Pose e0 = graph.keyframe_state(0).value().T_w_b;
  const Pose eN = graph.keyframe_state(static_cast<KeyId>(kKeyframes - 1)).value().T_w_b;
  out.end_to_start_m =
      ((e0.inverse() * eN.p) - (circle_pose(0).inverse() * circle_pose(kKeyframes - 1).p)).norm();
  for (const Proposed& l : loops) {
    if (l.aliased && pcm) {
      out.aliased_in_graph +=
          graph.remove_loop_factor(static_cast<KeyId>(l.i), static_cast<KeyId>(l.j)) ? 1 : 0;
    }
  }
  return out;
}

TEST(LoopComponent, PcmRemovesAliasedLoopsKnownLoops) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const Config cfg = shipped_config();
  const OracleRun clean = run_oracle_laps(cfg, false, true);
  const OracleRun pcm = run_oracle_laps(cfg, true, true);
  const OracleRun raw = run_oracle_laps(cfg, true, false);
  std::printf(
      "[ loop pcm ] known loops + 3 aliased: end-to-start PCM %.3f m (no aliased %.3f m, "
      "Cauchy only %.3f m); %d removed, %d rejected, %d aliased left in the graph\n",
      pcm.end_to_start_m,
      clean.end_to_start_m,
      raw.end_to_start_m,
      pcm.removed,
      pcm.rejected,
      pcm.aliased_in_graph);
  EXPECT_EQ(clean.rejected, 0);  // every true loop is selected
  EXPECT_EQ(pcm.aliased_in_graph, 0);
  EXPECT_GE(pcm.removed, 1);  // (1, 70) was selected first, then removed
  EXPECT_LT(std::abs(pcm.end_to_start_m - clean.end_to_start_m), 0.01);
  EXPECT_GT(raw.end_to_start_m, pcm.end_to_start_m);  // the kernel leaves some drag
}

// --- Whole estimator on the box world -------------------------------------------------------

// Records the accepted loops with their keyframe times.
class LoopObserver : public detail::CoreObserver {
 public:
  void on_loop_search(common::Timestamp t, const loop::LoopResult& r) override {
    ++searches;
    if (r.outcome == loop::LoopOutcome::kAccepted) {
      accepted.push_back({r.candidate_t, t, r.constraint.T_bi_bj, r.constraint.i, r.constraint.j});
    }
  }
  struct Loop {
    common::Timestamp t_i;
    common::Timestamp t_j;
    Pose T_bi_bj;
    KeyId i;
    KeyId j;
  };
  int searches = 0;
  std::vector<Loop> accepted;
};

TEST(LoopComponent, BoxWorldEstimatorLoops) {
  test::BoxWorldParams p;
  p.motion_s = 55.0;  // two laps of the 2 m circle (25.1 s per lap) after the ramp
  const std::filesystem::path dir = ct::test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const Config off = ct::box_config(p, false, false, dir / "off");
  const Config on =
      ct::box_config(p, false, false, dir / "on", "    loop_closure:\n      enabled: true\n");
  const ct::ReplayResult r_off = ct::replay(dir / "data", off);
  LoopObserver obs;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  ct::ReplayResult r_on;
  {
    ct::HooksScope scope(&hooks);
    r_on = ct::replay(dir / "data", on);
  }
  const ct::Errors e_off = ct::trajectory_errors(p, r_off.estimates);
  const ct::Errors e_on = ct::trajectory_errors(p, r_on.estimates);
  std::printf(
      "[ loop box world ] gtsam %d: ATE off %.4f m on %.4f m; searches %llu, accepted %llu, "
      "rejected %llu, rebuilds %llu, match failures off %llu on %llu\n",
      ct::kWithGtsam ? 1 : 0,
      e_off.ate_m,
      e_on.ate_m,
      static_cast<unsigned long long>(r_on.stats.n_loop_searches),
      static_cast<unsigned long long>(r_on.stats.n_loops_accepted),
      static_cast<unsigned long long>(r_on.stats.n_loops_rejected),
      static_cast<unsigned long long>(r_on.stats.n_map_rebuilds),
      static_cast<unsigned long long>(r_off.stats.n_match_failures),
      static_cast<unsigned long long>(r_on.stats.n_match_failures));
  EXPECT_EQ(r_off.stats.n_loop_searches, 0U);
  EXPECT_EQ(r_off.stats.n_loops_accepted, 0U);
  if (!ct::kWithGtsam) {
    // Stub backend: no keyframe pose, so no search runs and the result is the flag-off one.
    EXPECT_EQ(r_on.stats.n_loop_searches, 0U);
    EXPECT_EQ(r_on.estimates.size(), r_off.estimates.size());
    return;
  }
  EXPECT_GT(r_on.stats.n_loops_accepted, 0U);
  EXPECT_EQ(static_cast<std::size_t>(r_on.stats.n_loops_accepted), obs.accepted.size());
  EXPECT_GE(r_on.stats.n_map_rebuilds, r_on.stats.n_loops_accepted);  // forced rebuild per loop
  for (const LoopObserver::Loop& l : obs.accepted) {
    const Pose truth = test::box_truth(p, l.t_i).T_w_b.inverse() * test::box_truth(p, l.t_j).T_w_b;
    EXPECT_LT((l.T_bi_bj.p - truth.p).norm(), kLoopTolM);
    EXPECT_LT(common::so3_log(truth.R.transpose() * l.T_bi_bj.R).norm(), kLoopTolRad);
  }
  EXPECT_EQ(r_on.estimates.size(), r_off.estimates.size());
  EXPECT_EQ(r_on.stats.n_match_failures, r_off.stats.n_match_failures);
  // No drift to remove on the box world: the loops must not degrade it (1 mm slack).
  EXPECT_LE(e_on.ate_m, 1.2 * e_off.ate_m + 0.001);
}

// --- The pipeline's PCM path (review H3, I-4) ------------------------------------------------
// The whole Estimator on the box world (two laps of the 2 m circle, lap period 2 pi / 0.25 s),
// the detector replaced by CoreTestHooks::loop_inject: at three moments of lap 2 a verified
// loop to the keyframe one lap earlier, from the truth with the floor covariance (5 cm / 1 deg).
// The first is wrong (along x of b_i: 0.5 m in the forget test, 0.8 m on a loosened chain in
// the removal test, where it drags the result by centimetres), the next two are true. Known answer
// through EstimatorCore::try_loop_closure: the wrong loop enters first, the first true one ties and
// is left out, the second true one outvotes the wrong one, which is removed from iSAM2 while the
// first true one is added back; counters, map rebuilds and the front-end re-sync follow, the graph
// holds the two true loops only, and the trajectory ends as the run that only ever had the two true
// loops.
constexpr double kInjectSigmaM = 0.05;          // injected loop covariance, position sigma (m)
constexpr double kInjectSigmaRad = 1.0 * kDeg;  // injected loop covariance, rotation sigma (rad)

struct InjectPlan {
  test::BoxWorldParams p;
  std::vector<double> at_s;     // s after t0: inject at the first keyframe at or after it
  std::vector<double> wrong_m;  // offset along x of b_i, m (0: true loop)
  std::vector<bool> done;
  double lap_s = 0.0;
  std::vector<std::pair<KeyId, KeyId>> injected;  // (i, j) of each injection, in order
};

std::optional<common::LoopConstraint> inject(InjectPlan& plan,
                                             common::Span<const loop::KeyframeView> views) {
  const loop::KeyframeView& q = views[views.size() - 1];
  const double tq = 1e-9 * static_cast<double>(q.t.ns - plan.p.t0_ns);
  for (std::size_t n = 0; n < plan.at_s.size(); ++n) {
    if (plan.done[n] || tq < plan.at_s[n]) {
      continue;
    }
    plan.done[n] = true;
    // The keyframe nearest to one lap earlier.
    std::size_t best = 0;
    double best_dt = 1e9;
    for (std::size_t k = 0; k + 1 < views.size(); ++k) {
      const double dt = std::abs(1e-9 * static_cast<double>(q.t.ns - views[k].t.ns) - plan.lap_s);
      if (dt < best_dt) {
        best_dt = dt;
        best = k;
      }
    }
    common::LoopConstraint lc;
    lc.i = views[best].id;
    lc.j = q.id;
    lc.T_bi_bj =
        test::box_truth(plan.p, views[best].t).T_w_b.inverse() * test::box_truth(plan.p, q.t).T_w_b;
    lc.T_bi_bj.p.x() += plan.wrong_m[n];
    lc.cov.setZero();
    const double vm = kInjectSigmaM * kInjectSigmaM;
    const double vr = kInjectSigmaRad * kInjectSigmaRad;
    lc.cov.diagonal() << vm, vm, vm, vr, vr, vr;
    plan.injected.emplace_back(lc.i, lc.j);
    return lc;
  }
  return std::nullopt;
}

struct InjectRun {
  ct::ReplayResult r;
  ct::Errors e;
  std::vector<std::pair<KeyId, KeyId>> injected;  // (i, j) of each injection, in order
  std::vector<bool> in_graph;  // Graph::has_loop_factor of each, after the last graph change
};

InjectRun run_injected(const std::string& name,
                       std::vector<double> at_s,
                       std::vector<double> wrong_m,
                       bool pcm,
                       int drop_updates,
                       double lidar_var_scale = 1.0) {
  InjectPlan plan;
  plan.p.motion_s = 55.0;
  plan.lap_s = 2.0 * test::kLoopPi / plan.p.yaw_rate_radps;
  plan.at_s = std::move(at_s);
  plan.wrong_m = std::move(wrong_m);
  plan.done.assign(plan.at_s.size(), false);
  const std::filesystem::path dir = ct::test_dir() / name;
  EXPECT_TRUE(test::write_box_dataset(dir / "data", plan.p).is_ok());
  const Config cfg =
      ct::box_config(plan.p,
                     false,
                     false,
                     dir / "cfg",
                     std::string("    loop_closure:\n      enabled: true\n      pcm_enabled: ") +
                         (pcm ? "true" : "false") + "\n");
  detail::CoreTestHooks hooks;
  hooks.loop_inject = [&plan](common::Span<const loop::KeyframeView> v) {
    loop::LoopResult r;
    const std::optional<common::LoopConstraint> lc = inject(plan, v);
    r.outcome = lc ? loop::LoopOutcome::kAccepted : loop::LoopOutcome::kNoCandidate;
    if (lc) {
      r.constraint = *lc;
      r.n_verified = 1;
    }
    return r;
  };
  hooks.drop_loop_updates = drop_updates;
  if (lidar_var_scale != 1.0) {
    hooks.lidar_cov = [lidar_var_scale](common::Timestamp /*t*/, const common::PoseCovariance& c) {
      const common::PoseCovariance scaled = lidar_var_scale * c;
      return scaled;
    };
  }
  InjectRun out;
  hooks.inspect_graph = [&plan, &out](const backend::Graph& g) {
    out.in_graph.clear();
    for (const auto& ij : plan.injected) {
      out.in_graph.push_back(g.has_loop_factor(ij.first, ij.second));
    }
  };
  {
    ct::HooksScope scope(&hooks);
    out.r = ct::replay(dir / "data", cfg);
  }
  out.e = ct::trajectory_errors(plan.p, out.r.estimates);
  for (std::size_t n = 0; n < plan.done.size(); ++n) {
    EXPECT_TRUE(plan.done[n]) << name << ": injection " << n << " never happened";
  }
  out.injected = plan.injected;
  return out;
}

TEST(LoopComponent, PipelinePcmRemovesAWrongLoopAndReAddsTheTrueOne) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const double t_lap2 = 3.0 + 2.0 + 2.0 * test::kLoopPi / 0.25;  // static + ramp + one lap
  const std::vector<double> at = {t_lap2 + 2.0, t_lap2 + 8.0, t_lap2 + 14.0};
  // A loose LiDAR chain (its covariance x100 through the lidar_cov seam) so that the wrong loop,
  // 0.8 m off, drags the graph by centimetres while it is in (review H3 round 2, I-2); on the
  // shipped stiff chain it moved the result by millimetres, too little to see a removal.
  constexpr double kLoose = 100.0;
  constexpr double kWrongM = 0.8;
  const InjectRun a = run_injected("wrong_then_true", at, {kWrongM, 0.0, 0.0}, true, 0, kLoose);
  const InjectRun b = run_injected("true_only", {at[1], at[2]}, {0.0, 0.0}, true, 0, kLoose);
  const InjectRun c = run_injected("wrong_kept", {at[0]}, {kWrongM}, false, 0, kLoose);
  ASSERT_FALSE(a.r.estimates.empty());
  ASSERT_FALSE(b.r.estimates.empty());
  ASSERT_FALSE(c.r.estimates.empty());
  const Pose ea = a.r.estimates.back().state.T_w_b;
  const Pose eb = b.r.estimates.back().state.T_w_b;
  const Pose ec = c.r.estimates.back().state.T_w_b;
  std::printf(
      "[ loop pcm pipeline ] wrong then true: %llu accepted, %llu removed, %llu "
      "PCM-rejected, %llu rebuilds; final pose against true only: %.4f m; wrong kept (no "
      "PCM) against true only: %.4f m\n",
      static_cast<unsigned long long>(a.r.stats.n_loops_accepted),
      static_cast<unsigned long long>(a.r.stats.n_loops_removed),
      static_cast<unsigned long long>(a.r.stats.n_loops_pcm_rejected),
      static_cast<unsigned long long>(a.r.stats.n_map_rebuilds),
      (ea.p - eb.p).norm(),
      (ec.p - eb.p).norm());
  // Counters: wrong in (1), first true left out, second true in with the first true back (2).
  EXPECT_EQ(a.r.stats.n_loops_accepted, 3U);
  EXPECT_EQ(a.r.stats.n_loops_removed, 1U);
  EXPECT_EQ(a.r.stats.n_loops_pcm_rejected, 1U);
  EXPECT_EQ(b.r.stats.n_loops_accepted, 2U);
  EXPECT_EQ(b.r.stats.n_loops_removed, 0U);
  // A forced map rebuild for each graph change (2 here: the wrong loop, then the swap).
  EXPECT_GE(a.r.stats.n_map_rebuilds, b.r.stats.n_map_rebuilds);
  EXPECT_GE(a.r.stats.n_map_rebuilds, 2U);
  // The graph after the last change holds the two true loops and not the wrong one
  // (Graph::has_loop_factor through CoreTestHooks::inspect_graph).
  ASSERT_EQ(a.injected.size(), 3U);
  ASSERT_EQ(a.in_graph.size(), 3U);
  EXPECT_FALSE(a.in_graph[0]);
  EXPECT_TRUE(a.in_graph[1]);
  EXPECT_TRUE(a.in_graph[2]);
  ASSERT_EQ(c.in_graph.size(), 1U);
  EXPECT_TRUE(c.in_graph[0]);
  // The wrong loop moves the result by centimetres when it stays (measured 6.0 cm); removed, the
  // run ends as the run that only ever had the two true loops (measured 3 mm), the front end
  // re-synced to it.
  EXPECT_GT((ec.p - eb.p).norm(), 0.03);
  EXPECT_LT((ea.p - eb.p).norm(), 0.01);
}

// Review H3 I-2: in repeated structure every search is ambiguous; after a kAmbiguous result
// the pipeline skips ambiguity_backoff_keyframes keyframes. Every search is made ambiguous
// (injected) on the box world: with a back-off of B the searches are about 1 / (1 + B) of
// those with none (counted by the hook, which only runs when a search runs).
TEST(LoopComponent, AmbiguousResultBacksOffTheSearch) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  test::BoxWorldParams p;
  p.motion_s = 30.0;
  const std::filesystem::path dir = ct::test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  int calls[2] = {0, 0};
  std::uint64_t searches[2] = {0, 0};
  const int backoff[2] = {0, 5};
  for (int n = 0; n < 2; ++n) {
    const Config cfg = ct::box_config(
        p,
        false,
        false,
        dir / ("cfg" + std::to_string(n)),
        "    loop_closure:\n      enabled: true\n      ambiguity_backoff_keyframes: " +
            std::to_string(backoff[n]) + "\n");
    detail::CoreTestHooks hooks;
    hooks.loop_inject = [&calls, n](common::Span<const loop::KeyframeView> /*v*/) {
      ++calls[n];
      loop::LoopResult r;
      r.outcome = loop::LoopOutcome::kAmbiguous;
      r.n_verified = 1;
      return r;
    };
    ct::ReplayResult r;
    {
      ct::HooksScope scope(&hooks);
      r = ct::replay(dir / "data", cfg);
    }
    searches[n] = r.stats.n_loop_searches;
  }
  std::printf("[ loop backoff ] searches without back-off %llu, with 5: %llu\n",
              static_cast<unsigned long long>(searches[0]),
              static_cast<unsigned long long>(searches[1]));
  EXPECT_EQ(static_cast<std::uint64_t>(calls[0]), searches[0]);
  EXPECT_EQ(static_cast<std::uint64_t>(calls[1]), searches[1]);
  ASSERT_GT(searches[0], 12U);
  // One search per 6 keyframes (the last block may be partial).
  EXPECT_LE(searches[1], searches[0] / 6 + 1);
  EXPECT_GE(searches[1], searches[0] / 6);
}

// Review H3 m-1: the update after the wrong loop entered fails and drops it (injected); the
// selector forgets it, so the first true loop is not left out by a tie with a loop that is not
// in the graph: two loops accepted, none removed, the run ends as with the true loops only.
TEST(LoopComponent, PipelinePcmForgetsALoopDroppedByAFailedUpdate) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  const double t_lap2 = 3.0 + 2.0 + 2.0 * test::kLoopPi / 0.25;
  const std::vector<double> at = {t_lap2 + 2.0, t_lap2 + 8.0, t_lap2 + 14.0};
  const InjectRun a = run_injected("dropped_wrong", at, {0.5, 0.0, 0.0}, true, 1);
  const InjectRun b = run_injected("true_only", {at[1], at[2]}, {0.0, 0.0}, true, 0);
  EXPECT_EQ(a.r.stats.n_loops_accepted, 2U);
  EXPECT_EQ(a.r.stats.n_loops_removed, 0U);
  EXPECT_EQ(a.r.stats.n_loops_pcm_rejected, 0U);
  EXPECT_LT(std::abs(a.e.ate_m - b.e.ate_m), 1e-3);
}

// I-4 (review C1): every iSAM2 update after the one that adds the loop fails (injected). The
// loop factor is committed by then, so the loop must count, the front end re-sync and the next
// min_interval_keyframes keyframes skip the search: no loop is added twice.
TEST(LoopComponent, ExtraUpdateFailureKeepsLoopCountedAndNotDuplicated) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (stub backend has no keyframe poses)";
  }
  test::BoxWorldParams p;
  p.motion_s = 55.0;
  const std::filesystem::path dir = ct::test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const Config on =
      ct::box_config(p, false, false, dir / "on", "    loop_closure:\n      enabled: true\n");
  ASSERT_GT(on.loop_closure.isam_extra_updates, 0);
  LoopObserver obs;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  hooks.fail_loop_extra_updates = true;
  ct::ReplayResult r;
  {
    ct::HooksScope scope(&hooks);
    r = ct::replay(dir / "data", on);
  }
  ASSERT_GT(obs.accepted.size(), 0U);
  // Every verified loop was added once and counted once.
  EXPECT_EQ(static_cast<std::size_t>(r.stats.n_loops_accepted), obs.accepted.size());
  EXPECT_GE(r.stats.n_map_rebuilds, r.stats.n_loops_accepted);
  for (std::size_t a = 1; a < obs.accepted.size(); ++a) {
    // The search was skipped for min_interval_keyframes keyframes after each loop.
    EXPECT_GT(obs.accepted[a].j - obs.accepted[a - 1].j,
              static_cast<KeyId>(on.loop_closure.min_interval_keyframes));
    for (std::size_t b = 0; b < a; ++b) {
      EXPECT_FALSE(obs.accepted[a].i == obs.accepted[b].i &&
                   obs.accepted[a].j == obs.accepted[b].j);
    }
  }
  const ct::Errors e = ct::trajectory_errors(p, r.estimates);
  EXPECT_LT(e.ate_m, 0.05);
}

// --- Repeated structure (C1-12) and known limits (review C1, I-2; ADR-026 amendment) --------
// LoopKnownLimits.* pin CURRENT behaviour stated in ADR-026; a fix (backlog C1-13, C1-3)
// changes them on purpose.

// Closed lap of n keyframes on a circle (0.5 m steps), noise-free odometry with the given
// sigmas, then a loop (0, n-1) whose measurement is truth * correction with covariance lcov.
// Returns how far the last keyframe moved relative to keyframe 0 (the applied correction).
double applied_correction(const Config& cfg,
                          const Pose& correction,
                          const common::PoseCovariance& lcov,
                          double sig_m,
                          double sig_rad) {
  constexpr int kN = 61;
  backend::Graph g(cfg.backend, cfg.imu, Vec3::Zero());
  const double R = 0.5 * (kN - 1) / (2.0 * test::kLoopPi);
  const auto truth = [&](int k) {
    const double a = 2.0 * test::kLoopPi * k / (kN - 1);
    return test::body_pose(R * std::sin(a), R * (1.0 - std::cos(a)), a);
  };
  common::PoseCovariance cov = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    cov(a, a) = sig_m * sig_m;
    cov(a + 3, a + 3) = sig_rad * sig_rad;
  }
  for (int k = 0; k < kN; ++k) {
    common::NavState x0;
    x0.t = common::Timestamp{k * kNsPerS};
    x0.T_w_b = truth(k);
    g.add_keyframe(x0.t, x0);
    if (k > 0) {
      g.add_lidar_factor(static_cast<KeyId>(k - 1),
                         static_cast<KeyId>(k),
                         truth(k - 1).inverse() * truth(k),
                         cov,
                         nullptr);
    }
    EXPECT_TRUE(g.optimize().is_ok());
  }
  common::LoopConstraint lc;
  lc.i = 0;
  lc.j = static_cast<KeyId>(kN - 1);
  lc.T_bi_bj = truth(0).inverse() * truth(kN - 1) * correction;
  lc.cov = lcov;
  g.add_loop_factor(lc);
  for (int u = 0; u < 3; ++u) {
    EXPECT_TRUE(g.optimize().is_ok());
  }
  const Pose T0 = g.keyframe_state(0).value().T_w_b;
  const Pose Tn = g.keyframe_state(lc.j).value().T_w_b;
  return ((T0.inverse() * Tn).p - (truth(0).inverse() * truth(kN - 1)).p).norm();
}

// Relative covariance of (0, n-1) of the closed lap above with the given chain sigmas, from
// the graph (as the pipeline's correction gate sees it).
common::PoseCovariance lap_relative_cov(const Config& cfg, double sig_m, double sig_rad) {
  constexpr int kN = 61;
  backend::Graph g(cfg.backend, cfg.imu, Vec3::Zero());
  const double R = 0.5 * (kN - 1) / (2.0 * test::kLoopPi);
  const auto truth = [&](int k) {
    const double a = 2.0 * test::kLoopPi * k / (kN - 1);
    return test::body_pose(R * std::sin(a), R * (1.0 - std::cos(a)), a);
  };
  common::PoseCovariance cov = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    cov(a, a) = sig_m * sig_m;
    cov(a + 3, a + 3) = sig_rad * sig_rad;
  }
  for (int k = 0; k < kN; ++k) {
    common::NavState x0;
    x0.t = common::Timestamp{k * kNsPerS};
    x0.T_w_b = truth(k);
    g.add_keyframe(x0.t, x0);
    if (k > 0) {
      g.add_lidar_factor(static_cast<KeyId>(k - 1),
                         static_cast<KeyId>(k),
                         truth(k - 1).inverse() * truth(k),
                         cov,
                         nullptr);
    }
    EXPECT_TRUE(g.optimize().is_ok());
  }
  const common::Result<common::PoseCovariance> r = g.relative_pose_cov(0, kN - 1);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  return r.is_ok() ? r.value() : common::PoseCovariance::Identity();
}

class FixedRelativeCovariance : public loop::RelativeCovarianceSource {
 public:
  explicit FixedRelativeCovariance(const common::PoseCovariance& c) : c_(c) {}
  common::Result<common::PoseCovariance> relative_cov(KeyId /*i*/, KeyId /*j*/) const override {
    return common::Result<common::PoseCovariance>::ok(c_);
  }

 private:
  common::PoseCovariance c_;
};

// The review's periodic corridor (C1-12): identical pillar pairs every P m, the query truly at
// the candidate's place, the graph `drift_m` further along the corridor.
struct AliasCase {
  loop::LoopResult r;
  double err_m = 0.0;  // accepted loop vs truth (translation), m
};

AliasCase alias_case(const Config& cfg,
                     double period_m,
                     double drift_m,
                     const loop::RelativeCovarianceSource* rel) {
  const std::vector<Vec3> w = test::periodic_corridor_w(period_m);
  const Pose T_b_l = test::loop_T_b_l();
  constexpr std::size_t kMaxPoints = 60000;
  common::ScanFrame cc(kMaxPoints);
  common::ScanFrame cq(kMaxPoints);
  const Pose tc = test::body_pose(0.0, 0.0, 0.0);
  const Pose tq = test::body_pose(0.1, 0.1, 3.0 * kDeg);
  test::scan_at(w, tc * T_b_l, 15.0, cc);
  test::scan_at(w, tq * T_b_l, 15.0, cq);
  loop::LoopDetector det(cfg.loop_closure, cfg.scan_match, cfg.map, T_b_l, kMaxPoints);
  std::vector<loop::KeyframeView> k(2);
  k[0].id = 0;
  k[0].t = common::Timestamp{0};
  k[0].T_w_b = tc;
  k[0].cloud_l = &cc;
  k[1].id = 1;
  k[1].t = common::Timestamp{100 * kNsPerS};
  k[1].T_w_b = test::body_pose(0.1 + drift_m, 0.1, 3.0 * kDeg);
  k[1].cloud_l = &cq;
  AliasCase out;
  out.r = det.detect(k[1], common::Span<const loop::KeyframeView>(k), rel);
  if (out.r.outcome == loop::LoopOutcome::kAccepted) {
    out.err_m = (out.r.constraint.T_bi_bj.p - (tc.inverse() * tq).p).norm();
  }
  return out;
}

// C1-12 with the shipped configuration (ADR-028: graph gate off): the review's aliasing probe,
// both periods (2, 2.5 m), drifts 0 to 2.5 m. No false loop survives: the copy one period away
// fits at 0.93-0.94 of the true fit, so every case is ambiguous and nothing is added to the
// graph (remaining drag bound: 0 m). The price is recall: no loop at all in an exactly
// periodic corridor. With the gate off detect() ignores the graph's covariance, so the chain
// stiffness plays no part: a source (the review's stiff chain) gives the same outcome and the
// same ICP work as none (review H3 m-3).
TEST(LoopAliasing, NoFalseLoopAtAnyDrift) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (relative marginal)";
  }
  Config cfg = shipped_config();
  cfg.loop_closure.enabled = true;
  ASSERT_FALSE(cfg.loop_closure.graph_covariance_gate);
  const FixedRelativeCovariance stiff(lap_relative_cov(cfg, 0.005, 0.0375 * kDeg));
  int false_loops = 0;
  for (const double period : {2.0, 2.5}) {
    for (const double drift : {0.0, 0.3, 0.6, 0.9, 1.1, 1.3, 1.6, 2.0, 2.5}) {
      const AliasCase c = alias_case(cfg, period, drift, nullptr);
      EXPECT_EQ(c.r.outcome, loop::LoopOutcome::kAmbiguous) << "P " << period << " drift " << drift;
      false_loops += (c.r.outcome == loop::LoopOutcome::kAccepted && c.err_m > kFalseLoopM);
      const AliasCase with_source = alias_case(cfg, period, drift, &stiff);
      EXPECT_EQ(with_source.r.outcome, c.r.outcome) << "P " << period << " drift " << drift;
      EXPECT_EQ(with_source.r.n_icp_runs, c.r.n_icp_runs) << "P " << period << " drift " << drift;
    }
  }
  EXPECT_EQ(false_loops, 0);
}

// The optional graph gate (graph_covariance_gate, ADR-026 amendment / ADR-028) with a covariance
// consistent with metre-level drift (6 cm / 0.45 deg per 0.5 m step, relative sigma ~0.6 m): no
// false loop for any drift up to 2 m at either period; on the loose chain (relative sigma
// ~0.2 m) a true loop below P/2 is accepted (the graph rules the copy out) and the review's
// first false loop (P = 2, drift 1.1 m) is ambiguous.
TEST(LoopAliasing, OptionalGraphGateArbitratesWhileDriftIsWithinGraphCovariance) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM (relative marginal)";
  }
  Config cfg = shipped_config();
  cfg.loop_closure.enabled = true;
  cfg.loop_closure.graph_covariance_gate = true;
  const FixedRelativeCovariance consistent(lap_relative_cov(cfg, 0.06, 0.45 * kDeg));
  const FixedRelativeCovariance loose(lap_relative_cov(cfg, 0.02, 0.15 * kDeg));
  for (const double period : {2.0, 2.5}) {
    for (const double drift : {0.0, 0.6, 1.1, 1.3, 1.6, 2.0}) {
      const AliasCase c = alias_case(cfg, period, drift, &consistent);
      if (c.r.outcome == loop::LoopOutcome::kAccepted) {
        EXPECT_LT(c.err_m, kLoopTolM) << "false loop, P " << period << " drift " << drift;
      }
    }
    const AliasCase good = alias_case(cfg, period, 0.6, &loose);
    ASSERT_EQ(good.r.outcome, loop::LoopOutcome::kAccepted) << "P " << period;
    EXPECT_LT(good.err_m, kLoopTolM);
  }
  const AliasCase first_false = alias_case(cfg, 2.0, 1.1, &loose);
  EXPECT_EQ(first_false.r.outcome, loop::LoopOutcome::kAmbiguous);
}

// Known limit of the optional graph gate (review H3 I-1, why it is off): with the claimed
// odometry covariance halved, the gate rejects every true loop of the two-lap run and the drift
// stays (end-to-start as with loops off); without the gate, seed 42 closes it to 0.548 m with
// or without PCM (LoopComponent.TwoLapOverconfidentPcmNoWorseThanWithoutAcrossSeeds).
TEST(LoopKnownLimits, GraphGateDropsTrueLoopsOnAnOverconfidentGraph) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM";
  }
  const Config cfg = shipped_config();
  LapOptions o;
  o.claim = 0.5;
  o.graph_gate = true;
  const LapRun on = run_two_laps(cfg, o);
  const LapRun off = run_two_laps(cfg, false);
  std::printf(
      "[ loop two laps ] claim x0.5, graph gate on: %d accepted, %d correction-gated, "
      "end-to-start %.3f m (loops off %.3f m)\n",
      on.accepted,
      on.correction,
      on.end_to_start_m,
      off.end_to_start_m);
  EXPECT_EQ(on.accepted, 0);
  EXPECT_GT(on.correction, 30);
  EXPECT_NEAR(on.end_to_start_m, off.end_to_start_m, 1e-6);
}

// Known limit of the optional graph gate: when the drift is far outside the graph's covariance
// (loose chain, relative sigma ~0.2 m, drift 1.6 m: 8 sigma) the graph prefers the copy and the
// false loop (error ~P) is accepted. On the loose chain such loops drag the lap by 0.34 to
// 0.64 m (drifts 1.3 to 2.5 m, both periods; ADR-026 amendment), on a stiff one by centimetres.
TEST(LoopKnownLimits, GraphGateAliasingWhenDriftExceedsGraphCovariance) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM";
  }
  Config cfg = shipped_config();
  cfg.loop_closure.enabled = true;
  cfg.loop_closure.graph_covariance_gate = true;
  constexpr double kPeriodM = 2.0;
  constexpr double kDriftM = 1.6;
  const FixedRelativeCovariance loose(lap_relative_cov(cfg, 0.02, 0.15 * kDeg));
  const AliasCase bad = alias_case(cfg, kPeriodM, kDriftM, &loose);
  ASSERT_EQ(bad.r.outcome, loop::LoopOutcome::kAccepted);
  EXPECT_GT(bad.err_m, 0.8 * kPeriodM);
  EXPECT_LT(bad.err_m, 1.2 * kPeriodM);
  const Pose tc = test::body_pose(0.0, 0.0, 0.0);
  const Pose asked = (tc.inverse() * test::body_pose(0.1 + kDriftM, 0.1, 3.0 * kDeg)).inverse() *
                     bad.r.constraint.T_bi_bj;
  const double loose_drag = applied_correction(cfg, asked, bad.r.constraint.cov, 0.02, 0.15 * kDeg);
  const double stiff_drag =
      applied_correction(cfg, asked, bad.r.constraint.cov, 0.005, 0.0375 * kDeg);
  std::printf(
      "[ aliasing ] graph gate, P %.1f drift %.1f: error %.3f m, drag loose %.3f m, "
      "stiff %.3f m\n",
      kPeriodM,
      kDriftM,
      bad.err_m,
      loose_drag,
      stiff_drag);
  EXPECT_GT(loose_drag, 0.15);
  EXPECT_LT(stiff_drag, 0.1);
}

// I-2: the Cauchy kernel cannot tell a true large correction from a false one: with the
// covariance at the floor (5 cm / 1 deg) a correct loop asking 2 m moves the lap end by under
// 20 % of it on a loose chain, and a 0.5 m request by under 15 % on a stiff one. Loops only
// close while the drift is small.
TEST(LoopKnownLimits, RobustKernelAppliesLittleOfLargeCorrections) {
  if (!ct::kWithGtsam) {
    GTEST_SKIP() << "needs GTSAM";
  }
  Config cfg = shipped_config();
  common::PoseCovariance C = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    C(a, a) = cfg.loop_closure.cov_floor_m * cfg.loop_closure.cov_floor_m;
    C(a + 3, a + 3) = cfg.loop_closure.cov_floor_rad * cfg.loop_closure.cov_floor_rad;
  }
  const auto frac = [&](double c_m, double sig_m) {
    const Pose corr{common::Mat3::Identity(), Vec3(c_m, 0.0, 0.0)};
    return applied_correction(cfg, corr, C, sig_m, sig_m * 0.15 * kDeg / 0.02) / c_m;
  };
  const double small_loose = frac(0.2, 0.02);
  const double large_loose = frac(2.0, 0.02);
  const double mid_stiff = frac(0.5, 0.005);
  std::printf("[ kernel ] applied fraction: 0.2 m loose %.2f, 2 m loose %.2f, 0.5 m stiff %.2f\n",
              small_loose,
              large_loose,
              mid_stiff);
  EXPECT_GT(small_loose, 0.8);  // small corrections go through
  EXPECT_LT(large_loose, 0.2);
  EXPECT_LT(mid_stiff, 0.15);
}

}  // namespace
}  // namespace mss::pipeline
