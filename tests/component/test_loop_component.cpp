// Loop closure end to end (ADR-026), known answers, no randomness except fixed seeds:
//   - LoopComponent.TwoLapDriftClosedByLoops: backend::Graph + loop::LoopDetector on a two-lap
//     circle in the pillar hall with drifting odometry (relative factors carrying a yaw bias,
//     covariance consistent with it): with loops the end-to-start error drops, every accepted
//     loop matches the truth.
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
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/loop/loop_detector.hpp"
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
constexpr double kYawSigmaRad = 0.15 * kDeg;  // claimed odometry yaw sigma per step
constexpr double kPosSigmaM = 0.02;           // claimed odometry position sigma per step

Pose circle_pose(int k) {
  const double a = k * kStepM / kRadiusM;  // arc angle
  return test::body_pose(kRadiusM * std::sin(a), kRadiusM * (1.0 - std::cos(a)) - kRadiusM, a);
}

struct LapRun {
  double end_to_start_m = 0.0;
  int accepted = 0;
  int false_loops = 0;
  double max_err_m = 0.0;
  double max_err_rad = 0.0;
};

LapRun run_two_laps(const Config& cfg, bool loops_on) {
  LapRun out;
  const std::vector<Vec3> hall = test::hall_points_w(0.4);
  const Pose T_b_l = test::loop_T_b_l();
  constexpr std::size_t kMaxPoints = 20000;
  backend::Graph graph(cfg.backend, cfg.imu, Vec3::Zero());
  loop::LoopConfig lc = cfg.loop_closure;
  lc.enabled = loops_on;
  loop::LoopDetector det(lc, cfg.scan_match, cfg.map, T_b_l, kMaxPoints);
  std::vector<std::unique_ptr<common::ScanFrame>> clouds;
  std::vector<loop::KeyframeView> views;
  std::mt19937 rng(42);
  std::normal_distribution<double> n01(0.0, 1.0);
  common::PoseCovariance cov = common::PoseCovariance::Zero();
  for (int a = 0; a < 3; ++a) {
    cov(a, a) = kPosSigmaM * kPosSigmaM;
    cov(a + 3, a + 3) = kYawSigmaRad * kYawSigmaRad;
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
    const loop::LoopResult r =
        det.detect(views.back(), common::Span<const loop::KeyframeView>(views));
    if (r.outcome != loop::LoopOutcome::kAccepted) {
      continue;
    }
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
    graph.add_loop_factor(r.constraint);
    for (int u = 0; u <= lc.isam_extra_updates; ++u) {
      EXPECT_TRUE(graph.optimize().is_ok());
    }
    guess = graph.keyframe_state(static_cast<KeyId>(k)).value().T_w_b;
    skip = lc.min_interval_keyframes;
  }
  const Pose e0 = graph.keyframe_state(0).value().T_w_b;
  const Pose eN = graph.keyframe_state(static_cast<KeyId>(kKeyframes - 1)).value().T_w_b;
  const Vec3 d_est = e0.inverse() * eN.p;
  const Vec3 d_true = circle_pose(0).inverse() * circle_pose(kKeyframes - 1).p;
  out.end_to_start_m = (d_est - d_true).norm();
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
  std::printf("[ loop two laps ] end-to-start off %.3f m, on %.3f m; %d loops accepted, %d false\n",
              off.end_to_start_m,
              on.end_to_start_m,
              on.accepted,
              on.false_loops);
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

// --- Whole estimator on the box world -------------------------------------------------------

// Records the accepted loops with their keyframe times.
class LoopObserver : public detail::CoreObserver {
 public:
  void on_loop_search(common::Timestamp t, const loop::LoopResult& r) override {
    ++searches;
    if (r.outcome == loop::LoopOutcome::kAccepted) {
      accepted.push_back({r.candidate_t, t, r.constraint.T_bi_bj});
    }
  }
  struct Loop {
    common::Timestamp t_i;
    common::Timestamp t_j;
    Pose T_bi_bj;
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

}  // namespace
}  // namespace mss::pipeline
