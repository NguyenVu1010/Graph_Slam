// ADR-018: BackendConfig::single_thread limits TBB to one thread for the rest of the
// process, so repeated runs of the same graph are bit-for-bit identical. The multi-thread
// runs are reported, not asserted (the ADR cites the printed numbers).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <gtsam/config.h>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

#include "backend_test_config.hpp"
#include "graph_scenarios.hpp"

namespace mss::backend {
namespace {

using common::KeyId;
using common::Mat6;
using common::NavState;
using common::Pose;
using common::Vec3;
using common::Vec6;
using test::default_backend_config;
using test::default_imu_config;

// 300 keyframes, 20 IMU samples (about 0.1 s at 200 Hz) apart; a GNSS fix and a loop
// (k - 40, k) every 50 keyframes, so iSAM2 re-eliminates large parts of the tree.
constexpr int kKeyframes = 300;
constexpr int kImuSteps = 20;
constexpr KeyId kEvery = 50;
constexpr KeyId kLoopSpan = 40;
constexpr int kRuns = 5;
constexpr std::uint32_t kImuSeed = 21;
constexpr std::uint32_t kNoiseSeed = 22;
constexpr std::size_t kDoublesPerKeyframe = 21;  // R (9), p, v, ba, bg

struct Inputs {
  test::ImuChain chain;
  std::vector<NavState> guess;
  std::vector<Pose> lidar;  // lidar[k]: k -> k + 1
  Mat6 lidar_cov = Mat6::Identity();
  Pose T_e_w;
  Vec3 p_b_g = Vec3(0.3, 0.0, 0.8);
  std::vector<std::pair<KeyId, common::GnssMeasurement>> fixes;
  std::vector<common::LoopConstraint> loops;  // loops[i] is added with fixes[i]
};

Inputs make_inputs() {
  Inputs in;
  in.chain = test::imu_chain(kImuSeed, kKeyframes, kImuSteps);
  const std::vector<NavState>& truth = in.chain.truth;
  std::mt19937 rng(kNoiseSeed);
  in.lidar_cov.setZero();
  in.lidar_cov.diagonal() << 4e-4, 4e-4, 4e-4, 4e-6, 4e-6, 4e-6;  // [rho, phi]
  const Vec6 sigmas = in.lidar_cov.diagonal().cwiseSqrt();
  std::normal_distribution<double> n(0.0, 1.0);
  const auto noise = [&]() {
    Vec6 v;
    for (int d = 0; d < 6; ++d) {
      v(d) = sigmas(d) * n(rng);
    }
    return common::se3_exp(v);
  };
  for (std::size_t k = 0; k + 1 < truth.size(); ++k) {
    in.lidar.push_back(truth[k].T_w_b.inverse() * truth[k + 1].T_w_b * noise());
  }
  in.T_e_w = test::pose_from(Vec3(0.0, 0.0, 0.5), Vec3(10.0, -5.0, 1.0));
  for (KeyId k = kEvery; k < truth.size(); k += kEvery) {
    common::GnssMeasurement m;
    m.t = truth[k].t;
    m.p_e_g = in.T_e_w * (truth[k].T_w_b * in.p_b_g) + test::random_vec3(rng, 0.05);
    m.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
    in.fixes.emplace_back(k, m);
    common::LoopConstraint lc;
    lc.i = k - kLoopSpan;
    lc.j = k;
    lc.T_bi_bj = truth[lc.i].T_w_b.inverse() * truth[k].T_w_b * noise();
    lc.cov = in.lidar_cov;
    in.loops.push_back(lc);
  }
  in.guess = truth;
  for (std::size_t k = 1; k < truth.size(); ++k) {
    in.guess[k].T_w_b = in.guess[k].T_w_b * noise();
    in.guess[k].v_w_b += test::random_vec3(rng, 0.05);
  }
  return in;
}

struct RunResult {
  std::vector<double> states;  // kDoublesPerKeyframe per keyframe, after the last optimize
  double optimize_s = 0.0;     // s, total wall time inside optimize()
};

void append(std::vector<double>& out, const double* p, std::size_t n) {
  out.insert(out.end(), p, p + n);
}

RunResult run_graph(const Inputs& in, bool single_thread) {
  BackendConfig cfg = default_backend_config();
  cfg.single_thread = single_thread;
  Graph g(cfg, default_imu_config(), in.p_b_g);
  RunResult r;
  std::size_t next_fix = 0;
  for (KeyId k = 0; k < static_cast<KeyId>(kKeyframes); ++k) {
    g.add_keyframe(in.chain.truth[k].t, in.guess[k]);
    if (k > 0) {
      g.add_imu_factor(k - 1, k, in.chain.deltas[k - 1]);
      g.add_lidar_factor(k - 1, k, in.lidar[k - 1], in.lidar_cov, nullptr);
    }
    if (next_fix < in.fixes.size() && in.fixes[next_fix].first == k) {
      g.add_gnss_factor(k, in.fixes[next_fix].second, in.T_e_w);
      g.add_loop_factor(in.loops[next_fix]);
      ++next_fix;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const common::Result<common::Estimate> e = g.optimize();
    r.optimize_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_TRUE(e.is_ok()) << "keyframe " << k << ": " << e.reason();
  }
  r.states.reserve(kDoublesPerKeyframe * kKeyframes);
  for (KeyId k = 0; k < static_cast<KeyId>(kKeyframes); ++k) {
    const common::Result<NavState> x = g.keyframe_state(k);
    EXPECT_TRUE(x.is_ok()) << "keyframe " << k << ": " << x.reason();
    const NavState s = x.is_ok() ? x.value() : NavState{};
    append(r.states, s.T_w_b.R.data(), 9);
    append(r.states, s.T_w_b.p.data(), 3);
    append(r.states, s.v_w_b.data(), 3);
    append(r.states, s.bias.ba.data(), 3);
    append(r.states, s.bias.bg.data(), 3);
  }
  return r;
}

bool bytes_equal(const std::vector<double>& a, const std::vector<double>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double worst = 0.0;
  for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

// True when TBB in this process is already capped at one thread (an earlier single-thread
// Graph; the limit is process-lifetime, ADR-018). Also true on a 1-CPU host, where the
// multi-thread report is skipped: it is informational only, nothing asserts on it.
bool tbb_limited() {
#ifdef GTSAM_USE_TBB
  using tbb::global_control;
  return global_control::active_value(global_control::max_allowed_parallelism) == 1U;
#else
  return false;
#endif
}

struct Summary {
  int differing = 0;  // runs whose states differ bytewise from the reference
  double max_diff = 0.0;
  double optimize_s = 0.0;  // s, mean optimize() wall time per run
};

Summary summarise(const std::vector<RunResult>& runs, const std::vector<double>& reference) {
  Summary s;
  for (const RunResult& r : runs) {
    if (!bytes_equal(r.states, reference)) {
      ++s.differing;
    }
    s.max_diff = std::max(s.max_diff, max_abs_diff(r.states, reference));
    s.optimize_s += r.optimize_s / static_cast<double>(runs.size());
  }
  return s;
}

// Each test is its own process under ctest (gtest_discover_tests). Timing for ADR-018 is
// taken with --gtest_filter per test, so the single-thread runs never share a process with
// TBB workers. Order matters when the whole binary runs in one process: the multi-thread
// test comes first because the single-thread limit is never released.
TEST(BackendDeterminism, MultiThreadRunsReported) {
  if (tbb_limited()) {
    GTEST_SKIP() << "TBB already limited in this process; run with --gtest_filter";
  }
  const Inputs in = make_inputs();
  std::vector<RunResult> multi;
  multi.reserve(kRuns);
  for (int i = 0; i < kRuns; ++i) {
    multi.push_back(run_graph(in, false));
  }
  const Summary vs_multi = summarise(multi, multi.front().states);
  // Reference single-thread run last: from here on this process is limited. Its timing
  // is not reported (the workers spawned above may spin).
  const RunResult single = run_graph(in, true);
  const Summary vs_single = summarise(multi, single.states);
  std::printf("[determinism] multi-thread, %d keyframes, %d runs\n", kKeyframes, kRuns);
  std::printf("[determinism] multi runs differing from multi run 0: %d of %d, max abs diff %.3e\n",
              vs_multi.differing,
              kRuns,
              vs_multi.max_diff);
  std::printf(
      "[determinism] multi runs differing from a single-thread run: %d of %d, max abs diff "
      "%.3e\n",
      vs_single.differing,
      kRuns,
      vs_single.max_diff);
  std::printf("[determinism] multi-thread optimize() wall time per run: %.3f s\n",
              vs_multi.optimize_s);
  RecordProperty("multi_vs_multi0_differing", vs_multi.differing);
  RecordProperty("multi_vs_single_differing", vs_single.differing);
  RecordProperty("multi_optimize_s_per_run", std::to_string(vs_multi.optimize_s));
}

TEST(BackendDeterminism, SingleThreadRunsAreBitIdentical) {
  const Inputs in = make_inputs();
  std::vector<RunResult> single;
  single.reserve(kRuns);
  for (int i = 0; i < kRuns; ++i) {
    single.push_back(run_graph(in, true));
  }
  ASSERT_EQ(single.front().states.size(), kDoublesPerKeyframe * kKeyframes);
  for (std::size_t i = 0; i < single.size(); ++i) {
    EXPECT_TRUE(bytes_equal(single[i].states, single.front().states)) << "single-thread run " << i;
  }
  const Summary s = summarise(single, single.front().states);
  std::printf("[determinism] single-thread optimize() wall time per run: %.3f s\n", s.optimize_s);
  RecordProperty("single_optimize_s_per_run", std::to_string(s.optimize_s));
}

// The guarantee itself: the limit is active once a single-thread Graph exists and stays
// active after it is destroyed; a later multi-thread Graph stays limited and warns.
TEST(BackendDeterminism, SingleThreadLimitIsProcessLifetime) {
#ifdef GTSAM_USE_TBB
  using tbb::global_control;
  {
    BackendConfig cfg = default_backend_config();
    cfg.single_thread = true;
    Graph single(cfg, default_imu_config(), Vec3::Zero());
    EXPECT_EQ(global_control::active_value(global_control::max_allowed_parallelism), 1U);
    const Graph moved(std::move(single));
    EXPECT_EQ(global_control::active_value(global_control::max_allowed_parallelism), 1U);
  }
  EXPECT_EQ(global_control::active_value(global_control::max_allowed_parallelism), 1U);
  test::CapturingLogger log;
  const Graph multi(default_backend_config(), default_imu_config(), Vec3::Zero());
  EXPECT_EQ(global_control::active_value(global_control::max_allowed_parallelism), 1U);
  const Graph multi2(default_backend_config(), default_imu_config(), Vec3::Zero());
  const auto n_warn = std::count_if(log.warnings.begin(), log.warnings.end(), [](const auto& m) {
    return m.find("TBB already limited to 1 thread") != std::string::npos;
  });
  EXPECT_EQ(n_warn, 1);  // logged once per process
#else
  GTEST_SKIP() << "GTSAM built without TBB: single_thread is a no-op";
#endif
}

}  // namespace
}  // namespace mss::backend
