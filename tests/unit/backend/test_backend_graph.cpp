// Graph: keyframes, add_* validation, optimize, marginals, failure recovery; B-02.
// Derivation: docs/derivations/backend_factors.md.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

#include "backend_test_config.hpp"
#include "graph_scenarios.hpp"

namespace mss::backend {
namespace {

using common::Estimate;
using common::KeyId;
using common::Mat3;
using common::Mat6;
using common::NavState;
using common::Pose;
using common::Vec3;
using common::Vec6;
using test::CapturingLogger;
using test::default_backend_config;
using test::default_imu_config;
constexpr double kDegToRad = EIGEN_PI / 180.0;

constexpr double kChi2Dof6P999 = 22.458;  // chi-square 0.999 quantile, 6 dof

double pose_error(const Pose& a, const Pose& b) {
  return common::se3_log(b.inverse() * a).norm();
}

NavState state_or_fail(const Graph& g, KeyId k) {
  const common::Result<NavState> r = g.keyframe_state(k);
  EXPECT_TRUE(r.is_ok()) << "keyframe " << k << ": " << r.reason();
  return r.is_ok() ? r.value() : NavState{};
}

// Runs the square loop: keyframe k with its LiDAR edge (k-1, k), the loop on keyframe 4,
// optimize() after each keyframe, then `extra` more calls.
void run_square(Graph& g, const test::Scenario& s, int extra) {
  for (std::size_t k = 0; k < 5; ++k) {
    g.add_keyframe(s.truth[k].t, s.guess[k]);
    if (k > 0) {
      g.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
    }
    if (k == 4) {
      common::LoopConstraint lc;
      lc.i = 0;
      lc.j = 4;
      lc.T_bi_bj = s.edge_meas[4];
      lc.cov = s.edge_cov;
      g.add_loop_factor(lc);
    }
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (int n = 0; n < extra; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
}

TEST(BackendGraph, B02_SquareLoopNoiseFreeConvergesExactly) {
  const test::Scenario s = test::square_loop(false, 0);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g, s, 5);
  for (KeyId k = 0; k < 5; ++k) {
    EXPECT_LT(pose_error(state_or_fail(g, k).T_w_b, s.truth[k].T_w_b), 1e-6) << "keyframe " << k;
  }
}

TEST(BackendGraph, B02_SquareLoopNoisyWithinMarginalBounds) {
  const test::Scenario s = test::square_loop(true, 42);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g, s, 5);
  for (KeyId k = 0; k < 5; ++k) {
    const Vec6 e = common::boxminus(state_or_fail(g, k).T_w_b, s.truth[k].T_w_b);
    const Mat6 cov = g.marginal_cov(k);
    ASSERT_TRUE(cov.allFinite());
    EXPECT_LT(e.dot(cov.ldlt().solve(e)), kChi2Dof6P999) << "keyframe " << k;
  }
  // The loop pulls the dead-reckoned trajectory towards the truth. (|p_4 - p_0| itself is
  // not a valid check: the loop measurement is noisy too, 0.02 m per axis.)
  double err_est = 0.0;
  double err_guess = 0.0;
  for (KeyId k = 0; k < 5; ++k) {
    err_est += (state_or_fail(g, k).T_w_b.p - s.truth[k].T_w_b.p).squaredNorm();
    err_guess += (s.guess[k].T_w_b.p - s.truth[k].T_w_b.p).squaredNorm();
  }
  EXPECT_LT(err_est, err_guess);
  RecordProperty("rms_pos_err_est_m", std::to_string(std::sqrt(err_est / 5.0)));
  RecordProperty("rms_pos_err_guess_m", std::to_string(std::sqrt(err_guess / 5.0)));
}

// Keyframe guesses: truth perturbed by 0.1 m / 0.1 m/s for k >= 1 (fixed offsets).
std::vector<NavState> perturbed_guesses(const std::vector<NavState>& truth) {
  std::vector<NavState> guess = truth;
  for (std::size_t k = 1; k < guess.size(); ++k) {
    guess[k].T_w_b.p += Vec3(0.1, -0.1, 0.05);
    guess[k].v_w_b += Vec3(-0.1, 0.1, 0.05);
  }
  return guess;
}

TEST(BackendGraph, ImuChainRecoversTrajectory) {
  const test::ImuChain c = test::imu_chain(7, 4, 40);
  const std::vector<NavState> guess = perturbed_guesses(c.truth);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  for (KeyId k = 0; k < 4; ++k) {
    g.add_keyframe(c.truth[k].t, guess[k]);
    if (k > 0) {
      g.add_imu_factor(k - 1, k, c.deltas[k - 1]);
    }
    ASSERT_TRUE(g.optimize().is_ok());
  }
  common::Result<Estimate> last = g.optimize();
  for (int n = 0; n < 5; ++n) {
    last = g.optimize();
    ASSERT_TRUE(last.is_ok());
  }
  for (KeyId k = 0; k < 4; ++k) {
    const NavState x = state_or_fail(g, k);
    EXPECT_LT(pose_error(x.T_w_b, c.truth[k].T_w_b), 1e-6) << "keyframe " << k;
    EXPECT_LT((x.v_w_b - c.truth[k].v_w_b).norm(), 1e-6) << "keyframe " << k;
  }
  EXPECT_EQ(last.value().keyframe_id, 3U);
  EXPECT_LT((last.value().state.v_w_b - c.truth[3].v_w_b).norm(), 1e-6);
}

TEST(BackendGraph, ImuTimeCheckRejectsMismatch) {
  const test::ImuChain c = test::imu_chain(7, 2, 40);
  constexpr std::int64_t kTolNs = 7'500'000;  // 1.5e9 / 200 Hz
  for (const std::int64_t shift_ns : {kTolNs + 1, kTolNs, -kTolNs - 1, -kTolNs}) {
    Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
    g.add_keyframe(c.truth[0].t, c.truth[0]);
    NavState x1 = c.truth[1];
    x1.t.ns += shift_ns;
    g.add_keyframe(x1.t, x1);
    CapturingLogger log;
    g.add_imu_factor(0, 1, c.deltas[0]);
    const bool beyond = shift_ns > kTolNs || shift_ns < -kTolNs;
    EXPECT_EQ(log.warnings.size(), beyond ? 1U : 0U) << "shift " << shift_ns;
    if (beyond) {
      EXPECT_NE(log.warnings[0].find("duration mismatch"), std::string::npos);
    }
  }
}

TEST(BackendGraph, ImuGapReanchorsVelocityAndBias) {
  const test::ImuChain c = test::imu_chain(7, 4, 40);
  const std::vector<NavState> guess = perturbed_guesses(c.truth);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  for (KeyId k = 0; k < 4; ++k) {
    g.add_keyframe(c.truth[k].t, guess[k]);
  }
  common::PreintegratedDelta bad = c.deltas[1];
  bad.dt_s += 0.01;  // 10 ms > 7.5 ms
  {
    CapturingLogger log;
    g.add_imu_factor(1, 2, bad);
    ASSERT_EQ(log.warnings.size(), 1U);
  }
  g.add_imu_factor(0, 1, c.deltas[0]);
  g.add_imu_factor(2, 3, c.deltas[2]);
  Mat6 cov = Mat6::Identity() * 1e-4;
  g.add_lidar_factor(1, 2, c.truth[1].T_w_b.inverse() * c.truth[2].T_w_b, cov, nullptr);
  const common::Result<Estimate> r = g.optimize();
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().keyframe_id, 3U);
  EXPECT_TRUE(r.value().cov.allFinite());
  for (int n = 0; n < 5; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
  // X(1) follows from the anchored IMU factor 0 -> 1, X(2) from the LiDAR edge.
  for (KeyId k = 1; k <= 2; ++k) {
    EXPECT_LT(pose_error(state_or_fail(g, k).T_w_b, c.truth[k].T_w_b), 1e-6) << "keyframe " << k;
  }
}

TEST(BackendGraph, DuplicateImuFactorRejected) {
  const test::ImuChain c = test::imu_chain(7, 2, 40);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  g.add_keyframe(c.truth[0].t, c.truth[0]);
  g.add_keyframe(c.truth[1].t, c.truth[1]);
  {
    CapturingLogger log;
    g.add_imu_factor(0, 1, c.deltas[0]);
    EXPECT_TRUE(log.warnings.empty());
    g.add_imu_factor(0, 1, c.deltas[0]);  // pending duplicate
    ASSERT_EQ(log.warnings.size(), 1U);
    EXPECT_NE(log.warnings[0].find("duplicate"), std::string::npos);
  }
  ASSERT_TRUE(g.optimize().is_ok());
  CapturingLogger log;
  g.add_imu_factor(0, 1, c.deltas[0]);  // committed duplicate
  EXPECT_EQ(log.warnings.size(), 1U);
}

// Review I-1: after a failed optimize() the factors linking keyframe 1 were dropped; the
// later IMU + LiDAR chain 1 -> 2 -> 3 -> 4 must still be solvable (pose re-anchor, §9).
TEST(BackendGraph, FailedOptimizeDoesNotStrandLaterKeyframes) {
  const test::ImuChain c = test::imu_chain(7, 5, 40);
  std::vector<NavState> guess = perturbed_guesses(c.truth);
  guess[1] = c.truth[1];
  const Mat6 lidar_cov = Mat6::Identity() * 1e-4;
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  g.add_keyframe(c.truth[0].t, c.truth[0]);
  ASSERT_TRUE(g.optimize().is_ok());
  g.add_keyframe(c.truth[1].t, guess[1]);
  common::GnssMeasurement m;
  m.p_e_g = c.truth[1].T_w_b.p;
  m.cov = Mat3::Identity() * 0.01;
  g.add_gnss_factor(1, m, Pose{});  // 3 of 6 DoF: indeterminant
  ASSERT_FALSE(g.optimize().is_ok());
  for (KeyId k = 2; k <= 4; ++k) {
    g.add_keyframe(c.truth[k].t, guess[k]);
    g.add_imu_factor(k - 1, k, c.deltas[k - 1]);
    g.add_lidar_factor(
        k - 1, k, c.truth[k - 1].T_w_b.inverse() * c.truth[k].T_w_b, lidar_cov, nullptr);
    const common::Result<Estimate> r = g.optimize();
    ASSERT_TRUE(r.is_ok()) << "keyframe " << k << ": " << r.reason();
    EXPECT_EQ(r.value().keyframe_id, k);
  }
  for (int n = 0; n < 5; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (KeyId k = 1; k <= 4; ++k) {
    const NavState x = state_or_fail(g, k);
    EXPECT_TRUE(x.T_w_b.p.allFinite() && x.T_w_b.R.allFinite() && x.v_w_b.allFinite());
    // The re-anchor prior holds X(1) at its guess (= truth); LiDAR edges are exact.
    EXPECT_LT(pose_error(x.T_w_b, c.truth[k].T_w_b), 1e-6) << "keyframe " << k;
  }
  EXPECT_TRUE(g.marginal_cov(4).allFinite());
}

// Review I-2: a non-finite guess never reaches GTSAM.
TEST(BackendGraph, AddKeyframeNonFiniteGuessIsSanitised) {
  const test::Scenario s = test::square_loop(false, 0);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  NavState bad = s.truth[1];
  bad.T_w_b.p.x() = std::nan("");
  bad.v_w_b.y() = std::numeric_limits<double>::infinity();
  {
    CapturingLogger log;
    EXPECT_EQ(g.add_keyframe(bad.t, bad), 1U);
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  g.add_lidar_factor(0, 1, s.edge_meas[0], s.edge_cov, nullptr);
  for (int n = 0; n < 5; ++n) {
    const common::Result<Estimate> r = g.optimize();
    ASSERT_TRUE(r.is_ok()) << r.reason();
  }
  EXPECT_LT(pose_error(state_or_fail(g, 1).T_w_b, s.truth[1].T_w_b), 1e-6);
}

TEST(BackendGraph, GnssFactorPullsPosition) {
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const Pose T_e_w = test::pose_from(Vec3(0.0, 0.0, 0.5), Vec3(10.0, -5.0, 1.0));
  const test::Scenario s = test::square_loop(false, 0);
  Graph g(default_backend_config(), default_imu_config(), p_b_g);
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  NavState guess1 = s.truth[1];
  guess1.T_w_b.p += Vec3(1.0, 0.0, 0.0);
  g.add_keyframe(s.truth[1].t, guess1);
  g.add_lidar_factor(0, 1, s.edge_meas[0], s.edge_cov, nullptr);
  common::GnssMeasurement m;
  m.t = s.truth[1].t;
  m.p_e_g = T_e_w * (s.truth[1].T_w_b * p_b_g);
  m.cov = Vec3(0.04, 0.04, 0.09).asDiagonal();
  g.add_gnss_factor(1, m, T_e_w);
  for (int n = 0; n < 3; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
  const Vec3 err = state_or_fail(g, 1).T_w_b.p - s.truth[1].T_w_b.p;
  for (int a = 0; a < 3; ++a) {
    EXPECT_LT(std::fabs(err(a)), 3.0 * std::sqrt(m.cov(a, a)));
  }
}

// ADR-019: noise-free fixes from the true T_e_w, factors given a T_e_w that is 3 deg and
// 1.4 m off. A fixed T_e_w pulls the keyframes; the frame variable absorbs the error and the
// keyframes stay at the truth (known answer).
TEST(BackendGraph, GnssFrameVariableAbsorbsAlignmentError) {
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const Pose T_e_w = test::pose_from(Vec3(0.0, 0.0, 0.5), Vec3(10.0, -5.0, 1.0));
  const Pose T_e_w_off =
      test::pose_from(Vec3(0.0, 0.0, 0.5 + 3.0 * kDegToRad), Vec3(11.0, -6.0, 1.0));
  const test::Scenario s = test::square_loop(false, 0);
  const auto run = [&](bool frame_variable) {
    BackendConfig cfg = default_backend_config();
    cfg.gnss_frame_variable = frame_variable;
    cfg.gnss_frame_tilt_sigma_rad = 0.01 * kDegToRad;
    cfg.gnss_frame_yaw_sigma_rad = 30.0 * kDegToRad;
    cfg.gnss_frame_pos_sigma_m = 10.0;
    Graph g(cfg, default_imu_config(), p_b_g);
    double max_err_m = 0.0;
    for (KeyId k = 0; k < 4; ++k) {
      g.add_keyframe(s.truth[k].t, s.truth[k]);
      if (k > 0) {
        g.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
        common::GnssMeasurement m;
        m.t = s.truth[k].t;
        m.p_e_g = T_e_w * (s.truth[k].T_w_b * p_b_g);
        m.cov = Vec3(0.04, 0.04, 0.09).asDiagonal();
        g.add_gnss_factor(k, m, T_e_w_off);
      }
      for (int n = 0; n < 3; ++n) {
        EXPECT_TRUE(g.optimize().is_ok());
      }
    }
    for (KeyId k = 0; k < 4; ++k) {
      max_err_m = std::max(max_err_m, (state_or_fail(g, k).T_w_b.p - s.truth[k].T_w_b.p).norm());
    }
    return max_err_m;
  };
  EXPECT_LT(run(true), 5e-3);
  EXPECT_GT(run(false), 0.05);
}

// Review I-4: after a failed optimize(), keyframes linked to the old graph only through
// GNSS factors on the frame variable E are not anchored by E: they get the section-9
// re-anchor prior (two antenna positions leave the rotation about their line free).
TEST(BackendGraph, GnssFrameVariableDoesNotAnchorOrphans) {
  const Pose T_e_w = test::pose_from(Vec3(0.0, 0.0, 0.5), Vec3(10.0, -5.0, 1.0));
  const test::Scenario s = test::square_loop(false, 0);
  BackendConfig cfg = default_backend_config();
  cfg.gnss_frame_variable = true;
  cfg.gnss_frame_tilt_sigma_rad = 0.01 * kDegToRad;
  cfg.gnss_frame_yaw_sigma_rad = 30.0 * kDegToRad;
  cfg.gnss_frame_pos_sigma_m = 10.0;
  Graph g(cfg, default_imu_config(), Vec3::Zero());
  const auto fix = [&](KeyId k) {
    common::GnssMeasurement m;
    m.t = s.truth[k].t;
    m.p_e_g = T_e_w * s.truth[k].T_w_b.p;
    m.cov = Mat3::Identity() * 0.01;
    g.add_gnss_factor(k, m, T_e_w);
  };
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  g.add_keyframe(s.truth[1].t, s.truth[1]);
  g.add_lidar_factor(0, 1, s.edge_meas[0], s.edge_cov, nullptr);
  fix(1);
  ASSERT_TRUE(g.optimize().is_ok());
  g.add_keyframe(s.truth[2].t, s.truth[2]);
  fix(2);  // position only: the rotation of X(2) is free
  ASSERT_FALSE(g.optimize().is_ok());
  g.add_keyframe(s.truth[3].t, s.truth[3]);
  g.add_lidar_factor(2, 3, s.edge_meas[2], s.edge_cov, nullptr);
  fix(2);
  fix(3);
  const common::Result<Estimate> r = g.optimize();
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (KeyId k = 0; k <= 3; ++k) {
    EXPECT_LT(pose_error(state_or_fail(g, k).T_w_b, s.truth[k].T_w_b), 1e-6) << "keyframe " << k;
  }
}

TEST(BackendGraph, LatestKeyframeWithoutFactorsReturnsGuessAndNanCov) {
  const test::Scenario s = test::square_loop(false, 0);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  ASSERT_TRUE(g.optimize().is_ok());
  g.add_keyframe(s.guess[1].t, s.guess[1]);
  const common::Result<Estimate> r = g.optimize();
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().keyframe_id, 1U);
  EXPECT_EQ(r.value().state.T_w_b.p, s.guess[1].T_w_b.p);
  EXPECT_EQ(r.value().state.T_w_b.R, s.guess[1].T_w_b.R);
  EXPECT_TRUE(r.value().cov.array().isNaN().all());
  EXPECT_TRUE(g.marginal_cov(1).array().isNaN().all());
  EXPECT_FALSE(g.keyframe_state(1).is_ok());
  EXPECT_TRUE(g.keyframe_state(0).is_ok());
}

TEST(BackendGraph, RejectsBadCovariances) {
  const test::Scenario s = test::square_loop(false, 0);
  const test::ImuChain c = test::imu_chain(7, 2, 40);
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  g.add_keyframe(c.truth[0].t, c.truth[0]);
  g.add_keyframe(c.truth[1].t, c.truth[1]);
  const Pose rel = c.truth[0].T_w_b.inverse() * c.truth[1].T_w_b;
  {
    CapturingLogger log;
    g.add_lidar_factor(0, 1, rel, Mat6::Zero(), nullptr);
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  {
    CapturingLogger log;
    common::GnssMeasurement m;
    m.cov(1, 1) = std::nan("");
    g.add_gnss_factor(1, m, Pose{});
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  {
    CapturingLogger log;
    common::LoopConstraint lc;
    lc.i = 0;
    lc.j = 1;
    lc.T_bi_bj = rel;
    lc.cov = Mat6::Identity();
    lc.cov(5, 5) = -1.0;
    g.add_loop_factor(lc);
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  {
    CapturingLogger log;
    common::PreintegratedDelta d = c.deltas[0];
    d.cov.setZero();
    g.add_imu_factor(0, 1, d);
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  {
    CapturingLogger log;
    common::OdomDelta od = test::wheel_delta(Pose{});
    od.cov.setZero();
    g.add_odom_factor(0, 1, od);
    EXPECT_EQ(log.warnings.size(), 1U);
  }
  {
    CapturingLogger log;
    g.add_static_gravity_factor(1, Vec3(0.0, 0.0, 9.8), Mat3::Zero());
    g.add_static_gravity_factor(7, Vec3(0.0, 0.0, 9.8), Mat3::Identity());  // unknown keyframe
    g.add_static_gravity_factor(1, Vec3(std::nan(""), 0.0, 9.8), Mat3::Identity());
    EXPECT_EQ(log.warnings.size(), 3U);
  }
  {
    CapturingLogger log;
    g.add_lidar_factor(0, 1, rel, Mat6::Identity() * 1e-4, nullptr);
    EXPECT_TRUE(log.warnings.empty());
    const common::Result<Estimate> r = g.optimize();
    ASSERT_TRUE(r.is_ok()) << r.reason();
    // Only the valid LiDAR factor (and the priors) are in the graph.
    EXPECT_LT(pose_error(r.value().state.T_w_b, c.truth[1].T_w_b), 1e-6);
  }
}

TEST(BackendGraph, OptimizeFailureDropsPendingFactorsKeepsKeyframes) {
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const test::Scenario s = test::square_loop(false, 0);
  Graph g(default_backend_config(), default_imu_config(), p_b_g);
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  g.add_keyframe(s.truth[1].t, s.truth[1]);
  common::GnssMeasurement m;
  m.p_e_g = s.truth[1].T_w_b * p_b_g + Vec3(1.0, 0.0, 0.0);  // T_e_w = identity, 1 m off
  m.cov = Mat3::Identity() * 0.01;
  g.add_gnss_factor(1, m, Pose{});  // 3 of 6 DoF of keyframe 1: indeterminant
  const common::Result<Estimate> failed = g.optimize();
  ASSERT_FALSE(failed.is_ok()) << "GTSAM solved the rank-deficient system without throwing";
  EXPECT_EQ(failed.reason().rfind("optimization failed: ", 0), 0U) << failed.reason();
  RecordProperty("failure_reason", failed.reason());

  g.add_lidar_factor(0, 1, s.edge_meas[0], s.edge_cov, nullptr);
  const common::Result<Estimate> r = g.optimize();
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().keyframe_id, 1U);
  // The GNSS factor was dropped: keyframe 1 follows the LiDAR edge exactly.
  EXPECT_LT(pose_error(r.value().state.T_w_b, s.truth[1].T_w_b), 1e-6);
  EXPECT_TRUE(r.value().cov.allFinite());
}

TEST(BackendGraph, MarginalCovIsMssOrder) {
  const BackendConfig cfg = default_backend_config();
  Graph g(cfg, default_imu_config(), Vec3::Zero());
  NavState x0;
  x0.T_w_b = test::pose_from(Vec3(0.2, -0.1, 0.4), Vec3(1.0, 2.0, 3.0));
  g.add_keyframe(x0.t, x0);
  ASSERT_TRUE(g.optimize().is_ok());
  Mat6 expected = Mat6::Zero();
  expected.diagonal() << Vec3::Constant(cfg.prior_pos_sigma_m * cfg.prior_pos_sigma_m),
      Vec3::Constant(cfg.prior_rot_sigma_rad * cfg.prior_rot_sigma_rad);
  const Mat6 cov = g.marginal_cov(0);
  EXPECT_LT((cov - expected).norm(), 1e-9 * expected.norm());
}

}  // namespace
}  // namespace mss::backend
