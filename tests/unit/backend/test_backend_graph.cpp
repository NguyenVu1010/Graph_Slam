// Graph: keyframes, add_* validation, optimize, marginals, failure recovery; B-02.
// Derivation: docs/derivations/backend_factors.md.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
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
    const common::Result<Pose> E = g.gnss_frame_estimate();
    EXPECT_EQ(E.is_ok(), frame_variable);
    if (E.is_ok()) {
      EXPECT_LT((E.value().p - T_e_w.p).norm(), 0.05);  // m, the offset guess was 1.4 m off
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
    g.add_static_gravity_factor(1, Vec3::Zero(), Mat3::Identity());  // zero specific force
    EXPECT_EQ(log.warnings.size(), 4U);
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
  // ADR-020: the keyframe-0 gravity factor rides with the priors and survives the failure.
  const Vec3 g_w(0.0, 0.0, -default_imu_config().gravity_mps2);
  g.add_static_gravity_factor(0, -(s.truth[0].T_w_b.R.transpose() * g_w), Mat3::Identity() * 1e-5);
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
  const common::Result<double> chi2 = g.static_gravity_chi2();
  ASSERT_TRUE(chi2.is_ok()) << chi2.reason();
  EXPECT_LT(chi2.value(), 1e-6);  // consistent with the truth
}

// ADR-020 known answer: keyframe 0 alone, guessed 10 mrad off in roll, true bias 0, the
// gravity factor measured at the true attitude. The factor fixes g*theta - ba (theta the
// roll error), the priors centred on the guess (rotation sigma_r) and on ba = 0 (sigma_b)
// share the rest: theta = theta_0 / (1 + g^2 sigma_r^2 / sigma_b^2) to first order, and the
// roll marginal is 1 / sqrt(1 / sigma_r^2 + g^2 / (sigma_b^2 + s^2 sigma_f^2)).
TEST(BackendGraph, StaticGravityFactorKnownAnswer) {
  const BackendConfig cfg = default_backend_config();
  const imu::ImuConfig imu_cfg = default_imu_config();
  Graph g(cfg, imu_cfg, Vec3::Zero());
  const double g_mps2 = imu_cfg.gravity_mps2;
  const double theta0_rad = 0.01;
  NavState guess;
  guess.T_w_b.R = common::so3_exp(Vec3(theta0_rad, 0.0, 0.0));  // truth: level, ba = 0
  g.add_keyframe(guess.t, guess);
  const double var_f = 1.7e-5;  // (m/s^2)^2 per axis, the window's covariance of the mean
  g.add_static_gravity_factor(0, Vec3(0.0, 0.0, g_mps2), Mat3::Identity() * var_f);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
  const common::Result<NavState> x = g.keyframe_state(0);
  ASSERT_TRUE(x.is_ok());
  const double theta_rad = common::so3_log(x.value().T_w_b.R).x();
  const double sr2 = cfg.prior_rot_sigma_rad * cfg.prior_rot_sigma_rad;
  const double sb2 = cfg.prior_bias_acc_sigma_mps2 * cfg.prior_bias_acc_sigma_mps2 +
                     cfg.static_gravity_sigma_scale * cfg.static_gravity_sigma_scale * var_f;
  const double expected_rad = theta0_rad / (1.0 + g_mps2 * g_mps2 * sr2 / sb2);
  EXPECT_NEAR(theta_rad, expected_rad, 2e-4);  // rad, first order in theta0
  const double sigma_rad = 1.0 / std::sqrt(1.0 / sr2 + g_mps2 * g_mps2 / sb2);
  EXPECT_NEAR(std::sqrt(g.marginal_cov(0)(3, 3)), sigma_rad, 0.05 * sigma_rad);
  // The window and the bias agree: the factor's residual is small.
  const common::Result<double> chi2 = g.static_gravity_chi2();
  ASSERT_TRUE(chi2.is_ok());
  EXPECT_LT(chi2.value(), 1.0);
}

// ADR-020 innovation statistic, known answer: keyframe 0 alone, guessed theta0 off in roll,
// the window level (f_b = [0, 0, g]). To first order the residual is g*theta - ba_y (one
// axis); the rest of the graph is the rotation prior (sigma_r, centred on theta0) and the
// bias prior (sigma_b, centred on 0). In units of the factor sigma s*sigma_f:
//   innovation nu = g theta0 / (s sigma_f), rest variance q = (g^2 sigma_r^2 + sigma_b^2) /
//   (s sigma_f)^2, innovation statistic nu^2 / (1 + q), posterior statistic nu^2 / (1 + q)^2.
// The numbers put the posterior statistic below the 5.991 gate and the innovation above it:
// the window mean is inconsistent with the rest of the graph, and only the innovation form
// sees it.
TEST(BackendGraph, StaticGravityInnovationKnownAnswer) {
  BackendConfig cfg = default_backend_config();
  cfg.prior_rot_sigma_rad = 0.01;       // rad
  cfg.prior_bias_acc_sigma_mps2 = 0.1;  // m/s^2
  cfg.static_gravity_sigma_scale = 1.0;
  const imu::ImuConfig imu_cfg = default_imu_config();
  const double g_mps2 = imu_cfg.gravity_mps2;
  const double var_f = 0.0025;  // (m/s^2)^2, (s sigma_f)^2
  const double q = (g_mps2 * g_mps2 * cfg.prior_rot_sigma_rad * cfg.prior_rot_sigma_rad +
                    cfg.prior_bias_acc_sigma_mps2 * cfg.prior_bias_acc_sigma_mps2) /
                   var_f;
  const double innovation_target = 20.0;
  const double nu = std::sqrt(innovation_target * (1.0 + q));
  const double theta0_rad = std::asin(nu * std::sqrt(var_f) / g_mps2);
  Graph g(cfg, imu_cfg, Vec3::Zero());
  NavState guess;
  guess.T_w_b.R = common::so3_exp(Vec3(theta0_rad, 0.0, 0.0));
  g.add_keyframe(guess.t, guess);
  g.add_static_gravity_factor(0, Vec3(0.0, 0.0, g_mps2), Mat3::Identity() * var_f);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
  const double posterior = nu * nu / ((1.0 + q) * (1.0 + q));
  ASSERT_LT(posterior, 5.991);  // the round-1 posterior statistic would keep the factor
  const common::Result<double> chi2 = g.static_gravity_chi2();
  ASSERT_TRUE(chi2.is_ok()) << chi2.reason();
  RecordProperty("innovation", std::to_string(chi2.value()));
  EXPECT_NEAR(chi2.value(), innovation_target, 0.02 * innovation_target);  // first order
  EXPECT_GT(chi2.value(), 5.991);
}

// The factor as practically the only information on what it measures (its variance 1e-16
// (m/s^2)^2 against priors of about 1e-2): S = I - J P J^T is about 1e-14, below the
// backend bound, and the check is reported as not evaluated.
TEST(BackendGraph, StaticGravityInnovationUncheckedWhenFactorIsOnlyInformation) {
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  NavState x0;
  g.add_keyframe(x0.t, x0);
  g.add_static_gravity_factor(0, Vec3(0.0, 0.0, 9.80665), Mat3::Identity() * 1e-16);
  ASSERT_TRUE(g.optimize().is_ok());
  const common::Result<double> chi2 = g.static_gravity_chi2();
  ASSERT_FALSE(chi2.is_ok());
  EXPECT_EQ(chi2.reason().rfind("innovation covariance not positive definite", 0), 0U)
      << chi2.reason();
}

// ADR-020 consistency check: a window that was not at rest (a 0.2 m/s^2 horizontal
// acceleration read as tilt) against tight priors gives a large statistic; the factor is
// removed from iSAM2 and from the rebuild set, and the graph still optimises. Then:
// - a failed optimize() after the removal rebuilds iSAM2 from the committed factors, which
//   no longer hold the factor (the estimate stays on the priors);
// - a factor removed while still pending (before the first optimize()) never reaches iSAM2.
TEST(BackendGraph, StaticGravityFactorRemovedOnRequest) {
  BackendConfig cfg = default_backend_config();
  cfg.prior_bias_acc_sigma_mps2 = 1e-4;
  cfg.prior_rot_sigma_rad = 1e-4;
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const test::Scenario s = test::square_loop(false, 0);
  const Vec3 up_w(0.0, 0.0, 9.80665);  // -g_w, m/s^2
  const Vec3 accel_w(0.2, 0.0, 0.0);   // m/s^2, read as tilt
  {
    Graph g(cfg, default_imu_config(), p_b_g);
    g.add_keyframe(s.truth[0].t, s.truth[0]);
    const Vec3 f_b = s.truth[0].T_w_b.R.transpose() * (up_w + accel_w);
    g.add_static_gravity_factor(0, f_b, Mat3::Identity() * 1.7e-5);
    ASSERT_TRUE(g.optimize().is_ok());
    const common::Result<double> chi2 = g.static_gravity_chi2();
    ASSERT_TRUE(chi2.is_ok()) << chi2.reason();
    EXPECT_GT(chi2.value(), 5.991);
    EXPECT_TRUE(g.remove_static_gravity_factor());
    EXPECT_FALSE(g.static_gravity_chi2().is_ok());
    EXPECT_FALSE(g.remove_static_gravity_factor());
    const common::Result<Estimate> r = g.optimize();
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(pose_error(r.value().state.T_w_b, s.truth[0].T_w_b), 1e-6);  // on the priors

    // Forced failure after the removal (as OptimizeFailureDropsPendingFactorsKeepsKeyframes):
    // the rebuild must not bring the factor back.
    g.add_keyframe(s.truth[1].t, s.truth[1]);
    common::GnssMeasurement m;
    m.p_e_g = s.truth[1].T_w_b * p_b_g;
    m.cov = Mat3::Identity() * 0.01;
    g.add_gnss_factor(1, m, Pose{});  // 3 of 6 DoF of keyframe 1: indeterminant
    ASSERT_FALSE(g.optimize().is_ok());
    g.add_lidar_factor(0, 1, s.edge_meas[0], s.edge_cov, nullptr);
    const common::Result<Estimate> after = g.optimize();
    ASSERT_TRUE(after.is_ok()) << after.reason();
    EXPECT_FALSE(g.static_gravity_chi2().is_ok());  // still no gravity factor
    const common::Result<NavState> x0 = g.keyframe_state(0);
    ASSERT_TRUE(x0.is_ok());
    EXPECT_LT(pose_error(x0.value().T_w_b, s.truth[0].T_w_b), 1e-6);
  }
  {
    Graph g(cfg, default_imu_config(), p_b_g);
    g.add_keyframe(s.truth[0].t, s.truth[0]);
    const Vec3 f_b = s.truth[0].T_w_b.R.transpose() * (up_w + accel_w);
    g.add_static_gravity_factor(0, f_b, Mat3::Identity() * 1.7e-5);
    EXPECT_FALSE(g.static_gravity_chi2().is_ok());  // not optimised yet
    EXPECT_TRUE(g.remove_static_gravity_factor());  // pending only
    EXPECT_FALSE(g.remove_static_gravity_factor());
    const common::Result<Estimate> r = g.optimize();
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(pose_error(r.value().state.T_w_b, s.truth[0].T_w_b), 1e-6);  // priors only
    EXPECT_FALSE(g.static_gravity_chi2().is_ok());
  }
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

// --- relative_pose_cov (ADR-026 amendment, loop_closure.md section 9) -----------------------

// A random SPD 6x6 covariance in [rho, phi] with m^2 / rad^2 scales (fixed seed).
Mat6 random_cov(std::mt19937& rng, double sig_m, double sig_rad) {
  Mat6 A;
  std::normal_distribution<double> n(0.0, 1.0);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      A(r, c) = n(rng);
    }
  }
  Vec6 s;
  s << sig_m, sig_m, sig_m, sig_rad, sig_rad, sig_rad;
  const Mat6 B = s.asDiagonal() * A;
  const Mat6 C = 0.25 * B * B.transpose() + Mat6(s.cwiseAbs2().asDiagonal());
  return C;
}

// Known answer: a noise-free chain 0 -> 1 -> 2 of exact SE(3) relative factors with full
// covariances C1, C2 (linearised at the truth, so first order is exact). The relative pose
// T_02 = Z1 Exp(n1) Z2 Exp(n2) = Z1 Z2 Exp(Ad(Z2^-1) n1 + n2), so
//   cov(0,1) = C1,  cov(1,2) = C2,  cov(0,2) = Ad(Z2^-1) C1 Ad(Z2^-1)^T + C2,
// whatever the prior on keyframe 0.
TEST(BackendGraph, RelativePoseCovKnownAnswerChain) {
  BackendConfig cfg = default_backend_config();
  cfg.relative_factor = RelativeFactor::kSe3Exact;
  Graph g(cfg, default_imu_config(), Vec3::Zero());
  std::mt19937 rng(7);
  const Pose T0 = test::pose_from(Vec3(0.1, -0.2, 0.3), Vec3(1.0, 2.0, 0.5));
  const Pose Z1 = test::pose_from(Vec3(0.05, 0.02, 0.4), Vec3(1.5, 0.3, -0.1));
  const Pose Z2 = test::pose_from(Vec3(-0.03, 0.06, -0.7), Vec3(0.8, -1.2, 0.2));
  const Mat6 C1 = random_cov(rng, 0.03, 0.01);
  const Mat6 C2 = random_cov(rng, 0.05, 0.02);
  const Pose truth[3] = {T0, T0 * Z1, T0 * Z1 * Z2};
  for (KeyId k = 0; k < 3; ++k) {
    NavState x;
    x.t = common::Timestamp{static_cast<std::int64_t>(k) * 1'000'000'000};
    x.T_w_b = truth[k];
    g.add_keyframe(x.t, x);
  }
  g.add_lidar_factor(0, 1, Z1, C1, nullptr);
  g.add_lidar_factor(1, 2, Z2, C2, nullptr);
  ASSERT_TRUE(g.optimize().is_ok());
  const common::Result<Mat6> r01 = g.relative_pose_cov(0, 1);
  const common::Result<Mat6> r12 = g.relative_pose_cov(1, 2);
  const common::Result<Mat6> r02 = g.relative_pose_cov(0, 2);
  ASSERT_TRUE(r01.is_ok()) << r01.reason();
  ASSERT_TRUE(r12.is_ok()) << r12.reason();
  ASSERT_TRUE(r02.is_ok()) << r02.reason();
  const Mat6 Ad = common::se3_adj(Z2.inverse());
  const Mat6 expect02 = Ad * C1 * Ad.transpose() + C2;
  EXPECT_LT((r01.value() - C1).norm(), 1e-6 * C1.norm());
  EXPECT_LT((r12.value() - C2).norm(), 1e-6 * C2.norm());
  EXPECT_LT((r02.value() - expect02).norm(), 1e-6 * expect02.norm());
}

// Loopy graph (B-02 square with its loop, noisy): two independent checks of the joint.
//   1. Keyframe 0 pinned by a tight prior: cov(0, j) -> marginal_cov(j) (iSAM2's own
//      single-variable marginal, a different code path).
//   2. Inverse relation: T_ji = T_ij^-1, so cov(j, i) = Ad(T_ij) cov(i, j) Ad(T_ij)^T.
TEST(BackendGraph, RelativePoseCovConsistentOnLoopGraph) {
  BackendConfig cfg = default_backend_config();
  cfg.prior_pos_sigma_m = 1e-6;
  cfg.prior_rot_sigma_rad = 1e-6;
  const test::Scenario s = test::square_loop(true, 3);
  Graph g(cfg, default_imu_config(), Vec3::Zero());
  run_square(g, s, 2);
  for (KeyId j = 1; j < 5; ++j) {
    const common::Result<Mat6> r = g.relative_pose_cov(0, j);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    const Mat6 m = g.marginal_cov(j);
    EXPECT_LT((r.value() - m).norm(), 1e-4 * m.norm()) << "keyframe " << j;
  }
  for (KeyId i = 1; i < 4; ++i) {
    for (KeyId j = i + 1; j < 5; ++j) {
      const common::Result<Mat6> rij = g.relative_pose_cov(i, j);
      const common::Result<Mat6> rji = g.relative_pose_cov(j, i);
      ASSERT_TRUE(rij.is_ok() && rji.is_ok());
      const Pose T_ij = state_or_fail(g, i).T_w_b.inverse() * state_or_fail(g, j).T_w_b;
      const Mat6 expect = common::transform_covariance(rij.value(), T_ij);
      EXPECT_LT((rji.value() - expect).norm(), 1e-6 * expect.norm()) << i << " " << j;
    }
  }
}

// --- remove_loop_factor (ADR-028) ------------------------------------------------------------

// A pending loop (no optimize() yet) is dropped from the pending set: the graph optimises as if
// it had never been added. Unknown pairs and lidar factors are not loop factors.
TEST(BackendGraph, RemoveLoopFactorPendingAndUnknown) {
  const test::Scenario s = test::square_loop(true, 5);
  Graph ref(default_backend_config(), default_imu_config(), Vec3::Zero());
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  for (KeyId k = 0; k < 5; ++k) {
    for (Graph* x : {&ref, &g}) {
      x->add_keyframe(s.truth[k].t, s.guess[k]);
      if (k > 0) {
        x->add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
      }
    }
    if (k == 4) {
      common::LoopConstraint lc;
      lc.i = 0;
      lc.j = 4;
      lc.T_bi_bj = s.edge_meas[4];
      lc.cov = s.edge_cov;
      g.add_loop_factor(lc);
      EXPECT_TRUE(g.has_loop_factor(0, 4));
      EXPECT_FALSE(g.has_loop_factor(3, 4));     // a lidar edge, not a loop
      EXPECT_FALSE(g.remove_loop_factor(3, 4));  // a lidar edge, not a loop
      EXPECT_FALSE(g.remove_loop_factor(1, 4));  // never added
      EXPECT_TRUE(g.remove_loop_factor(0, 4));   // pending
      EXPECT_FALSE(g.has_loop_factor(0, 4));
    }
    ASSERT_TRUE(ref.optimize().is_ok());
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (KeyId k = 0; k < 5; ++k) {
    EXPECT_EQ(state_or_fail(ref, k).T_w_b.p, state_or_fail(g, k).T_w_b.p) << k;
  }
}

// The square without its loop, optimised like run_square (5 keyframes + `extra` updates).
void run_square_without_loop(Graph& g, const test::Scenario& s, int extra) {
  for (KeyId k = 0; k < 5; ++k) {
    g.add_keyframe(s.truth[k].t, s.guess[k]);
    if (k > 0) {
      g.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
    }
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (int n = 0; n < extra; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
}

// The square's loop, measured `off_m` metres off along x of body 0 (a wrong loop inside the
// Cauchy core when small: it drags the square by a large part of it).
common::LoopConstraint square_loop_off(const test::Scenario& s, double off_m) {
  common::LoopConstraint lc;
  lc.i = 0;
  lc.j = 4;
  lc.T_bi_bj = s.edge_meas[4];
  lc.T_bi_bj.p.x() += off_m;
  lc.cov = s.edge_cov;
  return lc;
}

// A committed wrong loop (0.5 m off: inside the Cauchy core, it drags the square by
// centimetres) removed later, followed by the pipeline's update count (1 + isam_extra_updates
// = 3): the graph equals the one that never had it within 1e-4 m (iSAM2 relinearises only the
// variables that moved more than relinearize_threshold; the residual is reported), and the
// marginal of the last pose grows back.
TEST(BackendGraph, RemoveLoopFactorCommitted) {
  const test::Scenario s = test::square_loop(true, 9);
  Graph with(default_backend_config(), default_imu_config(), Vec3::Zero());
  Graph without(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square_without_loop(without, s, 3);
  for (KeyId k = 0; k < 5; ++k) {
    with.add_keyframe(s.truth[k].t, s.guess[k]);
    if (k > 0) {
      with.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
    }
    if (k == 4) {
      with.add_loop_factor(square_loop_off(s, 0.5));
    }
    ASSERT_TRUE(with.optimize().is_ok());
  }
  double drag = 0.0;
  for (KeyId k = 0; k < 5; ++k) {
    drag =
        std::max(drag, (state_or_fail(with, k).T_w_b.p - state_or_fail(without, k).T_w_b.p).norm());
  }
  const double tr_with = with.marginal_cov(4).trace();
  ASSERT_TRUE(with.has_loop_factor(0, 4));
  ASSERT_TRUE(with.remove_loop_factor(0, 4));
  EXPECT_FALSE(with.has_loop_factor(0, 4));
  for (int n = 0; n < 3; ++n) {
    ASSERT_TRUE(with.optimize().is_ok());
  }
  double after = 0.0;
  for (KeyId k = 0; k < 5; ++k) {
    after = std::max(after,
                     (state_or_fail(with, k).T_w_b.p - state_or_fail(without, k).T_w_b.p).norm());
  }
  RecordProperty("drag_before_m", std::to_string(drag));
  RecordProperty("drag_after_m", std::to_string(after));
  EXPECT_GT(drag, 0.05);  // the wrong loop really dragged the square
  EXPECT_LT(after, 1e-4);
  EXPECT_GT(with.marginal_cov(4).trace(), tr_with);
  EXPECT_LT((with.marginal_cov(4) - without.marginal_cov(4)).norm(),
            1e-3 * without.marginal_cov(4).norm());
}

// A pair added twice: each removal takes the most recent one; after the first the graph holds
// only the older (equal to a graph built with only it), after the second none, a third finds
// nothing.
TEST(BackendGraph, RemoveLoopFactorOfAPairAddedTwice) {
  const test::Scenario s = test::square_loop(true, 9);
  Graph twice(default_backend_config(), default_imu_config(), Vec3::Zero());
  Graph older(default_backend_config(), default_imu_config(), Vec3::Zero());
  for (KeyId k = 0; k < 5; ++k) {
    for (Graph* x : {&twice, &older}) {
      x->add_keyframe(s.truth[k].t, s.guess[k]);
      if (k > 0) {
        x->add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
      }
    }
    if (k == 4) {
      twice.add_loop_factor(square_loop_off(s, 0.3));  // older
      older.add_loop_factor(square_loop_off(s, 0.3));
    }
    ASSERT_TRUE(twice.optimize().is_ok());
    ASSERT_TRUE(older.optimize().is_ok());
  }
  twice.add_loop_factor(square_loop_off(s, 0.0));  // newer, committed by the next update
  ASSERT_TRUE(twice.optimize().is_ok());
  ASSERT_TRUE(twice.remove_loop_factor(0, 4));  // the newer one
  EXPECT_TRUE(twice.has_loop_factor(0, 4));     // the older one stays
  for (int n = 0; n < 3; ++n) {
    ASSERT_TRUE(twice.optimize().is_ok());
    ASSERT_TRUE(older.optimize().is_ok());
  }
  for (KeyId k = 0; k < 5; ++k) {
    EXPECT_LT((state_or_fail(twice, k).T_w_b.p - state_or_fail(older, k).T_w_b.p).norm(), 1e-4)
        << k;
  }
  EXPECT_TRUE(twice.remove_loop_factor(0, 4));
  EXPECT_FALSE(twice.has_loop_factor(0, 4));
  EXPECT_FALSE(twice.remove_loop_factor(0, 4));
}

// Removal followed by a failing update (a new keyframe held only by a rank-deficient GNSS
// factor, as in OptimizeFailureDropsPendingFactorsKeepsKeyframes): iSAM2 is rebuilt from the
// committed factors, which no longer hold the loop, so it does not come back; the next updates
// match the square without the loop.
TEST(BackendGraph, RemoveLoopFactorSurvivesAFailedUpdate) {
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const test::Scenario s = test::square_loop(true, 9);
  Graph with(default_backend_config(), default_imu_config(), p_b_g);
  Graph without(default_backend_config(), default_imu_config(), p_b_g);
  for (KeyId k = 0; k < 5; ++k) {
    for (Graph* x : {&with, &without}) {
      x->add_keyframe(s.truth[k].t, s.guess[k]);
      if (k > 0) {
        x->add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
      }
    }
    if (k == 4) {
      with.add_loop_factor(square_loop_off(s, 0.5));
    }
    ASSERT_TRUE(with.optimize().is_ok());
    ASSERT_TRUE(without.optimize().is_ok());
  }
  ASSERT_TRUE(with.remove_loop_factor(0, 4));
  NavState x5 = s.truth[4];
  x5.t = common::Timestamp{s.truth[4].t.ns + 1'000'000'000};
  common::GnssMeasurement m;
  m.p_e_g = x5.T_w_b * p_b_g;
  m.cov = Mat3::Identity() * 0.01;
  for (Graph* x : {&with, &without}) {
    x->add_keyframe(x5.t, x5);
    x->add_gnss_factor(5, m, Pose{});  // 3 of 6 DoF of keyframe 5: indeterminant
    ASSERT_FALSE(x->optimize().is_ok());
    x->add_lidar_factor(4, 5, s.truth[4].T_w_b.inverse() * x5.T_w_b, s.edge_cov, nullptr);
    for (int n = 0; n < 3; ++n) {
      ASSERT_TRUE(x->optimize().is_ok());
    }
  }
  EXPECT_FALSE(with.has_loop_factor(0, 4));
  for (KeyId k = 0; k < 6; ++k) {
    EXPECT_LT((state_or_fail(with, k).T_w_b.p - state_or_fail(without, k).T_w_b.p).norm(), 1e-4)
        << k;
  }
}

// --- relative_pose_cov against an independent dense solve and Monte Carlo (review H3, M-2) ---
// A 75-keyframe circle built incrementally (one iSAM2 update per keyframe) with exact SE(3)
// relative factors, random full covariances (fixed seed) and 7 loops one lap back, so the Bayes
// tree has many cliques and the joint P_ij crosses branches. Reference: the dense information
// matrix in mss order (prior + J_i = -Ad(T_j^-1 T_i), J_j = I per factor), inverted, then the
// relative-pose formula; and 100k samples of that 12x12 joint pushed through T_i^-1 T_j.
struct DenseFactor {
  int i = 0;
  int j = 0;
  Pose Z;
  Mat6 C = Mat6::Identity();
};

struct DenseGraph {
  std::vector<Pose> T;
  std::vector<DenseFactor> f;
};

DenseGraph build_dense_check_graph(Graph& g, int n, std::mt19937& rng) {
  DenseGraph b;
  constexpr double kTwoPi = 6.283185307179586;
  for (int k = 0; k < n; ++k) {
    const double a = kTwoPi * k / 30.0;
    b.T.push_back(
        test::pose_from(Vec3(0.05 * std::sin(0.7 * k), 0.03 * std::cos(0.3 * k), a + 0.2),
                        Vec3(5.0 * std::cos(a), 5.0 * std::sin(a), 0.2 * std::sin(0.5 * k))));
  }
  for (int k = 0; k < n; ++k) {
    NavState x;
    x.t = common::Timestamp{static_cast<std::int64_t>(k) * 1'000'000'000};
    x.T_w_b = b.T[static_cast<std::size_t>(k)];
    g.add_keyframe(x.t, x);
    const auto add = [&](int i, double sm, double sr) {
      DenseFactor f;
      f.i = i;
      f.j = k;
      f.Z = b.T[static_cast<std::size_t>(i)].inverse() * b.T[static_cast<std::size_t>(k)];
      f.C = random_cov(rng, sm, sr);
      g.add_lidar_factor(static_cast<KeyId>(i), static_cast<KeyId>(k), f.Z, f.C, nullptr);
      b.f.push_back(f);
    };
    if (k > 0) {
      add(k - 1, 0.02, 0.004);
    }
    if (k >= 30 && k % 7 == 0) {
      add(k - 30, 0.05, 0.01);
    }
    EXPECT_TRUE(g.optimize().is_ok());
  }
  return b;
}

TEST(BackendGraph, RelativePoseCovDenseAndMonteCarloOnLoopyGraph) {
  BackendConfig cfg = default_backend_config();
  cfg.relative_factor = RelativeFactor::kSe3Exact;
  Graph g(cfg, default_imu_config(), Vec3::Zero());
  std::mt19937 rng(11);
  constexpr int kN = 75;
  const DenseGraph b = build_dense_check_graph(g, kN, rng);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6 * kN, 6 * kN);
  Vec6 sp;
  sp << Vec3::Constant(cfg.prior_pos_sigma_m), Vec3::Constant(cfg.prior_rot_sigma_rad);
  H.block<6, 6>(0, 0) += Mat6(sp.cwiseAbs2().cwiseInverse().asDiagonal());
  for (const DenseFactor& f : b.f) {
    const Mat6 Ji = -common::se3_adj(b.T[static_cast<std::size_t>(f.j)].inverse() *
                                     b.T[static_cast<std::size_t>(f.i)]);
    const Mat6 W = f.C.inverse();
    H.block<6, 6>(6 * f.i, 6 * f.i) += Ji.transpose() * W * Ji;
    H.block<6, 6>(6 * f.i, 6 * f.j) += Ji.transpose() * W;
    H.block<6, 6>(6 * f.j, 6 * f.i) += W * Ji;
    H.block<6, 6>(6 * f.j, 6 * f.j) += W;
  }
  const Eigen::MatrixXd P = H.inverse();
  const int pairs[][2] = {{0, 74},
                          {3, 74},
                          {10, 74},
                          {44, 74},
                          {5, 60},
                          {12, 40},
                          {20, 21},
                          {1, 50},
                          {33, 70},
                          {2, 3},
                          {60, 73},
                          {7, 37}};
  std::normal_distribution<double> nd(0.0, 1.0);
  for (const auto& pr : pairs) {
    const int i = pr[0];
    const int j = pr[1];
    const Pose& Ti = b.T[static_cast<std::size_t>(i)];
    const Pose& Tj = b.T[static_cast<std::size_t>(j)];
    const common::Result<Mat6> r =
        g.relative_pose_cov(static_cast<KeyId>(i), static_cast<KeyId>(j));
    ASSERT_TRUE(r.is_ok()) << r.reason();
    const Mat6 Ad = common::se3_adj(Tj.inverse() * Ti);
    const Mat6 Pii = P.block<6, 6>(6 * i, 6 * i);
    const Mat6 Pjj = P.block<6, 6>(6 * j, 6 * j);
    const Mat6 Pij = P.block<6, 6>(6 * i, 6 * j);
    const Mat6 expect =
        Ad * Pii * Ad.transpose() - Ad * Pij - Pij.transpose() * Ad.transpose() + Pjj;
    EXPECT_LT((r.value() - expect).norm(), 1e-6 * expect.norm()) << i << "," << j;
    // The cross term matters: leaving it out is visibly wrong (> 50 %) for every pair.
    const Mat6 no_cross = Ad * Pii * Ad.transpose() + Pjj;
    EXPECT_GT((no_cross - expect).norm(), 0.5 * expect.norm()) << i << "," << j;
    // Monte Carlo of the 12x12 joint through Log(T_ij^-1 T_i'^-1 T_j').
    Eigen::Matrix<double, 12, 12> Pj;
    Pj << Pii, Pij, Pij.transpose(), Pjj;
    const Eigen::Matrix<double, 12, 12> L = Pj.llt().matrixL();
    const Pose Tij = Ti.inverse() * Tj;
    Mat6 S = Mat6::Zero();
    constexpr int kSamples = 100000;
    for (int n = 0; n < kSamples; ++n) {
      Eigen::Matrix<double, 12, 1> z;
      for (int k = 0; k < 12; ++k) {
        z(k) = nd(rng);
      }
      const Eigen::Matrix<double, 12, 1> d = L * z;
      const Pose Ti2 = common::boxplus(Ti, Vec6(d.head<6>()));
      const Pose Tj2 = common::boxplus(Tj, Vec6(d.tail<6>()));
      const Vec6 e = common::boxminus(Pose(Ti2.inverse() * Tj2), Tij);
      S += e * e.transpose();
    }
    S /= kSamples;
    const Eigen::LLT<Mat6> llt(r.value());
    const Mat6 Li = llt.matrixL().solve(Mat6::Identity());
    const Mat6 M = Li * S * Li.transpose();
    const Eigen::SelfAdjointEigenSolver<Mat6> es(Mat6(0.5 * (M + M.transpose())));
    EXPECT_LT(std::abs(es.eigenvalues()(0) - 1.0), 0.04) << i << "," << j;
    EXPECT_LT(std::abs(es.eigenvalues()(5) - 1.0), 0.04) << i << "," << j;
  }
}

TEST(BackendGraph, RelativePoseCovErrors) {
  Graph g(default_backend_config(), default_imu_config(), Vec3::Zero());
  EXPECT_FALSE(g.relative_pose_cov(0, 1).is_ok());  // unknown
  NavState x0;
  g.add_keyframe(x0.t, x0);
  NavState x1;
  x1.t = common::Timestamp{1'000'000'000};
  g.add_keyframe(x1.t, x1);
  EXPECT_FALSE(g.relative_pose_cov(0, 0).is_ok());  // i == j
  EXPECT_FALSE(g.relative_pose_cov(0, 1).is_ok());  // not optimised
  Mat6 C = Mat6::Identity() * 1e-4;
  g.add_lidar_factor(0, 1, Pose{}, C, nullptr);
  ASSERT_TRUE(g.optimize().is_ok());
  EXPECT_TRUE(g.relative_pose_cov(0, 1).is_ok());
}

}  // namespace
}  // namespace mss::backend
