// B-03 (robust outlier loop), B-04 (iSAM2 vs the batch Gauss-Newton fixed point), determinism.
// Derivation: docs/derivations/backend_factors.md.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "mss/backend/graph.hpp"
#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

#include "backend_test_config.hpp"
#include "constants.hpp"
#include "conversions.hpp"
#include "gnss_factor.hpp"
#include "graph_scenarios.hpp"
#include "imu_factor.hpp"
#include "odom_factor.hpp"
#include "relative_pose_factor.hpp"

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

using detail::kLoopCauchyK;          // constants.hpp, backend_factors.md section 5
constexpr double kDragLimitM = 0.1;  // B-03: 5 x the 0.02 m edge position sigma

NavState state_or_fail(const Graph& g, KeyId k) {
  const common::Result<NavState> r = g.keyframe_state(k);
  EXPECT_TRUE(r.is_ok()) << "keyframe " << k << ": " << r.reason();
  return r.is_ok() ? r.value() : NavState{};
}

enum class Outlier { kNone, kLoop, kLidar };

// Square loop as in B-02. Optional outlier (1, 3) added with keyframe 4, and an optional
// degeneracy report on edge (1, 2).
void run_square(Graph& g,
                const test::Scenario& s,
                Outlier outlier,
                const common::DegeneracyReport* degeneracy_1_2) {
  for (std::size_t k = 0; k < 5; ++k) {
    g.add_keyframe(s.truth[k].t, s.guess[k]);
    if (k > 0) {
      g.add_lidar_factor(
          k - 1, k, s.edge_meas[k - 1], s.edge_cov, k == 2 ? degeneracy_1_2 : nullptr);
    }
    if (k == 4) {
      common::LoopConstraint lc;
      lc.i = 0;
      lc.j = 4;
      lc.T_bi_bj = s.edge_meas[4];
      lc.cov = s.edge_cov;
      g.add_loop_factor(lc);
      Vec6 wrong;
      wrong << 5.0, 0.0, 0.0, 0.0, 0.0, 0.5;
      common::LoopConstraint bad;
      bad.i = 1;
      bad.j = 3;
      bad.T_bi_bj = s.truth[1].T_w_b.inverse() * s.truth[3].T_w_b * common::se3_exp(wrong);
      bad.cov = s.edge_cov;
      if (outlier == Outlier::kLoop) {
        g.add_loop_factor(bad);
      } else if (outlier == Outlier::kLidar) {
        g.add_lidar_factor(bad.i, bad.j, bad.T_bi_bj, bad.cov, nullptr);
      }
    }
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (int n = 0; n < 5; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
}

double max_position_drag(const Graph& a, const Graph& b) {
  double worst = 0.0;
  for (KeyId k = 0; k < 5; ++k) {
    worst = std::max(worst, (state_or_fail(a, k).T_w_b.p - state_or_fail(b, k).T_w_b.p).norm());
  }
  return worst;
}

TEST(BackendIsam, B03_OutlierLoopDragsLessThanTenCm) {
  const test::Scenario s = test::square_loop(true, 42);
  Graph g1(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g1, s, Outlier::kNone, nullptr);
  Graph g2(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g2, s, Outlier::kLoop, nullptr);
  const double drag = max_position_drag(g1, g2);
  RecordProperty("drag_m", std::to_string(drag));
  EXPECT_LT(drag, kDragLimitM);
}

TEST(BackendIsam, B03_ControlNonRobustOutlierDragsMoreThanTenCm) {
  const test::Scenario s = test::square_loop(true, 42);
  Graph g1(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g1, s, Outlier::kNone, nullptr);
  Graph g2(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g2, s, Outlier::kLidar, nullptr);
  const double drag = max_position_drag(g1, g2);
  RecordProperty("drag_m", std::to_string(drag));
  EXPECT_GT(drag, kDragLimitM);
}

TEST(BackendIsam, DegeneracyReportIsDiagnosticOnly) {
  const test::Scenario s = test::square_loop(false, 0);
  Graph plain(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(plain, s, Outlier::kNone, nullptr);
  common::DegeneracyReport report;
  report.degenerate[0] = true;
  CapturingLogger log;
  Graph flagged(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(flagged, s, Outlier::kNone, &report);
  for (KeyId k = 0; k < 5; ++k) {
    EXPECT_EQ(state_or_fail(plain, k).T_w_b.p, state_or_fail(flagged, k).T_w_b.p);
    EXPECT_EQ(state_or_fail(plain, k).T_w_b.R, state_or_fail(flagged, k).T_w_b.R);
  }
  const auto n_degenerate =
      std::count_if(log.debugs.begin(), log.debugs.end(), [](const std::string& m) {
        return m.find("degenerate") != std::string::npos;
      });
  EXPECT_EQ(n_degenerate, 1);
}

// ---------------------------------------------------------------------------------------
// B-04 graph: 6 keyframes with IMU factors, noisy LiDAR edges (seed 11), GNSS fixes on
// keyframes 2 and 5, an odom factor 3 -> 4 (rank 3) and an inlier loop (0, 5).

struct B04Inputs {
  test::ImuChain chain;
  std::vector<NavState> guess;
  std::vector<Pose> lidar;  // lidar[k]: k -> k + 1
  Mat6 lidar_cov = Mat6::Identity();
  Pose T_e_w;
  Vec3 p_b_g = Vec3(0.3, 0.0, 0.8);
  common::GnssMeasurement fix2;
  common::GnssMeasurement fix5;
  common::OdomDelta odom_3_4;
  common::LoopConstraint loop;
};

B04Inputs b04_inputs() {
  B04Inputs in;
  in.chain = test::imu_chain(7, 6, 40);
  const std::vector<NavState>& truth = in.chain.truth;
  std::mt19937 rng(11);
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
  in.fix2.p_e_g = in.T_e_w * (truth[2].T_w_b * in.p_b_g) + Vec3(0.05, -0.03, 0.02);
  in.fix2.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
  in.fix5.p_e_g = in.T_e_w * (truth[5].T_w_b * in.p_b_g) + Vec3(-0.02, 0.04, -0.03);
  in.fix5.cov = Vec3(0.01, 0.01, 0.04).asDiagonal();
  in.odom_3_4 = test::wheel_delta(test::pose_from(Vec3(0.1, -0.2, 0.3), Vec3(0.2, 0.0, 0.1)));
  in.odom_3_4.T_b0_b1 = truth[3].T_w_b.inverse() * truth[4].T_w_b * noise();
  in.loop.i = 0;
  in.loop.j = 5;
  in.loop.T_bi_bj = truth[0].T_w_b.inverse() * truth[5].T_w_b * noise();
  in.loop.cov = in.lidar_cov;
  in.guess = truth;
  for (std::size_t k = 1; k < truth.size(); ++k) {
    in.guess[k].T_w_b = in.guess[k].T_w_b * common::se3_exp(Vec6::Constant(0.02));
    in.guess[k].v_w_b += Vec3(0.05, -0.05, 0.02);
  }
  return in;
}

BackendConfig b04_config() {
  BackendConfig cfg = default_backend_config();
  cfg.relinearize_threshold = 1e-10;  // relinearise every variable every update
  cfg.relinearize_skip = 1;
  return cfg;
}

void feed_b04(Graph& g, const B04Inputs& in, int extra) {
  for (KeyId k = 0; k < 6; ++k) {
    g.add_keyframe(in.chain.truth[k].t, in.guess[k]);
    if (k > 0) {
      g.add_imu_factor(k - 1, k, in.chain.deltas[k - 1]);
      g.add_lidar_factor(k - 1, k, in.lidar[k - 1], in.lidar_cov, nullptr);
    }
    if (k == 2) {
      g.add_gnss_factor(2, in.fix2, in.T_e_w);
    }
    if (k == 4) {
      g.add_odom_factor(3, 4, in.odom_3_4);
    }
    if (k == 5) {
      g.add_gnss_factor(5, in.fix5, in.T_e_w);
      g.add_loop_factor(in.loop);
    }
    ASSERT_TRUE(g.optimize().is_ok());
  }
  for (int n = 0; n < extra; ++n) {
    ASSERT_TRUE(g.optimize().is_ok());
  }
}

// The LiDAR / loop factor of the configured form (backend.relative_factor), as Graph builds it.
gtsam::NonlinearFactor::shared_ptr relative_factor_of(RelativeFactor form,
                                                      gtsam::Key x_i,
                                                      gtsam::Key x_j,
                                                      const Pose& T_bi_bj,
                                                      const gtsam::SharedNoiseModel& noise) {
  switch (form) {
    case RelativeFactor::kSe3Exact:
      return boost::make_shared<detail::Se3RelativePoseFactor>(x_i, x_j, T_bi_bj, noise);
    case RelativeFactor::kDecoupled:
      return boost::make_shared<detail::DecoupledRelativePoseFactor>(x_i, x_j, T_bi_bj, noise);
    case RelativeFactor::kBetween:
      break;
  }
  return boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      x_i, x_j, detail::to_gtsam(T_bi_bj), noise);
}

// The same factors as Graph builds them (backend_factors.md sections 2-7), for the batch
// reference, with the LiDAR / loop form of cfg.relative_factor. Initial values: the keyframe
// guesses.
void build_batch(const B04Inputs& in,
                 const BackendConfig& cfg,
                 const imu::ImuConfig& imu_cfg,
                 gtsam::NonlinearFactorGraph& graph,
                 gtsam::Values& init) {
  using detail::bias_key;
  using detail::pose_key;
  using detail::vel_key;
  Vec6 pose_sigmas;
  pose_sigmas << Vec3::Constant(cfg.prior_rot_sigma_rad), Vec3::Constant(cfg.prior_pos_sigma_m);
  Vec6 bias_sigmas;
  bias_sigmas << Vec3::Constant(cfg.prior_bias_acc_sigma_mps2),
      Vec3::Constant(cfg.prior_bias_gyro_sigma_radps);
  const NavState& x0 = in.guess[0];
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      pose_key(0), detail::to_gtsam(x0.T_w_b), gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      vel_key(0),
      gtsam::Vector3(x0.v_w_b),
      gtsam::noiseModel::Isotropic::Sigma(3, cfg.prior_vel_sigma_mps));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      bias_key(0), detail::to_gtsam(x0.bias), gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  const Vec3 g_w(0.0, 0.0, -imu_cfg.gravity_mps2);
  for (KeyId k = 0; k < 6; ++k) {
    init.insert(pose_key(k), detail::to_gtsam(in.guess[k].T_w_b));
    init.insert(vel_key(k), gtsam::Vector3(in.guess[k].v_w_b));
    init.insert(bias_key(k), detail::to_gtsam(in.guess[k].bias));
    if (k == 0) {
      continue;
    }
    const common::PreintegratedDelta& d = in.chain.deltas[k - 1];
    graph.emplace_shared<detail::ImuPreintFactor>(
        pose_key(k - 1), vel_key(k - 1), pose_key(k), vel_key(k), bias_key(k - 1), d, g_w);
    Vec6 rw;
    rw << Vec3::Constant(imu_cfg.acc_random_walk * std::sqrt(d.dt_s)),
        Vec3::Constant(imu_cfg.gyro_random_walk * std::sqrt(d.dt_s));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        bias_key(k - 1),
        bias_key(k),
        gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(rw));
    graph.push_back(relative_factor_of(
        cfg.relative_factor,
        pose_key(k - 1),
        pose_key(k),
        in.lidar[k - 1],
        gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(in.lidar_cov))));
  }
  graph.emplace_shared<detail::GnssPositionFactor>(
      pose_key(2), in.fix2.p_e_g, in.fix2.cov, in.T_e_w, in.p_b_g);
  graph.emplace_shared<detail::GnssPositionFactor>(
      pose_key(5), in.fix5.p_e_g, in.fix5.cov, in.T_e_w, in.p_b_g);
  const std::optional<detail::OdomBetweenFactor> odom =
      detail::OdomBetweenFactor::create(pose_key(3), pose_key(4), in.odom_3_4);
  ASSERT_TRUE(odom.has_value());
  ASSERT_EQ(odom->rank(), 3);
  graph.push_back(odom->clone());
  graph.push_back(relative_factor_of(
      cfg.relative_factor,
      pose_key(0),
      pose_key(5),
      in.loop.T_bi_bj,
      gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(kLoopCauchyK),
          gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(in.loop.cov)))));
}

constexpr int kB04GaussNewtonIterations = 10;  // fixed Gauss-Newton steps after LM (B04)

std::string sci(double v) {  // scientific notation for RecordProperty
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", v);
  return buf.data();
}

// Every relative-factor form (default decoupled, ADR-021; between and se3_exact selectable):
// iSAM2 agrees with the batch solution of the same factors (LM, then Gauss-Newton to the
// fixed point of the linearisation).
TEST(BackendIsam, B04_IsamMatchesBatchReference) {
  const B04Inputs in = b04_inputs();
  for (const RelativeFactor form :
       {RelativeFactor::kDecoupled, RelativeFactor::kBetween, RelativeFactor::kSe3Exact}) {
    SCOPED_TRACE(static_cast<int>(form));
    BackendConfig cfg = b04_config();
    cfg.relative_factor = form;
    Graph g(cfg, default_imu_config(), in.p_b_g);
    {
      CapturingLogger log;  // every factor must be accepted
      feed_b04(g, in, 20);
      EXPECT_TRUE(log.warnings.empty()) << log.warnings.front();
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values init;
    build_batch(in, cfg, default_imu_config(), graph, init);
    gtsam::LevenbergMarquardtParams params;
    params.relativeErrorTol = 1e-15;
    params.absoluteErrorTol = 1e-15;
    params.maxIterations = 100;
    // The reference is the batch Gauss-Newton fixed point of the same linearisation iSAM2
    // uses. Plain LM is not a valid reference: gtsam::PriorFactor<Pose3> (keyframe 0) returns
    // an identity Jacobian for its Logmap residual (and so does BetweenFactor<Pose3> in the
    // between form), so the linearisation's fixed point is not the cost minimum (true cost
    // gradient about 0.09 there for decoupled, 0.68 for between), and LM, which accepts steps by
    // the true cost, stalls at its lambda bound away from it. LM gives the start; fixed
    // Gauss-Newton iterations (linearize, solve, retract) reach the fixed point, which is asserted.
    gtsam::Values batch = gtsam::LevenbergMarquardtOptimizer(graph, init, params).optimize();
    for (int it = 0; it < kB04GaussNewtonIterations; ++it) {
      batch = batch.retract(graph.linearize(batch)->optimize());
    }
    const double grad_norm = graph.linearize(batch)->gradientAtZero().vector().norm();
    RecordProperty("reference_gradient_norm_form_" + std::to_string(static_cast<int>(form)),
                   sci(grad_norm));
    ASSERT_LT(grad_norm, 1e-5);  // the reference is converged

    double worst = 0.0;  // largest pose / velocity / bias difference over the keyframes
    for (KeyId k = 0; k < 6; ++k) {
      const NavState x = state_or_fail(g, k);
      const Pose T_batch = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::pose_key(k)));
      const double d_pose = common::se3_log(T_batch.inverse() * x.T_w_b).norm();
      const double d_vel = (batch.at<gtsam::Vector3>(detail::vel_key(k)) - x.v_w_b).norm();
      const common::ImuBias b =
          detail::from_gtsam(batch.at<gtsam::imuBias::ConstantBias>(detail::bias_key(k)));
      const double d_ba = (b.ba - x.bias.ba).norm();
      const double d_bg = (b.bg - x.bias.bg).norm();
      EXPECT_LT(d_pose, 1e-6) << "keyframe " << k;
      EXPECT_LT(d_vel, 1e-6) << "keyframe " << k;
      EXPECT_LT(d_ba, 1e-6) << "keyframe " << k;
      EXPECT_LT(d_bg, 1e-6) << "keyframe " << k;
      worst = std::max({worst, d_pose, d_vel, d_ba, d_bg});
    }
    RecordProperty("worst_difference_form_" + std::to_string(static_cast<int>(form)), sci(worst));
  }
}

TEST(BackendIsam, RepeatedOptimizeIsStableAndDeterministic) {
  const B04Inputs in = b04_inputs();
  Graph a(b04_config(), default_imu_config(), in.p_b_g);
  feed_b04(a, in, 20);
  std::vector<NavState> before;
  for (KeyId k = 0; k < 6; ++k) {
    before.push_back(state_or_fail(a, k));
  }
  const common::Result<Estimate> ra = a.optimize();
  ASSERT_TRUE(ra.is_ok());
  for (KeyId k = 0; k < 6; ++k) {
    const NavState x = state_or_fail(a, k);
    EXPECT_LT((x.T_w_b.p - before[k].T_w_b.p).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((x.T_w_b.R - before[k].T_w_b.R).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((x.v_w_b - before[k].v_w_b).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((x.bias.ba - before[k].bias.ba).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((x.bias.bg - before[k].bias.bg).cwiseAbs().maxCoeff(), 1e-12);
  }

  Graph b(b04_config(), default_imu_config(), in.p_b_g);
  feed_b04(b, in, 20);
  const common::Result<Estimate> rb = b.optimize();
  ASSERT_TRUE(rb.is_ok());
  const Estimate& ea = ra.value();
  const Estimate& eb = rb.value();
  EXPECT_EQ(ea.keyframe_id, eb.keyframe_id);
  EXPECT_EQ(ea.state.T_w_b.p, eb.state.T_w_b.p);
  EXPECT_EQ(ea.state.T_w_b.R, eb.state.T_w_b.R);
  EXPECT_EQ(ea.state.v_w_b, eb.state.v_w_b);
  EXPECT_EQ(ea.state.bias.ba, eb.state.bias.ba);
  EXPECT_EQ(ea.state.bias.bg, eb.state.bias.bg);
  EXPECT_EQ(ea.cov, eb.cov);
}

}  // namespace
}  // namespace mss::backend
