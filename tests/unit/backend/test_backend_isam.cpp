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
#include "pose_prior_factor.hpp"
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

// B-03 with ADR-028: the outlier loop is removed after it was committed (remove_loop_factor,
// as a loop selector does when it finds the loop inconsistent), then the pipeline's 3 updates:
// the graph returns to the one without the outlier, so no drag is left (tolerance 1e-6 m).
TEST(BackendIsam, B03_OutlierLoopRemovedLeavesNoDrag) {
  const test::Scenario s = test::square_loop(true, 42);
  Graph g1(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g1, s, Outlier::kNone, nullptr);
  Graph g2(default_backend_config(), default_imu_config(), Vec3::Zero());
  run_square(g2, s, Outlier::kLoop, nullptr);
  const double before = max_position_drag(g1, g2);
  ASSERT_TRUE(g2.remove_loop_factor(1, 3));
  for (int n = 0; n < 3; ++n) {  // as the pipeline: 1 + isam_extra_updates
    ASSERT_TRUE(g2.optimize().is_ok());
  }
  const double after = max_position_drag(g1, g2);
  RecordProperty("drag_before_m", std::to_string(before));
  RecordProperty("drag_after_m", std::to_string(after));
  EXPECT_GT(before, 1e-4);  // the kernel leaves a small drag (B03_OutlierLoopDragsLessThanTenCm)
  EXPECT_LT(after, 1e-6);
  EXPECT_FALSE(g2.remove_loop_factor(1, 3));  // already removed
  EXPECT_TRUE(g2.remove_loop_factor(0, 4));   // the true loop is still there
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
  graph.emplace_shared<detail::PosePriorFactor>(
      pose_key(0), x0.T_w_b, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
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
constexpr double kB04GradientStep = 1e-6;      // central-difference step of the cost gradient

// Norm of the gradient of the true cost graph.error() at `values`, by central differences
// along every tangent coordinate (independent of the factors' analytic Jacobians).
double cost_gradient_norm(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
  const gtsam::VectorValues zero = values.zeroVectors();
  double sum_sq = 0.0;
  for (const auto& entry : zero) {
    for (Eigen::Index i = 0; i < entry.second.size(); ++i) {
      gtsam::VectorValues plus = zero;
      gtsam::VectorValues minus = zero;
      plus.at(entry.first)(i) = kB04GradientStep;
      minus.at(entry.first)(i) = -kB04GradientStep;
      const double d = (graph.error(values.retract(plus)) - graph.error(values.retract(minus))) /
                       (2.0 * kB04GradientStep);
      sum_sq += d * d;
    }
  }
  return std::sqrt(sum_sq);
}

std::string sci(double v) {  // scientific notation for RecordProperty
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", v);
  return buf.data();
}

// Every relative-factor form (default decoupled, ADR-021; between and se3_exact selectable):
// iSAM2 agrees with the batch solution of the same factors (LM, then Gauss-Newton to the
// fixed point of the linearisation); for the exact forms that point is the true cost minimum.
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
    // Reference: the batch Gauss-Newton fixed point of the same linearisation iSAM2 uses (LM,
    // then fixed iterations of linearize, solve, retract), asserted by its linearisation
    // gradient. For se3_exact and decoupled every pose factor has an exact Jacobian
    // (PosePriorFactor on keyframe 0, §2b; the relative factors, §5), so that fixed point is
    // also the minimum of the true cost: the central-difference gradient of graph.error() is
    // asserted at the reference and at the iSAM2 estimate (0.09 at the iSAM2 estimate with
    // gtsam::PriorFactor<Pose3>, whose Jacobian is the identity). The between form keeps
    // BetweenFactor<Pose3>, exact only at zero residual: its fixed point is not the cost
    // minimum, and the control asserts that (true gradient about 0.66).
    gtsam::Values batch = gtsam::LevenbergMarquardtOptimizer(graph, init, params).optimize();
    for (int it = 0; it < kB04GaussNewtonIterations; ++it) {
      batch = batch.retract(graph.linearize(batch)->optimize());
    }
    const double grad_norm = graph.linearize(batch)->gradientAtZero().vector().norm();
    RecordProperty("reference_gradient_norm_form_" + std::to_string(static_cast<int>(form)),
                   sci(grad_norm));
    ASSERT_LT(grad_norm, 1e-5);  // the fixed point is reached

    gtsam::Values isam = batch;  // the iSAM2 estimate, as Values of the batch graph
    for (KeyId k = 0; k < 6; ++k) {
      const NavState x = state_or_fail(g, k);
      isam.update(detail::pose_key(k), detail::to_gtsam(x.T_w_b));
      isam.update(detail::vel_key(k), gtsam::Vector3(x.v_w_b));
      isam.update(detail::bias_key(k), detail::to_gtsam(x.bias));
    }
    const double true_grad_ref = cost_gradient_norm(graph, batch);
    const double true_grad_isam = cost_gradient_norm(graph, isam);
    RecordProperty("true_gradient_reference_form_" + std::to_string(static_cast<int>(form)),
                   sci(true_grad_ref));
    RecordProperty("true_gradient_isam_form_" + std::to_string(static_cast<int>(form)),
                   sci(true_grad_isam));
    if (form == RelativeFactor::kBetween) {
      EXPECT_GT(true_grad_isam, 0.1);  // control: approximate Jacobian, not the cost minimum
    } else {
      EXPECT_LT(true_grad_ref, 1e-5);   // the fixed point is the cost minimum
      EXPECT_LT(true_grad_isam, 1e-5);  // and iSAM2 converges to it
    }

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

// Review I-2 (exact pose prior, backend_factors.md section 2b): the GNSS frame variable E far
// from its prior. Noise-free fixes from the true T_e_w on the square loop; the factors are
// given a T_e_w 20 deg (yaw) and 3 m off, which centres the E prior there, so E settles near
// the truth, about 20 deg and 3 m from its prior. iSAM2 must reach the batch minimum of the
// true cost. Control: the same batch graph with gtsam::PriorFactor<Pose3> on E (identity
// Jacobian, same cost) has a Gauss-Newton fixed point away from that minimum.
TEST(BackendIsam, FrameVariableMatchesBatchAtLargePriorResidual) {
  constexpr KeyId kKeyframes = 4;
  const Vec3 p_b_g(0.3, 0.0, 0.8);
  const Pose T_e_w = test::pose_from(Vec3(0.0, 0.0, 0.5), Vec3(10.0, -5.0, 1.0));
  const double yaw_off_rad = 20.0 * EIGEN_PI / 180.0;
  const Pose T_e_w_off =
      test::pose_from(Vec3(0.0, 0.0, 0.5 + yaw_off_rad), Vec3(12.4, -6.8, 1.0));  // 3 m off
  const test::Scenario s = test::square_loop(false, 0);
  BackendConfig cfg = b04_config();
  cfg.gnss_frame_variable = true;
  cfg.gnss_frame_tilt_sigma_rad = 0.01 * EIGEN_PI / 180.0;
  cfg.gnss_frame_yaw_sigma_rad = 30.0 * EIGEN_PI / 180.0;
  cfg.gnss_frame_pos_sigma_m = 10.0;
  const Mat3 fix_cov = Vec3(0.04, 0.04, 0.09).asDiagonal();

  Graph g(cfg, default_imu_config(), p_b_g);
  std::vector<common::GnssMeasurement> fixes;
  {
    CapturingLogger log;  // every factor must be accepted
    for (KeyId k = 0; k < kKeyframes; ++k) {
      g.add_keyframe(s.truth[k].t, s.guess[k]);
      if (k > 0) {
        g.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
        common::GnssMeasurement m;
        m.t = s.truth[k].t;
        m.p_e_g = T_e_w * (s.truth[k].T_w_b * p_b_g);
        m.cov = fix_cov;
        g.add_gnss_factor(k, m, T_e_w_off);
        fixes.push_back(m);
      }
      ASSERT_TRUE(g.optimize().is_ok());
    }
    for (int n = 0; n < 20; ++n) {
      ASSERT_TRUE(g.optimize().is_ok());
    }
    EXPECT_TRUE(log.warnings.empty()) << log.warnings.front();
  }
  const common::Result<Pose> E = g.gnss_frame_estimate();
  ASSERT_TRUE(E.is_ok());
  const Vec6 r_prior = common::se3_log(T_e_w_off.inverse() * E.value());  // [rho, phi]
  RecordProperty("frame_prior_residual_deg", sci(r_prior.tail<3>().norm() * 180.0 / EIGEN_PI));
  RecordProperty("frame_prior_residual_m", sci(r_prior.head<3>().norm()));
  EXPECT_GT(r_prior.tail<3>().norm(), 15.0 * EIGEN_PI / 180.0);  // the large-residual regime
  EXPECT_GT(r_prior.head<3>().norm(), 2.0);

  // The batch graph of the same factors as Graph builds them (section 2 priors, section 5
  // relative factors, section 6b frame factors and E prior); the E prior exact or GTSAM's.
  Vec6 pose_sigmas;
  pose_sigmas << Vec3::Constant(cfg.prior_rot_sigma_rad), Vec3::Constant(cfg.prior_pos_sigma_m);
  Vec6 bias_sigmas;
  bias_sigmas << Vec3::Constant(cfg.prior_bias_acc_sigma_mps2),
      Vec3::Constant(cfg.prior_bias_gyro_sigma_radps);
  Vec6 frame_sigmas;
  frame_sigmas << cfg.gnss_frame_tilt_sigma_rad, cfg.gnss_frame_tilt_sigma_rad,
      cfg.gnss_frame_yaw_sigma_rad, Vec3::Constant(cfg.gnss_frame_pos_sigma_m);
  const auto build = [&](bool exact_frame_prior) {
    gtsam::NonlinearFactorGraph graph;
    const NavState& x0 = s.guess[0];
    graph.emplace_shared<detail::PosePriorFactor>(
        detail::pose_key(0), x0.T_w_b, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        detail::vel_key(0),
        gtsam::Vector3(x0.v_w_b),
        gtsam::noiseModel::Isotropic::Sigma(3, cfg.prior_vel_sigma_mps));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        detail::bias_key(0),
        detail::to_gtsam(x0.bias),
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
    if (exact_frame_prior) {
      graph.emplace_shared<detail::PosePriorFactor>(
          detail::frame_key(), T_e_w_off, gtsam::noiseModel::Diagonal::Sigmas(frame_sigmas));
    } else {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          detail::frame_key(),
          detail::to_gtsam(T_e_w_off),
          gtsam::noiseModel::Diagonal::Sigmas(frame_sigmas));
    }
    for (KeyId k = 1; k < kKeyframes; ++k) {
      graph.push_back(relative_factor_of(
          cfg.relative_factor,
          detail::pose_key(k - 1),
          detail::pose_key(k),
          s.edge_meas[k - 1],
          gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(s.edge_cov))));
      const common::GnssMeasurement& m = fixes[k - 1];
      graph.emplace_shared<detail::GnssFrameFactor>(
          detail::pose_key(k), detail::frame_key(), m.p_e_g, m.cov, p_b_g);
    }
    return graph;
  };
  gtsam::Values init;
  for (KeyId k = 0; k < kKeyframes; ++k) {
    init.insert(detail::pose_key(k), detail::to_gtsam(s.guess[k].T_w_b));
  }
  init.insert(detail::vel_key(0), gtsam::Vector3(s.guess[0].v_w_b));
  init.insert(detail::bias_key(0), detail::to_gtsam(s.guess[0].bias));
  init.insert(detail::frame_key(), detail::to_gtsam(T_e_w_off));

  const gtsam::NonlinearFactorGraph exact = build(true);
  gtsam::LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-15;
  params.absoluteErrorTol = 1e-15;
  params.maxIterations = 100;
  gtsam::Values batch = gtsam::LevenbergMarquardtOptimizer(exact, init, params).optimize();
  for (int it = 0; it < kB04GaussNewtonIterations; ++it) {
    batch = batch.retract(exact.linearize(batch)->optimize());
  }
  const double true_grad_batch = cost_gradient_norm(exact, batch);
  RecordProperty("true_gradient_batch", sci(true_grad_batch));
  ASSERT_LT(true_grad_batch, 1e-5);  // the batch minimum of the true cost

  gtsam::Values isam = batch;  // the iSAM2 estimate, as Values of the batch graph
  double worst = 0.0;
  for (KeyId k = 0; k < kKeyframes; ++k) {
    const NavState x = state_or_fail(g, k);
    isam.update(detail::pose_key(k), detail::to_gtsam(x.T_w_b));
    const Pose T_batch = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::pose_key(k)));
    worst = std::max(worst, common::se3_log(T_batch.inverse() * x.T_w_b).norm());
  }
  isam.update(detail::frame_key(), detail::to_gtsam(E.value()));
  const Pose E_batch = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::frame_key()));
  worst = std::max(worst, common::se3_log(E_batch.inverse() * E.value()).norm());
  const double true_grad_isam = cost_gradient_norm(exact, isam);
  RecordProperty("worst_difference", sci(worst));
  RecordProperty("true_gradient_isam", sci(true_grad_isam));
  EXPECT_LT(worst, 1e-6);
  EXPECT_LT(true_grad_isam, 1e-5);

  // Control: with gtsam::PriorFactor<Pose3> on E the cost is the same, but the Gauss-Newton
  // fixed point of its linearisation (what iSAM2 converges to) fails the 1e-6 limit.
  const gtsam::NonlinearFactorGraph approx = build(false);
  EXPECT_NEAR(approx.error(batch), exact.error(batch), 1e-9 * exact.error(batch));
  gtsam::Values gn = batch;
  for (int it = 0; it < 4 * kB04GaussNewtonIterations; ++it) {
    gn = gn.retract(approx.linearize(gn)->optimize());
  }
  double control_diff = 0.0;
  for (KeyId k = 0; k < kKeyframes; ++k) {
    const Pose a = detail::from_gtsam(gn.at<gtsam::Pose3>(detail::pose_key(k)));
    const Pose b = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::pose_key(k)));
    control_diff = std::max(control_diff, common::se3_log(a.inverse() * b).norm());
  }
  const Pose E_gn = detail::from_gtsam(gn.at<gtsam::Pose3>(detail::frame_key()));
  control_diff = std::max(control_diff, common::se3_log(E_gn.inverse() * E_batch).norm());
  const double control_grad = cost_gradient_norm(exact, gn);
  RecordProperty("control_prior_factor_difference", sci(control_diff));
  RecordProperty("control_prior_factor_true_gradient", sci(control_grad));
  EXPECT_GT(control_diff, 1e-6);  // what iSAM2 converges to with it fails the 1e-6 limit
}

// Re-anchor site of the exact pose prior (backend_factors.md sections 2b and 9). Keyframe 1
// fails to optimise (a GNSS fix alone: indeterminant), so the island 1..4 added after it gets
// the re-anchor prior on X(1), centred on its guess: the truth moved by 20 deg of yaw and 3 m.
// Noise-free LiDAR edges and GNSS fixes from the truth on keyframes 2..4 pull the island back,
// so X(1) settles far from its prior. iSAM2 must reach the batch minimum of the true cost.
// Control: the same batch graph with gtsam::PriorFactor<Pose3> as the re-anchor prior (same
// cost, identity Jacobian) has a Gauss-Newton fixed point outside the 1e-6 limit.
TEST(BackendIsam, ReanchorPriorMatchesBatchAtLargeResidual) {
  constexpr KeyId kKeyframes = 5;
  const test::Scenario s = test::square_loop(false, 0);
  BackendConfig cfg = b04_config();
  cfg.prior_rot_sigma_rad = 10.0 * EIGEN_PI / 180.0;  // a loose prior: the fixes dominate
  cfg.prior_pos_sigma_m = 1.0;
  const Mat3 fix_cov = Mat3::Identity() * 0.01;  // m^2
  const Pose T_off = test::pose_from(Vec3(0.0, 0.0, 20.0 * EIGEN_PI / 180.0), Vec3(2.4, -1.8, 0.0));
  std::vector<NavState> guess = s.truth;
  for (KeyId k = 1; k < kKeyframes; ++k) {
    guess[k].T_w_b = T_off * s.truth[k].T_w_b;  // the island rigidly moved
  }
  const auto fix_at = [&](KeyId k) {
    common::GnssMeasurement m;
    m.t = s.truth[k].t;
    m.p_e_g = s.truth[k].T_w_b.p;  // T_e_w = identity, lever arm 0
    m.cov = fix_cov;
    return m;
  };

  Graph g(cfg, default_imu_config(), Vec3::Zero());
  g.add_keyframe(s.truth[0].t, s.truth[0]);
  ASSERT_TRUE(g.optimize().is_ok());
  g.add_keyframe(s.truth[1].t, guess[1]);
  g.add_gnss_factor(1, fix_at(1), Pose{});  // 3 of 6 DoF: indeterminant
  {
    CapturingLogger log;
    ASSERT_FALSE(g.optimize().is_ok());
  }
  {
    CapturingLogger log;
    for (KeyId k = 2; k < kKeyframes; ++k) {
      g.add_keyframe(s.truth[k].t, guess[k]);
      g.add_lidar_factor(k - 1, k, s.edge_meas[k - 1], s.edge_cov, nullptr);
      g.add_gnss_factor(k, fix_at(k), Pose{});
    }
    for (int n = 0; n < 20; ++n) {
      ASSERT_TRUE(g.optimize().is_ok());
    }
    ASSERT_EQ(log.warnings.size(), 1U);  // the one re-anchor line
    EXPECT_NE(log.warnings.front().find("re-anchored"), std::string::npos);
  }

  // The island's batch graph, as Graph builds it (section 9 re-anchor prior with the
  // first-keyframe sigmas, section 5 relative factors, section 6 GNSS factors).
  Vec6 pose_sigmas;
  pose_sigmas << Vec3::Constant(cfg.prior_rot_sigma_rad), Vec3::Constant(cfg.prior_pos_sigma_m);
  const auto build = [&](bool exact_prior) {
    gtsam::NonlinearFactorGraph graph;
    if (exact_prior) {
      graph.emplace_shared<detail::PosePriorFactor>(
          detail::pose_key(1), guess[1].T_w_b, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
    } else {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          detail::pose_key(1),
          detail::to_gtsam(guess[1].T_w_b),
          gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
    }
    for (KeyId k = 2; k < kKeyframes; ++k) {
      graph.push_back(relative_factor_of(
          cfg.relative_factor,
          detail::pose_key(k - 1),
          detail::pose_key(k),
          s.edge_meas[k - 1],
          gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(s.edge_cov))));
      graph.emplace_shared<detail::GnssPositionFactor>(
          detail::pose_key(k), fix_at(k).p_e_g, fix_cov, Pose{}, Vec3::Zero());
    }
    return graph;
  };
  gtsam::Values init;
  for (KeyId k = 1; k < kKeyframes; ++k) {
    init.insert(detail::pose_key(k), detail::to_gtsam(guess[k].T_w_b));
  }
  const gtsam::NonlinearFactorGraph exact = build(true);
  gtsam::LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-15;
  params.absoluteErrorTol = 1e-15;
  params.maxIterations = 100;
  gtsam::Values batch = gtsam::LevenbergMarquardtOptimizer(exact, init, params).optimize();
  for (int it = 0; it < kB04GaussNewtonIterations; ++it) {
    batch = batch.retract(exact.linearize(batch)->optimize());
  }
  const double true_grad_batch = cost_gradient_norm(exact, batch);
  RecordProperty("true_gradient_batch", sci(true_grad_batch));
  ASSERT_LT(true_grad_batch, 1e-5);  // the batch minimum of the true cost

  const Pose X1 = state_or_fail(g, 1).T_w_b;
  const Vec6 r_prior = common::se3_log(guess[1].T_w_b.inverse() * X1);  // [rho, phi]
  RecordProperty("reanchor_prior_residual_deg", sci(r_prior.tail<3>().norm() * 180.0 / EIGEN_PI));
  RecordProperty("reanchor_prior_residual_m", sci(r_prior.head<3>().norm()));
  EXPECT_GT(r_prior.tail<3>().norm(), 10.0 * EIGEN_PI / 180.0);  // the large-residual regime
  EXPECT_GT(r_prior.head<3>().norm(), 1.0);

  gtsam::Values isam = batch;  // the iSAM2 estimate, as Values of the batch graph
  double worst = 0.0;
  for (KeyId k = 1; k < kKeyframes; ++k) {
    const NavState x = state_or_fail(g, k);
    isam.update(detail::pose_key(k), detail::to_gtsam(x.T_w_b));
    const Pose T_batch = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::pose_key(k)));
    worst = std::max(worst, common::se3_log(T_batch.inverse() * x.T_w_b).norm());
  }
  const double true_grad_isam = cost_gradient_norm(exact, isam);
  RecordProperty("worst_difference", sci(worst));
  RecordProperty("true_gradient_isam", sci(true_grad_isam));
  EXPECT_LT(worst, 1e-6);
  EXPECT_LT(true_grad_isam, 1e-5);

  // Control: with gtsam::PriorFactor<Pose3> the cost is the same, but the Gauss-Newton fixed
  // point of its linearisation (what iSAM2 converges to) fails the 1e-6 limit.
  const gtsam::NonlinearFactorGraph approx = build(false);
  EXPECT_NEAR(approx.error(batch), exact.error(batch), 1e-9 * exact.error(batch));
  gtsam::Values gn = batch;
  for (int it = 0; it < 4 * kB04GaussNewtonIterations; ++it) {
    gn = gn.retract(approx.linearize(gn)->optimize());
  }
  double control_diff = 0.0;
  for (KeyId k = 1; k < kKeyframes; ++k) {
    const Pose a = detail::from_gtsam(gn.at<gtsam::Pose3>(detail::pose_key(k)));
    const Pose b = detail::from_gtsam(batch.at<gtsam::Pose3>(detail::pose_key(k)));
    control_diff = std::max(control_diff, common::se3_log(a.inverse() * b).norm());
  }
  RecordProperty("control_prior_factor_difference", sci(control_diff));
  EXPECT_GT(control_diff, 1e-6);
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
