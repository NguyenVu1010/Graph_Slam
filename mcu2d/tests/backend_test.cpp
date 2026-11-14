#include "mss2d/backend.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_sim.hpp"

namespace mss2d {
namespace {

using sim::Pose2d;

constexpr int kRpeDeltaScans = 10;

Config config_for(const sim::Scenario& sc) {
  Config cfg;
  cfg.scan_rate_hz = static_cast<float>(sc.lidar.rate_hz);
  cfg.lidar.x_m = static_cast<float>(sc.lidar.T_b_l.x);
  cfg.lidar.y_m = static_cast<float>(sc.lidar.T_b_l.y);
  cfg.lidar.yaw_rad = static_cast<float>(sc.lidar.T_b_l.yaw);
  return cfg;
}

void print(const char* name, const sim::Metrics& m) {
  std::printf("[ %-10s ] ATE %.4f m (max %.4f), RPE %.4f m %.3f deg\n",
              name,
              m.ate_rmse_m,
              m.ate_max_m,
              m.rpe_trans_rmse_m,
              m.rpe_rot_rmse_deg);
}

void print(const BackendStats& s) {
  std::printf(
      "[ backend    ] nodes %u, candidates %u, loops %u, rejected score %u degenerate %u "
      "window %u chi2 %u capacity %u, solves %u\n",
      s.n_nodes,
      s.n_candidates,
      s.n_loops,
      s.n_rejected_score,
      s.n_rejected_degenerate,
      s.n_rejected_window,
      s.n_rejected_chi2,
      s.n_rejected_capacity,
      s.n_solves);
}

TEST(BackendRevisit, LoopClosingRemovesTheCorridorDriftOfTheFrontEnd) {
  const sim::Scenario sc = sim::revisit_scenario();
  const Config cfg = config_for(sc);
  ASSERT_TRUE(cfg.validate().is_ok()) << cfg.validate().reason();
  const sim::RunResult fe = sim::run_frontend(sc, cfg);
  const sim::RunResult g = sim::run_frontend(sc, cfg, true);
  const sim::Metrics m_fe = sim::evaluate(sim::poses_of(fe.est), fe.truth, kRpeDeltaScans);
  const sim::Metrics m_g = sim::evaluate(g.optimized, g.truth, kRpeDeltaScans);
  print("front end", m_fe);
  print("graph", m_g);
  const BackendStats& s = g.backend->stats();
  print(s);

  // Measured (seeds 1-5): front end ATE 0.64-0.72 m, graph 0.10-0.19 m; seed 1: 0.717 m
  // against 0.104 m, RPE 0.023 m against 0.015 m.
  EXPECT_GT(m_fe.ate_rmse_m, 0.5);
  EXPECT_LT(m_g.ate_rmse_m, 0.2);
  EXPECT_LT(m_g.ate_rmse_m, 0.3 * m_fe.ate_rmse_m);
  EXPECT_LT(m_g.rpe_trans_rmse_m, m_fe.rpe_trans_rmse_m * 1.1);
  EXPECT_LT(m_g.rpe_rot_rmse_deg, 0.25);
  // The start corner closes the loop; candidates in the featureless corridor are refused.
  EXPECT_GE(s.n_loops, 3U);
  EXPECT_GT(s.n_rejected_degenerate, 0U);
  EXPECT_EQ(s.n_rejected_chi2, 0U);
  EXPECT_EQ(s.n_dropped_nodes, 0U);
  EXPECT_EQ(s.n_rejected_capacity, 0U);
  EXPECT_EQ(static_cast<std::uint32_t>(g.backend->graph().n_nodes()), s.n_nodes);
  EXPECT_EQ(g.solve_ms.size(), static_cast<std::size_t>(s.n_solves));
}

TEST(BackendRevisit, WithoutDegeneracyAwareOdometryTheLoopsLookInconsistent) {
  const sim::Scenario sc = sim::revisit_scenario();
  Config cfg = config_for(sc);
  const sim::RunResult with = sim::run_frontend(sc, cfg, true);
  cfg.graph.odom_degenerate_fraction = 0.0F;
  const sim::RunResult without = sim::run_frontend(sc, cfg, true);
  const sim::Metrics m_with = sim::evaluate(with.optimized, with.truth, kRpeDeltaScans);
  const sim::Metrics m_without = sim::evaluate(without.optimized, without.truth, kRpeDeltaScans);
  print("deg info", m_with);
  print(with.backend->stats());
  print("no deg", m_without);
  print(without.backend->stats());
  // Without it the odometry is too stiff along the corridor to absorb the 0.7 m the loops
  // ask for: their chi2 after the solve is large, they are dropped as inconsistent and the
  // estimate stays at the front end's (seed 1: ATE 0.717 m against 0.104 m).
  EXPECT_GT(without.backend->stats().n_rejected_chi2, 0U);
  EXPECT_EQ(with.backend->stats().n_rejected_chi2, 0U);
  EXPECT_LT(m_with.ate_rmse_m, 0.3 * m_without.ate_rmse_m);
}

TEST(BackendCorridor, AmbiguousCandidatesInASymmetricCorridorAreRejected) {
  // Every candidate of the featureless corridor (allowed from 1 m back) is degenerate along
  // the axis: no loop edge, so the graph keeps the front-end odometry.
  sim::Scenario sc = sim::corridor_scenario();
  sc.traj = sim::Trajectory(Pose2d(0.0, 0.0, 0.0), 0.5);
  sc.traj.wait(0.5);
  sc.traj.straight(12.0);
  Config cfg = config_for(sc);
  cfg.loop.min_path_m = 1.0F;
  cfg.loop.search_radius_m = 5.0F;  // the query scan is 2 m ahead of its submap origin
  ASSERT_TRUE(cfg.validate().is_ok());
  const sim::RunResult g = sim::run_frontend(sc, cfg, true);
  const BackendStats& s = g.backend->stats();
  print(s);
  EXPECT_GT(s.n_candidates, 0U);
  EXPECT_EQ(s.n_loops, 0U);
  EXPECT_EQ(s.n_rejected_degenerate + s.n_rejected_score, s.n_candidates);
  EXPECT_GT(s.n_rejected_degenerate, 0U);
  EXPECT_EQ(s.n_solves, 0U);
}

TEST(BackendLoop, ClosingTheWellConstrainedLoopDoesNotHurt) {
  const sim::Scenario sc = sim::loop_scenario();
  const sim::RunResult g = sim::run_frontend(sc, config_for(sc), true);
  const sim::Metrics m = sim::evaluate(g.optimized, g.truth, kRpeDeltaScans);
  print("loop graph", m);
  print(g.backend->stats());
  // Front end alone: ATE 0.064 m (seed 1); with the graph 0.058 m.
  EXPECT_GT(g.backend->stats().n_loops, 0U);
  EXPECT_LT(m.ate_rmse_m, 0.08);
  EXPECT_LT(m.rpe_rot_rmse_deg, 0.25);
}

// --- correction interface ---

class LastPose final : public PoseSink {
 public:
  void on_pose(const ScanPose& p) override { last = p; }
  ScanPose last;
};

TEST(FrontEndCorrection, MovesThePoseAndActiveSubmapsRigidly) {
  Config cfg;
  LastPose sink;
  auto fe = std::make_unique<FrontEnd>(cfg, &sink);
  auto scan = std::make_unique<Scan2d>();
  sim::make_scan(sim::room_world(), Pose2d(0.2, -0.1, 0.3), 720, 12.0, 0.0, 1, *scan);
  const std::int64_t period = 100000000;
  auto step = [&](int k) {
    scan->t_start = Timestamp{k * period};
    for (int j = 0; j < 20; ++j) {
      ImuSample2d s;
      s.t = Timestamp{k * period + j * 5000000};
      fe->on_imu(s);
    }
    fe->on_scan(*scan);
  };
  for (int k = 0; k < 3; ++k) {
    step(k);
  }
  Pose2 T_w_s_before;
  ASSERT_TRUE(fe->active_submap_pose(sink.last.submap_id, T_w_s_before));
  const Pose2 C(1.0F, -2.0F, 0.4F);
  fe->apply_correction(C);
  Pose2 T_w_s_after;
  ASSERT_TRUE(fe->active_submap_pose(sink.last.submap_id, T_w_s_after));
  const Pose2 expect_s = C * T_w_s_before;
  EXPECT_NEAR(T_w_s_after.x, expect_s.x, 1e-6F);
  EXPECT_NEAR(T_w_s_after.yaw, expect_s.yaw, 1e-6F);
  const Pose2 expect_b = C * sink.last.T_w_b;
  EXPECT_NEAR(fe->last_pose().T_w_b.x, expect_b.x, 1e-6F);
  EXPECT_NEAR(fe->last_pose().T_w_b.y, expect_b.y, 1e-6F);

  // The robot stands still: the next scan matches at the corrected pose, with the same pose
  // relative to its submap as before.
  const Pose2 T_s_b_before = sink.last.T_s_b;
  step(3);
  EXPECT_EQ(sink.last.status, FrontEndStatus::kOk);
  EXPECT_NEAR(sink.last.T_w_b.x, expect_b.x, 0.01F);
  EXPECT_NEAR(sink.last.T_w_b.y, expect_b.y, 0.01F);
  EXPECT_NEAR(normalize_angle(sink.last.T_w_b.yaw - expect_b.yaw), 0.0F, 0.005F);
  EXPECT_NEAR(sink.last.T_s_b.x, T_s_b_before.x, 0.01F);
  EXPECT_NEAR(sink.last.T_s_b.y, T_s_b_before.y, 0.01F);
}

}  // namespace
}  // namespace mss2d
