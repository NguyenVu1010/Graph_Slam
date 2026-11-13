#include "mss2d/frontend.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "frontend_sim.hpp"

namespace mss2d {
namespace {

using sim::Pose2d;

constexpr int kRpeDeltaScans = 10;  // about 1.8 s at 5.5 Hz

Config config_for(const sim::Scenario& sc) {
  Config cfg;
  cfg.scan_rate_hz = static_cast<float>(sc.lidar.rate_hz);
  cfg.lidar.x_m = static_cast<float>(sc.lidar.T_b_l.x);
  cfg.lidar.y_m = static_cast<float>(sc.lidar.T_b_l.y);
  cfg.lidar.yaw_rad = static_cast<float>(sc.lidar.T_b_l.yaw);
  return cfg;
}

sim::Metrics run_and_report(const char* name,
                            const sim::Scenario& sc,
                            const Config& cfg,
                            sim::RunResult* out = nullptr) {
  EXPECT_TRUE(cfg.validate().is_ok()) << cfg.validate().reason();
  sim::RunResult r = sim::run_frontend(sc, cfg);
  const sim::Metrics m = sim::evaluate(sim::poses_of(r.est), r.truth, kRpeDeltaScans);
  std::printf(
      "[ %-10s ] %zu scans (ok %u, degenerate %u, predicted %u): ATE %.4f m (max %.4f), "
      "RPE %.4f m %.3f deg, gyro bias %.4f rad/s\n",
      name,
      r.est.size(),
      r.stats.n_ok,
      r.stats.n_degenerate,
      r.stats.n_predicted,
      m.ate_rmse_m,
      m.ate_max_m,
      m.rpe_trans_rmse_m,
      m.rpe_rot_rmse_deg,
      static_cast<double>(r.gyro_bias_rad_s));
  if (out != nullptr) {
    *out = std::move(r);
  }
  return m;
}

// --- metrics ---

TEST(FrontEndEval, MetricsAreZeroForARigidlyMovedCopyAndSeeAnOffset) {
  std::vector<Pose2d> truth;
  for (int i = 0; i < 30; ++i) {
    truth.emplace_back(0.1 * i, 0.02 * i * i, 0.05 * i);
  }
  const Pose2d move(3.0, -2.0, 1.1);
  std::vector<Pose2d> est;
  for (const Pose2d& T : truth) {
    est.push_back(move * T);
  }
  const sim::Metrics m = sim::evaluate(est, truth, 5);
  EXPECT_LT(m.ate_rmse_m, 1e-9);
  EXPECT_LT(m.rpe_trans_rmse_m, 1e-9);
  EXPECT_LT(m.rpe_rot_rmse_deg, 1e-9);

  // Every pose but the first shifted by 0.1 m in the world: ATE sqrt(29/30) * 0.1.
  for (std::size_t i = 1; i < est.size(); ++i) {
    est[i] = Pose2d(truth[i].x + 0.1, truth[i].y, truth[i].yaw);
  }
  est[0] = truth[0];
  EXPECT_NEAR(sim::evaluate(est, truth, 5).ate_rmse_m, 0.1 * std::sqrt(29.0 / 30.0), 1e-9);
}

TEST(FrontEndEval, LoopTrajectoryClosesAndLidarTimingIsRotatingMirrorLike) {
  const sim::Scenario sc = sim::loop_scenario();
  const Pose2d a = sc.traj.at(0.0).T_w_b;
  const Pose2d b = sc.traj.at(sc.traj.duration_s()).T_w_b;
  EXPECT_NEAR(a.x, b.x, 1e-9);
  EXPECT_NEAR(a.y, b.y, 1e-9);
  EXPECT_NEAR(normalize_angle(a.yaw - b.yaw), 0.0, 1e-9);

  sim::SyntheticLidar lidar(sc.world, sc.traj, sc.lidar, 1);
  auto scan = std::make_unique<Scan2d>();
  EXPECT_FALSE(lidar.next_scan(*scan));  // clock at 0: the first turn is not complete
  lidar.set_time(Timestamp{1000000000});
  ASSERT_TRUE(lidar.next_scan(*scan));
  const std::int64_t period_ns = 181818182;  // 5.5 Hz
  EXPECT_EQ(scan->t_start.ns, 0);
  EXPECT_EQ(scan->count, 720U);
  EXPECT_EQ(scan->points[360].t_offset_ns, period_ns / 2);
  EXPECT_EQ(scan->key().ns, 719 * period_ns / 720);
  ASSERT_TRUE(lidar.next_scan(*scan));
  EXPECT_EQ(scan->t_start.ns, period_ns);
}

// --- closed loop on the synthetic ring corridor ---

TEST(FrontEndLoop, DeskewedOdometryIsAccurateAndDeskewMeasurablyHelps) {
  const sim::Scenario sc = sim::loop_scenario();
  Config cfg = config_for(sc);
  sim::RunResult r;
  const sim::Metrics with = run_and_report("deskew", sc, cfg, &r);
  cfg.motion.deskew = false;
  const sim::Metrics without = run_and_report("no deskew", sc, cfg);

  // Measured (seeds 1-5): ATE 0.053-0.084 m, RPE 0.009-0.011 m and 0.07-0.10 deg with
  // deskew; ATE 0.15-0.25 m and RPE 2.2-2.4 deg without.
  EXPECT_EQ(r.stats.n_predicted, 0U);
  EXPECT_LT(with.ate_rmse_m, 0.15);
  EXPECT_LT(with.rpe_trans_rmse_m, 0.025);
  EXPECT_LT(with.rpe_rot_rmse_deg, 0.25);
  EXPECT_GT(without.ate_rmse_m, 1.5 * with.ate_rmse_m);
  EXPECT_GT(without.rpe_rot_rmse_deg, 1.0);
  // The gyro bias (0.01 rad/s in the simulation) is learnt from the matched yaw.
  EXPECT_NEAR(r.gyro_bias_rad_s, sc.gyro_bias_rad_s, 0.003);
  // Submap pool: one or two active (after a finish by distance the newer one may still be
  // below half), the rest finished; once the pool is full, every new submap recycles the
  // oldest finished one.
  const std::uint32_t n_active = r.stats.n_submaps_started - r.stats.n_submaps_finished;
  EXPECT_GT(r.stats.n_submaps_started, static_cast<std::uint32_t>(kSubmapPoolSize));
  EXPECT_TRUE(n_active == 1U || n_active == 2U) << n_active;
  EXPECT_EQ(r.stats.n_submaps_recycled,
            r.stats.n_submaps_started - static_cast<std::uint32_t>(kSubmapPoolSize));
}

TEST(FrontEndLoop, WithoutWheelOdometryConstantVelocityPredictionHolds) {
  sim::Scenario sc = sim::loop_scenario();
  sc.wheel = false;
  Config cfg = config_for(sc);
  // Without the wheel, a direction flagged degenerate follows constant velocity only; the
  // corridors of this world are weakly but truly constrained (end walls), so flag less.
  cfg.matcher.degeneracy_eig_threshold = 10.0F;
  const sim::Metrics m = run_and_report("no wheel", sc, cfg);
  // Measured (seeds 1-5): ATE 0.03-0.08 m.
  EXPECT_LT(m.ate_rmse_m, 0.2);
  EXPECT_LT(m.rpe_rot_rmse_deg, 0.25);
}

TEST(FrontEndLoop, RunsAreBitForBitRepeatable) {
  const sim::Scenario sc = sim::corridor_scenario();
  const Config cfg = config_for(sc);
  const sim::RunResult a = sim::run_frontend(sc, cfg);
  const sim::RunResult b = sim::run_frontend(sc, cfg);
  ASSERT_EQ(a.est.size(), b.est.size());
  for (std::size_t i = 0; i < a.est.size(); ++i) {
    EXPECT_EQ(a.est[i].T_w_b.x, b.est[i].T_w_b.x);
    EXPECT_EQ(a.est[i].T_w_b.y, b.est[i].T_w_b.y);
    EXPECT_EQ(a.est[i].T_w_b.yaw, b.est[i].T_w_b.yaw);
  }
}

// --- degeneracy fallback ---

TEST(FrontEndCorridor, KeepsTheWheelPredictionAlongTheCorridorAxis) {
  const sim::Scenario sc = sim::corridor_scenario();
  sim::RunResult r;
  const sim::Metrics m = run_and_report("corridor", sc, config_for(sc), &r);
  // The featureless corridor is flagged; x comes from the wheel, y and yaw from the walls.
  EXPECT_GT(r.stats.n_degenerate, static_cast<std::uint32_t>(r.est.size() / 2));
  const Pose2d align =
      r.truth[0] * Pose2d(r.est[0].T_w_b.x, r.est[0].T_w_b.y, r.est[0].T_w_b.yaw).inverse();
  const Pose2d last =
      align * Pose2d(r.est.back().T_w_b.x, r.est.back().T_w_b.y, r.est.back().T_w_b.yaw);
  std::printf("[ corridor   ] final error x %.4f m, y %.4f m over %.2f m\n",
              last.x - r.truth.back().x,
              last.y - r.truth.back().y,
              r.truth.back().x - r.truth[0].x);
  EXPECT_NEAR(last.x, r.truth.back().x, 0.05);
  EXPECT_NEAR(last.y, r.truth.back().y, 0.05);
  EXPECT_LT(m.ate_rmse_m, 0.05);

  // With the flag off, the scan locks onto its own map along the axis and x stalls.
  Config off = config_for(sc);
  off.matcher.degeneracy_eig_threshold = 0.0F;
  const sim::Metrics m_off = run_and_report("corr noflag", sc, off);
  EXPECT_GT(m_off.ate_rmse_m, 10.0 * m.ate_rmse_m);
}

// --- submap rotation and status on a standing robot ---

class CountingPoses final : public PoseSink {
 public:
  void on_pose(const ScanPose& p) override { poses.push_back(p); }
  std::vector<ScanPose> poses;
};

TEST(FrontEndSubmaps, RotateEveryHalfLimitAndKeepTheNewestFinished) {
  Config cfg;
  cfg.submap.scans_per_submap = 4;
  ASSERT_TRUE(cfg.validate().is_ok());
  CountingPoses sink;
  auto fe = std::make_unique<FrontEnd>(cfg, &sink);
  auto scan = std::make_unique<Scan2d>();
  const sim::World world = sim::room_world();
  const std::int64_t period = 100000000;  // 10 Hz
  for (int k = 0; k < 10; ++k) {
    sim::make_scan(world, Pose2d(0.2, -0.1, 0.3), 720, 12.0, 0.0, 1, *scan);
    scan->t_start = Timestamp{k * period};
    for (std::uint32_t i = 0; i < scan->count; ++i) {
      scan->points[i].t_offset_ns = static_cast<std::int32_t>(i * (period / 720));
    }
    for (int j = 0; j < 20; ++j) {  // standing still, zero gyro
      ImuSample2d s;
      s.t = Timestamp{k * period + j * 5000000};
      fe->on_imu(s);
    }
    fe->on_scan(*scan);
  }
  ASSERT_EQ(sink.poses.size(), 10U);
  EXPECT_EQ(sink.poses[0].status, FrontEndStatus::kInitialised);
  for (std::size_t i = 1; i < sink.poses.size(); ++i) {
    EXPECT_EQ(sink.poses[i].status, FrontEndStatus::kOk) << i;
    EXPECT_NEAR(sink.poses[i].T_w_b.x, 0.0F, 0.01F);
    EXPECT_NEAR(sink.poses[i].T_w_b.y, 0.0F, 0.01F);
    EXPECT_NEAR(sink.poses[i].T_w_b.yaw, 0.0F, 0.005F);
  }
  // A new submap every 2 scans (half of 4): started at scans 1, 2, 4, 6, 8, 10.
  const FrontEndStats& st = fe->stats();
  EXPECT_EQ(st.n_submaps_started, 6U);
  EXPECT_EQ(st.n_submaps_finished, 4U);
  EXPECT_EQ(st.n_submaps_recycled, 2U);
  EXPECT_EQ(fe->n_finished_submaps(), kSubmapPoolSize - 2);
  // Submap ids grow; the matched one is the older active submap.
  EXPECT_GT(sink.poses.back().submap_id, sink.poses[1].submap_id);
}

TEST(FrontEndStatus, TooFewPointsFallsBackToThePrediction) {
  Config cfg;
  CountingPoses sink;
  auto fe = std::make_unique<FrontEnd>(cfg, &sink);
  auto scan = std::make_unique<Scan2d>();
  sim::make_scan(sim::room_world(), Pose2d(), 720, 12.0, 0.0, 1, *scan);
  scan->t_start = Timestamp{0};
  fe->on_scan(*scan);
  scan->count = 10;  // below matcher.min_points
  scan->t_start = Timestamp{100000000};
  fe->on_scan(*scan);
  ASSERT_EQ(sink.poses.size(), 2U);
  EXPECT_EQ(sink.poses[1].status, FrontEndStatus::kPredicted);
  EXPECT_EQ(fe->stats().n_predicted, 1U);
  // An old or repeated scan is ignored.
  fe->on_scan(*scan);
  EXPECT_EQ(sink.poses.size(), 2U);
}

}  // namespace
}  // namespace mss2d
