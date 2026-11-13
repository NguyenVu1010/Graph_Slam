#include "mss2d/motion.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

#include <gtest/gtest.h>

#include "frontend_sim.hpp"

namespace mss2d {
namespace {

constexpr std::int64_t kMs = 1000000;  // ns

void feed(MotionModel& m, double gyro_rad_s, double v_m_s, std::int64_t until_ns) {
  for (std::int64_t t = 0; t <= until_ns; t += 5 * kMs) {  // 200 Hz
    ImuSample2d s;
    s.t = Timestamp{t};
    s.gyro_z_rad_s = static_cast<float>(gyro_rad_s);
    m.add_imu(s);
  }
  for (std::int64_t t = 0; t <= until_ns; t += 20 * kMs) {  // 50 Hz
    WheelSample w;
    w.t = Timestamp{t};
    w.v_m_s = static_cast<float>(v_m_s);
    m.add_wheel(w);
  }
}

TEST(MotionModel, IntegratesTheBiasCorrectedConstantTwist) {
  MotionConfig cfg;
  cfg.gyro_bias_rad_s = 0.1F;
  MotionModel m(cfg);
  feed(m, 0.6, 0.4, 600 * kMs);  // true yaw rate 0.5 rad/s
  const Pose2 T = m.integrate(Timestamp{50 * kMs}, Timestamp{550 * kMs});
  Vec3f d;
  d[0] = 0.4F * 0.5F;
  d[2] = 0.5F * 0.5F;
  const Pose2 expected = Pose2::exp(d);  // exact for a constant body twist
  EXPECT_NEAR(T.x, expected.x, 1e-5F);
  EXPECT_NEAR(T.y, expected.y, 1e-5F);
  EXPECT_NEAR(T.yaw, expected.yaw, 1e-5F);
  // Reversed or empty interval: identity.
  EXPECT_EQ(m.integrate(Timestamp{550 * kMs}, Timestamp{50 * kMs}).x, 0.0F);
}

TEST(MotionModel, YawRateInterpolatesAndUsesTheNearestSampleOutside) {
  MotionModel m(MotionConfig{});
  ImuSample2d a;
  a.t = Timestamp{10 * kMs};
  a.gyro_z_rad_s = 1.0F;
  ImuSample2d b = a;
  b.t = Timestamp{20 * kMs};
  b.gyro_z_rad_s = 2.0F;
  m.add_imu(a);
  m.add_imu(b);
  m.add_imu(a);  // older than the newest: ignored
  EXPECT_FLOAT_EQ(m.yaw_rate(Timestamp{15 * kMs}), 1.5F);
  EXPECT_FLOAT_EQ(m.yaw_rate(Timestamp{0}), 1.0F);
  EXPECT_FLOAT_EQ(m.yaw_rate(Timestamp{99 * kMs}), 2.0F);
}

TEST(MotionModel, FallsBackToConstantVelocityWithoutARecentWheelSample) {
  MotionConfig cfg;
  MotionModel m(cfg);
  m.set_constant_velocity(0.3F, -0.1F, 0.0F);
  feed(m, 0.0, 0.8, 100 * kMs);
  float vx = 0.0F;
  float vy = 0.0F;
  EXPECT_TRUE(m.velocity(Timestamp{150 * kMs}, vx, vy));  // 50 ms after the last sample
  EXPECT_FLOAT_EQ(vx, 0.8F);
  EXPECT_FALSE(m.velocity(Timestamp{300 * kMs}, vx, vy));  // beyond wheel_timeout_s
  EXPECT_FLOAT_EQ(vx, 0.3F);
  EXPECT_FLOAT_EQ(vy, -0.1F);

  cfg.use_wheel = false;
  MotionModel no_wheel(cfg);
  no_wheel.set_constant_velocity(0.3F, -0.1F, 0.0F);
  feed(no_wheel, 0.0, 0.8, 100 * kMs);
  EXPECT_FALSE(no_wheel.velocity(Timestamp{50 * kMs}, vx, vy));
  EXPECT_FLOAT_EQ(vx, 0.3F);
}

TEST(MotionModel, GyroBiasIsClamped) {
  MotionConfig cfg;
  cfg.max_gyro_bias_rad_s = 0.05F;
  MotionModel m(cfg);
  m.set_gyro_bias(0.2F);
  EXPECT_FLOAT_EQ(m.gyro_bias(), 0.05F);
  m.set_gyro_bias(-0.2F);
  EXPECT_FLOAT_EQ(m.gyro_bias(), -0.05F);
}

// Known answer: a scan taken while turning at 1 rad/s and driving at 0.5 m/s, with the LiDAR
// off the body origin. Each beam's true world point is T_w_l(t_i) p_i; after deskew the same
// point seen from the scan-end LiDAR pose, T_w_l(t_end) p_end, must coincide with it.
TEST(Deskew, MovesEveryBeamIntoTheScanEndFrame) {
  using sim::Pose2d;
  const sim::World world = sim::room_world();
  sim::Trajectory traj(Pose2d(-1.0, -0.5, 0.2), 0.5);
  traj.arc(1.5, 0.5);  // w = v / r = 1 rad/s
  sim::LidarModel lm;
  lm.sigma_m = 0.0;
  lm.T_b_l = Pose2d(0.15, 0.05, 0.3);
  sim::SyntheticLidar lidar(world, traj, lm, 1);
  auto scan = std::make_unique<Scan2d>();
  lidar.set_time(Timestamp{1000 * kMs});
  ASSERT_TRUE(lidar.next_scan(*scan));

  MotionModel m(MotionConfig{});
  for (std::int64_t t = 0; t <= scan->key().ns + 10 * kMs; t += 5 * kMs) {
    const sim::Trajectory::State st = traj.at(static_cast<double>(t) * 1e-9);
    ImuSample2d s;
    s.t = Timestamp{t};
    s.gyro_z_rad_s = static_cast<float>(st.w_rad_s);
    m.add_imu(s);
    WheelSample w;
    w.t = Timestamp{t};
    w.v_m_s = static_cast<float>(st.v_m_s);
    m.add_wheel(w);
  }
  const Pose2 T_b_l(0.15F, 0.05F, 0.3F);
  auto table = std::make_unique<DeskewTable>();
  table->build(m, scan->t_start, scan->key(), MotionConfig{}.deskew_knots, T_b_l);
  auto pts = std::make_unique<PointCloud2d>();
  deskew_scan(*scan, *table, 0.0F, 100.0F, *pts);
  auto raw = std::make_unique<PointCloud2d>();
  scan_to_points(*scan, 0.0F, 100.0F, *raw);
  ASSERT_EQ(pts->count, raw->count);

  const Pose2d T_w_le = traj.at(static_cast<double>(scan->key().ns) * 1e-9).T_w_b * lm.T_b_l;
  double worst = 0.0;
  double worst_raw = 0.0;
  std::uint32_t k = 0;
  for (std::uint32_t i = 0; i < scan->count; ++i) {
    const ScanPoint2d& sp = scan->points[i];
    if (!(sp.range_m > 0.0F)) {
      continue;
    }
    const double t_i = static_cast<double>(scan->t_start.ns + sp.t_offset_ns) * 1e-9;
    const Pose2d T_w_li = traj.at(t_i).T_w_b * lm.T_b_l;
    const Vec2T<double> p_w =
        T_w_li * Vec2T<double>{sp.range_m * std::cos(static_cast<double>(sp.angle_rad)),
                               sp.range_m * std::sin(static_cast<double>(sp.angle_rad))};
    const Vec2T<double> q = T_w_le * Vec2T<double>{pts->pts[k].x, pts->pts[k].y};
    const Vec2T<double> q_raw = T_w_le * Vec2T<double>{raw->pts[k].x, raw->pts[k].y};
    worst = std::fmax(worst, std::hypot(q.x - p_w.x, q.y - p_w.y));
    worst_raw = std::fmax(worst_raw, std::hypot(q_raw.x - p_w.x, q_raw.y - p_w.y));
    ++k;
  }
  std::printf("[ deskew     ] worst point error %.4f m (no deskew %.4f m)\n", worst, worst_raw);
  EXPECT_LT(worst, 0.005);
  EXPECT_GT(worst_raw, 0.3);
}

TEST(Deskew, ZeroLengthScanIsTheIdentity) {
  MotionModel m(MotionConfig{});
  DeskewTable table;
  table.build(m, Timestamp{100}, Timestamp{100}, 8, Pose2(0.1F, 0.0F, 0.2F));
  const Pose2 T = table.at(50);
  EXPECT_EQ(T.x, 0.0F);
  EXPECT_EQ(T.y, 0.0F);
  EXPECT_EQ(T.yaw, 0.0F);
}

}  // namespace
}  // namespace mss2d
