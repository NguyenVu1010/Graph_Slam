#include "mss2d/config.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "mss2d/messages.hpp"

namespace mss2d {
namespace {

bool names(const Status& s, const char* key) {
  return !s.is_ok() && std::strstr(s.reason(), key) != nullptr;
}

TEST(Config, DefaultsAreValid) {
  EXPECT_TRUE(Config{}.validate().is_ok());
}

TEST(Config, DepthsAboveCapacityOrZeroNameTheKey) {
  Config c;
  c.scan_ring_depth = kScanRingCapacity + 1;
  EXPECT_TRUE(names(c.validate(), "scan_ring_depth"));
  c = Config{};
  c.scan_ring_depth = 0;
  EXPECT_TRUE(names(c.validate(), "scan_ring_depth"));
  c = Config{};
  c.imu_ring_depth = kImuRingCapacity + 1;
  EXPECT_TRUE(names(c.validate(), "imu_ring_depth"));
  c = Config{};
  c.wheel_ring_depth = kWheelRingCapacity + 1;
  EXPECT_TRUE(names(c.validate(), "wheel_ring_depth"));
}

TEST(Config, RatesAndBuffer) {
  Config c;
  c.imu_rate_hz = 0.0F;
  EXPECT_TRUE(names(c.validate(), "imu_rate_hz"));
  c = Config{};
  c.imu_rate_hz = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(names(c.validate(), "imu_rate_hz"));
  c = Config{};
  c.scan_rate_hz = 100.0F;
  EXPECT_TRUE(names(c.validate(), "scan_rate_hz"));
  c = Config{};
  c.imu_buffer_s = -1.0F;
  EXPECT_TRUE(names(c.validate(), "imu_buffer_s"));
}

TEST(Config, ImuRingMustHoldTheBuffer) {
  Config c;
  c.imu_ring_depth = 399;  // 200 Hz * 2 s = 400 samples
  EXPECT_TRUE(names(c.validate(), "imu_ring_depth"));
  c.imu_ring_depth = 400;
  EXPECT_TRUE(c.validate().is_ok());
}

TEST(Config, MaxScanPoints) {
  Config c;
  c.max_scan_points = 0;
  EXPECT_TRUE(names(c.validate(), "max_scan_points"));
  c.max_scan_points = kMaxScanPoints + 1;
  EXPECT_TRUE(names(c.validate(), "max_scan_points"));
}

TEST(Config, MotionKeys) {
  Config c;
  c.motion.integration_step_s = 0.0F;
  EXPECT_TRUE(names(c.validate(), "motion.integration_step_s"));
  c = Config{};
  c.motion.wheel_timeout_s = 2.0F;
  EXPECT_TRUE(names(c.validate(), "motion.wheel_timeout_s"));
  c = Config{};
  c.motion.gyro_bias_rad_s = 0.3F;  // beyond max_gyro_bias_rad_s
  EXPECT_TRUE(names(c.validate(), "motion.gyro_bias_rad_s"));
  c = Config{};
  c.motion.gyro_bias_gain = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(names(c.validate(), "motion.gyro_bias_gain"));
  c = Config{};
  c.motion.deskew_knots = kMaxDeskewKnots + 1;
  EXPECT_TRUE(names(c.validate(), "motion.deskew_knots"));
  c = Config{};
  c.imu_rate_hz = 1000.0F;  // 2 turns at 10 Hz = 200 samples: fits
  c.imu_ring_depth = kImuRingCapacity;
  c.imu_buffer_s = 0.5F;
  EXPECT_TRUE(c.validate().is_ok());
  c.scan_rate_hz = 5.0F;  // 400 samples > kMotionImuHistory
  EXPECT_TRUE(names(c.validate(), "kMotionImuHistory"));
}

TEST(Config, SubmapAndLidarKeys) {
  Config c;
  c.submap.scans_per_submap = 1;
  EXPECT_TRUE(names(c.validate(), "submap.scans_per_submap"));
  c = Config{};
  c.submap.max_distance_m = 4.6F;  // grid half side 5 m minus the 0.5 m field clamp
  EXPECT_TRUE(names(c.validate(), "submap.max_distance_m"));
  c.submap.max_distance_m = 4.4F;
  EXPECT_TRUE(c.validate().is_ok());
  c = Config{};
  c.lidar.x_m = 3.0F;
  EXPECT_TRUE(names(c.validate(), "lidar.x_m"));
  c = Config{};
  c.lidar.yaw_rad = 4.0F;
  EXPECT_TRUE(names(c.validate(), "lidar.yaw_rad"));
}

TEST(Scan2d, KeyIsStartPlusMaxOffset) {
  Scan2d s;
  s.t_start = Timestamp{1000};
  s.count = 3;
  s.points[0].t_offset_ns = 5;
  s.points[1].t_offset_ns = 90;
  s.points[2].t_offset_ns = 40;
  s.points[3].t_offset_ns = 9999;  // beyond count: ignored
  EXPECT_EQ(s.key().ns, 1090);
}

}  // namespace
}  // namespace mss2d
