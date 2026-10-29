// Known-answer tests of the mss_node conversions (ADR-024).
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "mss_ros2/conversions.hpp"

namespace {

using mss::common::Timestamp;
using sensor_msgs::msg::PointField;

constexpr double kTol = 1e-12;
constexpr std::int32_t kSec = 1700000000;  // the sim bags start here
constexpr std::uint32_t kNsec = 123456789;

struct BridgePoint {
  float x, y, z, intensity;
  std::uint32_t t;
  std::uint16_t ring;
};

PointField field(const std::string& name, std::uint32_t offset, std::uint8_t type) {
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

// PointCloud2 laid out like sim/mss_sim/bag_io.py arrays_to_cloud (22 bytes per point),
// optionally with `pad` extra bytes per point and the fields listed in reverse order.
sensor_msgs::msg::PointCloud2 bridge_cloud(const std::vector<BridgePoint>& pts,
                                           std::uint32_t pad = 0,
                                           bool reversed = false) {
  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp.sec = kSec;
  m.header.stamp.nanosec = kNsec;
  m.header.frame_id = "lidar_link";
  m.height = 1;
  m.width = static_cast<std::uint32_t>(pts.size());
  m.fields = {field("x", 0, PointField::FLOAT32),
              field("y", 4, PointField::FLOAT32),
              field("z", 8, PointField::FLOAT32),
              field("intensity", 12, PointField::FLOAT32),
              field("t", 16, PointField::UINT32),
              field("ring", 20, PointField::UINT16)};
  if (reversed) {
    m.fields = std::vector<PointField>(m.fields.rbegin(), m.fields.rend());
  }
  m.is_bigendian = false;
  m.point_step = 22 + pad;
  m.row_step = m.point_step * m.width;
  m.data.assign(static_cast<std::size_t>(m.row_step), 0);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::uint8_t* p = m.data.data() + i * m.point_step;
    std::memcpy(p + 0, &pts[i].x, 4);
    std::memcpy(p + 4, &pts[i].y, 4);
    std::memcpy(p + 8, &pts[i].z, 4);
    std::memcpy(p + 12, &pts[i].intensity, 4);
    std::memcpy(p + 16, &pts[i].t, 4);
    std::memcpy(p + 20, &pts[i].ring, 2);
  }
  return m;
}

const std::vector<BridgePoint> kPoints = {
    {1.0F, 2.0F, 3.0F, 10.0F, 0U, 0},
    {-4.5F, 0.25F, -1.0F, 20.0F, 50000000U, 7},
    {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F, 99999999U, 15},
};

void expect_scan_matches(const mss::common::ScanFrame& s) {
  ASSERT_EQ(s.size(), kPoints.size());
  EXPECT_EQ(s.t_start.ns, static_cast<std::int64_t>(kSec) * 1000000000 + kNsec);
  for (std::size_t i = 0; i < kPoints.size(); ++i) {
    if (std::isnan(kPoints[i].x)) {
      EXPECT_TRUE(std::isnan(s.xyz()[3 * i]));  // kept: preprocess drops it, as in mss_bench
    } else {
      EXPECT_EQ(s.xyz()[3 * i], kPoints[i].x);
    }
    EXPECT_EQ(s.xyz()[3 * i + 1], kPoints[i].y);
    EXPECT_EQ(s.xyz()[3 * i + 2], kPoints[i].z);
    EXPECT_EQ(s.intensity()[i], kPoints[i].intensity);
    EXPECT_EQ(s.t_offset_ns()[i], kPoints[i].t);
    EXPECT_EQ(s.ring()[i], kPoints[i].ring);
  }
}

TEST(Conversions, TimestampRoundTrip) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = kSec;
  stamp.nanosec = kNsec;
  const Timestamp t = mss_ros2::to_timestamp(stamp);
  EXPECT_EQ(t.ns, 1700000000123456789LL);
  const builtin_interfaces::msg::Time back = mss_ros2::to_msg_time(t);
  EXPECT_EQ(back.sec, kSec);
  EXPECT_EQ(back.nanosec, kNsec);
  // Before the epoch: floor division keeps nanosec in [0, 1e9).
  const builtin_interfaces::msg::Time neg = mss_ros2::to_msg_time(Timestamp{-1});
  EXPECT_EQ(neg.sec, -1);
  EXPECT_EQ(neg.nanosec, 999999999U);
}

TEST(Conversions, BridgeCloudToScanFrame) {
  mss::common::ScanFrame scan(16);
  ASSERT_TRUE(mss_ros2::scan_from_cloud(bridge_cloud(kPoints), scan).is_ok());
  expect_scan_matches(scan);
}

TEST(Conversions, CloudUsesOwnOffsetsAndPointStep) {
  mss::common::ScanFrame scan(16);
  ASSERT_TRUE(mss_ros2::scan_from_cloud(bridge_cloud(kPoints, 10, true), scan).is_ok());
  expect_scan_matches(scan);
}

TEST(Conversions, CloudWithoutIntensityGivesZero) {
  sensor_msgs::msg::PointCloud2 m = bridge_cloud(kPoints);
  m.fields.erase(m.fields.begin() + 3);
  mss::common::ScanFrame scan(16);
  ASSERT_TRUE(mss_ros2::scan_from_cloud(m, scan).is_ok());
  for (const float v : scan.intensity()) {
    EXPECT_EQ(v, 0.0F);
  }
}

TEST(Conversions, CloudErrorsLeaveScanEmpty) {
  mss::common::ScanFrame scan(16);
  sensor_msgs::msg::PointCloud2 no_t = bridge_cloud(kPoints);
  no_t.fields.erase(no_t.fields.begin() + 4);
  mss::common::Status st = mss_ros2::scan_from_cloud(no_t, scan);
  EXPECT_FALSE(st.is_ok());
  EXPECT_NE(st.reason().find("bridge"), std::string::npos);
  EXPECT_EQ(scan.size(), 0U);

  sensor_msgs::msg::PointCloud2 t_float = bridge_cloud(kPoints);
  t_float.fields[4].datatype = PointField::FLOAT32;
  EXPECT_FALSE(mss_ros2::scan_from_cloud(t_float, scan).is_ok());

  sensor_msgs::msg::PointCloud2 x_double = bridge_cloud(kPoints);
  x_double.fields[0].datatype = PointField::FLOAT64;
  EXPECT_FALSE(mss_ros2::scan_from_cloud(x_double, scan).is_ok());

  sensor_msgs::msg::PointCloud2 truncated = bridge_cloud(kPoints);
  truncated.data.pop_back();
  EXPECT_FALSE(mss_ros2::scan_from_cloud(truncated, scan).is_ok());

  sensor_msgs::msg::PointCloud2 past_step = bridge_cloud(kPoints);
  past_step.fields[5].offset = 21;  // ring would end at byte 23 > point_step 22
  EXPECT_FALSE(mss_ros2::scan_from_cloud(past_step, scan).is_ok());

  // Offsets near UINT32_MAX: offset + width wraps in 32-bit arithmetic; before the fix both
  // passed the bound check and read about 4 GB past the buffer (SIGSEGV, review B-1).
  sensor_msgs::msg::PointCloud2 ring_wrap = bridge_cloud(kPoints);
  ring_wrap.fields[5].offset = 0xFFFFFFFFU;
  EXPECT_FALSE(mss_ros2::scan_from_cloud(ring_wrap, scan).is_ok());
  EXPECT_EQ(scan.size(), 0U);
  sensor_msgs::msg::PointCloud2 x_wrap = bridge_cloud(kPoints);
  x_wrap.fields[0].offset = 0xFFFFFFFDU;
  EXPECT_FALSE(mss_ros2::scan_from_cloud(x_wrap, scan).is_ok());
  EXPECT_EQ(scan.size(), 0U);

  sensor_msgs::msg::PointCloud2 big_endian = bridge_cloud(kPoints);
  big_endian.is_bigendian = true;
  EXPECT_FALSE(mss_ros2::scan_from_cloud(big_endian, scan).is_ok());

  mss::common::ScanFrame small(2);
  st = mss_ros2::scan_from_cloud(bridge_cloud(kPoints), small);
  EXPECT_FALSE(st.is_ok());
  EXPECT_NE(st.reason().find("max_points_per_scan"), std::string::npos);
  EXPECT_EQ(small.size(), 0U);
}

TEST(Conversions, ImuToSample) {
  sensor_msgs::msg::Imu m;
  m.header.stamp.sec = kSec;
  m.header.stamp.nanosec = kNsec;
  m.linear_acceleration.x = 0.1;
  m.linear_acceleration.y = -0.2;
  m.linear_acceleration.z = 9.8;
  m.angular_velocity.x = 0.01;
  m.angular_velocity.y = 0.02;
  m.angular_velocity.z = -0.3;
  const mss::common::ImuSample s = mss_ros2::imu_from_msg(m);
  EXPECT_EQ(s.t.ns, 1700000000123456789LL);
  EXPECT_EQ(s.acc_b, mss::common::Vec3(0.1, -0.2, 9.8));
  EXPECT_EQ(s.gyro_b, mss::common::Vec3(0.01, 0.02, -0.3));
}

TEST(Conversions, OdometryToWheelSample) {
  nav_msgs::msg::Odometry m;
  m.header.stamp.sec = 3;
  m.header.stamp.nanosec = 500;
  m.pose.pose.position.x = 99.0;  // ignored
  m.twist.twist.linear.x = 0.5;
  m.twist.twist.linear.y = 7.0;  // ignored (differential drive)
  m.twist.twist.angular.z = 0.3;
  const mss::common::WheelSample s = mss_ros2::wheel_from_odometry(m);
  EXPECT_EQ(s.t.ns, 3000000500LL);
  EXPECT_EQ(s.v_mps, 0.5);
  EXPECT_EQ(s.omega_radps, 0.3);
}

sensor_msgs::msg::NavSatFix fix_at(const mss::gnss::Lla& lla) {
  sensor_msgs::msg::NavSatFix m;
  m.header.stamp.sec = kSec;
  m.latitude = lla.lat_deg;
  m.longitude = lla.lon_deg;
  m.altitude = lla.alt_m;
  m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  return m;
}

TEST(Conversions, NavSatFixToEnu) {
  const mss::gnss::Lla origin{21.0278, 105.8342, 12.0};  // E-01 origin_lla
  const mss::common::Vec3 p_true(3.0, -4.0, 5.0);
  sensor_msgs::msg::NavSatFix m = fix_at(mss::gnss::enu_to_lla(p_true, origin));
  m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  m.position_covariance = {0.25, 0.0, 0.0, 0.0, 0.36, 0.0, 0.0, 0.0, 4.0};
  m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  const mss_ros2::GnssFallbackSigma sigma{1.5, 3.0};
  const mss::common::GnssMeasurement g = mss_ros2::gnss_from_navsat(m, origin, sigma);
  EXPECT_EQ(g.t.ns, static_cast<std::int64_t>(kSec) * 1000000000);
  EXPECT_LT((g.p_e_g - p_true).norm(), 1e-6);  // G-01 round-trip accuracy is < 1 mm
  EXPECT_EQ(g.cov(0, 0), 0.25);
  EXPECT_EQ(g.cov(1, 1), 0.36);
  EXPECT_EQ(g.cov(2, 2), 4.0);
  EXPECT_EQ(g.cov(0, 1), 0.0);
  EXPECT_EQ(g.status, 1);

  // COVARIANCE_TYPE_UNKNOWN: diag(sigma_h^2, sigma_h^2, sigma_v^2), as bag_to_mssdata fills it.
  m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  const mss::common::GnssMeasurement u = mss_ros2::gnss_from_navsat(m, origin, sigma);
  const mss::common::Mat3 expected = mss::common::Vec3(2.25, 2.25, 9.0).asDiagonal();
  EXPECT_EQ(u.cov, expected);
}

TEST(Conversions, OriginNeedsAValidFix) {
  const mss::gnss::Lla lla{21.0278, 105.8342, 12.0};
  mss::gnss::Lla out;
  sensor_msgs::msg::NavSatFix m = fix_at(lla);
  ASSERT_TRUE(mss_ros2::origin_from_fix(m, out));
  EXPECT_EQ(out.lat_deg, lla.lat_deg);
  EXPECT_EQ(out.lon_deg, lla.lon_deg);
  EXPECT_EQ(out.alt_m, lla.alt_m);
  // The origin's own fix maps to the ENU origin.
  EXPECT_LT(mss_ros2::gnss_from_navsat(m, out, {1.0, 1.0}).p_e_g.norm(), 1e-9);

  m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  EXPECT_FALSE(mss_ros2::origin_from_fix(m, out));
  m = fix_at(lla);
  m.altitude = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mss_ros2::origin_from_fix(m, out));
}

mss::common::Pose yaw90_pose() {
  mss::common::Pose T;
  T.R = Eigen::AngleAxisd(M_PI / 2.0, mss::common::Vec3::UnitZ()).toRotationMatrix();
  T.p = mss::common::Vec3(1.0, 2.0, 3.0);
  return T;
}

TEST(Conversions, PoseHamiltonQuaternion) {
  const geometry_msgs::msg::Pose m = mss_ros2::pose_to_msg(yaw90_pose());
  EXPECT_EQ(m.position.x, 1.0);
  EXPECT_EQ(m.position.y, 2.0);
  EXPECT_EQ(m.position.z, 3.0);
  // 90 deg about z: (w, x, y, z) = (cos 45, 0, 0, sin 45).
  EXPECT_NEAR(m.orientation.w, std::sqrt(0.5), kTol);
  EXPECT_NEAR(m.orientation.x, 0.0, kTol);
  EXPECT_NEAR(m.orientation.y, 0.0, kTol);
  EXPECT_NEAR(m.orientation.z, std::sqrt(0.5), kTol);
}

TEST(Conversions, CovarianceRotatedToWorldAxes) {
  // Body x variance 1, body y variance 4 (rho), rotation variances 0.01 / 0.02 / 0.03 (phi).
  mss::common::Mat6 cov = mss::common::Mat6::Zero();
  cov.diagonal() << 1.0, 4.0, 9.0, 0.01, 0.02, 0.03;
  cov(0, 3) = cov(3, 0) = 0.05;  // rho_x / phi_x correlation
  const mss::common::Mat6 ros = mss_ros2::cov_to_ros(yaw90_pose(), cov);
  // R_w_b = Rz(90): body x is world y, body y is world -x.
  EXPECT_NEAR(ros(0, 0), 4.0, kTol);
  EXPECT_NEAR(ros(1, 1), 1.0, kTol);
  EXPECT_NEAR(ros(2, 2), 9.0, kTol);
  EXPECT_NEAR(ros(3, 3), 0.02, kTol);
  EXPECT_NEAR(ros(4, 4), 0.01, kTol);
  EXPECT_NEAR(ros(5, 5), 0.03, kTol);
  EXPECT_NEAR(ros(1, 4), 0.05, kTol);  // world y / rot y
  EXPECT_NEAR(ros(0, 3), 0.0, kTol);

  // First-order check against a right perturbation: T * Exp([rho, 0]) moves the origin by
  // R rho (no rotation), which is what J = blkdiag(R, R) maps.
  const mss::common::Pose T = yaw90_pose();
  const mss::common::Vec3 rho(1e-3, 0.0, 0.0);
  mss::common::Pose d;
  d.p = rho;
  const mss::common::Vec3 dp = (T * d).p - T.p;
  EXPECT_LT((dp - mss::common::Vec3(0.0, 1e-3, 0.0)).norm(), kTol);
}

TEST(Conversions, OdometryFromEstimate) {
  mss::common::Estimate e;
  e.state.t = Timestamp{1700000000123456789LL};
  e.state.T_w_b = yaw90_pose();
  e.state.v_w_b = mss::common::Vec3(0.0, 0.5, 0.0);  // moving along world y = body x
  e.cov.diagonal() << 1.0, 4.0, 9.0, 0.01, 0.02, 0.03;
  const nav_msgs::msg::Odometry o = mss_ros2::odometry_from_estimate(e, {"map", "base_link"});
  EXPECT_EQ(o.header.frame_id, "map");
  EXPECT_EQ(o.child_frame_id, "base_link");
  EXPECT_EQ(o.header.stamp.sec, kSec);
  EXPECT_EQ(o.header.stamp.nanosec, kNsec);
  EXPECT_EQ(o.pose.pose.position.y, 2.0);
  EXPECT_NEAR(o.pose.covariance[0], 4.0, kTol);    // (x, x)
  EXPECT_NEAR(o.pose.covariance[7], 1.0, kTol);    // (y, y)
  EXPECT_NEAR(o.pose.covariance[35], 0.03, kTol);  // (rotz, rotz)
  EXPECT_NEAR(o.twist.twist.linear.x, 0.5, kTol);  // in body
  EXPECT_NEAR(o.twist.twist.linear.y, 0.0, kTol);
}

TEST(Conversions, PathEntryAndTransform) {
  const Timestamp t{5000000001LL};
  const geometry_msgs::msg::PoseStamped ps = mss_ros2::pose_stamped(yaw90_pose(), t, "map");
  EXPECT_EQ(ps.header.frame_id, "map");
  EXPECT_EQ(ps.header.stamp.sec, 5);
  EXPECT_EQ(ps.header.stamp.nanosec, 1U);
  EXPECT_EQ(ps.pose.position.x, 1.0);

  const geometry_msgs::msg::TransformStamped tf =
      mss_ros2::transform_from_pose(yaw90_pose(), t, {"map", "base_link"});
  EXPECT_EQ(tf.header.frame_id, "map");
  EXPECT_EQ(tf.child_frame_id, "base_link");
  EXPECT_EQ(tf.transform.translation.x, 1.0);
  EXPECT_EQ(tf.transform.translation.y, 2.0);
  EXPECT_EQ(tf.transform.translation.z, 3.0);
  EXPECT_NEAR(tf.transform.rotation.z, std::sqrt(0.5), kTol);
  // The TF maps body to map: the body x axis (1, 0, 0) lands on world y.
  const Eigen::Quaterniond q(tf.transform.rotation.w,
                             tf.transform.rotation.x,
                             tf.transform.rotation.y,
                             tf.transform.rotation.z);  // (w, x, y, z)
  const mss::common::Vec3 x_w = q * mss::common::Vec3::UnitX();
  EXPECT_LT((x_w - mss::common::Vec3::UnitY()).norm(), kTol);
}

TEST(Conversions, MapPointsToCloud) {
  const std::vector<Eigen::Vector3f> pts = {Eigen::Vector3f(1.0F, 2.0F, 3.0F),
                                            Eigen::Vector3f(-0.5F, 0.0F, 8.25F)};
  sensor_msgs::msg::PointCloud2 m;
  mss_ros2::cloud_from_points(pts, Timestamp{2000000000LL}, "map", m);
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.header.stamp.sec, 2);
  EXPECT_EQ(m.height, 1U);
  EXPECT_EQ(m.width, 2U);
  EXPECT_EQ(m.point_step, 12U);
  EXPECT_EQ(m.row_step, 24U);
  EXPECT_FALSE(m.is_bigendian);
  ASSERT_EQ(m.fields.size(), 3U);
  EXPECT_EQ(m.fields[2].name, "z");
  EXPECT_EQ(m.fields[2].offset, 8U);
  EXPECT_EQ(m.fields[2].datatype, PointField::FLOAT32);
  ASSERT_EQ(m.data.size(), 24U);
  float v[6];
  std::memcpy(v, m.data.data(), sizeof(v));
  EXPECT_EQ(v[0], 1.0F);
  EXPECT_EQ(v[2], 3.0F);
  EXPECT_EQ(v[3], -0.5F);
  EXPECT_EQ(v[5], 8.25F);

  // Reuse: a smaller map shrinks the message.
  mss_ros2::cloud_from_points({pts[0]}, Timestamp{}, "map", m);
  EXPECT_EQ(m.width, 1U);
  EXPECT_EQ(m.data.size(), 12U);
  // The bridge cloud reader accepts nothing without t / ring, so the map is not a scan.
  mss::common::ScanFrame scan(4);
  EXPECT_FALSE(mss_ros2::scan_from_cloud(m, scan).is_ok());
}

}  // namespace
