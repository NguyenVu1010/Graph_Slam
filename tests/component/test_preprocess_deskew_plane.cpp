// P-01 (docs/Test_CI_CD.md): deskew of a synthetic plane scan under known constant motion.
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/preprocess/config.hpp"
#include "mss/preprocess/preprocessor.hpp"

namespace mss::preprocess {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::int64_t kTStartNs = 3'000'000'000;
constexpr std::int64_t kImuPeriodNs = 5'000'000;  // 200 Hz (imu.rate_hz)
constexpr std::size_t kImuSamples = 21;
constexpr std::int64_t kScanLengthNs = 100'000'000;  // 10 Hz LiDAR (ADR-011)
constexpr std::size_t kPoints = 3600;
constexpr std::size_t kRings = 16;
constexpr double kPlaneOffsetM = 5.0;  // n . p = 5 m in l0

common::Pose rig_T_b_l() {  // ADR-011 section 2 rig: R_y(2 deg), p = [0.10, 0, 0] m
  common::Pose T;
  T.R = Eigen::AngleAxisd(2.0 * kDegToRad, common::Vec3::UnitY()).toRotationMatrix();
  T.p = common::Vec3(0.10, 0.0, 0.0);
  return T;
}

common::Vec6 body_twist() {  // v = [1, 0, 0] m/s, w = [0, 0, 1] rad/s (ADR-011 section 2)
  common::Vec6 xi;
  xi << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  return xi;
}

common::Pose T_b0_b(std::int64_t offset_ns) {
  const double t_s = static_cast<double>(offset_ns) / 1e9;
  const common::Vec6 xi = t_s * body_twist();
  return common::se3_exp(xi);
}

common::Vec3 plane_normal() {
  return common::Vec3(1.0, 0.2, 0.1).normalized();
}

imu::ImuTrajectory constant_motion_traj() {
  imu::ImuTrajectory traj;
  traj.t_start = common::Timestamp{kTStartNs};
  traj.size = kImuSamples;
  traj.samples[0] = {traj.t_start, common::Pose{}};
  for (std::size_t k = 1; k < kImuSamples; ++k) {
    const std::int64_t off = static_cast<std::int64_t>(k) * kImuPeriodNs;
    traj.samples[k] = {common::Timestamp{kTStartNs + off}, T_b0_b(off)};
  }
  return traj;
}

// Each point is the intersection of its beam (fixed in l(t_i)) with the plane, so the
// scan is skewed exactly as a LiDAR on the moving body would see it.
common::ScanFrame skewed_plane_scan() {
  const common::Pose T_b_l = rig_T_b_l();
  const common::Vec3 n_l0 = plane_normal();
  common::ScanFrame scan(kPoints);
  scan.t_start = common::Timestamp{kTStartNs};
  for (std::size_t i = 0; i < kPoints; ++i) {
    const std::int64_t off =
        static_cast<std::int64_t>(i) * kScanLengthNs / static_cast<std::int64_t>(kPoints);
    const common::Pose T_l0_l = T_b_l.inverse() * T_b0_b(off) * T_b_l;
    // Plane n . p_l0 = d in l(t_i): (R^T n) . q = d - n . p.
    const common::Vec3 n_l = T_l0_l.R.transpose() * n_l0;
    const double d_l = kPlaneOffsetM - n_l0.dot(T_l0_l.p);
    const double az = (-60.0 + 120.0 * static_cast<double>(i) / kPoints) * kDegToRad;
    const double el = (-15.0 + 2.0 * static_cast<double>(i % kRings)) * kDegToRad;
    const common::Vec3 dir(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    const double range_m = d_l / n_l.dot(dir);
    const common::Vec3 q = range_m * dir;
    scan.push(static_cast<float>(q.x()),
              static_cast<float>(q.y()),
              static_cast<float>(q.z()),
              1.0F,
              static_cast<std::uint32_t>(off),
              static_cast<std::uint16_t>(i % kRings));
  }
  return scan;
}

double plane_rms_m(const common::ScanFrame& out) {
  const common::Vec3 n = plane_normal();
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    const common::Vec3 p(out.xyz()[3 * i], out.xyz()[3 * i + 1], out.xyz()[3 * i + 2]);
    const double e = n.dot(p) - kPlaneOffsetM;
    sum_sq += e * e;
  }
  return std::sqrt(sum_sq / static_cast<double>(out.size()));
}

TEST(PreprocessComponent, P01_DeskewConstantMotionPlane) {
  const common::ScanFrame in = skewed_plane_scan();
  common::ScanFrame out(kPoints);
  Preprocessor pre(PreprocessConfig{0.5, 100.0, 0.0, true}, rig_T_b_l(), kPoints);
  ASSERT_TRUE(pre.process(in, constant_motion_traj(), out).is_ok());
  ASSERT_EQ(out.size(), kPoints);
  EXPECT_LT(plane_rms_m(out), 1e-3);  // Test_CI_CD P-01: RMS < 1 mm
}

TEST(PreprocessComponent, P01_WithoutDeskewPlaneIsSmeared) {
  const common::ScanFrame in = skewed_plane_scan();
  common::ScanFrame out(kPoints);
  Preprocessor pre(PreprocessConfig{0.5, 100.0, 0.0, false}, rig_T_b_l(), kPoints);
  ASSERT_TRUE(pre.process(in, constant_motion_traj(), out).is_ok());
  ASSERT_EQ(out.size(), kPoints);
  EXPECT_GT(plane_rms_m(out), 1e-2);  // the test discriminates: skew is visible
}

}  // namespace
}  // namespace mss::preprocess
