#pragma once

// Synthetic box world for pipeline component and integration tests
// (docs/derivations/pipeline.md section 7): a body that stands still, ramps up its speed
// and then drives a circle inside an axis-aligned box; exact IMU, wheel and GNSS models
// with optional white noise (fixed seed), a ray-cast LiDAR, and an mssdata writer.
// Header-only, test code only.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <random>
#include <string>
#include <system_error>

#include <Eigen/Geometry>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_format.hpp"
#include "mss/io/dataset_writer.hpp"

namespace mss::test {

constexpr double kBoxPi = 3.14159265358979323846;
constexpr double kBoxGravity = 9.80665;  // m/s^2

// ADR-011 LiDAR extrinsic: (0.10, 0, 0.50) m, quaternion_wxyz (0.99984770, 0, 0.01745241, 0)
// (Hamilton order w, x, y, z; 2 deg pitch).
inline common::Pose box_default_T_b_l() {
  common::Pose T;
  const Eigen::Quaterniond q(0.99984770, 0.0, 0.01745241, 0.0);  // (w, x, y, z)
  T.R = q.normalized().toRotationMatrix();
  T.p = common::Vec3(0.10, 0.0, 0.50);
  return T;
}

struct BoxWorldParams {
  std::int64_t t0_ns = 1'000'000'000;      // ns, dataset start
  double static_s = 3.0;                   // s, still at the start
  double ramp_s = 2.0;                     // s, speed ramp
  double motion_s = 15.0;                  // s, cruise after the ramp
  double speed_mps = 0.5;                  // m/s, cruise speed V
  double yaw_rate_radps = 0.25;            // rad/s, cruise yaw rate W (kappa = W / V)
  common::Vec3 box_half_m{8.0, 5.0, 1.5};  // m, half extents of the box centred at origin
  int rings = 16;
  int columns = 720;
  double elevation_min_deg = -15.0;
  double elevation_max_deg = 15.0;
  double scan_period_s = 0.1;
  double imu_rate_hz = 200.0;
  double wheel_rate_hz = 50.0;
  double gnss_rate_hz = 5.0;
  std::int64_t gnss_offset_ns = 0;  // ns, GNSS stamps at t0 + offset + k / gnss_rate_hz
  common::Pose T_b_l = box_default_T_b_l();
  common::Vec3 p_b_g{-0.20, 0.10, 0.60};  // m, antenna lever arm in b
  // IMU white noise densities, equal to config/default.yaml.
  double acc_noise_density = 5.8e-3;      // m/s^2/sqrt(Hz)
  double gyro_noise_density = 3.5e-4;     // rad/s/sqrt(Hz)
  double gnss_sigma_m = 0.5;              // m, GNSS white noise per axis
  double gnss_yaw_e_w_deg = 30.0;         // deg, yaw of T_e_w
  common::Vec3 p_e_w_m{12.0, -7.0, 1.0};  // m, translation of T_e_w
  std::uint32_t seed = 42;
};

inline std::int64_t box_ns(double s) {
  return static_cast<std::int64_t>(std::llround(s * 1e9));
}

// Kinematics along the path at time t: distance s (m), speed v (m/s), tangential
// acceleration a_t (m/s^2).
struct PathState {
  double s_m = 0.0;
  double v_mps = 0.0;
  double a_mps2 = 0.0;
};

inline PathState box_path(const BoxWorldParams& p, common::Timestamp t) {
  const double tau = static_cast<double>(t.ns - p.t0_ns - box_ns(p.static_s)) * 1e-9;
  const double V = p.speed_mps;
  PathState out;
  if (tau < 0.0) {
    return out;
  }
  if (tau < p.ramp_s) {
    const double x = kBoxPi * tau / p.ramp_s;
    out.v_mps = V * (1.0 - std::cos(x)) / 2.0;
    out.s_m = V / 2.0 * (tau - (p.ramp_s / kBoxPi) * std::sin(x));
    out.a_mps2 = V * kBoxPi / (2.0 * p.ramp_s) * std::sin(x);
    return out;
  }
  out.v_mps = V;
  out.s_m = V * p.ramp_s / 2.0 + V * (tau - p.ramp_s);
  return out;
}

inline double box_kappa(const BoxWorldParams& p) {
  return p.yaw_rate_radps / p.speed_mps;  // 1/m
}

inline common::Mat3 box_rz(double yaw) {
  common::Mat3 R;
  R << std::cos(yaw), -std::sin(yaw), 0.0,  //
      std::sin(yaw), std::cos(yaw), 0.0,    //
      0.0, 0.0, 1.0;
  return R;
}

// True body state at t (w = level start frame, origin at the start position).
inline common::NavState box_truth(const BoxWorldParams& p, common::Timestamp t) {
  const PathState ps = box_path(p, t);
  const double kappa = box_kappa(p);
  const double psi = kappa * ps.s_m;
  common::NavState x;
  x.t = t;
  x.T_w_b.R = box_rz(psi);
  x.T_w_b.p = common::Vec3(std::sin(psi), 1.0 - std::cos(psi), 0.0) / kappa;
  x.v_w_b = ps.v_mps * common::Vec3(std::cos(psi), std::sin(psi), 0.0);
  return x;
}

// IMU sample at t; white noise from `rng` when not null.
inline common::ImuSample box_imu(const BoxWorldParams& p, common::Timestamp t, std::mt19937* rng) {
  const PathState ps = box_path(p, t);
  const double kappa = box_kappa(p);
  const double psi = kappa * ps.s_m;
  const common::Vec3 a_w =
      ps.a_mps2 * common::Vec3(std::cos(psi), std::sin(psi), 0.0) +
      kappa * ps.v_mps * ps.v_mps * common::Vec3(-std::sin(psi), std::cos(psi), 0.0);
  const common::Vec3 g_w(0.0, 0.0, -kBoxGravity);
  common::ImuSample s;
  s.t = t;
  s.acc_b = box_rz(psi).transpose() * (a_w - g_w);
  s.gyro_b = common::Vec3(0.0, 0.0, kappa * ps.v_mps);
  if (rng != nullptr) {
    std::normal_distribution<double> n01(0.0, 1.0);
    const double sa = p.acc_noise_density * std::sqrt(p.imu_rate_hz);
    const double sg = p.gyro_noise_density * std::sqrt(p.imu_rate_hz);
    for (int k = 0; k < 3; ++k) {
      s.acc_b(k) += sa * n01(*rng);
    }
    for (int k = 0; k < 3; ++k) {
      s.gyro_b(k) += sg * n01(*rng);
    }
  }
  return s;
}

inline common::WheelSample box_wheel(const BoxWorldParams& p, common::Timestamp t) {
  const PathState ps = box_path(p, t);
  common::WheelSample w;
  w.t = t;
  w.v_mps = ps.v_mps;
  w.omega_radps = box_kappa(p) * ps.v_mps;
  return w;
}

inline common::Pose box_T_e_w(const BoxWorldParams& p) {
  common::Pose T;
  T.R = box_rz(p.gnss_yaw_e_w_deg * kBoxPi / 180.0);
  T.p = p.p_e_w_m;
  return T;
}

// Ray-cast scan starting at t_start: column c fired at t_start + c * scan_period / columns
// from the LiDAR pose at that time, ring-major order, points in l at their own time.
// `out` must have capacity rings * columns.
inline void box_scan(const BoxWorldParams& p, common::Timestamp t_start, common::ScanFrame& out) {
  out.clear();
  out.t_start = t_start;
  const std::int64_t period_ns = box_ns(p.scan_period_s);
  const double deg = kBoxPi / 180.0;
  for (int r = 0; r < p.rings; ++r) {
    const double el =
        (p.elevation_min_deg + (p.elevation_max_deg - p.elevation_min_deg) * r / (p.rings - 1)) *
        deg;
    for (int c = 0; c < p.columns; ++c) {
      const std::int64_t off = period_ns * c / p.columns;
      const double az = 2.0 * kBoxPi * c / p.columns;
      const common::Vec3 d_l(
          std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      const common::Pose T_w_l = box_truth(p, common::Timestamp{t_start.ns + off}).T_w_b * p.T_b_l;
      const common::Vec3 o = T_w_l.p;
      const common::Vec3 d = T_w_l.R * d_l;
      double range = 1e9;
      for (int k = 0; k < 3; ++k) {
        if (std::abs(d(k)) > 1e-12) {
          const double wall = d(k) > 0.0 ? p.box_half_m(k) : -p.box_half_m(k);
          const double hit = (wall - o(k)) / d(k);
          if (hit > 0.0 && hit < range) {
            range = hit;
          }
        }
      }
      const common::Vec3 p_l = range * d_l;
      (void)out.push(static_cast<float>(p_l.x()),
                     static_cast<float>(p_l.y()),
                     static_cast<float>(p_l.z()),
                     1.0F,
                     static_cast<std::uint32_t>(off),
                     static_cast<std::uint16_t>(r));
    }
  }
}

// Scan k (k >= 1) starts at t0 + k * scan_period; the last one ends near the end of motion.
inline int box_num_scans(const BoxWorldParams& p) {
  const double total_s = p.static_s + p.ramp_s + p.motion_s;
  return static_cast<int>(std::llround(total_s / p.scan_period_s)) - 1;
}

inline common::Timestamp box_scan_time(const BoxWorldParams& p, int k) {
  return common::Timestamp{p.t0_ns + k * box_ns(p.scan_period_s)};
}

// End of every stream: one IMU period past the last scan's end, so flush() leaves no scan
// uncovered.
inline common::Timestamp box_data_end(const BoxWorldParams& p) {
  const std::int64_t period_ns = box_ns(p.scan_period_s);
  const std::int64_t last_end =
      box_scan_time(p, box_num_scans(p)).ns + period_ns * (p.columns - 1) / p.columns;
  return common::Timestamp{last_end + box_ns(1.0 / p.imu_rate_hz)};
}

// Writes the whole dataset (IMU, wheel {v, omega}, GNSS p_e_g + N(0, sigma^2) with
// cov sigma^2 I, ground truth at every scan, scans) into `dir` (removed first).
inline common::Status write_box_dataset(const std::filesystem::path& dir, const BoxWorldParams& p) {
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  io::DatasetIndex meta;
  meta.T_b_l = p.T_b_l;
  meta.p_b_g = p.p_b_g;
  auto created = io::DatasetWriter::create(dir, meta);
  if (!created.is_ok()) {
    return common::Status::error(created.reason());
  }
  io::DatasetWriter w = std::move(created).value();
  const common::Timestamp t_end = box_data_end(p);
  std::mt19937 imu_rng(p.seed);
  const std::int64_t imu_period = box_ns(1.0 / p.imu_rate_hz);
  for (std::int64_t t = p.t0_ns; t <= t_end.ns; t += imu_period) {
    if (!w.write_imu(box_imu(p, common::Timestamp{t}, &imu_rng)).is_ok()) {
      return common::Status::error("write_imu failed");
    }
  }
  const std::int64_t wheel_period = box_ns(1.0 / p.wheel_rate_hz);
  for (std::int64_t t = p.t0_ns; t <= t_end.ns; t += wheel_period) {
    if (!w.write_odom(box_wheel(p, common::Timestamp{t})).is_ok()) {
      return common::Status::error("write_odom failed");
    }
  }
  std::mt19937 gnss_rng(p.seed + 1U);
  std::normal_distribution<double> n01(0.0, 1.0);
  const common::Pose T_e_w = box_T_e_w(p);
  const std::int64_t gnss_period = box_ns(1.0 / p.gnss_rate_hz);
  for (std::int64_t t = p.t0_ns + p.gnss_offset_ns; t <= t_end.ns; t += gnss_period) {
    const common::NavState x = box_truth(p, common::Timestamp{t});
    common::GnssMeasurement m;
    m.t = common::Timestamp{t};
    m.p_e_g = T_e_w * (x.T_w_b * p.p_b_g);
    for (int k = 0; k < 3; ++k) {
      m.p_e_g(k) += p.gnss_sigma_m * n01(gnss_rng);
    }
    m.cov = common::Mat3::Identity() * (p.gnss_sigma_m * p.gnss_sigma_m);
    m.status = 0;
    if (!w.write_gnss(m).is_ok()) {
      return common::Status::error("write_gnss failed");
    }
  }
  common::ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  for (int k = 1; k <= box_num_scans(p); ++k) {
    const common::Timestamp ts = box_scan_time(p, k);
    box_scan(p, ts, scan);
    if (!w.write_lidar(scan).is_ok() || !w.write_gt(box_truth(p, ts)).is_ok()) {
      return common::Status::error("write_lidar failed");
    }
  }
  return w.finish();
}

// Overlay for config/default.yaml: the box world's extrinsics and GNSS + wheel odometry
// enabled when asked, else both disabled.
inline std::string box_overlay_yaml(const BoxWorldParams& p, bool gnss_and_wheel) {
  const Eigen::Quaterniond q(p.T_b_l.R);
  char buf[1024];
  std::snprintf(buf,
                sizeof(buf),
                "mss_node:\n"
                "  ros__parameters:\n"
                "    frames:\n"
                "      T_b_l:\n"
                "        translation_m: [%.17g, %.17g, %.17g]\n"
                "        quaternion_wxyz: [%.17g, %.17g, %.17g, %.17g]\n"
                "      p_b_g_m: [%.17g, %.17g, %.17g]\n",
                p.T_b_l.p.x(),
                p.T_b_l.p.y(),
                p.T_b_l.p.z(),
                q.w(),
                q.x(),
                q.y(),
                q.z(),
                p.p_b_g.x(),
                p.p_b_g.y(),
                p.p_b_g.z());
  std::string out(buf);
  // Both flags explicit: config/default.yaml enables GNSS (PR 4a).
  out += gnss_and_wheel
             ? "    gnss:\n      enabled: true\n    wheel_odom:\n      enabled: true\n"
             : "    gnss:\n      enabled: false\n    wheel_odom:\n      enabled: false\n";
  return out;
}

}  // namespace mss::test
