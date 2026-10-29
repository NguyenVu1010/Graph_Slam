#pragma once

// Synthetic scenes for the loop-closure tests (L-01..L-03, LoopComponent): a hall with
// pillars of different heights at irregular places, sampled on a grid, and a "scan" that
// keeps every world point within a range of the sensor (no occlusion: both scans of a pair
// follow the same model). Header-only, test code.

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <vector>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/config.hpp"

namespace mss::test {

constexpr double kLoopPi = 3.14159265358979323846;

inline common::Mat3 loop_rz(double yaw) {
  common::Mat3 R;
  R << std::cos(yaw), -std::sin(yaw), 0.0,  //
      std::sin(yaw), std::cos(yaw), 0.0,    //
      0.0, 0.0, 1.0;
  return R;
}

// Axis-aligned box surface (all 6 faces except the bottom), grid spacing `s` (m).
inline void add_box(std::vector<common::Vec3>& out,
                    const common::Vec3& lo,
                    const common::Vec3& hi,
                    double s) {
  for (double x = lo.x(); x <= hi.x() + 1e-9; x += s) {
    for (double z = lo.z(); z <= hi.z() + 1e-9; z += s) {
      out.emplace_back(x, lo.y(), z);
      out.emplace_back(x, hi.y(), z);
    }
  }
  for (double y = lo.y() + s; y <= hi.y() - s + 1e-9; y += s) {
    for (double z = lo.z(); z <= hi.z() + 1e-9; z += s) {
      out.emplace_back(lo.x(), y, z);
      out.emplace_back(hi.x(), y, z);
    }
  }
  for (double x = lo.x() + s; x <= hi.x() - s + 1e-9; x += s) {
    for (double y = lo.y() + s; y <= hi.y() - s + 1e-9; y += s) {
      out.emplace_back(x, y, hi.z());
    }
  }
}

// Hall x in [-20, 20], y in [-8, 8] m: floor at z = 0, walls 4 m high, and pillars of
// different heights at irregular places, so no two places look alike.
inline std::vector<common::Vec3> hall_points_w(double s = 0.25) {
  std::vector<common::Vec3> p;
  constexpr double kX = 20.0;
  constexpr double kY = 8.0;
  constexpr double kH = 4.0;
  for (double x = -kX; x <= kX + 1e-9; x += s) {
    for (double y = -kY; y <= kY + 1e-9; y += s) {
      p.emplace_back(x, y, 0.0);
    }
    for (double z = s; z <= kH + 1e-9; z += s) {
      p.emplace_back(x, -kY, z);
      p.emplace_back(x, kY, z);
    }
  }
  for (double y = -kY + s; y <= kY - s + 1e-9; y += s) {
    for (double z = s; z <= kH + 1e-9; z += s) {
      p.emplace_back(-kX, y, z);
      p.emplace_back(kX, y, z);
    }
  }
  // Pillars (x, y, height): irregular spacing and heights.
  const double pillars[][3] = {{-15.0, 3.0, 1.0},
                               {-11.5, -4.5, 2.5},
                               {-7.0, 1.5, 3.5},
                               {-3.5, -6.0, 1.5},
                               {0.5, 5.0, 2.0},
                               {2.5, -2.0, 3.0},
                               {6.5, 4.0, 1.2},
                               {9.0, -5.5, 2.8},
                               {13.5, 0.5, 1.8},
                               {16.5, 6.0, 3.2},
                               {-18.0, -2.5, 2.2},
                               {11.0, 2.8, 0.8}};
  for (const auto& q : pillars) {
    add_box(
        p, common::Vec3(q[0] - 0.5, q[1] - 0.5, s), common::Vec3(q[0] + 0.5, q[1] + 0.5, q[2]), s);
  }
  // Two shelves (long, low boxes).
  add_box(p, common::Vec3(-12.0, 6.0, s), common::Vec3(-6.0, 7.0, 2.0), s);
  add_box(p, common::Vec3(4.0, -7.5, s), common::Vec3(12.0, -6.5, 1.0), s);
  return p;
}

// Every world point within `range_m` of the LiDAR, in l.
inline void scan_at(const std::vector<common::Vec3>& p_w,
                    const common::Pose& T_w_l,
                    double range_m,
                    common::ScanFrame& out) {
  out.clear();
  const common::Pose T_l_w = T_w_l.inverse();
  for (const common::Vec3& q : p_w) {
    if ((q - T_w_l.p).norm() > range_m) {
      continue;
    }
    const common::Vec3 p_l = T_l_w * q;
    (void)out.push(static_cast<float>(p_l.x()),
                   static_cast<float>(p_l.y()),
                   static_cast<float>(p_l.z()),
                   0.0F,
                   0U,
                   0U);
  }
}

// Body pose on the floor plane: (x, y) and yaw, body 0.3 m above the floor.
inline common::Pose body_pose(double x, double y, double yaw) {
  return common::Pose{loop_rz(yaw), common::Vec3(x, y, 0.3)};
}

// LiDAR 0.5 m above the body, level (the extrinsic of the tests).
inline common::Pose loop_T_b_l() {
  return common::Pose{common::Mat3::Identity(), common::Vec3(0.1, 0.0, 0.5)};
}

// config/default.yaml with loop_closure.enabled (the shipped values of every other key).
inline pipeline::Config shipped_loop_config() {
  const common::Result<pipeline::Config> r =
      pipeline::load_config(std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml");
  pipeline::Config c = r.is_ok() ? r.value() : pipeline::Config{};
  c.loop_closure.enabled = r.is_ok();
  return c;
}

// Corridor along x, walls y = +-2 m, 3 m high, floor, with identical 0.6 x 0.6 x 2 m pillars
// against both walls every `period_m`; no end wall in range, so the scene repeats along x with
// that period (review C1, probe I-1: rack bays in a warehouse aisle).
inline std::vector<common::Vec3> periodic_corridor_w(double period_m, double s = 0.25) {
  std::vector<common::Vec3> p;
  for (double x = -40.0; x <= 40.0 + 1e-9; x += s) {
    for (double y = -2.0; y <= 2.0 + 1e-9; y += s) {
      p.emplace_back(x, y, 0.0);
    }
    for (double z = s; z <= 3.0 + 1e-9; z += s) {
      p.emplace_back(x, -2.0, z);
      p.emplace_back(x, 2.0, z);
    }
  }
  for (double x = -40.0; x <= 40.0 + 1e-9; x += period_m) {
    add_box(p, common::Vec3(x - 0.3, -2.0, s), common::Vec3(x + 0.3, -1.4, 2.0), s);
    add_box(p, common::Vec3(x - 0.3, 1.4, s), common::Vec3(x + 0.3, 2.0, 2.0), s);
  }
  return p;
}

// Corridor along x (walls y = +-1.5 m, 3 m high, floor), x in [-30, 30] m: no end wall within
// a 15 m range, so translation along x is unobserved.
inline std::vector<common::Vec3> corridor_w(double s = 0.25) {
  std::vector<common::Vec3> p;
  for (double x = -30.0; x <= 30.0 + 1e-9; x += s) {
    for (double y = -1.5; y <= 1.5 + 1e-9; y += s) {
      p.emplace_back(x, y, 0.0);
    }
    for (double z = s; z <= 3.0 + 1e-9; z += s) {
      p.emplace_back(x, -1.5, z);
      p.emplace_back(x, 1.5, z);
    }
  }
  return p;
}

}  // namespace mss::test
