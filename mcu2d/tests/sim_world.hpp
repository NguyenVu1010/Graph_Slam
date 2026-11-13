#pragma once

// Synthetic 2D worlds and an RPLIDAR-like ray caster for the tests (test code: heap is fine).

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "mss2d/messages.hpp"
#include "mss2d/pose2.hpp"

namespace mss2d {
namespace sim {

constexpr double kPi = 3.14159265358979323846;

struct Segment {
  double x0, y0, x1, y1;
};

struct World {
  std::vector<Segment> segs;

  void add_box(double cx, double cy, double w, double h, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double xs[4] = {-w / 2, w / 2, w / 2, -w / 2};
    const double ys[4] = {-h / 2, -h / 2, h / 2, h / 2};
    for (int i = 0; i < 4; ++i) {
      const int j = (i + 1) % 4;
      segs.push_back({cx + c * xs[i] - s * ys[i],
                      cy + s * xs[i] + c * ys[i],
                      cx + c * xs[j] - s * ys[j],
                      cy + s * xs[j] + c * ys[j]});
    }
  }
};

/// 8 x 6 m room (walls off the 5 cm cell lattice on purpose), an axis-aligned box, a
/// rotated box and a pillar. Fits a 10 x 10 m submap centred at the origin.
inline World room_world() {
  World w;
  w.add_box(0.025, 0.025, 8.03, 5.99, 0.0);
  w.add_box(1.3, 1.0, 0.6, 0.5, 0.0);
  w.add_box(-1.6, -0.9, 0.7, 0.45, 0.5);
  w.add_box(2.2, -1.7, 0.25, 0.25, 0.2);
  return w;
}

/// Featureless corridor along x, 2 m wide, 200 m long.
inline World corridor_world() {
  World w;
  w.segs.push_back({-100.0, -1.01, 100.0, -1.01});
  w.segs.push_back({-100.0, 0.99, 100.0, 0.99});
  return w;
}

/// Range along the ray from (ox, oy) at angle a, or 0 (no return) beyond max_range.
inline double cast(const World& w, double ox, double oy, double a, double max_range) {
  const double dx = std::cos(a);
  const double dy = std::sin(a);
  double best = max_range + 1.0;
  for (const Segment& s : w.segs) {
    const double ex = s.x1 - s.x0;
    const double ey = s.y1 - s.y0;
    const double den = dx * ey - dy * ex;
    if (std::abs(den) < 1e-12) {
      continue;
    }
    const double qx = s.x0 - ox;
    const double qy = s.y0 - oy;
    const double t = (qx * ey - qy * ex) / den;  // along the ray
    const double u = (qx * dy - qy * dx) / den;  // along the segment
    if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best) {
      best = t;
    }
  }
  return best <= max_range ? best : 0.0;
}

/// One turn of `beams` beams from the LiDAR pose T_w_l, gaussian range noise (sigma_m, fixed
/// seed). Beam i at angle 2 pi i / beams in l.
inline void make_scan(const World& w,
                      const Pose2T<double>& T_w_l,
                      int beams,
                      double max_range,
                      double sigma_m,
                      std::uint32_t seed,
                      Scan2d& out) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, sigma_m > 0.0 ? sigma_m : 1.0);
  out.count = static_cast<std::uint32_t>(beams);
  for (int i = 0; i < beams; ++i) {
    const double a = 2.0 * kPi * i / beams;
    double r = cast(w, T_w_l.x, T_w_l.y, T_w_l.yaw + a, max_range);
    const double n = noise(rng);
    if (r > 0.0 && sigma_m > 0.0) {
      r += n;
    }
    out.points[i].range_m = static_cast<float>(r);
    out.points[i].angle_rad = static_cast<float>(a);
    out.points[i].t_offset_ns = 0;
  }
}

}  // namespace sim
}  // namespace mss2d
