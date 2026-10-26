#pragma once

// Synthetic scenes for the scan_match component tests (known pose). Header-only, test code.

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/voxel_map.hpp"

namespace mss::test {

// Cell-centred samples of [lo, hi) with step `spacing_m`: lo + (i + 0.5) spacing.
inline std::vector<double> cell_centres(double lo, double hi, double spacing_m) {
  std::vector<double> v;
  const int n = static_cast<int>(std::lround((hi - lo) / spacing_m));
  v.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    v.push_back(lo + (i + 0.5) * spacing_m);
  }
  return v;
}

// Faces of the box x in [-4, 4], y in [-3, 3], z in [-1, 2] m, cell-centred grid per face.
inline std::vector<common::Vec3> room_points_w(double spacing_m) {
  constexpr double kX = 4.0;
  constexpr double kY = 3.0;
  constexpr double kZLo = -1.0;
  constexpr double kZHi = 2.0;
  const std::vector<double> xs = cell_centres(-kX, kX, spacing_m);
  const std::vector<double> ys = cell_centres(-kY, kY, spacing_m);
  const std::vector<double> zs = cell_centres(kZLo, kZHi, spacing_m);
  std::vector<common::Vec3> p;
  for (const double x : {-kX, kX}) {
    for (const double y : ys) {
      for (const double z : zs) {
        p.emplace_back(x, y, z);
      }
    }
  }
  for (const double y : {-kY, kY}) {
    for (const double x : xs) {
      for (const double z : zs) {
        p.emplace_back(x, y, z);
      }
    }
  }
  for (const double z : {kZLo, kZHi}) {
    for (const double x : xs) {
      for (const double y : ys) {
        p.emplace_back(x, y, z);
      }
    }
  }
  return p;
}

// Corridor along x: walls y = +-1.5 (z in [-1, 2]) and floor z = -1 (y in [-1.5, 1.5]),
// x in [-half_length_m, half_length_m], open ends.
inline std::vector<common::Vec3> corridor_points_w(double spacing_m, double half_length_m) {
  constexpr double kHalfWidth = 1.5;
  constexpr double kZLo = -1.0;
  constexpr double kZHi = 2.0;
  const std::vector<double> xs = cell_centres(-half_length_m, half_length_m, spacing_m);
  const std::vector<double> ys = cell_centres(-kHalfWidth, kHalfWidth, spacing_m);
  const std::vector<double> zs = cell_centres(kZLo, kZHi, spacing_m);
  std::vector<common::Vec3> p;
  for (const double y : {-kHalfWidth, kHalfWidth}) {
    for (const double x : xs) {
      for (const double z : zs) {
        p.emplace_back(x, y, z);
      }
    }
  }
  for (const double x : xs) {
    for (const double y : ys) {
      p.emplace_back(x, y, kZLo);
    }
  }
  return p;
}

// Points (x, 0, 0), x cell-centred in [-half_length_m, half_length_m].
inline std::vector<common::Vec3> line_points_w(double spacing_m, double half_length_m) {
  std::vector<common::Vec3> p;
  for (const double x : cell_centres(-half_length_m, half_length_m, spacing_m)) {
    p.emplace_back(x, 0.0, 0.0);
  }
  return p;
}

// Writes `T_w_l^-1 * p_w` as float into `scan` (offsets 0, ring 0, intensity 0).
// Precondition: scan.capacity() >= p_w.size().
inline void fill_scan(common::ScanFrame& scan,
                      const std::vector<common::Vec3>& p_w,
                      const common::Pose& T_w_l) {
  const common::Pose T_l_w = T_w_l.inverse();
  float* xyz = scan.xyz_mut();
  for (std::size_t i = 0; i < p_w.size(); ++i) {
    const common::Vec3 p_l = T_l_w * p_w[i];
    xyz[3 * i] = static_cast<float>(p_l.x());
    xyz[3 * i + 1] = static_cast<float>(p_l.y());
    xyz[3 * i + 2] = static_cast<float>(p_l.z());
    scan.intensity_mut()[i] = 0.0F;
    scan.t_offset_ns_mut()[i] = 0U;
    scan.ring_mut()[i] = 0U;
  }
  scan.set_size(p_w.size());
}

// Inserts `p_w` into `map` as one ScanFrame with T_w_l = I.
inline void insert_points(map::VoxelMap& map, const std::vector<common::Vec3>& p_w) {
  common::ScanFrame scan(p_w.size());
  fill_scan(scan, p_w, common::Pose{});
  map.insert(scan, common::Pose{});
}

inline common::Vec3 random_unit(std::mt19937& rng) {
  std::normal_distribution<double> n01(0.0, 1.0);
  const double x = n01(rng);
  const double y = n01(rng);
  const double z = n01(rng);
  const common::Vec3 v(x, y, z);
  return v.normalized();
}

// Pose{T.R * Exp(rot_rad * u1), T.p + trans_m * u2}, u1, u2 random unit vectors.
inline common::Pose perturb(const common::Pose& T,
                            double trans_m,
                            double rot_rad,
                            std::mt19937& rng) {
  const common::Vec3 u1 = random_unit(rng);
  const common::Vec3 u2 = random_unit(rng);
  const common::Vec3 phi = rot_rad * u1;
  const common::Vec3 dp = trans_m * u2;
  return common::Pose{T.R * common::so3_exp(phi), T.p + dp};
}

struct PoseError {
  double trans_m;
  double rot_rad;
};

// |p_est - p_true| and |Log(R_true^T R_est)|.
inline PoseError pose_error(const common::Pose& est, const common::Pose& truth) {
  const common::Vec3 dp = est.p - truth.p;
  const common::Mat3 dR = truth.R.transpose() * est.R;
  const common::Vec3 dphi = common::so3_log(dR);
  return PoseError{dp.norm(), dphi.norm()};
}

}  // namespace mss::test
