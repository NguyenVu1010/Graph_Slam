#pragma once

// Synthetic point-to-plane scenes for the degeneracy component tests (D-01..D-03).
//
// The sensor sits at the origin of l with T_w_l = I, so the scan matcher's Jacobian row
// (scan_matcher.hpp, R_w_l = I) is J_r_xi = [n ; -(n^T skew(p))^T] = [n ; p x n] and the
// Gauss-Newton Hessian is H = sum J_r_xi J_r_xi^T (all IRLS weights 1). Tangent [rho, phi]
// of the right perturbation of T_w_l. Grids are 0.25 m with samples at cell centres.

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"

namespace mss::test {

struct PlaneSample {
  common::Vec3 p_l;  // m, point on the plane in l
  common::Vec3 n_l;  // unit plane normal in l
};

namespace detail {
constexpr double kCellM = 0.25;  // m, grid spacing

inline double cell(double start_m, int i) {
  return start_m + kCellM * static_cast<double>(i);
}

// Floor z = -1 m (normal +z) over x in [-4, 4] m, y in [-3, 3] m: 32 x 24 samples.
inline void append_floor(std::vector<PlaneSample>& samples) {
  for (int i = 0; i <= 31; ++i) {
    for (int j = 0; j <= 23; ++j) {
      samples.push_back(
          {common::Vec3(cell(-3.875, i), cell(-2.875, j), -1.0), common::Vec3::UnitZ()});
    }
  }
}
}  // namespace detail

/// Corridor along x (before rotation): walls y = +-1.5 m (normals inward, -+y), floor
/// z = -1 m (normal +z), x in [-10, 10] m. Points and normals are then rotated by
/// R_z(yaw_rad), so the corridor axis in l is R_z(yaw_rad) e_x. 2916 samples.
inline std::vector<PlaneSample> make_corridor_scene(double yaw_rad) {
  const common::Mat3 R_l_c = Eigen::AngleAxisd(yaw_rad, common::Vec3::UnitZ()).toRotationMatrix();
  std::vector<PlaneSample> samples;
  samples.reserve(2916);
  for (int i = 0; i <= 80; ++i) {
    const double x = detail::cell(-10.0, i);
    for (int j = 0; j <= 11; ++j) {
      const double z = detail::cell(-0.875, j);
      samples.push_back({common::Vec3(x, 1.5, z), -common::Vec3::UnitY()});
      samples.push_back({common::Vec3(x, -1.5, z), common::Vec3::UnitY()});
      const double y = detail::cell(-1.375, j);
      samples.push_back({common::Vec3(x, y, -1.0), common::Vec3::UnitZ()});
    }
  }
  for (PlaneSample& s : samples) {
    s.p_l = R_l_c * s.p_l;
    s.n_l = R_l_c * s.n_l;
  }
  return samples;
}

/// Closed room 8 x 6 x 3 m: floor z = -1, ceiling z = 2, walls x = +-4 and y = +-3 (m).
/// Normals point inward. 2880 samples.
inline std::vector<PlaneSample> make_room_scene() {
  std::vector<PlaneSample> samples;
  samples.reserve(2880);
  detail::append_floor(samples);
  for (int i = 0; i <= 31; ++i) {
    for (int j = 0; j <= 23; ++j) {
      samples.push_back({common::Vec3(detail::cell(-3.875, i), detail::cell(-2.875, j), 2.0),
                         -common::Vec3::UnitZ()});
    }
  }
  for (int k = 0; k <= 11; ++k) {
    const double z = detail::cell(-0.875, k);
    for (int j = 0; j <= 23; ++j) {
      const double y = detail::cell(-2.875, j);
      samples.push_back({common::Vec3(4.0, y, z), -common::Vec3::UnitX()});
      samples.push_back({common::Vec3(-4.0, y, z), common::Vec3::UnitX()});
    }
    for (int i = 0; i <= 31; ++i) {
      const double x = detail::cell(-3.875, i);
      samples.push_back({common::Vec3(x, 3.0, z), -common::Vec3::UnitY()});
      samples.push_back({common::Vec3(x, -3.0, z), common::Vec3::UnitY()});
    }
  }
  return samples;
}

/// The room's floor only (single plane z = -1 m, normal +z). 768 samples.
inline std::vector<PlaneSample> make_floor_scene() {
  std::vector<PlaneSample> samples;
  samples.reserve(768);
  detail::append_floor(samples);
  return samples;
}

/// H = sum J J^T over the samples after perturbing them with std::mt19937(seed):
/// p += N(0, point_sigma_m) per axis, n = normalize(n + N(0, normal_sigma)) per axis.
inline common::Mat6 point_to_plane_hessian(const std::vector<PlaneSample>& samples,
                                           double point_sigma_m,
                                           double normal_sigma,
                                           std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  common::Mat6 H = common::Mat6::Zero();
  for (const PlaneSample& s : samples) {
    const common::Vec3 p_noise(unit(rng), unit(rng), unit(rng));
    const common::Vec3 n_noise(unit(rng), unit(rng), unit(rng));
    const common::Vec3 p_l = s.p_l + point_sigma_m * p_noise;
    const common::Vec3 n_l = (s.n_l + normal_sigma * n_noise).normalized();
    common::Vec6 J_r_xi;
    J_r_xi.head<3>() = n_l;
    J_r_xi.tail<3>() = -(n_l.transpose() * common::skew(p_l)).transpose();
    H.noalias() += J_r_xi * J_r_xi.transpose();
  }
  return H;
}

}  // namespace mss::test
