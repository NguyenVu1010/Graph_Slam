#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/occupancy_grid.hpp"
#include "mss2d/small_matrix.hpp"

namespace mss2d {

namespace detail {

/// Bilinear interpolation on the unit square: v00 at (0,0), v10 at (1,0), v01 at (0,1),
/// v11 at (1,1); fx, fy in [0, 1]. Returns the value and its derivatives wrt fx and fy.
template <typename S>
S bilinear(S v00, S v10, S v01, S v11, S fx, S fy, S& d_fx, S& d_fy) {
  d_fx = (1 - fy) * (v10 - v00) + fy * (v11 - v01);
  d_fy = (1 - fx) * (v01 - v00) + fx * (v11 - v10);
  return (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v10 + (1 - fx) * fy * v01 + fx * fy * v11;
}

}  // namespace detail

/// Euclidean distance (m) from each cell centre to the nearest occupied cell centre,
/// clamped to max_distance_m. Exact EDT (Felzenszwalb-Huttenlocher, separable: an exact 1D
/// pass per column, then the lower envelope of parabolas per row). Stored as uint8 in steps
/// of max_distance_m / 255 (2 mm at the 0.5 m default). About 42 KB: static storage.
class DistanceField {
 public:
  /// `cfg` must have passed GridConfig::validate().
  explicit DistanceField(const GridConfig& cfg);

  /// Recomputes the field from `grid` (cells with log-odds above occupied_log_odds).
  void compute(const OccupancyGrid& grid);

  float max_distance() const { return max_d_; }
  /// Stored (quantised) distance of cell (ix, iy), m. Cell must be in bounds.
  float at_cell(int ix, int iy) const { return static_cast<float>(q_[iy * side_ + ix]) * step_; }

  /// Bilinear interpolation between cell centres at the s-frame point p_s. False when p_s
  /// is outside the interpolation domain (the outer half cell). (d_dx, d_dy) is the
  /// gradient of the interpolant (unitless).
  bool sample(const Vec2f& p_s, float& d, float& d_dx, float& d_dy) const;

 private:
  int side_;
  float res_;
  float inv_res_;
  float max_d_;
  float step_;
  std::array<std::uint8_t, kMaxGridCells> q_{};
  // Per-row scratch for the parabola envelope.
  std::array<float, kMaxGridSide> f_{};
  std::array<int, kMaxGridSide> v_{};
  std::array<float, kMaxGridSide + 1> z_{};
};

}  // namespace mss2d
