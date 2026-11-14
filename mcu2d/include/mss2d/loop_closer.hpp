#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/occupancy_grid.hpp"
#include "mss2d/points2d.hpp"
#include "mss2d/pose2.hpp"

namespace mss2d {

/// A finished submap kept for loop closing: 1 bit per cell at kLoopGridDownsample times the
/// submap resolution, same frame s (origin at the grid centre). 1250 bytes at the defaults.
struct CompactSubmap {
  std::array<std::uint8_t, kLoopGridBytes> bits{};

  bool occupied(int ix, int iy, int side) const {
    const int k = iy * side + ix;
    return ((bits[static_cast<std::size_t>(k >> 3)] >> (k & 7)) & 1U) != 0U;
  }
};

/// Cell (I, J) of `out` is occupied iff any submap cell (D I + a, D J + b), a, b < D, is
/// (D = kLoopGridDownsample). The grid side must be a multiple of D (Config::validate).
void compact_submap(const OccupancyGrid& grid, CompactSubmap& out);

enum class LoopStatus : std::uint8_t {
  kAccepted = 0,
  kTooFewPoints,   ///< fewer than 16 query points
  kLowScore,       ///< no pose in the window reaches loop.min_score
  kDegenerate,     ///< the translation Hessian is too elongated (corridor): ambiguous
  kOutsideWindow,  ///< the refinement left the search window
};

struct LoopMatch {
  Pose2 T_s_l{};              ///< scan pose in the candidate submap frame
  float coarse_score = 0.0F;  ///< best lattice score, in [0, 1]
  float score = 0.0F;         ///< mean smoothed occupancy at the refined pose, in [0, 1]
  float eig_ratio = 0.0F;     ///< smallest / largest eigenvalue of the translation Hessian
  int n_points = 0;           ///< query points used
  LoopStatus status = LoopStatus::kTooFewPoints;
};

/// Verifies a loop candidate (docs/backend.md section 5): the scan is matched against a
/// CompactSubmap over a window around a guess. The bitmap is smoothed into an 8-bit
/// likelihood V (gaussian of the distance to the nearest occupied cell, sigma one cell,
/// radius two); a branch-and-bound correlative search over the (x, y, yaw) lattice finds the
/// best mean V (bound: a max-pooled V over 4 x 4 translation blocks; the acceptance
/// threshold seeds the bound), then Gauss-Newton on 1 - V refines below the lattice. About
/// 37 KB of scratch, no allocation. Single-threaded.
class LoopMatcher {
 public:
  /// `cfg` must have passed Config::validate().
  explicit LoopMatcher(const Config& cfg);

  LoopMatch match(const CompactSubmap& map,
                  const PointCloud2d& pts_l,
                  const Pose2& T_s_l_guess,
                  float window_m,
                  float window_rad);

  int side() const { return side_; }
  float resolution() const { return res_; }

 private:
  static constexpr int kBlock = 4;  ///< translation block of the bound, cells
  static constexpr int kPoolSide = kLoopGridSide + kBlock - 1;

  void smooth(const CompactSubmap& map);
  int v_at(int ix, int iy) const {
    return ix >= 0 && iy >= 0 && ix < side_ && iy < side_
               ? v_[static_cast<std::size_t>(iy * side_ + ix)]
               : 0;
  }
  int pool_at(int ix, int iy) const;
  /// Bilinear V / 255 between cell centres at p_s and its gradient (1/m); false outside.
  bool sample(const Vec2f& p_s, float& v, float& dv_dx, float& dv_dy) const;
  float score_at(const Pose2& T_s_l) const;

  LoopConfig cfg_;
  int side_;
  float res_;
  float inv_res_;
  int lut_[9]{};  ///< V by squared cell distance 0..8
  std::array<std::uint8_t, kLoopGridSide * kLoopGridSide> v_{};
  std::array<std::uint8_t, kPoolSide * kPoolSide> pool_{};
  std::array<Vec2f, kMaxScanPoints> q_{};
  int n_q_ = 0;
  std::array<std::int32_t, kMaxScanPoints> cu_{};
  std::array<std::int32_t, kMaxScanPoints> cv_{};
};

}  // namespace mss2d
