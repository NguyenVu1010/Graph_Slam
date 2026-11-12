#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/points2d.hpp"
#include "mss2d/pose2.hpp"

namespace mss2d {

constexpr int kMaxGridCells = kMaxGridSide * kMaxGridSide;

/// Square occupancy submap in its own frame s. Cell (ix, iy) covers
/// [(ix - side/2) res, (ix + 1 - side/2) res) in x (same in y), so the frame origin is the
/// grid centre. T_w_s places the submap in the world. Log-odds are int8 in steps of
/// max_log_odds / 127, clamped to [-127, 127]; 0 = unknown (p = 0.5).
/// About 46 KB: give it static storage.
class OccupancyGrid {
 public:
  /// `cfg` must have passed GridConfig::validate().
  explicit OccupancyGrid(const GridConfig& cfg);

  /// Clears every cell to unknown and sets the submap pose.
  void reset(const Pose2& T_w_s);

  const Pose2& T_w_s() const { return T_w_s_; }
  int side() const { return side_; }
  float resolution() const { return res_; }

  /// Cell containing the s-frame point p_s; false when outside the grid.
  bool cell_of(const Vec2f& p_s, int& ix, int& iy) const;
  /// Centre of cell (ix, iy) in s, m.
  Vec2f cell_center(int ix, int iy) const;
  bool in_bounds(int ix, int iy) const { return ix >= 0 && iy >= 0 && ix < side_ && iy < side_; }

  /// Quantised log-odds in [-127, 127]. Cell must be in bounds.
  std::int8_t log_odds(int ix, int iy) const { return cells_[index(ix, iy)]; }
  /// Occupancy probability from a table (no exp per call). Cell must be in bounds.
  float probability(int ix, int iy) const { return prob_lut_[lut_index(log_odds(ix, iy))]; }
  /// Probability, or 0 outside the grid (correlative score).
  float probability_or_zero(int ix, int iy) const {
    return in_bounds(ix, iy) ? probability(ix, iy) : 0.0F;
  }
  bool occupied(int ix, int iy) const { return log_odds(ix, iy) > occupied_q_; }

  /// Inserts one scan seen from the sensor pose T_s_l: every cell a beam crosses gets a
  /// miss, the end cell a hit (Bresenham from the sensor cell). Each cell is updated at
  /// most once per scan and a hit wins over a miss. A sensor outside the grid inserts
  /// nothing; a beam ending outside updates the crossed cells inside.
  void insert(const PointCloud2d& pts_l, const Pose2& T_s_l);

  /// Quantised increments (tests).
  int hit_step() const { return hit_q_; }
  int miss_step() const { return miss_q_; }

 private:
  int index(int ix, int iy) const { return iy * side_ + ix; }
  static int lut_index(std::int8_t lo) { return static_cast<int>(lo) + 128; }
  bool mark(int idx);  ///< true if idx was not yet updated in this scan

  GridConfig cfg_;
  int side_;
  float res_;
  float inv_res_;
  Pose2 T_w_s_{};
  int hit_q_;
  int miss_q_;
  int occupied_q_;
  std::array<std::int8_t, kMaxGridCells> cells_{};
  std::array<std::uint32_t, (kMaxGridCells + 31) / 32> updated_{};  ///< per-scan marks
  std::array<float, 256> prob_lut_{};
};

}  // namespace mss2d
