#include "mss2d/occupancy_grid.hpp"

#include <cmath>
#include <cstdlib>

namespace mss2d {

namespace {

constexpr int kLogOddsMaxQ = 127;  // int8 storage range used: [-127, 127]

int quantise(float log_odds, float step) {
  return static_cast<int>(std::lround(log_odds / step));
}

}  // namespace

OccupancyGrid::OccupancyGrid(const GridConfig& cfg)
    : cfg_(cfg),
      side_(cfg.side_cells),
      res_(cfg.resolution_m),
      inv_res_(1.0F / cfg.resolution_m),
      hit_q_(0),
      miss_q_(0),
      occupied_q_(0) {
  const float step = cfg.max_log_odds / static_cast<float>(kLogOddsMaxQ);
  hit_q_ = quantise(cfg.hit_log_odds, step);
  miss_q_ = quantise(cfg.miss_log_odds, step);
  occupied_q_ = quantise(cfg.occupied_log_odds, step);
  for (int i = 0; i < 256; ++i) {
    const float lo = static_cast<float>(i - 128) * step;
    prob_lut_[i] = 1.0F / (1.0F + std::exp(-lo));
  }
}

void OccupancyGrid::reset(const Pose2& T_w_s) {
  T_w_s_ = T_w_s;
  cells_.fill(0);
}

bool OccupancyGrid::cell_of(const Vec2f& p_s, int& ix, int& iy) const {
  const float u = p_s.x * inv_res_ + 0.5F * static_cast<float>(side_);
  const float v = p_s.y * inv_res_ + 0.5F * static_cast<float>(side_);
  // Negated so NaN fails; also keeps the int conversion in range.
  if (!(u >= 0.0F && v >= 0.0F && u < static_cast<float>(side_) && v < static_cast<float>(side_))) {
    return false;
  }
  ix = static_cast<int>(u);
  iy = static_cast<int>(v);
  return ix < side_ && iy < side_;
}

Vec2f OccupancyGrid::cell_center(int ix, int iy) const {
  const float h = 0.5F * static_cast<float>(side_);
  return Vec2f{(static_cast<float>(ix) + 0.5F - h) * res_,
               (static_cast<float>(iy) + 0.5F - h) * res_};
}

bool OccupancyGrid::mark(int idx) {
  std::uint32_t& word = updated_[static_cast<std::size_t>(idx) >> 5];
  const std::uint32_t bit = 1u << (static_cast<std::uint32_t>(idx) & 31u);
  if ((word & bit) != 0u) {
    return false;
  }
  word |= bit;
  return true;
}

namespace {

void add_clamped(std::int8_t& cell, int delta) {
  int v = static_cast<int>(cell) + delta;
  v = v > kLogOddsMaxQ ? kLogOddsMaxQ : (v < -kLogOddsMaxQ ? -kLogOddsMaxQ : v);
  cell = static_cast<std::int8_t>(v);
}

}  // namespace

void OccupancyGrid::insert(const PointCloud2d& pts_l, const Pose2& T_s_l) {
  int ox = 0;
  int oy = 0;
  if (!cell_of(Vec2f{T_s_l.x, T_s_l.y}, ox, oy)) {
    return;
  }
  updated_.fill(0u);
  const float c = std::cos(T_s_l.yaw);
  const float s = std::sin(T_s_l.yaw);
  const float h = 0.5F * static_cast<float>(side_);

  // Pass 1: hits, so that a miss from another beam cannot touch a hit cell in this scan.
  for (std::uint32_t i = 0; i < pts_l.count; ++i) {
    const Vec2f& p = pts_l.pts[i];
    int ex = 0;
    int ey = 0;
    if (cell_of(Vec2f{T_s_l.x + c * p.x - s * p.y, T_s_l.y + s * p.x + c * p.y}, ex, ey)) {
      const int idx = index(ex, ey);
      if (mark(idx)) {
        add_clamped(cells_[idx], hit_q_);
      }
    }
  }

  // Pass 2: misses along each beam (Bresenham, end cell excluded), each cell once.
  for (std::uint32_t i = 0; i < pts_l.count; ++i) {
    const Vec2f& p = pts_l.pts[i];
    // End cell in unbounded integer coordinates (may lie outside the grid). The clamp only
    // keeps the float -> int conversion defined; a 1e6-cell beam is never inside anyway.
    const float fu = std::floor((T_s_l.x + c * p.x - s * p.y) * inv_res_ + h);
    const float fv = std::floor((T_s_l.y + s * p.x + c * p.y) * inv_res_ + h);
    if (!(std::abs(fu) < 1e6F && std::abs(fv) < 1e6F)) {
      continue;
    }
    const int ex = static_cast<int>(fu);
    const int ey = static_cast<int>(fv);
    const int dx = std::abs(ex - ox);
    const int dy = -std::abs(ey - oy);
    const int sx = ox < ex ? 1 : -1;
    const int sy = oy < ey ? 1 : -1;
    int err = dx + dy;
    int x = ox;
    int y = oy;
    while (!(x == ex && y == ey)) {
      if (!in_bounds(x, y)) {
        break;  // left the grid: it cannot come back on a straight line
      }
      const int idx = index(x, y);
      if (mark(idx)) {
        add_clamped(cells_[idx], miss_q_);
      }
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
  }
}

}  // namespace mss2d
