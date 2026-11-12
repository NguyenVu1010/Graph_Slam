#include "mss2d/distance_field.hpp"

#include <cmath>

namespace mss2d {

DistanceField::DistanceField(const GridConfig& cfg)
    : side_(cfg.side_cells),
      res_(cfg.resolution_m),
      inv_res_(1.0F / cfg.resolution_m),
      max_d_(cfg.max_distance_m),
      step_(cfg.max_distance_m / 255.0F) {}

void DistanceField::compute(const OccupancyGrid& grid) {
  // Distances are in cells until the final conversion. Anything beyond `cap` cells is
  // clamped anyway, so the column pass stores min(distance, cap) in the uint8 array itself
  // (cap <= 251 by GridConfig::validate()) and the row pass needs only per-row scratch.
  const int cap = static_cast<int>(std::ceil(max_d_ * inv_res_)) + 1;

  // Pass 1, per column: exact 1D distance to the nearest occupied cell (two sweeps).
  for (int x = 0; x < side_; ++x) {
    int d = cap;
    for (int y = 0; y < side_; ++y) {
      d = grid.occupied(x, y) ? 0 : (d < cap ? d + 1 : cap);
      q_[y * side_ + x] = static_cast<std::uint8_t>(d);
    }
    d = cap;
    for (int y = side_ - 1; y >= 0; --y) {
      std::uint8_t& g = q_[y * side_ + x];
      d = g == 0 ? 0 : (d < cap ? d + 1 : cap);
      if (d < g) {
        g = static_cast<std::uint8_t>(d);
      }
    }
  }

  // Pass 2, per row: D(x) = min_x' (x - x')^2 + g(x')^2 by the lower envelope of parabolas
  // (Felzenszwalb and Huttenlocher 2012). Exact for every value below cap; values at or
  // above cap stay >= cap and are clamped below.
  const float inf = 1e30F;
  const float scale = res_ / step_;  // cells -> quantisation steps
  for (int y = 0; y < side_; ++y) {
    std::uint8_t* row = &q_[y * side_];
    for (int x = 0; x < side_; ++x) {
      const float g = static_cast<float>(row[x]);
      f_[x] = g * g;
    }
    int k = 0;
    v_[0] = 0;
    z_[0] = -inf;
    z_[1] = inf;
    for (int x = 1; x < side_; ++x) {
      const float fx = f_[x] + static_cast<float>(x * x);
      float s = 0.0F;
      while (true) {
        const int vk = v_[k];
        s = (fx - (f_[vk] + static_cast<float>(vk * vk))) / static_cast<float>(2 * (x - vk));
        if (s > z_[k]) {  // always true at k == 0 (z_[0] = -inf)
          break;
        }
        --k;
      }
      ++k;
      v_[k] = x;
      z_[k] = s;
      z_[k + 1] = inf;
    }
    k = 0;
    for (int x = 0; x < side_; ++x) {
      while (z_[k + 1] < static_cast<float>(x)) {
        ++k;
      }
      const int dx = x - v_[k];
      const float d_cells = std::sqrt(static_cast<float>(dx * dx) + f_[v_[k]]);
      const float q = d_cells * scale;
      row[x] = q >= 255.0F ? std::uint8_t{255} : static_cast<std::uint8_t>(std::lround(q));
    }
  }
}

bool DistanceField::sample(const Vec2f& p_s, float& d, float& d_dx, float& d_dy) const {
  // Continuous cell coordinates with cell centres at integers.
  const float u = p_s.x * inv_res_ + 0.5F * static_cast<float>(side_) - 0.5F;
  const float v = p_s.y * inv_res_ + 0.5F * static_cast<float>(side_) - 0.5F;
  const float hi = static_cast<float>(side_ - 1);
  if (!(u >= 0.0F && v >= 0.0F && u < hi && v < hi)) {
    return false;
  }
  const int i = static_cast<int>(u);
  const int j = static_cast<int>(v);
  const float fx = u - static_cast<float>(i);
  const float fy = v - static_cast<float>(j);
  const std::uint8_t* r0 = &q_[j * side_ + i];
  const std::uint8_t* r1 = r0 + side_;
  float d_fx = 0.0F;
  float d_fy = 0.0F;
  d = step_ * detail::bilinear(static_cast<float>(r0[0]),
                               static_cast<float>(r0[1]),
                               static_cast<float>(r1[0]),
                               static_cast<float>(r1[1]),
                               fx,
                               fy,
                               d_fx,
                               d_fy);
  d_dx = step_ * inv_res_ * d_fx;
  d_dy = step_ * inv_res_ * d_fy;
  return true;
}

}  // namespace mss2d
