#include "mss2d/loop_closer.hpp"

#include <cmath>

#include "mss2d/degeneracy.hpp"

namespace mss2d {

namespace {

constexpr int kMinQueryPoints = 16;
constexpr int kSmoothRadius = 2;  ///< cells
constexpr float kVMax = 255.0F;

/// floor() into int, saturated so the conversion is defined for any input.
std::int32_t floor_cell(float u) {
  constexpr float kLimit = 1e6F;
  if (!(u > -kLimit)) {
    return -static_cast<std::int32_t>(kLimit);
  }
  if (u > kLimit) {
    return static_cast<std::int32_t>(kLimit);
  }
  return static_cast<std::int32_t>(std::floor(u));
}

}  // namespace

void compact_submap(const OccupancyGrid& grid, CompactSubmap& out) {
  const int d = kLoopGridDownsample;
  const int side = grid.side() / d;
  out.bits.fill(0U);
  for (int y = 0; y < grid.side(); ++y) {
    for (int x = 0; x < grid.side(); ++x) {
      if (grid.occupied(x, y)) {
        const int k = (y / d) * side + x / d;
        out.bits[static_cast<std::size_t>(k >> 3)] |= static_cast<std::uint8_t>(1U << (k & 7));
      }
    }
  }
}

LoopMatcher::LoopMatcher(const Config& cfg)
    : cfg_(cfg.loop),
      side_(cfg.grid.side_cells / kLoopGridDownsample),
      res_(cfg.grid.resolution_m * static_cast<float>(kLoopGridDownsample)),
      inv_res_(1.0F / res_) {
  for (int d2 = 0; d2 <= 8; ++d2) {
    // Gaussian of the distance, sigma one cell.
    lut_[d2] = static_cast<int>(std::lround(kVMax * std::exp(-0.5F * static_cast<float>(d2))));
  }
}

void LoopMatcher::smooth(const CompactSubmap& map) {
  v_.fill(0U);
  for (int y = 0; y < side_; ++y) {
    for (int x = 0; x < side_; ++x) {
      if (!map.occupied(x, y, side_)) {
        continue;
      }
      for (int dy = -kSmoothRadius; dy <= kSmoothRadius; ++dy) {
        for (int dx = -kSmoothRadius; dx <= kSmoothRadius; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          const int d2 = dx * dx + dy * dy;
          if (nx < 0 || ny < 0 || nx >= side_ || ny >= side_ || d2 > 8) {
            continue;
          }
          std::uint8_t& v = v_[static_cast<std::size_t>(ny * side_ + nx)];
          if (lut_[d2] > v) {
            v = static_cast<std::uint8_t>(lut_[d2]);
          }
        }
      }
    }
  }
  // pool(a, b) = max V over [a, a + kBlock) x [b, b + kBlock), a, b in [1 - kBlock, side).
  const int ps = side_ + kBlock - 1;
  for (int b = 0; b < ps; ++b) {
    for (int a = 0; a < ps; ++a) {
      int m = 0;
      for (int j = 0; j < kBlock; ++j) {
        for (int i = 0; i < kBlock; ++i) {
          const int v = v_at(a - (kBlock - 1) + i, b - (kBlock - 1) + j);
          m = v > m ? v : m;
        }
      }
      pool_[static_cast<std::size_t>(b * ps + a)] = static_cast<std::uint8_t>(m);
    }
  }
}

int LoopMatcher::pool_at(int ix, int iy) const {
  const int a = ix + kBlock - 1;
  const int b = iy + kBlock - 1;
  const int ps = side_ + kBlock - 1;
  return a >= 0 && b >= 0 && a < ps && b < ps ? pool_[static_cast<std::size_t>(b * ps + a)] : 0;
}

bool LoopMatcher::sample(const Vec2f& p_s, float& v, float& dv_dx, float& dv_dy) const {
  const float u = p_s.x * inv_res_ + 0.5F * static_cast<float>(side_) - 0.5F;
  const float w = p_s.y * inv_res_ + 0.5F * static_cast<float>(side_) - 0.5F;
  const float hi = static_cast<float>(side_ - 1);
  if (!(u >= 0.0F && w >= 0.0F && u < hi && w < hi)) {
    return false;
  }
  const int i = static_cast<int>(u);
  const int j = static_cast<int>(w);
  const float fx = u - static_cast<float>(i);
  const float fy = w - static_cast<float>(j);
  const float v00 = static_cast<float>(v_at(i, j));
  const float v10 = static_cast<float>(v_at(i + 1, j));
  const float v01 = static_cast<float>(v_at(i, j + 1));
  const float v11 = static_cast<float>(v_at(i + 1, j + 1));
  const float d_fx = (1.0F - fy) * (v10 - v00) + fy * (v11 - v01);
  const float d_fy = (1.0F - fx) * (v01 - v00) + fx * (v11 - v10);
  v = ((1.0F - fx) * (1.0F - fy) * v00 + fx * (1.0F - fy) * v10 + (1.0F - fx) * fy * v01 +
       fx * fy * v11) /
      kVMax;
  dv_dx = d_fx * inv_res_ / kVMax;
  dv_dy = d_fy * inv_res_ / kVMax;
  return true;
}

float LoopMatcher::score_at(const Pose2& T) const {
  float sum = 0.0F;
  for (int m = 0; m < n_q_; ++m) {
    float v = 0.0F;
    float gx = 0.0F;
    float gy = 0.0F;
    if (sample(T * q_[static_cast<std::size_t>(m)], v, gx, gy)) {
      sum += v;
    }
  }
  return sum / static_cast<float>(n_q_);
}

LoopMatch LoopMatcher::match(const CompactSubmap& map,
                             const PointCloud2d& pts_l,
                             const Pose2& guess,
                             float window_m,
                             float window_rad) {
  LoopMatch out;
  out.T_s_l = guess;
  // Query: an even stride through the scan, at most max_query_points.
  const int n = static_cast<int>(pts_l.count);
  const int stride = (n + cfg_.max_query_points - 1) / cfg_.max_query_points;
  n_q_ = 0;
  for (int k = 0; stride > 0 && k < n; k += stride) {
    q_[static_cast<std::size_t>(n_q_++)] = pts_l.pts[static_cast<std::size_t>(k)];
  }
  out.n_points = n_q_;
  if (n_q_ < kMinQueryPoints) {
    out.status = LoopStatus::kTooFewPoints;
    return out;
  }
  smooth(map);

  // Branch and bound on the lattice guess + (i res, j res, k step). Sums are in V units.
  const int w_cells = static_cast<int>(std::floor(window_m * inv_res_ + 1e-3F));
  const int n_ang = static_cast<int>(std::floor(window_rad / cfg_.angle_step_rad + 1e-3F));
  const float half = 0.5F * static_cast<float>(side_);
  int best = static_cast<int>(std::ceil(cfg_.min_score * kVMax * static_cast<float>(n_q_))) - 1;
  bool found = false;
  int best_d2 = 0;
  int best_i = 0;
  int best_j = 0;
  int best_k = 0;
  for (int k = -n_ang; k <= n_ang; ++k) {
    const float yaw = guess.yaw + static_cast<float>(k) * cfg_.angle_step_rad;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    for (int m = 0; m < n_q_; ++m) {
      const Vec2f& p = q_[static_cast<std::size_t>(m)];
      cu_[static_cast<std::size_t>(m)] =
          floor_cell((guess.x + c * p.x - s * p.y) * inv_res_ + half);
      cv_[static_cast<std::size_t>(m)] =
          floor_cell((guess.y + s * p.x + c * p.y) * inv_res_ + half);
    }
    for (int bj = -w_cells; bj <= w_cells; bj += kBlock) {
      for (int bi = -w_cells; bi <= w_cells; bi += kBlock) {
        int bound = 0;
        for (int m = 0; m < n_q_; ++m) {
          bound +=
              pool_at(cu_[static_cast<std::size_t>(m)] + bi, cv_[static_cast<std::size_t>(m)] + bj);
        }
        if (bound < best) {
          continue;  // no shift of this block can reach the best (or the threshold)
        }
        for (int j = bj; j < bj + kBlock && j <= w_cells; ++j) {
          for (int i = bi; i < bi + kBlock && i <= w_cells; ++i) {
            int sum = 0;
            for (int m = 0; m < n_q_; ++m) {
              sum +=
                  v_at(cu_[static_cast<std::size_t>(m)] + i, cv_[static_cast<std::size_t>(m)] + j);
            }
            const int d2 = i * i + j * j + k * k;
            if (sum > best || (found && sum == best && d2 < best_d2)) {
              best = sum;
              found = true;
              best_d2 = d2;
              best_i = i;
              best_j = j;
              best_k = k;
            }
          }
        }
      }
    }
  }
  if (!found) {
    out.status = LoopStatus::kLowScore;
    return out;
  }
  out.coarse_score = static_cast<float>(best) / (kVMax * static_cast<float>(n_q_));

  // Gauss-Newton on r = 1 - V(T p), right perturbation; J_r_delta = -grad V^T [R, R perp(p)]
  // (same form as residual_jacobian). A rising cost halves the step from the last pose.
  Pose2 T(guess.x + static_cast<float>(best_i) * res_,
          guess.y + static_cast<float>(best_j) * res_,
          guess.yaw + static_cast<float>(best_k) * cfg_.angle_step_rad);
  Pose2 prev_T = T;
  float prev_cost = 0.0F;
  Vec3f delta;
  Mat3f h_last;
  for (int it = 0; it < cfg_.refine_iterations; ++it) {
    Mat3f h;
    float g[3] = {0.0F, 0.0F, 0.0F};
    float cost = 0.0F;
    const float c = std::cos(T.yaw);
    const float s = std::sin(T.yaw);
    for (int m = 0; m < n_q_; ++m) {
      const Vec2f& p = q_[static_cast<std::size_t>(m)];
      float v = 0.0F;
      float gx = 0.0F;
      float gy = 0.0F;
      if (!sample(T * p, v, gx, gy)) {
        continue;
      }
      const float r = 1.0F - v;
      Vec3f j;
      j[0] = -(gx * c + gy * s);
      j[1] = -(-gx * s + gy * c);
      j[2] = -j[0] * p.y + j[1] * p.x;
      for (int a = 0; a < 3; ++a) {
        for (int b = a; b < 3; ++b) {
          h(a, b) += j[a] * j[b];
        }
        g[a] += j[a] * r;
      }
      cost += r * r;
    }
    if (it > 0 && cost > prev_cost) {
      for (int a = 0; a < 3; ++a) {
        delta[a] *= 0.5F;
      }
      T = prev_T * Pose2::exp(delta);
      continue;
    }
    h(1, 0) = h(0, 1);
    h(2, 0) = h(0, 2);
    h(2, 1) = h(1, 2);
    h_last = h;
    // Step on the well-conditioned eigen-directions only.
    const SymEigen3 e = sym_eigen3(h);
    delta = Vec3f{};
    const float floor = 1e-6F * e.values[2];
    for (int k = 0; k < 3; ++k) {
      if (!(e.values[k] > floor)) {
        continue;
      }
      const float vg = e.vectors(0, k) * g[0] + e.vectors(1, k) * g[1] + e.vectors(2, k) * g[2];
      for (int a = 0; a < 3; ++a) {
        delta[a] -= vg / e.values[k] * e.vectors(a, k);
      }
    }
    prev_T = T;
    prev_cost = cost;
    T = T * Pose2::exp(delta);
  }
  // Keep the last accepted iterate if the final step went uphill.
  if (score_at(T) < score_at(prev_T)) {
    T = prev_T;
  }
  out.T_s_l = T;
  out.score = score_at(T);

  // Translation block: lambda_min / lambda_max of [[a, b], [b, d]].
  const float a = h_last(0, 0);
  const float b = h_last(0, 1);
  const float d = h_last(1, 1);
  const float mean = 0.5F * (a + d);
  const float rad = std::sqrt(0.25F * (a - d) * (a - d) + b * b);
  out.eig_ratio = mean + rad > 0.0F ? (mean - rad) / (mean + rad) : 0.0F;

  const float dx = T.x - guess.x;
  const float dy = T.y - guess.y;
  if (out.score < cfg_.min_score) {
    out.status = LoopStatus::kLowScore;
  } else if (out.eig_ratio < cfg_.min_eig_ratio) {
    out.status = LoopStatus::kDegenerate;
  } else if (std::abs(dx) > window_m + res_ || std::abs(dy) > window_m + res_ ||
             std::abs(normalize_angle(T.yaw - guess.yaw)) > window_rad + cfg_.angle_step_rad) {
    out.status = LoopStatus::kOutsideWindow;
  } else {
    out.status = LoopStatus::kAccepted;
  }
  return out;
}

}  // namespace mss2d
