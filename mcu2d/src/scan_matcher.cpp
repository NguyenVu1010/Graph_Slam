#include "mss2d/scan_matcher.hpp"

#include <cmath>

namespace mss2d {

namespace {

/// floor() into int, saturated so the conversion is defined for any finite or NaN input.
std::int32_t floor_cell(float u) {
  constexpr float kLimit = 1e6F;  // far outside any grid
  if (!(u > -kLimit)) {
    return -static_cast<std::int32_t>(kLimit);
  }
  if (u > kLimit) {
    return static_cast<std::int32_t>(kLimit);
  }
  return static_cast<std::int32_t>(std::floor(u));
}

}  // namespace

MatchResult ScanMatcher::match(const OccupancyGrid& grid,
                               const DistanceField& df,
                               const PointCloud2d& pts_l,
                               const Pose2& T_s_l_guess) {
  if (pts_l.count < static_cast<std::uint32_t>(cfg_.min_points)) {
    MatchResult r;
    r.T_s_l = T_s_l_guess;
    r.status = MatchStatus::kTooFewPoints;
    return r;
  }
  float score = 0.0F;
  const Pose2 coarse = correlative_search(grid, pts_l, T_s_l_guess, score);
  MatchResult r = refine(df, pts_l, coarse);
  r.correlative_score = score;
  return r;
}

Pose2 ScanMatcher::correlative_search(const OccupancyGrid& grid,
                                      const PointCloud2d& pts_l,
                                      const Pose2& T_s_l_guess,
                                      float& best_score) {
  best_score = 0.0F;
  const std::uint32_t n = pts_l.count;
  if (n == 0) {
    return T_s_l_guess;
  }
  const float res = grid.resolution();
  const float inv_res = 1.0F / res;
  const float half = 0.5F * static_cast<float>(grid.side());
  // Small epsilon so 0.30 / 0.05 gives 6, not 5, after float rounding.
  const int n_lin = static_cast<int>(std::floor(cfg_.search_linear_m * inv_res + 1e-3F));
  const int n_ang =
      static_cast<int>(std::floor(cfg_.search_angle_rad / cfg_.search_angle_step_rad + 1e-3F));
  const float inv_n = 1.0F / static_cast<float>(n);

  float best = -1.0F;
  int best_dist2 = 0;
  int best_i = 0;
  int best_j = 0;
  int best_k = 0;
  for (int k = -n_ang; k <= n_ang; ++k) {
    // Rotate once per angle; a lattice translation is then an integer cell shift, so the
    // inner loops are table lookups (Olson 2009, real-time correlative scan matching).
    const float yaw = T_s_l_guess.yaw + static_cast<float>(k) * cfg_.search_angle_step_rad;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    for (std::uint32_t i = 0; i < n; ++i) {
      const Vec2f& p = pts_l.pts[i];
      cx_[i] = floor_cell((T_s_l_guess.x + c * p.x - s * p.y) * inv_res + half);
      cy_[i] = floor_cell((T_s_l_guess.y + s * p.x + c * p.y) * inv_res + half);
    }
    for (int j = -n_lin; j <= n_lin; ++j) {
      for (int i = -n_lin; i <= n_lin; ++i) {
        float sum = 0.0F;
        for (std::uint32_t m = 0; m < n; ++m) {
          sum += grid.probability_or_zero(cx_[m] + i, cy_[m] + j);
        }
        const float score = sum * inv_n;
        const int dist2 = i * i + j * j + k * k;  // in lattice steps; tie-break only
        if (score > best || (score == best && dist2 < best_dist2)) {
          best = score;
          best_dist2 = dist2;
          best_i = i;
          best_j = j;
          best_k = k;
        }
      }
    }
  }
  best_score = best;
  return Pose2(T_s_l_guess.x + static_cast<float>(best_i) * res,
               T_s_l_guess.y + static_cast<float>(best_j) * res,
               T_s_l_guess.yaw + static_cast<float>(best_k) * cfg_.search_angle_step_rad);
}

bool ScanMatcher::step_is_small(const Vec3f& delta) const {
  return std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]) < cfg_.converged_translation_m &&
         std::abs(delta[2]) < cfg_.converged_rotation_rad;
}

MatchResult ScanMatcher::refine(const DistanceField& df,
                                const PointCloud2d& pts_l,
                                const Pose2& start) {
  MatchResult r;
  r.T_s_l = start;
  // Points whose interpolated distance reaches the clamp carry no gradient and are
  // outliers for this submap: skip them (half a quantisation step below the clamp).
  const float d_skip = df.max_distance() * (1.0F - 0.5F / 255.0F);
  Pose2 T = start;
  Pose2 prev_T = start;
  float prev_cost = 0.0F;
  Vec3f delta;
  bool converged = false;
  for (int it = 0; it < cfg_.max_iterations; ++it) {
    Mat3f h;
    float g[3] = {0.0F, 0.0F, 0.0F};
    float cost = 0.0F;
    int n = 0;
    for (std::uint32_t i = 0; i < pts_l.count; ++i) {
      const Vec2f& p = pts_l.pts[i];
      const Vec2f q = T * p;
      float d = 0.0F;
      float gx = 0.0F;
      float gy = 0.0F;
      if (!df.sample(q, d, gx, gy) || d >= d_skip) {
        continue;
      }
      // Huber IRLS weight: points the submap does not explain (occlusion changes, new
      // objects) pull with a bounded force. rho(d) = d^2 below k, 2 k d - k^2 above.
      const float k = cfg_.huber_m;
      const float w = d <= k ? 1.0F : k / d;
      const Vec3f j = residual_jacobian(T, p, gx, gy);
      for (int a = 0; a < 3; ++a) {
        for (int b = a; b < 3; ++b) {
          h(a, b) += w * j[a] * j[b];
        }
        g[a] += w * j[a] * d;
      }
      cost += d <= k ? d * d : 2.0F * k * d - k * k;
      ++n;
    }
    r.iterations = it + 1;
    if (n < cfg_.min_points) {
      // Keep the last accepted pose; only the start pose has none.
      r.status = MatchStatus::kTooFewPoints;
      r.T_s_l = it == 0 ? start : prev_T;
      return r;
    }
    cost /= static_cast<float>(n);
    if (it > 0 && cost > prev_cost) {
      // Uphill: the unsigned distance is V-shaped at the walls, so a full step can jump
      // across the minimum. Halve the step from the last accepted pose and try again.
      for (int a = 0; a < 3; ++a) {
        delta[a] *= 0.5F;
      }
      T = prev_T * Pose2::exp(delta);
      if (step_is_small(delta)) {
        T = prev_T;
        converged = true;
        break;
      }
      continue;
    }
    h(1, 0) = h(0, 1);
    h(2, 0) = h(0, 2);
    h(2, 1) = h(1, 2);
    r.hessian = h;
    r.cost_m2 = cost;
    r.n_points = n;
    r.degeneracy = analyse_degeneracy(h, cfg_.degeneracy_eig_threshold);

    // Gauss-Newton step H delta = -g restricted to the non-degenerate eigen-directions
    // (projected update, docs/derivations/degeneracy.md section 3).
    const SymEigen3& e = r.degeneracy.eig;
    delta = Vec3f{};
    for (int k = 0; k < 3; ++k) {
      if (r.degeneracy.degenerate[k] || !(e.values[k] > 0.0F)) {
        continue;
      }
      const float vg = e.vectors(0, k) * g[0] + e.vectors(1, k) * g[1] + e.vectors(2, k) * g[2];
      const float w = -vg / e.values[k];
      for (int a = 0; a < 3; ++a) {
        delta[a] += w * e.vectors(a, k);
      }
    }
    prev_T = T;
    prev_cost = cost;
    T = T * Pose2::exp(delta);
    if (step_is_small(delta)) {
      converged = true;
      break;
    }
  }
  r.T_s_l = T;
  r.status = converged ? MatchStatus::kOk : MatchStatus::kNotConverged;
  return r;
}

}  // namespace mss2d
