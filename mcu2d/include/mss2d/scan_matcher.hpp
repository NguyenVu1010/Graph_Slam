#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/degeneracy.hpp"
#include "mss2d/distance_field.hpp"
#include "mss2d/occupancy_grid.hpp"
#include "mss2d/points2d.hpp"
#include "mss2d/pose2.hpp"

namespace mss2d {

/// Residual r = D(T_s_l * p_l) (m) for one point; (gx, gy) is the gradient of the distance
/// field D at q = T_s_l * p_l. Variable: T_s_l with right perturbation
/// T_s_l <- T_s_l * Exp(delta), delta = [x, y, yaw]. To first order
/// T Exp(delta) p = R (p + delta_xy + delta_yaw perp(p)) + t, perp(p) = [-p.y, p.x], so
///   J_r_delta = [ g^T R,  g^T R perp(p) ]   (1x3; unitless, unitless, m/rad).
/// Templated so the test checks the formula in double (docs/scan_matching.md).
template <typename S>
Vec3T<S> residual_jacobian(const Pose2T<S>& T_s_l, const Vec2T<S>& p_l, S gx, S gy) {
  const S c = std::cos(T_s_l.yaw);
  const S s = std::sin(T_s_l.yaw);
  Vec3T<S> j;
  j[0] = gx * c + gy * s;   // g^T R e_x
  j[1] = -gx * s + gy * c;  // g^T R e_y
  j[2] = -j[0] * p_l.y + j[1] * p_l.x;
  return j;
}

enum class MatchStatus : std::uint8_t {
  kOk = 0,
  kNotConverged,  ///< max_iterations reached; the pose is the last iterate
  kTooFewPoints,  ///< fewer than min_points usable points; the pose is the start pose
};

struct MatchResult {
  Pose2 T_s_l{};                   ///< refined LiDAR pose in the submap frame
  Mat3f hessian;                   ///< J^T W J at the last accepted pose, tangent [x, y, yaw]
  int iterations = 0;              ///< Gauss-Newton iterations run
  int n_points = 0;                ///< points used in the last evaluation
  float cost_m2 = 0.0F;            ///< mean Huber cost at that pose (= mean squared residual), m^2
  float correlative_score = 0.0F;  ///< mean cell probability of the best coarse candidate
  MatchStatus status = MatchStatus::kTooFewPoints;
  DegeneracyReport degeneracy;  ///< of `hessian`; flagged directions get no update
};

/// Scan-to-submap matcher: correlative coarse search on the occupancy probability, then
/// Gauss-Newton on the distance field. No allocation; 8 KB of index scratch.
class ScanMatcher {
 public:
  /// `cfg` must have passed MatcherConfig::validate().
  explicit ScanMatcher(const MatcherConfig& cfg) : cfg_(cfg) {}

  /// Both stages from the guess. correlative_score is set; the rest comes from refine().
  MatchResult match(const OccupancyGrid& grid,
                    const DistanceField& df,
                    const PointCloud2d& pts_l,
                    const Pose2& T_s_l_guess);

  /// Coarse stage: best pose on the lattice guess + (i res, j res, k angle_step) inside the
  /// window, by mean cell probability. Ties go to the candidate closest to the guess.
  Pose2 correlative_search(const OccupancyGrid& grid,
                           const PointCloud2d& pts_l,
                           const Pose2& T_s_l_guess,
                           float& best_score);

  /// Fine stage: Gauss-Newton on the distance field from `start`.
  MatchResult refine(const DistanceField& df, const PointCloud2d& pts_l, const Pose2& start);

 private:
  bool step_is_small(const Vec3f& delta) const;

  MatcherConfig cfg_;
  std::array<std::int32_t, kMaxScanPoints> cx_{};
  std::array<std::int32_t, kMaxScanPoints> cy_{};
};

}  // namespace mss2d
