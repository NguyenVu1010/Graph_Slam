#pragma once

#include <cstddef>
#include <vector>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/scan_match/config.hpp"

namespace mss::scan_match {

/// Point-to-plane residual of one LiDAR point against a local map plane.
///
/// Residual: `r = n_w . (T_w_l * p_l - q_w)` (signed distance, metres).
/// Variable: the pose `T_w_l`, right perturbation `T_w_l <- T_w_l * Exp(xi)`,
///   `xi = [rho, phi]` (translation first, rotation second; docs/conventions.md).
/// Jacobian: `J_r_xi = dr/dxi` at xi = 0, a 1x6 row stored as a Vec6:
///   `J_r_xi = [ (R_w_l^T n_w)^T ,  -(R_w_l^T n_w)^T * skew(p_l) ]`
///   (first 3 entries w.r.t. rho, last 3 w.r.t. phi).
/// Frames: `p_l` in l; `q_w` (a point on the plane) and `n_w` (plane normal) in w.
/// Units: metres; `J_r_xi` entries are unitless (rho) and metres per radian (phi).
/// Preconditions: `n_w` has unit norm (not re-normalised); all inputs finite.
/// Errors: none (pure function).
/// `J_r_xi` may be nullptr; then only the residual is computed.
/// Hot path: yes, per point per iteration. No allocation, logging or exceptions.
/// Tests: S-01 (analytic J_r_xi vs numerical differentiation, relative error < 1e-6).
double point_to_plane_residual(const common::Vec3& p_l,
                               const common::Vec3& q_w,
                               const common::Vec3& n_w,
                               const common::Pose& T_w_l,
                               common::Vec6* J_r_xi);

/// Scan-to-map registration: iterative Gauss-Newton on point-to-plane residuals with a
/// Huber kernel.
///
/// Ownership/threading: single-threaded; one instance owned by pipeline::Estimator,
/// used only on the front-end thread. Holds per-point scratch buffers sized once.
class ScanMatcher {
 public:
  /// Validates `cfg` and reserves scratch buffers for `max_points` scan points.
  /// `deg_cfg` controls degeneracy-aware updates inside align() (see align()).
  /// Units: `max_points` in points (normally realtime.max_points_per_scan).
  /// Errors: throws std::invalid_argument when `max_iterations <= 0`, `rot_tol_rad <= 0`,
  ///   `trans_tol_m <= 0`, `max_corr_dist_m <= 0`, `plane_rms_max_m <= 0`,
  ///   `robust_huber_delta_m <= 0`, `min_inliers < 6`, `cov_scale <= 0` or
  ///   `max_points == 0`. (6 = number of pose DoF, not a tunable.)
  /// Hot path: no (allocates).
  /// Tests: S-02..S-05 construct it.
  ScanMatcher(const ScanMatchConfig& cfg,
              const degeneracy::DegeneracyConfig& deg_cfg,
              std::size_t max_points);

  /// Estimates the pose of `scan_l` in the map, starting from `T_w_l_init`.
  ///
  /// Per iteration, for each scan point: transform with the current `T_w_l`, query
  /// `map.knn(p_w, 0, buf)` (k = MapConfig::knn_k); keep the correspondence only if at
  /// least 3 neighbours were found, the nearest is within `max_corr_dist_m`, and the
  /// least-squares plane through the neighbours has RMS point-to-plane distance
  /// <= `plane_rms_max_m`. Then `q_w` = neighbour centroid, `n_w` = unit plane normal,
  /// residual and Jacobian from point_to_plane_residual(), IRLS Huber weight
  /// `w = 1 if |r| <= delta else delta / |r|` (delta = `robust_huber_delta_m`), applied to
  /// both `J^T J` and `J^T r`. Solve `H xi = -g` (H = sum w J^T J, g = sum w J^T r).
  /// If `deg_cfg.enabled`, on EVERY iteration: `report = degeneracy::analyze(H, deg_cfg)`;
  /// the flagged eigenvalues of H are raised to `eig_threshold`
  /// (`H_reg = H + sum_{k flagged} (eig_threshold - lambda_k) v_k v_k^T`); solve
  /// `H_reg xi = -g`; then `xi = degeneracy::project_update(xi, report)`. For an
  /// invertible H this equals projecting `-H^-1 g`, and it stays defined when H is
  /// singular along a flagged direction (docs/derivations/scan_match.md). With nothing
  /// flagged `H_reg == H`. Then update
  /// `T_w_l <- T_w_l * Exp(xi)`.
  /// Stops when `|xi.head<3>()| < trans_tol_m` and `|xi.tail<3>()| < rot_tol_rad`
  /// (converged = true) or after `max_iterations` updates (converged = false).
  ///
  /// Output MatchResult (all in the [rho, phi] tangent of the right perturbation of
  /// `T_w_l`, i.e. expressed in the LiDAR frame l):
  ///   - `T_w_l`: final estimate; `iterations`: updates applied; `inliers`: correspondences
  ///     kept in the last iteration;
  ///   - `H`: the Gauss-Newton Hessian `sum w J_r_xi^T J_r_xi` of the final iteration
  ///     (linearised at the estimate before the last update), NOT projected. The
  ///     pipeline still calls degeneracy::analyze + relax_covariance on it for the
  ///     factor covariance;
  ///   - `cov = cov_scale * sigma_r^2 * A^-1` (m^2, rad^2 blocks), symmetrised, where
  ///     `A = H` (degeneracy disabled) or `H_reg` (enabled) of the final iteration,
  ///     `sigma_r^2 = sum(w r^2) / sum(w)` (m^2) is the Huber-weighted mean squared
  ///     point-to-plane residual of that iteration's inliers, and `cov_scale` is the
  ///     dimensionless multiplier from config.
  /// Body-frame conversion is the pipeline's job, not this class's: the pipeline uses
  /// `transform_covariance(cov, T_b_l)` (lie.hpp) to express cov in b, and rotates the
  /// degeneracy directions with `se3_adj(T_b_l)` (see degeneracy.hpp).
  /// Frames: `scan_l` in l at `scan_l.t_start` (preprocessed); `map` in w;
  ///   `T_w_l_init` and result `T_w_l` = pose of l in w at `scan_l.t_start`.
  /// Units: metres, radians.
  /// Preconditions: `scan_l.size() <= max_points` given to the constructor.
  /// Errors (Result::error, no partial result):
  ///   - "scan larger than reserved max_points";
  ///   - "non-finite initial pose";
  ///   - "too few inliers" when inliers < `min_inliers` in any iteration (includes an
  ///     empty scan or empty map);
  ///   - "singular Hessian" when `Eigen::LLT<Mat6>(A).info() != Eigen::Success` in any
  ///     iteration (A = H, or H_reg when degeneracy is enabled: then only an unflagged
  ///     direction that is not positive definite fails), or when the update or the final
  ///     pose / cov is not finite.
  ///   Not converging within max_iterations is NOT an error: ok with converged = false;
  ///   the pipeline decides whether to use it.
  /// Keeps no reference to `scan_l` or `map` after return.
  /// Hot path: yes, per scan. No heap allocation, no logging, no exceptions; the scan is
  ///   read through spans, never copied.
  /// Tests: S-02 (known transform, <= 10 iterations), S-03 (convergence basin),
  ///   S-04 (Gaussian noise), S-05 (20 % outliers, robust kernel), D-03 (projected
  ///   update along the degenerate direction is 0).
  common::Result<common::MatchResult> align(const common::ScanFrame& scan_l,
                                            const map::VoxelMap& map,
                                            const common::Pose& T_w_l_init);

 private:
  // One Gauss-Newton accumulation at `T_w_l` over `p_l_[0, n)`: fills `hessian` = H and
  // `gradient` = g (both zeroed first),
  // `sum_w_r2_m2` = sum w r^2 (m^2) and `sum_w` = sum w; returns the inlier count.
  // Hot path: no allocation, logging or exceptions.
  int build_system(const map::VoxelMap& map,
                   std::size_t n,
                   const common::Pose& T_w_l,
                   common::Mat6& hessian,
                   common::Vec6& gradient,
                   double& sum_w_r2_m2,
                   double& sum_w);

  ScanMatchConfig cfg_;
  degeneracy::DegeneracyConfig deg_cfg_;
  std::size_t max_points_ = 0;     // points, capacity of p_l_
  std::vector<common::Vec3> p_l_;  // m, frame l: finite scan points in double, sized once
  map::NeighborBuffer nbr_;        // k-NN scratch, reused per point
};

}  // namespace mss::scan_match
