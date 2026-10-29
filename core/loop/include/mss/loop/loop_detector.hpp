#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/loop/config.hpp"
#include "mss/map/config.hpp"
#include "mss/scan_match/config.hpp"

namespace mss::scan_match {
class ScanMatcher;
}
namespace mss::map {
class VoxelMap;
}

namespace mss::loop {

/// One keyframe as the detector sees it: a view, nothing is copied. `T_w_b` is the backend's
/// optimised pose at the time of the call; `cloud_l` the keyframe-store cloud in l (ADR-016),
/// valid for the duration of the call.
struct KeyframeView {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::KeyId id = 0;
  common::Timestamp t{};
  common::Pose T_w_b;
  const common::ScanFrame* cloud_l = nullptr;
};

/// Why a detect() call produced no loop (or kAccepted).
enum class LoopOutcome : std::uint8_t {
  kAccepted,     ///< a verified loop, LoopResult::constraint is valid
  kNoCandidate,  ///< no keyframe within search_radius_m and older than min_time_gap_s
  kDescriptor,   ///< every candidate failed the Scan Context distance gate (sc_max_distance)
  kIcpFailed,    ///< ScanMatcher::align failed or did not converge
  kCorrection,   ///< converged beyond max_correction_m / max_correction_deg of the graph pose
  kInliers,      ///< inlier ratio below min_inlier_ratio
  kRms,          ///< inlier RMS above max_rms_m
  kDegenerate,   ///< smallest ICP Hessian eigenvalue below min_hessian_eig
};

/// Result of one detect() call: the outcome of the best verified candidate (the last one
/// tried when none passed) and its diagnostics.
struct LoopResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopOutcome outcome = LoopOutcome::kNoCandidate;
  /// Valid when outcome == kAccepted: i = candidate (older), j = query, T_bi_bj, cov in
  /// [rho, phi] of the right perturbation of T_bi_bj (frame b_j), m^2 / rad^2 blocks.
  common::LoopConstraint constraint;
  int n_candidates = 0;             ///< keyframes that passed the radius and time-gap test
  int n_verified = 0;               ///< candidates that reached ICP
  common::KeyId candidate = 0;      ///< the reported candidate (when n_candidates > 0)
  common::Timestamp candidate_t{};  ///< its keyframe time
  double sc_distance = 1.0;         ///< Scan Context distance at the best column shift, [0, 1]
  double sc_yaw_rad = 0.0;          ///< relative yaw query - candidate from the best shift, rad
  double inlier_ratio = 0.0;        ///< of the reported ICP result
  double rms_m = 0.0;               ///< m, RMS inlier distance of the reported ICP result
  double min_eig = 0.0;             ///< smallest eigenvalue of the reported ICP Hessian
};

/// Loop-closure detection and verification (ADR-026; docs/derivations/loop_closure.md).
///
/// detect() for a query keyframe q:
///   1. Candidates: keyframes with `t <= q.t - min_time_gap_s` and horizontal-and-vertical
///      distance `|p_w_c - p_w_q| <= search_radius_m` (both from the backend's optimised
///      poses), nearest first, at most `max_candidates`. Drift larger than the radius is not
///      found (ADR-026, limit stated).
///   2. Scan Context of q and of c (each in its own gravity-levelled body frame: the pose's
///      roll and pitch removed, yaw kept) and the column shift with the smallest
///      column-cosine distance; distance > sc_max_distance rejects c. The shift gives the
///      relative yaw query - candidate, quantised to one sector (360 / sc_sectors deg).
///   3. ICP (scan_match::ScanMatcher, point-to-plane, scan_match.* with icp_* overrides,
///      degeneracy-aware update off) of q's cloud against a submap of c and its
///      `submap_keyframes` store neighbours on each side, placed in w at their optimised
///      poses. Seeds: the graph pose of q and, when it differs by more than one sector, the
///      same pose with the yaw from step 2; the seed with the higher inlier ratio wins.
///      A converged seed whose pose differs from the graph pose of q by more than
///      max_correction_m or max_correction_deg (yaw) is discarded: a correction that large is
///      not drift the graph can have accumulated since c, it is a symmetric false fit.
///   4. Acceptance: converged, inlier ratio (nearest submap point within inlier_dist_m and a
///      plane through the knn_k neighbours) >= min_inlier_ratio, point-to-plane RMS of the
///      inliers <= max_rms_m, smallest eigenvalue of the ICP Hessian >= min_hessian_eig.
///   5. Measurement `T_bc_bq = T_w_bc^-1 * T_w_lq(ICP) * T_l_b`; covariance = the ICP
///      covariance (MatchResult::cov, in l) moved to b with the adjoint of T_b_l, then
///      floored: `C >= diag(cov_floor_m^2 x3, cov_floor_rad^2 x3)` in the PSD sense.
///
/// Ownership/threading: single-threaded; owned by pipeline's EstimatorCore and run at keyframe
/// time on the estimator thread (ADR-026: no loop thread yet). Holds the ScanMatcher, the
/// submap VoxelMap and the descriptor buffers, all reserved in the constructor.
class LoopDetector {
 public:
  /// When `cfg.enabled` is false nothing is validated or reserved and detect() returns
  /// kNoCandidate. Otherwise `sm` (scan_match.*) is copied with max_iterations =
  /// icp_max_iterations and max_corr_dist_m = icp_max_corr_dist_m; `map_cfg` (map.*) with
  /// max_voxels = submap_max_voxels builds the submap. `T_b_l`: LiDAR extrinsic.
  /// `max_points`: the largest query cloud (realtime.max_points_per_scan).
  /// Errors: throws std::invalid_argument when enabled and a LoopConfig value is out of the
  ///   range documented in config.hpp, or from the ScanMatcher / VoxelMap constructors.
  /// Hot path: no (allocates). Tests: L-01..L-03 construct it.
  LoopDetector(const LoopConfig& cfg,
               const scan_match::ScanMatchConfig& sm,
               const map::MapConfig& map_cfg,
               const common::Pose& T_b_l,
               std::size_t max_points);
  ~LoopDetector();
  LoopDetector(const LoopDetector&) = delete;
  LoopDetector& operator=(const LoopDetector&) = delete;
  LoopDetector(LoopDetector&&) noexcept;
  LoopDetector& operator=(LoopDetector&&) noexcept;

  /// Searches `keyframes` (store order: oldest first, adjacent entries are adjacent
  /// keyframes; the query may be among them, it is never its own candidate) for a loop with
  /// `query` (steps above).
  /// Frames: T_w_b in w; clouds in l. Units: SI.
  /// Errors: none thrown; a failed verification is an outcome. Entries with a null or empty
  ///   cloud, or a non-finite pose, are skipped.
  /// Allocation: none (buffers reserved in the constructor). Hot path: no (per keyframe,
  ///   at most max_candidates ICP runs of up to two seeds each).
  /// Tests: L-01 (yaw within one sector), L-02 (no false loop), L-03 (wrong candidate
  ///   rejected by the fitness gates), LoopComponent.TwoLapDrift.
  LoopResult detect(const KeyframeView& query, common::Span<const KeyframeView> keyframes);

  /// Scan Context of `cloud_l` at body pose `T_w_b` into `out` (sc_rings x sc_sectors,
  /// row-major by ring): exposed for the tests. Precondition: enabled.
  void descriptor(const common::ScanFrame& cloud_l,
                  const common::Pose& T_w_b,
                  std::vector<float>& out) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Column-shift Scan Context distance (Kim & Kim 2018): for every shift s in [0, sectors),
/// `d(s) = 1 - mean_j cos(q[:, j], c[:, (j + s) mod sectors])` over the columns where both are
/// non-empty (d = 1 when none). Returns the smallest d and writes its shift to `best_shift`:
/// the relative yaw query - candidate is `best_shift * 2 pi / sectors` (wrapped to (-pi, pi]).
/// Hot path: no; no allocation. Tests: L-01.
double scan_context_distance(common::Span<const float> q,
                             common::Span<const float> c,
                             int rings,
                             int sectors,
                             int& best_shift);

}  // namespace mss::loop
