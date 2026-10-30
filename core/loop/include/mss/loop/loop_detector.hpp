#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "mss/common/result.hpp"
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
  /// A second, separated fit almost as good as the best (ambiguity check, repeated structure).
  kAmbiguous,
  /// A RelativeCovarianceSource was given but returned an error for the candidate: the
  /// correction gate cannot run, the candidate is rejected (fail closed).
  kNoMarginal,
};

/// The graph's uncertainty of the relative pose of two keyframes, for the correction gate
/// (ADR-026 amendment). Implemented by the pipeline over backend::Graph::relative_pose_cov, so
/// the loop module needs no backend (and no GTSAM).
class RelativeCovarianceSource {
 public:
  virtual ~RelativeCovarianceSource() = default;
  /// Covariance of T_bi_bj = T_w_bi^-1 * T_w_bj at the current estimate: 6x6 [rho, phi], right
  /// perturbation of T_bi_bj (frame b_j), m^2 / rad^2 blocks. Errors: Result::error when the
  /// graph cannot provide it (unknown or not optimised keyframe, marginal failure).
  virtual common::Result<common::PoseCovariance> relative_cov(common::KeyId i,
                                                              common::KeyId j) const = 0;
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
  /// e^T S^-1 e of the reported fit against the graph (correction gate); NaN when no
  /// RelativeCovarianceSource was given or no fit reached the gate.
  double correction_chi2 = std::numeric_limits<double>::quiet_NaN();
  /// Inlier ratio of the best competing fit / the reported one (ambiguity check); 0 when no
  /// separated competitor passed the gates.
  double ambiguity_ratio = 0.0;
  int n_icp_runs = 0;  ///< ICP runs of this call (seeds plus ambiguity seeds)
  /// Ambiguity seeds skipped by the fitness pre-screen (ambiguity_prescreen_ratio).
  int n_prescreen_skips = 0;
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
///      With a RelativeCovarianceSource, a converged seed must also pass the correction gate
///      from the graph's uncertainty: e^T S^-1 e <= correction_chi2_gate, e the 6-DoF
///      difference of the fitted and the graph relative pose (i, j), S the graph's relative
///      marginal plus the floored loop covariance (step 5).
///   4. Acceptance: converged, inlier ratio (nearest submap point within inlier_dist_m and a
///      plane through the knn_k neighbours) >= min_inlier_ratio, point-to-plane RMS of the
///      inliers <= max_rms_m, smallest eigenvalue of the ICP Hessian >= min_hessian_eig.
///      Then the ambiguity check (ambiguity_seeds_per_side > 0): ICP again from the best fit
///      shifted by +-k ambiguity_seed_step_m along the least-constrained horizontal direction
///      of its Hessian; any converged fit (these or the step-3 seeds) within the fixed
///      correction gates and max_rms_m, beyond ambiguity_min_separation_m / _rad of the best,
///      that reaches ambiguity_max_ratio x its inlier ratio rejects the loop (kAmbiguous).
///      Each shifted seed is pre-screened by one fitness pass at the shifted pose (no ICP; ICP
///      only when it reaches ambiguity_prescreen_ratio x the best ratio), and its ICP runs at
///      most ambiguity_icp_max_iterations (an unconverged fit is still judged by fitness).
///      A competitor counts unless (graph_covariance_gate only) the graph rules it out: it
///      fails correction_chi2_gate AND its chi-square exceeds the best fit's by at least
///      ambiguity_chi2_margin.
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
  /// The placeholder contract's constructor, kept (additive change, ADR-026): validate-only.
  /// Errors: throws std::invalid_argument when `cfg.enabled` and a LoopConfig value is out of
  ///   range (including `search_radius_m <= 0` or `min_time_gap_s < 0`, the placeholder's
  ///   checks). It reserves nothing and has no matcher, so detect() always returns
  ///   kNoCandidate; searching needs the full constructor below.
  /// Hot path: no. Tests: LoopDetector.PlaceholderConstructorValidatesOnly.
  explicit LoopDetector(const LoopConfig& cfg);

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
  /// `query` (steps above). Same as detect(query, keyframes, nullptr): no correction gate
  /// from the graph's uncertainty, only the fixed max_correction_m / _deg gates.
  /// Frames: T_w_b in w; clouds in l. Units: SI.
  /// Errors: none thrown; a failed verification is an outcome. Entries with a null or empty
  ///   cloud, or a non-finite pose, are skipped.
  /// Allocation: none (buffers reserved in the constructor). Hot path: no (per keyframe,
  ///   at most max_candidates ICP runs of up to two seeds each, plus
  ///   2 * ambiguity_seeds_per_side for a candidate that passes every gate).
  /// Tests: L-01 (yaw within one sector), L-02 (no false loop), L-03 (wrong candidate
  ///   rejected by the fitness gates), LoopComponent.TwoLapDrift.
  LoopResult detect(const KeyframeView& query, common::Span<const KeyframeView> keyframes);

  /// detect() with the correction gate from the graph's uncertainty (step 3) when `rel` is not
  /// null and loop_closure.graph_covariance_gate is true (ADR-028; otherwise `rel` is not
  /// used): `rel->relative_cov(candidate, query)` is asked once per candidate that has a
  /// converged fit. An error from it rejects that candidate (kNoMarginal).
  /// Tests: LoopFixture.CorrectionGate*, LoopFixture.Ambiguity*, LoopComponent.*,
  ///   LoopAliasing.*.
  LoopResult detect(const KeyframeView& query,
                    common::Span<const KeyframeView> keyframes,
                    const RelativeCovarianceSource* rel);

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
