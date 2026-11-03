#pragma once

#include <cstddef>

namespace mss::loop {

// YAML: loop_closure.*  Loaded and validated by mss::pipeline::load_config (ADR-026). Every
// value below is validated only when `enabled` is true; the defaults here are placeholders,
// config/default.yaml holds the shipped values.
struct LoopConfig {
  bool enabled = false;          // feature flag (default false, CONTRIBUTING.md)
  double search_radius_m = 0.0;  // m, candidate keyframes within this distance (> 0)
  double min_time_gap_s = 0.0;   // s, candidates must be at least this much older (>= 0)
  // Candidates verified per keyframe, nearest first (>= 1). Bounds the work per keyframe.
  int max_candidates = 1;
  // Place search (ADR-030, backlog C1-1, C1-31): the candidates within search_radius_m are
  // ranked by Scan Context distance instead of by distance, and ICP also starts from the query
  // placed at the candidate's position with the Scan Context yaw. With drift of metres the
  // nearest keyframe on the drifted estimate is not the revisited place and the graph seed is
  // outside the ICP basin. false: nearest first, graph seeds only (the behaviour before).
  bool place_search = false;
  // Place search bounds (ADR-030 fix round, review B-2, B-3). Only the nearest
  // place_search_max_ranked candidates (>= max_candidates) are ranked by Scan Context, so the
  // per-keyframe cost stays bounded. Ranking and the place seed are used only when the best
  // candidate is distinct: its Scan Context distance is at most place_search_sc_ratio (in
  // (0, 1]) times the best one among candidates at least place_search_min_separation_m (> 0)
  // from it; otherwise (a look-alike elsewhere) the search falls back to nearest first and the
  // graph seeds only.
  int place_search_max_ranked = 1;
  double place_search_sc_ratio = 1.0;
  double place_search_min_separation_m = 0.0;  // m
  // A place-seed fit is a loop only when its correction is within icp_max_corr_dist_m plus this
  // ratio times the distance travelled from the candidate to the query (review N-4; > 0).
  double place_search_drift_ratio = 0.0;  // m per m travelled
  // A cached descriptor is recomputed when the keyframe's roll and pitch moved by more than this
  // since it was levelled (a graph correction can tilt keyframes; review N2-1; > 0).
  double place_search_cache_tilt_rad = 0.0;  // rad (YAML: place_search_cache_tilt_deg)
  // Keyframes skipped after an accepted loop before the next search (>= 0).
  int min_interval_keyframes = 0;
  // Submap: the candidate keyframe plus this many store neighbours on each side (>= 0).
  int submap_keyframes = 0;
  std::size_t submap_max_voxels = 1;  // voxels reserved for the submap (> 0)
  // Scan Context descriptor (Kim & Kim 2018): rings x sectors polar grid in a gravity-levelled
  // body frame, value = max point height + sc_height_offset_m; cells <= 0 are empty.
  int sc_rings = 1;             // radial bins (>= 1)
  int sc_sectors = 1;           // azimuth bins (>= 4); one sector = 360 / sc_sectors deg
  double sc_max_range_m = 0.0;  // m, points beyond this horizontal range are ignored (> 0)
  // m (> 0), below the body height above the ground so ground returns are empty.
  double sc_height_offset_m = 0.0;
  double sc_max_distance = 0.0;  // column-cosine distance gate in (0, 1]
  // ICP verification (scan_match::ScanMatcher with these overrides of scan_match.*).
  int icp_max_iterations = 0;        // Gauss-Newton iterations (> 0)
  double icp_max_corr_dist_m = 0.0;  // m, correspondence distance (> 0)
  // Acceptance after ICP: a query point is an inlier when its nearest submap point is within
  // inlier_dist_m and its map.knn_k neighbours fit a plane; residual = distance to that plane.
  double inlier_dist_m = 0.0;     // m (> 0)
  double min_inlier_ratio = 0.0;  // inliers / query points, in (0, 1]
  double max_rms_m = 0.0;         // m, point-to-plane RMS of the inliers (> 0)
  // Smallest eigenvalue of the ICP Hessian H (units of MatchResult::H) for every direction;
  // below it the loop is rejected as degenerate (> 0).
  double min_hessian_eig = 0.0;
  // Covariance floor of the loop measurement (C >= diag(floor^2) in the PSD sense).
  double cov_floor_m = 0.0;    // m (> 0)
  double cov_floor_rad = 0.0;  // rad (> 0; YAML: cov_floor_deg)
  // Largest correction of the query pose against its graph pose (drift since the candidate),
  // checked on every converged ICP seed; larger: a symmetric false fit, discarded.
  double max_correction_m = 0.0;    // m (> 0)
  double max_correction_rad = 0.0;  // rad (> 0; YAML: max_correction_deg)
  // Extra iSAM2 updates after the update that adds an accepted loop (>= 0).
  int isam_extra_updates = 0;
  // Correction gate from the graph's uncertainty (ADR-026 amendment, loop_closure.md section 9).
  // Optional, off by default (ADR-028: it drops every true loop once the graph is moderately
  // overconfident). When true and detect() is given a RelativeCovarianceSource: a converged
  // fit is kept only when
  // e^T S^-1 e <= correction_chi2_gate, with e = Log(T_graph_ij^-1 T_fit_ij) in [rho, phi]
  // (all six axes, roll and pitch included) and S = the graph's marginal covariance of the
  // relative pose (i, j) + the floored loop covariance. Chi-square threshold, 6 DoF (> 0).
  double correction_chi2_gate = 0.0;
  bool graph_covariance_gate = false;  // the gate above, and graph arbitration below
  // Ambiguity check (loop_closure.md section 10): ICP is re-run from the best fit shifted by
  // +-k * ambiguity_seed_step_m, k = 1..ambiguity_seeds_per_side, along the least-constrained
  // horizontal direction of the ICP Hessian. A second converged fit within the fixed
  // correction gates and the RMS gate, more than ambiguity_min_separation_m (or
  // ambiguity_min_separation_rad) from the best, with an inlier ratio >= ambiguity_max_ratio
  // x the best one makes the loop ambiguous: rejected. Only with graph_covariance_gate (and
  // a RelativeCovarianceSource) the graph may rule the competitor out: it fails
  // correction_chi2_gate AND its chi-square exceeds the best one's by at least
  // ambiguity_chi2_margin; seeds the graph already rules out are then skipped.
  int ambiguity_seeds_per_side = 0;           // >= 0; 0 turns the check off
  double ambiguity_seed_step_m = 0.0;         // m (> 0)
  double ambiguity_min_separation_m = 0.0;    // m (> 0)
  double ambiguity_min_separation_rad = 0.0;  // rad (> 0; YAML: ambiguity_min_separation_deg)
  double ambiguity_max_ratio = 0.0;           // second-best / best inlier ratio, in (0, 1]
  double ambiguity_chi2_margin = 0.0;         // chi-square difference (> 0)
  // Latency bounds of the ambiguity check (review H3, I-2): keyframes not searched after a
  // kAmbiguous result (the pipeline; >= 0), ICP iterations of a shifted seed (> 0), and the
  // fitness pre-screen: ICP from a shifted seed only when one fitness pass there reaches this x
  // the best fit's inlier ratio ([0, 1]; 0 turns the pre-screen off).
  int ambiguity_backoff_keyframes = 0;
  int ambiguity_icp_max_iterations = 1;
  double ambiguity_prescreen_ratio = 0.0;
  // Loop selection by pairwise consistency (PCM, ADR-028, loop_closure.md section 11): the
  // verified loops are kept in the graph only while they belong to a maximum set of mutually
  // consistent loops; a loop that leaves the set is removed from the graph.
  bool pcm_enabled = false;    // false: every verified loop goes into the graph (YAML: true)
  double pcm_chi2_gate = 0.0;  // pairwise cycle chi-square, 6 DoF (> 0)
  // The odometry covariance is multiplied by this squared in the pairwise test (>= 1): the
  // front end's covariance is not trusted at its absolute scale (ADR-028; review H3 I-1).
  double pcm_odometry_sigma_scale = 1.0;
  // Unknown constant yaw drift rate of the odometry, 1 sigma, per metre travelled (>= 0; 0: no
  // drift term). Its effect on a segment (yaw ~ distance, lateral ~ distance^2) is added to the
  // odometry covariance of the pairwise test (ADR-028 amendment C, loop_closure.md 11b).
  double pcm_yaw_drift_rad_per_m = 0.0;  // rad/m (YAML: pcm_yaw_drift_deg_per_m)
  int pcm_max_loops = 1;         // loops held, [1, 64]; the oldest leaves (frozen if selected)
  int pcm_max_clique_steps = 1;  // Bron-Kerbosch call bound (> 0)
  // Kernel release (ADR-030, backlog C1-13): with PCM on, a selected loop that has a partner, an
  // other selected loop whose keyframes i and j both lie within kernel_release_window_keyframes
  // of its own, enters the graph without the Cauchy kernel (Gaussian), so its whole correction
  // applies; a loop without such a partner keeps the kernel. false: every loop keeps the kernel
  // (the behaviour before ADR-030). Needs pcm_enabled.
  bool kernel_release = false;
  int kernel_release_window_keyframes = 1;  // keyframes (>= 1)
  // Partner evidence that one aliased spot cannot produce (ADR-030 fix round, review B-1): the
  // two loops' candidates and queries are each at least kernel_release_min_separation_m apart
  // (two places, not one look-alike seen twice), and each loop asks a correction no larger than
  // kernel_release_drift_ratio times the distance travelled from its candidate to its query.
  double kernel_release_min_separation_m = 0.0;  // m (> 0)
  double kernel_release_drift_ratio = 0.0;       // m of correction per m travelled (> 0)
};

}  // namespace mss::loop
