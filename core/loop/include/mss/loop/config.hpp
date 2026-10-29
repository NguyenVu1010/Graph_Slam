#pragma once

#include <cstddef>

namespace mss::loop {

// YAML: loop_closure.*  Loaded and validated by mss::pipeline::load_config (ADR-026). Every
// value below is validated only when `enabled` is true; the defaults here are placeholders,
// config/default.yaml holds the shipped values.
struct LoopConfig {
  bool enabled = false;          // feature flag (default false, CLAUDE.md)
  double search_radius_m = 0.0;  // m, candidate keyframes within this distance (> 0)
  double min_time_gap_s = 0.0;   // s, candidates must be at least this much older (>= 0)
  // Candidates verified per keyframe, nearest first (>= 1). Bounds the work per keyframe.
  int max_candidates = 1;
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
};

}  // namespace mss::loop
