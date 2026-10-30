#pragma once

#include <limits>

namespace mss::gnss {

/// Radians per degree (pi / 180), for struct defaults given in degrees.
inline constexpr double kRadPerDegree = 3.14159265358979323846 / 180.0;

// YAML: gnss.*  Loaded and validated by mss::pipeline::load_config.
struct GnssConfig {
  bool enabled = false;     // feature flag
  double sigma_h_m = 0.0;   // m, horizontal fix sigma
  double sigma_v_m = 0.0;   // m, vertical fix sigma
  double max_cov_m2 = 0.0;  // m^2, gate_fix rejects a larger per-axis variance (> 0)
  // GnssAligner thresholds and the G-02 chi-square level. The defaults equal the former
  // header constants so value-initialised configs keep their behaviour; load_config
  // always overwrites them from YAML.
  int align_min_pairs = 10;         // pairs needed before the aligner returns T_e_w (>= 1)
  double align_min_spread_m = 1.0;  // m, min RMS horizontal spread of the pairs (> 0)
  // Backlog G1: GnssAligner also waits until its predicted yaw 1-sigma (declared fix variance,
  // floored at align_yaw_min_var_m2) is at most align_yaw_max_sigma_rad. On by default since
  // G13 (ADR-025); load_config overwrites it from YAML.
  bool align_check_yaw_sigma = true;
  // Backlog G15: GnssAligner's yaw sigma also counts the front-end antenna position covariance
  // (the w side, from the pose covariance the pipeline passes), so a dead-reckoned front end along
  // a LiDAR-degenerate axis no longer looks exact (gnss.md section 4.1). The struct default
  // keeps the previous behaviour; load_config overwrites it from YAML.
  bool align_yaw_sigma_frontend_cov = false;
  double chi2_gate = 7.814727903251178;  // NIS threshold, 3 DoF (chi-square q = 0.95) (> 0)
  // Consecutive chi-square rejections (fixes) after which the pipeline restarts the aligner
  // so a front end that drifted during an outage can be re-aligned (docs/derivations/
  // pipeline.md section 5). The struct default never restarts (previous behaviour);
  // load_config overwrites it from YAML (>= 1).
  int max_consecutive_rejects = std::numeric_limits<int>::max();
  // ADR-022: fixes received before the first alignment (the static ones at keyframe 0
  // included) become factors once aligned. The struct default keeps the previous behaviour
  // (dropped); load_config overwrites both from YAML.
  bool attach_prealign_fixes = false;
  int prealign_max_fixes = 1;  // fixes, capacity of the pre-alignment buffer (>= 1)
  // ADR-023: the yaw of T_e_w from the GNSS track direction against the body heading
  // (GnssTrackYawAligner) instead of GnssAligner's fit of front-end positions. The struct
  // default keeps the previous behaviour; load_config overwrites all of them from YAML.
  bool align_yaw_from_track = false;
  int align_yaw_chunk_fixes = 10;  // fixes per chunk (>= 2)
  // rad, yaw 1-sigma needed to align (> 0); also GnssAligner's with align_check_yaw_sigma
  double align_yaw_max_sigma_rad = 3.0 * kRadPerDegree;
  double align_yaw_min_track_m = 5.0;     // m, along-heading GNSS track needed to align (> 0)
  double align_yaw_min_coherence = 0.5;   // along-heading track / GNSS path length to align (0, 1]
  double align_yaw_reverse_nsigma = 3.0;  // sigmas: a segment this far backwards reverses (> 0)
  // rad, turn gate: max heading change across a segment, or within a chunk (0, pi]
  double align_yaw_max_turn_rad = 45.0 * kRadPerDegree;
  // m, wheel travel below which a segment is stationary and skipped (> 0)
  double align_yaw_min_wheel_travel_m = 0.2;
  // m^2, floor on the declared horizontal fix variance (> 0), in both aligners' yaw sigma
  double align_yaw_min_var_m2 = 0.25;
  // ADR-023 amendment (design 2b, gnss.md section 5.1). With wheel data, a segment the turn
  // gate rejects takes its direction from the wheel travel integrated along the body heading
  // (antenna path shape in w) instead of being skipped. The struct default keeps the previous
  // behaviour (turns skipped); load_config overwrites every field from YAML.
  bool align_yaw_wheel_turns = false;
  // 1-sigma wheel travel error as a fraction of the path, along and across the motion (slip,
  // skid), fully correlated within a chunk, in (0, 1).
  double align_yaw_wheel_slip_frac = 0.05;
  // m, along-heading track needed to align when every segment used has its direction from
  // wheel data, in place of align_yaw_min_track_m (>= 0). Its own value, not tied to
  // align_yaw_min_track_m: 3 m from the absolute-error envelope at 6 deg (gnss.md section 5.1).
  double align_yaw_min_track_wheel_m = 3.0;
  // m, wheel travel over which the segments since the last wheel/GNSS mismatch must agree with the
  // wheels before they replace all segments (wheels that turned on a still body; gnss.md section
  // 5.1) (> 0). 10 m: stuck wheels with 0.2 m per fix of GNSS drift align on 4 of 100 runs (6 m:
  // 16, 3 m: 83).
  double align_yaw_wheel_restart_m = 10.0;
  // Root-cause study of the residual limits (gnss_fusion_study.md section 16.13). The struct
  // defaults keep the previous behaviour; load_config overwrites every field from YAML.
  // fixes: a run of at most this many fixes without an odometer reading is bridged, the travel
  // over it coming from the next reading (the odometer is absolute), instead of leaving the chunk
  // half without wheel data (0: never bridged) (>= 0, < align_yaw_chunk_fixes / 2).
  int align_yaw_wheel_max_gap_fixes = 0;
  // chunks: a stationary segment is extended over up to this many more chunks until it has
  // align_yaw_min_wheel_travel_m of travel, so that a slow robot's travel adds up instead of
  // being skipped segment by segment (0: never extended) (>= 0).
  int align_yaw_merge_max_chunks = 0;
  // When the LiDAR observes the travel axis between two chunk centres, the front-end
  // displacement of o decides that a segment is stationary (wheels turning on a still body, a
  // parked robot), and without wheel data it gives the direction of travel. A scan observes the
  // axis when the scan matcher's marginal information along it reaches degeneracy.eig_threshold
  // (degeneracy::marginal_information, backlog G3).
  bool align_yaw_frontend_travel = false;
};

}  // namespace mss::gnss
