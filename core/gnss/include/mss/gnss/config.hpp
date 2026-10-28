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
  int align_min_pairs = 10;              // pairs needed before the aligner returns T_e_w (>= 1)
  double align_min_spread_m = 1.0;       // m, min RMS horizontal spread of the pairs (> 0)
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
  int align_yaw_chunk_fixes = 10;                        // fixes per chunk (>= 2)
  double align_yaw_max_sigma_rad = 3.0 * kRadPerDegree;  // rad, yaw 1-sigma needed to align (> 0)
  double align_yaw_min_track_m = 5.0;     // m, along-heading GNSS track needed to align (> 0)
  double align_yaw_min_coherence = 0.5;   // along-heading track / GNSS path length to align (0, 1]
  double align_yaw_reverse_nsigma = 3.0;  // sigmas: a segment this far backwards reverses (> 0)
  // rad, turn gate: max heading change across a segment, or within a chunk (0, pi]
  double align_yaw_max_turn_rad = 45.0 * kRadPerDegree;
  // m, wheel travel below which a segment is stationary and skipped (> 0)
  double align_yaw_min_wheel_travel_m = 0.2;
  double align_yaw_min_var_m2 = 0.25;  // m^2, floor on the declared horizontal fix variance (> 0)
};

}  // namespace mss::gnss
