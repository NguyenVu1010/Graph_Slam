#pragma once

#include <limits>

namespace mss::gnss {

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
};

}  // namespace mss::gnss
