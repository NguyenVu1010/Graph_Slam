#pragma once

namespace mss::gnss {

// YAML: gnss.*  Loaded and validated by mss::pipeline::load_config.
struct GnssConfig {
  bool enabled = false;     // feature flag
  double sigma_h_m = 0.0;   // m, horizontal fix sigma
  double sigma_v_m = 0.0;   // m, vertical fix sigma
  double max_cov_m2 = 0.0;  // m^2, gate_fix rejects a larger per-axis variance (> 0)
};

}  // namespace mss::gnss
