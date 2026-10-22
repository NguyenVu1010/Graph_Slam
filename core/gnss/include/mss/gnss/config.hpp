#pragma once

namespace mss::gnss {

// YAML: gnss.*  Loaded and validated by mss::pipeline::load_config.
struct GnssConfig {
  bool enabled = false;    // feature flag
  double sigma_h_m = 0.0;  // m, horizontal fix sigma
  double sigma_v_m = 0.0;  // m, vertical fix sigma
};

}  // namespace mss::gnss
