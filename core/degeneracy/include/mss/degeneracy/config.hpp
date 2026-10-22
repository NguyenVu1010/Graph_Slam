#pragma once

namespace mss::degeneracy {

// YAML: degeneracy.*  Loaded and validated by mss::pipeline::load_config.
struct DegeneracyConfig {
  bool enabled = false;        // feature flag (D-03), off until evaluated
  double eig_threshold = 0.0;  // absolute eigenvalue of H below which a direction degenerates
  double relax_factor = 1.0;   // covariance inflation along degenerate directions (>= 1)
};

}  // namespace mss::degeneracy
