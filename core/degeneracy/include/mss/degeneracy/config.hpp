#pragma once

namespace mss::degeneracy {

// YAML: degeneracy.*  Loaded and validated by mss::pipeline::load_config.
struct DegeneracyConfig {
  bool enabled = false;        // feature flag (D-03)
  double eig_threshold = 0.0;  // absolute eigenvalue of H below which a direction degenerates
  // >= 1: floor factor of unobserved_covariance() (relax_factor * c / eig_threshold) and the
  // inflation factor of relax_covariance().
  double relax_factor = 1.0;
};

}  // namespace mss::degeneracy
