#pragma once

namespace mss::loop {

// YAML: loop_closure.*  Loaded and validated by mss::pipeline::load_config.
struct LoopConfig {
  bool enabled = false;          // feature flag
  double search_radius_m = 0.0;  // m, candidate keyframes within this distance
  double min_time_gap_s = 0.0;   // s, candidates must be at least this much older
};

}  // namespace mss::loop
