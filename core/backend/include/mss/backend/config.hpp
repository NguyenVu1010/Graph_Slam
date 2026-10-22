#pragma once

namespace mss::backend {

// YAML: backend.*  Loaded and validated by mss::pipeline::load_config.
struct BackendConfig {
  double keyframe_dist_m = 0.0;        // m, new keyframe after this translation (> 0)
  double keyframe_rot_rad = 0.0;       // rad, new keyframe after this rotation (YAML: _deg)
  double relinearize_threshold = 0.0;  // iSAM2 relinearisation threshold (tangent norm)
  int relinearize_skip = 0;            // iSAM2 relinearises every N updates
};

}  // namespace mss::backend
