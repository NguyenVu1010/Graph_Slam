#pragma once

namespace mss::backend {

// YAML: backend.*  Loaded and validated by mss::pipeline::load_config.
struct BackendConfig {
  double keyframe_dist_m = 0.0;        // m, new keyframe after this translation (> 0)
  double keyframe_rot_rad = 0.0;       // rad, new keyframe after this rotation (YAML: _deg)
  double relinearize_threshold = 0.0;  // iSAM2 relinearisation threshold (tangent norm)
  int relinearize_skip = 0;            // iSAM2 relinearises every N updates
  // Priors anchoring the first keyframe (Graph::add_keyframe), all > 0.
  double prior_pos_sigma_m = 0.0;            // m, position sigma
  double prior_rot_sigma_rad = 0.0;          // rad, rotation sigma (YAML: _deg)
  double prior_vel_sigma_mps = 0.0;          // m/s, velocity sigma
  double prior_bias_acc_sigma_mps2 = 0.0;    // m/s^2, accelerometer bias sigma
  double prior_bias_gyro_sigma_radps = 0.0;  // rad/s, gyroscope bias sigma
};

}  // namespace mss::backend
