#pragma once

#include <cstdint>

namespace mss::backend {

// Form of the LiDAR and loop relative-pose factors (docs/derivations/backend_factors.md
// section 5; ADR-021: default kDecoupled). YAML: backend.relative_factor.
enum class RelativeFactor : std::uint8_t {
  kBetween,   // "between": gtsam::BetweenFactor<Pose3>, approximate Jacobian (before ADR-021)
  kSe3Exact,  // "se3_exact": the same residual Log(Z^-1 T_i^-1 T_j), exact Jacobians
  kDecoupled  // "decoupled": translation and rotation residuals separate, exact Jacobians (default)
};

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
  // Keyframe 0 also gets the static-window gravity-direction factor (Graph::
  // add_static_gravity_factor, imu_only_observability.md sections 9, 10) when the pipeline has
  // the initializer's window statistics; false: priors only (the behaviour before PR 4d).
  bool static_gravity_factor = false;
  // Validated (> 0) only when static_gravity_factor is true.
  double static_gravity_sigma_scale = 2.0;  // multiplier on the window std of the mean
  // Keyframes before the consistency check; 0 = check off (shipped, ADR-020: evaluated, its
  // statistic at rest overlaps its values at moving starts).
  int static_gravity_check_keyframes = 0;
  double static_gravity_chi2_gate = 5.991464547107979;  // chi-square 2 DoF, 0.95
  // Relative-pose factor form; default kDecoupled (ADR-021). kBetween and kSe3Exact remain
  // selectable for comparison.
  RelativeFactor relative_factor = RelativeFactor::kDecoupled;
  // ADR-019: T_e_w is one graph variable E shared by every GNSS factor (true) instead of a
  // fixed parameter per factor (false, the previous behaviour). E gets one prior, centred on
  // the first T_e_w given to add_gnss_factor, with these sigmas (all > 0 when true):
  bool gnss_frame_variable = false;
  double gnss_frame_tilt_sigma_rad = 0.0;  // rad, roll and pitch of E (tight: E is 4-DoF)
  double gnss_frame_yaw_sigma_rad = 0.0;   // rad, yaw of E (loose)
  double gnss_frame_pos_sigma_m = 0.0;     // m, translation of E (loose)
  // Limits TBB to one thread for the rest of the process (ADR-018); set by pipeline from
  // `deterministic`, not a YAML key (ADR-008). No-op when GTSAM is built without TBB.
  bool single_thread = false;
};

}  // namespace mss::backend
