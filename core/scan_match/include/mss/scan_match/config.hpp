#pragma once

namespace mss::scan_match {

// YAML: scan_match.*  Loaded and validated by mss::pipeline::load_config.
struct ScanMatchConfig {
  int max_iterations = 0;             // Gauss-Newton iterations per scan (> 0)
  double rot_tol_rad = 0.0;           // rad, convergence threshold on the rotation update
  double trans_tol_m = 0.0;           // m, convergence threshold on the translation update
  double max_corr_dist_m = 0.0;       // m, reject correspondences farther than this
  double plane_rms_max_m = 0.0;       // m, reject neighbourhoods with a worse plane fit
  double robust_huber_delta_m = 0.0;  // m, Huber threshold on the point-to-plane residual
  int min_inliers = 0;                // fewer inliers -> match fails
  double cov_scale = 0.0;             // multiplier on the inverse-Hessian covariance
  double min_residual_sigma_m = 0.0;  // m, floor on the residual sigma used for the covariance
};

}  // namespace mss::scan_match
