# Derivations

One Markdown file per residual: state the residual, the variables, the perturbation
convention (right, `[rho, phi]`), then the Jacobians `J_<residual>_<variable>`.
Each file is referenced from the module that implements it and from its Jacobian test.

- `imu_preintegration.md`: IMU preintegration (core/imu).
- `scan_match.md`: point-to-plane residual, plane fit, degeneracy-aware solve, covariance (core/scan_match).
- `pipeline.md`: estimator data flow: IMU cut at scan time, deskew trajectory, l->b covariance, LiDAR factor measurement, GNSS gate and time shift, keyframe/rebuild metrics, synthetic box world (core/pipeline).
- `gnss_fusion_study.md`: why GNSS degraded the box world (ATE, aligner error, NEES, controls), the fixes evaluated, and the result of T_e_w as a graph variable (PR 4a, ADR-019).
- `imu_only_observability.md`: why the estimate along an axis the LiDAR does not observe drifted 10 to 40 sigma beyond dead reckoning (controlled experiments: dead reckoning, factor information, feedback, bias, relinearisation, keyframes, oracle), the mechanism, and the static-window gravity factor (PR 4d).
