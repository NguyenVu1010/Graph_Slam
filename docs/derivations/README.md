# Derivations

One Markdown file per residual: state the residual, the variables, the perturbation
convention (right, `[rho, phi]`), then the Jacobians `J_<residual>_<variable>`.
Each file is referenced from the module that implements it and from its Jacobian test.

- `imu_preintegration.md`: IMU preintegration (core/imu).
- `scan_match.md`: point-to-plane residual, plane fit, degeneracy-aware solve, covariance (core/scan_match).
- `loop_closure.md`: loop-closure candidates, Scan Context yaw, ICP verification and fitness, the loop measurement and its floored covariance, the robust loop factor and iSAM2 after a loop (core/loop, ADR-026).
- `pipeline.md`: estimator data flow: IMU cut at scan time, deskew trajectory, l->b covariance, LiDAR factor measurement, GNSS gate and time shift, keyframe/rebuild metrics, synthetic box world (core/pipeline).
- `gnss_fusion_study.md`: why GNSS degraded the box world (ATE, aligner error, NEES, controls), the fixes evaluated, and the result of T_e_w as a graph variable (PR 4a, ADR-019).
- `imu_only_observability.md`: why the estimate along an axis the LiDAR does not observe drifted 10 to 40 sigma beyond dead reckoning (controlled experiments: dead reckoning, factor information, feedback, bias, relinearisation, keyframes, oracle), the mechanism, and the static-window gravity factor (PR 4d).
- `vertical_drift_study.md`: why the Gazebo E-04 map climbs and E-01's ATE is vertical (hypotheses one at a time on recordings over three seeds: gravity anchor, degeneracy, LiDAR render, extrinsic, z bias, horizontal bias; oracles for the bias, the floor and the tilt), the mechanism (horizontal accelerometer bias read as tilt), and the level start (ADR-027).
