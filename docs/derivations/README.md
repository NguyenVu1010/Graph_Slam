# Derivations

One Markdown file per residual: state the residual, the variables, the perturbation
convention (right, `[rho, phi]`), then the Jacobians `J_<residual>_<variable>`.
Each file is referenced from the module that implements it and from its Jacobian test.

- `imu_preintegration.md`: IMU preintegration (core/imu).
- `scan_match.md`: point-to-plane residual, plane fit, degeneracy-aware solve, covariance (core/scan_match).
