# Math and frame conventions

Every module follows these. If a change needs something different, write an ADR first.

## Frames

| Symbol | Frame |
|--------|-------|
| `w`    | world / map, gravity-aligned, z up |
| `b`    | body = IMU |
| `l`    | LiDAR |
| `e`    | ENU, local tangent plane at the GNSS origin |
| `g`    | GNSS antenna phase centre |

## Transforms

- `T_a_b` maps points from `b` to `a`: `p_a = T_a_b * p_b`.
- Composition cancels inner frames: `T_w_l = T_w_b * T_b_l`.
- `R_a_b` rotation, `p_a_b` origin of `b` expressed in `a`, `v_w_b` velocity of `b` in `w`.
- Extrinsics in config: `T_b_l` (LiDAR in body), `p_b_g` (antenna lever arm).

## Perturbation

Right perturbation on SE(3): `T <- T * Exp(delta)`, tangent order `delta = [rho, phi]`
(translation, rotation). This matches GTSAM `Pose3`, so backend factors need no re-ordering.

Jacobians are named `J_<residual>_<variable>`: `J_r_xi`, `J_dp_bg`. Before writing one,
state the residual, the variables and the perturbation in a comment, then write the
numerical-difference test (relative error < 1e-6).

## Units and types

- SI everywhere. Radians in code; degrees only in YAML/logs with a `_deg` suffix.
- Quaternions only at I/O boundaries, Hamilton order `(w, x, y, z)`. Comment the order at
  every construction site. TUM files are `(x, y, z, w)`; `eval/mss_eval/tum.py` owns that.
- `Timestamp` is int64 nanoseconds on the sensor clock. Convert to double only for `dt`
  via `seconds_between`. Never compare floating-point timestamps.
- Gravity `[0, 0, -9.80665]` in `w` unless IMU initialisation estimates it.

## Core types (`mss/common/types.hpp`)

`Pose{R, p}`, `PoseCovariance` (6x6, `[rho, phi]` order), `ImuSample{t, acc_b, gyro_b}`.
Modules outside `backend` exchange only these; GTSAM types never cross a module boundary.
