# pipeline: single-thread estimator data flow

Implemented in `core/pipeline/src/` (`estimator_core.{hpp,cpp}` and the internal helpers
`imu_buffer`, `imu_trajectory_builder`, `frames`, `keyframe_store`, `keyframe_policy`,
`gnss_fusion`); public contract `core/pipeline/include/mss/pipeline/estimator.hpp`.
Tested by `tests/unit/test_pipeline_internals.cpp`, `tests/unit/test_pipeline_gnss_fusion.cpp`,
`tests/unit/test_estimator.cpp` and `tests/component/test_pipeline_component.cpp`.

Conventions (`lie_conventions.md`): `T_a_b` maps b to a; right perturbation
`T <- T * Exp(xi)`, `xi = [rho, phi]`; time is integer nanoseconds, converted to seconds only
for `dt`. Keyframe time = scan `t_start`.

## 1. IMU cut at keyframe / scan time

`imu::Preintegrator` integrates sample k with its own reading over `[t_{k-1}, t_k]` (Euler,
as GTSAM). To make a delta end exactly at a scan start `t_start`, the pipeline feeds every
buffered sample with `t < t_start`, then the interpolated sample

    s(t_start) = s_a + (t_start - t_a) / (t_b - t_a) * (s_b - s_a)     (acc and gyro)

with `t_a <= t_start <= t_b` the bracketing raw samples (`s(t_start) = s_a` when
`t_a == t_start`). The next interval starts by re-feeding `s(t_start)` as its first sample
after `reset()`. Hence `delta.dt_s * 1e9 == t_j - t_i` up to rounding, well inside the
backend tolerance `1.5e9 / rate_hz` ns, and `predict(x(t_i), g_w)` returns a state exactly at
`t_start`. The interpolation weight is computed from integer ns differences, then converted
to double.

## 2. Deskew trajectory

`x0` = predicted state at `t_start` (section 1). A scratch preintegrator is reset at
`x0.bias` and fed `s(t_start)` then each buffered sample with `t_start < t`, up to and
including the first sample with `t >= t_end` (`t_end = scan_end_time(scan)`). For each fed
sample k:

    T_b0_b(t_k) = x0.T_w_b^-1 * predict(x0, g_w).T_w_b

so `T_b0_b(t_start)` is exactly the identity (it is written as such, not computed).
When the number of entries N exceeds `ImuTrajectory::kMaxImuPerScan` (= M), the kept entry
indices are `round(j * (N - 1) / (M - 1))`, `j = 0..M-1` (integer arithmetic; both ends kept,
strictly increasing because `N > M`). Every sample is still integrated; only kept entries are
stored. Missing IMU samples (E-03) simply give a longer `dt` for the next sample.

## 3. LiDAR result to body

The matcher returns `T_w_l` and `cov_l` in the right-perturbation tangent of `T_w_l`. The
body pose is `T_w_b = T_w_l * T_b_l^-1`. Perturbing `T_w_l`:

    T_w_l Exp(xi_l) T_l_b = T_w_b (T_b_l Exp(xi_l) T_b_l^-1) = T_w_b Exp(Ad(T_b_l) xi_l)

so `J_xib_xil = Ad(T_b_l)` and

    cov_b = Ad(T_b_l) cov_l Ad(T_b_l)^T          (transform_covariance(cov_l, T_b_l))

Degeneracy report: eigenvectors `V_b = Ad(T_b_l) V_l` (column by column); eigenvalues and
flags are copied unchanged and the columns are not re-normalised (`Ad` is not orthogonal
when `p_b_l != 0`; the report is diagnostic only, `graph.hpp`).
Test: `Frames.LidarCovInBodyMatchesNumericalJacobian` compares `Ad(T_b_l)` with central
differences of `boxminus(T_w_l Exp(+-h e_k) T_l_b, T_w_b)`, `h = 1e-6`, relative error
< 1e-6.

## 4. LiDAR factor measurement

Between keyframe i (reference) and the new keyframe j:

    T_bi_bj = (T_w_bi^ref)^-1 * T_w_bj^match

`T_w_bi^ref` is the pose keyframe i had when its cloud was inserted into the map and used as
the keyframe reference (optimised pose when `optimize()` succeeded, else the matched pose).
Covariance = `cov_b` of scan j: the uncertainty of scan j relative to the map, as in
LIO-SAM. ADR-004 notes that this is optimistic (the map itself is not error-free).

## 5. GNSS

Each fix first passes `gnss::gate_fix(m, max_cov_m2)`. Once the aligner has a transform
`T_e_w`, the innovation is tested before the aligner sees the fix:

    r = p_e_g - T_e_w (T_w_b p_b_g),   J_r_xi = [-R_e_w R_w_b, R_e_w R_w_b skew(p_b_g)]
    S = m.cov + J_r_xi P J_r_xi^T,     reject when r^T S^-1 r > gnss.chi2_gate

with `T_w_b` the front-end pose predicted to the fix time and `P` the latest pose covariance
(`P := 0` when any entry is non-finite, e.g. the NaN marginal of the no-GTSAM stub: neither
"reject everything" nor "accept everything"). Only accepted fixes update the aligner (a
rejected jump never moves `T_e_w`).

Accepted fixes wait for the next keyframe k (`t_k >= m.t`) and are shifted to its time with
the front-end poses:

    p_e_g' = p_e_g + R_e_w (T_w_bk p_b_g - T_w_b(t_fix) p_b_g),     cov unchanged

so the factor residual at the keyframe pose equals the residual at the fix time. The factor
uses the latest aligner estimate of `T_e_w` when it is added.

## 6. Metrics

- New keyframe: `|p_ref - p| >= keyframe_dist_m` or `|so3_log(R_ref^T R)| >= keyframe_rot_rad`
  (`ref` = reference pose of the last keyframe, section 4).
- Map rebuild (ADR-016): the oldest keyframe of the map window moved, after optimisation,
  `|dp| > rebuild_trans_m` or `|so3_log(R_ins^T R_opt)| > rebuild_rot_rad` (strict).
- Wheel delta time check (`graph.hpp` add_odom_factor), in integer ns with `P` the last
  observed wheel interval: `2 |t0 - t_i| <= 3 P` and `2 |t1 - t_j| <= 3 P`.

## 7. Synthetic box world (component tests, shared with the integration tests)

Static for `static_s`, then a speed ramp over `ramp_s` on a circle of constant curvature
`kappa = W / V` (`W` yaw rate, `V` cruise speed); `tau = t - T0 - static_s`:

    v = 0                                   (tau < 0)
        V (1 - cos(pi tau / ramp_s)) / 2    (0 <= tau < ramp_s)
        V                                   (tau >= ramp_s)
    s = 0;  V/2 (tau - (ramp_s/pi) sin(pi tau / ramp_s));  V ramp_s / 2 + V (tau - ramp_s)
    a_t = V pi / (2 ramp_s) sin(pi tau / ramp_s) in the ramp, else 0
    psi = kappa s,  omega = kappa v
    p_w_b = (1/kappa) [sin psi, 1 - cos psi, 0],   R_w_b = Rz(psi)
    a_w = a_t [cos psi, sin psi, 0] + kappa v^2 [-sin psi, cos psi, 0]
    acc_b = R_w_b^T (a_w - g_w),   gyro_b = [0, 0, omega]

`static_s = ramp_s = 0` reduces to the constant circle of `sim/mss_sim/synth.py`. The LiDAR
is ray-cast against the six planes of an axis-aligned box centred at the origin; column `c`
of a scan is fired at `t_start + c * scan_period / columns`, from the LiDAR pose at that time.
