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
when `p_b_l != 0`; the report is diagnostic only, `graph.hpp`). After the transform only the
flagged (degenerate) directions are meaningful, as directions in the b tangent: the columns
are neither unit-length nor mutually orthogonal and are not eigenvectors of the Hessian
expressed in b, so they must not be used as an orthonormal basis (for example for an
information projection) without re-orthonormalising.
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

Pose at the fix time. GNSS is merged by `m.t`, so the IMU sample at or after `m.t` usually
sorts after the fix. The core therefore defers each fix in a fixed ring
(`kMaxDeferredGnss` = 32; full: oldest dropped, `Stats::n_gnss_ring_overflow`; fixes still
waiting at `flush()` because no IMU sample follows them: `Stats::n_gnss_unfused_eof`) until an IMU sample with
`t >= m.t` is buffered; then `T_w_b(m.t)` is predicted from the front-end state at the last
scan start `t_s` through the buffered samples in `(t_s, m.t)` and the sample interpolated
at `m.t` (section 1). A fix with `m.t < t_s` (pushed late, after a scan with a later start
was processed) can no longer be predicted from the front-end state and is rejected
(`Stats::n_gnss_late`, one log line per streak). Deferred fixes are processed before the next
scan changes the state.

Each fix then passes `gnss::gate_fix(m, max_cov_m2)`. Once the aligner has a transform
`T_e_w`, the innovation is tested before the aligner sees the fix:

    r = p_e_g - T_e_w (T_w_b p_b_g),   J_r_xi = [-R_e_w R_w_b, R_e_w R_w_b skew(p_b_g)]
    S = m.cov + J_r_xi P J_r_xi^T,     reject when r^T S^-1 r > gnss.chi2_gate

with `T_w_b` the front-end pose predicted to the fix time and `P` the latest pose covariance
(`P := 0` when any entry is non-finite, e.g. the NaN marginal of the no-GTSAM stub: neither
"reject everything" nor "accept everything"). Only accepted fixes update the aligner (a
rejected jump never moves `T_e_w`).

The aligner is `gnss::GnssAligner` (`gnss.md` section 4), or, with `gnss.align_yaw_from_track`
(ADR-023), `gnss::GnssTrackYawAligner` (`gnss.md` section 5: yaw from the GNSS track against
the body heading, alignment once its predicted yaw sigma is within
`gnss.align_yaw_max_sigma_deg`, the minimum track and coherence pass). For it,
`EstimatorCore` keeps a signed wheel odometer (trapezoid of `WheelSample::v_mps`, only with
`wheel_odom.enabled`) and passes the travel since the previous fix to `GnssFusion::on_fix`
(NaN without wheel data); the aligner takes the direction of travel of each segment from its
sign. Everything else in this section is the same for both; a realign rebuilds the one in use.

Lock-out guard. A rejected fix feeds neither the aligner nor the graph, so a front end that
drifted past the gate (for example during a fix outage while the LiDAR drifts) could never
be pulled back: every later fix would be rejected too. After `gnss.max_consecutive_rejects`
chi-square rejections in a row (an accepted fix resets the count) the aligner is rebuilt
empty (fixed-size sums: no allocation), `T_e_w` is unaligned again and the rejected fix is
the first pair of the new alignment. Once the new alignment is available, fixes are gated
against the current front end and flow again as factors; with
`backend.gnss_frame_variable` they share the graph's `E` with the factors from before the
outage. Counted in `Stats::n_gnss_realign` (bench.json `n_gnss_realign`), one warning per
restart. Test: `GnssFusion.RealignsAfterMaxConsecutiveRejects`,
`PipelineComponent.CorridorOutageRealignsAfterRejects`.

Track-yaw aligner inputs (ADR-023, design 2b, `gnss.md` section 5.1). With
`gnss.align_yaw_from_track`, `EstimatorCore` passes with each fix the signed wheel odometer
reading and the total absolute heading variation of the front end (the sum of `|heading change|`
from scan to scan, kept only while the flag is on), and `GnssFusion` builds the aligner with
`frames.T_b_o`. With `gnss.align_yaw_frontend_travel` it also passes the count of scans that did
not observe the travel axis: the match failed, or the marginal information of the scan match
along the horizontal `x_o` (`degeneracy::marginal_information`, one 6x6 eigen-solve per scan
while the flag is on) was below `degeneracy.eig_threshold` (backlog G3, `gnss.md` section 5.3).
It also passes the fix time and the sum of the travel bounds of the
wheel-sample gaps the odometer (`detail::WheelOdometer`) interpolated across: an interval
between two finite samples longer than `gnss.align_yaw_wheel_gap_periods` nominal periods (the
measured mean regular interval) adds `max(|v0|, |v1|)` times its excess over one period, and is
counted in `Stats::n_wheel_gap_bounded` (bench.json, one log line; backlog G9). Chunks
where the GNSS track and the wheel travel disagree are counted in `Stats::n_gnss_wheel_mismatch`
(bench.json `n_gnss_wheel_mismatch`), one warning until the next alignment; a count that keeps
growing while driving means wheels that slip or an odometer scale that is wrong by 2x or more.
Chunks where the front end held the robot still while the wheels moved are counted in
`Stats::n_gnss_frontend_wheel_disagree` (bench.json), one warning per run: wheels spinning on a
still robot, or a LiDAR that sees surroundings moving with the robot.

Fixes before the first alignment (ADR-022). With `gnss.attach_prealign_fixes` false (default)
a fix received before keyframe 0 (no estimate yet) or before the aligner has a transform is
dropped. With it true such fixes are kept in a buffer of `gnss.prealign_max_fixes` (reserved
once; full before keyframe 0: the oldest is dropped, after it: the new one; counted in
`Stats::n_gnss_prealign_dropped`; a fix before keyframe 0 that fails `gate_fix` counts in
`n_gnss_rejected`):

- a fix before keyframe 0 with `t_k0 - imu.init_window_s <= m.t <= t_k0` is at rest at
  keyframe 0 (only when keyframe 0 is the IMU initialisation scan and the wheels did not move):
  it is attached to keyframe 0 with no shift (`Stats::n_gnss_prealign_static` once added); any
  other fix before keyframe 0 is dropped (`n_gnss_prealign_not_static`);
- a fix after keyframe 0 (the aligner still sees it) is assigned, when the keyframe after it
  is added, to the nearer in time of that keyframe and the previous one, and shifted as below,
  using the previous keyframe's optimised reference pose for a backward shift. `T_w_b(t_fix)`
  is predicted from the front-end state of the last scan before the fix, so a backward shift
  also carries the scan-match chain from that reference pose to that scan;
- at the first keyframe where the aligner has `T_e_w` (and later ones for fixes still waiting
  for a keyframe), each assigned fix passes the chi-square gate at its fix time with the
  current `T_e_w` and becomes a factor on its keyframe (`Stats::n_gnss_prealign_attached`,
  rejections `n_gnss_prealign_rejected`). The buffer closes at the first alignment; a realign
  does not reopen it.

Accepted fixes wait for the next keyframe k (`t_k >= m.t`) and are shifted to its time with
the front-end poses:

    p_e_g' = p_e_g + R_e_w (T_w_bk p_b_g - T_w_b(t_fix) p_b_g),     cov unchanged

so the factor residual at the keyframe pose equals the residual at the fix time. With
`backend.gnss_frame_variable` (default, ADR-019) every factor shares the graph variable
`E = T_e_w`, initialised with the aligner's estimate when the first factor is added
(`backend_factors.md` section 6b); the aligner's latest `T_e_w` is still the one used by the
chi-square gate and the shift above. With it false, each factor keeps the aligner estimate
of the moment it was added, which pulls the gauge of w and made the box world 3.7x worse
(`gnss_fusion_study.md`).

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

## 8. Message hand-over and IMU initialisation window

The Estimator's k-way merge keys a scan by its end time, so the IMU sample that covers the
scan end (the first with `t >= t_end`, needed by section 2) sorts after the scan. When a
scan is dequeued, `drain` first hands the core every queued IMU sample up to and including
that one; only IMU samples move ahead of the scan, never wheel or GNSS messages, so the
processing order of everything else is unchanged.

Static initialisation at the first scan with start `t` uses the buffered samples from the
last one at or before `t - init_window_s` to the first one at or after `t`, so the window
spans at least `init_window_s` whatever the sample phase. Until it succeeds no estimate is
published (`latest_state().state.t == Timestamp{}`). The window mean and the covariance of
that mean (`imu::InitState::acc_mean_b`, `acc_mean_cov`) feed the optional keyframe-0 gravity
factor (`backend.static_gravity_factor`, default on, `backend_factors.md` section 6c) when keyframe 0 is
the initialisation scan (ADR-020).

Keyframe-0 gravity factor (ADR-020). It is added only when the window also passes the rest
gate: |mean gyro| <= `imu.init_max_mean_gyro_radps`, no wheel speed above
`imu.init_max_wheel_speed_mps` inside the window `[t - init_window_s, t]` (when
`wheel_odom.enabled`; samples are checked as they are consumed, in time order, so samples
after `t` already queued do not count), and
||f| - g| <= `imu.init_gravity_check_sigma` x sqrt(sigma_mean^2 + prior_bias_acc^2). A
failure is logged and counted (`Stats::n_gravity_factor_gated`). After keyframe
`backend.static_gravity_check_keyframes` (> 0; default 0 = off, ADR-020) is optimised, the
factor is removed if its innovation statistic (`backend_factors.md` section 6c) exceeds
`backend.static_gravity_chi2_gate` (`Stats::n_gravity_factor_removed`, one warning). A
failed removal leaves the factor (warning, no retry); a statistic that cannot be evaluated
leaves it too (`Stats::n_gravity_factor_unchecked`, one info line).

## 9. Test-only seam

`detail::CoreTestHooks` (`estimator_core.hpp`, `set_core_test_hooks`) lets the studies and
component tests observe factors, force `T_e_w`, change the keyframe rule, replace a LiDAR
measurement or covariance, override the gravity factor's covariance, and replace the shift
of a GNSS fix to its keyframe (section 5) with a given displacement (`gnss_shift_oracle`,
`gnss_fusion_study.md` section 13). The core keeps one
`const CoreTestHooks*`, read at construction. It is never set outside tests: production
builds pass nullptr, and every branch on it is a null-pointer test.
