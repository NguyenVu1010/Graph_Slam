# GNSS: geodesy, residual, chi-square gate, ENU alignment

Implemented in `core/gnss/` (`mss::gnss`). Tested by `tests/unit/test_gnss_*.cpp` and
`tests/component/test_gnss_component.cpp` (G-01 to G-04 in `docs/Test_CI_CD.md`).
Frames and perturbation follow `docs/conventions.md` and `docs/derivations/lie_conventions.md`:
`e` is ENU (x east, y north, z up), `w` the SLAM world frame, `b` body, `g` GNSS antenna;
right perturbation `T <- T * Exp(xi)`, `xi = [rho, phi]`.

## 1. Geodesy (WGS-84)

Code: `lla_to_enu`, `enu_to_lla` in `core/gnss/src/geodesy.cpp`. The algorithm and constants
are copied from `sim/mss_sim/geodesy.py` so C++ and Python agree to about 1e-12 relative.

Constants: `a = 6378137.0` m (`kWgs84AM`), `f = 1/298.257223563` (`kWgs84F`),
`b = a (1 - f)` (`kWgs84BM`), `e2 = f (2 - f)` (`kE2`), `ep2 = e2 / (1 - e2)` (`kEp2`).

LLA to ECEF (`lla_to_ecef`), with `lat`, `lon` in radians and `h` in metres:

    N = a / sqrt(1 - e2 sin^2 lat)
    x = (N + h) cos lat cos lon
    y = (N + h) cos lat sin lon
    z = (N (1 - e2) + h) sin lat

ECEF to LLA (`ecef_to_lla`), Bowring, `kBowringIterations = 5`:

    p    = hypot(x, y)
    lon  = atan2(y, x)                      (in (-pi, pi])
    beta = atan2(z a, p b)
    repeat 5 times:
      lat  = atan2(z + ep2 b sin^3 beta, p - e2 a cos^3 beta)
      beta = atan2((1 - f) sin lat, cos lat)
    h = p cos lat + z sin lat - a^2 / N(lat)

Rotation ECEF to ENU at the origin (`r_e_ecef`), rows:

    [ -sin lon,           cos lon,          0       ]
    [ -sin lat cos lon,  -sin lat sin lon,  cos lat ]
    [  cos lat cos lon,   cos lat sin lon,  sin lat ]

    p_e    = R_e_ecef (x_ecef - x0_ecef)
    x_ecef = x0_ecef + R_e_ecef^T p_e

## 2. GNSS position residual and Jacobian

Code: `detail::gnss_residual` in `core/gnss/src/innovation.hpp`.

Residual (3x1, metres, expressed in e):

    r = p_e_g - T_e_w (T_w_b p_b_g)
      = p_e_g - R_e_w (R_w_b p_b_g + p_w_b) - p_e_w

Variable: `T_w_b`, right perturbation `T_w_b <- T_w_b Exp([rho, phi])`. To first order:

    p_w_b                <- p_w_b + R_w_b rho
    R_w_b Exp(phi) p_b_g ~= R_w_b p_b_g + R_w_b skew(phi) p_b_g
                          = R_w_b p_b_g - R_w_b skew(p_b_g) phi

so

    J_r_xi = [ -R_e_w R_w_b ,  R_e_w R_w_b skew(p_b_g) ]     (3x6, columns [rho, phi])

## 3. Chi-square innovation gate

Code: `detail::innovation_nis`, `detail::chi_square_gate`.

The caller forms the innovation covariance

    S = Sigma_fix + J_r_xi P J_r_xi^T

with `P` the 6x6 `[rho, phi]` covariance of `T_w_b`. The normalised innovation squared is
`NIS = r^T S^-1 r`, computed with an LLT of `S` (fixed size, no heap). Non-finite `r` or
`S` and a `S` that is not positive definite are errors (a NaN NIS would otherwise compare
false against the threshold and pass). The fix is rejected when `NIS > chi2_3(q)`:

| q    | chi2, 3 DoF        |
|------|--------------------|
| 0.95 | 7.814727903251178  |
| 0.99 | 11.344866730144373 |

The module uses q = 0.95 (`detail::kChi2Gate3Dof95`): consistent fixes are accepted 95% of
the time, the middle of the 90-99% band of G-02.

## 4. 4-DoF alignment of w to e

Code: `GnssAligner` in `core/gnss/src/aligner.cpp`.

w and e are both gravity-aligned with z up, so `R_e_w = Rz(yaw)` and `p_e_w` is 3-D.
Pairs: `p_w_g,i = T_w_b,i p_b_g` and `p_e_g,i`. For conditioning (ENU coordinates can be
of order 1e5 m), every pair is shifted by the first one:

    a_i = p_w_g,i - p_w_g,0        b_i = p_e_g,i - p_e_g,0

Running sums: `n`, `sum a`, `sum b`, `C = sum a_h b_h^T` (2x2, horizontal parts),
`sum |a_h|^2`. Centred cross-covariance:

    C_c = C - (sum a_h)(sum b_h)^T / n

Maximising `sum b_c^T Rz(yaw) a_c` gives

    yaw = atan2(C_c(0,1) - C_c(1,0), C_c(0,0) + C_c(1,1))

and the translation, with the un-shifted means,

    p_e_w = mean(p_e_g) - Rz(yaw) mean(p_w_g)
          = (p_e_g,0 + sum b / n) - Rz(yaw) (p_w_g,0 + sum a / n)

Use of the result: the pipeline gates fixes with this `T_e_w` (chi-square, `pipeline.md`
section 5) and uses its rotation to shift fixes to keyframe time. With
`backend.gnss_frame_variable` (ADR-019, default) it only seeds the graph's variable
`E = T_e_w` (`backend_factors.md` section 6b); the factors use `E`, not this estimate.

Horizontal spread (RMS distance of `p_w_g,h` from its centroid):

    spread = sqrt(max(0, sum |a_h|^2 / n - |sum a_h / n|^2))

The same spread is computed on the e side from `sum b` and `sum |b_h|^2`. If either side is
below `kMinHorizontalSpreadM`, yaw is unobservable and no transform is returned. The e-side
check catches a receiver that repeats or holds one fix while the body moves: then
`C_c ~ 0` and `atan2(~0, ~0)` would give an arbitrary yaw.

Observability: `sigma_yaw ~ sigma_h / sqrt(sum |a_c,h|^2)`. A straight 30 m path at 1 m/s
and 5 Hz has 151 fixes and `sum |a_c,h|^2 ~ n L^2 / 12 ~ 151 * 900 / 12 ~ 1.13e4 m^2`, so at
`sigma_h = 1.0` m `sigma_yaw ~ 1 / 106 rad ~ 0.54 deg`. G-03 therefore uses
`sigma_h = 0.2` m (1-sigma about 0.11 deg) to test the 0.5 deg requirement with margin.

### 4.1 Predicted yaw sigma and the yaw check (backlog G1)

The spread check is a fixed 1 m and does not scale with the fix noise. On E-01 (fixes with
1.0 m white horizontal noise on a 1.67 m circle) it passes at 13 s with a 7.6 deg yaw sigma, and
the first fit is 21 deg off (`gnss_fusion_study.md` section 17). The e-side spread also passes
on noise alone: 1.72 m there against 1.01 m of true spread.

Model. The w side (front-end antenna positions) is taken as exact; fix `i` has white noise
`n_i` with covariance `C_i` in e. Linearise the fit of this section about the true yaw:
`b_c,i = Rz(yaw) a_c,i + n_c,i`. The score `sum b_c^T Rz(yaw) a_c` is stationary where

    delta_yaw = sum_i (Rz(yaw) J a_c,i) . n_i / sum_i |a_c,i|^2,     J = [[0, -1], [1, 0]]

(the centring of `n` drops out because `sum a_c,i = 0`). Its variance is
`sum_i u_i^T C_i u_i / (sum |a_c,i|^2)^2` with `u_i = Rz(yaw) J a_c,i`, `|u_i| = |a_c,i|`. The
aligner bounds `u^T C_i u` by `var_i |u|^2` with `var_i` the larger eigenvalue of the
horizontal 2x2 block of the declared covariance, floored at `gnss.align_yaw_min_var_m2` (as
section 5; a declared variance <= 0 gets the floor), so that the yaw does not need to be known:

    sigma_yaw^2 = sum_i var_i |a_c,i|^2 / (sum_i |a_c,i|^2)^2

With `a_c,i = a_h,i - m`, `m = sum a_h / n`, the numerator expands over three running sums
(`S0 = sum var_i`, `S1 = sum var_i a_h,i`, `S2 = sum var_i |a_h,i|^2`):

    sum_i var_i |a_c,i|^2 = S2 - 2 m . S1 + |m|^2 S0

and the denominator is `n spread_w^2`. Equal variances give the observability rule above,
`sigma / sqrt(sum |a_c,h|^2)`. `GnssAligner::yaw_sigma_rad()` reports it at every fit, and
`GnssFusion::align_yaw_sigma_rad()` passes it to the study observer.

With `gnss.align_check_yaw_sigma` (default true since G13; false is the previous behaviour) the aligner returns no
transform while `sigma_yaw > gnss.align_yaw_max_sigma_deg` (the same threshold as section 5,
6 deg), error "yaw sigma above threshold"; the pair is still added. The 3-argument `update()`
has no covariance and uses the floor for every fix.

Front-end covariance (backlog G15, `gnss.align_yaw_sigma_frontend_cov`, default false). The
w side carries the front end's error `e_i`; to first order it moves the yaw by
`-sum_i (J a_c,i) . e_c,i / sum_i |a_c,i|^2`. Along a LiDAR-degenerate axis `e_i` is dead
reckoning: one correlated error (the corridor front end ran backwards, a scale of -1 along the
axis), not independent noise. For errors of any correlation

    Var(sum_i u_i . e_i) <= (sum_i |u_i| s_i)^2 <= (sum_i |u_i|^2) (sum_i s_i^2)

with `|u_i| = |a_c,i|` and `s_i^2` the larger horizontal eigenvalue of the antenna position
covariance `J P_w_b J^T`, `J = d(T_w_b p_b_g)/d[rho, phi] = [R_w_b, -R_w_b skew(p_b_g)]`, from the
pose covariance the pipeline passes (the one of the chi-square gate; non-finite: zero). So

    sigma_yaw^2 = sum_i var_i |a_c,i|^2 / (sum_i |a_c,i|^2)^2 + sum_i s_i^2 / sum_i |a_c,i|^2

one more running sum. The bound does not average down with the number of fixes: a constant
`s` gives `s / spread`, so the alignment waits until the spread is large against the front-end
uncertainty. Counting the `s_i` as independent, as the fix noise, would average it away: with
1000 m^2 along a reversed axis that version aligned 180 deg off with a 6 deg sigma after 96 s
(`GnssAligner.FrontEndCovCatchesADeadReckonedAxis`). The pose covariance is absolute (it holds
the error common to all fixes, which the centring removes), so the term is an upper bound.
Numbers: `gnss_fusion_study.md` section 18.2.

Limits.
- Without `gnss.align_yaw_sigma_frontend_cov` the front end is taken as exact. Along a
  LiDAR-degenerate axis its positions are dead reckoning (section 5), and the sigma is then
  optimistic: the check does not prevent the corridor's 180 deg starts (seeds 51, 52), which
  need section 5 or the front-end covariance.
- The noise is taken as white and honestly declared. A slow common drift cancels in the
  centring; multipath steps and drift that changes within the window do not.
- The sigma does not enter the prior of `E` (ADR-019 keeps 30 deg; changing that needs
  `graph.hpp`), so the check only delays the alignment until its yaw is supported.

## 5. Yaw of `T_e_w` from the track direction (ADR-023)

Code: `GnssTrackYawAligner` in `core/gnss/src/track_yaw.cpp`, used instead of section 4 when
`gnss.align_yaw_from_track` is true.

Why. Section 4 fits the front-end positions `p_w_g,i`. Along a LiDAR-degenerate axis their
along-axis part is IMU dead reckoning. In the corridor of `gnss_fusion_study.md` section 13 that
velocity even changes sign before the first alignment (seed 51: -1.7 m/s against +0.5 m/s true,
at a heading error below 0.2 deg), and the fit starts 180 deg off. The heading, from the gyro
and the LiDAR rotation, is accurate there, so this estimator takes the yaw from the heading, the
GNSS track (course over ground) and, when available, the sign of the wheel speed.

Model. As section 4, `R_e_w = Rz(yaw)`. The body moves along its forward axis `x_b`, forwards
or backwards: its horizontal velocity in w is `v(t) (cos psi, sin psi)`, with `psi` the heading
of `(R_w_b x_b)_h` and a signed speed `v(t)` whose magnitude is not used. In e, the horizontal
antenna displacement over an interval is `Rz(yaw) * integral v (cos psi, sin psi) dt`, plus the
lever-arm term below.

Assumptions, each a stated limit:

- **Direction of travel.** With wheel odometry, the sign of the wheel travel decides forward or
  backward per segment. Without wheel data, the net GNSS displacement along the heading must be
  forward until the estimate is confident. If it is backwards (for example 3 m forward, then a
  longer move backwards; test `GnssTrackYaw.NetBackwardMotionBeforeConfidenceFlips`), the
  estimate converges to `yaw + 180 deg`, and later forward segments are then read as reversals.
- **White, honestly declared fix noise.** `sigma_yaw` uses the declared horizontal variance of
  each fix, floored at `gnss.align_yaw_min_var_m2`, and treats fixes as independent. A fix with
  a declared horizontal variance `<= 0` on either axis is rejected ("non-positive
  covariance"). Coloured error (multipath, the slow wander of a parked receiver) is not in
  `sigma_yaw`; the minimum track, the coherence test and, with wheel data, the stationary skip
  bound its effect (envelope below).
- **Heading turn below pi between consecutive fixes** (15.7 rad/s at 5 Hz), so the heading can
  be unwrapped.

Heading. `psi` is unwrapped from fix to fix: `psi_i = psi_{i-1} + atan2(u_{i-1} x u_i,
u_{i-1} . u_i)`, with `u` the unit horizontal heading. The unwrapped value is what makes the
turn gate below measure the real turn: a mean of unit vectors cannot tell a 350 deg spin from
a 10 deg one.

Chunks. Consecutive fixes are grouped by `m = gnss.align_yaw_chunk_fixes`. Per chunk `k`:

- the mean horizontal fix position `c_k` (e, shifted by the first fix for conditioning);
- the mean unwrapped heading `psi_k` and its span (max - min);
- the mean horizontal lever arm `m_k = mean (R_w_b p_b_g)_h` (w);
- the per-axis variance of `c_k`, `v_k = sum_i max(var_i, floor) / m^2`, with
  `var_i = (cov_i(0,0) + cov_i(1,1)) / 2`;
- the signed wheel travel of its first and of its second half, and whether every fix there
  came with wheel data.

Segments. Segment `k` joins chunks `k` and `k + 1`:

    b_k = c_{k+1} - c_k   (e)      l_k = m_{k+1} - m_k   (w)
    a_k = s_k (cos, sin)((psi_k + psi_{k+1}) / 2)   (w),  s_k = +-1

The wheel travel of a fix is the difference between the absolute odometer reading at that fix
and at the last fix the aligner added, so the travel during fixes rejected before the aligner
(gate, chi-square, a non-positive covariance) is carried to the next one, not lost.

On an arc of constant curvature and speed, the centroids of two chunks lie at the same radius,
so the chord between them is along the tangent at the mean angle, which is `a_k`. It changes
sign only when the turn reaches `2 pi`. Even so, the chord bends with speed changes along the
arc, and course over ground is a poor heading reference in turns. So a segment is used only if:

- **Turn gate.** `|psi_{k+1} - psi_k|` and the span of both chunks are at most
  `gnss.align_yaw_max_turn_deg` (45 deg). Otherwise `a_k = 0` and it is counted in
  `n_skipped_turns()`. 45 deg keeps the box world's 2 m circle at 0.25 rad/s (29 deg per 2 s
  chunk) and E-01 (34 deg). A fast spin is never used, so it delays the alignment and never
  flips it (test `GnssTrackYaw.FastTurnNeverFlips`: no transform above about 0.08 rad per fix,
  exact below).
- **Direction `s_k` from the wheel.** When every fix of the second half of chunk `k` and of the
  first half of chunk `k + 1` came with wheel data (the motion between the two centroids),
  `s_k` is the sign of that wheel travel. A travel below `gnss.align_yaw_min_wheel_travel_m`
  (0.2 m) is a stationary segment and is skipped (`a_k = 0`, `n_stationary_segments()`): it
  carries no direction, and the random sign of wheel noise at rest would alternate `a_k`.
  Consequence: with 2 s between chunk centres, a robot slower than about 0.1 m/s never aligned
  with wheel data (test `GnssTrackYaw.LowSpeedWithWheelNeverAligns`, struct defaults; without
  wheel data the same track aligns). Since the root-cause study a stationary segment is extended
  over up to `gnss.align_yaw_merge_max_chunks` more chunks instead (section 5.2).
  Before this rule, E-01 with the flag aligned so late that it got 8 GNSS factors instead of 65
  (test `GnssTrackYaw.StationaryWheelNoiseDoesNotDelay`).
- **No wheel data (fallback).** `s_k = +1` (forward driving), counted in
  `n_no_wheel_segments()`. Once the estimate is confident, a segment whose lever-corrected GNSS
  displacement points against the heading rotated into e by more than
  `gnss.align_yaw_reverse_nsigma` times its sigma gets `s_k = -1`.

(The first version used `a_k = unit(h_k + h_{k+1})` of the mean unit headings. That bisector
points backwards once the turn exceeds pi, which gave an exact 180 deg flip with a sigma below
0.5 deg on a fast spin (review I1). It also took the direction of travel from the forward
assumption alone (review I2).)

Lever arm and the closed form. The segment joins antenna positions, but the heading is the
body's. The antenna also moves by the rotation of the lever arm, `l_k = m_{k+1} - m_k`. It
depends only on the rotation, so it is known in w. On an arc of curvature `kappa` it turns the
chord by about `p_x kappa`, for example 0.2 m x 0.5 1/m = 5.7 deg on the box world's 2 m circle.
It is removed: `b'_k = b_k - Rz(yaw) l_k`.

In complex notation (`x + i y` for a 2-D vector), the estimator is the 2-D Procrustes solution
`e^{i yaw} = sum conj(a_k) b'_k / |...|`. With

    Z = sum conj(a_k) b_k = sum a.b + i sum a x b
    L = sum conj(a_k) l_k = sum a.l + i sum a x l

this reads `Z - e^{i yaw} L = r e^{i yaw}` with `r > 0`, so `Z = e^{i yaw} (r + L)` and
`|Z|^2 = (r + Re L)^2 + (Im L)^2`:

    r = -Re(L) + sqrt(|Z|^2 - Im(L)^2)        e^{i yaw} = Z / (r + L)

This is exact on a constant arc for any lever arm. It needs only four running sums, and has a
defined failure: when `|Z|^2 < Im(L)^2` or `r <= 0` there is no solution with net motion along
`a`, and the aligner reports "not enough track". The larger root is the one with the antenna
moving along `a` on net (`r + Re L > 0`).

(The first version iterated `yaw = atan2(...)` four times. Its contraction factor
`|l_k| / |b_k|` approaches 1 as the radius approaches the lever arm, which left -3.96 deg at
R = 0.2 m with a sigma of about 1 deg (review I1). Test `GnssTrackYaw.TightArcWithLeverArmExact`
checks R = 0.2, 0.3 and 2 m with the box-world lever arm to 1e-9 rad.)

Noise. `c_k` enters segment `k - 1` with `+a_{k-1}` and segment `k` with `-a_k`, so the noise
of `sum a x b` is `sum_k (a_{k-1} - a_k) x n_k`. Here `a_{-1} = a_K = 0`, and a skipped
segment counts as `a = 0`. Since `r` and `arg(r + L)` depend on `Z` only through `|Z|`, to
first order `delta_yaw = Im(conj(Z) delta_Z) / |Z|^2`. For isotropic horizontal noise
(variance `v_k` per axis):

    sigma_yaw = sqrt(sum_k v_k |a_{k-1} - a_k|^2) / |Z|,      |Z| = |r + L|

(The first versions divided by `r`. The two differ when the lever arm turns: on the box
circle `|r + L| / r` is about 0.95, on a 1 m circle about 0.92, so the sigma was optimistic.
Test `GnssTrackYaw.CircleWithLeverArmSigmaConsistent`: 2000 runs on a 1 m circle with the
box lever arm give a normalised squared yaw error of 1.03; re-review N2.)

On a straight line `a` is constant, the sum telescopes to the first and the last chunk, and
`sigma_yaw = sqrt(v_0 + v_K) / L` (no lever-arm turn, so `|Z| = r`), with `L` the distance
between them. At 0.5 m per axis and
`m = 10`, `v = 0.025 m^2`, so `L = 4.3 m` gives 3 deg. Heading noise is neglected (below
0.2 deg in the corridor), and `l_k` carries no noise term. The running sums give
`sum v_k |a_{k-1} - a_k|^2` in O(1): chunk `k`'s term is added when segment `k` is formed, and
the last chunk's term at evaluation.

Acceptance ("confident"). A transform is returned once all of these hold:

1. `r >= gnss.align_yaw_min_track_m` (5 m). A minimum length of track supporting the yaw: a
   0.3 m common error over 5 m of track is 3.4 deg. When every segment used has its direction
   from wheel data, `gnss.align_yaw_min_track_wheel_m` (3 m) instead (section 5.1).
2. `r >= gnss.align_yaw_min_coherence * sum |b_k|` (0.5), over the segments used (with wheel
   data also against the wheel-predicted displacement, section 5.1). For steady
   motion the ratio is near 1. For a parked receiver whose error wanders, the segment
   directions are random, so `r` grows like `sqrt(K)`, `sum |b_k|` like `K`, and the ratio
   falls like `1 / sqrt(K)`.
3. `sigma_yaw <= gnss.align_yaw_max_sigma_deg` (3 deg in ADR-023, 6 deg since design 2b,
   section 5.1).

Together these replace `align_min_pairs` and `align_min_spread_m`. No motion, or a frozen
receiver, gives `r ~ 0` and no transform.

Parked robot with drifting GNSS: a random walk of `step` per fix plus 0.3 m white noise,
declared 0.5 m, 5 minutes, 100 seeds. At 0.05 m per fix, by gate:

| gate | seeds aligned |
| --- | --- |
| sigma only (first version) | 13, up to 180 deg off (review probe) |
| plus the 5 m minimum track | 6 |
| plus the coherence 0.5 (shipped) | 0 |

The envelope of what ships (test `GnssTrackYaw.ParkedDriftingGnssEnvelope`; the re-review
probe used other seeds):

| random walk per fix | without wheel data | with wheel data |
| --- | --- | --- |
| 0.05 m | 0 of 100 | 0 of 100 |
| 0.10 m | 0 of 100 (probe: 3 of 100, up to 154 deg off) | 0 of 100 |
| 0.20 m | 16 of 100, up to 176 deg off (probe: 27 of 100, up to 178 deg) | 0 of 100 |

So without wheel data a parked robot whose GNSS wanders by 0.1 m per fix or more can align
on noise. The IMU gives no stationary signal for that case: the rest detector used at
keyframe 0 tests the spread of acceleration and rate, which is the same at rest and at
constant velocity (in the simulator both are pure sensor noise, 0.08 m/s^2 per sample), so it
would also block the straight cruise that the corridor needs. Where the LiDAR observes the
travel axis the front end is that signal (section 5.2: none of 100 align up to 0.30 m per fix);
where it does not, this is a sensor-set limit (ADR-023).

Variance floor (test `GnssTrackYaw.VarianceFloor`): fixes declared at 1 cm with 0.5 m true noise,
100 seeds. Without the floor, 48 seeds are more than 10 deg off at the first transform (95
beyond 3 sigma); with the 0.25 m^2 floor, none (0 beyond 3 sigma).

Translation. As section 4, `p_e_w = mean(p_e_g) - Rz(yaw) mean(p_w_g)` over all fixes. It still
carries the front-end along-axis offset; `gnss.attach_prealign_fixes` (ADR-022) anchors the
translation of the graph's `E` by measurements.

Roll and pitch stay zero: `E`'s tight tilt prior of ADR-019 is unchanged.

Pipeline input. `EstimatorCore` keeps a signed wheel odometer (trapezoid of
`WheelSample::v_mps`) when `wheel_odom.enabled`, and passes the travel since the previous fix
to `GnssFusion::on_fix`. That travel is NaN when wheel odometry is off, or when no wheel sample
arrived since the previous fix.

### 5.1 Design 2b: less motion before the alignment (ADR-023 amendment)

Code: the same class. Keys `gnss.align_yaw_max_sigma_deg` (default 3 -> 6 deg),
`gnss.align_yaw_wheel_turns`, `gnss.align_yaw_wheel_slip_frac` and
`gnss.align_yaw_min_track_wheel_m`; the odometry frame `frames.T_b_o`. Measurements:
`gnss_fusion_study.md` section 16.

Waiting for the alignment does not limit local navigation. The LiDAR-inertial pose in w is
valid from keyframe 0; only the ENU (global) output waits for `T_e_w`.

**What sets the distance to the alignment.** On a straight track `sigma_yaw = sqrt(2 v) / L`
(section 5), so with standard GNSS (0.5 m per axis, `v = 0.025 m^2` per 10-fix chunk mean at
5 Hz) the distance between the first and the last chunk centre is `L = 0.224 m / sigma_yaw`: 4.3
m for 3 deg, 2.8 m for 4.5 deg, 2.1 m for 6 deg. The alignment is decided when a chunk closes
(every 2 s), and at 3 deg the 5 m minimum track, not the sigma, binds on a straight track. On a
turn the chunk directions change, so the sum telescopes less and the sigma falls more slowly
(2 m circle: 14 s at 3 deg). So the levers, in the order they act: the sigma threshold, the
minimum track, the segments the turn gate discards. A fit of the known path shape (section 4 on
an exact w path) would need about 15 % less distance on a straight track and about 30 % less
time on the 2 m circle (study section 16.1); it needs a w path that is right along the track,
which a degenerate corridor without wheel data does not have, so it is not built.

**Sigma threshold 6 deg (proposed; the change of target is pending the owner's decision).** The
graph refines `E`'s yaw after the alignment (prior 30 deg), and the pipeline's chi-square gate
and fix shift keep using the aligner's `T_e_w`, which is re-estimated at every chunk after the
alignment. Bounded-error argument: on a straight track the yaw sigma after a track `L` is about
`0.224 m / L`, and the position error that a yaw error causes at a distance `d` from the fit
centroid is `d delta_yaw`; with `d` at most about `L / 2` for the fixes in the window, a 3-sigma
error is about `0.34 m`, inside the gate (`sqrt(7.81) x 0.5 m = 1.4 m`). Early fixes are near the
centroid, and the lock-out guard (`max_consecutive_rejects`) bounds the rest. Measured at 3 / 4.5
/ 6 deg (study section 16): the first-alignment yaw within its predicted sigma (NEES 0.66 to
1.54 over 15 seeds per scene, none beyond 3 sigma), no flip of `E`, no more chi-square
rejections after the alignment, fused ATE unchanged or better; a first `T_e_w` forced 18 deg off
converges (test `PipelineComponent.TrackYawConvergesFromEighteenDegrees`: `E`'s prior 20.8 /
-15.2 deg off, `E` within 0.1 deg at the end, no realign). The current target of study section
13.6, "below 10 deg at the first factor", is a proxy for "no flip and no lock-out"; at 6 deg a
first error of up to 3 sigma (about 15 to 18 deg) is possible. The proposal is to state the
target as "no flip, no lock-out"; until the owner decides, the 10 deg target is not met in the
tails. The evidence covers white noise; with coloured error the tails are larger (minimum track
below).

**Turns from the wheel path** (`align_yaw_wheel_turns`). Course over ground along the mean
heading is only trusted on near-straight segments: the chord between two chunk means on a turn
bends with the speed profile inside the turn, by up to half the turn angle. With wheel data the
shape of the path in w is known. The odometer measures the forward speed of the odometry frame
o (`frames.T_b_o`, the axle centre), so the path `q` of o is integrated from the wheel travel
along o's heading `psi` (the travel axis, below), one step per fix,

    q_i = q_{i-1} + s_i sinc(dpsi_i / 2) (cos, sin)(psi_{i-1} + dpsi_i / 2)

with `s_i` the odometer difference and `dpsi_i` the heading change since the previous fix
(exact for constant speed and yaw rate between the two fixes). Per chunk the mean `Q_k`, and the
mean lever arm from o, `mo_k = mean (R_w_b (p_b_g - p_b_o))_h`. A segment the turn gate rejects
is then used, when every step into the fixes of both chunks had wheel data, with

    g_k = Q_{k+1} - Q_k + mo_{k+1} - mo_k   (w, antenna displacement)
    a_k = g_k / |g_k|

The lever arm is inside `g_k`, so the segment adds nothing to `L`, and the closed form of
section 5 holds unchanged. The direction of travel is inside `g_k` too. Ignoring `T_b_o` (the
first version) rotated every chord on a constant turn: IMU 0.2 m ahead of the axle on a 1 m
circle gave a median yaw error of 12 deg, NEES 5.5; with it, NEES 0.87 (test
`GnssTrackYaw2b.WheelPathUsesOdometryFrame`). Segments that pass the gate keep the heading
direction: on a constant arc the two agree.

**Travel axis** (fix round 2, review N1). A wheeled robot moves along the x axis of its odometry
frame, so every direction of travel, for the heading segments, the wheel path and the
forward-driving assumption alike, is the heading of `x_o = R_w_b R_b_o (1, 0, 0)`, horizontal
part, unwrapped from fix to fix; the lever arm of the heading segments is also taken from o,
`R_w_b (p_b_g - p_b_o)`, so they follow the point that moves along `x_o`. With `T_b_o` identity
this is the body x axis and the body lever arm, bit for bit as before. The horizontal projection
of `x_o` is exact for any mount, including a tilted one; the assumption is that the robot moves
along `x_o` (not sideways), and a mount whose `x_o` is vertical has no heading ("heading
undefined"). Before, the heading segments used `x_b`: a mount yaw of 5 deg gave NEES 2.8 on a
straight track, 90 deg a 90 deg error, 180 deg a silent flip. Test
`GnssTrackYaw2b.MountYawSetsTheTravelAxis` (5, 90 and 180 deg; straight and 1 m circle; NEES
0.81 / 1.03).

**Turns hidden between fixes** (fix round 2, review N2, also in design 2). The aligner sees the
heading only at the fixes it receives. A GNSS outage over an S-jog (a turn and its opposite)
leaves a net heading change of zero, so the turn gate passed it and a heading segment received
the sideways GNSS displacement: on aisles, an outage over 6-16 s aligned 100 of 100 runs beyond 3
sigma (median 26 deg off), with or without wheel data, in design 2 as well. The pipeline now
passes, with each fix, the total absolute heading variation of the front end (the sum of
`|heading change|` of the travel axis `x_o` scan to scan, about 10 Hz; `EstimatorCore`, only with
`align_yaw_from_track`; read from `x_o`, not `x_b`, so that an IMU mounted with its x axis near
vertical does not feed attitude noise into the gate, fix round 3; the wiring is tested by
`PipelineComponent.TrackYawHeadingVariationUsesTheTravelAxis`, fix round 4). Per step the aligner takes its difference since the last fix it added,
at least the net change (the net change alone when no reading exists); it sums the steps per
chunk half, and the turn gate uses the larger of the net change between the chunk centres and the
total variation of the steps between them (for a constant arc the two are equal, so nothing
changes there). A step whose total variation exceeds `align_yaw_max_turn_deg` has no known arc
shape and is not used through the wheel path (a GNSS outage in a turn: a 9 s outage in a 1 rad/s
turn now aligns in 51 s, 36 s without the outage, 137 s in the first version). Test
`GnssTrackYaw2b.OutageOverAnSJogIsGated` (aisles, outages over 6-16, 8-18 and 10-20 s, with and
without wheel data, 100 seeds): at most 1 of 100 beyond 3 sigma, NEES 0.88 to 1.28; the 8-18 s
case with the wheel path, which showed a bias of 10.9 deg (NEES 3.7) before, is at 3.4 deg, NEES
1.28. Without the readings (and without wheel data) 100 of 100 are still beyond 3 sigma: the
gate needs them. No maximum segment duration was added: straight and circular outages are
harmless (NEES 0.92), only hidden turns matter. The scan-to-scan sum can miss the turn inside one
scan interval at each end of a step (about 0.1 rad at 1 rad/s), far below the 45 deg gate.

Error model of the wheel path, stated:

- Wheel slip (scale) and skid (motion not along x_o): `align_yaw_wheel_slip_frac` (0.05) of the
  path, along and across the motion, fully correlated inside each chunk (2 s), independent
  between chunks.
- Unknown speed profile between two fixes: the step direction is within `+-dpsi_i / 2` of the
  mid heading `u_i`, uniform, independent per step: variance `(s_i dpsi_i)^2 / 12` across `u_i`.
- The heading itself: neglected (gyro and LiDAR rotation, below 0.05 deg before the alignment on
  the 15 box seeds).

A step into fix `i` of chunk `k` enters segment `k - 1` (the chunk is the later one) with weight
`w_f = (n - i) / n` and segment `k` with `w_r = i / n`. With `c_k` the direction of segment `k`
when it comes from the wheel path (0 otherwise), a step error `e` moves `Z` across by
`e x (w_f c_{k-1} + w_r c_k)`, since `conj(c) b` is about `|g| e^{i yaw}`; an error `e_perp`
across `u_i` does so by `e_perp (w_f c_{k-1} + w_r c_k) . u_i`. So chunk `k` adds

    (slip (F_k |c_{k-1}| + R_k |c_k|))^2                                (slip, triangle bound)
    + c_{k-1}^T P_f,k c_{k-1} + c_k^T P_r,k c_k + 2 c_{k-1}^T P_x,k c_k   (speed profile)

with `F_k = sum w_f |s|`, `R_k = sum w_r |s|` and `P_f, P_r, P_x` the 2x2 sums of `w_f^2`,
`w_r^2`, `w_f w_r` times the step's profile variance times `u_i u_i^T`. This is the exact variance
of the shared steps (review M2 of round 1; round 2 added the projection on `u_i`, without which
the term was conservative by about 1.8x at 0.3 rad per fix) and adds to the numerator of
`sigma_yaw` (test `GnssTrackYaw2b.ProfileVarianceOfSharedStepsIsConsistent`: NEES 0.98 over 2000
runs where this term dominates):

    sigma_yaw = sqrt(sum_k v_k |a_{k-1} - a_k|^2 + sum_k (chunk k wheel-path term)) / |Z|

A segment with a weighted path below `align_yaw_min_wheel_travel_m` is stationary
(`n_stationary_segments()`), so a spin in place has no direction of travel and is never used.
Without wheel data, or with a missing reading in either chunk, the turn gate applies as before
(bit-identical results), and the segment breaks the wheel/GNSS consistency window below.

The variation inside a chunk (its span) is not covered by the variation between chunk centres
(fix round 3, review R2-3): every step lies between exactly one pair of centres, so a turn
hidden in one step gates that segment, but the chunk holding the step mixes positions from before
and after the hidden S-jog in its centroid, which biases the other segment that uses the chunk.
Test `GnssTrackYaw2b.ShortOutageOverAQuickSJogInsideAChunk` (a 2.8 s outage over a quick S-jog at
10 positions in a chunk, 200 runs): NEES 0.62, none beyond 3 sigma; without the span term 2.06 and
5 beyond 3 sigma.

Tests (`test_gnss_track_yaw_2b.cpp`): exact on a noise-free 1 m circle where every segment is
gated; aisles with 2 m straights and 90 deg corners at 1 rad/s, 200 seeds: at 3 deg, without the
wheel path 2 of 200 runs aligned within the path (median 33.7 m driven), with it all 200 (median
7.9 m, NEES 0.85); at 6 deg 8.9 -> 2.6 m (NEES 1.03); with 5 % slip and skid drawn per turn NEES
0.99, none beyond 3 sigma; spin in place never aligns.

Limit (pinned, `GnssTrackYaw2b.PersistentSkidPinned`): a skid that persists through a long turn
in one direction (a skid-steer robot's offset centre of rotation does this) is correlated between
chunks, outside the model. On a 1 m circle at 6 deg: 5 deg of skid, NEES 1.6, 1 of 200 beyond 3
sigma; 10 deg, NEES 4.3, 24 of 200 beyond 3 sigma, up to 24 deg off. Alternating turns cancel it.

**Wheel/GNSS consistency window** (round 1 and fix round 2, reviews I1 and N3). With wheel data
on every segment, a robot whose wheels turn while the body does not move (stuck against an
obstacle, lifted on a stand) gives segments with the heading as direction and GNSS drift as
displacement; the odometer does not see that. Each segment with wheel data has a
wheel-predicted displacement `d_k`: the wheel travel between the two chunk centroids, weighted as
the steps enter them (exact on a straight track, also while accelerating), or `|g_k|` for a
wheel-path segment. Besides all segments, the aligner sums the segments since the last
mismatch; a mismatch is

    | r_recent - sum_recent d_k | > align_yaw_reverse_nsigma x sigma_recent + slip x sum_recent d_k

(their noise across Z, `sigma_recent`, and the slip fraction; counted in `n_wheel_mismatches()`,
`Stats::n_gnss_wheel_mismatch`). Moving, `r / sum d` is about 1 (a chord against the same chord);
stuck, `r` is the drift while `sum d` grows. After a mismatch every older segment is suspect: no
transform ("track not coherent") until the segments since the mismatch agree with the wheels over
`align_yaw_wheel_restart_m` (10 m) of wheel travel, and then they replace all segments
(`n_wheel_restarts()`). A segment without wheel data (a missing odometer reading) cannot be
checked: it breaks the window, which starts again at the next segment, and a pending restart stays
pending (fix round 3, review R2-1). Before, such a segment switched the check off for good: after
a mismatch, one missing reading while driving locked the alignment out (0 of 100 aligned in 540 s
of driving; now 100 of 100 after 12 to 18 m, NEES 0.88 to 1.10), and without one the stuck-wheel
check was lost (0.20 m per fix: 20 of 100 aligned on noise; now 1 of 100, the review probe's
seeds, against 6 without the missing reading). The cost: with 2 % of fixes missing their reading,
the 10 m of agreeing travel must fall between two missing readings, so after a mismatch the
alignment can wait long (median 28.9 m, max 198 m in the probe). Agreement implies `r >= (1 - slip) sum d - n sigma`, so the round-1
coherence test against the wheel travel (`r >= 0.5 max(sum |b|, sum d)`) is implied and was
removed. It needs no front-end motion, which would fail in a degenerate corridor.

Before fix round 2 the wheel travel was summed for ever: after T seconds of spinning the
alignment waited for about the spun distance (300 s: 150 m), and the stuck drift stayed in Z
(NEES up to 135 with 0.05 m per fix of drift). Envelope (tests
`GnssTrackYaw2b.StuckWheelsDriftingGnssEnvelope` and `StuckThenDriveAlignsAfterBoundedTravel`;
probe on the review seeds for the window length):

| `align_yaw_wheel_restart_m` | stuck, 0.20 m per fix: aligned of 100 | spin 30 / 60 / 300 s then drive: median (max) distance to the alignment | NEES after the stuck phase |
| --- | --- | --- | --- |
| no window (round 1) | 16 | 15.9 / 30.9 / 149.9 m (300 s: 42 of 100 aligned within 150 m) | up to 77 (0.05 m per fix of drift) |
| 3 m | 83 | 3.9 / 3.9 / 3.9 m | 0.62 to 1.12 |
| 6 m | 16 | 5.9 / 5.9 / 6.9 m | 0.71 to 1.12 |
| **10 m (shipped)** | **4** | **9.9 / 9.9 / 10.9 m (max 17.9 m)** | 0.70 to 1.12 |
| 15 m | 4 | 14.9 / 14.9 / 15.9 m | 0.75 to 1.12 |

At 0.05 and 0.10 m per fix of drift no run aligns while stuck at any length. 10 m is the shortest
window whose stuck-wheel rate is at its floor (at 0.20 m per fix, 1 to 7 of 100 depending on the
seed set: 2 in `StuckWheelsDriftingGnssEnvelope`, 4 in the window-length probe above, 6 in the
re-review 3 probe); without the stuck phase the robot aligns after 4.9 m as before. The cost: a slow robot
in strong coloured error (0.12 m/s, 0.05 m per fix) waits instead of aligning in 6 of 200 runs
within 5 min. An odometer that over-reads by 2x or more mismatches at every chunk and never
aligns; the mismatch counter shows it.

**Minimum track with wheel data** (`align_yaw_min_track_wheel_m`, 3 m, its own value). It applies
when every segment used has its direction from wheel data; otherwise `align_yaw_min_track_m`
(5 m) holds. Chosen from the absolute-error envelope at the shipped 6 deg (straight track,
0.3 m white plus a random walk, 200 seeds; study section 16.3, probe seeds):

| minimum | 0.02 m per fix: max error, runs > 15 deg (0.12 / 0.20 / 0.50 m/s) | 0.05 m per fix: runs > 15 deg / > 30 deg (sum of 3 speeds) | stuck wheels, 0.20 m per fix | time to align at 0.5 m/s |
| --- | --- | --- | --- | --- |
| 0 m | 17.1 / 13.7 / 11.1 deg, 2 | 52 / 3 | 37 of 100 | 7.8 s |
| 1 m, 2 m | the same as 0 m | 52 / 3 | 37 of 100 | 7.8 s |
| 3 m | 14.9 / 8.9 / 9.3 deg, 0 | 30 / 1 | 16 of 100 | 9.8 s |
| 5 m | 8.3 / 9.0 / 8.5 deg, 0 | 15 / 0 | 7 of 100 | 13.8 s |

3 m halves the tails of 0 m (no run beyond 15 deg at 0.02 m per fix with these seeds, the stuck
case at the design-2 no-wheel level) for 2 s at 0.5 m/s; 5 m costs another 4 s. A parked robot
whose wheels stop is caught by the odometer at any minimum (every segment stationary: 0 of 100 at
0.05, 0.10 and 0.20 m per fix). Coloured error is in none of the gates' models: the minimum
track, the coherence test and the consistency window only bound its tails, as the table shows.
(The stuck-wheel column is from before the consistency window of fix round 2, which brings it to
1 to 7 of 100 depending on the seed set.)

**Front-end displacement gated on its covariance: not used** (it is used gated on the
scan-matching information along the travel axis since the root-cause study, section 5.2).
Measured on the corridor
(study section 16.4): with degeneracy detection off (the shipped default) the scan matcher's
along-axis sigma is 1.5 to 4.6 cm while the front end's 2 s displacement is wrong by 1.0 m (it
does not move along the axis); with detection on and the shipped `unobserved_var_m2` (0.001) it
says 3.2 cm while the error reaches 5.5 m. Only the study's 1000 m^2 setting flags the axis. A
test on that covariance would trust exactly the displacement that flipped seeds 51 and 52, so
the direction of travel without wheel data stays the forward assumption (limit of section 5).

**Yaw from rotating in place: not built.** The antenna circles the body origin at the lever arm
`|l|` (0.22 m horizontal), so a 2-D fit of that circle gives `sigma_yaw = sigma / (|l| sqrt(N))`
(perpendicular noise over `sum |q - mean q|^2 = N |l|^2`). One rotation in 10 s at 0.5 m: at the
simulator's 5 Hz (N = 50) 19.5 deg (18.1 predicted), at 10 Hz (N = 100) 12.9 deg (12.8); with RTK
at 2 cm, 0.50 deg at 10 Hz; with 0.4 m of error correlated over 5 s (plus 0.3 m white), 46 deg
(test `GnssTrackYaw2b.InPlaceRotationYawObservability`, 2000 runs each). The brief's
`sigma / (|l| sqrt(N / 2))` overstates the white case by sqrt(2).

### 5.2 Residual limits: causes and tolerances (root-cause study)

Study: `gnss_fusion_study.md` section 16.13. For each residual limit of design 2b the noise
source was swept, and then the suspect step was replaced by an ideal one with the noise
unchanged. A limit is algorithm-caused when the failures disappear and the pipeline has the
information; noise-caused when the sensor gives misleading data; a sensor-set limit when no
sensor in the set measures what is missing. Three changes follow from it (all behind
`gnss.align_yaw_from_track`; struct defaults keep the previous behaviour).

**Bridged odometer gaps** (`gnss.align_yaw_wheel_max_gap_fixes`, 4). The pipeline's reading is a
running integral of wheel speed, interpolated across a wheel-message gap, so a bridged step is an
estimate; section 5.3 bounds it. With `m` the number of consecutive fixes without a reading before a reading, the
travel `odo_i - odo_last` is taken when `m <= max_gap` and lands on fix `i` (its half, and the
weights of fix `i` in `d_k`). Placing a step one to four fixes late moves the weighted travel by
at most `max_gap / n` of that step (0.04 m at 0.5 m/s), well inside the tolerance of the window
(`3 sigma + slip d`). The wheel path is not bridged, because the arc of a step across a gap has
an unknown shape. When `m` exceeds the bound, every chunk half the run touched has no wheel data.
A run that started in the chunk before always overflows in the first half of this chunk, which
the segment using the earlier half also uses (the bound is below half a chunk).

**Extended stationary segments** (`gnss.align_yaw_merge_max_chunks`, 10). A segment with less than
`align_yaw_min_wheel_travel_m` of travel (or a still front end) is not added. Instead its earlier
chunk `A` stays the anchor, and the next segment runs from `A` to the next chunk. Its travel and
the wheel, path and front-end flags are the sums over the chunks between (`carry`); its
wheel-predicted displacement `d_k` is the length of its path. Its noise follows from the telescoping of section 5, because the inner chunks have
coefficient 0.

The direction of an extended segment is the path of `o` between the chunk means:
`unit(p_o + l_k)` from the front end where it observed the travel axis, else `unit(g_k)` from the
wheel path. Without either path the segment is skipped. The mean heading of the two end chunks
is not used, because on a long span a turn at one end biases it: on noise-free slow S-jogs the
first yaw was 4.8 deg off with it, and exact with the path. The path follows the turns, so the
turn gate does not apply.

The slip and profile terms of the inner chunks are left out. Their wheel travel is below
`align_yaw_min_wheel_travel_m` in total, so their slip error is below `slip x 0.2 m` (1 cm),
against the 0.16 m noise of a chunk mean.

A stationary segment is extended only while such a path will exist: the front end observed the
travel axis, or every chunk so far is in the wheel path. Otherwise it is added with `a_k = 0` as
before. On E-01 a reading was missing at the first fix, so the rest chunk had no wheel path, and
extending there lost a segment of track.

After `max_chunks` the stationary segment is added with `a_k = 0` as before. So a robot slower
than `min_travel / (2 s (max_chunks + 1))`, about 0.01 m/s, still does not align.

**Front-end travel where the LiDAR observes the travel axis** (`gnss.align_yaw_frontend_travel`).
- **The observability gate** (since backlog G3, section 5.3: the marginal information along the
  axis against `degeneracy.eig_threshold`). It was `d^T H_rho d`, with `d` the horizontal unit of
  `x_o` in the LiDAR frame and `H_rho` the translation block of the scan-matching Hessian in the
  `[rho, phi]` tangent of `T_w_l`: the robust-weighted count of point-to-plane constraints along
  the axis, against `gnss.align_yaw_frontend_min_info` (100, removed). A scan below the
  threshold, or a scan whose match failed, increments a count that the pipeline passes with each
  fix.
- **Observed segment.** A segment is observed when the count did not change over every step from
  the first fix of its earlier chunk to the last fix of its later chunk.
- **What the front end then decides:**
  - the displacement `p_o` between the two chunk means of the front-end position of `o`; below
    `align_yaw_min_wheel_travel_m` the segment is stationary, whatever the wheels say;
  - without wheel data, the sign of `p_o . a_k` replaces the forward assumption;
  - in a turn, `a_k = unit(p_o + l_k)`, the antenna displacement from the front-end path, with no
    slip or profile term.
- **Measured per scan** (the raw count; section 5.3 has the marginal information on recorded
  Gazebo bags):

  | scene | constraints along `x_o` |
  | --- | --- |
  | box world | 239 to 466 |
  | E-01 | 304 to 577 |
  | box-world corridor | 0.2 to 66 |

  The scan matcher's covariance did not separate these cases (section 5.1); its information
  does.

**Classification of the limits:**
1. **Wheels turning on a still body, with GNSS drift.** Algorithm where the LiDAR observes the
   travel axis (fixed: none of 100 up to 0.50 m per fix). Where it does not, reclassified in
   section 5.3 (backlog G11): observable at the spin-up with IMU + wheels, a sensor-set limit only
   in steady state beyond the IMU's latch horizon. There the tolerance is 0.15 m per fix; above
   it the rate depends on the seeds, 0 to 14 of 100.
2. **Missing wheel readings after a mismatch.** Algorithm: the window break threw away travel
   that the next reading still gives (in the pipeline, interpolated across the gap, bounded in
   section 5.3). Fixed: up to 20 % missing, the robot aligns after a
   median 10.9 m (max 21.9 m). Before, at 2 % missing, it was 36 m (max 311 m).
3. **A slow robot with wheel data.** Algorithm, with two causes: the per-segment travel threshold,
   and the coherence test failing on short segments that noise dominates. Fixed down to about
   0.01 m/s. What remains is noise: the coloured GNSS drift per metre of track. With the drift
   term of section 5.3 it is in the sigma up to the declared rate; above that rate a slow robot
   can still align with a wrong yaw.
4. **Without wheel data.**
   - Backward travel: a sensor-set limit, because the travel sign is not in the GNSS positions or
     the heading. GNSS Doppler does not give it either. Fixed where the LiDAR observes the axis.
   - Parked with drift: as 1 (observable from the rest of the initialisation with the IMU, section
     5.3), with a tolerance of 0.10 m per fix where the axis is not observed.
5. **A skid held through a one-way turn.** Noise (physics) for wheels plus gyro: the sideslip is
   the angle between the velocity and `x_o`, and neither sensor measures it. Fixed where the
   LiDAR observes the axis, through the front-end path. Elsewhere the tolerance is 5 deg.

**Tolerance envelope** (simulator defaults: 0.5 m, 5 Hz, 0.22 m lever arm, wheel data, shipped
keys). Each value is the largest level with at most 1 of 100 failures and, where the robot drives,
alignment within 10 m. The two rows marked * use their own criterion, stated in the row.

| source | LiDAR observes the travel axis | it does not |
| --- | --- | --- |
| GNSS drift, wheels turning on a still body | none up to 0.50 m per fix | 0.15 m per fix |
| GNSS drift, parked without wheel data | none up to 0.30 m per fix | 0.10 m per fix |
| GNSS drift while driving at speed `v` | about `0.02 m sqrt(v / 0.5 m/s)` per fix | the same |
| white GNSS sigma (declared = true) | 1.0 m | 1.0 m |
| missing wheel readings* (restart distance after a mismatch, which includes the 10 m window) | 20 %: median 10.9 m, max 21.9 m | the same |
| speed with wheel data* (drift up to the declared rate, section 5.3) | from 0.02 m/s (bound about 0.01 m/s) | the same |
| GNSS coloured drift rate while aligning (`gnss.align_yaw_drift_var_m2_per_s`, per receiver) | up to the declared rate: at most 1 of 100 beyond 3 sigma on three seed sets; above it (7x) up to 38 of 100 | the same |
| persistent skid in a one-way turn | none up to 10 deg | 5 deg |
| backward travel without wheel data | handled | not handled |

These values are hardware requirements for the GNSS receiver, the odometer and the drive train,
not bugs. The open issues F1 to F6 of that study are closed in section 5.3.

**GNSS Doppler velocity** (study only; a future ADR). At 0.05 m/s per axis, a still body is seen
from the velocity in every environment, and the yaw reaches a 6 deg sigma within 0.4 to 2.1 m at
0.02 to 0.5 m/s, independent of position drift.
- It would remove limits 1 and 4b everywhere, and the drift-per-metre limit of slow robots.
- It would not remove limit 4a (the same sign ambiguity) or limit 5 (a skid rotates the velocity
  too).

### 5.3 Hardening of design 2b

Numbers: `gnss_fusion_study.md` section 18. Everything here is behind `gnss.align_yaw_from_track`
(off); the struct defaults keep the previous behaviour and `config/default.yaml` sets the values
below.

**Observability gate of the front end (G3).** A scan observes the travel axis when the marginal
information of its matching along the axis reaches `degeneracy.eig_threshold` (100), the
degeneracy module's own threshold. With `d_l` the horizontal travel axis in the LiDAR frame and
`p_l_o` the origin of the odometry frame `o` in it (`T_b_l^-1 T_b_o`), under the right
perturbation `T_w_l Exp([rho; phi])` the point `o` moves by `rho + phi x p_l_o`, so along the axis
by `d_l . rho + phi . (p_l_o x d_l)`:

    u = [d_l; p_l_o x d_l]

(H4 review m-2: the first version used `[d_l; 0]`, the LiDAR's own origin; the front-end path the
aligner uses is that of `o`). Then

    I(u) = 1 / (u^T H^-1 u) = 1 / sum_k (v_k . u)^2 / lambda_k

over the eigen-decomposition of the scan-matching Hessian (`degeneracy::marginal_information`,
`degeneracy.md` section 8). It is the information along `u` once the rotation and the other
translations are marginalised: the Schur complement of `H` onto `u`. Eigenvalues at or below
1e-9 of the largest are the null space (a rank-deficient `H` has round-off of either sign there):
a component of `u` along it gives 0, unless it is below 1e-9 of `|u|^2` (round-off of a null
space orthogonal to `u`, which is skipped). A non-finite `H` gives 0. The gate needs
`0 < I(u) >= threshold`, finite, and `load_config` requires `degeneracy.eig_threshold > 0` while
the gate is on (H4 review I-2: a threshold of 0 passed every scan, a corridor included). The gate
shares the threshold with the degeneracy module: retuning `degeneracy.eig_threshold` moves both.
A config with the removed `gnss.align_yaw_frontend_min_info` fails with a message naming its
replacement (H4 review I-3).
- **One definition with the degeneracy module.** Along an eigenvector `I(u)` is its eigenvalue,
  so the gate and `degeneracy::analyze` agree on whether that direction is degenerate. A travel
  axis at an angle to a degenerate corridor axis `v_k` gets at most `lambda_k / (v_k . u)^2`,
  while the raw count `u^T H u` it replaces also counts the lateral constraints. It is basis-free
  and does not depend on `degeneracy.enabled`.
- **Rotation coupling.** The raw count is an upper bound: in the synthetic box world the marginal
  information is 189 to 437 per scan against a raw 239 to 466.
- **Measured per scan** (marginal / raw; scans below 100; degeneracy flags with
  `eig_threshold` 100, any direction / one along the travel axis):

  | scene | marginal | raw | below 100 | flags any / along |
  | --- | --- | --- | --- | --- |
  | box world (synthetic) | 189 to 437 | 239 to 466 | 0 of 179 | 35 / 0 |
  | box-world corridor (synthetic) | 2.9 to 3.6 | 3.0 to 3.7 | 179 of 179 | 179 / 179 |
  | E-01, Gazebo recording (warehouse) | 203 to 415 (p1 236) | 288 to 482 | 0 of 627 | 10 / 0 |
  | E-04, Gazebo recording (corridor) | 10 to 66 (p1 13) | 11 to 70 | 708 of 708 | 708 / 708 |
  | E-07, Gazebo recording (warehouse, 3 laps) | 212 to 413 (p1 239) | 262 to 480 | 0 of 1710 | 8 / 0 |

  The gate and the degeneracy flags along the axis agree on every scan; the margins are 1.5x
  (E-04's largest, 66) below and 2.0x (E-01's smallest, 203) above the threshold.
- **Cost.** One 6x6 eigen-solve per scan, only with the flag on.
- **Limit: the lateral axis of a turn.** The gate measures the travel axis of each scan. A turning
  segment whose direction comes from the front-end path (`a_k = unit(p_o + l_k)`) also uses the
  displacement across that axis, which the gate does not check. In a corridor that is the
  observed direction, and in a turn the axis sweeps through the lateral one within the segment,
  so a scan that sees neither is not passed; a scene that observes the instantaneous travel axis
  but not the lateral one through a whole turn is not covered (H4 review m-3, backlog G21).
- **Front-end/wheel disagreement.** A segment that the observed front end holds still (`o`
  moved less than `align_yaw_min_wheel_travel_m`) while every step had wheel data and the wheels
  moved by more than that much beyond it is counted (`n_frontend_wheel_disagreements()`,
  `Stats::n_gnss_frontend_wheel_disagree`, bench.json, one warning per run). The front end still
  decides. It is right for wheels turning on a still body, the case it was built for; it is wrong
  for a LiDAR that sees surroundings moving with the robot (a lift, a pushed cart), which then
  never aligns. Before, that case was silent (review F4: 0 of 100 aligned, nothing counted); now
  it is counted in 100 of 100 runs. An exact front end, also at 0.05 m/s where its displacements
  sit at the 0.2 m threshold, is never counted (the margin is the threshold itself).

**Interpolated wheel travel (G9).** The pipeline's odometer (`pipeline::detail::WheelOdometer`)
integrates wheel speed with a trapezoid between finite samples; there is no absolute odometer.
Its nominal sample period `P` is the mean of the regular intervals so far, measured, so the rule
holds at any wheel rate. An interval `T` longer than `gnss.align_yaw_wheel_gap_periods` (3)
nominal periods is a gap, for example a wheel driver silent while the robot stops: one period of
it is an ordinary interval, whose trapezoid error the slip term covers as for every other, and
over the excess `T - P` the travel is interpolated. If the speed stays within the end speeds and
does not reverse inside the gap, the travel over the excess lies in `[0, max(|v0|, |v1|) (T - P)]`,
so the interpolation adds at most

    g = max(|v0|, |v1|) (T - P).

A regular interval, with its jitter, adds nothing: at 10 Hz with +-10 ms, 3 periods leave a
margin of 0.19 s. (H4 review I-1: the first version used an absolute 0.1 s threshold and bounded
the whole interval, so at 10 Hz half the ordinary intervals added `|v| x 0.1 s`; the bound grew to
half the travel, swallowed the consistency window, and a still robot with spinning wheels aligned
on drift in 97 of 100 runs, with 46 flips.) `EstimatorCore` sums `g` (an absolute,
non-decreasing reading), counts the gaps (`Stats::n_wheel_gap_bounded`, bench.json, one log line)
and passes the sum with each fix (`TrackYawReadings::wheel_gap_bound_m`). The growth of the sum
since the last reading bounds the error of that reading's step, so the aligner splits it as the
step enters the two segments around its chunk (`i / n` to the later segment, `(n - i) / n` to the
earlier one, the weights of the wheel travel `d_k`; H4 review m-9: the first version put all of
it on the half chunk of the fix, which left part of the step unbounded), sums it over the
segments of the window that use their wheel travel, and widens the wheel/GNSS consistency test:

    |r - sum d_k| > n sigma + slip sum d_k + sum g_k.

A step with `g > 0` is not used in the wheel path. Bridging stays: at constant speed the
interpolation is good, and across a stop the bound covers it. Review F1's stop-and-go (0.7 s
stops every 4 s, silent driver, phantom 0.35 m per stop): 85 to 100 of 100 aligned (99 before
bridging), median 3.3 m. The stuck-wheel protection does not use gaps: wheels that keep
reporting a speed while the body is still have no gap; at 50, 10 and 5 Hz with +-10 ms jitter it
is the same (`GnssTrackYawHardening.WheelRateKeepsTheProtectionAndTheAisles`: 4 of 100 at 0.20 m
per fix of drift at each rate; aisles align after a median 4.5 m at each rate).

**Mutation coverage (G12).** The four mutants that survived the 2b review are killed:
`ext_path_ok` and the carried `wheel_ok` of an extended segment
(`GnssTrackYawHardening.ExtendedSegmentWithAHoleInItsPathIsSkipped`,
`ExtendedSegmentKeepsItsMissingReading`), the half marking of a gap that crosses a chunk boundary
(`GnssTrackYawRootCause.GapLongerThanTheBoundIsNotBridged`, case 27-31), and the front-end
position of `b` instead of `o` (`GnssTrackYawHardening.FrontEndPositionIsOfTheOdometryFrame`).
The mutants of each change of this section: study section 18.4; the full set: study section 18.9.

**Coloured GNSS drift (G10).** The fix error is taken as white plus a random walk with per-axis
variance rate `q` (`gnss.align_yaw_drift_var_m2_per_s`; a drift of `d` per fix at rate `f` is
`d^2 f`). The YAML value 0.0017 m^2/s is the simulator's Gauss-Markov drift (`2 sigma^2 / tau`,
0.5 m and 300 s), about 0.018 m per fix at 5 Hz. Segment `k`, whose chunk centres are `dt_k`
apart, gets a random-walk increment in `b_k` with per-axis variance `q dt_k`, independent of the
other segments (increments over disjoint intervals). It enters `Z` with `|a_k|^2`:

    var_Z += q sum_k dt_k |a_k|^2

in the yaw sigma and in the noise of the consistency window. On a straight track the sum is `q T`,
with `T` the time between the first and the last chunk centre; the true variance of the
end-to-end displacement of two chunk means `D` long is `q (T - D / 3)`, so the term is a slight
upper bound. Then `sigma_drift = sqrt(q / T) / v` and a slow track waits until its displacement
outgrows the drift. The fix times come with each fix (`TrackYawReadings::t`); without them the
term is 0. An extended segment spans many chunks and gets its full `q dt` (review F2: the
extension had no drift term). Review F2's case, 0.05 m/s at 0.02 m per fix: 10 / 8 / 10 of 100
beyond 3 sigma to 0 / 1 / 1 (three seed sets), every run aligned (median 4.5 m against 3.3 m).
Within the declared rate no cell of the study grid has more than 1 of 100 beyond 3 sigma. White
fixes only: a 0.03 m/s robot still aligns, after 6.0 m instead of 3.1 m. A drift above the
declared rate is outside the model: at 0.05 m per fix (7x the rate) 1 to 38 of 100 runs from
0.05 m/s up are beyond 3 sigma (2b: 4 to 52), and at 0.02 m/s 10 to 11 of the 26 to 35 that now
align (2b: 1 to 4 of 2 to 5; the term widens the window, so more slow robots align, with a sigma
too small for that drift); with the rate set to that drift at most 1 of 100 anywhere.

**Choosing the drift rate** (owner ruling 2026-10-30, backlog G18). `q` is a per-receiver YAML
value, > 0 and finite (checked at start-up); the default 0.0017 m^2/s is the simulator's
receiver. How to choose it for a real receiver: log fixes of the antenna standing still for several drift time constants (tens of minutes), take the per-axis variance of the position change over a lag T well below the time constant, and divide by T (a random walk); or from a Gauss-Markov fit, 2 sigma^2 / tau. Too low: a slow robot under more drift aligns with a confident wrong yaw. Too high: the alignment waits longer (slow robots much longer).

**Gentle turns on the path (G5).** A segment whose turn (the turn gate's measure) exceeds
`gnss.align_yaw_path_min_turn_deg` (10 deg) but not `align_yaw_max_turn_deg` takes its
direction from the path of `o` when it exists: the front end where it observes the travel axis,
else the wheel path, exactly as a turn-gated segment. A still one is stationary (and may be
extended). Without either path the mean heading is used as before. Reason: the chord between
two chunk centroids lies along the mean heading only for a symmetric arc at constant speed; on
an S-jog or a speed change inside the segment it bends by up to half the turn. Review F5
measured 1.4 to 2.3 deg of first-yaw bias on noise-free S-jogs (0.52 deg on main); through the
path it is below 0.01 deg at the same distance, and with 0.5 m noise the mean error goes from
1.3 to 0.1 deg (NEES 1.05 to 0.68).

**Spin-up and rest from the IMU (G11).** Where the LiDAR does not observe the travel axis, limits
1 (wheels turning on a still body) and 4b (parked without wheel data) were classified as
sensor-set limits without an oracle. Run on the recorded Gazebo bags (E-01, E-04, E-07, offline
probe; study section 18.7):
- **Transitions are observable.** Over the first second of motion the IMU velocity change along
  `x_o` (gravity-compensated with the shipped initialisation) is 0.455 / 0.501 / 0.455 m/s
  against 0.463 / 0.503 / 0.462 m/s from the wheels and 0.455 / 0.496 / 0.454 m/s true; at rest
  the same 1 s integral has a 0.0014 to 0.0022 m/s standard deviation. A spin-up of the wheels
  under a still body would be a disagreement of about 200 standard deviations; a stall of a moving
  robot is a deceleration of the same size.
- **Steady state needs a latch, whose horizon is short.** The IMU speed integrated from the rest
  of the initialisation drifts: with the shipped initialisation (the accelerometer's horizontal
  bias, 0.61 m/s^2 on these bags, read as tilt, backlog S2) 0.01 to 0.05 m/s after 10 s, but 8
  m/s after 30 s on E-01 and E-07, because once the body yaws the absorbed bias reappears as up
  to 1.3 m/s^2 of phantom acceleration (0.17 on the straight E-04). With the level start
  (`imu.init_level_start`, ADR-027, off) 0.18 to 0.31 m/s after 30 s and 0.8 to 1.2 m/s after
  60 s; an event-gated integral (only while the 1 s mean exceeds 0.05 m/s^2) stays within 0.03 /
  0.19 / 5.7 m/s over the whole runs.
- **Most of the aligner's failures are early.** In the probe the still body aligns on drift a
  median 10 s after the start at 0.30 m per fix (limit 1; 122 s at 0.50) and 20 to 26 s (limit
  4b), mostly inside a horizon of some tens of seconds; with an ideal stillness oracle none of
  100 align at any drift.
- **Classification.** Limits 1 and 4b where the axis is not observed: observable at the
  transition with IMU + wheels (1) or IMU (4b, from the rest at initialisation); a sensor-set
  limit only in steady state beyond the latch horizon, which the IMU's bias handling sets.
- **Not implemented here.** A detector that fixes them must latch the still state from the
  transition, end the latch when its integrated error exceeds the speeds it tells apart, and
  needs the level start or an equivalent bias estimate (with the shipped initialisation the latch
  breaks at the first yaw: 8 m/s of error in 30 s). That is the shared stationarity detector of
  backlog G7 (also for a zero-velocity factor), a design that needs an ADR, and it depends on the
  owner's decision S5 (level start). The simulator has no drive vibration, the usual rest cue
  of real robots, so its false-still rate cannot be measured here either.
