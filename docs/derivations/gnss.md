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
  Consequence: with 2 s between chunk centres, a robot slower than about 0.1 m/s never aligns
  with wheel data at the defaults (test `GnssTrackYaw.LowSpeedWithWheelNeverAligns`; without
  wheel data the same track aligns).
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
   0.3 m common error over 5 m of track is 3.4 deg.
2. `r >= gnss.align_yaw_min_coherence * sum |b_k|` (0.5), over the segments used. For steady
   motion the ratio is near 1. For a parked receiver whose error wanders, the segment
   directions are random, so `r` grows like `sqrt(K)`, `sum |b_k|` like `K`, and the ratio
   falls like `1 / sqrt(K)`.
3. `sigma_yaw <= gnss.align_yaw_max_sigma_deg` (3 deg).

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
on noise. No cheap stationary signal exists for that case: the IMU rest detector used at
keyframe 0 tests the spread of acceleration and rate, which is the same at rest and at
constant velocity (in the simulator both are pure sensor noise, 0.08 m/s^2 per sample), so it
would also block the straight cruise that the corridor needs. This is a documented limit
(ADR-023).

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
