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

**Front-end displacement where the LiDAR is not degenerate: not used.** Measured on the corridor
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
