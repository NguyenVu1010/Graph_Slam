# Backend: GTSAM factors, charts and marginals

Implemented in `core/backend/` (`mss::backend::Graph`, internals in `mss::backend::detail`,
`core/backend/src/`). Tested by `tests/unit/backend/test_backend_*.cpp` (B-01 to B-04 of
`docs/Test_CI_CD.md`) and `tests/unit/test_backend_api.cpp`. Conventions:
`docs/conventions.md` and `docs/derivations/lie_conventions.md` ("Library orders"). The IMU
factor replaces the `CombinedImuFactor` of ADR-004 (ADR-015).

## 1. Charts

`graph.cpp` refuses to compile unless GTSAM defines `GTSAM_POSE3_EXPMAP`, so

    Pose3::retract(xi_g) = T * Expmap(xi_g),   xi_g = [dphi, drho]
    R <- R Exp(dphi),   p <- p + R drho + O(|xi|^2)

This is the mss right perturbation `T <- T * Exp(xi_m)` with `xi_m = [drho, dphi] = P xi_g`,
`P = [0 I; I 0]` (`P = P^T = P^-1`). `Vector3` (velocity, m/s) is additive.
`imuBias::ConstantBias` is additive with tangent `[dba, dbg]` (accelerometer first).

Permutation rules at the boundary (code: `core/backend/src/conversions.hpp`):

    vectors       xi_g = P xi_m
    covariances   Sigma_g = P Sigma_m P^T
    Jacobian cols J_g = J_m P          (a residual differentiated w.r.t. xi_g = P xi_m)

Rows need no permutation when a factor whitens its own residual: the residual order of a
custom factor is internal to it, and only `e^T Sigma^-1 e` reaches the optimiser. The
relative-pose factors (section 5, all three forms) return their residual in GTSAM order
`[rotation, translation]` (for `between` and `se3_exact` the `BetweenFactor<Pose3>` error),
so their noise is `P Sigma_m P^T` (checked by
`BackendConversions.CovariancePermutationMatchesGtsamWhitening`).

## 2. Priors on keyframe 0 (and the IMU-gap re-anchor, §9)

Centred on the keyframe guess `x0`:

- `PosePriorFactor(X(0), x0.T_w_b)` (§2b), error `Logmap(x0^-1 T)` in `[phi, rho]`, sigmas
  `[prior_rot_sigma_rad x3, prior_pos_sigma_m x3]`. In mss order this is the header's
  `diag(prior_pos^2 x3, prior_rot^2 x3)` on the `[rho, phi]` tangent of `T_w_b`.
- `PriorFactor<Vector3>(V(0), x0.v_w_b)`, sigma `prior_vel_sigma_mps x3`.
- `PriorFactor<ConstantBias>(B(0), x0.bias)`, sigmas
  `[prior_bias_acc_sigma_mps2 x3, prior_bias_gyro_sigma_radps x3]`.

## 2b. Pose prior (`detail::PosePriorFactor`)

Every pose prior in `Graph` uses it: keyframe 0 (§2), the re-anchor prior of an orphaned
component (§9) and the prior of the GNSS frame variable `E` (§6b). Variable `X` (Pose3,
right perturbation `[dphi, drho]`), prior `X0` (a fixed pose of the same frames):

    e   = P r_m,   r_m = Log(X0^-1 X)            ([rho, phi] -> [phi, rho], rad / m)
    H   = P Jr^-1(r_m) P                          (6x6, d e / d xi_g)

`r_m` and `Jr^-1(r_m)` come from `odom::relative_pose_residual(X0, X, Identity)` (its
`J_r_xi_j`); the permutation is that of §1, as in `Se3RelativePoseFactor` (§5).

Why not `gtsam::PriorFactor<Pose3>`: it returns the same error `-Local(X, X0) =
Logmap(X0^-1 X)` (with `GTSAM_POSE3_EXPMAP`), so the cost and the `[phi, rho]` sigmas are
unchanged, but its Jacobian is the identity, which is `Jr^-1(r_m)` only at `r_m = 0`. The
error of the identity grows with the residual: about `1/2 |phi|` in the rotation block plus a
`1/2 [rho]_x` coupling of rotation into translation. At 30 deg / 10 m and 90 deg / 5 m it
is more than 30 % (relative Frobenius, control test below). Where it matters:

- keyframe 0 sits about 1.8 sigma from its prior in B-04; there the identity Jacobian put
  0.087 of error into the cost gradient, so the Gauss-Newton fixed point that iSAM2
  converges to was not the cost minimum (true gradient 0.09);
- the prior of `E` has a loose yaw (`gnss_frame_yaw_sigma_rad`) and position sigma, so `E`
  moves far from it: in the GNSS fusion study, up to 38 deg of yaw and 1.9 m (box world,
  GNSS 2 m) and 5.2 m (corridor);
- a re-anchor prior is centred on a guess, not on an estimate.

The SE(3) Log residual of `PriorFactor<Pose3>` is kept, not the decoupled chart of §5: the
prior sigmas are set for it and are a regularisation, not a measured covariance in another
chart, and keeping the residual makes this a pure linearisation fix (same cost at every
point, no YAML flag).

## 3. IMU factor (`detail::ImuPreintFactor`)

Variables `X_i = (R_i, p_i)`, `V_i = v_i`, `X_j`, `V_j`, `B_i = (ba_i, bg_i)`. Constants: the
delta of `core/imu` (`dR, dv, dp, dt_s = T`, `J_*`, `bias_lin`, `cov`) and
`g_w = [0, 0, -gravity_mps2]`. First-order bias correction (`imu_preintegration.md` §5) with
`db = B_i - bias_lin`, `theta = J_dR_bg dbg`:

    dR' = dR Exp(theta)
    dv' = dv + J_dv_ba dba + J_dv_bg dbg
    dp' = dp + J_dp_ba dba + J_dp_bg dbg

With

    A = R_i^T (p_j - p_i - v_i T - 0.5 g_w T^2)
    C = R_i^T (v_j - v_i - g_w T)

the 9-vector residual, order `[phi, p, v]` (the order of `delta.cov`), is

    r_phi = Log(dR'^T R_i^T R_j)
    r_p   = dR'^T (A - dp')
    r_v   = dR'^T (C - dv')

It is zero exactly when `X_j, V_j = Preintegrator::predict(X_i, V_i, B_i)`, and equals GTSAM
`NavState::localCoordinates` of `state_j` with respect to the prediction (rotation, then
position and velocity rotated into the predicted body frame), i.e. the delta chart of
`imu_preintegration.md` §1. Noise: `noiseModel::Gaussian::Covariance(delta.cov)`, no
permutation.

Jacobians (GTSAM charts, pose columns `[dphi, drho]`, bias columns `[dba, dbg]`),
`Jri = Jr^-1(r_phi)`. They follow from `Exp(phi)^T u ~ u + skew(u) phi`,
`Exp(-u) E = E Exp(-E^T u)` and `Exp(theta + d) ~ Exp(theta) Exp(Jr(theta) d)`:

    X_i:  d r_phi = [ -Jri R_j^T R_i ,  0      ]
          d r_p   = [ dR'^T skew(A)  , -dR'^T  ]
          d r_v   = [ dR'^T skew(C)  ,  0      ]
    V_i:  [ 0 ;  -dR'^T R_i^T T ;  -dR'^T R_i^T ]
    X_j:  [ Jri , 0 ;  0 , dR'^T R_i^T R_j ;  0 , 0 ]
    V_j:  [ 0 ;  0 ;  dR'^T R_i^T ]
    B_i:  d r_phi = [ 0              , -Jri Exp(r_phi)^T Jr(theta) J_dR_bg            ]
          d r_p   = [ -dR'^T J_dp_ba , skew(r_p) Jr(theta) J_dR_bg - dR'^T J_dp_bg ]
          d r_v   = [ -dR'^T J_dv_ba , skew(r_v) Jr(theta) J_dR_bg - dR'^T J_dv_bg ]

Two representative lines. `X_i` rotation: `R_i^T <- Exp(-dphi) R_i^T`, so
`A <- A + skew(A) dphi`, and `dR'^T R_i^T R_j <- dR'^T R_i^T R_j Exp(-(R_i^T R_j)^T dphi)`,
whose Log moves by `-Jri R_j^T R_i dphi`. `B_i` gyro bias: `dR'^T <- Exp(-u) dR'^T` with
`u = Jr(theta) J_dR_bg dbg`; then `r_p <- r_p - skew(u) r_p = r_p + skew(r_p) u`, and
`r_phi` moves by `-Jri Exp(r_phi)^T u`.

## 4. Bias random walk

`BetweenFactor<ConstantBias>(B_i, B_j, zero)`, sigmas
`[acc_random_walk sqrt(T) x3, gyro_random_walk sqrt(T) x3]` (m/s^2, rad/s). The constructor
rejects a random walk `<= 0` with `std::invalid_argument`: a zero sigma is a hard constraint
that the iSAM2 Cholesky factorisation cannot handle.

## 5. LiDAR and loop factors (`backend.relative_factor`, default `decoupled`, ADR-021)

Variables `X_i = T_w_bi`, `X_j = T_w_bj`; measurement `Z = T_bi_bj` (scan matching or loop
closure) with `cov`, the 6x6 `[rho, phi]` covariance of the right perturbation of `Z` in
`b_j`. For LiDAR it is the scan matcher's covariance (degeneracy relaxation already applied),
converted from `l` to `b` by the pipeline with the adjoint of `T_b_l`.

**Chart of the covariance.** The scan matcher builds `cov = cov_scale * max(sigma_r^2,
min_residual_sigma_m^2) * A^-1` (`scan_matcher.hpp`, `MatchResult::cov`), with `A` the
Gauss-Newton Hessian `sum w J_r_xi^T J_r_xi` of the point-to-plane cost (or its regularised
form when degeneracy handling is on, then relaxed along flagged directions) at the solution,
in the `[rho, phi]` tangent of the right perturbation `T_w_l <- T_w_l Exp(xi)`. It is a
first-order object at the solution: at `xi = 0` the SE(3) chart and the SO(3) x R3 chart
coincide (`V(0) = I`), but along a free translation the point-to-plane cost is flat in
`dt` at every rotation, which is the SO(3) x R3 chart (see "Unobserved axis" below). The
default form `decoupled` reads `cov` in that chart (ADR-021).

Three forms are selectable: `decoupled` is the default (ADR-021); `between` (the behaviour
before ADR-021) and `se3_exact` remain selectable for comparison. All three return a
GTSAM-ordered residual `[rotation, translation]` and use the same noise,
`noiseModel::Gaussian::Covariance(P cov P^T)` (`permute_cov`), so the whitened cost is
`r^T cov^-1 r` with `r` in mss order.

**`between`: `gtsam::BetweenFactor<Pose3>`.** Error `Logmap(Z^-1 T_i^-1 T_j)` in `[phi, rho]`
= `P r_m` with the SE(3) residual `r_m` below. Its Jacobian is that of `Between()` only: it
omits the factor `Jr^-1(e)` of the `Logmap` (the derivative of `Local(Z, .)`), unless GTSAM is
built with `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`, which the pinned GTSAM 4.2 is not. Exact at
zero error, wrong by a relative `O(|e|)` elsewhere (`Jr^-1(e) = I + ad(e) / 2 + ...`, and `ad`
contains `skew(rho)`, so metres of translation residual count fully): at 5 m / 30 deg the
relative Jacobian error exceeds 0.1 (`B01_BetweenFactorJacobianApproximateAtLargeResidual`).

**`se3_exact`: `detail::Se3RelativePoseFactor`.** The same residual with exact Jacobians, from
`odom::relative_pose_residual` (O-01, `wheel_odom.md` sections 4 and 5), right perturbation
`T <- T Exp(xi_m)`, `[rho, phi]`:

    r_m = Log(Z^-1 T_i^-1 T_j)
    J_r_xi_j = Jr^-1(r_m)
    J_r_xi_i = -Jr^-1(r_m) Ad(T_j^-1 T_i)

`J_r_xi_j` follows from `T_j <- T_j Exp(xi)`: `Log(E Exp(xi)) = r_m + Jr^-1(r_m) xi + O(|xi|^2)`,
`E = Z^-1 T_i^-1 T_j`. `J_r_xi_i` from `T_i^-1 <- Exp(-xi) T_i^-1` and
`Exp(-xi) T_i^-1 T_j = T_i^-1 T_j Exp(-Ad(T_j^-1 T_i) xi)`. GTSAM order: `e = P r_m` and, with
the Pose3 chart `xi_g = [dphi, drho] = P xi_m` (section 1), `H_x_i = P J_r_xi_i P`,
`H_x_j = P J_r_xi_j P`. Equal to GTSAM's exact `between` x `between` x `Logmap` chain to 1e-9
(`B01_Se3RelativePoseFactorMatchesGtsamExactChain`).

**`decoupled`: `detail::DecoupledRelativePoseFactor`.** Translation and rotation of `E` as
separate residuals (the SO(3) x R3 chart of `E` instead of the SE(3) `Log`), with
`t_rel = R_i^T (p_j - p_i)`:

    r_phi = Log(R_Z^T R_i^T R_j)                 (rad)
    r_p   = R_Z^T (t_rel - p_Z)                  (m, in the frame of Z's end, b_j)

With `E = Z^-1 T_i^-1 T_j = (R_E, t_E)` and `phi_E = Log(R_E)`, the forms differ only in how
they combine the two parts: the SE(3) residual is `[V(phi_E)^-1 t_E, phi_E]`, the decoupled
residual `[t_E, phi_E]`. The rotation parts are equal, and the translation parts are related
exactly by

    r_p = t_E = V(phi_E) rho_SE3,   rho_SE3 = V(phi_E)^-1 t_E = t_E - 0.5 phi_E x t_E + O(|phi_E|^2 |t_E|)

so the two agree to first order in the residual `E` (`V(0) = I`). Jacobians, columns `[dphi, drho]` of `T_w_bi` and of `T_w_bj`,
`Jri = Jr^-1(r_phi)`:

    X_i:  d r_phi = [ -Jri R_j^T R_i  ,  0      ]
          d r_p   = [ R_Z^T skew(t_rel), -R_Z^T  ]
    X_j:  d r_phi = [ Jri ,  0                   ]
          d r_p   = [ 0   ,  R_Z^T R_i^T R_j     ]

From `R_i^T <- Exp(-dphi) R_i^T` (so `t_rel <- t_rel + skew(t_rel) dphi` and the rotation
residual moves by `-Jri R_j^T R_i dphi`), `p_i <- p_i + R_i drho` (`t_rel <- t_rel - drho`),
`R_j <- R_j Exp(dphi)` (`Jri`) and `p_j <- p_j + R_j drho` (`t_rel <- t_rel + R_i^T R_j drho`).

Both exact forms pass `EXPECT_CORRECT_FACTOR_JACOBIANS` and the numerical check (< 1e-6) at small
residuals and at 30 deg / 5 m (`B01_Se3RelativePoseFactorJacobians`,
`B01_DecoupledRelativePoseFactorJacobians`). Cost per evaluation: `se3_exact` one SE(3)
`Jr^-1` (6x6) and one adjoint; `decoupled` one SO(3) `Jr^-1` (3x3); once per factor and
linearisation (E-01 bench below).

**Same covariance, different extrapolation.** All three forms consume the scan matcher's
`cov` identically. `cov` is `cov_scale * sigma^2 * H^-1`, with `H` the Gauss-Newton Hessian of
the point-to-plane cost in the right-perturbation tangent of the scan pose at `xi = 0`
(`scan_matcher.hpp`): a first-order object at the estimate. At `E = Exp(delta)` the SE(3)
residual is exactly `delta` and the decoupled residual is `[V(dphi) drho, dphi]`, equal to
`delta` to first order, so the quadratic form `r^T cov^-1 r` gives `delta` the same weight in
all three forms. The `l -> b` conversion agrees too: in the SO(3) x R3 chart a perturbation of
`T_w_l` moves the body translation by `R_b_l dt + skew(p_b_l) R_b_l dphi`, the `rho` row of
`Ad(T_b_l)`. The forms differ only away from the estimate, at large residuals, through the
exact `V(phi_E)^-1` coupling above. Choosing a form chooses how the same Gaussian is extended
there, not what `cov` means.

**Unobserved axis.** A LiDAR factor with no information along a translation direction `d`
(degeneracy relaxation, PR 4b's "unobserved" covariance `cov + s^2 d d^T` with `s` = 1 km)
carries residuals of tens to hundreds of metres along `d` at a negligible cost. In the
SO(3) x R3 chart the scan pose is `(R Exp(dphi), p + R dt)`, so the point-to-plane residual is
linear in `dt` at every `dphi`: the free direction is a fixed `dt` direction at any rotation,
and the decoupled Gaussian is flat along exactly that direction for every residual size, as
the point-to-plane cost is. In the SE(3) chart the same motion `t_E = s d` with rotation
`phi_E` has coordinates `rho = V(phi_E)^-1 s d = s d - 0.5 s phi_E x d + ...`: the off-axis
part `0.5 s |phi_E|` is charged against the observed (centimetre) sigmas although the true
cost is flat. At `s` of hundreds of metres and mrad rotations that is metres of spurious
information. So for a translational degeneracy `decoupled` keeps the free direction free and
`se3_exact` bends it; `decoupled` is the more faithful use of the same covariance. (For a
rotational degeneracy neither chart is exact.) With `between`, in addition, the omitted
`ad(e) / 2` gives a wrong Gauss-Newton step with a gain of the residual's size, and iSAM2 takes
one step per keyframe, so the wrong step is not iterated away. The acceptance control in §5a
is the empirical confirmation: `between` diverges, `se3_exact` does not diverge but is
over-confident, `decoupled` stays within its sigma.

Loop: the same factor of the configured form with `noiseModel::Robust::Create(
mEstimator::Cauchy::Create(k), Gaussian::Covariance(P cov P^T))`. GTSAM reweights on the
whitened norm `|e|` with weight `w = 1 / (1 + (|e| / k)^2)`.
`k = kLoopCauchyK = sqrt(chi2_6(0.95)) = sqrt(12.5916) = 3.5485`: a loop inside the 95 % gate of
its own covariance keeps a weight above 0.5. A `backend.loop_cauchy_k` YAML key comes with the
loop module. Degeneracy report: diagnostics only (one debug log line per degenerate keyframe,
with the running count); the factor is unchanged.

### 5a. Evaluation (PR 4e; decided in ADR-021)

Synthetic consistency (`BackendComponent/UnobservedAxis.*`, 50 seeds x 24 keyframes on a
circle, a noisy IMU chain, LiDAR factors with `(1 km)^2` along world x, one optimize() per
keyframe; each form under noise drawn in its own chart, so the test checks the factor and
the optimiser, not the chart): world-x NEES of each new keyframe,
expectation 1: `se3_exact` 0.85, `decoupled` 0.92; `between` under the SE(3) noise: the graph
becomes indeterminant at seed 1000, keyframe 17 (control).

Acceptance control (`PipelineComponent.RelativeFactorSeeds`, seed 42, deg on, x removed, no
backend feedback, gravity factor off; `imu_only_observability.md` "Exploratory controls"):
backend last-keyframe x error (marginal sigma).

| form | circle | straight |
| --- | --- | --- |
| between | +5772.3 m (14.05): 411 sigma | +11.37 m (20.90): 0.54 sigma |
| se3_exact | -28.23 m (1.75): 16 sigma | +7.47 m (24.28): 0.31 sigma |
| decoupled | -8.71 m (24.46): 0.36 sigma | +4.36 m (24.48): 0.18 sigma |

`se3_exact` removes the divergence but is over-confident on the circle (its sigma drops from 14
to 1.75 m); `decoupled` stays within its sigma in both scenes.

Shipped configuration measured at db84bf9 (before the exact pose prior, §2b), seeds 42..56
paired against `between` (median (max) | wins / losses |
95 % bootstrap CI of the median difference, form - between; lower is better; box NEES
expectation 6). `scan_match.cov_scale` 4.7 was calibrated with `between`; the box NEES of the
exact forms stays within 7.8 +/- 1 at 4.7, so no recalibrated rows were needed.

| metric | between | se3_exact | decoupled |
| --- | --- | --- | --- |
| 60 s corridor + GNSS, ATE (m) | 1.85 (18.54) | 2.58 (26.41) \| 4/11 \| [-0.06, +1.02] | 3.37 (23.56) \| 6/9 \| [-0.09, +2.03] |
| 60 s corridor, LiDAR + IMU, ATE (m) | 38.34 (205.39) | 41.14 (152.24) \| 8/7 \| [-5.88, +20.07] | 45.06 (149.52) \| 7/8 \| [-12.07, +11.67] |
| box LiDAR factor NEES | 8.57 (12.47) | 8.58 (12.47) \| 5/10 \| [-0.00, +0.01] | 8.47 (12.49) \| 5/10 \| [-0.00, +0.02] |
| 20 s circle x free, \|end x err\| (m) | 1.45 (7.02) | 2.60 (13.63) \| 6/9 \| [-0.65, +1.71] | 2.30 (7.37) \| 6/9 \| [-0.21, +1.01] |
| 20 s circle x free, along-x RMS (m) | 1.36 (2.84) | 1.33 (5.81) \| 8/7 \| [-0.91, +1.13] | 1.68 (3.25) \| 6/9 \| [-0.84, +1.35] |
| 20 s straight x free, \|end x err\| (m) | 2.72 (6.28) | 3.26 (5.90) \| 6/9 \| [-0.39, +0.40] | 3.38 (8.18) \| 5/10 \| [-0.26, +0.82] |

Every CI contains 0 at db84bf9. After the exact pose prior (2026-10-28, PR #28) the 60 s
corridor + GNSS row reads: between 1.87 (15.32); se3_exact 2.66 (28.01) | 4/11 |
[+0.06, +0.59], the only CI that excludes 0; decoupled 3.47 (29.13) | 4/11 | [-0.08, +0.56].
These IMU-only and corridor scenes are chaotic: the same decoupled form
evaluated with a different floating-point order of the same products (an earlier env-gated
build) gave a circle median of 3.30 m instead of 2.30 m. `CorridorImuOnlyAlongAxisWithin
DeadReckoning` (seeds 42..44, run at `between` in PR 4e) circle end errors: between -1.29 / +0.38 /
-1.45 m, se3_exact -13.63 / -2.60 / -7.33 m (its median 7.33 m would fail the 3 sigma_DR 2.11 m
limit), decoupled +2.30 / -0.78 / -0.08 m.

E-01..E-03 (scenario overlay): ATE 2.181 / 2.181 / 2.175 mm (between, `est.tum` md5 equal to
main), 2.189 / 2.189 / 2.176 mm (se3_exact), 2.185 / 2.185 / 2.179 mm (decoupled). E-01 bench,
median of 5 (scan p50 / p95, replay wall, CPU): main 4.83 / 6.57 ms, 4.54 s, 3.05 s; between
4.84 / 6.35 ms, 4.46 s, 3.03 s; se3_exact 4.74 / 6.50 ms, 4.23 s, 3.01 s; decoupled 4.75 / 6.53
ms, 4.34 s, 3.04 s (optimize() is not timed separately; no measurable cost).

Decision (ADR-021): `decoupled` is the default. It is the only form that meets the acceptance
control; on the shipped scenes no form is statistically different. With `decoupled` as the
default, `CorridorImuOnlyAlongAxisWithinDeadReckoning` passes with unchanged limits:
circle median end error 0.780 m (limit 2.113 m), median RMS 0.742 m (3.0 m); straight 0.815 m
(1.86 m) and 1.430 m (1.88 m). E-01..E-03 2.185 / 2.185 / 2.179 mm, `est.tum` of E-01 / E-03
md5-equal to the PR 4e decoupled run; E-01 bench (median of 5, interleaved) main / this branch:
scan p50 4.40 / 4.39 ms, p95 5.77 / 5.92 ms, wall 3.67 / 3.60 s, CPU 2.73 / 2.73 s.

## 6. GNSS position factor (`detail::GnssPositionFactor`)

Variable `X_k = (R_w_b, p_w_b)`; constants `T_e_w = (R_e_w, p_e_w)` and the lever arm
`p_b_g`. With the header sign (opposite of `gnss::detail::gnss_residual`):

    r = R_e_w (R_w_b p_b_g + p_w_b) + p_e_w - p_e_g          (3x1, m)
    d r / [dphi, drho] = [ -R_e_w R_w_b skew(p_b_g) ,  R_e_w R_w_b ]

Noise `noiseModel::Gaussian::Covariance(m.cov)`.

## 6b. GNSS factor with the frame variable (`detail::GnssFrameFactor`, ADR-019)

Used when `BackendConfig::gnss_frame_variable` is true. Variables `X_k = (R_w_b, p_w_b)` and
`E = T_e_w = (R_e_w, p_e_w)`, one `E` (key `E(0)`) shared by every GNSS factor; constant
lever arm `p_b_g`. With `q = R_w_b p_b_g + p_w_b` (antenna in w):

    r = R_e_w q + p_e_w - p_e_g                                (3x1, m)

Both variables use the GTSAM right perturbation `[dphi, drho]`. For `X_k` the Jacobian is
that of section 6. For `E`, `R_e_w Exp(dphi) q ~= R_e_w q - R_e_w skew(q) dphi` and
`p_e_w <- p_e_w + R_e_w drho`, so

    d r / [dphi, drho]_E = [ -R_e_w skew(q) ,  R_e_w ]

`E` gets one prior (`PosePriorFactor`, §2b) when it is created (first accepted
`add_gnss_factor`, kept with the keyframe-0 priors), centred on the aligner's `T_e_w` of that call, GTSAM order
`[phi, rho]`, diagonal sigmas

    [tilt, tilt, yaw, pos, pos, pos]  =  [gnss_frame_tilt_sigma_rad (x2),
                                          gnss_frame_yaw_sigma_rad, gnss_frame_pos_sigma_m (x3)]

The right-perturbation `phi` of `E` is expressed in w, and w and e share their z axis
(`R_e_w` is a yaw rotation), so `phi_z` is yaw and `phi_x`, `phi_y` are tilt: the tight tilt
sigmas keep `E` gravity-aligned (4-DoF, as in `gnss.md` section 4). The loose
yaw and position sigmas only regularise `E` before the fixes span enough to determine it.
Why the frame is a variable: `gnss_fusion_study.md`. Tests: `B01_GnssFrameFactorJacobians`
(numerical differentiation and `EXPECT_CORRECT_FACTOR_JACOBIANS`, random `E` including
tilt), `GnssFrameVariableAbsorbsAlignmentError` (known answer).

## 6c. Static-window gravity-direction factor (`detail::StaticGravityFactor`, PR 4d, ADR-020)

Variables `X_0 = T_w_b` (keyframe 0) and `B_0`. Measurement `f_b`: the mean accelerometer
reading of the static initialisation window (`imu::InitState::acc_mean_b`, m/s^2, in b);
`C` = the covariance of that mean (`acc_mean_cov`: per axis
`max(s_k^2, acc_noise_density^2 rate_hz) / n`, `s_k^2` the sample variance over the `n`
window samples), scaled by `backend.static_gravity_sigma_scale^2` in `Graph`. At rest the
specific force is `f = ba - R_w_b^T g_w`. The full residual

    e = f_b - (ba - R_w_b^T g_w)                                  (3x1, m/s^2)

is projected on the plane normal to `f_b`, so only the direction is kept (the magnitude,
`|g|` against `ba_z`, is dropped; `imu_only_observability.md` section 10b):

    r = P e   (2x1),   P = [e1^T; e2^T],   (e1, e2, u = f_b / |f_b|) orthonormal

with `e1 = normalize(u x a)`, `a` the coordinate axis least aligned with `u`, and
`e2 = u x e1` (`StaticGravityFactor::projection`). Right perturbation `R <- R Exp(dphi)`:
`R^T g -> Exp(-dphi) R^T g = R^T g + skew(R^T g) dphi`, hence, in GTSAM column order
`[dphi, drho]` for the pose and `[dba, dbg]` for `ConstantBias` (additive):

    J_r_x = P [skew(R_w_b^T g_w), 0_3],      J_r_b = P [-I_3, 0_3]

Noise `Gaussian::Covariance(P C P^T)` (`P` constant: built from the measurement). The
factor constrains the combination `g dtheta_h - dba_h` (tilt and horizontal accelerometer
bias); it does not split tilt from bias, which a static window cannot. It goes with the
keyframe-0 priors (kept when an `optimize()` fails). The pipeline adds it when
`backend.static_gravity_factor` is true, keyframe 0 is the scan at which the
initialisation passed (otherwise the window no longer describes keyframe 0) and the window
passes the rest gate of `imu_only_observability.md` section 12 (`imu.init_max_mean_gyro_radps`,
`imu.init_max_wheel_speed_mps`, `imu.init_gravity_check_sigma`; a failure is logged and
counted in `Stats::n_gravity_factor_gated`). A zero or non-finite `f_b`, a covariance that is
not SPD, or `static_gravity_sigma_scale <= 0` is rejected.

The window is used twice: the keyframe-0 rotation prior is centred on the initializer's
`R_w_b`, computed from the same `acc_mean`. The prior's tilt information (1 / (17.5
mrad)^2) is about 0.2 % of the factor's at the shipped noise (1 / (0.84 mrad)^2), so the
double count is negligible.

Consistency check (ADR-020; evaluated and shipped off, `backend.static_gravity_check_keyframes:
0`): with a value > 0, after that keyframe is optimised, `Graph::static_gravity_chi2()`
returns the factor's **innovation statistic** against the rest of the graph. In whitened units (`W` the factor's square-root
information), with `e = W r` at the estimate, `J = W dr/d[X_k, B_k]`, `P` the joint marginal
covariance of `(X_k, B_k)` with the factor and `P_rest` without it:

    S = Cov(e) = I - J P J^T,      S^-1 = I + J P_rest J^T  (Woodbury: the factor touches
                                                             only X_k, B_k)
    statistic = e^T S^-1 e = nu^T (I + J P_rest J^T)^-1 nu   (to first order)

with `nu` the residual at the estimate without the factor. It is chi-square with 2 DoF (3
for the study-only full residual) when the window was at rest and the rest of the graph is
consistent with its own covariance; then the gate's 5 % is the false-removal rate. Measured
at rest the rate is higher where the rest of the graph is not consistent at the check
keyframe (`imu_only_observability.md` 10g: 6 of 15 GNSS-corridor runs, none in the box world). The fix-round-1 statistic `|e|^2` (the posterior residual, after the
factor has pulled tilt and bias) has expected value `2 - leverage` at rest, below 2: its
false-removal rate is below 5 % and its power against a window in motion lower (the
`StaticGravityInnovationKnownAnswer` case: posterior 2.3, innovation 20). `S` is formed as
`(I + J P_rest J^T)^-1`, not by the subtraction, which cancels to round-off when the
factor dominates (a test measured 1e-3 for a true 1e-14). `P_rest` is one
`gtsam::Marginals` of the iSAM2 factors without the gravity factor, at the estimate: once
per run, off the hot path.

The keyframe-0 rotation prior is centred on the initializer's `R_w_b`, from the same window
mean, so it is not independent of the factor: it pulls `nu` toward 0. Its tilt information is
0.2 % of the factor's (above), so the statistic is biased low by about that much.

If `S`'s smallest eigenvalue is at most `kMinGravityInnovationEigen` = 1e-9
(`constants.hpp`), or the rest of the graph does not determine `(X_k, B_k)`, the factor is
the only information on what it measures and nothing can contradict it: the statistic is
not evaluated, the factor stays, and the pipeline counts `Stats::n_gravity_factor_unchecked`.
Measured with the check on (`imu_only_observability.md` 10g): 0.013 to 0.66.

Above `backend.static_gravity_chi2_gate` (5.991, 0.95)
`Graph::remove_static_gravity_factor()` removes it from iSAM2 (`ISAM2::update` with
`removeFactorIndices`, the slot recorded from `ISAM2Result::newFactorsIndices` when it was
committed or rebuilt) and from the factors kept for a rebuild; a factor still pending is
dropped from the pending set. If iSAM2 throws or the new estimate is not finite, iSAM2 is
rebuilt from the committed factors at the last good estimate (section 9) and the factor
stays: `remove_static_gravity_factor()` returns false, the pipeline logs a warning and
does not retry (the check runs once), and only a successful removal counts in
`Stats::n_gravity_factor_removed`. The observer (`CoreObserver::on_gravity_check`) gets the
statistic and the outcome (kept, removed, removal failed, unchecked).

Study seams (`study_seams.hpp`, `test_imu_only_study.cpp` only; production never calls
them): `detail::set_static_gravity_full_residual` restores the round-1 3x1 residual (its
check uses the 3-DoF gate 7.815 in the study), and
`detail::set_static_gravity_posterior_statistic` restores the fix-round-1 posterior
statistic.

Tests: `BackendFactors.B01_StaticGravityFactorJacobians` (`EXPECT_CORRECT_FACTOR_JACOBIANS`
and relative error < 1e-6 against central differences; residual equals `P e`, `P P^T = I`,
`P f_b = 0`), `B01_StaticGravityFactorFullResidualJacobians` (the same for the study-only
3x1 residual), `B01_StaticGravityFactorZeroAtRest`, `B01_StaticGravityFactorIgnoresMagnitude`,
`BackendGraph.StaticGravityFactorKnownAnswer` (keyframe 0 guessed 10 mrad off: the optimised
tilt and its marginal against the closed form), `StaticGravityInnovationKnownAnswer`
(innovation statistic against its closed form, posterior below the gate and innovation
above), `StaticGravityInnovationUncheckedWhenFactorIsOnlyInformation`,
`StaticGravityFactorRemovedOnRequest` (also a failed optimize after the removal, and a
removal before the first optimize),
`OptimizeFailureDropsPendingFactorsKeepsKeyframes` (kept after a failed optimize),
`RejectsBadCovariances` (including `f_b = 0`), `BackendStub.*` (stub errors),
`ImuInitializer.WindowMeanAndCovarianceOfTheMean`.

## 7. Wheel-odometry factor (`detail::OdomBetweenFactor`)

Variables `X_i, X_j`. `r_m, J_m_i, J_m_j` come from
`odom::relative_pose_residual(T_i, T_j, odom.T_b0_b1)` (mss order, right perturbation).
`odom.cov` is singular by design. Eigen split (`SelfAdjointEigenSolver`, ascending)
`cov = U diag(lambda) U^T`; keep the `n` columns with `lambda > kOdomRankRelTol * lambda_max`,
`kOdomRankRelTol = 1e-9` (a numerical rank tolerance, not a tunable). Whitening

    W = diag(lambda_kept)^-1/2 U_kept^T                    (n x 6)
    residual  W r_m,   Jacobians  W J_m_i P,  W J_m_j P,   noise Unit(n)

so `|W r_m|^2 = r_m^T cov^+ r_m` and the unobserved directions carry no information.
`n = 0` (all-zero covariance, e.g. `t0 == t1`) or a non-finite input is rejected.

## 8. Marginals

`ISAM2::marginalCovariance(X(k))` is `[phi, rho]` at the current estimate, right
perturbation; `Graph::marginal_cov` returns `P Sigma P^T`.

## 9. Failure handling and IMU-gap re-anchor

Failure. `optimize()` inserts only the variables referenced by pending factors (an
unconstrained keyframe is never inserted; its state is reported from the guess with a NaN
covariance). When `ISAM2::update` throws (e.g. indeterminant system) or the estimate is not
finite, the iSAM2 instance may be half-updated, so it is rebuilt from the committed factors
at the last good estimate; pending factors are dropped, pending keyframe-0 priors and the
keyframe guesses are kept, and `optimize()` returns `"optimization failed: ..."`. If the
rebuild itself throws, iSAM2 restarts empty: the last estimate becomes the keyframe guesses
and every committed factor is queued again for the next `optimize()` (logged as an error).

Pose re-anchor after a failure. Dropping the pending factors also drops the edges `(k-1, k)`
that tied the keyframes added since the last success to the anchored graph, and the pipeline
never adds them again. Every keyframe whose pose is not in iSAM2 after a failure is marked
orphaned. In each `optimize()`, the keys of the pending factors are split into connected
components (by shared factors); a component that holds no key already in iSAM2 and not
`X(0)`, and contains an orphaned keyframe, gets a `PosePriorFactor` (§2b) on its lowest pose
key (the oldest keyframe of the island), centred on that keyframe's guess, with the
first-keyframe sigmas (§2), and one warning line. A keyframe stops being orphaned once its
pose is in iSAM2. Components without an orphaned keyframe are not anchored, so an
under-constrained new keyframe (e.g. a GNSS fix alone) still fails as before.

Non-finite guess. `add_keyframe` replaces a non-finite `x0` pose, velocity or bias by the
previous keyframe's guess (identity / zero for keyframe 0) with one warning line; the
keyframe is still added, so KeyIds stay dense, and no non-finite value reaches GTSAM.

Re-anchor. If an IMU factor is rejected in the middle of the chain (e.g. `1 -> 2`), the next
accepted factor `2 -> 3` would bring in `V(2)` and `B(2)` with no link to keyframe 0, so the
system would stay indeterminant. When `add_imu_factor(i, j)` is accepted with `i > 0` and no
accepted IMU factor ends at `i`, priors on `V(i)` and `B(i)` with the first-keyframe sigmas
(§2), centred on the guess of keyframe `i`, are added with it.

## 10. Checks

| Formula | Test |
| --- | --- |
| §1 permutation of covariances | `BackendConversions.CovariancePermutationMatchesGtsamWhitening`, `BackendConversions.PermuteCovIsInvolution` |
| §1 pose conversion | `BackendConversions.PoseRoundTrip` |
| §3 IMU Jacobians (numerical, < 1e-6) and full `delta.cov` noise | B-01 `BackendFactors.B01_ImuFactorJacobians` |
| §3 zero residual at the prediction | B-01 `BackendFactors.B01_ImuFactorZeroAtPrediction` |
| §6 GNSS Jacobian and sign | B-01 `BackendFactors.B01_GnssFactorJacobians`, `BackendFactors.B01_GnssFactorZeroWhenConsistent` |
| §7 rank-3 odom whitening and Jacobians | B-01 `BackendFactors.B01_OdomFactorRotatedSingularCov`, `BackendFactors.B01_OdomFactorZeroCovRejected` |
| §5 exact Jacobians of `se3_exact` and `decoupled` (numerical < 1e-6 and `EXPECT_CORRECT_FACTOR_JACOBIANS`, small and 30 deg / 5 m residuals; known residual and whitened error); `se3_exact` = GTSAM's exact `Logmap` chain and the `between` error; `between` control at the large residual | B-01 `BackendFactors.B01_Se3RelativePoseFactorJacobians`, `BackendFactors.B01_DecoupledRelativePoseFactorJacobians`, `BackendFactors.B01_Se3RelativePoseFactorMatchesGtsamExactChain`, `BackendFactors.B01_BetweenFactorJacobianApproximateAtLargeResidual` |
| §2b pose prior: numerical < 1e-6 and `EXPECT_CORRECT_FACTOR_JACOBIANS` at small residuals and at 30 deg yaw / 10 m and 90 deg tilted / 5 m; error equal to `PriorFactor<Pose3>` to 1e-12; zero at the prior; `PriorFactor<Pose3>` control (> 1e-2 relative at the large offsets) | B-01 `BackendFactors.B01_PosePriorFactorJacobians`, `BackendFactors.B01_PriorFactorPose3JacobianApproximateAtLargeResidual` |
| §5 Cauchy wrap of both exact forms (numerical at zero error; `sqrt(w)` scaling at 0.1 and at the large residual) | B-01 `BackendFactors.B01_RelativePoseRobustLoopJacobians` |
| §5 permuted noise and Cauchy wrap of the `between` form (numerical at zero error; `sqrt(w)` scaling at 0.1) | B-01 `BackendFactors.B01_BetweenAndRobustLoopJacobians` |
| §5 estimate consistent with its marginal along an unobserved axis, per form (each under its own noise model); `between` control | `BackendComponent/UnobservedAxis.EstimateConsistentWithMarginal/{se3_exact,decoupled}`, `BackendComponent.UnobservedAxisBetweenFactorControlIsInconsistent` |
| §5 `backend.relative_factor` parsing | `Config.RelativeFactorKey` |
| §2, §8 priors and marginal order | `BackendGraph.MarginalCovIsMssOrder` |
| §5 pose graph | B-02 `BackendGraph.B02_SquareLoopNoiseFreeConvergesExactly`, `BackendGraph.B02_SquareLoopNoisyWithinMarginalBounds` |
| §3, §4 IMU chain and time check | `BackendGraph.ImuChainRecoversTrajectory`, `BackendGraph.ImuTimeCheckRejectsMismatch` |
| §6 in the graph | `BackendGraph.GnssFactorPullsPosition` |
| §9 pose re-anchor, non-finite guess, duplicate IMU factor | `BackendGraph.FailedOptimizeDoesNotStrandLaterKeyframes`, `BackendGraph.AddKeyframeNonFiniteGuessIsSanitised`, `BackendGraph.DuplicateImuFactorRejected` |
| §9 | `BackendGraph.ImuGapReanchorsVelocityAndBias`, `BackendGraph.OptimizeFailureDropsPendingFactorsKeepsKeyframes`, `BackendGraph.LatestKeyframeWithoutFactorsReturnsGuessAndNanCov` |
| input validation (including an unknown `relative_factor`) | `BackendGraph.RejectsBadCovariances`, `BackendApi.ConstructorRejectsInvalidConfig` |
| §5 robust loop | B-03 `BackendIsam.B03_OutlierLoopDragsLessThanTenCm`, `BackendIsam.B03_ControlNonRobustOutlierDragsMoreThanTenCm` |
| iSAM2 vs the batch solution (LM, then Gauss-Newton to the fixed point of the linearisation, gradient < 1e-5), per relative-factor form; for `se3_exact` and `decoupled` that point is the true cost minimum (central-difference gradient < 1e-5 at the reference and at the iSAM2 estimate; §2b); `between` control (true gradient > 0.1) | B-04 `BackendIsam.B04_IsamMatchesBatchReference` |
| §2b, §6b frame variable `E` about 20 deg / 3 m from its prior: iSAM2 = batch minimum < 1e-6, true gradient < 1e-5; control with `PriorFactor<Pose3>` on `E` | `BackendIsam.FrameVariableMatchesBatchAtLargePriorResidual` |
| §2b, §9 re-anchor prior about 20 deg / 2.4 m from its guess: iSAM2 = batch minimum < 1e-6, true gradient < 1e-5; control with `PriorFactor<Pose3>` fails the 1e-6 limit | `BackendIsam.ReanchorPriorMatchesBatchAtLargeResidual` |
| determinism (single-thread TBB in `deterministic` mode: ADR-018), diagnostics | `BackendIsam.RepeatedOptimizeIsStableAndDeterministic`, `BackendDeterminism.SingleThreadLimitIsProcessLifetime`, `BackendDeterminism.SingleThreadRunsAreBitIdentical`, `BackendDeterminism.MultiThreadRunsReported`, `BackendIsam.DegeneracyReportIsDiagnosticOnly` |
