# scan_match: point-to-plane scan-to-map registration (S-01..S-05, D-03)

Implemented in `core/scan_match/src/` (contract:
`core/scan_match/include/mss/scan_match/scan_matcher.hpp`). Tested by
`tests/unit/test_scan_match.cpp` and `tests/component/test_scan_match_component.cpp`.
Design background: ADR-004 (LiDAR factor covariance from the inverse Hessian, inflated along
degenerate directions), ADR-007 (voxel hash map, k-NN plane, RMS rejection). Degeneracy
analysis: `degeneracy.md`.

Conventions: tangent `xi = [rho, phi]` (translation first), right perturbation
`T_w_l <- T_w_l * Exp(xi)` (`lie_conventions.md`). H, g, xi, the covariance and the
degeneracy report all live in the tangent of the LiDAR frame l. No frame conversion here.

## 1. Residual

For a scan point `p_l` (frame l, m) and a map plane through `q_w` with unit normal `n_w`
(frame w):

    r = n_w . (T_w_l * p_l - q_w)        (signed distance, m)

Variable: the pose `T_w_l`, perturbed on the right by `xi = [rho, phi]`.

## 2. Jacobian `J_r_xi`

To first order `T Exp(xi) p = R (p + rho + phi x p) + t = R (p + rho - skew(p) phi) + t`, so

    dr/drho =  n_w^T R            = (R^T n_w)^T
    dr/dphi = -n_w^T R skew(p_l)  = -(R^T n_w)^T skew(p_l)

With `n_l = R^T n_w` and `-n^T skew(p) = (p x n)^T`:

    J_r_xi = [ n_l ; p_l x n_l ]^T       (1x6, stored as Vec6)

Units: the `rho` entries are unitless, the `phi` entries m/rad. A flipped normal flips `r`
and `J_r_xi` together, so `J^T J` and `J^T r` (hence H, g and the update) do not depend on
the normal's sign. Test S-01 compares `J_r_xi` with central differences of
`r(boxplus(T, h e_k))`, `h = 1e-6`, relative error < 1e-6.

## 3. Plane fit (internal `detail::fit_plane`)

From the k nearest map neighbours `x_i` (`k = map.knn_k`, at most `kMaxNeighbors`):

    q = (1/n) sum x_i
    S = sum (x_i - q)(x_i - q)^T     (centred first, then accumulated)
    n_w = eigenvector of lambda_min(S)  (unit)
    rms = sqrt(max(lambda_min, 0) / n)

`lambda_min / n` is the mean squared distance of the neighbours to the best plane, so `rms` is
the RMS point-to-plane distance. A correspondence is kept iff `n >= 3`, the nearest neighbour
is within `max_corr_dist_m` (`dist2[0] <= max_corr_dist_m^2`), `rms <= plane_rms_max_m` and
the two plane-fit gates below pass.

A collinear neighbourhood has two zero eigenvalues: `rms = 0` and the normal is any unit vector
orthogonal to the line. Without the collinearity gate it is accepted; the matcher guarantees
that such data never produces NaN (it either fails with an error or returns finite values,
test `CollinearMapNeverNaN`).

### Plane-fit gates (ADR-007 amendment 2026-10-27)

With the scatter eigenvalues `lambda_min <= lambda_mid <= lambda_max` (already computed for the
normal), both ratios dimensionless in `[0, 1)`, `0` = off:

    planarity     (rho = plane_min_eig_ratio):  reject iff  lambda_min >  rho * lambda_mid
    collinearity  (mu  = min_mid_eig_ratio):    reject iff  lambda_mid <= mu * lambda_max

A plane patch has `lambda_min << lambda_mid ~ lambda_max`. Two faces meeting at an edge, or a
corner, give `lambda_min` of the order of `lambda_mid` (planarity rejects them). Points on one
line, or a long thin strip such as two parallel point columns, give `lambda_mid << lambda_max`
and an ill-defined in-plane direction (collinearity rejects them). Both tests are written
without a division; identical points (all eigenvalues 0) fail the collinearity test.
Unit tests: `PlaneFit.PlanarityGateRejectsEdgeAcceptsPlane` (two faces at 90 deg with 5 points:
`rms = 0.038 m`, ratio 0.46; a plane with 3 mm noise: 3e-4) and
`PlaneFit.CollinearityGateRejectsLineAcceptsPatch` (a line: `lambda_mid = 0`; two rings 1 cm
apart over 0.5 m: 8e-4; a square patch: > 0.5).

Evaluation (map spacing off; S-02 room at N = 20, 10 trials, `S02_GateSweep`; corridor axis
share and inliers from `CorridorAxisLeak`; box from `PipelineComponent.BoxWorld*`; E-01 fused
from `tools/run_integration.py`, E-01 LiDAR-only = the same replay with GNSS and wheel off; base
9ec6919):

| gates | S-02 worst error / min inliers | corridor axis share / inliers | box ATE LiDAR / GNSS+wheel | E-01 fused / LiDAR-only |
| --- | --- | --- | --- | --- |
| off | 9.50 mm / 0.083° / 720 | 1.15 % / 1838 | 8.8 / 32.9 mm | 0.07086 m / 3.0 mm |
| planarity 0.01 | 7.19 mm / 0.092° / 707 | 0.10 % / 1771 | - | - |
| planarity 0.05 | 9.76 mm / 0.085° / 720 | 0.037 % / 1785 | 9.1 / 33.3 mm | 0.07094 m / 1.9 mm |
| planarity 0.1 | 9.72 mm / 0.083° / 720 | 0.47 % / 1811 | - | 0.07110 m / - |
| planarity 0.2 | 9.46 mm / 0.080° / 720 | - | - | 0.07089 m / - |
| collinearity 0.05 | 9.50 mm / 0.083° / 720 | 0.88 % / 1400 | 6.5 / 31.8 mm | 0.07102 m / 2.8 mm |
| collinearity 0.1 | 4.96 mm / 0.053° / 705 | 0.67 % / 1111 | 6.2 / 32.0 mm | 0.07130 m / - |
| collinearity 0.2 | 0.28 mm / 0.017° / 688 | 0.59 % / 740 | 5.9 / 32.3 mm | 0.07136 m / 2.4 mm |
| planarity 0.05 + collinearity 0.2 (9ec6919 choice) | 0.28 mm / 0.017° / 688 | 0.040 % / 720 | 6.1 / 32.1 mm | 0.07110 m / 1.8 mm |
| default + map spacing 0.1 m | 0.27 mm / 0.016° / 714 | 0.040 % / 720 | 6.3 / 32.1 mm | 0.0735 m / - |

Findings:

- S-02 edge bias: the biasing neighbourhoods are two parallel point columns on two faces,
  exactly coplanar (`lambda_min = 0`), so planarity cannot see them; they are long and thin,
  and collinearity at 0.2 removes them (9.5 -> 0.28 mm) without any map change.
- Corridor axis leak: the spurious x information comes from neighbourhoods across the
  wall-floor and wall-ceiling edges, not from single-ring lines. Planarity at 0.05 cuts it
  30x (1.15 % -> 0.037 %); collinearity alone only halves it while dropping 60 % of the
  inliers. (PR 4b attributed it to collinear single-ring neighbourhoods; this sweep does not
  support that.)
- Together (0.05 / 0.2) on 9ec6919: box ATE improves for LiDAR-only (8.8 -> 6.1 mm) and
  GNSS+wheel (32.9 -> 32.1 mm), E-01 LiDAR-only improves 3.0 -> 1.8 mm, E-01 fused moves
  0.07086 -> 0.07110 m (+0.33 %, 96 % of that ATE is GNSS noise, `docs/derivations/map.md`
  "Đánh giá").

On base 9ec6919 this table selected planarity 0.05 + collinearity 0.2; the re-measurement after
#19 below replaced that choice.

#### Re-measured after the rebase on 0d1df78 (#19: T_e_w graph variable, `cov_scale` 4.7)

E-01..E-03 now use the scenario overlay of `tools/run_integration.py` (about 2 mm ATE). Box
from `PipelineComponent.BoxWorld*`, NEES from `LidarFactorCovarianceConsistent` (bound [3, 12]),
corridor tests from #19 (`CorridorDriftCorrectedByGnss`: GNSS ATE <= 0.5 x LiDAR-only and
> 200 GNSS factors; `CorridorDefaultDegeneracyGnssDoesNotDegrade`; `CorridorOutageRealignsAfterRejects`):

| gates (spacing off) | E-01 / E-03 ATE | box ATE LiDAR / GNSS+wheel | GNSS-only ratio (0.1 / 0.5 / 2 m) | NEES GTSAM / no-GTSAM | corridor drift: LiDAR-only -> GNSS ATE | corridor default degeneracy, LiDAR-only ATE |
| --- | --- | --- | --- | --- | --- | --- |
| off (main, shipped) | 2.20 / 2.11 mm | 9.0 / 5.4 mm | 1.00 / 1.00 / 1.00 | 7.78 / 3.83 | 15.4 -> 1.8 m (pass) | 11.5 m |
| planarity 0.05 | 1.37 / 1.29 mm | 9.1 / 5.1 mm | 1.00 / 1.00 / 1.01 | 9.85 / 4.09 | 44.1 -> 12.8 m, 144 factors (**fails** > 200) | 5.4 m |
| collinearity 0.2 | 2.59 / 2.57 mm | 5.7 / 5.0 mm | 1.00 / 1.00 / 1.00 | 6.39 / 3.30 | 36.4 -> 4.5 m (pass) | 216.6 m |
| 0.05 + 0.2 | 1.65 / 1.59 mm | 6.0 / 4.7 mm | 1.00 / 1.00 / 1.01 | 8.48 / **2.83 (fails >= 3)** | 42.8 -> 36.9 m (**fails** 0.5x) | 241.5 m |
| planarity 0.01 | 0.95 / 0.97 mm | 7.2 / 4.1 mm | 1.00 / 1.00 / 1.02 | 6.48 / 3.51 | 96.9 -> 5.2 m (passes 0.5x; fails the absolute guard added in fix round 1) | 13.0 m |

S-02 and the corridor axis leak are scan-matching tests and did not change with #19 (planarity
0.01: S-02 7.19 mm / 0.092 deg, axis share 0.10 %). In the #19 corridor (5 m wide, 120 m
long) the collinearity gate removes most wall neighbourhoods (they are thin strips at that
range) and the LiDAR-only drift grows 20x; the corridor outcomes are not monotone in either
ratio. At 0.05 + 0.2 two post-#19 bounds fail (marked); they were not loosened.

#### Defaults: both gates off (evaluated tunables)

Three candidates were compared on base 0d1df78 with the noise-free synthetic data:

| candidate | result |
| --- | --- |
| planarity 0.05 + collinearity 0.2 | best S-02 (0.28 mm) and axis share (0.04 %); fails `CorridorDriftCorrectedByGnss` and the no-GTSAM NEES bound, narrow-corridor LiDAR-only drift x20 |
| planarity 0.05 | E-01 1.37 mm; fails `CorridorDriftCorrectedByGnss` (144 GNSS factors, bound > 200), S-02 9.8 mm |
| planarity 0.01 | passes every bound, E-01 2.20 -> 0.95 mm, box LiDAR 9.0 -> 7.2 mm, GNSS+wheel 5.4 -> 4.1 mm, axis share 0.10 %, S-02 7.2 mm |

None is shipped: `plane_min_eig_ratio` and `min_mid_eig_ratio` default to **0** and the
shipped behaviour is bit-identical to main (E-01 `est.tum` md5 equal on the scenario data and
on the ov_plain replay). Reasons:

- **Noise.** For a planar patch `lambda_min / n ~ sigma_perp^2` and `lambda_mid / n ~ s^2` (`s`
  the in-plane spread), so the planarity ratio compares range noise with patch size, and map
  points carry the full range noise (the preprocessor keeps the first point per voxel,
  unaveraged). A Monte Carlo of 5 points in a disk of radius 0.19 m (the measured mean
  5th-neighbour distance) rejects 14 / 49 / 81 / 90 % of planar patches at 0.01 for
  sigma 0.5 / 1 / 2 / 3 cm (2 / 11 / 40 / 60 % at 0.05). Every dataset above is noise-free in
  range (E-01 bridge without `range_sigma_apply`, exact ray cast), so the E-01 gain at 0.01 is
  a noise-free result.
- **Non-ideal replay.** The ov_plain replay (old E-01 data with the default extrinsics, which
  do not match the data, so the map is not ideal) is the one imperfect-data run:

  | `plane_min_eig_ratio` | ATE | scan p95 | inlier fraction per scan (mean / p5) | GN iterations (mean / scans at 10) |
  | --- | --- | --- | --- | --- |
  | 0 (main, shipped) | 14.8 mm | 6.1 ms | 0.992 / 0.916 | 2.75 / 30 |
  | 0.01 | 24.0 mm | 11.5 ms | 0.727 / 0.539 | 3.24 / 47 |
  | 0.05 | 14.5 mm | 5.5 ms | 0.923 / 0.879 | 2.43 / 14 |

  Mechanism, as far as measured (temporary per-scan instrumentation, 578 scans): at 0.01 the
  gate rejects 27 % of the correspondences on average and almost half on the worst 5 % of
  scans (mean inliers 832 against 1137), the remaining residuals are smaller (mean |r| 3.2
  against 4.5 mm) and Gauss-Newton needs more iterations (47 scans hit the 10-iteration cap
  instead of 30). The latency and the ATE regression come from those starved scans; 0.05
  rejects 7 % and does not regress on this replay.
- **Conditioning is not accuracy.** The axis-leak reduction (1.15 % -> 0.10 % at 0.01) removes
  edge neighbourhoods whose spurious x information happened to anchor the unobservable
  corridor axis; in the pipeline corridor the fused GNSS ATE gets worse (1.8 -> 5.2 m at 0.01,
  LiDAR-only 15.4 -> 96.9 m). `CorridorDriftCorrectedByGnss` now also has an absolute guard
  (fused ATE <= 1.2 x the 1.820 m of main).

To make the planarity gate usable on real data it would need a noise-aware form, for example
`lambda_min > rho * lambda_mid + n * (k * sigma_range)^2`; that is a design change for a later
ADR. The collinearity gate at 0.2 removes the S-02 edge bias but rejects the thin wall strips
of a 5 m wide corridor (LiDAR-only drift 11.5 -> 241 m with planarity 0.05).
`S02_StrongerSettingsRemoveEdgeBias` keeps the demonstration that collinearity 0.2 or map
spacing 0.1 m removes the bias. S-02 at the shipped config: 9.50 mm / 0.083 deg (bounds 1 cm /
0.1 deg), identical under GCC 13 and Clang 18.

## 4. Huber IRLS Gauss-Newton

Per correspondence, with `delta = robust_huber_delta_m`:

    w = 1             if |r| <= delta
    w = delta / |r|   otherwise

    H = sum w J^T J      g = sum w J^T r      H xi = -g

`w` multiplies both `J^T J` and `J^T r` (iteratively reweighted least squares for the Huber
loss). H and g are accumulated in scan order on one thread, so results are bit-reproducible.

## 5. Degeneracy-aware solve (when `degeneracy.enabled`)

Let `H = V diag(lambda) V^T` from `degeneracy::analyze`, `tau = eig_threshold`, `F` the set of
flagged directions (`lambda_k < tau`) and `P = V diag(m) V^T` the projector of
`degeneracy::project_update` (`m_k = 0` for `k in F`, else 1). The matcher factorises

    H_reg = H + sum_{k in F} (tau - lambda_k) v_k v_k^T  =  V diag(lambda') V^T,
    lambda'_k = tau for k in F, lambda_k otherwise,

solves `H_reg xi = -g` and then applies `xi <- P xi`.

Claim: for invertible H, `P H_reg^-1 (-g) = P H^-1 (-g)`. Both inverses share V, so
`P H_reg^-1 = V diag(m_k / lambda'_k) V^T` and `P H^-1 = V diag(m_k / lambda_k) V^T`. For
`k in F`, `m_k = 0` in both; for `k not in F`, `lambda'_k = lambda_k`. Hence equal. So the
order "regularise, solve, project" gives exactly the projected Gauss-Newton step of the
header when H is invertible, and it stays defined when H is singular along a flagged
direction (an exactly degenerate corridor, `lambda = 0` along its axis), where `LLT(H)` would
fail. With no flags `H_reg == H` bit for bit, so enabling the flag in a well-constrained scene
changes nothing. `LLT(H_reg)` can only fail when an UNflagged `lambda_k <= 0`, which cannot
happen when `tau > 0` (every `lambda_k < tau` is flagged) except through non-finite H; that
remains the "singular Hessian" error. `MatchResult::H` stays the raw, unregularised H, which
the pipeline may analyse again to obtain the DegeneracyReport (section 6).

## 6. Covariance

The Gauss-Newton covariance of the estimate is `sigma_r^2 H^-1`, where `sigma_r^2` is the
variance of the residuals (H is built with unit weights per residual, so `H^-1` alone assumes
`sigma_r = 1 m`). The matcher estimates `sigma_r^2` from the final iteration's inliers as the
Huber-weighted mean squared residual

    sigma_r^2 = sum(w_i r_i^2) / sum(w_i)       (m^2)

computed during the last accumulation pass (no extra pass), at the same linearisation point
as the reported H. Then

    s2  = max(sigma_r^2, min_residual_sigma_m^2)
    C   = cov_scale * s2 * A^-1,   A = H (disabled) or H_reg (enabled),
    C  <- 0.5 (C + C^T)
    cov = C                                        (degeneracy disabled)
    cov = relax_covariance(C, report, deg_cfg)     (enabled; report of the final iteration)

with `cov_scale` a dimensionless multiplier (`scan_match.cov_scale`, default 1.0) and
`min_residual_sigma_m` a floor on the residual sigma (`scan_match.min_residual_sigma_m`,
default 0.005 m, validated > 0). The variance scales with the noise level: for scan noise
`sigma`, `sigma_r^2 ~ sigma^2` and H barely changes, so 3x the noise gives about 9x the
position variance (test `CovarianceScalesWithNoise`). ADR-004 notes that Hessian covariances
are over-confident (correlated residuals, model error); `cov_scale` is the knob for that.

Floor: a noise-free synthetic scan (Gazebo with noise disabled) fits to float rounding, so
`sigma_r^2` is near zero and `C` would be near zero or exactly zero, which a backend noise
model cannot use (singular, or an ill-conditioned information of ~1e14). The floor makes
`C >= cov_scale * min_residual_sigma_m^2 * A^-1` (test `NoiseFreeCovarianceAtFloor`).

Degenerate directions: `C` alone would be wrong along a flagged direction `v_k`. There
`H_reg` has eigenvalue `tau` instead of `lambda_k < tau`, so `v_k^T C v_k =
cov_scale * s2 / tau`, SMALLER than the unregularised `cov_scale * s2 / lambda_k` (infinite for
an exact corridor) and not a valid uncertainty. The matcher therefore applies the degeneracy
module's `relax_covariance` itself, so `MatchResult::cov` is final (the pipeline must not relax
it again): along `v_k` the variance becomes `relax_factor * cov_scale * s2 / tau`, at least
`relax_factor` times the regularised value and, with the defaults, larger than every unflagged
variance `cov_scale * s2 / lambda_j` (`lambda_j >= tau`) since `relax_factor > 1` (test
`D03_CorridorProjectedNotSingular`). This is still a finite number, not "no information".
The relaxed covariance is the only degeneracy mitigation in the factor; the pipeline passes
the DegeneracyReport to backend for diagnostics only (see graph.hpp) and must not call
relax_covariance again. With
nothing flagged `relax_covariance` is the exact identity, so enabling the flag in a
well-constrained scene changes nothing.

Tuning note: `eig_threshold` is absolute, so it depends on the point count. A sparse but
well-constrained scan (the 720-point synthetic room) has a smallest eigenvalue of 77 in the
first iteration and 141 at convergence, so `tau = 100` flags a healthy direction there.

Note: `sum(w)` rather than `n - 6` in the denominator: with hundreds of inliers the
difference is negligible, and the weighted mean is consistent with the IRLS weights.

## 7. Loop and errors

    if scan.size() > max_points           -> "scan larger than reserved max_points"
    if T_init has a non-finite entry      -> "non-finite initial pose"
    copy the finite scan points to double once (non-finite points are skipped)
    for it = 1 .. max_iterations:
      build H, g, sum w r^2, sum w at T     (section 4)
      if inliers < min_inliers            -> "too few inliers"
      A = H or H_reg (section 5); LLT(A) fails or xi non-finite -> "singular Hessian"
      xi = -A^-1 g, projected if enabled; T <- T * Exp(xi)
      stop when |xi_rho| < trans_tol_m and |xi_phi| < rot_tol_rad   (converged)
    cov from the last factorisation, floored and relaxed (section 6); non-finite -> "singular Hessian"

The reported H, inliers and covariance belong to the last iteration, i.e. linearised at the
estimate before the last update; `T_w_l` is the estimate after it. Not converging within
`max_iterations` is not an error (`converged = false`).
