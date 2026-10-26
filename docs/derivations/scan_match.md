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
is within `max_corr_dist_m` (`dist2[0] <= max_corr_dist_m^2`) and `rms <= plane_rms_max_m`.

A collinear neighbourhood has two zero eigenvalues: `rms = 0` and the normal is any unit vector
orthogonal to the line. It is accepted. Rejecting it would need a new tunable (a minimum ratio
of the middle eigenvalue), which is an open question outside this module's contract; the
matcher only guarantees that such data never produces NaN (it either fails with an error or
returns finite values, test `CollinearMapNeverNaN`).

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
