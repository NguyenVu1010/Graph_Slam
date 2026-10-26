# Degeneracy detection and handling (D-01..D-03)

Implemented in `core/degeneracy/src/degeneracy.cpp` (contract:
`core/degeneracy/include/mss/degeneracy/degeneracy.hpp`). Tested by
`tests/unit/test_degeneracy.cpp` and `tests/component/test_degeneracy_scenes.cpp`.
Design background: ADR-004 (LiDAR factor covariance = inverse Hessian inflated along
degenerate directions), ADR-007 (voxel map, source of the k-NN planes).

Conventions: tangent `xi = [rho, phi]` (translation first), right perturbation
`T <- T * Exp(xi)` (see `lie_conventions.md`). This module converts no frames.

## 1. Source of the Hessian

Scan matching minimises point-to-plane residuals, one per scan point `p_l` with map plane
point `q_w` and unit normal `n_w`:

    r = n_w . (T_w_l * p_l - q_w)

With `T_w_l <- T_w_l * Exp(xi)` the Jacobian at `xi = 0` (`scan_matcher.hpp`) is the row

    J_r_xi = [ (R_w_l^T n_w)^T ,  -(R_w_l^T n_w)^T skew(p_l) ]

and the Gauss-Newton Hessian is `H = sum_i w_i J_r_xi_i^T J_r_xi_i` (IRLS weights `w_i`).
Write `n = R_w_l^T n_w` (normal in l). Since `-n^T skew(p) = (p x n)^T`, the row is
`[n ; p x n]^T`. In particular at `R_w_l = I` it is `[n_w ; p_l x n_w]^T`.

Variable: `xi`, the right perturbation of `T_w_l`, so H lives in the tangent of l.

## 2. Eigen-analysis and flags

H is symmetric positive semi-definite, so

    H = V diag(lambda) V^T,   lambda_0 <= ... <= lambda_5,   V^T V = I.

Direction `k` (eigenvector `v_k = V.col(k)`) is degenerate iff

    enabled  and  lambda_k < eig_threshold      (strict).

`lambda` is used as is: absolute, not normalised by the number of points or by the largest
eigenvalue (header decision). Its units are mixed: the `rho-rho` block is unitless
(`J` entries for `rho` are unitless), the `rho-phi` and `phi-phi` blocks carry `m` and
`m^2` from the lever arm `p x n`. The threshold is therefore tuned for the scan matcher's typical
point count and range (`config/default.yaml`).

If every normal is orthogonal to a translation direction `t` (e.g. `t` = corridor axis,
normals on the walls and the floor), then `[t ; 0]^T J_r_xi_i^T = n_i . t = 0` for every
`i`, so `H [t ; 0] = 0` and `lambda = 0` exactly along that direction.

## 3. Projected update

Let `m_k = 0` if direction `k` is flagged, else `1`. The projected Gauss-Newton update is

    delta' = V diag(m) V^T delta = delta - sum_{k flagged} v_k (v_k^T delta).

Proof of the contract: for any `j`, `v_j^T delta' = v_j^T delta - sum_{k flagged}
(v_j^T v_k)(v_k^T delta) = v_j^T delta - [j flagged] v_j^T delta`, using `v_j^T v_k =
delta_jk`. Hence `v_j^T delta' = 0` for flagged `j` and is unchanged for the others.

Repeated eigenvalues: the flag depends only on `lambda_k`, so all eigenvectors of a
repeated eigenvalue are flagged together. `sum_{k flagged} v_k v_k^T` is then the
orthogonal projector onto the whole eigenspace, which does not depend on the basis the
solver picks inside it (example: a single floor plane, three zero eigenvalues spanning
`rho_x, rho_y, phi_z`).

## 4. Covariance inflation

Let `f = relax_factor >= 1`, `s_k = sqrt(f)` if flagged else `1`:

    A = V diag(s) V^T = I + sum_{k flagged} (sqrt(f) - 1) v_k v_k^T
    cov' = A cov A^T,  then symmetrised as 0.5 (cov' + cov'^T).

`A` is symmetric and `A v_k = s_k v_k`, so

    v_k^T cov' v_k = (A v_k)^T cov (A v_k) = s_k^2 v_k^T cov v_k,

i.e. `f v_k^T cov v_k` for flagged directions and `v_k^T cov v_k` for the others. The
symmetrisation removes rounding asymmetry only (it does not change `v^T cov' v`). With
`f = 1`, `A = I` up to rounding. `cov'` stays positive definite because `A` is invertible.

Caller: `scan_match::ScanMatcher::align` applies `relax_covariance` to `MatchResult::cov`
(when enabled, with the final iteration's report), so that covariance is final. The relaxed
covariance is the only degeneracy mitigation in the factor; the pipeline passes the
DegeneracyReport to backend for diagnostics only (see graph.hpp) and must not call
relax_covariance again.

## 5. Frames

Everything here lives in the tangent of the pose the Hessian was built for: l for scan
matching. The pipeline converts afterwards: the covariance with
`transform_covariance(cov', T_b_l)` and the degenerate directions with
`se3_adj(T_b_l) * v_k` (`lie.hpp`).

## 6. Non-finite input

If any entry of the lower triangle of H is NaN or Inf (or the eigen solver fails), the
report is `lambda = 0`, `V = I` and, when enabled, every direction is flagged. Then
`delta' = delta - sum_k e_k e_k^T delta = 0` (no update from a broken linearisation) and
`A = sqrt(f) I`, so `cov' = f cov` (never NaN from H). When disabled nothing is flagged
and both functions are identities.
