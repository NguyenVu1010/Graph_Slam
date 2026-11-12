# mcu2d scan matching: submap, distance field, correlative search, Gauss-Newton

Implemented in `mcu2d/src/{occupancy_grid,distance_field,scan_matcher,degeneracy}.cpp`
(contracts in `mcu2d/include/mss2d/`). Tested by `mcu2d/tests/{pose2,grid,degeneracy,
matcher}_test.cpp`. Design background: ADR-034 (MCU constraints), ADR-004 and
`docs/derivations/degeneracy.md` (degeneracy rule and projected update, reused in 2D).

Conventions (`docs/conventions.md`): `T_a_b` maps points from `b` to `a`. Frames: `s` submap,
`l` LiDAR, `w` world, `T_w_s` stored with the submap. SE(2) tangent `delta = [x, y, yaw]`
(translation first, like `[rho, phi]` in 3D), right perturbation `T <- T * Exp(delta)`.
SI units, radians. Data are `float`; tests check the formulas in `double` through templates.

## 1. SE(2)

`T = (R(yaw), t)`, `T * p = R p + t`, `T_a_b * T_b_c = (R_ab R_bc, R_ab t_bc + t_ab)`.

    Exp([x, y, th]) = (R(th), V(th) [x, y]),  V = [[A, -B], [B, A]],
    A = sin(th)/th,  B = (1 - cos(th))/th          (Taylor 1 - th^2/6, th/2 - th^3/24 near 0)
    Log(T): V^-1 = [[h, th/2], [-th/2, h]],  h = (th/2) cot(th/2)   (1 - th^2/12 near 0)

`Exp(delta)` is the pose reached by moving with the constant body twist `delta` for one second
(test `ExpIsTheIntegratedConstantTwist`). Yaw is kept in `[-pi, pi)`; Log inverts Exp for
`|th| < pi`.

## 2. Occupancy submap

Square grid of `side` cells of size `res` in frame `s`; cell `(ix, iy)` covers
`[(ix - side/2) res, (ix + 1 - side/2) res)` in x (same in y), so the origin of `s` is the
grid centre. Log-odds `L` are stored as int8 in steps of `q = max_log_odds / 127` and clamped
to `[-127, 127]` (so `|L| <= max_log_odds`); `0` is unknown. `p(L) = 1 / (1 + exp(-L q))`
comes from a 256-entry table built once.

Insertion of a scan from the sensor pose `T_s_l`: each end point `T_s_l p_l` adds `hit` to
its cell; each cell on the Bresenham line from the sensor cell to the end cell (end excluded,
8-connected, `max(|dx|, |dy|)` cells) adds `miss`. Hits are applied first and every cell is
updated at most once per scan (a 1-bit mark per cell), so a grazing beam cannot erase the hit
of a neighbouring beam in the same scan. A beam leaving the grid updates the cells inside up
to the border.

## 3. Distance field

`D(c)` = Euclidean distance from the centre of cell `c` to the nearest occupied cell centre
(`L > occupied_log_odds`), clamped to `max_distance_m`. Computed exactly with the separable
algorithm of Felzenszwalb and Huttenlocher (2012):

1. per column, the exact 1D distance `g` (two linear sweeps);
2. per row, `D(x)^2 = min_x' (x - x')^2 + g(x')^2` as the lower envelope of parabolas.

Values above `cap = ceil(max_distance / res) + 1` cells are clamped anyway, so `g` is stored as
`min(g, cap)` in the uint8 output array itself and the row pass needs only `O(side)` scratch.
The envelope stays exact below `cap`. The result is quantised to uint8 in steps of
`max_distance_m / 255` (2 mm at the default 0.5 m). Test: brute force on random grids, error at
most half a step.

Sampling at `q_s`: with cell centres on the integer lattice `u = q.x / res + side/2 - 1/2`
(same for `v`), `i = floor(u)`, `fx = u - i`,

    D = (1-fx)(1-fy) D00 + fx(1-fy) D10 + (1-fx) fy D01 + fx fy D11
    dD/dx = [(1-fy)(D10 - D00) + fy (D11 - D01)] / res
    dD/dy = [(1-fx)(D01 - D00) + fx (D11 - D10)] / res

The point is rejected outside `0 <= u, v < side - 1`.

## 4. Correlative search (coarse)

Candidates: `guess * ` lattice `(x0 + i res, y0 + j res, yaw0 + k step)` for
`|i|, |j| <= search_linear_m / res`, `|k| <= search_angle_rad / step`. The translation step
is one cell, so after rotating the points once per `k` every translation is an integer cell
shift and the score is a table lookup per point (Olson 2009):

    score = (1/N) sum_n p(cell(R(yaw0 + k step) p_n + t0) + (i, j))

(cells outside the grid count 0). The best score wins; ties go to the smallest
`i^2 + j^2 + k^2`, so a featureless direction stays at the guess. Cost: `(2K+1)(2W+1)^2 N`
lookups, 21 x 13 x 13 x 720 = 2.6 M at the defaults.

## 5. Gauss-Newton refinement (fine)

Residual per point `p_n` (frame l), variable `T_s_l`:

    r_n(T) = D(T p_n)        (m)

With `T Exp(delta) p = R (V(th) delta_xy + R(th) p) + t = R (p + delta_xy + th perp(p)) + t + O(|delta|^2)`,
`perp(p) = [-p.y, p.x]`, and `g = grad D` at `q = T p`:

    J_r_delta = [ g^T R e_x,  g^T R e_y,  g^T R perp(p) ]       (unitless, unitless, m/rad)

(`residual_jacobian`, checked against central differences of `r(T Exp(h e_k))`, `h = 1e-6`,
on a smooth analytic field in double: relative error < 1e-6.)

Huber IRLS with threshold `k = huber_m`: `w_n = 1` if `r_n <= k`, else `k / r_n`;
cost `rho(r) = r^2` below `k`, `2 k r - k^2` above. Each iteration accumulates

    H = sum w_n J^T J,   b = sum w_n J^T r_n,   cost = (1/N) sum rho(r_n)

over the points inside the field and below the clamp (a clamped point has no gradient and no
correspondence in this submap). Step: `H delta = -b` restricted to the non-degenerate
eigen-directions (section 6), `T <- T Exp(delta)`. Converged when `|delta_xy| <
converged_translation_m` and `|delta_yaw| < converged_rotation_rad`. `D` is an unsigned
distance with a V-shaped minimum on the occupied cells, so a full step can overshoot: if the
cost rises, the step is halved from the last accepted pose (counts as an iteration) until it is
below the convergence thresholds, then the last accepted pose is returned.

Result: pose, `H` and cost at the last accepted pose, iterations, points used, status
(`kOk`, `kNotConverged` after `max_iterations`, `kTooFewPoints` below `min_points`).

Basin: with the default 0.5 m clamp, Gauss-Newton alone converged from every corner of the
default window (+-0.3 m, +-10 deg) in the test room; with a 0.3 m clamp it fails from
(-0.28 m, -0.27 m, +9.5 deg) and the correlative stage is needed (test
`CorrelativeSearchRescuesAStartOutsideTheGaussNewtonBasin`).

Accuracy floor: the map stores walls at occupied cell centres, so a wall is displaced by up to
`res/2` in the field. The pose bias is the weighted mean of these displacements over the
visible walls; the known-answer tests use walls off the cell lattice so it is not hidden.

## 6. Degeneracy

`H` is symmetric 3x3; `sym_eigen3` diagonalises it with cyclic Jacobi rotations (at most 16
sweeps, stopped when the off-diagonal sum is below `1e-9` of the diagonal), sorted ascending.
Direction `k` is degenerate iff `degeneracy_eig_threshold > 0` and `lambda_k < threshold`
(absolute, as in `core/degeneracy`). The update is the projected one,

    delta = - sum_{k not flagged, lambda_k > 0} v_k (v_k^T b) / lambda_k,

so nothing moves along a flagged direction (`degeneracy.md` section 3). Units: the `x, y`
block of `H` is a sum of squared unitless gradients (about "points constraining that
direction"), the yaw entries carry m and m^2, so the threshold is tuned for the scan's point
count and range. In a straight corridor along `x` every wall gradient is `(0, +-1)` and
`H e_x = 0` up to holes in the far wall cells (about 20 with a five-scan submap and 720 beams,
against about 280 for the weakest direction in the test room); the default threshold 50 sits
between them and the test flags exactly the corridor axis.
