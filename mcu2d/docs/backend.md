# mcu2d backend: SE(2) pose graph and submap loop closing

Implemented in `mcu2d/src/{pose_graph,loop_closer,backend}.cpp` (contracts in
`mcu2d/include/mss2d/{pose_graph,loop_closer,backend}.hpp`). Tested by
`mcu2d/tests/{pose_graph,loop_closer,backend}_test.cpp` and `no_alloc_test.cpp`. Builds on
`docs/frontend.md` (submaps, degeneracy flags) and `docs/scan_matching.md` (lattice search,
Gauss-Newton on a grid); design constraints from ADR-034.

Frames (`docs/conventions.md`): `w` map, `b` body, `l` LiDAR, `s_i` the frame of submap `i`
(origin at the grid centre). SE(2) tangent `[x, y, yaw]`, right perturbation
`X <- X * Exp(delta)`. Config: `Config::graph`, `Config::loop`; capacities in `config.hpp`.

## 1. Data flow

The main loop calls `Backend::on_scan_done(fe)` after every `FrontEnd::on_scan()`. It is
single-threaded and runs to completion:

1. accumulates the travelled path and the travel along flagged directions from consecutive
   front-end poses (section 2.2);
2. records the path when a submap starts;
3. when a submap finishes: adds a node with an odometry edge, keeps the submap as a compact
   bitmap (section 4), verifies loop candidates against older nodes with the last scan
   (section 5) and, if a loop was added, solves (section 3) and hands the correction to the
   front end (section 6).

## 2. Pose graph

**Nodes** are submap origins `X_i = T_w_si`, one per finished submap (about one per metre with
the default submap rotation: a new submap every half of `submap.max_distance_m` = 1 m or 25
scans). Scan poses are not nodes: a scan's pose is `X_i * T_si_b` with the front end's local
pose `T_si_b` (`ScanPose::T_s_b`), which is accurate inside one submap. This keeps the graph
at `kMaxGraphNodes = 128` nodes for 128 m of travel at the defaults, instead of ten times as
many scan nodes. Node 0 is fixed (gauge). When the graph is full, new submaps are not added
(`BackendStats::n_dropped_nodes`) and the front end continues as odometry.

### 2.1 Residual and Jacobians

Edge `(i, j)` with measurement `Z = T_si_sj`:

    e_xy  = R_z^T (R_i^T (t_j - t_i) - t_z)       (m, in the measured frame j)
    e_yaw = wrap(yaw_j - yaw_i - yaw_z)            (rad)

zero when `X_i^-1 X_j = Z`. With `S = [[0, -1], [1, 0]]` and `d = R_i^T (t_j - t_i)`:

    J_e_xi = [[-R_z^T, -R_z^T S d], [0, 0, -1]]
    J_e_xj = [[R(yaw_j - yaw_i - yaw_z), 0], [0, 0, 1]]

`X Exp(delta)` moves the position by `R V(delta_yaw) delta_xy`, whose derivative in `delta_yaw`
vanishes at `delta_xy = 0`, so the Jacobians are exact (no small-residual approximation of
the SE(2) right Jacobian). `edge_residual` is templated; the test checks it in double against
central differences of `e(X Exp(h e_k))`, `h = 1e-6`, over 50 random cases: relative error
< 1e-6.

### 2.2 Odometry edges

Between consecutive nodes, `Z = O_{i}^-1 O_j` with `O` the submap origins in the front-end
frame. Covariance in frame `j`, from the path `d` (m) between the two origins and the yaw
change:

    sigma_xy  = odom_sigma_xy_m + odom_sigma_xy_per_m d
    sigma_yaw = odom_sigma_yaw_rad + odom_sigma_yaw_per_m d + odom_sigma_yaw_per_rad |dyaw|
    Sigma_xy  = sigma_xy^2 I + k^2 tr(D) D,     k = odom_degenerate_fraction

`D = sum over the scans of step * P`, `P = sum over the matcher's flagged eigenvectors of
u u^T`, `u` their translation part rotated into `w` (`ScanPose::degenerate_xy_w`). Along a
flagged direction the front end follows the prediction (wheel), whose error grows with the
distance: for a travel `L` along one direction `u`, `D = L u u^T` and the extra term is
`(k L)^2 u u^T`, a standard deviation of `k L` along `u` only. `Sigma_xy` is rotated into frame
`j` and inverted; the yaw block is independent.

Defaults: 5 mm + 0.3 % of the path, 1 mrad + 0.5 mrad/m + 0.5 % of the turn, `k = 0.05`.
They follow the measured front end: 0.06 m ATE over the 37 m loop of `docs/frontend.md`
where every direction is constrained, and the 4 % wheel scale error of the revisit scenario
where it is not. Looser yaw (2 mrad/m) let the solver absorb a corridor's drift by bending the
well-constrained corridors (ATE 0.19 m instead of 0.10 m in the revisit scenario).

### 2.3 Loop edges

`Z = T_si_l * T_sj_l^-1`: the matched scan pose in the old submap `i` and the same scan in the
newest submap `j` from the front end. Information `diag(1/sigma_xy^2, 1/sigma_xy^2,
1/sigma_yaw^2)` from `loop.sigma_xy_m` (5 cm, a quarter of a compact cell's diagonal) and
`loop.sigma_yaw_rad` (0.01). A Cauchy kernel with `c = loop.cauchy_c` (3, in standard
deviations) bounds a wrong loop: `rho(s) = c^2 ln(1 + s / c^2)` of `s = e^T Info e`, applied as
the IRLS weight `1 / (1 + s / c^2)`.

## 3. Solver

Levenberg-Marquardt: each iteration assembles `H = sum w J^T Info J` and `b`, solves
`(H + lambda diag(H)) delta = -b`, retracts every free node by `Exp(delta_i)` and keeps the step
only if the robust cost does not rise (`lambda` / 10, else `lambda` x 10 and the nodes are
restored). It stops when the largest step entry is below `graph.converged_step` (1e-5), the
relative cost decrease below `graph.converged_rel_cost`, or after `graph.max_iterations`.

**Sparse structure.** Odometry makes the graph a chain; each loop links two distant nodes.
A dense `3N x 3N` factor at 128 nodes needs 74 k doubles (580 KB, lower triangle), too much for
the MCU. Instead the free nodes are ordered by reverse Cuthill-McKee (breadth-first from a
lowest-degree node, neighbours by degree, reversed; `O(N + E)` on fixed arrays), and `H` is
stored by rows in its envelope: row `r` from the first column any of its node's neighbours
reaches. A skyline Cholesky factors in place and its fill stays inside the envelope, so the
memory is the envelope size, checked against `kMaxFactorEntries = 12288` doubles (96 KB)
before a solve (`SolveStatus::kTooLarge`; the backend then drops the new loops). With many
loops from late to early nodes (a revisit) the ordering interleaves the laps: two laps of a
60-node ring with 19 loops need 4 818 doubles against 11 865 in insertion order (test
`ReverseCuthillMcKeeNarrowsARevisitedRing`). With a few chords the breadth-first front around
the whole ring can be wider than the chain plus a few long rows, so the envelope of the
insertion order is computed as well and the smaller one is used (test
`KeepsTheInsertionOrderWhenItIsNarrower`). The revisit scenario's 70 nodes and 6 loops need
1 566 doubles. A random graph with 384 edges does not fit and is refused (test
`RefusesAGraphWhoseFactorDoesNotFit`).

**Precision.** Nodes, the normal equations and the factor are double. Information values
span 1e2 (a degenerate odometry direction) to 1e6 (yaw) and the chain couples up to 128
nodes, so `H` is badly conditioned (it grows with the square of the chain length times the
spread of the information); in float (eps 6e-8) the Cholesky pivots of a long chain lose
most of their digits. A Cortex-M7 with a double-precision FPU
does this in hardware; the inputs (measurements, information) are stored as float. The test
`SkylineSolveMatchesADenseReference` compares the solution with an independent dense
Gauss-Newton: agreement below 1e-6.

**Steppable.** `begin_solve()` orders the graph and sizes the envelope; each `step()` runs
one LM iteration (assemble `O(E)`, factor `O(envelope x bandwidth)`, solve `O(envelope)`), so
a later time-sliced backend can run one iteration per idle slice. Nodes and edges must not
change during a solve. `solve()` is the same sequence and gives bit-identical results (test
`SteppedSolveIsBitIdenticalToTheBatchSolve`). Today the backend solves to completion inside
`on_scan_done`.

## 4. Compact submaps

A finished submap is kept as 1 bit per cell at twice the submap cell size
(`kLoopGridDownsample = 2`, 10 cm): a coarse cell is occupied if any of its four cells is.
1 250 bytes per node, 160 KB for 128 nodes, against 46 KB for one full submap (int8
log-odds at 5 cm). The free/unknown distinction is dropped: the loop score only rewards scan
points on occupied cells. The bitmap is taken when the submap finishes, so the front end may
recycle the slot.

## 5. Loop closing

**Candidates.** Query: the last scan of the submap just finished (`FrontEnd::last_points()`,
LiDAR frame, deskewed), placed in the graph by `X_j * T_sj_l`. Candidates are the older nodes
at least `loop.min_path_m` (6 m) back along the travelled path whose origin lies within
`loop.search_radius_m + loop.drift_per_m * path` (3 m + 3 % of the path) of the query; the
`loop.max_candidates` (2) nearest are verified. The search window grows the same way,
`min_window + drift * path`, clamped to `loop.max_window_m` (1.5 m) and `loop.max_window_rad`
(0.35 rad).

**Verification** (`LoopMatcher`, test file `loop_closer_test.cpp`):

1. The bitmap is smoothed into an 8-bit likelihood `V = 255 exp(-d^2 / 2)`, `d` the distance
   in cells to the nearest occupied cell (radius 2). At most `loop.max_query_points` (360)
   scan points are used, by an even stride.
2. Branch and bound on the lattice `guess + (i res, j res, k step)`: for each yaw the points
   are rotated once; for each 4 x 4 block of translations the bound is the sum of a max-pooled
   `V` (maximum over the block's shifts) at the points, and the block is expanded only if the
   bound reaches the best sum so far. The best sum starts at the acceptance threshold, so a
   window with no acceptable pose costs only the bounds. Ties go to the shift closest to the
   guess.
3. Gauss-Newton on `r = 1 - V(T p) / 255` (bilinear `V`, Jacobian of the same form as
   `residual_jacobian`), step halved on a rising cost, `loop.refine_iterations` (10).
4. Accept when the mean `V / 255` at the refined pose is at least `loop.min_score` (0.55),
   the ratio of the smallest to the largest eigenvalue of the translation block of the
   Gauss-Newton Hessian is at least `loop.min_eig_ratio` (0.15) and the refined pose stayed in
   the window. The ratio rejects the symmetric corridor: every shift along its axis scores
   alike (score 0.90, ratio 0.009 in test `RejectsTheSymmetricCorridorAsDegenerate`).

Accuracy: a room scan matched from a guess 0.78 m and 11 deg off lands 2 cm and 0.15 deg
from the truth (test `RecoversAKnownPoseFromAWrongGuessInsideTheWindow`), a quarter of a
coarse cell.

**Consistency.** After the solve, a new loop edge whose `e^T Info e` exceeds `loop.max_chi2`
(25) disagrees with the odometry and the older loops; it is deactivated and the graph solved
again (`n_rejected_chi2`). Under the Cauchy kernel a false loop keeps a large chi2 (486 in test
`CauchyKernelBoundsAFalseLoopAndItsChi2StandsOut`, a loop 1.5 m off in a noisy 40-node ring);
under a quadratic cost the ring bends until the same false edge has a chi2 of 2 and cannot be
told apart, and the worst node moves 0.40 m instead of 0.12 m.

**Capacity.** The edge buffer keeps room for the odometry edges of the nodes still to come;
a loop that would not fit, or whose envelope exceeds `kMaxFactorEntries`, is dropped
(`n_rejected_capacity`).

## 6. Correction to the front end

After a solve, `C = X_j * O_j^-1` maps the front-end frame onto the graph at the newest node.
`FrontEnd::apply_correction(C)` left-multiplies the current pose and every active submap
origin: the scan-to-submap geometry, the motion model (body frame) and the submap contents are
unchanged, so matching continues seamlessly (test `MovesThePoseAndActiveSubmapsRigidly`). The
backend applies `C` to its own copy of the newest origin, so the next odometry edge, computed
in the corrected front-end frame, stays consistent. Finished submaps in the front end's pool
are not moved (the graph owns them). Online poses jump by `C` at a correction (the online RPE
of the revisit run is 0.08 m against 0.02 m without the backend); a consumer that needs a
smooth pose uses `T_si_b` with the graph's `X_i`.

## 7. Synthetic evaluation

`revisit_scenario()` in `sim/frontend_sim.cpp`: a 24 x 14 m ring corridor, 2 m wide. The 20 m
bottom corridor is featureless, so the front end flags its axis and follows the wheel, which
reads 4 % fast; the right, top and left corridors have rotated boxes every metre on
alternate walls and are constrained. Gyro bias 0.02 rad/s, noise 0.01 rad/s. One lap (67 m)
from the bottom-left corner and 8 m more down the bottom corridor; 830 scans, 70 nodes. The
front end leaves the bottom corridor 0.77 m long and keeps that error around the lap.

ATE after aligning the first pose (SE(2)), RPE over 10 scans, as in `docs/frontend.md`.
"Graph" is every scan re-expressed through the final graph (`X_i * T_si_b`).

| run | ATE RMSE | ATE max | RPE trans | RPE rot | loops |
| --- | --- | --- | --- | --- | --- |
| front end only, seed 1 | 0.717 m | 1.09 m | 0.023 m | 0.14 deg | |
| graph, seed 1 | 0.104 m | 0.26 m | 0.015 m | 0.14 deg | 6 of 24 candidates |
| graph, `odom_degenerate_fraction` 0 | 0.717 m | 1.09 m | 0.023 m | 0.14 deg | 0 (6 dropped by chi2) |
| front end only, seeds 1-5 | 0.64-0.72 m | | 0.021-0.023 m | 0.12-0.14 deg | |
| graph, seeds 1-5 | 0.10-0.19 m | | 0.014-0.017 m | 0.12-0.14 deg | 5-7 |

The remaining error is the last 8 m down the bottom corridor after the final loop (0.16 m at
the end), where the candidates are degenerate and rejected (14 to 17 per run). Without the
degenerate term in the odometry information, the loops ask for a 0.7 m correction the stiff
odometry cannot give; their chi2 stays high and the consistency check drops all of them.

The well-constrained loop scenario of `docs/frontend.md`: ATE 0.064 m front end only,
0.058 m with the graph (4 loops, no rejection by chi2). The tests assert, for the revisit
scenario, front end ATE > 0.5 m, graph ATE < 0.2 m and < 0.3 times the front end, RPE no
worse than 1.1 times, at least 3 loops, degenerate rejections, no chi2 rejection; for the
loop scenario ATE < 0.08 m; in a 12 m straight featureless corridor every candidate is
rejected (17 of 17 as degenerate) and no solve runs.

Run it: `mss2d_frontend_eval --revisit --graph [--seed N] [--out DIR]` prints both runs,
the loop statistics and the costs below; with `--out` it also writes `DIR/optimized.tum`.

## 8. Cost on the host and memory

Desktop x86 core, Release, revisit scenario (seed 1):

| step | median | max |
| --- | --- | --- |
| `on_scan_done` that adds a node (bitmap, candidates, no loop) | 0.03 ms | 3.1 ms (2 candidates at the full window) |
| `on_scan_done` with a solve (verification included) | 2.8 ms | 3.1 ms |
| one candidate, 1 m and 20 deg window, 360 points | 0.74 ms | |
| online solve, 70 nodes, 3-4 iterations | about 0.1 ms | |
| full solve of the final graph from the odometry chain (70 nodes, 75 edges, 4-6 iterations) | 0.09-0.12 ms | 0.14 ms |

Verification dominates; the solve itself is small at this size. On the MCU (ADR-034,
Cortex-M7 about 480 MHz) expect one to two orders of magnitude more, so a candidate is tens of
milliseconds: one per idle slice in the time-sliced backend.

Static footprint (`sizeof`, bytes):

| object | bytes |
| --- | --- |
| `Backend` (total) | 332 504 |
| of which 128 x `CompactSubmap` | 160 000 |
| of which `PoseGraph` (nodes, 384 edges, ordering, 12 288-double envelope) | 130 168 |
| of which `LoopMatcher` scratch (smoothed and pooled grids, query points) | 37 124 |

With the front end (252 KB) and the merger (39 KB) this is about 620 KB, well above the
280 KB first SRAM estimate of ADR-034. Candidates for external memory: the compact submaps
(read once per candidate) and the factor envelope (only during a solve); the measured envelope
at 70 nodes is 1 566 doubles, so `kMaxFactorEntries` can shrink once the target's graph sizes
are known. A 4x downsample (20 cm) would cut the bitmaps to 40 KB at a lower loop accuracy.
