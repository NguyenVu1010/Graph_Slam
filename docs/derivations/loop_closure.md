# loop: loop-closure detection, verification and the loop factor (L-01..L-03, B-02, B-03)

Implemented in `core/loop/src/loop_detector.cpp` (contract:
`core/loop/include/mss/loop/loop_detector.hpp`) and wired in
`core/pipeline/src/estimator_core.cpp` (`EstimatorCore::try_loop_closure`). Decision record:
ADR-026. Tests: `tests/unit/test_loop_detector.cpp` (L-01..L-03, gates, contract),
`tests/component/test_loop_component.cpp` (two laps with drift, whole estimator),
`tests/unit/backend/test_backend_graph.cpp` (B-02), `test_backend_isam.cpp` (B-03),
`test_backend_factors.cpp` (Jacobians of the robust loop factor).

Conventions: right perturbation, tangent `[rho, phi]` (translation first) everywhere outside
`backend` (`lie_conventions.md`). Frames: `w` world, `b` body, `l` LiDAR; `h` is the
gravity-levelled body frame of one keyframe (below). Units SI.

## 1. Candidates

For the query keyframe `q` (the newest, already optimised and in the keyframe store) and every
stored keyframe `c`, with `T_w_b` the backend's optimised poses at this keyframe:

    t_c <= t_q - min_time_gap_s     and     |p_w_c - p_w_q| <= search_radius_m

sorted by distance, nearest first; at most `max_candidates` are verified. Distances use the
optimised poses, so earlier loops (and GNSS, when on) shrink the drift seen here.

Limit: a revisit is found only when the accumulated drift leaves the true place within
`search_radius_m` of the estimate. Larger drift needs a global place search (ring-key /
descriptor database), listed in the backlog (ADR-026).

## 2. Scan Context and the relative yaw

Levelled frame of a keyframe with `R_w_b = Rz(psi) R_tilt` (yaw `psi = atan2(R10, R00)`):
`R_h_b = R_tilt`, origin at the body. A stored point maps to `p_h = R_tilt (T_b_l p_l)`.

Descriptor (Kim & Kim 2018): `sc_rings x sc_sectors` cells over the horizontal plane of `h`
up to `sc_max_range_m`; ring `floor(rho / sc_max_range_m * rings)`, sector
`floor((atan2(y, x) + pi) / 2pi * sectors)`; cell value = max over its points of
`z_h + sc_height_offset_m`, and cells `<= 0` stay empty (0). `sc_height_offset_m` is set
below the body height above the ground (sim robot: 0.10 m, value 0.05 m), so ground returns are
empty: with the ground in every cell, every place looked alike (L-02 failed at an offset of
2 m: column cosine is high for any two non-negative profiles that share a floor).

Distance with column shift `s`:

    d(s) = 1 - mean_j cos(q[:, j], c[:, (j + s) mod S])     (over columns non-empty in both)

A world feature at bearing `beta` appears at azimuth `beta - psi_q` in `h_q` and `beta - psi_c`
in `h_c`, so column `j` of `q` matches column `j + s` of `c` with
`s * 2pi / S = psi_q - psi_c`: the shift is the relative yaw, quantised to one sector
(6 deg at 60 sectors). `d(s*) > sc_max_distance` rejects the candidate (no ICP).

## 3. ICP verification and fitness

Submap: the stored clouds of `c` and of up to `submap_keyframes` store neighbours on each side
(only those old enough, so the query's own recent clouds never enter), inserted into a private
`VoxelMap` at `T_w_b(k) * T_b_l` (optimised poses), cleared before each candidate.

ICP: `scan_match::ScanMatcher::align` (point-to-plane, Huber; `scan_match.*` with
`max_iterations = icp_max_iterations`, `max_corr_dist_m = icp_max_corr_dist_m`; degeneracy
projection off) of the query cloud against the submap. Seeds: the graph pose `T_w_bq`, and,
when the Scan Context yaw `psi_c + s* 2pi/S` differs from the graph yaw `psi_q` by more than
one sector, the same pose with that yaw (`Rz(psi_c + s* 2pi/S) R_tilt_q`). Each converged seed
whose result differs from `T_w_bq` by more than `max_correction_m` in position or
`max_correction_deg` in yaw is discarded (a rectangular box matched 180 deg off fitted
perfectly on E-01 data); of the rest, the higher inlier ratio wins.

Fitness, over the query's structure points only (`z_h + sc_height_offset_m > 0`, the set the
descriptor sees; ground fits any place and raised the ratio of a false pair 6 m apart to 0.83):
a point is an inlier when its nearest submap point is within `inlier_dist_m` and its `knn_k`
neighbours give a plane; its residual is the distance to the least-squares plane through them.

    ratio = n_inliers / n_structure        rms = sqrt(mean of residual^2 over inliers)

Nearest-point distances are not used as the residual: the stored clouds are spaced by
`map.keyframe_voxel_m` (0.5 m), so a perfect alignment still had RMS 0.09 to 0.12 m.

Acceptance: converged, `ratio >= min_inlier_ratio`, `rms <= max_rms_m`, and the smallest
eigenvalue of the final ICP Hessian `H` (tangent of `T_w_l`) `>= min_hessian_eig` (a corridor
without end walls in range is rejected here: `CorridorRejectedAsDegenerate`).

## 4. Measurement and covariance

    T_bc_bq = T_w_bc^-1 * (T_w_lq^ICP * T_l_b)

ICP perturbs `T_w_lq <- T_w_lq Exp(xi_l)`. Then
`T_bc_bq(xi) = T_bc_bq * T_b_l Exp(xi_l) T_l_b = T_bc_bq * Exp(Ad(T_b_l) xi_l)`, so the
covariance of the right perturbation of `T_bc_bq` (frame `b_q`, the `LoopConstraint::cov`
contract) is `Ad(T_b_l) C_l Ad(T_b_l)^T` (`transform_covariance`), with `C_l` =
`MatchResult::cov` (inverse Hessian scaled by `cov_scale * max(sigma_r^2, floor^2)`).

Floor: with `D = diag(cov_floor_m x3, cov_floor_rad x3)`,
`W = D^-1 C D^-1 = V diag(lambda) V^T`, `C' = D V diag(max(lambda, 1)) V^T D`. Then
`C' - D^2` is positive semi-definite (`CovarianceFloored`), directions already above the floor
are unchanged, and the floor is unit-aware (m and rad are not mixed in one eigen-problem). The
inverse Hessian alone is millimetre-level for a submap match; such a loop would be stiffer than
the whole odometry chain, so a wrong loop that passes the gates would win before the robust
kernel can act.

## 5. Backend factor and iSAM2 after a loop

`Graph::add_loop_factor(LoopConstraint)` (unchanged contract) adds the relative-pose factor of
`backend.relative_factor` (ADR-021, default `decoupled`) on `(X_i, X_j)` with noise
`Robust(Cauchy(k = 3.5485), Gaussian(P C' P^T))`, `P` the `[rho, phi] -> [phi, rho]`
permutation (`backend_factors.md` §5). `k = sqrt(chi2_6(0.95))` in whitened units: an inlier
loop keeps weight `>= 1/2` up to the 95 % gate, an outlier at `e` sigma keeps `1/(1 + (e/k)^2)`
(B-03: an outlier loop of 5 m / 29 deg drags the square graph by < 0.1 m; the same edge
without the kernel drags it by more). Cauchy rather than Huber: Huber's influence stays
constant for large residuals, so a loop that passes the gates but is metres off keeps pulling
with weight `k / e`; Cauchy's influence goes to zero.

After the update that adds the loop the pipeline runs `isam_extra_updates` more `optimize()`
calls with nothing new: each is one iSAM2 update, which relinearises the variables whose
linearisation point moved more than `backend.relinearize_threshold` and re-solves. One update
is a single Gauss-Newton step on the now non-linear correction; the extra ones let it settle
(two on the shipped configuration). Then:
- the front-end state, the keyframe reference `ref_` and the keyframe preintegrator restart
  from the optimised state of the query keyframe;
- the `VoxelMap` cache is rebuilt from the store at the optimised poses (forced, ADR-016
  decision 3, "backend reports a loop factor"), so scan matching continues in the corrected
  map;
- the global map export (`Estimator::visit_map`, ADR-016 amendment) reads the optimised poses at
  export time, so it follows the loop with no change.
After an accepted loop the next `min_interval_keyframes` keyframes are not searched (each
accepted loop costs an iSAM2 update plus a map rebuild).

## 6. Cost (per keyframe, desktop)

Candidate search: one `keyframe_state` lookup per stored keyframe and a distance. Per verified
candidate: two descriptors (`O(points)`), the shift search (`S^2 R` = 72k products), a submap
of up to `2 submap_keyframes + 1` stored clouds, at most two ICP runs and one fitness pass. The
bench numbers are in ADR-026.

## 7. No-GTSAM build

The stub backend has no keyframe pose (`keyframe_state` returns an error), so no keyframe view
is built, no search runs and `n_loop_searches` stays 0: loop closure is a no-op there
(`LoopComponent.BoxWorldEstimatorLoops`, stub branch).
