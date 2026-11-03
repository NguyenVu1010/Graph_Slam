# loop: loop-closure detection, verification and the loop factor (L-01..L-03, B-02, B-03)

Implemented in `core/loop/src/loop_detector.cpp` (contract:
`core/loop/include/mss/loop/loop_detector.hpp`) and wired in
`core/pipeline/src/estimator_core.cpp` (`EstimatorCore::try_loop_closure`). Decision record:
ADR-026. Tests: `tests/unit/test_loop_detector.cpp` (L-01..L-03, gates, contract),
`tests/component/test_loop_component.cpp` (two laps with drift, whole estimator),
`tests/unit/backend/test_backend_graph.cpp` (B-02), `test_backend_isam.cpp` (B-03),
`test_backend_factors.cpp` (Jacobians of the robust loop factor). Sections 9 to 12 (optional
correction gate from the graph's uncertainty, ambiguity check, PCM loop selection; ADR-026
amendment, ADR-028, backlog C1-12 / C1-16): `LoopFixture.CorrectionGate*`,
`LoopFixture.Ambiguity*`, `LoopSelector.*` (`tests/unit/test_loop_selector.cpp`),
`LoopAliasing.*`, `LoopComponent.TwoLapPcmNoWorseThanWithoutAcrossSeeds`,
`LoopComponent.PipelinePcm*`, `LoopComponent.AmbiguousResultBacksOffTheSearch`,
`LoopComponent.PcmRemovesAliasedLoopsKnownLoops`, `BackendGraph.RelativePoseCov*`,
`BackendGraph.RemoveLoopFactor*`, `BackendIsam.B03_OutlierLoopRemovedLeavesNoDrag`. Sections 13 to
16 (H6: diagnosis on E-07, loop NEES and floor, place search and kernel release, ADR-030; PCM
scale): `LoopNees.SyntheticScenes`, `LoopKernelRelease.*`, `LoopComponent.KernelRelease*`,
`LoopKnownLimits.KernelReleaseConsistentWrongPair`, `PlaceSearchFixture.*`,
`LoopAliasing.NoFalseLoopAtAnyDriftWithPlaceSearch`,
`LoopAliasing.LongPeriodCopyRankedFirstNoFalseLoopWithLoopSceneSettings`,
`LoopFixture.L02_DifferentPlacesNoFalseLoopWithPlaceSearch`,
`LoopDetector.PlaceSearchCostIsBoundedOnADenseRevisit`,
`BackendGraph.LoopFactorWithoutKernelAppliesTheGaussianShare`; evaluation of recordings:
`tools/loop_eval.py`.

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
descriptor database), listed in the backlog (ADR-026). Being found and passing the gates does
not mean being closed: see section 7 (aliasing above half the structure period, and the
kernel applying only a few percent of large corrections). Loops work only while the drift is
small.

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
perfectly on E-01 data). When the pipeline passes the graph's relative covariance, a fit must
also pass the chi-square correction gate (section 9); a fit that fails only that gate is kept
as a possible competitor (section 10) but cannot be the loop. Of the fits that can, the higher
inlier ratio wins.

Fitness, over the query's structure points only (`z_h + sc_height_offset_m > 0`, the set the
descriptor sees; ground fits any place and raised the ratio of a false pair 6 m apart to 0.83):
a point is an inlier when its nearest submap point is within `inlier_dist_m` and its `knn_k`
neighbours give a plane; its residual is the distance to the least-squares plane through them.

    ratio = n_inliers / n_structure        rms = sqrt(mean of residual^2 over inliers)

Nearest-point distances are not used as the residual: the stored clouds are spaced by
`map.keyframe_voxel_m` (0.5 m), so a perfect alignment still had RMS 0.09 to 0.12 m.

Acceptance: converged, `ratio >= min_inlier_ratio`, `rms <= max_rms_m`, and the smallest
eigenvalue of the final ICP Hessian `H` (tangent of `T_w_l`) `>= min_hessian_eig` (a corridor
without end walls in range is rejected here: `CorridorRejectedAsDegenerate`). A fit that passes
all of these then goes through the ambiguity check (section 10).

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
of up to `2 submap_keyframes + 1` stored clouds, at most two ICP runs and one fitness pass per
run. A candidate with a converged fit also asks the graph for one relative marginal (section
9); a candidate that passes every gate also runs up to `2 ambiguity_seeds_per_side` more ICP
runs (section 10). The bench numbers are in ADR-026.

## 7. Known limits (review C1; pinned by `LoopKnownLimits.*`)

**Aliasing (sections 10 and 11).** With structure repeating every `P` along a
direction the scan cannot otherwise resolve (identical pillar pairs in a corridor without end
walls in range), ICP from the graph seed converges to the copy nearest the seed. Before the
amendment, once the graph drift along that direction exceeded `P/2` (and stayed inside the fixed
3 m / 30 deg gate) the loop was wrong by about `P` and passed every gate (P = 2 m, drift 1.3 m:
error 1.98 m; Scan Context 0.13, ratio 0.93 to 0.94, RMS 3.7 cm, smallest eigenvalue above 300),
and dragged a loose chain (2 cm / 0.15 deg per 0.5 m step) by 0.34 to 0.64 m. With the
shipped configuration (ADR-028) the ambiguity check finds the true fit as a competitor and
rejects the loop at every drift (the gate is off, so the chain stiffness plays no part)
(`LoopAliasing.NoFalseLoopAtAnyDrift`): nothing false enters the graph, at the
price of no loop at all in an exactly periodic corridor. A false loop that still gets through
(one the ambiguity search misses) is removed by PCM as soon as true loops outnumber it
(section 11). With the optional graph gate on, the graph may rule the true fit out when the drift
is far outside its covariance, and the false loop is accepted: on the loose chain at drifts of
1.3 to 2.5 m (both periods) such loops drag the lap by 0.34 to 0.64 m, on the stiff chain only the
drift of exactly one period passes (drag about 1.4 cm)
(`LoopKnownLimits.GraphGateAliasingWhenDriftExceedsGraphCovariance`).

**Kernel against large true corrections.** With `C'` at the floor (5 cm / 1 deg) a correction
`c` is `c / 0.05` whitened sigmas; the Cauchy weight `1 / (1 + (e/k)^2)` at `e = 20` (1 m) is
0.03. A correct loop asking 0.5 / 1 / 2 m moves the lap end by about 10 / 3 / 1 % on a stiff
chain (5 mm per step) and 92 / 60 / 13 % on a loose one (2 cm). So a first revisit after a long
excursion with 0.5 to 2 m of drift is accepted but barely applied. The two-lap component test
passes because its loops arrive while the drift is centimetres (backlog C1-13: gate, then a
wider kernel or switchable constraints / GNC, and a floor calibrated by NEES). Addressed behind
flags by ADR-030 (section 15: no kernel for a loop with a PCM partner) and section 14 (floor).

## 8. No-GTSAM build

The stub backend has no keyframe pose (`keyframe_state` returns an error), so no keyframe view
is built, no search runs and `n_loop_searches` stays 0: loop closure is a no-op there
(`LoopComponent.BoxWorldEstimatorLoops`, stub branch).

## 9. Optional correction gate from the graph's uncertainty (ADR-026 amendment, ADR-028)

Off by default (`loop_closure.graph_covariance_gate: false`, ADR-028): it trusts the graph's
absolute covariance, and with a moderately overconfident graph it rejects every true loop
(below). The formulas stay for when the covariance is calibrated (C1-3).

The fixed gates (`max_correction_m`, `max_correction_deg`) ask "is this correction small?",
which is the wrong question: 1 m is plausible after a long loose excursion and implausible
after ten stiff keyframes. The right one is "is this correction consistent with how uncertain
the graph is about the relative pose of `c` and `q`?".

**Relative marginal.** `Graph::relative_pose_cov(i, j)` returns the covariance of
`T_bi_bj = T_w_bi^-1 T_w_bj` in the loop's own tangent (right perturbation, `[rho, phi]`,
frame `b_j`). With right perturbations of both poses, `T_w_bi Exp(d_i)` and `T_w_bj Exp(d_j)`:

    (T_w_bi Exp(d_i))^-1 T_w_bj Exp(d_j) = Exp(-d_i) T_bi_bj Exp(d_j)
                                          = T_bi_bj Exp(-Ad(T_bj_bi) d_i) Exp(d_j)
                                          ~ T_bi_bj Exp(d_j - Ad(T_bj_bi) d_i)

so with `J = [-Ad(T_bj_bi), I]` and `P` the 12x12 joint marginal of `(X_i, X_j)`:

    Sigma_ij = J P J^T = Ad P_ii Ad^T - Ad P_ij - P_ji Ad^T + P_jj,     Ad = Ad(T_bj_bi)

`P` comes from the iSAM2 Bayes tree (`backend_factors.md` section 8b, with its tests: known
answer on a chain, dense information solve and Monte Carlo on a 75-keyframe loopy graph).

**Cost** (`backend_factors.md` section 8b): it grows with the depth of `X_i` in the Bayes tree.
On a pose + velocity + bias graph with IMU, LiDAR and loop factors: 1.5 / 3.9 / 12.2 ms mean and
3.5 / 9.4 / 29 ms max at 61 / 300 / 1000 keyframes on the desktop, one to four `optimize()`
calls; several times that on a Pi 5 (not measured). It is called at most once per loop search,
and not at all with the gate off.

**Gate.** For a fit `T_w_bq^fit` and the graph pose `T_w_bq`, with `Z_g = T_w_bc^-1 T_w_bq` and
`Z_f = T_w_bc^-1 T_w_bq^fit`:

    e = Log(Z_g^-1 Z_f)  in [rho, phi],     S = Sigma_cq + C'_fit,     chi2 = e^T S^-1 e

`C'_fit` is the floored loop covariance of that fit (section 4): `Z_f = Z_g Exp(e)` with both
uncertainties in the same tangent, independent to first order, so `S` is the covariance of the
innovation `e` and `chi2` is chi-square with 6 DoF when the loop is true and both covariances
are consistent. A fit with `chi2 > correction_chi2_gate` (22.46, the 0.999 quantile) cannot be
the loop. All six axes enter, so a converged fit with a large roll or pitch correction is
caught here (C1-16: 6 deg of roll against a confident graph gives `chi2` above 30, the fixed
gates let it through, `CorrectionGateCoversRollAndPitch`). The fixed gates stay as a coarse
bound that needs no marginal.

**Why it is off (review H3, I-1).** With a stiff chain (`Sigma_cq` at the millimetre level, as
the pipeline's LiDAR factors are) `S` is dominated by the 5 cm / 1 deg floor and the gate admits
about `sqrt(22.46) * 5 cm` ~ 0.24 m along one axis, so a graph that is only moderately
overconfident loses every true loop. Two-lap drift test with the claimed odometry sigma halved
(noise unchanged): without the gate the 4 loops bring the end-to-start error from 1.21 m to
0.55 m; with it 43 searches are correction-gated, no loop is accepted and the error stays
1.21 m (`LoopKnownLimits.GraphGateDropsTrueLoopsOnAnOverconfidentGraph`). In the pillar hall on
the stiff chain a correct loop with 0.4 m of drift gives chi2 33 and is rejected. The pipeline's
box NEES is already 7.5 to 8.6 against 6, so the graph is overconfident in practice; the gate
needs the covariance calibrated first (C1-3), and a wider kernel (C1-13) cannot help large
corrections while it is on. With the gate on, if the graph cannot give the marginal the
candidate is rejected (`kNoMarginal`, fail closed; the pipeline logs the reason and counts
`n_loops_no_marginal`).

## 10. Ambiguity check: a second, separated fit (C1-12)

A false loop in repeated structure is a fit that is geometrically as good as the true one.
Gates on the fit alone (ratio, RMS, Hessian) cannot reject it, and the correction gate alone
cannot either when the copy happens to be closer to the drifted graph pose than the truth. The
check looks for a second fit and rejects the loop when two well separated fits are about
equally good.

Two options were weighed:
- compare the Scan Context distance of the best and second-best column shift, or the fitness at
  translated poses without ICP: cheap, but the descriptor is coarse (0.5 to 2 m rings) and the
  fitness with a 0.5 m inlier tolerance is flat in a corridor (walls fit at any shift), so
  both separate badly;
- **run ICP again from shifted seeds and compare converged fits** (chosen): each alternative is
  refined to its own optimum and judged by the same fitness as the acceptance, so a seed that
  falls back into the best fit's basin is recognised (same pose), and one that finds a copy
  gives a fit as good as the best. It costs ICP runs, bounded below.

**Direction.** Aliasing happens along the direction the scan constrains least. From the best
fit's Hessian `H` (tangent `[rho, phi]` of `T_w_l`) the translation information with rotation
marginalised out is the Schur complement `H_t = H_rr - H_rf H_ff^-1 H_fr` (in `l`);
`R_w_l H_t R_w_l^T` in `w`, its horizontal 2x2 block, and the eigenvector `u` of its smallest
eigenvalue is the search direction (the corridor axis in the pillar corridor).

**Seeds.** `T_w_bq^best` shifted by `+-k * ambiguity_seed_step_m * u`, `k = 1..ambiguity_seeds_per_side`
(1 m, 2: offsets +-1 and +-2 m), nearest first; a seed farther than `max_correction_m` from the
graph pose is skipped. ICP from a seed converges into the basin of whatever copy is nearest it,
so with 1 m steps every copy within about 2.5 m of the best fit is reached (pillar periods of
2 and 2.5 m: `AmbiguityRejectsPeriodicStructure`). The other step-3 seed's fit, when there is
one, is a competitor too.

**Competitor.** A converged fit that passes the fixed gates, has `rms <= max_rms_m`, and lies
more than `ambiguity_min_separation_m` (0.5 m) or `ambiguity_min_separation_deg` (5 deg) from the
best. Its weight is `ratio_other / ratio_best`; when it reaches `ambiguity_max_ratio` (0.9) the
loop is rejected (`kAmbiguous`). In the pillar corridor the copy one period away fits at 0.93
to 0.94 of the true fit; in the irregular hall no competitor reaches the gate
(`AmbiguityCheckKeepsUniquePlaces`, L-01..L-03).

**The graph decides only when it is clear (optional gate on).** With the graph's covariance
(section 9, `graph_covariance_gate` true) a
competitor is ruled out only when both hold: it fails the chi-square gate itself
(`chi2_other > correction_chi2_gate`) and it is at least `ambiguity_chi2_margin` (13.8, a
likelihood ratio of `exp(13.8 / 2)` = 1000) worse than the best in `chi2`. So:
- a competitor the graph finds plausible always competes. While the true drift is within the
  gate, the true fit is such a competitor whenever ICP snapped to a copy, and the false loop is
  rejected. With a covariance consistent with metre-level drift (relative sigma ~0.6 m) no false
  loop is accepted for any drift up to 2 m at either period (`LoopAliasing.*`);
- a copy far outside the gate while the best fit is near the graph is ignored: the true loop is
  accepted (review's loose chain, relative sigma ~0.2 m, drift 0 to 0.6 m: accepted, error
  7 to 8 mm), where without the graph it would be ambiguous;
- what the graph cannot fix: drift far beyond its covariance. The copy is then the plausible fit
  and the truth the implausible one, and the graph picks the copy (loose chain, drift 1.3 to
  2.5 m; section 7). The symmetric case shows why no rule can do better from this data: drift 0
  with the copy 2 m away and drift 2 m with the truth 2 m away give the detector the same two
  fits and the same graph covariance.

With the gate off (shipped) every competitor counts, so in an exactly periodic corridor no
loop is accepted (`AmbiguityRejectsPeriodicStructure`, `NoFalseLoopAtAnyDrift`):
recall is given up where the place cannot be told apart.

**Cost.** Only a candidate that passes every other gate runs the check: up to
`2 * ambiguity_seeds_per_side` (4) more ICP runs, stopping at the first competitor. With the
optional gate on, a seed is skipped when the graph already rules out everything it can reach:
the chi2 the gate would give the best fit shifted to the inner edge of the seed's reach
(offset `(k - 1/2) * ambiguity_seed_step_m`, same rotation and covariance) is above the gate
and `ambiguity_chi2_margin` above the best fit's. ICP from a seed converges to a copy within
about half a step of it, so such a seed cannot produce a competitor that counts (first order:
the actual fit's covariance and rotation differ slightly from the prediction). On a stiff graph
(the pipeline's: millimetre LiDAR factors, `S` at the 5 cm floor) every 1 m seed is skipped and
the check costs nothing; on the loose and consistent chains of the tests it runs.

**Latency bounds without the graph (review H3, I-2).** With the gate off (shipped) the graph
prunes nothing, and the 4 ICP runs doubled the loop keyframe's scan time on E-01 (max 35 to
61 ms) while changing nothing. Three bounds, none needing the graph's covariance:
- **Fitness pre-screen.** Before a shifted seed's ICP, one fitness pass (one kNN query per
  structure point, about one ICP iteration) at the best fit shifted to the seed; ICP runs only
  when that raw inlier ratio reaches `ambiguity_prescreen_ratio` (0.8) x the best fit's. A seed
  that lands on a copy of the scene scores like the copy; a seed in a unique place scores low (the
  structure that fitted is now displaced by 1 or 2 m). In the pillar hall 2 to 4 of the 4 seeds
  per place stop here (`AmbiguityCheckKeepsUniquePlaces`); the periodic corridor stays ambiguous
  at every drift (`AmbiguityRejectsPeriodicStructure`, `NoFalseLoopAtAnyDrift`).
- **ICP iteration cap.** A shifted seed's ICP stops after `ambiguity_icp_max_iterations` (10, a
  second `ScanMatcher` with that limit) and an unconverged fit is still judged by its fitness: a
  competitor only has to reach the copy's basin, not converge in it.
- **Back-off.** After a `kAmbiguous` result the pipeline skips `ambiguity_backoff_keyframes` (5)
  keyframes: in a periodic aisle every search is ambiguous, so without it every keyframe paid one
  detection plus 4 ICP runs (`LoopComponent.AmbiguousResultBacksOffTheSearch`: 43 searches
  without back-off, 8 with 5).
Measured on the synthetic scenes (dense clouds, 40k to 60k points): 25 to 40 ms per full ICP;
with the whole estimator see section 12.

## 11. Loop selection by pairwise consistency (PCM, ADR-028)

Mangelson, Dominic, Eustice and Vasudevan (2018): a set of loops is trustworthy when its loops
agree with each other; the largest set of pairwise consistent loops is selected (maximum clique).
This needs no trust in the graph's absolute covariance: when the two odometry segments of a
pair's cycle are of similar length and direction (two loops of one revisit), the drift
accumulated between the two passes is common to both loops and cancels in their cycle. Over long
cycles it does not cancel (a loop `(i, i+N)` against `(i, i+2N)` holds a whole lap of odometry),
and the odometry's drift has to be in the covariance: section 11b.

**Odometry chain** (`loop::LoopSelector`, no GTSAM). The pipeline gives each keyframe's LiDAR
factor measurement `Z_k = T_{k-1,k}` with its covariance `C_k` (`[rho, phi]`, frame `b_k`),
exactly what entered the graph. Prefix sums: `T_0k = T_0,k-1 Z_k` and, in the left tangent at
the origin, `S_k = S_{k-1} + Ad(T_0k) (s^2 C_k) Ad(T_0k)^T` with `s` = `pcm_odometry_sigma_scale`.
With `T_0a` fixed and right perturbations `d_m` on the increments `m = a+1..b`,

    T_0a prod_m (Z_m Exp(d_m)) ~ Exp(sum_m Ad(T_0m) d_m) T_0b

so the left-tangent covariance of `T_0b` given `T_0a` is `S_b - S_a`, and the right
perturbation of `T_ab = T_0a^-1 T_0b` follows from `T_0a^-1 Exp(eta) T_0b = T_ab Exp(Ad(T_0b^-1) eta)`:

    O(a -> b):  T_ab = T_0a^-1 T_0b,   C_ab = Ad(T_0b^-1) (S_b - S_a) Ad(T_0b^-1)^T

O(1) per query (`OdometryPrefixSumsMatchDirectCompounding`: equal to compounding the increments
one by one, `C <- Ad(Z^-1) C Ad(Z^-1)^T + C_m`, to 1e-9). `O(b -> a)` is the inverse,
`(A, C)^-1 = (A^-1, Ad(A) C Ad(A)^T)`. The graph's marginal is not used: it already holds the
loops being judged.

**Pairwise test.** Loops `a = (i_a, j_a, Z_a, C_a)` and `b`. Around the cycle
`b_ja -> b_ia -> b_ib -> b_jb -> b_ja`:

    X = Z_a^-1 * O(i_a -> i_b) * Z_b * O(j_b -> j_a),     e = Log(X)

with right-perturbation composition `(A, Ca)(B, Cb) = (A B, Ad(B^-1) Ca Ad(B^-1)^T + Cb)` of the
four pieces taken independent (the two odometry segments overlap only when the loops overlap in
time; ignored). Consistent when `e^T Sigma_X^-1 e <= pcm_chi2_gate` (22.46).

**Scale of the odometry covariance (before the drift term).** `s = pcm_odometry_sigma_scale`
inflates the odometry covariance in the test only (the graph is unchanged): the front end may
claim sigmas up to `s` times too small. Two-lap test, pairs of true loops (seed 42, chi2 with
`s = 1`, no drift term):

| claimed sigma | pairs one lap apart | pairs with the two-lap loop (0, 150) |
| --- | --- | --- |
| x1 | 0.2 to 1.2 | 17.7 to 19.6 |
| x0.5 | 0.3 to 3.4 | 33.6 to 41.3 |
| x0.25 | 0.3 to 6.5 | 51.8 to 79.4 |

The pair with `(0, 150)` has segments of 0 and 78 keyframes, so the lap's yaw bias (0.1 deg per
step, 7.8 deg) is not common to both and does not cancel. A fixed `s` cannot fix that: the
bias error grows with the segment length (yaw like `L`, lateral like `L^2`) while a white-noise
covariance grows like `L` (yaw variance) and `L^3` (lateral variance), so every `s` fails beyond
some cycle length (review H3, B-1: three laps of radius 6 to 20 m lost 2 to 4 true loops even
with `s = 2`). The bias is modelled instead (section 11b), and `s` drops to 1.5. Two loops
aliased by the same world shift are consistent with each other: aliasing that is consistent
along a whole aisle is not PCM's to catch, and is left to the ambiguity check (section 10).

### 11b. Yaw drift in the odometry covariance (ADR-028 amendment, option C)

**Model.** The odometry heading drifts at an unknown constant rate `b` (rad per metre
travelled), `b ~ N(0, sigma_b^2)`, `sigma_b = pcm_yaw_drift_deg_per_m`. Over a segment the
drift is fully correlated (one `b` for every step), unlike the white step noise `C_k`. Step `k`
(length `d_k = |p(Z_k)|`) gains `b d_k` of yaw about its body z at its end:
`Z_k -> Z_k Exp(b d_k e_yaw)`, `e_yaw = [0 0 0 0 0 1]^T` in `[rho, phi]`.

**Propagation.** With `T_0a` fixed, to first order

    T_0a prod_{k=a+1..b} Z_k Exp(b d_k e_yaw) = Exp(b sum_k d_k Ad(T_0k) e_yaw) T_0b

(moving each factor to the left through `T_0k`), so in the left tangent at the origin the drift
direction is `G_b - G_a` with the prefix `G_k = G_{k-1} + d_k Ad(T_0k) e_yaw`, and the right
perturbation of `T_ab` is `g = Ad(T_0b^-1) (G_b - G_a)` times `b`:

    C_ab = Ad(T_0b^-1) (S_b - S_a + sigma_b^2 (G_b - G_a)(G_b - G_a)^T) Ad(T_0b^-1)^T

`Ad(T_0k) e_yaw = [p_0k x z_k ; z_k]` (`z_k` the body z axis of step k in the origin frame): the
yaw part of `g` is the distance travelled `L`, and the translation part is the lateral lever of
each step's yaw error about the segment end, `sum_k d_k (p_k - p_b) x z_k`, which for a straight
segment is `L^2 / 2` perpendicular to the path. So the dominant cross term, yaw against lateral
position, has covariance `sigma_b^2 L^3 / 2`: lateral error equals distance times yaw error,
fully correlated. Prefix sums keep the query O(1); the cost is one 6-vector per keyframe.
`YawDriftCovarianceMatchesNumericalDifference` checks `g` against central differences of the
re-composed chain (perturbing `b` by 1e-6 rad/m) to 1e-6, in both directions. The two odometry
segments of a cycle get independent `b`s. When the cycle traverses them in opposite senses (two
loops of one revisit in the same direction: `i_a < i_b` and `j_a < j_b`) a common drift cancels
between them, so independence over-states the cycle covariance (more permissive). When it
traverses them in the same sense (a revisit in the reverse direction: `i_a < i_b` but
`j_a > j_b`) a common drift adds, up to twice the variance independence gives, and the test is
stricter than the model intends. The review's out-and-back probe (corridors of 20 / 50 / 100 m,
reverse revisits, drift at 1 and 1.5 sigma) stayed below chi2 8.4, so this has not mattered at
the default; a correlated-drift model across segments is a refinement if it does.

**Default.** `sigma_b = 0.2 deg/m`: the repo's own odometry error model, the yaw bias of the
synthetic drifting odometry (0.1 deg per 0.5 m step, `LoopComponent.TwoLapDrift*`, the review's
three-lap probe), taken as one sigma of an unknown drift. It is large for real LiDAR odometry
(a heading drift of a few tenths of a degree per 100 m is typical); a value from the front end's
measured drift on recordings replaces it when that is calibrated (C1-27, C1-3). It is the
largest drift the pairwise test then forgives.

**Remaining inflation.** With the drift term, `s = 1` closes every true loop of the three-lap
probe at the claimed covariance but rejects one at claimed sigma x0.5 (radius 9 m, chi2 25.1, the
white part of a whole lap claimed at half its size); `s = 1.5` closes the probe's laps up to 125 m
(radius 20 m). So `s` covers a claimed white step noise up to 1.5 times too small, not the bias.
**Honest limit (review H3 I-1):** claim x0.5 is 2 times too small, and 1.5 holds it only for
cycles up to about 125 m. On a 250 m lap (radius 40 m, 12 true loops, 7 seeds) claim x1 keeps
every loop (largest chi2 12.3) but claim x0.5 loses 1 to 3 true loops at 3 of 7 seeds (seed 2024:
3 rejected, chi2 24.5; seed 3: 1 removed and 1 rejected, 33.2; seed 31337: 2 rejected, 38.0)
(`ThreeLapTrueLoopsAllStaySelected` pins the covered range). Owner ruling (C1-27, closed): 1.5 is
final and this limit is accepted; `s = 2` would cover it at the price of weaker aliasing
rejection against far loops (section 16).

**First-order limit (review H3 m-2).** The drift term is linear in `b`. With a true drift of 1.5
to 2 times `sigma_b` on laps of 125 m and more, the yaw accumulated over a lap (about 50 to 100
deg) is far outside the linearisation, and true loops are rejected and removed again (up to 9 of
12 removed at 2 sigma, radius 40 m, claim x0.5). The model holds while `sigma_b` bounds the real
drift and the accumulated yaw per cycle stays moderate; calibrating `sigma_b` (C1-27) sets that.

| three laps, 4 + 5 true loops, seed 42 | `s = 2`, no drift term (before) | `s = 1.5`, `sigma_b = 0.2 deg/m` |
| --- | --- | --- |
| radius 6 m (75 keyframes per lap), claim x1 / x0.5 | 0 / 2 removed | 0 / 0 (largest chi2 3.9 / 9.4) |
| radius 9 m (113), x1 / x0.5 | 0 / 4 removed | 0 / 0 (5.0 / 13.5) |
| radius 12 m (151), x1 / x0.5 | 0 / 4 removed | 0 / 0 (2.6 / 6.4) |
| radius 20 m (251), x1 / x0.5 | 4 / 4 removed | 0 / 0 (2.6 / 4.9) |

**Aliasing.** The drift term grows with the segment, so it weakens the rejection of an alias
only against far true loops: a loop aliased by 2 m against true loops 10, 13, 30 and 50 keyframes
away gives chi2 161.4 / 100.7 / 49.9 / 33.8 with the term and 161.5 / 100.7 / 51.3 / 35.5 without
(`YawDriftTermKeepsNearAliasesInconsistent`, `s = 1.5`, all four asserted): at most 5 %, all
still above the gate.
With `s = 2` and no drift term the 50-keyframe pair had dropped to 21 (below the gate).

**Selection.** Up to `pcm_max_loops` (64) loops, one bit each: consistency bitsets, maximum
clique by Bron-Kerbosch with pivoting (bounded by `pcm_max_clique_steps`). Ties keep the most
currently selected loops (no flip-flop on equal evidence), then the oldest. The difference with
the previous selection gives the loops to add to the graph (oldest first; a loop that left and
comes back is added again) and to remove (`Graph::remove_loop_factor`, one iSAM2 update,
`backend_factors.md` section 8c). The pipeline then runs the extra updates, rebuilds the map
and re-syncs the front end as after a new loop (section 5).

- **First loop and few loops.** The first loop forms a clique of one and is added at once (as
  before ADR-028, with Cauchy around it). A second loop inconsistent with it ties and is not
  selected; a third that agrees with one of them decides
  (`LargestConsistentSetWinsAndRemovesLaterInconsistentLoops`,
  `B03_OutlierLoopRemovedWhenTwoTrueLoopsAgree`). Two inconsistent loops and no third: the
  earlier one stays.
- **Full.** When `pcm_max_loops` are held the oldest leaves PCM: frozen in the graph if it is
  selected (no longer reconsidered), dropped otherwise (`OldestLoopLeavesWhenFull`).
- **Truncated search.** The Bron-Kerbosch search starts from the current selection (still a
  clique: adjacency among held loops never changes, an evicted slot is cleared from it), so a
  search cut short by `pcm_max_clique_steps` never makes the selection worse
  (`TruncatedCliqueSearchNeverMakesTheSelectionWorse`).
- **Re-sync after a failure.** A failed `optimize()` right after PCM changed the graph drops the
  pending loop factors; the pipeline asks `Graph::has_loop_factor` for each loop it just added and
  calls `LoopSelector::forget` for the lost ones, so the selector never counts a loop that is not
  in the graph (`ForgetDropsAHeldLoopSilently`, `LoopComponent.PipelinePcmForgetsALoopDroppedByAFailedUpdate`).
  A removal cannot fail any more: it is queued into the next update and survives a failed one
  (`backend_factors.md` section 8c).
- **Accounting.** Each loop factor that enters the graph is logged and counted
  (`n_loops_accepted`), including a loop that enters after its detection or comes back; each
  removal is logged and counted (`n_loops_removed`).
- **Cost.** Per proposed loop: up to 64 pairwise tests (four 6x6 compositions and an LDLT each)
  and one maximum clique on at most 64 vertices; no allocation (`ProposeDoesNotAllocate`). Per
  keyframe: one prefix update (one 6x6 adjoint product). A removal costs no update of its own: it
  rides in the next `optimize()`, only when PCM changes its mind.
- **Long cycles (review H3, B-1, fixed by section 11b).** When a cycle holds a long stretch
  of odometry that does not cancel (a loop `(i, i+N)` against a loop `(i, i+2N)`: a whole lap),
  an odometry bias missing from the covariance made true loops look inconsistent: three laps lost
  2 to 4 true loops of the first revisit, the two-lap test one at seed 7 (x0.5) and seed 2024
  (x0.25). With the drift term every true loop stays (`ThreeLapTrueLoopsAllStaySelected`,
  `LoopComponent.TwoLapOverconfidentPcmNoWorseThanWithoutAcrossSeeds`).

## 12. Results with the shipped configuration (PCM on, graph gate off)

| test | before ADR-028 (graph gate on) | after |
| --- | --- | --- |
| two-lap drift, claimed sigma x1 | 1.207 -> 0.082 m, 4 loops | 1.207 -> 0.082 m, 4 loops |
| two-lap drift, seeds 42 / 1 / 7 / 2024, claimed sigma x1 | as without PCM | the same loops as without PCM, end-to-start equal (0.082 / 0.125 / 1.199 / 0.454 m), none removed |
| two-lap drift, seeds 42 / 1 / 7 / 2024, claimed sigma x0.5 (review I-1) | 1.207 m at seed 42, 0 loops, 43 gated | as without PCM: 0.548 / 1.156 / 1.129 / 0.299 m, none rejected or removed (before the drift term seed 7 lost a true loop: 1.350 m) |
| the same, claimed sigma x0.25 | 1.207 m, 0 loops | as without PCM: 1.123 / 1.399 / 1.698 / 0.511 m (before the drift term seed 2024 lost a true loop: 0.576 m) |
| three laps, 12 true loops, 7 seeds: radius 6 to 40 m at claim x1, 6 to 20 m at x0.5 (review B-1) | n/a | all true loops selected, none removed (before the drift term: 2 to 4 removed); claim x0.5 at radius 40 m loses 1 to 3 at 3 of 7 seeds (section 11b, known limit) |
| known loops + 3 aliased, one before any true loop | n/a | aliased one removed, others never selected, end-to-start 0.055 m (0.051 m without them; Cauchy only 0.077 m) |
| whole estimator, injected wrong loop (0.8 m, loosened chain) then two true ones | n/a | wrong loop removed (`has_loop_factor` false), the true ones in the graph, final pose 3 mm from the true-only run against 6.0 cm when the wrong loop is kept; a mutant that skips `remove_loop_factor` fails (4.5 cm) (`PipelinePcmRemovesAWrongLoopAndReAddsTheTrueOne`) |
| aliasing probe, P = 2 / 2.5 m, drift 0 to 2.5 m | false loops at large drift on the loose chain (drag 0.34 to 0.64 m) | every case ambiguous, nothing added (drag 0); the graph's covariance is not used, so chain stiffness plays no part |
| B-03 outlier loop | Cauchy: drag < 0.1 m | removed: drag < 1e-6 m (backend); PCM removes it once two true loops agree |

Whole estimator (desktop WSL, median of 5 interleaved runs, loops on; ADR-026 amendment for the
table): E-01 and E-07 (the warehouse three-lap scene, now E-07W; re-recorded after #41) give the same `est.tum` as before ADR-028 (PCM
selects every loop: 6 and 16, none removed, no false loop). With the latency bounds of section 10
the ambiguity check costs nothing measurable: max per scan on the loop keyframes 44.8 ms against
43.3 ms with the check off (E-01) and 66.5 against 73.8 ms (E-07W); before the bounds it was 73 and
106 ms. Loops-on max against loops-off max: 1.37 x (E-01) and 1.29 x (E-07W), as at e946ee9 (1.37,
1.37): the rest is the loop's own cost (map rebuild, iSAM2 updates; C1-15). PCM itself costs 0.08
to 0.12 ms per proposed loop (at most 0.28 ms with 64 loops held, 61 to 1000 keyframes), the
odometry prefix update 0.2 to 0.3 us per keyframe.

## 13. Diagnosis on E-07 (H6, backlog C1-31)

Where each candidate of the E-07 outdoor loop stops, on the recording of 2026-11-02 (seed 42,
body frame `imu_link`, L2 lever arm; `imu.init_level_start` on, owner ruling S5) with the
shipped loop settings (`config/sim/outdoor_e07_loops_on.yaml` before ADR-030). Instruments
(diagnostics only, no effect on the estimate): one debug line per search,
`loop search: j .. outcome .. n_cand .. cand .. sc .. ratio .. rms .. eig .. amb .. corr_m ..
corr_yaw_deg` (`LoopResult::correction_m` / `correction_yaw_rad`: the correction the reported
fit asks of the graph pose), the measurement and the floored sigmas of a verified loop, one
`loop applied: .. residual_m a -> b` line per loop that enters (the loop residual against the
graph before the loop and after the updates), and `mss_bench`'s `kf_opt.tum` (the stored
keyframes at their optimised poses after the replay). The truth comes from the recording's
`gt.tum`; "true distance" is between the query and the candidate.

| stage | what the data shows | blocks true loops? |
| --- | --- | --- |
| search radius (C1-1) | 31 keyframes of the revisit have a true earlier place within 3 m (0.0 to 2.6 m); on the estimate that place is 3.2 to 4.5 m away, inside the 10 m radius. The radius never excludes it. | no |
| candidate choice (nearest on the estimate) | the drift (4.0 m at the return) moves the query next to other keyframes: the nearest one on the estimate is 2.6 to 12.8 m from the query in truth (2.6 to 3.4 m once the robot is back at the start), while the revisited keyframe (0.0 to 0.5 m) is never tried (`max_candidates` 1) | yes: the root cause |
| Scan Context gate | 21 searches: the nearest-on-estimate candidate is a different place (sc 0.47 to 0.73 > 0.4) | follows from the choice |
| ICP verification | from the graph seed the true pose is 3 to 4 m away, beyond `icp_max_corr_dist_m` (2 m): 1 failed, 4 converged to partial fits (corrections 1.6 to 2.7 m) | follows from the choice |
| ambiguity check | those 4 partial fits have a competitor at 0.95 to 1.04 of their inlier ratio: rejected (correctly, they are not the true alignment) | no (it removed wrong fits) |
| `max_correction_m` gate (3 m) | 3 converged fits ask 3.27 to 3.33 m: discarded | yes, at a drift above 3 m |
| PCM | no loop reached it | - |
| kernel (C1-13) | no loop reached it | - |

Reproduce with `tools/loop_eval.py` (commands in section 15). On recording B (2026-11-03 04:40
UTC; drift 3.1 m at the return) the shipped settings give 321 searches: 21 Scan Context, 6
correction gate, 2 ambiguous, 2 accepted (true, within 0.9 cm), which apply 8 and 10 % of what they
ask (2.66 -> 2.45, 2.30 -> 2.08 m); on recording C (3.7 m) 315 searches: 22 Scan Context, 3
ambiguous, 2 accepted (true, within 1.1 cm), applying 7 and 8 % (2.97 -> 2.78, 2.64 -> 2.43 m).
The rest of this section is recording A (2026-11-02).

So with the level start this recording drifts 4.0 m at the return (2.6 m without it, the
opposite of the H5 recording: 2.8 and 3.7 m), and no loop is accepted: 318 searches, 289 without
a candidate, 21 Scan Context, 1 ICP failure, 4 ambiguous, 3 correction gate.

Without the level start (2.6 m of drift, below the gate) the same settings accept 5 loops, all
true (1.3 to 2.7 cm, 0.01 to 0.08 deg against the truth), and the kernel is the limit: each loop
applies 10 to 34 % of what it asks (`loop applied`: 2.82 -> 2.56, 2.34 -> 2.07, 1.90 -> 1.62,
1.48 -> 1.12, 1.01 -> 0.66 m). The return error of the smoothed trajectory falls from 2.64 to
0.74 m, the final one from 1.95 to 0.55 m; ATE of `kf_opt.tum` 0.718 -> 0.260 m.

Two probes on the level-start run, configuration only: `max_correction_m` 6 accepts 1 loop
(the nearest-on-estimate candidate is still the wrong place); `max_correction_m` 6 with all
candidates in the radius verified (`max_candidates` 20) accepts 3 true loops but costs 353 ms on
one keyframe (20 candidates verified). Both leave the kernel applying 5 to 7 % of a 3 to 4 m
correction.

The fix follows the data (ADR-030): rank the candidates in the radius by Scan Context and seed
ICP at the candidate's place, let the loop scenes pass corrections up to 6 m, and drop the
kernel for loops that PCM corroborates with a nearby partner (section 15).

## 14. Loop-factor NEES and the covariance floor (H6, backlog C1-3)

NEES of a verified loop against the truth, in the chart of the loop factor (`decoupled`,
ADR-021): `e = [R_Z^T (p_true - p_Z), Log(R_Z^T R_true)]`, the factor's residual at the true
poses, `NEES = e^T C'^-1 e` with `C'` the floored covariance that enters the graph; 6 expected
(3 per block) for a consistent covariance (lessons 1.5). The truth of a recording is its
`gt.tum`, the `/ground_truth` OdometryPublisher of the body link `imu_link`, which is the
estimator's body frame `b`, so the true relative pose is `T_w_b(t_i)^-1 T_w_b(t_j)` with no
extrinsic. On every loop below the ICP covariance is below the floor in every direction (the
logged sigmas equal the floor), so `C'` is the floor itself and the NEES of another floor follows
from the same errors. Reproduce with `tools/loop_eval.py` (section 15 gives the exact commands).
Recordings: A = 2026-11-02, B = 2026-11-03 04:40 UTC, C = 2026-11-03 06:11 UTC (all seed 42; Gazebo
is not deterministic). For A and B the truth was the nearest `gt.tum` sample (5 ms apart, up to
about 2.5 mm at 1 m/s); from C on it is interpolated in time (positions linear, rotations SLERP,
at most 5 ms between samples, review R-1). With interpolation the keyframe-0 loops of E-07W drop
from 0.55 to 0.23 deg, so part of the earlier tail was the matching.

| scene (loops) | errors against the truth | NEES, 5 cm / 1 deg (shipped) | NEES, 2 cm / 0.25 deg |
| --- | --- | --- | --- |
| E-07W, recording C (16), truth interpolated | 0.2 to 3.2 cm, 0.02 to 0.23 deg | mean 0.05 | mean 0.37, max 3.3 |
| E-07, recording C, loop-scene settings (5), truth interpolated | 0.4 to 5.5 cm, 0.01 to 0.07 deg | 0.29 | 1.81, max 7.6 |
| E-07W, recording B (16), nearest truth sample | 0.1 to 4.0 cm, 0.01 to 0.57 deg | mean 0.11 | mean 0.95, max 8.1 |
| E-07, recording B (5), nearest truth sample | 0.2 to 3.7 cm, 0.01 to 0.11 deg | 0.20 | 1.28, max 3.5 |
| E-07 and E-07W, recording A (26), nearest truth sample | 0.1 to 5.0 cm, 0.00 to 0.55 deg | 0.12 to 0.28 | 0.67 to 2.1, max 8.2 |
| two-lap pillar hall, seeds 42 / 1 / 7 / 2024 (11; 10 with the new floor) | 0.8 to 5.8 cm, 0.01 to 0.44 deg | 0.56 | 4.42 |
| box world, whole estimator (7) | below 1 cm | 0.024 | 0.21 |

**This is a conservative floor, not a NEES-6 calibration.** The floor that would give NEES 6 on
the pooled Gazebo loops is about 1.1 cm / 0.08 deg, but the errors are heavy-tailed: on both
recordings the two loops to keyframes 0 and 1 of E-07W carry 0.21 to 0.57 deg (0.21 to 0.23 deg
with interpolated truth), against 0.01 to 0.11 deg for every other loop. They look systematic rather than ICP noise: the probable cause is the
one-sided submap at the first stored keyframe (`submap_keyframes` neighbours exist on one side
only, and those were taken during the static start), not verified; they are kept, not excluded.
The floor is set to the smallest round values for which (a) the mean NEES is below 6 on every
scene and (b) no measured loop exceeds the chi-square 0.95 quantile (12.6, 6 DoF): 2 cm / 0.25
deg. Criterion (b) is a choice for safety, not a consistency test (with 30 or more consistent
loops one or two would exceed 12.6), and it biases the floor loose: on E-07W the NEES is still
about 1 against 6, about 6 times too loose in variance. 1.5 cm / 0.15 deg already gives a largest
NEES of 19. The floor is fitted and evaluated on the same loops. Against 6, the shipped 5 cm /
1 deg is 30 to 55 times too loose in variance on the recordings (E-07W 6 / 0.11) and 11 times on
the hall. Owner ruling: the 2 cm / 0.25 deg floor is set in the loop overlays only
(`config/sim/*_e07_loops_on.yaml`); the default stays 5 cm / 1 deg.

Test: `LoopNees.SyntheticScenes` (both floors: mean NEES below 6 on the two synthetic scenes; the
shipped one below 1 on the hall, the scene floor above 2 there, so a looser covariance shows).

## 15. Drift of metres: place search, kernel release (ADR-030; backlog C1-1, C1-13, C1-31)

**Place search** (`loop_closure.place_search`, default off). The nearest `place_search_max_ranked`
(20) candidates within `search_radius_m` are ranked by Scan Context distance to the query, and
ICP gets a third seed, the query at the candidate's position with the Scan Context yaw,
`(Rz(psi_c + s* 2pi/S) R_tilt_q, p_w_c)`, but only when the most similar candidate is distinct:
its distance is at most `place_search_sc_ratio` (0.8) times the smallest among candidates at
least `place_search_min_separation_m` (2 m, beyond the neighbouring keyframes of one place) from
it. A look-alike elsewhere (a copy in a periodic aisle) fails that, and the search is the shipped
one: nearest first, graph seeds only. Each descriptor is computed once per search. A place-seed
fit beyond `max_correction_m` (yaw within the gate) cannot be the loop but competes in the
ambiguity check (without that, the 3 m gate let a 2.7 m false loop in on E-07). Tests:
`PlaceSearchFixture.*`; `L02_DifferentPlacesNoFalseLoopWithPlaceSearch` (6 of 36 accepted, all
true, as shipped); `NoFalseLoopAtAnyDriftWithPlaceSearch` (periods 2 and 2.5 m, drift 0 to 5 m:
0 accepted); `LongPeriodCopyRankedFirstNoFalseLoopWithLoopSceneSettings` (periods 3, 4, 5 m, the
copy first in the store and identical in Scan Context, drift 0 to 5 m: 0 false loops in 30 cases
with the loop-scene settings; the shipped detector accepts 7 false loops there, a limit older
than ADR-030, C1-36).

**Cost** (`LoopDetector.PlaceSearchCostIsBoundedOnADenseRevisit`, desktop WSL, 300 keyframes
within the radius: a robot turning in place). The Scan Context descriptor cache (reviews N-5,
N2-1) keeps each keyframe's descriptor: `map.max_keyframes` slots of `sc_rings x sc_sectors`
floats, about 4.8 MB at 1000 keyframes, plus the roll and pitch each was levelled with, reserved
in the constructor when place search is on; slot = id modulo the store bound; no allocation per
search. A descriptor is recomputed when the keyframe's roll and pitch moved by more than
`place_search_cache_tilt_deg` (0.1 deg) since it was levelled, so a graph correction that tilts
keyframes does not leave stale descriptors (`PlaceSearchCacheRefreshesOnTiltChange`: after a
2 deg tilt the warm ranking equals a fresh detector's bit for bit; 0.05 deg more keeps the
cache). The test counts descriptor computations: a later search computes only the query's, capped
or not. Per search, first / later (cache warm): shipped 21.4 / 22.9 ms,
place search capped at 20 candidates 56.3 / 45.3 ms, uncapped 130 / 62 ms. Before the cache the
capped search cost 48.5 ms every time and the uncapped one 126 ms. What is left over the shipped
search is the shift search per candidate (72k products) and the place seed's ICP. On E-07W with
place search on (three laps, the densest recording; recording C) p50 / p95 / max 5.2 / 8.9 /
34.4 ms against 5.2 / 8.6 / 22.2 ms without it. ADR-026's per-keyframe bound now reads:
`max_candidates` verifications and `place_search_max_ranked` Scan Context comparisons, each
descriptor computed once per keyframe (ADR-030).

**Place seed against an unvisited copy** (review N-4,
`LoopAliasing.OnlyTheCopyStoredNoFalseLoopWithLoopSceneSettings`): the copy one period away is
the only stored keyframe, so it counts as distinct and the place seed lands on it. A place-seed
fit is a loop only when its correction is within `icp_max_corr_dist_m` plus
`place_search_drift_ratio` (2 %) of the distance travelled from the candidate to the query;
beyond that it only competes. Without that bound the loop-scene settings accepted 7 false loops
in 30 cases there (periods 3 to 5 m, drift 0 to 5 m); with it 0 (the shipped detector: 6, C1-36).
A copy across the aisle cannot be built in these scenes (`scan_at` has no occlusion, so parallel
aisles see each other and their scans are mirror images); after a long lap the bound admits
metres, so an unvisited identical aisle is a known limit (C1-24, C1-36).

**Kernel release** (`loop_closure.kernel_release`, default off, needs PCM). A PCM-selected loop
enters without the Cauchy kernel only with a partner: another loop in the graph whose `i` and `j`
are within `kernel_release_window_keyframes` (20) of its own, whose candidate and query are each
at least `kernel_release_min_separation_m` (5 m) from its own (two places: a look-alike seen from
two nearby queries gives loops whose candidates lie within the submap span plus the ambiguity
reach, about 4 m), and both loops ask a correction within `kernel_release_drift_ratio` (2 %) of
the distance travelled from candidate to query. The partner is held by PCM (validated at start-up:
`pcm_max_loops > kernel_release_window_keyframes + 1`, review N-3). The 2 % is a scene value: 1.5
times the largest pipeline drift measured on E-07 (1.3 %, recording B without the level start);
raw wheel odometry drifts 0.87 % (E-07) and 1.74 % (E-07W). On a short lap it is the guard that
binds: on E-07W successive loops are already 5 m or more apart, so the separation rarely stops a
pair, and 2 % of a 24 m lap is 0.48 m, below any period-sized correction (ADR-030, C1-34, C1-36). A loop without one keeps Cauchy; a loop
whose partner leaves gets it back (remove and re-add in one update; a failed update drops both,
and the selector forgets them). Without the kernel a loop applies the Gaussian share
`sigma_chain^2 / (sigma_chain^2 + sigma_loop^2)` of a correction whatever its size
(`LoopKernelRelease.GaussianLoopAppliesLargeCorrections`, lap of section 7;
`BackendGraph.LoopFactorWithoutKernelAppliesTheGaussianShare`, closed form):

| chain per step | correction | Cauchy, 5 cm / 1 deg | no kernel, 5 cm / 1 deg | no kernel, 2 cm / 0.25 deg |
| --- | --- | --- | --- | --- |
| 2 cm / 0.15 deg | 0.5 / 1 / 2 / 3 m | 0.92 / 0.61 / 0.13 / 0.06 | 0.93 at every size | 0.99 |
| 5 mm / 0.0375 deg | 0.5 / 1 / 2 / 3 m | 0.10 / 0.03 / 0.007 / 0.003 | 0.48 | 0.84 |

Through the pipeline (box world, injected loops, its own release settings: the 2 m circle cannot
give 5 m of separation): two loops agreeing on a 1 m world shift leave 0.19 m of residual
released against 0.88 m with Cauchy (`KernelReleaseAppliesACorroboratedCorrection`); a lone loop
is bit-identical to the flag off; an outvoted wrong loop is removed before it is released
(`KernelReleaseOutvotedWrongLoopNeverReleased`); the restore path (`KernelReleaseRestoresTheKernel
WhenThePartnerLeaves`) and a failed update after a release (`KernelReleaseFailedUpdateDropsThe
ReleasedPair`) are covered. **Consistent wrong pair** (`LoopKnownLimits.KernelReleaseConsistent
WrongPair`, review B-1): two loops with the same wrong world shift from one place, 5 keyframes
apart. With the loop-scene rule, or the separation alone, neither is released and the run is
bit-identical to the flag off; the pair pulls the map 0.131 m (shift 1 m) and 0.046 m (3 m) as two
Cauchy loops, against 0.070 and 0.027 m for one such loop; without the separation they would be
released and pull 0.94 and 2.94 m.

**E-07 and E-07W** (recording C, 2026-11-03 06:11 UTC, level start on in both overlays;
measured on the review branch at ddc9682, before the tilt-keyed descriptor cache refresh (review N2-1); E-01..E-03 with the flags off were md5-checked against main at this branch's tip; E-07 and E-07W are to be re-run on the next recording (backlog C1-37);
ATE after SE(3) alignment with the number of pairs; `est.tum` is the online estimate, one pose per
scan; `kf_opt.tum` the stored keyframes at their optimised poses after the replay, the file E-07
and E-07W are scored on (owner ruling; E-01..E-03 stay on `est.tum`); return = the start revisited
at 394 s; final = the last pose; horizontal errors of the start-to-pose displacement in the start
frame; loops: searches / accepted / rejected / PCM-rejected / removed, released; latency per scan,
desktop WSL, two runs; CPU user time of the replay):

| run | ATE `kf_opt` [m] (pairs) | ATE `est` [m] (pairs) | return h `kf_opt` [m] | final h `kf_opt` [m] | loops | p50 / p95 / max [ms] | CPU [s] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| E-07 loops off | 1.016 (337) | 1.099 (4525) | 3.68 | 2.84 | - | 7.4 / 15.5 / 23.4 | 38.1 |
| E-07 shipped loop settings | 0.870 (337) | 1.070 (4525) | 3.17 | 2.44 | 315 / 2 / 3 / 0 / 0, 0 | 7.3 / 15.1 / 52.0 | 37.1 |
| E-07 loop-scene settings | 0.152 (337) | 1.001 (4525) | 0.042 | 0.17 | 312 / 5 / 0 / 0 / 0, 5 | 7.3 / 15.2 / 66.7 | 37.7 |
| E-07W loops off | 0.0192 (135) | 0.0163 (1708) | - | 0.009 | - | 5.6 / 9.8 / 18.2 | 10.1 |
| E-07W shipped loop settings | 0.0167 (135) | 0.0205 (1708) | - | 0.045 | 56 / 16 / 0 / 0 / 0, 0 | 5.2 / 8.5 / 24.0 | 9.5 |
| E-07W loop-scene settings | 0.0142 (135) | 0.0151 (1708) | - | 0.021 | 56 / 16 / 0 / 0 / 0, 15 | 5.2 / 8.6 / 22.2 | 9.6 |

Commands (WSL; `<rec>` the `tools/record_scenario.sh sim/scenarios/E-07.yaml <rec>` output, E-07W
alike with `warehouse_e07*.yaml`; every path an argument, results under `results/<run-id>/`):

```
python tools/bag_to_mssdata.py <rec>/bag results/<run-id>/data --origin-lla 21.0278 105.8342 12.0 \
    --pipeline-config config/sim/outdoor_e07.yaml
mss_bench config/default.yaml results/<run-id>/data --overlay config/sim/outdoor_e07_loops_on.yaml \
    --out results/<run-id>/on 2> results/<run-id>/on/bench.err
python tools/loop_eval.py --gt <rec>/gt.tum --run results/<run-id>/on --out results/<run-id>/on-eval \
    --revisit-after-s 100 --max-gt-gap-s 0.05 --floor-m 0.05 --floor-deg 1 --floor-m 0.02 --floor-deg 0.25
```

(`mkdir -p results/<run-id>/on` first, since the shell opens `bench.err` before the bench creates
the directory; loops off: `--overlay config/sim/outdoor_e07.yaml`; shipped loop settings: an
overlay that is `outdoor_e07.yaml` plus `loop_closure.enabled: true`; `PYTHONPATH=eval:sim:tools`.)

The two kinds of ATE are different sample sets (every scan, keyframes only); off against on is
like for like within each. Loop-scene settings: the `config/sim/*_e07_loops_on.yaml` overlays
(floor 2 cm / 0.25 deg, `kernel_release` with the default 5 m / 2 % rule; E-07 also
`place_search` and `max_correction_m` 6). No false loop in any run: every accepted loop is within
5.5 cm and 0.23 deg of the truth. This recording drifts 3.7 m at the return. On E-07 the first
loop (3.77 m asked) enters with the kernel and applies 6 %; the second (3.30 m asked, its
candidate and query 6 keyframes on, beyond 5 m) is its partner, and the residual falls from 3.30
to 0.03 m in that update. On recording B (3.1 m of drift): loops off / shipped / loop-scene
`kf_opt` ATE 0.885 / 0.732 / 0.161 m; each setting alone there: kernel release and the floor
without place search and with the 3 m gate, 2 loops released, 0.164 m; place search with the 6 m
gate without release, 5 loops, 0.423 m; level start off (4.1 m of drift), 1.185 -> 0.164 m. The online ATE moves less by construction: nearly 90 % of the run
precedes the first loop and `est.tum` is never rewritten.

Latency: the loop keyframes cost 66.7 to 68.1 ms at most on E-07 with the loop-scene settings
(recording C, two runs; 63.5 to 65 ms on B before the descriptor cache), 52 ms with the shipped
settings, 23 ms with loops off (the H5 recording had 74.5 ms shipped):
the place-search ICP runs and the update that moves every keyframe by metres (iSAM2 relinearises
nearly the whole graph). p50, p95 and CPU time move by under 2 %. Pi 5 is not measured (C1-5,
C1-35).

**Open, stated plainly.** Two wrong loops consistent with each other at two places at least 5 m
apart (a long periodic aisle that the ambiguity check misses) are still released (C1-34); on a
short indoor lap the drift ratio is the guard that binds (ADR-030), not the separation. A first
lone loop keeps the kernel and applies only a few percent until its partner arrives.
`max_correction_m` 6 is a scene value (C1-1). The online estimate after a large correction is
anchored to keyframe 0 only through its priors (C1-33).

## 16. `pcm_odometry_sigma_scale` 1.5 against 2 (backlog C1-27, closed)

Owner ruling: 1.5 is final. The limit it leaves is accepted: at claimed sigma x0.5 the white
part of a lap is covered only for cycles up to about 125 m (radius 20 m); on a 250 m lap 1 to 3
true loops are lost at 3 of 7 seeds (table below). The disabled radius-40 test is deleted; the
covered range stays pinned by `ThreeLapTrueLoopsAllStaySelected`. The measurement below was made
for that ruling (H6, recordings of 2026-11-02). Same binaries, only the scale changed (`config/default.yaml`
for the tests, the overlay for the recordings). Counts: accepted / rejected before PCM /
PCM-rejected / removed.

| probe | 1.5 (shipped) | 2 |
| --- | --- | --- |
| E-07, loop-scene settings (section 15) | 5 / 0 / 0 / 0, no false loop | `est.tum` identical (md5) |
| E-07, shipped loop settings | 0 / 8 / 0 / 0 | identical |
| E-07W, either settings | 16 / 0 / 0 / 0, no false loop | identical |
| three laps, claim x1, radius 6 to 40 m, 7 seeds, 12 true loops | all selected; largest pair chi2 5.0 to 12.3 | all selected; 3.6 to 7.6 |
| three laps, claim x0.5, radius 6 to 20 m | all selected; 12.0 to 18.5 | all selected; 8.6 to 12.4 |
| three laps, claim x0.5, radius 40 m (the deleted disabled test) | 3 of 7 seeds lose loops: 2024 3 PCM-rejected (chi2 24.5), 3 1 removed and 1 rejected (33.2), 31337 2 rejected (38.0) | 1 of 7: 31337 1 rejected (24.1); the others at most 21.3 |
| two laps, 4 seeds, claim x1 / x0.5 / x0.25 | the same loops and end-to-start as without PCM | identical |
| alias 2 m against true loops 10, 13, 30, 50 keyframes away (`YawDriftTermKeepsNearAliasesInconsistent`) | chi2 161.4 / 100.7 / 49.9 / 33.8: all inconsistent | 100.8 / 65.2 / 31.2 / 20.6: the farthest pair is consistent (the test fails) |
| known loops plus 3 aliased (`PcmRemovesAliasedLoopsKnownLoops`) | 1 removed, 3 rejected, none left in the graph | identical |
| detector aliasing probe (`LoopAliasing.*`, no PCM) | 0 false loops | identical |
| wrong loop (0.8 m) then two true ones, loosened chain, `kernel_release` with the box settings (0.5 m separation, ratio 1; `KernelReleaseOutvotedWrongLoopNeverReleased`, re-measured under the two-place rule) | wrong loop removed and never released; final pose 2.4 mm from the true-only run | wrong loop **kept and released**: final pose 0.53 m off (the test fails); with the loop-scene 5 m separation this box scene releases nothing |

Reading for the owner: 2 recovers most of the radius-40 limit (one loop at one seed instead of 6
loops at three seeds) and changes nothing on the recordings, but it weakens the pairwise test
against far aliases, and with ADR-030's kernel release it lets a wrong 0.8 m loop that a true
loop fails to contradict pull the map by half a metre. If 2 is chosen, kernel release needs a
stricter partner test than PCM's (for example its own chi-square on the pair, without the
inflation).
