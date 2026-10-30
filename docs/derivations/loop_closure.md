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
`BackendGraph.RemoveLoopFactor*`, `BackendIsam.B03_OutlierLoopRemovedLeavesNoDrag`.

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
wider kernel or switchable constraints / GNC, and a floor calibrated by NEES).

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
3 rejected, chi2 24.5; seed 3: 1 removed and 1 rejected, 33.2; seed 31337: 2 rejected, 38.0; the
largest pair chi2 as `DISABLED_ThreeLapRadius40HalfClaimAllStaySelected` prints it when run with
`--gtest_also_run_disabled_tests`) (`ThreeLapTrueLoopsAllStaySelected`). The disabled test is
re-enabled or deleted when the owner rules (backlog C1-27).
`s = 2` would cover it at the price of weaker aliasing rejection against far loops; the owner
decides after a manual test.

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
| three laps, radius 6 / 9 / 12 / 20 m, claimed sigma x1 and x0.5 (review B-1) | n/a | 9 of 9 true loops selected, none removed (before the drift term: 2 to 4 removed) |
| known loops + 3 aliased, one before any true loop | n/a | aliased one removed, others never selected, end-to-start 0.055 m (0.051 m without them; Cauchy only 0.077 m) |
| whole estimator, injected wrong loop then two true ones | n/a | wrong loop removed, first true one re-added, run ends as with the true loops only (`PipelinePcmRemovesAWrongLoopAndReAddsTheTrueOne`) |
| aliasing probe, P = 2 / 2.5 m, drift 0 to 2.5 m | false loops at large drift on the loose chain (drag 0.34 to 0.64 m) | every case ambiguous, nothing added (drag 0); the graph's covariance is not used, so chain stiffness plays no part |
| B-03 outlier loop | Cauchy: drag < 0.1 m | removed: drag < 1e-6 m (backend); PCM removes it once two true loops agree |

Whole estimator (desktop WSL, median of 5 interleaved runs, loops on; ADR-026 amendment for the
table): E-01 and E-07 (re-recorded after #41) give the same `est.tum` as before ADR-028 (PCM
selects every loop: 6 and 16, none removed, no false loop). With the latency bounds of section 10
the ambiguity check costs nothing measurable: max per scan on the loop keyframes 44.8 ms against
43.3 ms with the check off (E-01) and 66.5 against 73.8 ms (E-07); before the bounds it was 73 and
106 ms. Loops-on max against loops-off max: 1.37 x (E-01) and 1.29 x (E-07), as at e946ee9 (1.37,
1.37): the rest is the loop's own cost (map rebuild, iSAM2 updates; C1-15). PCM itself costs 0.08
to 0.12 ms per proposed loop (at most 0.28 ms with 64 loops held, 61 to 1000 keyframes), the
odometry prefix update 0.2 to 0.3 us per keyframe.
