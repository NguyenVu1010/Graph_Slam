# GNSS fusion study: why GNSS made the box world worse

Diagnosis for PR 4a (ADR-017 "Việc còn mở", first item), on main `906f57b`. Sections 1 to 6
are the diagnosis; section 7 evaluates the candidate fixes; section 9 gives the result of the
fix adopted (ADR-019, `backend_factors.md` section 6b).

## Setup

Synthetic box world of `tests/component/box_world.hpp` (`pipeline.md` section 7) with its
default parameters: 3 s static, 2 s ramp, 15 s cruise at 0.5 m/s on a circle of radius
2 m (yaw rate 0.25 rad/s), 199 scans, 200 Hz IMU, 5 Hz GNSS, fixed seeds (42, 43). The PR
brief asked for 60 s on a circle of R = 1.67 m (the integration data); the study uses the
box-world defaults of the component tests instead: 20 s, R = 2 m, about 85 fixes after the
first estimate. The 60 s evidence is E-01 (R = 1.67 m), which only compares the fixed `T_e_w`
with the frame variable (section 9). GNSS fixes are
`T_e_w (T_w_b p_b_g) + N(0, sigma^2 I)` with `cov = sigma^2 I`; `T_e_w` has yaw 30 deg.
Wheel odometry is off unless stated. GTSAM build, Release.

ATE is the RMS position error of every published estimate against the analytic truth in
w, without alignment (as in the component test). "ATE 4-DoF" is the RMS after the best
yaw + translation fit of the estimate to the truth (the aligner's closed form), so it
measures deformation only.

Driver: `PipelineComponent.GnssFusionStudy` in `tests/component/test_pipeline_component.cpp`,
skipped unless `MSS_GNSS_STUDY=1`. It regenerates every table of this file except the gate
rows of section 7 into `results/<run-id>/gnss_study.md` (`MSS_GNSS_STUDY_RUN_ID`, default
`gnss-study`). The perfect `T_e_w` and the per-factor logging go through the test-only seam
`detail::CoreTestHooks` (an observer of every LiDAR factor `(t_i, t_j, T_bi_bj, cov_b)`, GNSS
factor `(t_k, p_e_g', cov, T_e_w used)` and accepted fix). The run takes about 90 s.
Since PR 4a fix round 1 the pipeline also has the lock-out guard
`gnss.max_consecutive_rejects` (25 in round 1, 60 since round 2); it changes only corridor rows where the gate locked out
(noted in section 8).

NEES: for a LiDAR factor, `e = boxminus(T_bi_bj, T_true)` and NEES `e^T cov_b^-1 e`, expected
mean 6. For a GNSS factor, `r = p_e_g' - T_e_w (T_w_bk_true p_b_g)` and NEES `r^T cov^-1 r`,
expected mean 3, computed with the `T_e_w` the factor used ("used") and with the true one
("true").

## 1. ATE

| Variant | GNSS sigma (m) | factors | ATE (m) | ATE 4-DoF (m) | max yaw err (deg) |
| --- | --- | --- | --- | --- | --- |
| LiDAR + IMU | - | 0 | 0.0088 | 0.0038 | 0.027 |
| + wheel | - | 0 | 0.0079 | 0.0032 | 0.024 |
| + GNSS, aligner | 0.1 | 40 | 0.0256 | 0.0222 | 0.832 |
| + GNSS, aligner | 0.5 | 42 | 0.0322 | 0.0290 | 0.304 |
| + GNSS, aligner | 2.0 | 41 | 0.0101 | 0.0058 | 0.080 |
| + GNSS + wheel, aligner | 0.5 | 42 | 0.0329 | 0.0297 | 0.298 |
| + GNSS, perfect `T_e_w` | 0.1 | 82 | 0.0139 | 0.0089 | 0.379 |
| + GNSS, perfect `T_e_w` | 0.5 | 82 | 0.0609 | 0.0266 | 0.240 |
| + GNSS, perfect `T_e_w` | 2.0 | 83 | 0.0310 | 0.0146 | 0.076 |

With a perfect `T_e_w` the factors start at the first estimate (2.2 s), which gives about
82 factors instead of about 42.

## 2. Aligner `T_e_w` error

The aligner first returns a transform at 10.8 s, when both spreads reach 1.0 m RMS. After
that it re-estimates `T_e_w` from all pairs at every accepted fix, so it is not frozen.
Each GNSS factor, however, keeps the `T_e_w` of the moment it was added.

| GNSS sigma (m) | first: yaw err (deg), p err (m) | end: yaw err (deg), p err (m) | RMS yaw err of the `T_e_w` frozen in the factors (deg) |
| --- | --- | --- | --- |
| 0.1 | -0.066, 0.033 | 0.179, 0.030 | 0.213 |
| 0.5 | -0.549, 0.137 | 1.361, 0.129 | 1.338 |
| 2.0 | -2.359, 0.529 | 6.468, 0.517 | 6.238 |

The expected 1-sigma yaw error (`gnss.md` section 4, "Observability") is `sigma_yaw = sigma_h / (sqrt(n) spread)`.
At 0.5 m, with about 40 pairs and the 1.4 m RMS spread of the 2 m circle, this is
`0.5 / (6.3 * 1.4) = 0.056 rad = 3.2 deg`, so the measured 0.5 to 1.4 deg are typical.
Between the first and the last estimate the yaw moves by 1.9 deg.

## 3. Covariance consistency (mean NEES)

| Variant | LiDAR factors: NEES (exp. 6) | trans. err RMS / sqrt(tr cov_p) (m) | GNSS NEES used / true (exp. 3) |
| --- | --- | --- | --- |
| LiDAR + IMU | 27.98 | 0.0042 / 0.0019 | - |
| + GNSS 0.1, aligner | 486 | 0.0177 / 0.0020 | 2.24 / 2.33 |
| + GNSS 0.5, aligner | 534 | 0.0176 / 0.0020 | 2.38 / 2.46 |
| + GNSS 2.0, aligner | 159 | 0.0101 / 0.0019 | 2.28 / 2.36 |
| + GNSS 0.5, perfect | 344 | 0.0169 / 0.0022 | 2.57 / 2.57 |

- LiDAR: `cov_b` (from `cov = cov_scale sigma_r^2 H^-1`, `sigma_r >= 5 mm`) is about 4.7
  times too small in variance (NEES 28 where 6 is expected), as ADR-004 expects. A LiDAR
  factor's relative error is about 4 mm, while its covariance claims 2 mm.
- GNSS: NEES is 2.3 to 2.6 where 3 is expected, so the factor's `sigma^2 I` is right
  (slightly conservative, within sampling). The keyframe-time shift (`pipeline.md`
  section 5) adds no visible error: the "true" NEES matches the noise-only expectation
  within sampling.
- With GNSS on, the LiDAR factor error grows from 4 mm to 18 mm (NEES from 28 to 530). The
  LiDAR measurement `T_bi_bj = (T_w_bi^ref)^-1 T_w_bj^match` takes `ref` from the graph and
  `match` from the map. When GNSS moves the graph and the map is not rebuilt (below the
  ADR-016 thresholds), the two frames differ, and the difference enters the next LiDAR
  factor.

Applied in PR 4a fix round 1b: `scan_match.cov_scale` 4.7 in `config/default.yaml`. The
LiDAR-only NEES on the box world is then 7.8 (test `LidarFactorCovarianceConsistent` keeps
it in [3, 12]); results with it are in section 11.

## 4. Controls

| Variant (GNSS sigma 0.5 m) | ATE (m) | ATE 4-DoF (m) |
| --- | --- | --- |
| Default prior (0.1 m, 1 deg), aligner | 0.0322 | 0.0290 |
| Tight keyframe-0 prior (1 mm, 0.01 deg), LiDAR only | 0.0092 | 0.0036 |
| Tight prior, aligner | 0.0092 | 0.0036 |
| Tight prior, perfect `T_e_w` | 0.0091 | 0.0036 |
| `scan_match.cov_scale` 5 (LiDAR NEES consistent), LiDAR only | 0.0090 | 0.0039 |
| `cov_scale` 5, aligner | 0.0326 | 0.0292 |
| `cov_scale` 5, perfect `T_e_w` | 0.0607 | 0.0290 |
| `cov_scale` 5 + tight prior, aligner | 0.0092 | 0.0035 |
| map rebuilt at every keyframe (`rebuild_*` 1e-6), aligner | 0.0360 | 0.0314 |
| map rebuilt at every keyframe, perfect `T_e_w` | 0.0682 | 0.0356 |

- A consistent LiDAR covariance changes nothing, so covariance weighting is not the
  cause.
- Pinning the 4-DoF gauge of w (a tight prior on keyframe 0) removes all of the damage, with
  the aligner and with a perfect `T_e_w`. The damage is therefore a rigid pull of the whole
  trajectory in the four gauge directions (x, y, z, yaw), which the default prior leaves
  soft, together with the deformation it causes (4-DoF ATE 0.029 m without the tight prior).
- Rebuilding the map at every keyframe (22 rebuilds) removes the map/graph mismatch of
  section 3: the LiDAR NEES falls from 534 to 20. The deformation stays (4-DoF ATE 0.031 m),
  so the mismatch is not what deforms the trajectory: the GNSS pull itself bends the chain
  within the slack of the LiDAR factors (each about 4 mm, about 2 cm as a random walk over
  22 keyframes). The mismatch is a consequence of the ADR-016 thresholds (dated note there).

## 5. Expected optimum

With an unknown 4-DoF `T_e_w`, GNSS carries information only about the shape of the
trajectory and, through the prior on keyframe 0, about drift. It can help only when the
LiDAR chain drifts by more than the fixes can resolve, roughly `sigma_h / sqrt(n_fixes)`:

- Box world: `n` is about 85 and `sigma_h = 0.5` m, so `0.5 / sqrt(85) = 0.054` m per axis.
  The LiDAR deformation is 0.0038 m (4-DoF ATE), and 0.0088 m in w. GNSS is 6 to 14 times
  weaker than the LiDAR chain, so the optimum is the LiDAR-only estimate. At 0.1 m the bound
  is 0.011 m, still no better than LiDAR.
- With a perfect `T_e_w` the fixes do carry absolute information, but only to
  `sigma_h / sqrt(n)` = 0.054 m per axis. That matches the measured 0.036 m rigid shift and
  0.061 m ATE. The ground-truth origin (keyframe 0 at exactly 0) is better than that.

## 6. Cause of the 3.4x

1. The GNSS factors pull the 4-DoF gauge of w. The pull comes from the error of the `T_e_w`
   frozen in each factor. On a 2 m circle the aligner's yaw is poorly determined (1-sigma
   3 deg at 0.5 m), and each factor freezes the value of its moment, so the factor set is
   not zero-mean against the trajectory. With a perfect `T_e_w`, the pull is the mean of
   the fix noise instead. Either way the GNSS information is far weaker than the LiDAR
   chain's (section 5), so any pull is damage.
2. The pull also bends the trajectory within the slack of the LiDAR chain (4-DoF ATE 0.029
   m); pinning the gauge removes both. The map/graph mismatch (LiDAR factor errors 4x,
   3 to 4 rebuilds) comes with it but is not its cause: rebuilding at every keyframe removes
   the mismatch and keeps the deformation (section 4).
3. Ruled out: the GNSS covariance (NEES 2.4 where 3 is expected), the LiDAR covariance
   (4.7x optimistic, but making it consistent changes nothing), and the keyframe-time
   shift.

## 7. Candidate fixes and their evaluation

Two fixes follow from section 6. Both were implemented and measured (scratch build, same
seeds). "Corridor" is the drift scenario of section 8 (60 s, `relax_factor` 1e8).

**A. Alignment gate.** Withhold GNSS factors until `T_e_w` is well determined: an RMS
horizontal spread of at least `k * sigma_h_m` and a predicted yaw 1-sigma
`sigma_h / sqrt(sum |a_c,h|^2)` (2-D Procrustes, noise on the e side) of at most `y`. The
frozen `T_e_w` then differs little from the final one.

| Gate (k, y) | box 0.1 m | box 0.5 m | box 2.0 m | corridor (LiDAR only 26.1 m) |
| --- | --- | --- | --- | --- |
| none (main) | 0.0256 | 0.0322 | 0.0101 | 0.74 |
| (3, 0.5 deg) | - | - | - | 25.8 |
| (3, 3 deg) | - | - | - | 8.7 |
| (5, 1 deg) | - | - | - | 24.8 |
| (10, 1 deg) | 0.0256 | 0.0088 | 0.0088 | 25.3 |

ATE in m. These rows come from a prototype of the gate (fixed `T_e_w`, keys
`gnss.align_min_spread_sigma` and `gnss.align_max_yaw_sigma_deg` in `GnssAligner`) that was
not kept, so the committed study driver cannot regenerate them. The gate removes the damage only when it never opens (box world at 0.5 and 2 m);
at 0.1 m it opens and the damage stays (2.9x). In the corridor it delays the alignment until
the LiDAR + IMU chain has drifted by metres; the aligner then fits `T_e_w` to the drifted
trajectory, the chi-square gate rejects the later fixes, and GNSS no longer corrects
anything. Rejected.

**B. `T_e_w` as one graph variable (ADR-019).** Every GNSS factor shares the variable
`E = T_e_w`, initialised with the aligner's estimate, 4-DoF through a tight tilt prior. The
GNSS factor set can then no longer pull the gauge of w (a rigid offset is absorbed by `E`,
as the tight prior did in section 4) and no factor keeps a stale `T_e_w`. GNSS keeps only
the information it really has: the shape of the trajectory. Results in section 9. Adopted.

Also checked: the aligner keeps adding pairs while it has no transform, and re-estimates
`T_e_w` from all pairs at every fix (it is not frozen; now tested).

Out of scope at this point: the LiDAR covariance optimism (NEES 28), the degeneracy
covariance along an unobservable direction (section 8), and the map/graph mismatch after a
correction below the ADR-016 rebuild thresholds (a design consequence, section 4).

## 8. Where GNSS must help: a degenerate corridor

A fix must not simply switch GNSS off where it is needed, so the same box world was also
run as a corridor: half extents (60, 2.5, 1.5) m, yaw rate 0.0005 rad/s (almost straight),
60 s cruise (30 m), `preprocess.max_range_m` 10 (the end walls are out of range), and
`degeneracy.enabled` true. Along the corridor the LiDAR has no information, so the estimate
on that axis comes from the IMU.

| `degeneracy.relax_factor` | LiDAR only ATE (m) | + GNSS 0.5, aligner (m) | + GNSS 0.5, perfect `T_e_w` (m) |
| --- | --- | --- | --- |
| 10 (default) | 11.5 | 7.6 | 3.1 |
| 1e4 | 13.7 | 32.2 (22.5 with the lock-out guard) | 16.2 |
| 1e8 | 26.1 | 0.74 | 0.33 |

With the default `relax_factor` the LiDAR factor still claims about 2 mm along the
unobservable axis (`sqrt(tr cov_p)` 0.012 m against an error of 3 m, NEES 5e5), and no fix
can move it. Only when the covariance along the degenerate direction is really large (1e8)
does GNSS work as intended: 35x better. The degeneracy covariance belongs to its own
module (ADR-004, `degeneracy.md`). The corridor test of Task 3 uses 1e8 in its overlay.

## 9. Result of fix B (ADR-019)

Same scenarios and seeds, GTSAM build, `backend.gnss_frame_variable: true` (default).

| Scenario | LiDAR + IMU | + GNSS, fixed `T_e_w` (main) | + GNSS, frame variable |
| --- | --- | --- | --- |
| box, GNSS 0.1 m | 0.0088 | 0.0256 (2.9x) | 0.0088 (1.00x) |
| box, GNSS 0.5 m | 0.0088 | 0.0322 (3.7x) | 0.0089 (1.01x) |
| box, GNSS 2.0 m | 0.0088 | 0.0101 (1.15x) | 0.0089 (1.01x) |
| box, GNSS 0.5 m + wheel | 0.0079 (wheel) | 0.0329 | 0.0079 |
| corridor 20 s cruise | 3.04 | - | 2.01 (1.5x better) |
| corridor 30 s cruise | 7.27 | - | 2.53 (2.9x better) |
| corridor 60 s cruise | 26.1 | 0.74 (35x better) | 1.93 (13.5x better) |

ATE in m. On the box world GNSS no longer degrades the estimate at any sigma (no map rebuild
either), which is the optimum of section 5. In the corridor GNSS still corrects the drift
13.5x, but less than with the fixed `T_e_w` (0.74 m): there the fixed frame, fitted early
while the drift was still small, acted as an absolute anchor that the free frame does not
have; with the frame variable the absolute position along the corridor comes only from
keyframe 0 and the IMU chain before the first factor. The corridor numbers are sensitive to
the scenario (LiDAR only: 3 to 50 m depending on duration and
`backend.relinearize_threshold`), because the IMU-only axis there is itself poorly behaved;
the component test asserts only the 2x bound at 60 s. Without GTSAM (stub backend) GNSS adds
no factor and both scenarios equal LiDAR only.

## 10. Cost of the frame variable on a long run

`E` is connected to every keyframe that has a fix, so relinearising it can touch most of the
iSAM2 tree. `PipelineComponent.GnssFusionStudyLongRun` (`MSS_GNSS_STUDY_LONG=1`, about
150 s) replays a 10-minute box world (6049 scans, GNSS 0.5 m) and times the `run_once()` call
that processes each scan (keyframe scans include `optimize()`), desktop WSL:

| `T_e_w` | keyframes | GNSS factors | ATE (m) | p50 (ms) | p99 (ms) | max (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| fixed | 849 | 2820 | 0.0561 | 4.17 | 9.48 | 20.48 |
| frame variable | 859 | 2828 | 0.0081 | 4.44 | 9.32 | 20.27 |

No latency spike from `E`, so no separate relinearisation threshold for it. On the long run
the fixed `T_e_w` degrades the box world 7x, the frame variable does not.

## 11. Results with `scan_match.cov_scale` 4.7 (fix round 1b)

Regenerated with `GnssFusionStudy` after the covariance change and the lock-out guard
(`results/gnss-study-r1b`); sections 1 to 10 keep the numbers at `cov_scale` 1.0 that the
diagnosis was made with. ATE in m, LiDAR NEES in brackets.

| Box world | LiDAR + IMU | GNSS, fixed `T_e_w` | GNSS, perfect `T_e_w` | GNSS, frame variable |
| --- | --- | --- | --- | --- |
| GNSS 0.1 m | 0.0090 (7.8) | 0.0249 (97) | 0.0147 (56) | 0.0090 (7.8) |
| GNSS 0.5 m | 0.0090 (7.8) | 0.0326 (101) | 0.0611 (75) | 0.0090 (8.2) |
| GNSS 2.0 m | 0.0090 (7.8) | 0.0102 (30) | 0.0318 (40) | 0.0090 (8.6) |
| GNSS 0.5 m + wheel | 0.0054 (wheel only) | 0.0316 | - | 0.0054 |

The diagnosis is unchanged: with the fixed `T_e_w` the box world is still 2.8 to 3.6x worse,
the tight-prior control still removes the damage (0.0092 m), and the frame variable is
1.00x at every sigma.

| Corridor | LiDAR + IMU | fixed `T_e_w` | perfect `T_e_w` | frame variable |
| --- | --- | --- | --- | --- |
| 20 s cruise, relax 1e8 | 3.95 | 0.67 | 0.71 | 1.39 |
| 30 s cruise, relax 1e8 | 9.12 | 0.66 | 0.65 | 1.46 |
| 60 s cruise, relax 1e8 (test) | 15.36 | 0.67 | 0.51 | 1.82 (216 factors) |
| 60 s cruise, relax 1e4 | 15.70 | 0.50 | 0.33 | 27.8 |
| 60 s cruise, relax 10 (shipped) | 11.48 | 12.49 | 11.00 | 12.63 (1.10x, test) |

The corridor stays scenario-sensitive. The frame variable corrects the relax 1e8 corridor 8.4x
(the component test asserts >= 2x and more than 200 factors), but less than the fixed or the
perfect `T_e_w`, and at relax 1e4 it diverges (27.8 m, only 74 factors: the chi-square gate
locks out and the restarts do not recover the IMU-only chain). With the shipped
`relax_factor` 10 the LiDAR factor claims millimetres along the axis and no `T_e_w` source
helps; the frame variable stays within 1.2x of LiDAR only (component test). PR 4b replaces
`relax_factor` with an explicit variance along unobservable directions and will re-measure
this table.

Lock-out guard in the shipped-relax corridor with a 20 s outage (20 s to 40 s,
`CorridorOutageRealignsAfterRejects`, `gnss.max_consecutive_rejects` 60 = 12 s, longer than
ADR-011's 10 s multipath hold): without the guard no GNSS factor is added after the outage
(42 factors, 0 after); with it the aligner restarts once and 11 factors follow (53 factors),
ATE 12.85 m vs 12.97 m (printed, not asserted). The guard also fires without an outage in
this corridor (1 restart at 60, 2 at 25): the front end drifts along the axis while `P` stays
optimistic. With `relax_factor` 1e8 and `cov_scale` 4.7 the gate no longer locks out in the
outage (larger `P`), so the test uses the shipped relax.

## 12. Open items

- **In the corridor, fused LiDAR + GNSS is not reliably better than raw GNSS alone.** In the
  120 m corridor of section 8 (`CorridorDriftCorrectedByGnss`), raw GNSS at 0.5 m per axis is
  about 0.87 m 3D RMS. With the PR 4b "unobserved" covariance (1000 m^2 along the axis) the
  fused ATE is 0.51 / 0.59 / 1.70 m over seeds 42, 43, 44. Over seeds 42..56 the median is
  1.99 m, and four seeds are above 14 m (section 13).
  - Section 13 diagnoses this. The dominant cause is the GNSS frame `E`, whose initial value
    comes from front-end dead reckoning along the axis. A frame oracle alone gives 0.30 m, below
    raw GNSS on 13 of 15 seeds (CI excludes 0).
  - A secondary, independent cause is the shift of each fix to its keyframe with the front-end
    poses. Once the frame is right, removing it cuts the tail (max 4.09 to 0.35 m, CI excludes
    0).
  - No switch that could be shipped closes the gap, so nothing is fixed yet. Section 13.6
    lists the designs to build next, smallest first.
  - Design 1 (section 14, ADR-022, `gnss.attach_prealign_fixes`, off): the frame offset is
    fixed (median 0.08 m along the axis), the fused ATE is not yet (1.88 m median, CI includes
    0). Designs 2 and 3 are open.
- **With the shipped defaults (degeneracy off), this corridor never moves.** Scan matching
  succeeds on every scan, but the estimate stays at the start: one keyframe, no GNSS factor,
  17.35 m ATE (section 13.3, H3). The "3.47 m on main" quoted here before came from the ADR-021
  table, measured with degeneracy on before PR #29.

## 13. Corridor fusion diagnosis

Driver: `GnssCorridorStudy.Diagnosis` in `tests/component/test_gnss_corridor_study.cpp`.

- It is skipped unless `MSS_GNSS_CORRIDOR_STUDY=1` (GTSAM build), and takes about 17 min on the
  desktop WSL.
- It writes `results/<MSS_GNSS_CORRIDOR_RUN_ID or gnss-corridor-study>/gnss_corridor_study.md`,
  plus a `gnss_factors.csv` and a `keyframes.csv` per run.
- `MSS_GCS_VARIANTS`, `MSS_GCS_FIRST_SEED` and `MSS_GCS_LAST_SEED` select a subset.

Scene: `corridor_params()` (60 s cruise at 0.5 m/s, GNSS 0.5 m per axis at 5 Hz, no wheel),
seeds 42..56, GTSAM. Measured at a8a2bf9; the branch was rebased onto 9c7da91 (docs-only
difference). The runs are deterministic: three full runs gave identical per-seed lines.

The reference (`ref`) is the scene of `CorridorDriftCorrectedByGnss`: degeneracy on, absolute
detector, 1000 m^2 / 10 rad^2 along flagged directions. The other variants either change one
switch, or use the oracles of two test seams (named as such in 13.2):

- **`force_T_e_w` (the "aligner-frame oracle").** It puts the true `T_e_w` in four places: the
  centre of the `E` prior, the chi-square gate, the rotation of the shift, and the frozen
  factor frame when `gnss_frame_variable` is false. It also turns the aligner off.
- **`gnss_shift_oracle`.** It replaces only the displacement in w, which `collect()` still
  rotates into e with the aligner's `R_e_w`.

Metrics, all against the analytic truth without alignment:

- fused ATE of the published estimates, split along the corridor axis (w x), lateral (w y) and
  vertical (w z);
- raw GNSS RMS (3-D, and its component along w x) over the fixes after the IMU initialisation;
- the LiDAR-only ATE of the same data and degeneracy settings;
- GNSS factors, and fixes not accepted after the first accepted one (chi-square rejections,
  plus the not-aligned fixes after a realign);
- the aligner's `T_e_w` at the first and the last GNSS factor against the truth. This is the
  centre of the `E` prior, or, with `gnss_frame_variable` false, the frame the factors use;
- `E` at the end against its prior centre and against the true `T_e_w`, both as the x
  translation and yaw of `A^-1 E` in w ("n/a" without a frame variable). This x is the
  along-axis offset that `E` puts on the trajectory: the published along-axis error is about
  minus this x;
- the along-axis marginal sigma and NEES (1 DoF) of each optimised keyframe after the first
  GNSS factor;
- the NEES of each GNSS factor against the truth at its keyframe (expected 3);
- the error, along w x, of the shift of each fix to its keyframe (`pipeline.md` section 5):
  the factor residual against the truth minus the raw fix noise;
- scan-match failures (`Stats::n_match_failures`).

Paired statistics: median, wins / losses, and the 95 % bootstrap CI of the median per-seed
difference. The bootstrap uses 20000 resamples with `std::mt19937` seeded 7, and index
`rng() % n`, so the CI does not depend on the standard library. With n = 15, a CI bound within
about 0.1 m of 0 is not robust: the resampling RNG alone moved such bounds by up to 1 m in
review. Read those bounds as "about 0".

### 13.1 Reference, per seed

| seed | fused ATE | along x / lateral y / vertical z | raw GNSS 3D / x | LiDAR-only | GNSS factors (not accepted) | aligner yaw err at first factor (deg) | E - truth: x (m), yaw (deg) | end kf sigma x | kf NEES x (> 3 sigma) | GNSS factor NEES vs truth | shift error RMS / max along x |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | 0.51 | 0.39 / 0.003 / 0.32 | 0.83 / 0.49 | 35.1 | 273 (0) | +0.2 | +0.02, -0.1 | 0.45 | 0.4 (0 %) | 2.9 | 0.14 / 0.49 |
| 43 | 0.59 | 0.58 / 0.004 / 0.11 | 0.83 / 0.49 | 18.7 | 268 (2) | -5.8 | +0.49, -0.2 | 0.32 | 3.2 (0 %) | 2.8 | 0.09 / 0.36 |
| 44 | 1.70 | 1.68 / 0.054 / 0.26 | 0.88 / 0.49 | 19.4 | 260 (11) | -2.2 | -1.42, -0.5 | 0.52 | 6.6 (30 %) | 8.9 | 1.21 / 4.33 |
| 45 | 1.99 | 1.80 / 0.036 / 0.86 | 0.86 / 0.52 | 58.4 | 253 (19) | +1.3 | +0.11, -0.5 | 0.35 | 10.0 (15 %) | 8.4 | 1.20 / 3.76 |
| 46 | 1.24 | 1.21 / 0.005 / 0.27 | 0.87 / 0.47 | 18.1 | 265 (2) | +4.9 | +1.25, +0.4 | 0.46 | 7.2 (29 %) | 3.0 | 0.14 / 0.47 |
| 47 | 0.65 | 0.64 / 0.003 / 0.10 | 0.87 / 0.50 | 8.5 | 254 (5) | +2.9 | +0.43, -0.1 | 0.41 | 1.8 (0 %) | 3.0 | 0.19 / 0.94 |
| 48 | 3.76 | 3.75 / 0.010 / 0.32 | 0.87 / 0.49 | 24.1 | 254 (10) | -1.8 | +0.82, -0.5 | 0.34 | 44.9 (68 %) | 39.9 | 3.04 / 6.92 |
| 49 | 0.77 | 0.67 / 0.005 / 0.39 | 0.84 / 0.50 | 23.3 | 270 (1) | -3.5 | +0.52, +0.1 | 0.34 | 3.1 (4 %) | 3.0 | 0.21 / 0.71 |
| 50 | 3.20 | 2.95 / 0.052 / 1.24 | 0.88 / 0.53 | 40.1 | 224 (45) | +0.2 | +1.93, -0.3 | 0.39 | 19.3 (29 %) | 17.9 | 1.93 / 4.68 |
| 51 | 35.63 | 35.63 / 0.040 / 0.57 | 0.91 / 0.54 | 279.7 | 228 (21) | -178.9 | -2.83, -177.7 | 0.67 | 2621.7 (100 %) | 41.4 | 3.14 / 6.92 |
| 52 | 37.02 | 37.02 / 0.060 / 0.37 | 0.84 / 0.49 | 128.7 | 112 (128) | +131.8 | -1.82, -176.2 | 0.63 | 1690.1 (100 %) | 6.9 | 1.09 / 2.74 |
| 53 | 14.53 | 14.53 / 0.029 / 0.22 | 0.87 / 0.50 | 45.7 | 104 (86) | +4.2 | +6.34, +49.3 | 0.29 | 626.3 (100 %) | 50.7 | 3.52 / 7.19 |
| 54 | 27.70 | 27.69 / 0.234 / 0.66 | 0.86 / 0.49 | 12.3 | 160 (116) | -1.4 | +1.01, -176.9 | 0.88 | 4626.0 (86 %) | 21.6 | 2.14 / 4.86 |
| 55 | 3.91 | 2.79 / 0.085 / 2.74 | 0.84 / 0.45 | 54.2 | 231 (29) | -0.3 | +0.21, -0.7 | 0.57 | 6.5 (20 %) | 17.4 | 1.88 / 4.46 |
| 56 | 0.42 | 0.30 / 0.004 / 0.29 | 0.86 / 0.52 | 4.7 | 267 (1) | +3.3 | +0.04, +0.1 | 0.47 | 0.2 (0 %) | 2.9 | 0.12 / 0.57 |
| median | 1.99 | 1.80 / 0.029 / 0.32 | 0.86 / 0.49 | 24.1 | 254 (11) | | | 0.45 | 7.2 | 8.4 | 1.20 / 3.76 |

Along-axis RMS per 10 s window (median over seeds, m, windows from t0): 0.10 / 1.45 / 3.80 /
0.97 / 1.58 / 1.40 / 1.02. The error is along the axis (median lateral 3 cm, vertical 0.3 m).
The runs fall into two groups:

- Seeds 42, 43, 47, 49 and 56: shift errors of 0.1 to 0.2 m, GNSS factor NEES about 3, fused
  ATE 0.4 to 0.8 m, at or below raw GNSS.
- The other ten: shift errors of 1 to 3.5 m RMS (up to 7 m), GNSS factor NEES 7 to 51, and
  keyframe NEES along the axis 6 to 4600, so the estimate is over-confident along the axis. On
  seeds 51 to 54, `E` ends with its yaw 49 deg or about 180 deg from the truth, and the fused
  trajectory runs backwards along the corridor (15 to 37 m ATE). Section 13.3 (H2) gives the
  two origins of these flips.

The along-axis marginal sigma at the end is 0.3 to 0.9 m.

### 13.2 Ablations (15 paired seeds)

Cells: fused ATE in m, median (max) | wins / losses against `ref` | 95 % CI of the median paired
difference (variant - ref). The last column compares the same runs with the raw GNSS 3-D RMS of
each seed (variant - raw; negative means better than raw GNSS).

| variant | fused ATE vs ref | vs raw GNSS 3-D |
| --- | --- | --- |
| `ref` | 1.99 (37.0) | 5/10 \| [-0.22, +13.66] |
| H1 `gate_off`: `gnss.chi2_gate` 1e12 | 3.31 (36.2) \| 6/8 \| [-0.09, +0.52] | 5/10 \| [-0.20, +23.06] |
| H2 `frame_fixed`: `backend.gnss_frame_variable` false | 2.94 (52.8) \| 8/7 \| [-1.43, +1.85] | 5/10 \| [-0.29, +3.33] |
| H2 `frame_tight_pos`: `gnss_frame_pos_sigma_m` 0.01 | 1.73 (34.8) \| 12/3 \| [-2.26, -0.05] | 6/9 \| [-0.38, +2.12] |
| H2 `frame_truth_init`: aligner-frame oracle, `E` free (loose prior) | 0.56 (3.53) \| 12/3 \| [-11.25, -0.09] | 10/5 \| [-0.53, +2.01] |
| H2 `frame_truth_fixed`: aligner-frame oracle + `gnss_frame_variable` false | 0.30 (4.09) \| 15/0 \| [-14.35, -0.24] | 13/2 \| [-0.60, -0.20] |
| H3 `deg_off`: degeneracy off (shipped) | 17.35 (17.39) \| 3/12 \| [+2.82, +16.65] | 0/15 |
| H3 `deg_huge`: 1e6 m^2 along flagged directions | 2.45 (35.1) \| 7/8 \| [-1.94, +0.98] | 4/11 \| [+0.90, +2.81] |
| H3 `deg_var_shipped`: shipped 0.001 m^2 | 16.53 (21.4) \| 3/12 \| [+3.80, +14.85] | 0/15 |
| H5 `oracle_shift`: shift-displacement oracle | 1.44 (39.1) \| 8/7 \| [-0.97, +0.20] | 4/11 \| [-0.20, +1.77] |
| H5 + H2 `oracle_shift_truth_init` | 0.47 (1.88) \| 13/2 \| [-14.26, -0.18] | 12/3 \| [-0.56, -0.34] |
| H5 + H2 `oracle_shift_truth_fixed` | 0.21 (0.35) \| 15/0 \| [-14.39, -0.45] | 15/0 \| [-0.67, -0.62] |

The shift oracle paired directly against the same frame without it (fused ATE; emitted by the
driver):

| with the shift oracle | against | median (max) \| wins / losses \| CI |
| --- | --- | --- |
| `oracle_shift` | `ref` | 1.44 (39.1) \| 8/7 \| [-0.97, +0.20] |
| `oracle_shift_truth_init` | `frame_truth_init` | 0.47 (1.88) \| 11/4 \| [-1.48, +0.02]: not significant |
| `oracle_shift_truth_fixed` | `frame_truth_fixed` | 0.21 (0.35) \| 13/2 \| [-0.42, -0.05] |

Mechanism indicators per variant (medians over the 15 seeds unless counted; "n/a" means not
measured; `deg_off` adds no GNSS factor):

| variant | median shift error RMS (m) | seeds with GNSS NEES > 6 | median abs(E - truth x) (m) | seeds with abs(E yaw err) > 10 deg | seeds with abs(aligner yaw err) > 10 deg at the first / last factor | median not accepted |
| --- | --- | --- | --- | --- | --- | --- |
| ref | 1.20 | 9 | 0.82 | 4 | 2 / 2 | 11 |
| deg_huge | 1.52 | 11 | 0.64 | 1 | 1 / 1 | 12 |
| deg_var_shipped | 0.46 | 5 | 7.08 | 11 | 0 / 13 | 226 |
| gate_off | 0.30 | 7 | 0.80 | 4 | 2 / 4 | 0 |
| frame_fixed | 1.55 | 10 | n/a | n/a | 2 / 2 | 8 |
| frame_tight_pos | 1.20 | 8 | 0.13 | 2 | 2 / 1 | 12 |
| frame_truth_init | 0.14 | 5 | 0.26 | 0 | n/a (aligner off) | 4 |
| frame_truth_fixed | 0.10 | 2 | n/a | n/a | n/a (aligner off) | 7 |
| oracle_shift | 0.00 | 1 | 1.18 | 2 | 2 / 2 | 4 |
| oracle_shift_truth_fixed | 0.00 | 0 | n/a | n/a | n/a (aligner off) | 7 |
| oracle_shift_truth_init | 0.00 | 0 | 0.15 | 0 | n/a (aligner off) | 5 |

A prototype that is not kept, and that the driver cannot regenerate, dropped fixes older than
`s` before their keyframe instead of shifting them (`gnss.max_shift_s`). CIs from the earlier
bootstrap (2000 resamples):

| `s` | fused ATE vs ref |
| --- | --- |
| 0.25 s | 3.16 (48.2) \| 3/12 \| [+0.33, +2.45] (worse) |
| 0.5 s | 2.05 \| 5/10 \| [-1.00, +0.98] |
| 1.0 s | 2.20 \| 7/8 \| [-1.08, +1.48] |

Fewer but better fixes do not help while the frame is wrong.

### 13.3 Hypotheses

- **H1, chi-square rejections (lessons 1.2, 1.4): not the cause.** With the gate off the CI
  is [-0.09, +0.52]. Rejections (median 11 fixes; 128 on seed 52, with a realign) are a
  symptom of the bad runs: they follow the shift error and the wrong frame.

- **H2, the frame: the dominant cause.** The aligner-frame oracle alone closes the median gap.
  With the true frame frozen (`frame_truth_fixed`) the median is 0.30 m, better than raw GNSS
  on 13 of 15 seeds (CI [-0.60, -0.20]), which meets the acceptance rule of 13.6. With `E`
  free but started at the truth (`frame_truth_init`) the median is 0.56 m (CI against `ref`
  excludes 0). What the frame oracle does not remove is the tail (max 4.09 m), which comes from
  H5.
  - **Offset along the axis.** In `ref`, `E` ends 0.8 m (median, up to 6.3 m) from the truth
    along the axis, and the trajectory follows it. For example, on seed 55 with `oracle_shift`,
    `E` is off by +6.19 m and the last keyframe by -6.40 m. Along a straight corridor `E`
    absorbs any constant along-axis offset of the trajectory. The only absolute anchor is
    keyframe 0 plus the IMU-only chain up to the first GNSS factor (ADR-019, "Hệ quả").
  - **Initial yaw.** The aligner fits `T_e_w` to front-end positions, whose along-axis part is
    IMU dead reckoning. At the first GNSS factor its yaw is 0.2 to 5.8 deg off on 13 seeds.
  - **The 180 deg flips have two origins.** The "aligner yaw err at first factor" column of
    13.1, with `oracle_shift`, separates them:
    - *Initialisation flips (seeds 51, 52).* The aligner is already wrong at the first factor:
      -178.9 deg on seed 51, and +131.8 deg on seed 52, which also realigns once. The graph
      stays near this start. The flips remain under `oracle_shift`.
    - *Graph flips from a good start (seeds 53, 54).* The aligner starts within 4.2 deg (seed
      53) and 1.4 deg (seed 54). The graph then moves `E` by 49 deg (seed 53) and by about
      180 deg (seed 54). Both flips disappear under `oracle_shift` (final yaw error 0.2 and
      0.8 deg), so the shift error drives them.
    - A flipped solution costs only the IMU factors of the 2 s acceleration ramp: at constant
      speed, nothing else fixes the sign of the along-axis velocity.
  - **`frame_fixed` freezes the same aligner error.** The aligner frame frozen in each factor
    is -178.9 deg off from the first factor on seed 51 (27.7 m ATE), and -178.7 deg at the last
    factor on seed 52 (52.8 m).
  - **The shippable frame switches do not close the gap.** Freezing the aligner's frame
    (`frame_fixed`) changes nothing (CI contains 0). A tight translation prior centred on the
    aligner's first estimate helps a little (CI [-2.26, -0.05], median 1.73 m, still 2x raw
    GNSS), because that centre carries the same front-end drift.

- **H3, LiDAR information along the axis: not the cause at 1000 m^2.**
  - 1e6 m^2 gives the same result (CI [-1.94, +0.98]).
  - The shipped 0.001 m^2 holds the axis at the drifted map (16.5 m), and GNSS cannot correct
    it (section 11 and `CorridorDefaultDegeneracyGnssDoesNotDegrade`).
  - With degeneracy off (the shipped default), every seed gives the same picture. Scan
    matching succeeds on every scan (0 match failures), but the estimate stays at the start:
    one keyframe, no GNSS factor, ATE 17.30 to 17.39 m, about 30 m / sqrt(3).
    - A plausible mechanism, not measured here: the small along-axis information of the edge
      neighbourhoods (lessons 1.6) pins the match to the first map, so the 1 m keyframe rule
      never fires. Measuring the Hessian eigenvalue along x would test it.
    - The brief's premise, "main defaults (degeneracy off), 15-seed median 3.47 m", therefore
      does not hold. That number is the ADR-021 `RelativeFactorSeeds` row, measured at
      db84bf9 with the then overlay: degeneracy on, `relax_factor` 1e8.

- **H4, the GNSS noise model: not the cause.** With both oracles but `E` free
  (`oracle_shift_truth_init`), the GNSS factor NEES against the truth is 2.6 to 3.0 (expected
  3) on every seed. So the factor's `sigma^2 I`, which equals the simulator's, is right.
  - `oracle_shift` alone is not a clean test: on seed 51 its NEES is 17.6, because `collect()`
    still rotates the exact displacement with the aligner's 179 deg-off `R_e_w`.
  - The lever arm `p_b_g` matches the simulator by construction (the overlay writes the
    dataset's value), so this NEES does not test it.

- **H5 (found in the data), the shift of each fix to its keyframe: secondary, independent of
  H2, and the source of the tail.**
  - Fixes wait for the next keyframe and are shifted to it with the front-end displacement
    (`pipeline.md` section 5, "cov unchanged"). Along an axis that only the IMU observes, that
    displacement carries the front-end velocity error times the gap.
  - The gap itself grows when the front end underestimates the speed, because the 1 m keyframe
    rule then fires late. Seed 44 has gaps of 6.6 s, with 33 fixes shifted by up to 4.3 m
    (`keyframes.csv`, `gnss_factors.csv`).
  - The factor still claims 0.5 m, so the corrupted fixes pull the keyframe velocity, which
    makes the next shifts worse. They also drive the graph flips of H2.
  - Paired against the same frame without it, the exact shift is significant with the true
    frame frozen: 0.30 to 0.21 m, max 4.09 to 0.35 m, 13/2, CI [-0.42, -0.05]. It is not
    significant with `E` free (11/4, CI [-1.48, +0.02]) or with the aligner's frame (CI
    [-0.97, +0.20]).
  - Even under `frame_truth_init`, seeds 44, 46, 51, 52 and 53 keep shift errors of 1.5 to
    3.2 m RMS and GNSS NEES 12 to 43.

### 13.4 Cause

There is one root, the along-axis dead reckoning of the front end, and it is used in two
places:

1. **Dominant.** The frame (`E` and its aligner prior) is initialised from it, and along the
   axis it is anchored only by it. This gives a wrong offset, and on 2 of 15 seeds an
   initialisation flip of the yaw. A frame oracle alone gives 0.30 m, 13/2 better than raw
   GNSS.
2. **Secondary.** Each fix is shifted to its keyframe with it, and the factor's covariance does
   not include that displacement's error. This produces the tail and the graph flips. Given
   the true frame, an exact shift brings the worst seed from 4.09 to 0.35 m (CI excludes 0),
   and every seed ends better than raw GNSS (0.21 m median).

### 13.5 No fix in this PR

The brief allows a fix only when a hypothesis that could be shipped explains the gap with a CI
that excludes 0. Only the oracles do. Of the switches that could be shipped, four do nothing
(gate off, frame fixed, 1e6 m^2, dropping old fixes) and one helps only a little (tight
translation prior, still 2x raw GNSS). `CorridorDriftCorrectedByGnss` keeps its bounds.

Production behaviour is unchanged. The only core change is the study seam
`CoreTestHooks::gnss_shift_oracle`. It adds one null-pointer test per collected fix in
`GnssFusion::collect`, which runs per keyframe; this is the same pattern as `force_T_e_w`.
Against main 9c7da91:

- E-01..E-03 `est.tum` md5 are equal (`tools/run_integration.py`, all gates pass);
- E-01 bench, median of 5 interleaved runs: scan p50 / p95 4.36 / 5.86 ms vs 4.35 / 5.92 ms;
  replay wall 3.87 s vs 3.92 s; CPU 2.74 s vs 2.75 s.

### 13.6 What to build and measure next (smallest first)

Each item is a design change behind a YAML flag that defaults to the previous behaviour, with
an ADR. Measure it with this driver on the 15 seeds, against `ref` and the oracle rows. With
the flag off, the box world must be unchanged, E-01..E-03 `est.tum` byte-identical, and the
bench within noise.

1. **Frame offset: anchor `E` along the axis at start-up.** Local to `pipeline`. Done behind
   `gnss.attach_prealign_fixes` (ADR-022, section 14): target met (0.08 m), flag off.
   - Today the fixes received before the aligner returns a transform are dropped, including
     those from the static phase at keyframe 0. Once aligned, attach them as factors, each to
     its nearest keyframe in time and the static ones to keyframe 0. That ties the translation
     of `E` directly to keyframe 0.
   - Target: bring the median |`E` - truth| along x from 0.82 m (`ref`) to the
     `frame_truth_init` level (0.26 m).
2. **Frame yaw: an initialisation without the dead-reckoned axis.** Local to `pipeline` /
   `gnss`.
   - Use the direction of the GNSS track against the body heading (LiDAR + gyro) instead of
     front-end positions, and resolve the 180 deg ambiguity with the sign of the GNSS
     displacement.
   - Target: no seed with an aligner yaw error above 10 deg at the first factor (`ref`: 2 of
     15, seeds 51 and 52).
   - It cannot remove the graph flips of seeds 53 and 54, which start from a good prior: those
     need item 3. If item 3 leaves any, tie the yaw prior sigma of `E` to the alignment's
     predicted yaw sigma.
3. **Shift, smallest form: inflate the fix covariance by the shift uncertainty in
   `GnssFusion::collect`.** Behind a YAML flag in the `gnss` section; no `graph.hpp` change.
   - Add `J P_shift J^T`: the front-end velocity covariance times dt^2, plus the attitude
     term, as the chi-square gate already does with `P_w_b`.
   - Target: GNSS factor NEES above 6 on no seed (`ref`: 9 seeds); no graph flip from a good
     start (seeds 53, 54); combined with items 1 and 2, fused ATE at or below raw GNSS with a CI
     that excludes 0.
   - It needs a velocity covariance in the front end. If none is available without a backend
     API change, use a configured velocity sigma per unit time and state that in the ADR.
4. **Shift, full form, if item 3 is not enough.** Either a GNSS factor on (`X_k`, `V_k`, `E`)
   that uses the graph's keyframe velocity for the time offset, or attaching each fix to its
   nearest keyframe and re-expressing pending fixes after a keyframe correction. The factor
   changes `graph.hpp` (`add_gnss_factor`), which needs the owner's agreement. Target: shift
   error RMS below 0.2 m on every seed (`ref`: median 1.2 m).
5. **Acceptance for the set.** Fused ATE at or below raw GNSS with a CI that excludes 0, and a
   max below 1 m. For reference, `frame_truth_fixed` reaches 13/2 [-0.60, -0.20], max 4.09 m;
   `oracle_shift_truth_fixed` reaches 15/0, max 0.35 m.

## 14. Design 1: pre-alignment fixes as factors (ADR-022)

Section 13.6, item 1. `gnss.attach_prealign_fixes` (default off) keeps the fixes received before
the first alignment in a bounded buffer (`gnss.prealign_max_fixes`, 128) and adds them as
factors once the aligner has a transform:

- the fixes of the IMU initialisation window go to keyframe 0 unshifted (only when keyframe 0
  is the initialisation scan);
- the others go to the nearer keyframe in time, shifted with the front-end poses (the cause-2
  shift of section 13.4 still applies to them);
- the chi-square gate runs at release, with the current `T_e_w`;
- the aligner and the `E` prior are unchanged, and a realign does not reopen the buffer.

Details and alternatives: ADR-022.

Driver: `GnssCorridorStudy.Diagnosis` with `MSS_GCS_VARIANTS=prealign` (variant `prealign` =
`ref` + the flag), seeds 42..56, same scene and statistics as section 13. The `ref` per-seed
lines are identical to section 13.1, so with the flag off the corridor is unchanged.

### 14.1 Corridor, 15 paired seeds

| | `ref` | `prealign` | target / reference |
| --- | --- | --- | --- |
| median abs(`E` - truth x) at the end | 0.82 m | **0.08 m** | 0.26 m (`frame_truth_init`) |
| fused ATE median (max) | 1.99 (37.0) m | 1.88 (35.3) m | |
| paired vs `ref`: wins / losses, 95 % CI of the median difference | | 10/5, [-2.78, +0.61] m | |
| vs raw GNSS 3-D: better on, CI | 5/15, [-0.22, +13.66] m | 6/15, [-0.34, +2.19] m | `frame_truth_init`: 10/15 |
| seeds with abs(`E` yaw err) > 10 deg at the end | 4 (51, 52, 53, 54) | 2 (51, 52) | |
| seeds with abs(aligner yaw err) > 10 deg at the first factor | 2 (51, 52) | 2 (51, 52) | design 2 |
| GNSS factors, median | 254 | 301 | |
| median shift error RMS along x | 1.20 m | 1.19 m | |
| seeds with GNSS factor NEES > 6 | 9 | 8 | |
| along-x RMS per 10 s window, median | 0.10 / 1.45 / 3.80 / 0.97 / 1.58 / 1.40 / 1.02 | 0.10 / 0.66 / 0.68 / 0.86 / 0.93 / 0.95 / 0.72 | |

Per seed, fused ATE `ref` -> `prealign` (m): 42 0.51 -> 0.50, 43 0.59 -> 0.32, 44 1.70 -> 3.07,
45 1.99 -> 2.61, 46 1.24 -> 1.93, 47 0.65 -> 0.30, 48 3.76 -> 0.63, 49 0.77 -> 4.12, 50 3.20 ->
0.42, 51 35.6 -> 34.8, 52 37.0 -> 35.3, 53 14.5 -> 4.59, 54 27.7 -> 1.88, 55 3.91 -> 1.68, 56
0.42 -> 0.53.

Reading:

- **The frame target is met.** `E` ends within 0.08 m of the truth along the axis (median; max
  0.60 m on seed 51), better than the frame oracle with a loose prior (0.26 m). The static fixes
  and the early fixes of the ramp tie `E` to keyframe 0 by measurements.
- **The graph flips from a good start are gone.** Seeds 53 and 54 (section 13.3, H2) end within
  0.5 deg of the true yaw, and their ATE drops from 14.5 / 27.7 m to 4.6 / 1.9 m. The two
  initialisation flips (51, 52) remain: the aligner is unchanged, so they are design 2's.
- **Fused ATE does not improve significantly** (CI includes 0). The seeds that get worse (44,
  45, 46, 49; 49 from 0.77 to 4.12 m) are those whose shift error grows (1.4 to 3.2 m RMS, GNSS
  NEES 10 to 43): with `E` now anchored, the factors corrupted by the front-end shift (cause 2)
  pull the keyframes instead of the frame. That is design 3's target.
- Not better than raw GNSS (6/15). Acceptance of the set (13.6, item 5) needs designs 2 and 3.

### 14.2 Box world, 15 paired seeds

`GnssCorridorStudy.BoxWorldPrealignPairs` (`MSS_GNSS_BOX_PAIRS=1`): the default box world
(2 m circle, GNSS 0.5 m), seeds 42..56, flag off against on. About 52 pre-alignment fixes are
attached per run, none dropped.

| case | ATE median off / on | wins / losses (on vs off) | 95 % CI of the median difference |
| --- | --- | --- | --- |
| GNSS | 0.01019 / 0.01019 m | 9/6 | [-0.00005, +0.00002] m |
| GNSS + wheel | 0.00645 / 0.00640 m | 9/6 | [-0.00004, +0.00001] m |

Unchanged (CI within 0.05 mm of 0, both sides). The CI is from the 5-decimal per-seed values,
20000 resamples (Python `random.Random(7)`).

### 14.3 Integration and bench

- Flag off: E-01..E-03 `est.tum` md5 equal to main d09cbd3 (E-01 / E-02
  `d1b1e4ba9c87f38d39f4a27fb521081e`, E-03 `ca02a49fde59a5a99fbe73129c630226`); all gates pass.
- Flag on (E-01 has GNSS: `config/sim/warehouse.yaml` enables it): E-01 ATE 0.002198 m against
  0.002183 m off (limit 0.002621 m), 62 pre-alignment factors, 3 rejected by the chi-square gate
  at release, none dropped; E-03 0.002187 m against 0.002183 m.
- E-01 bench, median of 5 interleaved runs, scan p50 / p95, replay wall, CPU: main 4.49 / 6.08
  ms, 3.86 s, 2.82 s; branch off 4.45 / 5.93 ms, 3.74 s, 2.78 s; branch on 4.40 / 6.12 ms,
  3.79 s, 2.80 s. Within noise.

### 14.4 Ruling on the default

Both conditions of the brief for flipping hold: the box world is unchanged and the corridor
frame target is met. The flag still ships off: the corridor fused ATE is not significantly
better (CI [-2.78, +0.61] m), four seeds get worse by 0.6 to 3.3 m through the shift error, and
E-01 is 0.7 % worse. Recommendation: turn it on together with design 3 (shift covariance), which
targets exactly the seeds that lose here, and re-measure the set.
