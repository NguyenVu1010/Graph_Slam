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
  fused ATE is 0.51 / 0.59 / 1.70 m over seeds 42, 43, 44: better than raw GNSS on seeds 42 and
  43, 1.95x worse on seed 44. On main the 15-seed median is 3.47 m (4x worse, ADR-021 table).
  The defect exists on main, independent of PR 4b; it needs a diagnosis PR. PR 4b re-measured this corridor with its explicit variance
  (`degeneracy.md` section 7).
