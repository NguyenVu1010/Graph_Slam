# Vertical drift study: the E-04 climb and the E-01 vertical offset

Diagnosis for backlog S2 (A4, C1-4), on main `9171c48`. Symptoms: on Gazebo recordings the
E-04 corridor map climbs about 1.4 m over 45 m (A2 report), and E-01's ATE of about 9 cm is
almost all vertical, with 1 to 2 cm horizontal (`sim/README.md`, sensor ablation). Sections 1
to 4 are the diagnosis, one hypothesis at a time on the same recordings; section 5 is the fix
(ADR-027) evaluated; section 6 lists what is left.

## 1. Setup

- **Recordings.** `tools/record_scenario.sh` on this branch, 2026-10-30, three seeds per
  scenario: E-04 and E-01 with seeds 42, 7 and 2026, plus E-07 (three warehouse laps) with seed
  42. The scenario seed is the Gazebo seed, and Gazebo draws the IMU's constant biases from it
  once per run (profile sigma: acc 0.3 m/s^2, gyro 0.035 rad/s per axis, ADR-011). So the seed
  also selects the bias; E-01 and E-04 with the same seed get the same draw (seed 42:
  `ba` = (0.5885, 0.1564, 0.2360) m/s^2 in both). Every noise report passed.
- **Runs.** `mss_bench` (Release, GTSAM, `deterministic: true`) on `bag_to_mssdata.py` output
  with the scenario overlay (`config/sim/corridor.yaml`, `warehouse.yaml`,
  `warehouse_e07.yaml`) plus one change per row. Every bench exited 0. One change at a time,
  on the same bag, so each comparison is paired.
- **Metrics** (`tools/vertical_drift_study.py compare`):
  - `ez`: vertical error after aligning only the first estimate's position and yaw to the
    truth. SE(3)-aligned ATE hides a straight climb, because a rigid rotation absorbs it
    (E-04 seed 42: anchored end error 0.95 m, SE(3) vertical RMS 0.036 m).
  - Tilt error `c = up_true x up_est` in b, with `up = R_w_b^T e_z`. A pitch error shows in
    `c_y`, a roll error in `c_x`; the shipped initialiser predicts `c = (-b_y, b_x) / g` for a
    horizontal accelerometer bias `b`.
  - ATE: SE(3) Umeyama, split into horizontal and vertical RMS.
- **Oracles.**
  - `bias`: the true IMU biases, from the still start against the ground truth.
  - `debias`: the same dataset with the true bias subtracted from `imu.bin`.
  - `floor`: raw LiDAR points through the true body pose and the overlay's `T_b_l`.

## 2. The symptom, reproduced

| Run | true `ba` [m/s^2] | `(-b_y, b_x) / g` [mrad] | tilt `(c_x, c_y)` [mrad], run mean (E-01, E-07: first 10 s) | end `ez` [m] | ATE (h / v) [m] |
| --- | --- | --- | --- | --- | --- |
| E-04 seed 42 | 0.588, 0.156, 0.236 | -16.0, 60.0 | -13.9, 58.5 | +0.955 | 13.61 (13.61 / 0.036) |
| E-04 seed 7 | -0.188, 0.004, -0.036 | -0.4, -19.2 | +2.4, -18.6 | -0.299 | 13.84 (13.84 / 0.046) |
| E-04 seed 2026 | 0.471, -0.411, -0.007 | 41.9, 48.1 | 42.1, 47.1 | +0.909 | 12.89 (12.89 / 0.011) |
| E-01 seed 42 | as E-04 seed 42 | -16.0, 60.0 | -15.1, 61.2 | +0.050 (max 0.551) | 0.095 (0.013 / 0.094) |
| E-01 seed 7 | as E-04 seed 7 | -0.6, -19.4 | 0.6, -15.9 | +0.018 (max 0.145) | 0.035 (0.011 / 0.033) |
| E-01 seed 2026 | as E-04 seed 2026 | 41.8, 48.1 | 42.8, 50.4 | +0.041 (max 0.487) | 0.080 (0.015 / 0.079) |
| E-07 seed 42 | as E-04 seed 42 | -16.0, 60.0 | -16.0, 60.8 | +0.054 (max 0.537) | 0.087 (0.012 / 0.087) |

- On every recording the tilt error is the horizontal bias over g, within 3.5 mrad.
- On E-04 the tilt stays constant for the whole run (seed 42: 57.3 to 59.3 mrad at every 5 s
  sample). The height grows with the distance the *estimate* travels: 0.94 m over the first
  16 m of seed 42, about 58 mrad per metre. The horizontal error of 13 to 35 m is the known
  corridor degeneracy (section 6). That run's estimate stalls after 25 s, and the climb stops
  with it.
- On E-01 the tilt holds for the first straight (seed 42: `ez` 0.47 m at 20 s, 10 m along x).
  It collapses to 0 to 2 mrad at the first 90 degree turn (22 s). The rest of the lap carries
  the offset left in the map (2 to 5 cm at the end on the three seeds). This is the "+0.10 m in the first 10 s, decreasing
  later" pattern of `sim/README.md`.

## 3. Hypotheses, one at a time

End `ez` [m] on E-04, same bag per column:

| Change | seed 42 | seed 7 | seed 2026 |
| --- | --- | --- | --- |
| none (shipped overlay) | +0.955 | -0.299 | +0.909 |
| H1: static gravity factor off (`backend.static_gravity_factor: false`) | +0.955 | -0.297 | +0.827 |
| H2: degeneracy on (`corridor_degeneracy_on.yaml`) | -1.057 | -7.127 | +2.066 |
| H4: `T_b_l` z +5 cm in the pipeline only | +1.638 | -0.298 | +0.906 |
| H4: `T_b_l` pitch +0.5 deg in the pipeline only | +0.828 | -0.438 | +0.739 |
| H5: true z accelerometer bias removed (oracle) | +0.970 | -0.301 | +0.904 |
| H5: wider acc bias prior (`backend.prior_bias_acc_sigma_mps2` 0.1 -> 0.3) | +0.950 | -0.303 | +0.878 |
| H6: true gyro bias removed (oracle) | +0.955 | -0.299 | +0.909 |
| H6: true horizontal accelerometer bias removed (oracle) | -0.292 | -0.003 | +0.015 |
| H6: true accelerometer bias removed, all axes (oracle) | +0.032 | -0.032 | +0.016 |

The same ablation on E-01, ATE [m] (vertical RMS in brackets): shipped 0.095 (0.094) / 0.035
(0.033) / 0.080 (0.079) for seeds 42 / 7 / 2026. Gravity factor off: 0.095 / 0.034 / 0.080.
Bias prior 0.3: 0.080 / 0.031 / 0.080. `T_b_l` z +5 cm: 0.094 / 0.034 / 0.080. `T_b_l` pitch
+0.5 deg: 0.083 / 0.038 / 0.070. True z bias removed: 0.098 / 0.035 / 0.080. True gyro bias
removed: unchanged. True horizontal bias removed: 0.024 (0.021) / 0.013 (0.008) / 0.016
(0.012). All accelerometer bias removed: 0.015 (0.012) / 0.014 (0.009) / 0.016 (0.012).

- **H1, tilt or gravity anchor.** Yes, as the mechanism: the tilt error is the bias over g
  (section 2). The anchor itself is not at fault. With the ADR-020 factor off, the result is
  the same within chaos, because the initialiser's `R_w_b` comes from the same window mean.
  The factor ties tilt and horizontal bias in the combination the window measured (ADR-020).
  It cannot split them, and nothing else in a straight run can either.
- **H2, floor and ceiling under-constraint.** No. The LiDAR holds the robot on the map's
  floor, and that floor is tilted, because the map is built from keyframe 0 with the
  initialiser's tilt. More vertical structure would not change that. Degeneracy on makes the
  height worse on every seed (-1.06, -7.13, +2.07 m). Its "lifts z by about 1.1 m at
  start-up" (`config/default.yaml`) is the same tilt combined with the relaxed along-axis
  information (section 5 shows the level start removes most of it).
- **H3, LiDAR render or beam-angle bias.** No. Raw clouds through the *true* pose and the
  overlay `T_b_l` put the floor at z0 = +3.5 to +5.7 mm. The floor slope is 0.06 to 0.27 mrad
  along body x and -0.25 to +0.02 mrad along y. The ring medians are within 2.3 cm, with no
  trend over elevation (E-01 rings 0 to 7: +0.6, -0.1, +1.6, +2.8, -1.5, +7.2, -11.2,
  +20.8 mm, the outer rings with few floor points). A beam-angle error of 1 mrad would show a
  1 mrad slope.
- **H4, extrinsic or lever-arm z mismatch.** No. The same oracle bounds a pitch mismatch
  between Gazebo's mount and the overlay to below 0.3 mrad, against 19 to 60 mrad of tilt.
  `tools/gen_sim_assets.py` renders the SDF sensor pose from the same overlay. Deliberate
  mismatches move the result by less than the bias does (table rows; the seed-42 z +5 cm row
  is one where the corridor run did not stall, 0.39 m horizontal, so it travelled further and
  climbed more).
- **H5, accelerometer z bias or gravity magnitude.** No. Removing the true z bias changes
  nothing. Gazebo's world gravity is 9.80665 m/s^2 (both world SDFs), the same as
  `imu.gravity_mps2`. The ADR-020 factor uses only the direction, and the LiDAR observes
  height.
- **H6, the data's pointer: the horizontal accelerometer bias.** Yes. Removing it alone takes
  E-04's end error from 0.30 to 0.96 m down to 0.00 to 0.29 m. Removing all three axes takes
  it to 0.016 to 0.032 m, and E-01's ATE from 0.035 to 0.095 m down to 0.014 to 0.016 m. The
  seed-42 E-04 run with only the horizontal bias removed ends at -0.29 m with a -10 mrad tilt
  at the end; its z bias (0.236 m/s^2) is then left to a 0.1 m/s^2 prior. The gyro bias is
  estimated well already (no change).

## 4. Mechanism

The static window measures `f = ba - R_w_b^T g`. The shipped initialiser sets `ba = 0` and
takes roll and pitch from `f`. A horizontal bias `b` therefore becomes a tilt `(-b_y, b_x) / g`
(ADR-020, Context). Keyframe 0's rotation prior (1 deg) and the gravity factor are both
centred on that tilt. Keyframe 0's scan goes into the map with it, so the map's floor is
tilted by the same angle. From then on the LiDAR anchors the tilt to this map, and the IMU
sees no contradiction: on a straight run, a tilt and a horizontal bias are indistinguishable.
The estimate follows the tilted floor and climbs `|b| / g` per metre travelled. A turn rotates
`b` with the body but not the tilt, which separates the two. E-01's tilt collapses at its first
corner, but the map built before that corner keeps the offset that dominates its ATE.

The Gazebo IMU's bias of 0.3 m/s^2 per axis is the profile's (ADR-011). It is the size of an
uncalibrated MEMS part (MPU class: up to about 0.6 m/s^2), so the sim is right to have it. The
cause is in the estimator's initial split of `f`, not in the simulator.

## 5. Fix evaluated: level start (ADR-027, `imu.init_level_start`)

With the flag on, the initialiser takes the body as level at rest: `R_w_b = I`,
`ba = mean(f) + g_w`. The keyframe-0 priors then centre on the true split. Its residual
error is the floor slope and the IMU's mount tilt instead of `b / g`.

| Run | end `ez` [m] off -> on | max abs `ez` [m] off -> on | tilt mean `(c_x, c_y)` [mrad] on | ATE (v) [m] off -> on |
| --- | --- | --- | --- | --- |
| E-04 seed 42 | +0.955 -> +0.032 | 0.968 -> 0.039 | 2.9, 0.2 | 13.61 (0.036) -> 13.59 (0.041) |
| E-04 seed 7 | -0.299 -> -0.032 | 0.316 -> 0.116 | 5.9, -1.1 | 13.84 (0.046) -> 0.185 (0.063) |
| E-04 seed 2026 | +0.909 -> +0.001 | 0.927 -> 0.026 | 2.9, 0.2 | 12.89 (0.011) -> 13.94 (0.067) |
| E-04 degeneracy on, seed 42 | -1.057 -> -0.315 | 2.682 -> 0.549 | 0.3, -3.6 | 0.877 (0.750) -> 0.197 (0.082) |
| E-04 degeneracy on, seed 7 | -7.127 -> -0.052 | 15.95 -> 0.127 | 5.2, -1.0 | 3.786 (3.699) -> 0.166 (0.065) |
| E-04 degeneracy on, seed 2026 | +2.066 -> -0.138 | 2.440 -> 0.213 | -3.6, -1.7 | 0.427 (0.368) -> 0.182 (0.119) |
| E-01 seed 42 | +0.050 -> +0.039 | 0.551 -> 0.075 | 1.1, 1.5 | 0.095 (0.094) -> 0.015 (0.012) |
| E-01 seed 7 | +0.018 -> +0.025 | 0.145 -> 0.082 | 0.7, 1.6 | 0.035 (0.033) -> 0.014 (0.009) |
| E-01 seed 2026 | +0.041 -> +0.034 | 0.487 -> 0.082 | 0.3, 0.9 | 0.080 (0.079) -> 0.016 (0.012) |
| E-07 seed 42 (loops off) | +0.054 -> +0.035 | 0.537 -> 0.067 | -0.4, 0.5 | 0.087 (0.087) -> 0.015 (0.009) |
| E-07 seed 42 (loops on) | +0.074 -> +0.064 | 0.537 -> 0.067 | 0.2, 0.6 | 0.087 (0.086) -> 0.020 (0.014) |

- The level start matches the oracle: its end `ez` is within 1.5 cm of subtracting the true
  accelerometer bias on every E-04 and E-01 recording (section 3, last row; within 1 mm on
  five of the six). The
  remaining tilt is within 6 mrad, and the climb is gone.
- E-04's horizontal error is still the corridor's along-axis drift (degeneracy off). Seed 7
  stopped stalling with the flag on (13.8 -> 0.19 m); seed 2026 did not (12.9 -> 13.9 m). That
  is the chaotic part (lessons 1.15), not an effect of the flag.
- With degeneracy on, the start-up lift of metres is gone (max 0.13 to 0.55 m). Seed 42 still
  ends at -0.32 m (section 6).
- The gravity factor off together with the level start gives the same result (E-04 seeds 42,
  7, 2026: +0.032, -0.061, +0.013 m; E-01 ATE 0.012, 0.014, 0.016 m).
- Synthetic, flag on vs off:
  - box-world corridor component test with `b = (0.3, -0.1, 0)` m/s^2
    (`PipelineComponent.LevelStartRemovesBiasTilt`): end tilt (9.65, 30.14) -> (0.55, -0.18)
    mrad; max height error 0.238 -> 0.006 m.
  - 15-seed study, which has no IMU bias (`ImuStaticGravityRound2Seeds`, `MSS_IMU_R2_FIRST=10
    MSS_IMU_R2_LAST=10 MSS_IMU_R2_REF=4`): see the table below.
- Flag off: `est.tum` of E-01, E-02 and E-03 is md5-identical to main 9171c48
  (`43ee8422...`, `fd658717...`); E-01 ATE 0.002160 m, E-03 0.002174 m, every gate passes.

15-seed paired study (seeds 42 to 56, no IMU bias in the synthetic world, so the level start
can only show its side effects). The reference is the shipped form. Cells: median (max) |
wins / losses of the level start | 95 % bootstrap CI of the median paired difference
(level start minus shipped):

| Scene | shipped | level start | wins / losses | CI of median diff |
| --- | --- | --- | --- | --- |
| 60 s corridor + GNSS, ATE (m) | 1.99 (37.02) | 2.06 (33.47) | 10/5 | [-10.12, +4.35] |
| 60 s corridor, LiDAR + IMU, ATE (m) | 24.07 (279.66) | 25.59 (60.46) | 6/9 | [-16.17, +9.80] |
| box world LiDAR factor NEES (exp. 6) | 8.47 (12.49) | 8.73 (13.81) | 6/9 | [-0.12, +1.34] |
| 20 s circle, x free, abs end x error (m) | 2.09 (11.01) | 3.04 (12.09) | 7/8 | [-2.25, +3.93] |
| 20 s circle, max abs x error / sigma_DR | 7.13 (17.13) | 10.25 (20.70) | 4/11 | [-0.28, +3.85] |
| 20 s straight, x free, abs end x error (m) | 3.38 (8.18) | 1.77 (13.93) | 9/6 | [-3.07, +1.35] |
| 20 s straight, max abs x error / sigma_DR | 6.21 (13.27) | 7.00 (20.59) | 6/9 | [-0.94, +2.70] |

Every interval contains 0. In these scenes the true tilt is 0 and the window mean differs
from gravity only by noise (about 4 mm/s^2), so the two initialisations differ by about
0.4 mrad of tilt against bias. What moves the per-seed numbers is the chaos of the degenerate
corridors (lessons 1.12, 1.15), not a systematic effect. The fixed-seed component tests encode
single outcomes of that chaos: run with the flag on as the default, four of them fail
(`CorridorDriftCorrectedByGnss` seeds 43, 44; `PrealignFixesAnchorFrameAlongAxis` seed 42 by
0.006 m over its 2-sigma bound; `TrackYawAlignsCorridorFlipSeeds`, whose seeds 51 and 52 no
longer flip; `CorridorImuOnlyAlongAxisWithinDeadReckoning`). The box world, loop, map-export
and GNSS component suites pass. A default change has to re-pick those seeds or bounds in the
same baseline-update PR (backlog S5).

## 6. Not explained here (next to measure)

- **E-04 along-axis drift (13 to 35 m with degeneracy off).** It is the known corridor
  degeneracy and is separate from the vertical problem: removing the true bias leaves it
  unchanged on seeds 42 and 2026 (on seed 7 it went away with either change, which is chaos).
  With degeneracy on and the level start it is 0.14 to 0.18 m horizontal RMS. This is
  backlog D1/D2 and the degeneracy-on default.
- **E-04 degeneracy on, seed 42: -0.32 m at the end with the level start.** The tilt starts
  at 0 and drifts to -7.5 mrad by the end. The bias oracle with degeneracy on ends at -0.05,
  -0.05 and -0.16 m (seeds 42, 7, 2026; level start: -0.32, -0.05, -0.14 m). The two
  initialisations differ only by the window noise, so the seed-42 gap is the sensitivity of
  degeneracy-on runs, not bias. Next: log the tilt and the degenerate directions per keyframe
  (backlog S3).
- **About 3 to 4 cm residual height on E-01 and E-07 with the level start** (end `ez` +0.025
  to +0.039 m; max 0.08 m), at the first turn. Candidates: the remaining tilt of 1 to 3 mrad
  over the first 10 m straight; the map window (lessons 1.13).
- **The A2 figure of 1.4 m over 45 m** was read from map points in a 0.3 to 2.0 m height cut,
  which clips the climb. In the A2 run (seed 42, the same bias) the estimate did not stall and
  GNSS (sigma_v 3 m) was on; the seed-42 run here that did not stall (z +5 cm row) tilts by
  47 mrad on average and by 31 mrad at the end, climbing 1.64 m over 51 m. The mechanism is
  the same.
- **Real hardware.** The level start assumes a level floor and a level IMU mount. On the Pi
  robot, measure the MPU's mount tilt (or calibrate the accelerometer) before relying on it.
  The alternatives (a calibrated bias, a turn at start) are in ADR-027.

## Reproduce

```
tools/record_scenario.sh <scenario with seed S> results/<run>/rec/      # under flock
python tools/bag_to_mssdata.py results/<run>/rec/bag results/<run>/data \
    --origin-lla 21.0278 105.8342 12.0 --pipeline-config config/sim/corridor.yaml
mss_bench config/default.yaml results/<run>/data --overlay <overlay> --out results/<run>/<variant>
python tools/vertical_drift_study.py compare results/<run>/<variant>/est.tum results/<run>/data
python tools/vertical_drift_study.py bias results/<run>/data
python tools/vertical_drift_study.py debias results/<run>/data results/<run>/data_acc acc
python tools/vertical_drift_study.py floor results/<run>/data --pipeline-config config/sim/corridor.yaml
```

Overlays are the scenario overlay with one key changed (section 3 rows). The level start is
`imu: {init_level_start: true}`.
