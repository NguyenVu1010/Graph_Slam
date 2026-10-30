# Backlog

The project builds one end-to-end flow first:
- slice A: an offline map from a bag;
- slice B: the ROS 2 node and RViz, from a bag and then live in Gazebo;
- slice C: loop closure.

While those slices are built, a PR is merged once its blockers are fixed. A blocker is wrong output, a
crash, broken CI, or a change in behaviour with the feature flags off. Every other finding is listed
here and reviewed before each merge. After slice C, the hardening phase works through this list, and
every review finding is fixed before merge again.

Each item names where its numbers live.

## GNSS

| # | Item | Source | Notes |
|---|------|--------|-------|
| G1 | The default `GnssAligner` (flags of designs 1/2 off) aligns E-01 about 21° off at 13 s and reports no yaw sigma | design-2 review (#34) | Diagnosed and fixed behind `gnss.align_check_yaw_sigma` (ADR-025, study §17): fix noise on a 1 m spread, 2.6σ draw. The sigma is now always reported. Default: on, see G13. |
| G2 | Design 3: inflate each fix's covariance by the shift uncertainty; then decide the defaults of the design 1/2/2b/3 flags | `gnss_fusion_study.md` §13.6 | Measure it against `prealign_trackyaw`. |
| G3 | Done in H4 (PR pending, `fix/gnss-2b-gate`): the gate is the marginal information of the odometry frame's origin along the travel axis (`degeneracy::marginal_information`) against `degeneracy.eig_threshold`, which must be > 0 while the gate is on (owner ruling 2026-10-29, ADR-023 decisions 13 and 20); `gnss.align_yaw_frontend_min_info` removed with a migration message. Gazebo bags: E-01 203 to 415, E-07 212 to 413 (0 scans below 100), E-04 10 to 66 (all below), agreeing with the degeneracy flags on every scan. Front-end/wheel disagreements counted (`Stats::n_gnss_frontend_wheel_disagree`). | ADR-023 amendment 3, study §18.1 | |
| G4 | Design 2b: the residual limits outside LiDAR-observed scenes (the tolerance envelope) | ADR-023, lessons §1.18 | GNSS drift ≤ 0.15 m per fix with stuck wheels and ≤ 0.10 m per fix parked without wheels, skid ≤ 5°. |
| G5 | Done in H4 (PR pending, `fix/gnss-2b-turns`): turns above `gnss.align_yaw_path_min_turn_deg` (10) take the path of o; noise-free S-jog bias 2.2 deg -> at most 0.13 deg (0.1 rad/s; below 0.01 deg from 0.2 rad/s) at the same distance. | 2b review F5, study §18.6 | |
| G9 | Done in H4 (PR pending, `fix/gnss-2b-wheel-gap`): the pipeline bounds the travel it interpolates across a wheel-sample gap longer than `gnss.align_yaw_wheel_gap_periods` (3) measured nominal periods (larger end speed x the excess over one period; counted in `Stats::n_wheel_gap_bounded`), and the bound widens the wheel/GNSS test with the step's weights; stop-and-go with a silent driver 85 -> 100 of 100; 50 / 10 / 5 Hz wheel rates keep the stuck-wheel protection and the aisles distance (4.5 m). | 2b review F1, H4 review I-1, study §18.3 | |
| G10 | Done in H4 (PR pending, `fix/gnss-2b-drift`): coloured-drift term `gnss.align_yaw_drift_var_m2_per_s` (0.0017, the simulator's receiver) with fix times; 0.05 m/s at 0.02 m/fix 10 / 8 / 10 -> 0 / 1 / 1 of 100 beyond 3σ (three seed sets), still aligned; within the declared rate at most 1 of 100 in any cell. Drift above the declared rate stays outside (see G18); 0.02 m/s at 0.05 m/fix gets worse in count (4 / 1 / 3 -> 11 / 10 / 10, more runs align). | 2b review F2, H4 review m-5, study §18.5 | |
| G11 | Done in H4 (PR pending, `fix/gnss-2b-turns`): oracle on Gazebo bags. Transitions observable (IMU vs wheel delta-v within 0.01 m/s, rest spread 0.002 m/s); steady state needs a latch whose horizon the IMU bias handling sets (shipped init: broken at the first yaw; level start: tens of seconds). Reclassified; detector not implemented (needs G7 and S5). | 2b review F3, study §18.7 | |
| G12 | Done in H4 (PR pending, `fix/gnss-2b-wheel-gap`): the four mutants are killed (study §18.4); full mutation set re-run (study §18.9). | 2b review F6 | |
| G13 | Resolved (owner decision 2026-10-29, baseline-update PR, `docs/baselines/2026-10-29/`): `gnss.align_check_yaw_sigma` is on by default at 6 deg, shared with designs 2/2b (`gnss.align_yaw_max_sigma_deg`). At 6 deg E-01 still starts 15.4 deg off (2.6σ); a 10° first-factor target would need about 4.5° (18 s), 3° waits 34 s. E-01..E-03 ATE changed, baselines updated. | ADR-025, study §17.4 | Set `gnss.align_check_yaw_sigma: false` for the previous behaviour. |
| G14 | `GnssAligner`'s e-side spread check passes on fix noise alone (1.72 m against 1.01 m true on E-01); it only catches a frozen receiver | study §17.1 | Scale it by the declared sigma, or rely on the yaw check. |
| G15 | Done in H4 behind `gnss.align_yaw_sigma_frontend_cov` (off; PR pending, `fix/gnss-aligner-fecov`): the sigma adds sum s_i^2 / sum |a_c|^2, the bound for front-end errors of any correlation. Default: owner decision (G19). | ADR-025 amendment, study §18.2 | |
| G16 | The aligner's yaw sigma (both aligners) does not set the prior of `E` (30°) | ADR-025 decision 3, ADR-023 decision 7 | Needs a `graph.hpp` change and the owner's agreement. |
| G17 | G1 review minors: the linear small-noise assumption of the yaw sigma is not stated (holds on E-01, NEES 0.95 over 400 seeds); the 3-argument `update()` uses the 0.25 m² floor as fix variance, which halves the sigma for 1 m fixes (tests only); `E01LikeSigmaConsistent` has a two-run margin; "the drift cancels" is too strong (about 0.14 m of Gauss-Markov drift changes inside the window) | G1 review | |
| G18 | Decided (owner 2026-10-30): a per-receiver YAML value, default 0.0017 m^2/s, validated > 0 and finite, how to choose it in the config comment and `gnss.md` 5.3 (done in H4, `fix/gnss-2b-drift`). The YAML 0.0017 is the simulator's drift; at 0.05 m per fix (7x it) a robot at 0.05 m/s or faster still has 1 to 38 of 100 runs beyond 3σ (three seed sets), at 0.02 m/s 10 to 11 of the 26 to 35 that align; at the matched rate at most 1 of 100 everywhere, but a slow robot then aligns much later (0.05 m/s: 22 to 24 m) or not at all (0.02 m/s). | H4, study §18.5 | |
| G19 | Decided (owner 2026-10-30): `gnss.align_yaw_sigma_frontend_cov` (G15) stays off by default until design 2 is enabled. On, E-01..E-03 output changes and the corridor (front end dead-reckoned along the axis, 1000 m^2) no longer aligns with `GnssAligner` (numbers in study §18.2); a baseline-update PR. | H4 | |
| G20 | The H4 stationarity oracle (G11) shows a detector would remove limits 1 and 4b where the LiDAR is blind, within its latch horizon; it is G7 and needs the level start (S5) first. | H4, study §18.7 | |
| G21 | Design 2b: the front-end gate checks each scan's travel axis, not the lateral axis that a turning segment's front-end path also uses; a scene that observes the instantaneous axis but not the lateral one through a whole turn is not covered | H4 review m-3, re-review R-1 (Important); owner deferred 2026-10-30 | Fix: 2-D marginal of o's horizontal axes, `1 / lambda_max(U^T H^-1 U)` from the existing eigen-decomposition; count failing scans (`horizontal_unobserved_scans`); require the chunk flag on every branch that takes its direction from `g_fe`; known-answer test with a gentle turn and lateral drift (~100-150 lines). Must be done before 2b is enabled. |
| G24 | Design 2b: with `align_yaw_from_track` on, the drift term (G10), the wheel-gap bound (G9) and the gentle-turn path (G5) are always active and YAML cannot switch them off (G18 rule `> 0`, gap key `> 1`), so the previous 2b / design-2 behaviour cannot be reproduced (box world: design-2 keys align 2 s later, 0 factors instead of 8). Violates "flags default to the previous behaviour" for the parent-flag-on case. | H4 re-review R-2 (Important); owner deferred 2026-10-30 | Sub-flag `gnss.align_yaw_hardening_2b` (default false) that zeroes all three effects while keys stay validated; md5 test against A+C with the parent flag on. Must be done before 2b is enabled. |
| G25 | H4 re-review minors: (m-a) wheel nominal period is a running mean seeded by the first interval, a long first interval hides gaps for a while; (m-b) with 30 % wheel-message loss stuck wheels align 2-3/100 against 0 with the bound off, undocumented; (m-c) docs/YAML should say the gap bound covers only the excess over one period; (m-d) gap counter and log run with track-yaw off, contradicting the Stats doc; (m-e) no unit test for the migration error when the old key is in a base config; (m-f) ADR-023 header still says the contract change is additive only, decisions 17 and 19 not marked replaced. | H4 re-review; owner deferred 2026-10-30 | Fix with G21/G24 before 2b is enabled. |
| G22 | Decided (owner 2026-10-30): `gnss.align_yaw_frontend_min_info` stays removed; a config that sets it fails with a migration error naming `degeneracy.eig_threshold` (done in H4, `fix/gnss-2b-gate`) | H4 review I-3 | |
| G6 | GNSS Doppler velocity for the frame yaw: an ADR candidate | study §16 | Check whether the receiver outputs RMC/VTG or NAV-PVT velocity. |
| G7 | A shared stationarity detector (IMU + wheel + front end), used by GNSS and a possible zero-velocity factor | lessons §1.18 | Partly covered by 2b in scenes the LiDAR observes. |
| G8 | An "aligned" flag on the ENU output | study §12 | Planned for S2. |

## Degeneracy

| # | Item | Source |
|---|------|--------|
| D1 | PR 4b-2 (`feat/degeneracy-sigma`) is on hold until the map edge-leak fix | lessons §1.6 |
| D2 | Retune `unobserved_var_m2` toward 0.01 | 4b-1 (#29) |

## Simulation and evaluation (S2)

| # | Item |
|---|------|
| S1 | Outdoor and loop worlds, `testdata-v1`, WheelSlip |
| S2 | Diagnosed and fixed behind a flag (H2 PR `fix/vertical-drift`, `docs/derivations/vertical_drift_study.md`, ADR-027): the static window reads the Gazebo IMU's horizontal accelerometer bias (0.3 m/s^2 profile sigma) as a tilt `b / g` (19 to 60 mrad on seeds 7, 2026, 42), and the map is built tilted. `imu.init_level_start` (off) takes it out: E-04 end height error 0.30 to 0.96 m -> 0.00 to 0.03 m, E-01 ATE 0.035 to 0.095 m -> 0.014 to 0.016 m. LiDAR render, extrinsic, z bias, gravity magnitude and the gravity factor ruled out. Open: the default (S5); the E-04 along-axis drift is D1/D2, not this. |
| S5 | Owner decision: default of `imu.init_level_start` (ADR-027) or another source for the horizontal bias (calibrated bias, start turn). Turning it on changes E-01..E-03 (baseline-update PR) and moves the fixed-seed corridor component tests, which encode single chaotic outcomes. Full ctest with the flag on as the default (GTSAM, 554 tests) fails exactly 5, all to be updated in that PR: `Config.StaticGravityFactorKey` (asserts the shipped default is off), `PipelineComponent.CorridorDriftCorrectedByGnss`, `PipelineComponent.PrealignFixesAnchorFrameAlongAxis`, `PipelineComponent.TrackYawAlignsCorridorFlipSeeds`, `PipelineComponent.CorridorImuOnlyAlongAxisWithinDeadReckoning` (the 15-seed study shows no significant change, study section 5). `LevelStartRemovesBiasTilt` sets the flag in both arms and does not depend on the default. Measure the MPU mount tilt on the Pi robot first. |
| S6 | The pipeline's `backend.prior_bias_acc_sigma_mps2` (0.1) is tighter than the sim profile's `acc_bias_sigma_mps2` (0.3), and the ADR-020 rest gate's magnitude check uses it: on E-04 seed 42 `abs(abs(f) - g)` is 0.255 m/s^2 against a gate of about 0.30, so a z bias above about 0.3 m/s^2 gates the gravity factor. Widening the prior alone changes nothing measurable (study section 3). |
| S3 | Bench dump of the degeneracy state |
| S4 | `run_integration` variants |

## Map export (PR #38)

| # | Item | Severity |
|---|------|----------|
| M1 | Done in H1 (PR pending): `map_export.voxel_m` must be 0 (off) or >= 0.001 (1 mm), checked in `load_config` and guarded again in `voxel_centroids` (tiny edge unchanged, non-finite or huge points dropped) (a smaller edge overflows the int64 voxel index; 1 mm is far below any LiDAR map resolution). | Important |
| M2 | Done in H1 (PR pending): `mss_bench` checks `--map-out` (not a directory, parent exists, a probe file can be created) before the replay and exits 2. | Important |
| M3 | `FollowsBackendCorrection` has no independent truth for the moved map, so it passes even with stale poses. | Important |
| M4 | Map export takes no lock. A 1 Hz export from the ROS node during `optimize()` becomes a data race once the ADR-005 threads exist. | Important |
| M5 | No-GTSAM builds cannot export, because the stub backend has no keyframe poses (needs an ADR-016 amendment). | Minor |
| M6 | Keyframes beyond `map.max_keyframes` are missing from the map; export cost at 1000 keyframes on the Pi 5 is not measured; export stats are not in `bench.json`. | Minor |
| M7 | Done in H1 (PR pending): `write_pcd` writes `<path>.tmp` with `std::ofstream` (portable path), fsyncs (POSIX) and renames, with a documented guarantee and a fault-injection test of the cleanup; `MapExportInfo` counters are all `std::size_t`; doc comments name `test_map_export_component`. | Minor |

## Map demo (A2)

| # | Item | Severity |
|---|------|----------|
| A1 | Done in H1 (PR pending): `tools/map_demo_verdict.py` reads `integration.json`; `run_map_demo.sh` accepts exit 1 only when E-01's bench ran to the end (exit 0) with an ATE (gate fail on a Gazebo bag), and exits 2 on a bench failure or timeout. | Important |
| A2 | Partly done in H1: `map_demo.json` now has `scenario` and `gate` (verdict and note). Still open: E-04 output sits under the `run/E-01/` slot; the E-01 gate always "fails" on Gazebo bags (baselines are synthetic); the degeneracy-on config of E-04 cannot be selected. | Important |
| A3 | One in four Gazebo recordings failed in the bridge step (cause unknown, log lost); 8513 ground-truth queries were clamped to the end pose in the next run | Important |
| A4 | The climb is explained and fixed behind `imu.init_level_start` (S2, H2 PR `fix/vertical-drift`: horizontal accelerometer bias read as tilt, `abs(b) / g` per metre). Still open: no side view in `map-png`. | Important |
| A5 | Shell robustness: empty arrays under `set -u` on bash < 4.4, `--bag` not checked against the scenario, `out=/` becomes empty, GNU-only `date +%s.%N`, unnamed missing option value | Minor |
| A6 | `map-png`: the ascii PCD path is a per-row loop, fields with COUNT > 1 are not rejected, and the E-04 picture is mostly white space; README command block lost its line breaks | Minor |

## ROS 2 node (B1)

| # | Item | Severity |
|---|------|----------|
| R1 | Done in H1 (PR pending): `origin_from_fix` also requires `abs(lat) <= 90` and `abs(lon) <= 180`; an impossible first fix is skipped and counted in `gnss before origin`. | Important |
| R2 | `/mss/path` grows without bound and is republished in full on every estimate: about 18k poses (about 1.3 MB) at 10 Hz after 30 min, copied on the executor thread, delaying the IMU callbacks. Cap or decimate it, or publish a keyframe path at the map rate (review I-2). | Important |
| R3 | With `use_sim_time` rclcpp's TimeSource spawns its own clock thread (`NodeOptions::use_clock_thread`, default true). It never touches the Estimator, but ADR-024 says "one thread": note it in the ADR or set `use_clock_thread(false)` (review I-4). | Important |
| R4 | Done in H1 (PR pending): `StampGate` separates the check from the commit; `on_lidar` commits the stamp only after the cloud converted; `on_gnss` likewise commits only after the origin/range check. | Important |
| R5 | Bag loop or clock reset: the node only skips (ADR-024) until the stamps pass the old maximum, so after `ros2 bag play --loop` it stays idle. Add an opt-in reset (rebuild the Estimator, clear the path) on a backward jump above a threshold. | Important |
| R6 | No launch smoke test (play a tiny bag, check `/mss/path`): it needs the sim Python stack in the ROS image or a checked-in bag of about 3 s. clang-tidy was not run on the new files (the colcon build exports no `compile_commands.json`) (review M-7). | Important |
| R7 | `ros2 bag play --delay` (5 s default) is a guess: with 2 s the node lost the first second of a Gazebo bag. Start the player paused and resume once the node has matched its four subscriptions. | Important |
| R8 | `export_map` runs on the executor thread at 1 Hz and blocks the callbacks while it runs (1.1 ms at 90 keyframes; not measured at 1000 keyframes or on the Pi 5). Locking: see M4. | Important |
| R9 | `/mss/path` holds front-end poses as published, never the optimised ones; a keyframe path from `visit_map` poses would show the GNSS and loop corrections the map already shows. | Minor |
| R10 | Only the last estimate of a `run_once()` is published; two scans finished in one timer tick give one pose (not seen at 1x or 2x: 579 poses, as the bench). An estimate callback or queue from the estimator would publish every pose. | Minor |
| R11 | Done in B3: `tf.body_frame` defaults to `frames.body` (`imu_link`), synthetic bags use `imu_link`, both launches leave `body_frame` empty (ADR-024 amendment). | Minor |
| R12 | Odometry twist: angular velocity is not published (the bias-corrected gyro would do) and the twist covariance is all zeros, which consumers such as robot_localization read as perfectly known; mark it unknown (review M-3). | Minor |
| R13 | `row_step` is ignored: an organized cloud with padded rows (`row_step > width * point_step`, `height > 1`) is read with the wrong stride. Bridge clouds are packed. Reject or honour it (review M-1). | Minor |
| R14 | The LiDAR and IMU `header.frame_id` are not checked against the configured frames (extrinsics come from YAML); log once on a mismatch (review M-2). | Minor |
| R15 | `to_msg_time` truncates seconds to `int32` without a check (stamps after 2038 or garbage); the ROS type limits it, add a comment or a check (review M-4). | Minor |
| R16 | Subscriptions are best effort; large clouds over a real network may need reliable QoS. Add a `qos_reliable` parameter if a driver needs it. | Minor |
| R17 | Test runs share DDS domain 0 with other agents' Gazebo (same topic names): the node received two interleaved streams. Scripts and the dev-machine notes should set their own `ROS_DOMAIN_ID`. | Minor |
| R18 | The node is not bit-for-bit reproducible by design (processing depends on arrival time against the timer); it matched `mss_bench` to 1e-8 m here, but that is not guaranteed under load. | Minor |
| R19 | The RViz screenshot from `xwd` has false colours (channel order of the XWD-to-PNG step); evidence quality only (review M-5). | Minor |
| R20 | PR size: about 1650 lines, four times the 400-line target; it splits into conversions plus tests, then node plus launch (review M-6). | Minor |
| R21 | Done in H1 (PR pending): every GNSS fix is range-checked (`fix_in_range`), not only the origin; an impossible fix is skipped, counted in `convert_errors` and does not advance the stamp gate. | Minor |
| R22 | Done in H1 (PR pending): `test_mss_node` feeds the node callbacks directly (malformed cloud with a later stamp then a good one; impossible first GNSS fix then a good one; later impossible fix). | Minor |

## Live sim (B3)

| # | Item | Severity |
|---|------|----------|
| L1 | Handheld-rig sim variant: no wheels, 6-DoF walking motion (owner, 2026-10-29). | Important |
| L2 | GNSS lever arm of 0.2 m as the default in every scene (owner, 2026-10-29). | Important |
| L3 | Outdoor wheel slip, e.g. `WheelSlip` values for grass and wet patches in the `outdoor` world, still open from ADR-011 (owner, 2026-10-29). | Important |
| L4 | LiDAR model to be decided; the owner has only a 2D LiDAR today. 2D LiDAR support (scan type, matching, map) would need its own ADR (owner, 2026-10-29). | Important |
| L5 | No automated live test: CI has no Gazebo, so `check_launch.sh` only loads `mss_live.launch.py` (`--show-args`) and pytest covers the ROS-free core (`LiveNoise`). The rclpy wiring (`live_node`, `live_start`) and the launch run only by hand. | Important |
| L6 | Dropped messages are counted by comparing two logs (mss_node's exit summary against the noise node's `out` counters). Publish the counters (e.g. `/diagnostics`) so a script can check them. | Minor |
| L7 | Every rclpy node on sim time receives the 1 kHz `/clock`: `tf_static`, idle after start-up, uses 29-42 % of a core, the driver and `live_start` likewise. Lower the Gazebo clock rate for live runs or drop sim time where a node does not need it. | Minor |
| L8 | The noise node is one Python thread: 77-90 % of a core at E-01 rates, a scan 13-25 ms. A denser LiDAR or a second robot would fall behind; profile `process_scan` or move the cloud work off the callback thread. | Minor |
| L9 | The first scan can start before the first ground-truth sample: its early points are skewed with the clamped start pose (449 points in one of two runs, `gt clamped`). The robot is still then, so the effect is nil; offline has the same edge (A3). | Minor |
| L10 | The last scan of a run waits for ground truth past the final pause and is never published (one scan per run); `live_start` steps the paused world one `gz service` call per 1 ms until the robot exists. (The missing run-phase wall timeout is fixed in the B3 fix round: `--stall-timeout-s`.) | Minor |
| L11 | The Gazebo GUI screenshot is partly covered by the RViz window (no window manager control under WSLg); evidence only. | Minor |
| L12 | `core/io` `DatasetMeta::body_frame` still defaults to `base_link`; the converter writes the bag's own frame, so no output changes, but the default no longer matches the rig name. | Minor |
| L13 | PR size above the 400-line target (code, tests, launch and docs in one PR). | Minor |
| L14 | Fixed in the B3 fix round (review B-1). After a normal `exit_on_end:=true` shutdown (not only `headless:=false`, and not caused by `flock`), the real `gz sim -s` server outlived the launch at about 100 % of a core: launch signals only the `ruby gz` wrapper. The next run on the same gz partition and world name read the orphan's clock (about 62 s), logged 0.0 sim s and exited 0, and left a second orphan. Fix: `sim.launch.py` stops the gz processes below its own launch process on shutdown (no `pkill` by name), each live launch uses its own `GZ_PARTITION`, and `live_start` refuses a start clock not near 0 (I-1). | Blocker |
| L15 | Review I-1, fixed in the B3 fix round: `live_start` trusted any `/clock` and had no wall timeout in its run loop, so a stale or foreign Gazebo gave exit 0 with 0 s simulated and a frozen one waited forever. Now exit 2 on a start clock above `--max-start-clock-s` and on no progress for `--stall-timeout-s` (pytest `test_live_guard.py`). | Important |
| L16 | Review I-2: `mss_node`'s default TF child changed from `base_link` to `frames.body` (`imu_link`) for every launch, including `mss.launch.py` on the robot. A robot whose URDF publishes `base_link -> lidar_link` gets a split TF tree unless it sets `tf.body_frame:=base_link`. Documented in ADR-024; no repo test, bag or CI path expects `base_link`. | Important |
| L17 | Review I-3: synthetic bags now carry `imu_link`, so `run_integration.py`'s synthetic bags and their mssdata `index.json` `body_frame` differ from main. Core reads and writes `body_frame` as metadata only, so no ATE change is expected; the CI integration gate confirms it. | Important |
| L18 | Review m-1: shutdown tracebacks. `live_start` now also catches `ExternalShutdownException` (fix round), but `tf_static` and `driver` print `ExternalShutdownException` tracebacks on every shutdown, and Gazebo exits 139 on SIGINT. Log noise only. | Minor |
| L19 | Review m-2: ground-truth gaps are not counted. The GT subscription is best effort; a lost sample widens an interpolation interval for skew without any counter. Count GT gaps above two periods. | Minor |
| L20 | Review m-3: the `sim/README.md` live command lost its line continuation. Fixed in the fix round. | Minor |
| L21 | Review m-4: `live_start` waits for subscribers on the noisy and raw topics but not on `/ground_truth` for the noise node; it works because the noise node subscribes in its constructor. A lazy GT subscription would lose the first samples and shift `t0`. | Minor |

## Loop closure (C1)

ADR-026, `docs/derivations/loop_closure.md`; numbers in the C1 PR description.

| # | Item | Severity |
|---|------|----------|
| C1-1 | No global place search: a revisit is found only when the drift leaves the true place within `search_radius_m` (10 m) of the estimate, and closed only when the correction is within `max_correction_m` / `max_correction_deg` (3 m, 30°). Add a Scan Context ring-key search over the store (with its own false-loop evaluation) before long outdoor routes. | Important |
| C1-2 | The acceptance thresholds (`sc_max_distance` 0.4, `min_inlier_ratio` 0.6, `max_rms_m` 0.08, `min_hessian_eig` 20, correction gates) come from the synthetic box world, the pillar hall of the unit tests and one Gazebo warehouse bag (E-07, seed 42). Evaluate precision and recall over several seeds and worlds (S2 outdoor/loop worlds) and on real data. | Important |
| C1-3 | The loop covariance floor (0.05 m, 1°) is chosen, not calibrated: measure the loop factor NEES against ground truth (lessons §1.5) and set the floor from it. | Important |
| C1-4 | E-07 on the Gazebo warehouse shows almost no drift to remove (horizontal end-to-start 4 cm over 72 m with loops off). Its 8 cm ATE was the vertical offset of S2 (H2 PR `fix/vertical-drift`): on a re-recorded E-07 (seed 42, 2026-10-30) ATE 0.087 m -> 0.015 m with `imu.init_level_start` (vertical RMS 0.087 -> 0.009 m); horizontal end error 4.7 cm with loops off, 1.5 cm on, with or without the flag. A world or route where the front end drifts by decimetres is still needed to show the effect on a recording (the two-lap drift component test shows it on synthetic data: 1.21 m to 0.08 m). | Important |
| C1-5 | Loop detection runs on the estimator thread at keyframe time (ADR-026 decision 4). Pi 5 cost not measured; when the ADR-005 threads land, move it to the loop thread (input `KeyframeView`s, output `LoopConstraint`) with the store and graph locking that needs. | Important |
| C1-6 | `sc_height_offset_m` (0.05 m) depends on the body height above the ground (sim robot: 0.10 m). The handheld variant (platform backlog) needs its own value, or a ground estimate from the static window. | Minor |
| C1-7 | Each accepted loop forces a `VoxelMap` rebuild (ADR-016, about 35 ms at 50 keyframes on the desktop) and `isam_extra_updates` iSAM2 updates; `min_interval_keyframes` (5) bounds how often. Partial rebuild (only moved keyframes) is the ADR-016 open item. | Minor |
| C1-8 | Only the nearest candidate is verified (`max_candidates` 1); a near candidate that fails the Scan Context gate hides a farther good one until the next keyframe. | Minor |
| C1-9 | The stub (no-GTSAM) build has no keyframe poses, so loop closure is a no-op there by construction (documented, tested). | Minor |
| C1-10 | The loop log line (`loop closure accepted: ...`) is the only per-loop output; the E-07 false-loop check parses it. A structured loop list in `bench.json` (and a `/mss/loops` marker topic in the node) would be cleaner. | Minor |
| C1-11 | Live run with loops on (`mss_live.launch.py`, PR #41) and an RViz screenshot were not done in this PR. | Minor |
| C1-12 | **Done (fix/loop-aliasing, ADR-028 and the ADR-026 amendment):** ICP ambiguity check from shifted seeds (every exactly periodic place rejected, no false loop in the review probe at any drift or chain stiffness) and PCM selection with removal of loops that become inconsistent; the absolute graph-covariance gate is optional and off. Remainders: C1-23, C1-26, C1-30 (B-1, Blocker, owner decision). Review I-1: aliasing in repeated structure. Above half the period (P = 2 m: drift > 1 m) a false loop passes every gate and drags a loose chain 0.34–0.64 m (`LoopKnownLimits.PeriodicStructureAliasingAboveHalfPeriod`). Fix: a correction gate from the graph's relative covariance of (i, j) (Mahalanobis on the joint marginal) instead of the fixed 3 m / 30°, plus a second-best-fit check (second ICP basin or Scan Context second-best ratio). | Important |
| C1-13 | Review I-2: Cauchy with the 5 cm / 1° floor applies only 3–13 % of a true 0.5–2 m correction (stiff chain 10 / 3 / 1 % at 0.5 / 1 / 2 m, loose chain 13 % at 2 m; `LoopKnownLimits.RobustKernelAppliesLittleOfLargeCorrections`): loops close only while drift is small. Recalibrate kernel and floor (NEES, C1-3), gate first (C1-12) then a wider kernel or switchable constraints / GNC; add a component test with one late loop and > 0.5 m drift. | Important |
| C1-14 | Review I-3: E-07 is not met on Gazebo (end-to-start 0.078 m off, 0.081 m on). Re-record the E-07 bag after #41 (body frame `imu_link`; this bag was recorded at e5e8320) on a scene with real drift (C1-4), before quoting numbers as E-07. | Important |
| C1-15 | Review I-5: report max / p99 scan latency with the flag on (an accepted loop adds 2 iSAM2 updates with marginals and a forced rebuild, about 35 ms) and the Pi 5 numbers before enabling the flag (C1-5). | Important |
| C1-16 | **Done (fix/loop-aliasing):** the optional chi-square gate covers all six axes (`CorrectionGateCoversRollAndPitch`); with it off (shipped) a roll / pitch error is caught by fitness and by PCM against other loops. Review m-1: the correction gate checks position and yaw only; a converged result with a large roll / pitch error is caught only by fitness. | Minor |
| C1-17 | Review m-2: `load_config` range-checks `loop_closure.*` with the flag off, so a config that loaded on main (e.g. `search_radius_m: 0`, `enabled: false`) now fails; `config.hpp` says "validated only when enabled", contradicting `config.cpp`. Pick one. | Minor |
| C1-18 | Review m-3: `detect()` says "Allocation: none", but the candidate buffer grows with the store on the first calls after it grows (0 on a warm call). Fix the comment. | Minor |
| C1-19 | Review m-4: `min_hessian_eig` compares the smallest eigenvalue of a mixed-unit `[rho, phi]` Hessian (same convention as `degeneracy.eig_threshold`); note the scale dependence in the YAML comment. | Minor |
| C1-20 | Review m-5: = C1-8 (one nearest candidate hides a farther good one). | Minor |
| C1-21 | Review m-6: the PR is about 2000 lines against the 400-line guideline (split into two commits per the brief; reviewed as one squash). | Minor |
| C1-22 | Review m-7: `scan_context_distance` is O(S² R), 72k multiply-adds per candidate; use the ring-key / FFT variant if `max_candidates` grows. | Minor |
| C1-23 | Optional graph gate (`graph_covariance_gate`, off): when the drift is far outside the graph's relative covariance the copy is the plausible fit and a false loop is accepted; on the loose chain the remaining false loops drag the lap 0.34 to 0.64 m (drifts 1.3 to 2.5 m, both periods), on the stiff chain only the drift of one period passes (about 1.4 cm) (`LoopKnownLimits.GraphGateAliasingWhenDriftExceedsGraphCovariance`). And it drops every true loop on a moderately overconfident graph (two-lap, claimed sigma x0.5: 0 loops, 1.21 m; `GraphGateDropsTrueLoopsOnAnOverconfidentGraph`). Turning it on needs C1-3 (NEES calibration of the LiDAR and loop covariances) first; C1-13 (wider kernel) cannot close large corrections while it is on. | Important |
| C1-24 | The ambiguity check searches one direction only (the least-constrained horizontal axis of the ICP Hessian) and copies within about `ambiguity_seeds_per_side * ambiguity_seed_step_m` + 0.5 m (2.5 m) of the best fit. Repetition along the well-constrained axis (parallel aisles) or with a longer period is not searched; evaluate on S2 warehouse worlds with real rack spacing. | Minor |
| C1-25 | Without a graph covariance (detector alone) every exactly periodic place is rejected as ambiguous, and with one the recall in such places depends on the covariance; recall in repetitive scenes is not measured on Gazebo (C1-4, C1-14). | Minor |
| C1-26 | PCM (ADR-028) cannot tell which of two inconsistent loops is right without a third: the earlier one stays (ties keep the current selection). Aliasing consistent along a whole aisle (every loop shifted by the same period) is mutually consistent, so PCM keeps it; only the ambiguity check stops it. | Important |
| C1-27 | `pcm_odometry_sigma_scale` (1.5), `pcm_yaw_drift_deg_per_m` (0.2) and `pcm_chi2_gate` (22.46) are chosen from the synthetic two-lap and three-lap tests (the repo's odometry model: 0.1 deg yaw bias per 0.5 m step), not calibrated; 0.2 deg/m is large for real LiDAR odometry. Measure the front end's heading drift and step-noise consistency on recordings (with C1-3) and set both. Review H3 I-1 (owner decision, 1.5 or 2 after a manual test): when the owner rules, re-enable `LoopSelector.DISABLED_ThreeLapRadius40HalfClaimAllStaySelected` (scale 2) or delete it (1.5 kept, limit accepted). | Minor |
| C1-28 | **Done (fix/loop-aliasing, review H3 I-2):** the ambiguity check is bounded without the graph gate: fitness pre-screen of each shifted seed (`ambiguity_prescreen_ratio`), ICP iteration cap (`ambiguity_icp_max_iterations`), back-off after an ambiguous result (`ambiguity_backoff_keyframes`); E-01 / E-07 numbers in ADR-026. Pi 5 still not measured (C1-5, C1-15). | Minor |
| C1-29 | **Done (fix/loop-aliasing, review H3 m-1):** a failed update after a PCM change: the pipeline calls `LoopSelector::forget` for the loops `Graph::has_loop_factor` no longer holds; removals are queued into the next update and survive a failed one. | Minor |
| C1-30 | **Done (fix/loop-aliasing-b2, ADR-028 amendment C):** yaw-drift term in the PCM odometry covariance (`pcm_yaw_drift_deg_per_m` 0.2, `pcm_odometry_sigma_scale` 2 -> 1.5); three laps (7 seeds) keep every true loop at claimed sigma x1 up to a 250 m lap and at x0.5 up to 125 m (`ThreeLapTrueLoopsAllStaySelected`); x0.5 on a 250 m lap is the open I-1 limit (C1-27). Review H3 B-1: PCM rejected, and removed after commit, true loops whose cycle holds a long stretch of odometry with an unmodelled bias. | Blocker |
