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
| G1 | The default `GnssAligner` (flags of designs 1/2 off) aligns E-01 about 21° off at 13 s and reports no yaw sigma | design-2 review (#34) | First PR after slice B, or earlier if the slices need GNSS on. |
| G2 | Design 3: inflate each fix's covariance by the shift uncertainty; then decide the defaults of the design 1/2/2b/3 flags | `gnss_fusion_study.md` §13.6 | Measure it against `prealign_trackyaw`. |
| G3 | Design 2b: the front-end travel gate (number of scan-match constraints along the travel axis, at least 100) partly reverses ADR-023 decision 13 | ADR-023 amendment, study §16 | It needs the owner's confirmation and a check on a recorded Gazebo bag; compare it with the degeneracy module's flagged directions, and add a counter for blocked alignment (review F4). |
| G4 | Design 2b: the residual limits outside LiDAR-observed scenes (the tolerance envelope) | ADR-023, lessons §1.18 | GNSS drift ≤ 0.15 m per fix with stuck wheels and ≤ 0.10 m per fix parked without wheels, skid ≤ 5°. |
| G5 | First-yaw bias of 1.4–2.3° on gentle turns below the 45° gate: the mean heading is compared with the GNSS chord (0.52° on main) | 2b review F5 | Send gentle turns to the wheel path. |
| G9 | 2b gap bridging assumes an absolute odometer, but the pipeline integrates wheel speed: with stop-and-go and a silent driver, alignment drops from 99 to 85 of 100 | 2b review F1 | |
| G10 | 2b slow-segment extension: under realistic drift a slow robot aligns confidently wrong (0.05 m/s at 0.02 m/fix: 10 of 100 beyond 3σ) | 2b review F2 | Revert or gate it before turning the flag on. |
| G11 | 2b limits 1 and 4b were classified as "sensor set" without an oracle run; the IMU sees spin-up and rest | 2b review F3 | |
| G12 | 2b: four surviving mutants (`ext_path_ok`, carried `wheel_ok`, cross-chunk half mark, front-end position of `b` vs `o`) | 2b review F6 | |
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
| S2 | E-04 vertical drift; the E-01 vertical offset |
| S3 | Bench dump of the degeneracy state |
| S4 | `run_integration` variants |

## Map export (PR #38)

| # | Item | Severity |
|---|------|----------|
| M1 | A tiny `map_export.voxel_m` (e.g. 1e-20) passes validation (`>= 0`), and the int64 cast in `voxel_centroids` then overflows. Fix: set a minimum, e.g. 0 or at least 1 mm. | Important |
| M2 | A bad `--map-out` directory is only found after the full replay. Fix: check the path at start-up. | Important |
| M3 | `FollowsBackendCorrection` has no independent truth for the moved map, so it passes even with stale poses. | Important |
| M4 | Map export takes no lock. A 1 Hz export from the ROS node during `optimize()` becomes a data race once the ADR-005 threads exist. | Important |
| M5 | No-GTSAM builds cannot export, because the stub backend has no keyframe poses (needs an ADR-016 amendment). | Minor |
| M6 | Keyframes beyond `map.max_keyframes` are missing from the map; export cost at 1000 keyframes on the Pi 5 is not measured; export stats are not in `bench.json`. | Minor |
| M7 | A partial write leaves a truncated PCD (write to a temp file, then rename); `fopen(path.c_str())` is not portable to Windows; the counter types in `MapExportInfo` are mixed; doc comments name the wrong test file. | Minor |

## Map demo (A2)

| # | Item | Severity |
|---|------|----------|
| A1 | `run_map_demo.sh` treats `run_integration` exit 1 as success, so after a bench timeout it draws a partial `map.pcd` and prints OK | Important |
| A2 | `map_demo.json` has no scenario field and no gate verdict; E-04 output sits under the `run/E-01/` slot; the E-01 gate always "fails" on Gazebo bags (baselines are synthetic); the degeneracy-on config of E-04 cannot be selected | Important |
| A3 | One in four Gazebo recordings failed in the bridge step (cause unknown, log lost); 8513 ground-truth queries were clamped to the end pose in the next run | Important |
| A4 | E-04 map climbs about 1.4 m over 45 m (vertical drift, see S2); no side view in `map-png` | Important |
| A5 | Shell robustness: empty arrays under `set -u` on bash < 4.4, `--bag` not checked against the scenario, `out=/` becomes empty, GNU-only `date +%s.%N`, unnamed missing option value | Minor |
| A6 | `map-png`: the ascii PCD path is a per-row loop, fields with COUNT > 1 are not rejected, and the E-04 picture is mostly white space; README command block lost its line breaks | Minor |

## ROS 2 node (B1)

| # | Item | Severity |
|---|------|----------|
| R1 | The GNSS ENU origin is not range-checked: a first fix with `status >= 0` and a finite but impossible latitude (e.g. 1000) becomes the origin, against the `lla_to_enu` precondition, and every later fix maps to garbage ENU without an error. Check `abs(lat) <= 90` and `abs(lon) <= 180`, as the parameter path does (review I-1). | Important |
| R2 | `/mss/path` grows without bound and is republished in full on every estimate: about 18k poses (about 1.3 MB) at 10 Hz after 30 min, copied on the executor thread, delaying the IMU callbacks. Cap or decimate it, or publish a keyframe path at the map rate (review I-2). | Important |
| R3 | With `use_sim_time` rclcpp's TimeSource spawns its own clock thread (`NodeOptions::use_clock_thread`, default true). It never touches the Estimator, but ADR-024 says "one thread": note it in the ADR or set `use_clock_thread(false)` (review I-4). | Important |
| R4 | `on_lidar` accepts the stamp before converting the cloud, so a malformed cloud from a second publisher advances `last_stamp` and the real publisher's scan with an equal or earlier stamp is dropped as "not increasing". Accept the stamp only after a successful conversion (review I-5; seen in the shared-domain run). | Important |
| R5 | Bag loop or clock reset: the node only skips (ADR-024) until the stamps pass the old maximum, so after `ros2 bag play --loop` it stays idle. Add an opt-in reset (rebuild the Estimator, clear the path) on a backward jump above a threshold. | Important |
| R6 | No launch smoke test (play a tiny bag, check `/mss/path`): it needs the sim Python stack in the ROS image or a checked-in bag of about 3 s. clang-tidy was not run on the new files (the colcon build exports no `compile_commands.json`) (review M-7). | Important |
| R7 | `ros2 bag play --delay` (5 s default) is a guess: with 2 s the node lost the first second of a Gazebo bag. Start the player paused and resume once the node has matched its four subscriptions. | Important |
| R8 | `export_map` runs on the executor thread at 1 Hz and blocks the callbacks while it runs (1.1 ms at 90 keyframes; not measured at 1000 keyframes or on the Pi 5). Locking: see M4. | Important |
| R9 | `/mss/path` holds front-end poses as published, never the optimised ones; a keyframe path from `visit_map` poses would show the GNSS and loop corrections the map already shows. | Minor |
| R10 | Only the last estimate of a `run_once()` is published; two scans finished in one timer tick give one pose (not seen at 1x or 2x: 579 poses, as the bench). An estimate callback or queue from the estimator would publish every pose. | Minor |
| R11 | Body frame name: `default.yaml` names b `imu_link`, the node publishes `base_link`, and Gazebo bags hang `lidar_link` under `imu_link` (`body_frame:=imu_link` needed). Pick one name for the rig, or default `tf.body_frame` to `frames.body`. | Minor |
| R12 | Odometry twist: angular velocity is not published (the bias-corrected gyro would do) and the twist covariance is all zeros, which consumers such as robot_localization read as perfectly known; mark it unknown (review M-3). | Minor |
| R13 | `row_step` is ignored: an organized cloud with padded rows (`row_step > width * point_step`, `height > 1`) is read with the wrong stride. Bridge clouds are packed. Reject or honour it (review M-1). | Minor |
| R14 | The LiDAR and IMU `header.frame_id` are not checked against the configured frames (extrinsics come from YAML); log once on a mismatch (review M-2). | Minor |
| R15 | `to_msg_time` truncates seconds to `int32` without a check (stamps after 2038 or garbage); the ROS type limits it, add a comment or a check (review M-4). | Minor |
| R16 | Subscriptions are best effort; large clouds over a real network may need reliable QoS. Add a `qos_reliable` parameter if a driver needs it. | Minor |
| R17 | Test runs share DDS domain 0 with other agents' Gazebo (same topic names): the node received two interleaved streams. Scripts and the dev-machine notes should set their own `ROS_DOMAIN_ID`. | Minor |
| R18 | The node is not bit-for-bit reproducible by design (processing depends on arrival time against the timer); it matched `mss_bench` to 1e-8 m here, but that is not guaranteed under load. | Minor |
| R19 | The RViz screenshot from `xwd` has false colours (channel order of the XWD-to-PNG step); evidence quality only (review M-5). | Minor |
| R20 | PR size: about 1650 lines, four times the 400-line target; it splits into conversions plus tests, then node plus launch (review M-6). | Minor |
