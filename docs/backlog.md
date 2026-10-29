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
