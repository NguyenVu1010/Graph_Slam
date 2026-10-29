# multi-sensor-slam

LiDAR-inertial SLAM on a factor graph with GNSS and wheel-odometry factors. C++17 core
without ROS, a thin ROS 2 Jazzy adapter, Gazebo Harmonic simulation, and a latency
benchmark that runs on a Raspberry Pi 5.

Working rules: `CLAUDE.md`. Conventions: `docs/conventions.md`. Decisions: `docs/adr/`. Problems met and the rules they taught: `docs/lessons.md`.

## Layout

```
core/        C++ library, no ROS (colcon package mss_core)
ros2_node/   ROS 2 adapter (colcon package mss_ros2)
bench/       latency benchmark binary
tests/       unit, component, integration, python
sim/ eval/   Gazebo worlds; evaluation (TUM files only)
docs/        conventions, derivations, adr
docker/      deps image (GTSAM pinned) + multi-stage build
config/      default.yaml, the single source of tunables
```

## Build

Desktop or Pi5, no ROS:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release        # add -DMSS_WITH_GTSAM=OFF without GTSAM
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

WSL (dev machine, Ubuntu 24.04). `setup.sh` is idempotent and needs no sudo: if apt packages
are missing it prints the `sudo apt-get install` line to run, then builds Sophus and GTSAM into
`~/opt`. `build.sh` configures, builds and runs ctest; the last line is `BUILD OK: <dir>`.
Options: `--no-gtsam`, `--asan`, `--build-dir DIR`, `--target T` (builds and tests only `T`).
The default build dir is `~/mss-build/<branch>`.

```sh
tools/wsl/setup.sh
tools/wsl/build.sh --no-gtsam   # then without --no-gtsam for the full build
```

Latency benchmark and dataset replay (no ROS). Convert a bag once, then replay it:

```sh
python tools/bag_to_mssdata.py <bag> <out> --pipeline-config config/sim/warehouse.yaml
mss_bench <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR [--map-out FILE.pcd]
```

`--out` is required; `DIR` receives `est.tum` and `bench.json`, and a summary table goes to
stdout. `est.tum` has one line per scan that produced an estimate; there are none before the
IMU static initialisation, which needs `imu.init_window_s` of still IMU data before a scan.
Besides the record and drop counts, `bench.json` reports `n_keyframes`, `n_match_failures`,
`n_skipped_scans` (scans without an estimate), `n_uncovered_scans` (of those, no IMU up to
the scan end at end of data), `n_map_rebuilds`, `n_degenerate_keyframes` (keyframes whose LiDAR factor has a flagged
direction) and `n_degenerate_rot_keyframes` (of those, with a flagged rotation direction), `n_gnss_factors`, `n_odom_factors`,
`n_map_points` (VoxelMap points at the end) and `n_map_spacing_replaced` (points refreshed in
place by `map.min_point_dist_m`, summed over rebuilds; every rebuild re-inserts the keyframe
window, so it grows with `n_map_rebuilds` and is not a map-quality figure). Exit codes: 0 success; 2 usage, config, dataset or output error; 3 the replay finished
but some records were not processed (dropped or rejected, see `bench.json`). Format:
`docs/mssdata_format.md`, decision: `docs/adr/014-bench-dataset-format.md`.

`--map-out FILE.pcd` also writes the global map after the replay. The map is every keyframe
cloud in the keyframe store, placed at the backend's optimised pose at the end of the run, in
frame `w` and in metres. The file is binary PCD with `x y z` float32, which CloudCompare and
`pcl_viewer` open. `map_export.voxel_m` sets an optional centroid downsampling; the default 0
keeps every stored point, which the store already holds at `map.keyframe_voxel_m`. The point
count, export time and file size are printed. `est.tum` and `bench.json` do not change. Design
notes: ADR-016, amendment of 2026-10-29. Keyframes evicted by `map.max_keyframes` are not in the
map. The no-GTSAM build cannot export a map (exit 2).

Desktop with ROS 2 Jazzy (workspace contains this repo under `src/multi-sensor-slam`). colcon
stops at the root `CMakeLists.txt`, so name the three packages' paths (ADR-013):
`multi_sensor_slam` (root superbuild: core, tests, bench), `mss_core` (`core/`) and `mss_ros2`
(`ros2_node/`, finds `mss_core` through colcon's prefix path).

```sh
pkgs="src/multi-sensor-slam src/multi-sensor-slam/core src/multi-sensor-slam/ros2_node"
colcon build --paths $pkgs --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
ros2 launch mss_ros2 mss.launch.py   # config:=<file> overrides the config path
```

`mss_node` gets the path of `config/default.yaml` (installed to `share/mss_ros2/config`) as the
string parameter `config_path` (and an optional `config_overlay`) and logs `mss_node up; ...`
once. It subscribes to `/lidar/points` (bridged PointCloud2 with per-point `t` and `ring`),
`/imu/data`, `/gnss/fix` and `/wheel/odom`, and publishes `/mss/odometry`, `/mss/path`, TF
`map -> <frames.body>` (`imu_link`, the IMU) and `/mss/map` (the global map from `Estimator::export_map()`, 1 Hz,
transient local). Topics, frame names, QoS depths, timer periods and the GNSS ENU origin are
ROS parameters of the node; design and parameter list: ADR-024.

Play a bridged bag (the noise bridge's `bag/`, e.g. from `tools/record_scenario.sh` or
`tools/run_integration.py`'s `data/bridged/bag`) into the node, with RViz:

```sh
ros2 launch mss_ros2 mss_bag.launch.py bag:=<run>/bag rviz:=true \
  overlay:=install/mss_ros2/share/mss_ros2/config/sim/warehouse.yaml \
  origin_lla:="21.0278 105.8342 12.0"   # the scenario's origin_lla; omit to use the first fix
```

Other arguments: `rate:=1.0`, `config_path:=<default.yaml>`, `body_frame:=` (empty: the
config's `frames.body`, `imu_link`, the parent of `lidar_link` in the sim bags' `/tf_static`),
`delay_s:=5.0` (the node must have subscribed before play starts: with 2 s a Gazebo bag lost
its first second on a loaded desktop), `exit_on_end:=true` (stop when the bag ends).

Live from Gazebo instead of a bag (desktop with Gazebo Harmonic; `PYTHONPATH` must hold the
`sim/` Python dependencies, see `sim/README.md` "Live"):

```sh
ros2 launch mss_ros2 mss_live.launch.py scenario:=sim/scenarios/E-01.yaml rviz:=true
```

It starts Gazebo paused with the raw sensor topics under `/sim/raw`, the online noise node
(`mss_sim.live_node`, the offline bridge's models), `mss_node` with the scenario's overlay and
ENU origin, and RViz, then unpauses once every consumer has subscribed. Arguments: `rtf:=1.0`
(real-time factor), `headless:=true` (`false` opens the Gazebo GUI), `render_engine:=ogre2`,
`seed:=` (default: the scenario's), `assets_dir:=` (default `results/live/<id>/assets`),
`config_path:=`, `overlay:=` (default: the scenario's `pipeline_config`), `body_frame:=`,
`exit_on_end:=true` (shut down at `duration_s` + 2 s of sim time; by default the world pauses
there and everything stays up).

Docker: `docker/Dockerfile` (usage in its header) builds the same three packages, runs the
superbuild and `mss_ros2` tests (`colcon test --packages-select multi_sensor_slam mss_ros2`),
and installs the bench at `/ws/install/multi_sensor_slam/bin/mss_bench`; the image's default
command runs the launch file. CI then runs the launch check, which passes when the launch is
still up after 15 s and the startup line was printed, and the bag and live launch files load
(`--show-args`) (`tools/ci/check_launch.sh [INSTALL_DIR]`
also works on any machine with ROS 2 Jazzy):

```sh
docker run --rm -v "$PWD:$PWD" -w "$PWD" mss bash tools/ci/check_launch.sh
```

## Python tooling

```sh
pip install -e ".[dev,eval]"
ruff check . && ruff format .
pytest
```

Simulation noise layer (offline, no ROS; details in `sim/README.md` and `docs/adr/011-simulation.md`):

```sh
PYTHONPATH=sim python -m mss_sim bridge <raw_bag> sim/scenarios/E-01.yaml results/<run-id>   # raw bag -> noisy bag + gt.tum
python tools/mutate_bag.py <in_bag> <out_bag> --drop 0.05 --reorder 0.02                     # E-03 bag: packet loss, reordering
PYTHONPATH=eval python -m mss_eval.noise_report <bag_dir> sim/noise/<profile>.yaml --out results/<run-id>/noise  # declared vs measured noise
```

Gazebo simulation (desktop only; worlds, robot template, driver, `tools/record_scenario.sh`): `sim/README.md`, section Gazebo.

### Integration tests (E-01..E-03, PF-01)

`tools/run_integration.py` synthesises a bag, runs the noise bridge and `bag_to_mssdata.py`
(E-03: `mutate_bag.py` first), replays each dataset through `mss_bench` and scores `est.tum`
against the ground truth with `mss_eval.ate`, then compares with `eval/baselines.json`.

The data are a synthetic box circle, not the warehouse lap: a circle of radius
R = V / W = 0.5 / 0.3 = 1.67 m (5 s still, 2 s speed ramp) inside a 16 x 10 x 4 m box, using the
seed, duration, origin, noise profile and `T_b_l` of `sim/scenarios/E-01.yaml` and
`config/sim/warehouse.yaml`. The scenario's `world` and `trajectory` are not used yet. The
synthetic-data values come from the scenario's `prep` block (a scenario without one is a setup
error, exit 2):

```yaml
prep:
  static_s: 5.0              # still phase for IMU initialisation
  ramp_s: 2.0                # speed ramp after the still phase
  speed_mps: 0.5             # cruise speed V
  omega_radps: 0.3           # yaw rate W; circle radius R = V / W
  box_half_m: [8.0, 5.0, 2.0]
  mutate:                    # E-03 only, optional: mutate_bag.mutate() arguments
    drop_fraction: 0.05
    reorder_fraction: 0.02
    window_ns: 20000000
    seed: 42
```

Every value must be a finite number > 0, except in `mutate`: the fractions lie in [0, 1] with
at least one > 0, and `seed` >= 0. Without `mutate` the harness can run E-01 and E-02 only;
asking for E-03 is then a setup error (exit 2). These parameters and
`prep_source` (the scenario path) are written to `integration.json` under `data_source`, so a
baseline recorded on this data is not silently compared with a different dataset later.

Gates: E-01 fails if ATE RMSE is more than 20 % above its baseline; E-02 needs a byte-identical,
non-empty `est.tum` on a second run; E-03 must not crash (bench exit 0 or 3), must show that the
drops reached the bench (fewer scans processed than the E-01 dataset holds), and must stay
within the E-01 threshold; PF-01 only warns when the per-scan p95 latency is 20 % above
baseline. A `null` baseline passes with a note that prints the measured value. Each bench run
is killed after `--timeout-s` (default 10 x the scenario duration), which fails that test. The
bench is any command, so it can run in Docker or WSL; all paths are passed absolute. A
`{container}` in `--bench-cmd` (as in `docker run --rm --name {container} ...`) becomes
`mss-bench-<run-id>-<test-id>`, and on a timeout the harness also runs `docker kill` on that
name so the container does not keep running after its client is killed. Locally on
Windows with the bench built in WSL:

```sh
PYTHONPATH="sim;eval" python tools/run_integration.py \
  --bench-cmd "wsl -- env LD_LIBRARY_PATH=/home/<user>/opt/gtsam/lib /home/<user>/mss-build/<build-dir>/bench/mss_bench" \
  --path-style wsl --out results/local-<date> --plots
```

(`;` is the Windows `PYTHONPATH` separator; from Git Bash also set `MSYS_NO_PATHCONV=1`.)
On Linux drop `--path-style wsl` and pass the bench path directly; `--tests E-01,E-02` and
`--duration-s S` select a subset or shorten the scenario. Outputs (only under `--out`):
`E-0x/` (bench outputs, `ate.json`, `traj_xy.png`), `integration.json`, `summary.md`, and the
regenerable bags and datasets in `data/`. Exit codes: 0 every gated check passes, 1 a gated
check fails, 2 setup error (missing scenario, baselines or bench command, data preparation
failure, unexpected exception). CI runs it in the `docker-full` job; the summary goes to the job's step summary and
the results to the `integration-<run-id>` artifact.

## Configuration

All tunables live in `config/default.yaml` under `mss_node.ros__parameters` (ADR-008), one
section per module (`frames`, `imu`, `gnss`, `wheel_odom`, `lidar`, `preprocess`, `map`,
`scan_match`, `degeneracy`, `loop_closure`, `backend`, `realtime`); each key carries its unit in
the name or a comment. A scenario overlay (for example `config/sim/warehouse.yaml`) may only
override keys that exist in the default. `mss::pipeline::load_config(default, overlay)` merges
and validates both files at start-up and reports every bad key in one message
(`<dotted.key>: <reason>`): missing or unknown keys, wrong types, out-of-range values and
non-unit quaternions. Angles are given in degrees with a `_deg` suffix in YAML and loaded
into `_rad` fields in radians (angular rates use `_radps` keys and are already in rad/s);
quaternions are Hamilton `[w, x, y, z]` with a `_wxyz` suffix.

GNSS fusion keys: `gnss.align_min_pairs` (accepted fixes before the w->e alignment is used),
`gnss.align_min_spread_m` (minimum RMS horizontal spread of those fixes, m) and
`gnss.chi2_gate` (NIS threshold of the chi-square innovation gate, 3 DoF, default the 95 %
quantile 7.81), `gnss.max_consecutive_rejects` (fixes; after that many chi-square rejections in
a row the aligner restarts, so a front end that drifted during an outage can re-align).
`gnss.align_check_yaw_sigma` (ADR-025, default on since G13; false is the previous behaviour) makes the default aligner (`GnssAligner`)
also wait until its predicted yaw 1-sigma (from the declared fix covariance, floored at
`gnss.align_yaw_min_var_m2`) is at most `gnss.align_yaw_max_sigma_deg`; the sigma itself is
always computed (`docs/derivations/gnss.md` section 4.1).
`gnss.attach_prealign_fixes` (ADR-022, default off) keeps the fixes received before the first
alignment, the static ones of the IMU initialisation window included, in a buffer of
`gnss.prealign_max_fixes` fixes and adds them as factors on their nearest keyframes once
aligned, so the translation of the GNSS frame is tied to keyframe 0 by measurements.
`gnss.align_yaw_from_track` (ADR-023, default off) takes the yaw of the w->e alignment from the
direction of the GNSS track against the body heading instead of fitting front-end positions
(`docs/derivations/gnss.md` section 5). Its keys:

- `gnss.align_yaw_chunk_fixes`: fixes per chunk.
- `gnss.align_yaw_max_sigma_deg`, `gnss.align_yaw_min_track_m` and
  `gnss.align_yaw_min_coherence`: the alignment waits until the predicted yaw 1-sigma, the
  track length and the track coherence pass them. They replace `align_min_pairs` and
  `align_min_spread_m`.
- `gnss.align_yaw_max_turn_deg`: segments that turn more are not used.
- `gnss.align_yaw_min_var_m2`: floor on the declared fix variance.
- `gnss.align_yaw_min_wheel_travel_m`: below it a segment counts as stationary.
- `gnss.align_yaw_reverse_nsigma`: detects reverse motion when no wheel data exists. With
  wheel odometry the direction of travel comes from the sign of the wheel speed.
- Design 2b (ADR-023 amendment, `docs/derivations/gnss.md` section 5.1):
  `gnss.align_yaw_wheel_turns` gives turn-gated segments their direction from the wheel path
  (of the odometry frame `frames.T_b_o`), with `gnss.align_yaw_wheel_slip_frac` as its error;
  `gnss.align_yaw_min_track_wheel_m` (3 m) replaces the minimum track when every segment has
  wheel data, and a consistency window compares the GNSS track with the wheel travel
  (`gnss.align_yaw_wheel_restart_m`; mismatches in `Stats::n_gnss_wheel_mismatch`, which grows
  with an odometer that over-reads by 2x or more). The direction of travel is the odometry
  frame's x axis, and the turn gate sees turns hidden in a GNSS outage. The
  default yaw sigma is 6 deg (proposed; it drops the 10 deg first-factor target, pending the
  owner).
- Root-cause study of the residual limits (`docs/derivations/gnss.md` section 5.2): a run of up
  to `gnss.align_yaw_wheel_max_gap_fixes` (4) fixes without an odometer reading is bridged; a
  stationary segment is extended over up to `gnss.align_yaw_merge_max_chunks` (10) chunks, so a
  slow robot aligns; and with `gnss.align_yaw_frontend_travel` the front end decides stationarity,
  the travel sign without wheel data and the path of a turn, where the scan matcher has at least
  `gnss.align_yaw_frontend_min_info` (100) constraints along the travel axis. The tolerance
  envelope per noise source (GNSS drift, white sigma, missing readings, speed, skid) is a
  hardware requirement.
  Waiting for the alignment does not affect the local pose, only the ENU output.
`gnss.enabled` is off in `config/default.yaml` until multipath and outage are evaluated; the
sim scenarios turn it on by overlay. `backend.gnss_frame_variable` (ADR-019, default on)
estimates `T_e_w` as one graph variable shared by all GNSS factors, with a prior from
`backend.gnss_frame_tilt_sigma_deg` (tight, keeps it 4-DoF), `backend.gnss_frame_yaw_sigma_deg`
and `backend.gnss_frame_pos_sigma_m` (loose). `imu.init_max_acc_std_mps2` must exceed the per-sample accelerometer noise
`acc_noise_density * sqrt(rate_hz)` by about 3x, or the static initialisation never passes.
`backend.static_gravity_factor` (PR 4d, ADR-020, default on) adds the gravity direction of
the static initialisation window as a factor on keyframe 0's pose and bias (tilt against
horizontal accelerometer bias, no magnitude); `backend.static_gravity_sigma_scale` (default 2,
> 0) multiplies the window's standard error of the mean. It anchors the tilt, and so the
end-point drift along a corridor axis the LiDAR cannot see to what the window measured (it
does not stop mid-run wander, and the straight 20 s corridor misses 3 sigma_DR: ADR-020)
(`docs/derivations/imu_only_observability.md` sections 9, 10 and 12). **Start stationary**: the
factor reads the init window as the body at rest. A start in motion that the initializer still
accepts is caught by the rest gate when it turns or the wheels move
(`imu.init_max_mean_gyro_radps`, `imu.init_max_wheel_speed_mps`, `imu.init_gravity_check_sigma`;
counted in `n_gravity_factor_gated`). A steady straight acceleration passes the gate; the factor
is then wrong but costs at most +2.2 % ATE in the tests, because the initializer's attitude
comes from the same window, and in a scene that does not observe the travel axis no
statistic can detect it (ADR-020): starting at rest stays a precondition. The innovation
chi-square check `backend.static_gravity_check_keyframes` (default 0 = off; > 0 runs it at
that keyframe against `backend.static_gravity_chi2_gate`, counters `n_gravity_factor_removed`
and `n_gravity_factor_unchecked`) was evaluated and ships off: at rest it removed the factor
in 6 of 15 GNSS-corridor runs, with statistics that overlap those of starts in motion.

`backend.relative_factor` (default `decoupled`, ADR-021) selects the form of the LiDAR and
loop relative-pose factors: `decoupled` (translation and rotation residuals separate, exact
Jacobians), `between` (`gtsam::BetweenFactor<Pose3>`, whose Jacobian omits the `Jr^-1` of the
`Logmap`; the behaviour before ADR-021) or `se3_exact` (the same SE(3) residual with exact
Jacobians). All three read the scan matcher's covariance the same way to first order and
differ only at large residuals; for a free translation axis `decoupled` keeps the axis free at
any rotation, which is why it is the default. `between` and `se3_exact` remain selectable for
comparison (`docs/derivations/backend_factors.md` section 5); any other value is a config
error.

Plane-fit gates and map spacing (ADR-007 amendment), all evaluated and off by default (the
shipped behaviour is bit-identical to main): `scan_match.plane_min_eig_ratio` (`[0, 1)`)
rejects a neighbourhood with `lambda_min > ratio * lambda_mid` (two faces across an edge). It
improves E-01 on noise-free sim data (2.2 to 0.95 mm at 0.01), but the ratio compares range
noise with patch size: at 1 cm noise about half of the planar 5-NN patches are rejected at
0.01, and on the ov_plain replay 0.01 gives ATE 14.8 to 24.0 mm and scan p95 6.1 to 11.5 ms
(0.05: 14.5 mm, 5.5 ms). Its cut of the corridor-axis information (1.1 to 0.10 %) is a
conditioning statement, not an accuracy gain: the fused corridor ATE goes 1.8 to 5.2 m.
`scan_match.min_mid_eig_ratio` (`[0, 1)`) rejects `lambda_mid <= ratio * lambda_max` (lines,
thin strips); at 0.2 it removes the S-02 box-edge bias but breaks narrow corridors.
`map.min_point_dist_m` (m, `>= 0`) makes a new point replace the nearest stored point of its
voxel closer than this: a memory and speed option (0.1 m: about 30 % fewer map points).
Tables: `docs/derivations/map.md` and `docs/derivations/scan_match.md` section 3.

`deterministic: true` (the default) runs the estimator on one thread with a fixed seed so a
replay reproduces bit for bit. It also sets `backend.single_thread` (not a YAML key): the first
such backend `Graph` caps TBB at one thread for the rest of the process, so GTSAM's iSAM2
never runs on worker threads (ADR-018). The limit is process-wide and never released; a later
Graph with `single_thread: false` in the same process stays limited and logs a warning. On a
300-keyframe test graph, timed in separate processes per mode, single-thread `optimize()` was
faster (0.195 s against 0.262 s per run); the E-01 bench replay was within run-to-run noise,
with lower CPU use. With `deterministic: false` TBB keeps its default parallelism.

## Before committing

`pre-commit install` once; it runs clang-format and ruff. CI additionally builds with
`-Werror` and runs the tests.
