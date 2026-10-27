# sim

Gazebo Harmonic worlds (SDF), reference trajectories and sensor noise models.
Scripts take paths as arguments and write to results/<run-id>/.

## Noise layer (offline bridge)

Gazebo records a clean raw bag (`/ground_truth`, `/imu/data`, `/lidar/points`, `/gnss/fix`,
`/joint_states`, `/tf_static`). The noise layer turns it into the bag the SLAM pipeline reads.
Python package: `sim/mss_sim` (no ROS needed). Run from the repo root:

```
PYTHONPATH=sim python -m mss_sim bridge <raw_bag> sim/scenarios/E-01.yaml results/<run-id> [--seed N] [--no-skew]
PYTHONPATH=sim python -m mss_sim gt-to-tum <bag> <out.tum>
```

`bridge` prints the report as one JSON line and writes, under `<out_dir>`: `bag/` (MCAP; must not
exist yet), `gt.tum` and `bridge_report.json` (`n_msgs`, `n_dropped_gnss`, `n_odom_errors`,
`n_gt_clamped`, `n_gnss_rejected`). Output topics: `/lidar/points` (adds per-point `t` and `ring`, motion skew,
dropout, bad points), `/imu/data`, `/gnss/fix` (degraded, `position_covariance_type = 2`),
`/wheel/odom` (encoders read at `odom.rate_hz`, quantised, believed kinematics), `/ground_truth` and `/tf_static`
(copied). `/joint_states` is consumed, not re-emitted; `/clock` is not produced offline.
Geometry (`T_b_l`) and the true antenna position come from the raw bag.

Structure:
- `sim/noise/<profile>.yaml`: sensor noise (`imu`, `gnss`, `lidar`, `odom`), `clock_offset_ns`
  per sensor and `inflation_max`. Units are in key names; loaded and validated by `profile.py`.
- `sim/scenarios/<id>.yaml`: world, noise profile, pipeline config, trajectory, `seed`,
  `duration_s`, `origin_lla`, GNSS `gnss` segments (nominal/degraded/multipath/outage by time) and
  `gnss_zones` (ENU polygons), and the topic list.

Determinism (ADR-011 section 1): Gazebo is not deterministic, the noise layer is. One seed gives
`SeedSequence(seed).spawn(5)` streams for [lidar, gnss, odom, imu, mutate], and a sixth child,
`SeedSequence(seed).spawn(6)[5]`, is the `driver` stream (the route's speed factors, see Gazebo
below; spawning six children leaves the first five unchanged). The same raw bag,
scenario, profile and seed produce byte-identical messages and log times (SIM-05). `--seed`
overrides the scenario seed. Record the raw bag once and re-run the bridge to vary noise.

## Gazebo (ADR-011 part B)

Desktop only: ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) + `ros_gz_sim`, `ros_gz_bridge`,
`rosbag2_storage_mcap`. CI never runs Gazebo; it runs `tools/ci/check_sim_assets.sh`.

What lives where:
- `sim/models/robot.sdf.j2` + `sim/models/robot.yaml`: the robot (ADR-011 section 2). Never
  edit generated SDF. Wheel radius and track come from `odom.kinematics_true` of the profile,
  extrinsics (`T_b_l`, `p_b_g`) and frame names (`imu_link`, `lidar_link`, `gnss_link`, `map`)
  from the scenario's pipeline overlay. The model frame is the body frame `b` (IMU) at axle
  height; `OdometryPublisher` on it gives `/ground_truth` at 200 Hz.
- `sim/worlds/<world>.sdf`: hand-written worlds (DART, 1 ms, no real-time lock, floor mu 0.8):
  `warehouse` (40 x 25 m, 6 m ceiling, model `shelf_run_15m`), `corridor` (60 x 2.5 m,
  3.0 m ceiling, 6 x 6 m end room). The world frame is the ENU frame (spherical coordinates =
  the scenario `origin_lla`, tested).
- `sim/worlds/<world>.planes.yaml`: sidecar with the reference planes `n . p = d` (world, m)
  that `noise_report --planes` reads, the robot `spawn` pose and `enu_origin_w_m` (always 0).
  `python tools/gen_sim_assets.py --check-worlds` checks every plane lies on a box face (1 mm).
- `sim/trajectories/<route>.yaml`: `waypoints` `[x_m, y_m]` in world, `speed_mps`,
  `lookahead_m`; loaded and validated with the scenario (`Scenario.route`).
- `sim/mss_sim/pursuit.py` (no ROS) and `driver.py` (rclpy): pure pursuit, 5 s still start,
  speed x U(0.9, 1.1) per 5 m of route from the `driver` stream, v <= 1.0 m/s, a <= 0.5 m/s^2,
  omega <= 1.0 rad/s (`robot.yaml` `drive`), braking to a stop at the last waypoint.
- `sim/mss_sim/tf_static.py`: one latched `/tf_static` message with `T_b_l` and `p_b_g`.
- `sim/launch/sim.launch.py`: gz server, spawn, `parameter_bridge`, `/tf_static`, driver.

Commands (WSL or Ubuntu, after `source /opt/ros/jazzy/setup.bash`; `PYTHON` must have the
`pyproject.toml` deps):

```
python tools/gen_sim_assets.py --profile sim/noise/default.yaml \
    --pipeline-config config/sim/warehouse.yaml --out results/sim-assets
ros2 launch sim/launch/sim.launch.py scenario:=sim/scenarios/E-01.yaml \
    assets_dir:=results/sim-assets [headless:=false] [render_engine:=ogre] [driver:=false]
tools/record_scenario.sh sim/scenarios/E-01.yaml results/<run-id>/   # everything, one command
```

`record_scenario.sh` writes under `results/<run-id>/`: `assets/`, `raw_bag/` (raw, clean: the
topics below), `bag/` + `gt.tum` + `bridge_report.json` (bridged), `raw_gt.tum`,
`noise_report/`, `record_summary.json` (sim and wall time, real-time factor, raw bag files with
SHA256) and `logs/`. Exit 0 ok, 1 noise report failed, 2 setup/launch/recording/bridge failed.
It starts the recorder first, launches Gazebo paused, steps the world until the recorder has
subscribed, unpauses, pauses again at `duration_s` + 2 s of sim time, then stops everything
(a trap kills Gazebo and the bridge on any exit). Options: `--no-driver` (robot stays at its
spawn pose) and `--lidar-gz-sigma S` (Gazebo LiDAR noise, for the render calibration below).
Raw topics: `/lidar/points`, `/imu/data`, `/gnss/fix` (clean fix), `/ground_truth`,
`/tf_static`, `/clock`, `/joint_states` (wheel joints, every physics step = 1 kHz); MCAP, zstd
file compression, 100 MB splits, sim-time log stamps.

Wheel odometry: the bridge reads the encoders at `odom.rate_hz` (50 Hz) whatever the
joint-state rate. Wheel angles integrate between joint states and the ticks are read once per
20 ms window, so `/wheel/odom` is 50 Hz and the tick quantisation is one tick per window
(0.0078 m/s per wheel at 0.10 m radius), as on a real controller. `bridge_report.json`
`odom_errors` counts rejected joint states by reason: `missing_joint`, `non_finite`,
`stamp_not_increasing`. On Gazebo bags only the last occurs: after the final pause the paused
world keeps publishing joint states with an unchanged stamp (48 on E-01, 9 on E-04 below);
they carry no motion and are skipped.

Continuous -> discrete noise (`gen_sim_assets.py`, comment in the template):
- IMU white noise: Gazebo `stddev = N * sqrt(f)`, f = 200 Hz.
- IMU bias: `bias_stddev` = the profile's initial bias sigma (drawn once from the gz seed =
  scenario seed); `dynamic_bias_stddev = K` (random-walk density) and
  `dynamic_bias_correlation_time = tau` (1000 s). Gazebo's dynamic bias is first-order
  Gauss-Markov whose driving noise has density K: the bias increments have variance K^2 t for
  t << tau (checked: with K = 1 the bias reached 15-25 after 250-300 s, i.e. sqrt(t), not a
  stationary sigma of 1), stationary sigma K sqrt(tau/2). It follows the random walk only over
  tau in [0.01, 100] s (ADR-011 section 3).
- LiDAR range: Gazebo `stddev = sqrt(lidar.range_sigma_m^2 - lidar.render_sigma_m^2)`, see
  "LiDAR render error" below.
- GNSS: no Gazebo noise; the offline degrader adds it.

### LiDAR render error

gpu_lidar adds a range error of its own (depth-texture interpolation). Procedure, 2026-10-28:
record each world for 10 s with the robot still and the Gazebo LiDAR noise at 0, then read the
noise report's per-range-bin rows (along-ray residual to the sidecar planes, `|n.u| > 0.7`):

```
tools/record_scenario.sh <scenario with duration_s: 10> results/<run-id>/ --no-driver --lidar-gz-sigma 0
```

| World (static) | 0.5-2 m | 10-20 m | 20-40 m | 40-100 m | all |
| --- | --- | --- | --- | --- | --- |
| warehouse | - | 0.0108 m | 0.0151 m | - | 0.0113 m |
| corridor | 0.0012 m | 0.0029 m | - | 0.0006 m (640 pts) | 0.0013 m |

The error depends on range and scene beyond what one constant covers (0.0006 to 0.0151 m).
`lidar.render_sigma_m` = 0.003 m is the median of the five bins; it is not tuned to the gate.
Timing was ruled out:
- A driven warehouse run with the Gazebo noise at 0 gives the same 0.0113 m as the static one.
- On the driven E-01 bag, a -6..+6 ms sweep of the LiDAR stamps gives the smallest residual
  spread with the stamps unshifted (minimum at 0 within 1 ms, same on the raw and bridged bags),
  so there is no stamp vs ground-truth timing error.

Consequence: with a Gazebo noise of 0.0198 m the corridor measures 0.0198 m, but the warehouse
measures 0.0231 m, 0.4 points outside the +-15 % gate (see the table below). A single constant
cannot hold at the 10-100 m ranges of an outdoor world.

### Wheel contact calibration

Odometry with the true kinematics (so the believed-kinematics error is excluded), 50 Hz
encoder, against `/ground_truth`: a 10 m straight at 0.5 m/s and a spin in place of 2 turns at
0.5 rad/s, robot at the warehouse spawn, one change at a time (2026-10-28):

| Wheel collision | straight 10 m | spin in place |
| --- | --- | --- |
| cylinder 0.04 m wide (first recording) | -0.04 % | -36.25 deg/rev |
| cylinder 0.01 m wide | -0.04 % | -9.25 deg/rev |
| sphere, wheel radius (shipped) | -0.01 % | -0.08 deg/rev |

A cylinder contacts the floor at a rim, an effective track of ~0.36 m instead of 0.40 m; the
error scales with the width. A sphere of the wheel radius contacts in the wheel's mid-plane
(`robot.yaml` `wheel.collision_shape: sphere`; the visual stays a cylinder). On the E-01 lap
with the true kinematics, straight segments read 0.00 % before and after; turns read
-35.87 deg/rev before and +0.00 deg/rev after. WheelSlip (compliance 0) adds no slip, and it
can only add slip.

### Máy bàn

Dev machine: WSL Ubuntu 24.04, 9 GB RAM, ROS 2 Jazzy, gz-sim 8.15.0 (vendor packages under
`/opt/ros/jazzy/opt`), headless `ogre2` works under WSLg (no fallback to `ogre` needed). The WSL
system Python has no `rosbags`/`shapely`/`allantools` and no `python3-venv`; without sudo they
were installed with `pip install --no-deps --target <dir> rosbags shapely allantools zstandard
lz4 ruamel.yaml backports.zstd apsw` and that dir put on `PYTHONPATH`. With sudo:
`sudo apt-get install python3.12-venv` and a venv with `--system-site-packages` (rclpy).
Gazebo, if missing: `sudo apt-get install ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap`.
Image alternative: `docker/sim.Dockerfile` (usage in its header).

Numbers of the final recording (2026-10-28, `results/<run-id>` on `/mnt/c`, seed 42, sphere
wheels, `render_sigma_m` 0.003). Gazebo is not deterministic, so a re-recorded raw bag differs
in bytes; the SHA256 identify this recording. Provisional until `testdata-v1` is packaged.

| Scenario | sim s recorded | wall s (record) | real-time factor | raw bag |
| --- | --- | --- | --- | --- |
| E-01 | 65 | 32.2 | 2.02 | 89.1 MB, 3 x `.mcap.zstd` + metadata |
| E-04 | 74 | 36.5 | 2.03 | 99.6 MB, 4 x `.mcap.zstd` + metadata |

E-01 raw bag (`sha256sum`, bytes):

```
1ebe1ff2691b75db655928071cf6531c744943744beaab00f7beb1a37ade010d  raw_bag_0.mcap.zstd  32242816
64a71394057f8d9b1e8a53c4a1c6027973434e17c6866e370f9a2ed585f19376  raw_bag_1.mcap.zstd  32527774
6cfcfefa06ec5998ebd17681db41df0e0a37255df8b1334d54feb71c4208f6ea  raw_bag_2.mcap.zstd  24354610
47142eeb76a4e469c19933e00b6d824d2b9b5f514685ce6af59ca8b2e42d4e4e  metadata.yaml        6130
```

Message counts per run: IMU and `/ground_truth` 200 Hz with no gap, LiDAR 10 Hz, GNSS 5 Hz,
`/wheel/odom` 50 Hz (3328 on E-01). Noise report (E-01 / E-04):

| Row | declared | E-01 | E-04 |
| --- | --- | --- | --- |
| IMU gyro sigma x, y, z | 0.002475 | 0.00237, 0.00254, 0.00245 | 0.00247, 0.00234, 0.00249 |
| IMU acc sigma x, y, z | 0.04101 | 0.0407, 0.0404, 0.0419 | 0.0416, 0.0411, 0.0402 |
| GNSS sigma_h / sigma_v [m] | 1.0 / 2.0 | 1.010 / 1.929 | 1.018 / 1.897 |
| LiDAR range sigma [m] | 0.02 | **0.0231 (fail, +15.4 %)** | 0.0198 |
| LiDAR dropout | model | 0.0568 vs 0.0587 | 0.0413 vs 0.0419 |
| Odom drift (not gated) | 1-3 % | 1.66 % | 0.93 % |
| Odom yaw (not gated) | 4-5 deg/rev | 4.52 deg/rev | (no turn) |

E-01 exits 1 on the LiDAR range sigma only (render error at warehouse ranges, see above); every
other gated row passes for both. The odometry rows now meet ADR-011 section 3, with the
believed kinematics (1.01 radius, 0.399 track) as the only error.

#### Pipeline on the Gazebo bag

`tools/run_integration.py --bridged <record_scenario out_dir>` replaces the synthetic data by a
recorded bridge output (ENU origin = scenario `origin_lla`); the rest of the chain
(`bag_to_mssdata.py`, `mss_bench`, `mss_eval.ate`) takes the Gazebo bag unchanged. Bench:
WSL build of this branch (`tools/wsl/build.sh --target mss_bench`), desktop, 2026-10-28.

```
python tools/run_integration.py --bridged results/<E-01 run> --scenario sim/scenarios/E-01.yaml \
    --bench-cmd "wsl -- env LD_LIBRARY_PATH=<gtsam lib> <build>/bench/mss_bench" \
    --path-style wsl --out results/<run-id> --plots
```

| Run | ATE RMSE [m] (pairs) | scans | p50 / p95 scan [ms] | keyframes | GNSS factors | odom factors |
| --- | --- | --- | --- | --- | --- | --- |
| E-01 (warehouse lap) | 0.0965 (644) | 666 | 4.99 / 8.63 | 42 | 203 | 41 |
| E-02 (E-01 again) | 0.0965, `est.tum` byte-identical | 666 | 4.87 / 8.15 | 42 | 203 | 41 |
| E-03 (E-01 bag, 5 % drops) | 0.0945 (587) | 636 | 4.85 / 8.41 | 42 | 204 | 41 |
| E-04 corridor, degeneracy off | 13.66 (733) | 755 | 4.56 / 6.73 | 16 | 71 | 15 |
| E-04 corridor, degeneracy on | 1.55 (733) | 755 | 4.29 / 7.70 | 54 | 246 | 53 |

All runs: bench exit 0, 0 match failures, `n_gnss_late` / `n_gnss_ring_overflow` /
`n_gnss_unfused_eof` 0, `n_gravity_factor_gated` / `_removed` / `_unchecked` 0 (with
`static_gravity_check_keyframes: 0` the check never runs, so no counter moves). These are not
baselines: `eval/baselines.json` changes only in a `baseline-update` PR.

Sensor ablation on the same E-01 bag and bench, one overlay change each from
`config/sim/warehouse.yaml` (ATE after SE(3) alignment, 644 pairs; horizontal = RMS of the xy
error, vertical = RMS of z; `n_gnss_rejected` (bridge) = 0; `/wheel/odom` 3328 records):

| Overlay | ATE RMSE [m] | horiz. / vert. RMS [m] | max horiz. [m] | keyframes | p95 [ms] | GNSS factors | odom factors |
| --- | --- | --- | --- | --- | --- | --- | --- |
| (a) LiDAR + IMU | 0.093 | 0.016 / 0.092 | 0.048 | 42 | 7.93 | 0 | 0 |
| (b) + wheel (GNSS off) | 0.096 | 0.014 / 0.095 | 0.046 | 42 | 8.89 | 0 | 41 |
| (c) + GNSS (wheel off) | 0.093 | 0.016 / 0.092 | 0.048 | 42 | 7.99 | 203 | 0 |
| (d) shipped (both) | 0.097 | 0.014 / 0.096 | 0.047 | 42 | 8.62 | 203 | 41 |

With the 50 Hz encoder and the sphere wheels, wheel odometry no longer degrades the estimate:
horizontal RMS is 0.014 m with it vs 0.016 m without, and p95 is within 1 ms. GNSS still
changes nothing measurable. About 0.09 m of vertical error remains in every row, so it comes
from the LiDAR + IMU part; the table does not show which of the two.

## Mutating a bag (E-03)

`tools/mutate_bag.py` derives the robustness bag from a bridged one (packet loss and reordering
inside a time window, seeded):

```
python tools/mutate_bag.py <in_bag> <out_bag> [--drop P] [--reorder P] [--window-ns N] [--seed N]
```

E-03 bags skip the noise report: reordering violates its ordering check by design. Run the
report on the E-01 bag the E-03 bag was derived from.

## Chuyển đổi cho bench

`tools/bag_to_mssdata.py` chuyển bag đã qua cầu nối sang thư mục nhị phân phẳng `mssdata` cho `bench` (không cần ROS trên Pi5; ADR-014, layout trong `docs/mssdata_format.md`):

```
python tools/bag_to_mssdata.py <bag_dir> <out_dir> [--origin-lla LAT LON ALT] [--pipeline-config cfg.yaml]
```

Gốc ENU mặc định là fix GNSS đầu tiên. `--pipeline-config` cấp `frames.p_b_g_m` và sigma GNSS (dùng khi bag không có hiệp phương sai). In báo cáo một dòng JSON.

Tests: `python -m pytest tests/python`; lint: `ruff check . && ruff format --check .`.
