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
world keeps publishing joint states with an unchanged stamp (8 on E-01, 22 on E-04 below);
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
- LiDAR range: Gazebo `stddev = lidar.range_sigma_m`, exactly the declared value. The renderer
  adds its own error on top; that error is modelled, not compensated (see "LiDAR render error").
- GNSS: no Gazebo noise; the offline degrader adds it.

### LiDAR render error

The declared `lidar.range_sigma_m` is what the noise layer adds (the Gazebo noise). gpu_lidar
adds `sigma_render(r)` on top (depth-texture interpolation); this is measured, not tuned, and
stored as `lidar.render_model: {k_per_m, offset_m}`, with `sigma_render(r) = k_per_m * r +
offset_m`. The noise report expects `sqrt(sigma_declared^2 + sigma_render(r_bin)^2)` per range
bin, where `r_bin` is the bin's mean range. It gates each bin at +-15 % when the bin has at
least 1000 points, and shows bins with 100-999 points without gating; the pooled row is
informational.

Procedure, 2026-10-28: record each world for 10 s with the robot still and the Gazebo LiDAR
noise at 0, then read the noise report's per-range-bin rows (along-ray residual to the sidecar
planes, `|n.u| > 0.7`):

```
tools/record_scenario.sh <scenario with duration_s: 10> results/<run-id>/ --no-driver --lidar-gz-sigma 0
```

| World (static) | bin | points | mean range | sigma |
| --- | --- | --- | --- | --- |
| corridor | 0.5-2 m | 808 014 | 1.4 m | 0.0012 m |
| corridor | 10-20 m | 22 630 | 12.2 m | 0.0029 m |
| corridor | 40-100 m | 640 | 53.9 m | 0.0006 m |
| warehouse | 10-20 m | 167 160 | 13.6 m | 0.0108 m |
| warehouse | 20-40 m | 16 875 | 28.3 m | 0.0151 m |

Fits by least squares over the five bins, with residuals in bin order:
- `k * r`: k = 1.59e-4 m/m. Residuals +0.0010, +0.0010, -0.0080, +0.0087, +0.0106 m.
- `k * r + offset`: k = -1.6e-5 m/m, offset = 0.0065 m. Residuals -0.0053, -0.0034, -0.0050,
  +0.0046, +0.0090 m.

**No range model fits within the bin spread.** The bin standard errors are 1e-5 to 8e-5 m and
the residuals are up to 0.011 m. At the same range the error depends on incidence and on the
scene. Warehouse points at 10-20 m read 0.0049 m head-on (|n.u| > 0.97) and 0.0158 m at
|n.u| 0.7-0.8. Head-on walls read 0.0029 m at 12 m and 0.0006 m at 54 m in the corridor, but
0.0151 m at 28 m in the warehouse. That is consistent with mixed depth texels at the edges of
the cluttered warehouse, not with range alone. The profile stores the simplest fit, `k * r`
(k = 1.59e-4, offset 0), as measured.

Timing was ruled out:
- A driven warehouse run with the Gazebo noise at 0 gives the same 0.0113 m as the static one.
- On the driven E-01 bag, a -6..+6 ms sweep of the LiDAR stamps gives the smallest residual
  spread with the stamps unshifted (minimum at 0 within 1 ms, same on the raw and bridged bags).

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
wheels, Gazebo LiDAR noise 0.02 m = declared). Gazebo is not deterministic, so a re-recorded raw
bag differs in bytes; the SHA256 identify this recording. Provisional until `testdata-v1`.

| Scenario | sim s recorded | wall s (record) | real-time factor | raw bag |
| --- | --- | --- | --- | --- |
| E-01 | 65 | 32.2 | 2.02 | 87.9 MB, 3 x `.mcap.zstd` + metadata |
| E-04 | 74 | 36.5 | 2.03 | 99.7 MB, 4 x `.mcap.zstd` + metadata |

E-01 raw bag (`sha256sum`, bytes):

```
8cb224f36f2788a94c3f5af367b96ee921a65d135915f285bd0a70f3e120b544  raw_bag_0.mcap.zstd  32240194
c71238318122a8213ecbca190c75211124175a435a7fbbe47a990864fa42ad20  raw_bag_1.mcap.zstd  32531731
153c5a0abb97590761dadb06aeabe418c5517adce9f9a1c7edc68a786a60224e  raw_bag_2.mcap.zstd  23164479
95baeb7b850848e42f043f2d28cf69b1d268a9f0caf20fb5b8d820ed88a76ad0  metadata.yaml        6130
```

Message counts per run: IMU and `/ground_truth` 200 Hz with no gap, LiDAR 10 Hz, GNSS 5 Hz,
`/wheel/odom` 50 Hz (3280 on E-01). Noise report (E-01 / E-04):

| Row | declared | E-01 | E-04 |
| --- | --- | --- | --- |
| IMU gyro sigma x, y, z | 0.002475 | 0.00246, 0.00250, 0.00251 | 0.00256, 0.00237, 0.00258 |
| IMU acc sigma x, y, z | 0.04101 | 0.0417, 0.0411, 0.0430 | 0.0434, 0.0409, 0.0420 |
| GNSS sigma_h / sigma_v [m] | 1.0 / 2.0 | 1.011 / 1.936 | 1.018 / 1.897 |
| LiDAR dropout | model | 0.0567 vs 0.0587 | 0.0413 vs 0.0419 |
| Odom drift (not gated) | 1-3 % | 1.67 % | 0.92 % |
| Odom yaw (not gated) | 4-5 deg/rev | 4.49 deg/rev | (no turn) |

LiDAR range sigma per bin (expected = sqrt(0.02^2 + (1.59e-4 r)^2), +-15 %):

| World | bin | points | mean range | expected | measured | result |
| --- | --- | --- | --- | --- | --- | --- |
| E-01 warehouse | 10-20 m | 625 671 | 14.7 m | 0.0201 m | 0.0230 m (+14.4 %) | pass |
| E-01 warehouse | 20-40 m | 175 081 | 23.5 m | 0.0203 m | 0.0239 m (+17.7 %) | **fail** |
| E-04 corridor | 0.5-2 m | 3 433 118 | 1.4 m | 0.0200 m | 0.0201 m | pass |
| E-04 corridor | 2-5 m | 1 024 054 | 3.4 m | 0.0200 m | 0.0200 m | pass |
| E-04 corridor | 5-10 m | 51 974 | 7.1 m | 0.0200 m | 0.0202 m | pass |
| E-04 corridor | 10-20 m | 41 376 | 13.6 m | 0.0201 m | 0.0201 m | pass |
| E-04 corridor | 20-40 m | 15 579 | 27.6 m | 0.0205 m | 0.0199 m | pass |
| E-04 corridor | 40-100 m | 3 550 | 50.0 m | 0.0215 m | 0.0204 m | pass |

E-04 passes every gated row. E-01 fails only the 20-40 m range bin: no range model describes the
warehouse render error (see "LiDAR render error"). It is reported as measured and not tuned, so
`record_scenario.sh` E-01 exits 1. `odom_errors`: `stamp_not_increasing` only (8 on E-01, 22 on
E-04), the duplicate joint-state stamps after the final pause. The odometry meets ADR-011
section 3, with the believed kinematics (1.01 radius, 0.399 track) as the only error.

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
| E-01 (warehouse lap) | 0.0955 (635) | 657 | 4.92 / 8.65 | 42 | 203 | 41 |
| E-02 (E-01 again) | 0.0955, `est.tum` byte-identical | 657 | 5.00 / 8.88 | 42 | 203 | 41 |
| E-03 (E-01 bag, 5 % drops) | 0.0960 (579) | 623 | 4.99 / 8.75 | 42 | 203 | 41 |
| E-04 corridor, degeneracy off | 13.64 (734) | 756 | 4.33 / 6.59 | 16 | 72 | 15 |
| E-04 corridor, degeneracy on | 2.13 (734) | 756 | 5.00 / 8.42 | 53 | 108 | 52 |

All runs: bench exit 0, 0 match failures, `n_gnss_late` / `n_gnss_ring_overflow` /
`n_gnss_unfused_eof` 0, `n_gravity_factor_gated` / `_removed` / `_unchecked` 0 (with
`static_gravity_check_keyframes: 0` the check never runs, so no counter moves). These are not
baselines: `eval/baselines.json` changes only in a `baseline-update` PR.

Sensor ablation on the same E-01 bag and bench, one overlay change each from
`config/sim/warehouse.yaml` (ATE after SE(3) alignment, 635 pairs; horizontal = RMS of the xy
error, vertical = RMS of z; `n_gnss_rejected` (bridge) = 0; `/wheel/odom` 3280 records):

| Overlay | ATE RMSE [m] | horiz. / vert. RMS [m] | max horiz. [m] | keyframes | p95 [ms] | GNSS factors | odom factors |
| --- | --- | --- | --- | --- | --- | --- | --- |
| (a) LiDAR + IMU | 0.093 | 0.015 / 0.092 | 0.046 | 42 | 7.74 | 0 | 0 |
| (b) + wheel (GNSS off) | 0.095 | 0.013 / 0.094 | 0.035 | 42 | 8.91 | 0 | 41 |
| (c) + GNSS (wheel off) | 0.093 | 0.016 / 0.092 | 0.047 | 42 | 7.71 | 203 | 0 |
| (d) shipped (both) | 0.096 | 0.013 / 0.095 | 0.035 | 42 | 8.81 | 203 | 41 |

With the 50 Hz encoder and the sphere wheels, wheel odometry no longer degrades the estimate:
horizontal RMS is 0.013 m with it vs 0.015 m without, and p95 is within about 1 ms. GNSS still
changes nothing measurable. About 0.09 m of vertical error remains in every row, so it comes
from the LiDAR + IMU part.

Diagnostics for the pipeline follow-up (aligned error over time, RMS per window):

| Run | 0-10 s horiz. / vert. | 10-30 s | 30-50 s | 50 s-end |
| --- | --- | --- | --- | --- |
| E-01 LiDAR + IMU | 0.016 / 0.109 (mean z +0.099) | 0.021 / 0.122 (-0.037) | 0.012 / 0.070 | 0.008 / 0.055 |
| E-04 degeneracy on | 0.512 / 3.757 (mean z -3.714) | 0.207 / 2.071 (+1.054) | 0.197 / 1.856 (+1.780) | 0.406 / 1.490 (-1.062) |

In E-01 the vertical error is largest early, with an offset of +0.10 m in the first 10 s, and it
decreases later. E-04 with degeneracy on drifts vertically by metres while the horizontal error
stays near 0.2-0.5 m. The bench exports no degeneracy counters; its log flags 47 of 53 keyframes
degenerate (`add_lidar_factor ... degenerate`), without the directions.

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
