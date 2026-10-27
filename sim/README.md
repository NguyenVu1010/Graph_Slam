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
`/wheel/odom` (quantised encoders, believed kinematics), `/ground_truth` and `/tf_static`
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
python tools/gen_sim_assets.py --profile sim/noise/default.yaml     --pipeline-config config/sim/warehouse.yaml --out results/sim-assets
ros2 launch sim/launch/sim.launch.py scenario:=sim/scenarios/E-01.yaml     assets_dir:=results/sim-assets [headless:=false] [render_engine:=ogre]
tools/record_scenario.sh sim/scenarios/E-01.yaml results/<run-id>/   # everything, one command
```

`record_scenario.sh` writes under `results/<run-id>/`: `assets/`, `raw_bag/` (raw, clean: the
topics below), `bag/` + `gt.tum` + `bridge_report.json` (bridged), `raw_gt.tum`,
`noise_report/`, `record_summary.json` (sim and wall time, real-time factor, raw bag files with
SHA256) and `logs/`. Exit 0 ok, 1 noise report failed, 2 setup/launch/recording/bridge failed.
It starts the recorder first, launches Gazebo paused, steps the world until the recorder has
subscribed, unpauses, pauses again at `duration_s` + 2 s of sim time, then stops everything
(a trap kills Gazebo and the bridge on any exit). Raw topics: `/lidar/points`, `/imu/data`,
`/gnss/fix` (clean fix), `/ground_truth`, `/tf_static`, `/clock`, `/joint_states` (wheel
joints, 1 kHz); MCAP, zstd file compression, 100 MB splits, sim-time log stamps.

Continuous -> discrete noise (`gen_sim_assets.py`, comment in the template):
- IMU white noise: Gazebo `stddev = N * sqrt(f)`, f = 200 Hz.
- IMU bias: `bias_stddev` = the profile's initial bias sigma (drawn once from the gz seed =
  scenario seed); `dynamic_bias_stddev = K` (random-walk density) and
  `dynamic_bias_correlation_time = tau` (1000 s). Gazebo's dynamic bias is first-order
  Gauss-Markov whose driving noise has density K: the bias increments have variance K^2 t for
  t << tau (checked: with K = 1 the bias reached 15-25 after 250-300 s, i.e. sqrt(t), not a
  stationary sigma of 1), stationary sigma K sqrt(tau/2). It follows the random walk only over
  tau in [0.01, 100] s (ADR-011 section 3).
- LiDAR range: Gazebo `stddev = sqrt(range_sigma_m^2 - render_range_sigma_m^2)`.
  gpu_lidar adds its own range error (depth-texture interpolation, grows with range and
  incidence: ~0 at 1-3 m, 0.011 m at 12-20 m, 0.0125 m at 20-40 m with the Gazebo noise at 0).
  `robot.yaml` `lidar.render_range_sigma_m: 0.008` puts both worlds within +-8 % of the profile.
- GNSS: no Gazebo noise; the offline degrader adds it.

### Máy bàn

Dev machine: WSL Ubuntu 24.04, 9 GB RAM, ROS 2 Jazzy, gz-sim 8.15.0 (vendor packages under
`/opt/ros/jazzy/opt`), headless `ogre2` works under WSLg (no fallback to `ogre` needed). The WSL
system Python has no `rosbags`/`shapely`/`allantools` and no `python3-venv`; without sudo they
were installed with `pip install --no-deps --target <dir> rosbags shapely allantools zstandard
lz4 ruamel.yaml backports.zstd apsw` and that dir put on `PYTHONPATH`. With sudo:
`sudo apt-get install python3.12-venv` and a venv with `--system-site-packages` (rclpy).
Gazebo, if missing: `sudo apt-get install ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap`.
Image alternative: `docker/sim.Dockerfile` (usage in its header).

Expected numbers (2026-10-28, `results/<run-id>` on `/mnt/c`, seed 42). Gazebo is not
deterministic, so a re-recorded raw bag differs in bytes; the SHA256 identify this recording.

| Scenario | sim s recorded | wall s (record / total) | real-time factor | raw bag |
| --- | --- | --- | --- | --- |
| E-01 | 65 | 36.6 / 75 | 1.78 | 88.9 MB, 3 x `.mcap.zstd` + metadata |
| E-04 | 72 | 41.3 / 84 | 1.74 | 95.9 MB, 4 x `.mcap.zstd` + metadata |

E-01 raw bag (`sha256sum`, bytes):

```
70e658b7e50947a1bacfcff4f538e5390c42a93055617742e7fbaecc08577831  raw_bag_0.mcap.zstd  32245533
e99ecd4959a0aae44c8df46f404e5777b6cfb808c13d188d583e0f9d438ddc49  raw_bag_1.mcap.zstd  32561457
0611f361c3c5ff2cc978348a241a22246d965ce68bad1aa2418938d359e72ee4  raw_bag_2.mcap.zstd  24109436
c93c3068a0ea53633a5bc4fc04ec667f606c5295da955a433aa4fb9b51bdc43c  metadata.yaml        6130
```

Message counts per run: IMU and `/ground_truth` 200 Hz with no gap, LiDAR 10 Hz, GNSS 5 Hz.
Noise report: every gated row passes for E-01 and E-04 (IMU sigma within 6 %, GNSS sigma_h
1.01 / 1.03 m, sigma_v 1.93 / 1.90 m, LiDAR range sigma 0.0218 / 0.0184 m, dropout share
0.0566 vs 0.0586 / 0.0415 vs 0.0422). Not gated and not yet at the ADR targets (next PR, wheel
slip tuning): odom drift 12.5 % of distance and 31 deg/rev in the E-01 lap, 1.0 % on the
straight E-04 run.

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
