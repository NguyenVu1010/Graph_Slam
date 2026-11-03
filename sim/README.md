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
  The GNSS antenna sits at `p_b_g = [-0.12, 0.06, 0.15]` m (0.201 m, on top of the body box)
  in every overlay (ADR-011 amendment 2026-11-02, backlog L2); `run_integration.py` passes the
  overlay's lever arm to the synthetic generator too.
- `sim/worlds/<world>.sdf`: hand-written worlds (DART, 1 ms, no real-time lock, floor mu 0.8):
  `warehouse` (40 x 25 m, 6 m ceiling, model `shelf_run_15m`), `corridor` (60 x 2.5 m,
  3.0 m ceiling, 6 x 6 m end room), `outdoor` (120 x 120 m open ground, buildings, tree row,
  fence, bollards, grass and wet patches; see "Outdoor world and wheel slip"). The world frame
  is the ENU frame (spherical coordinates = the scenario `origin_lla`, tested).
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
The noise report gets `--render-table sim/noise/render/<world>.yaml` when that file exists.
Raw topics: `/lidar/points`, `/imu/data`, `/gnss/fix` (clean fix), `/ground_truth`,
`/tf_static`, `/clock`, `/joint_states` (wheel joints, every physics step = 1 kHz); MCAP, zstd
file compression, 100 MB splits, sim-time log stamps.

Wheel odometry: the bridge reads the encoders at `odom.rate_hz` (50 Hz) whatever the
joint-state rate. Wheel angles integrate between joint states and the ticks are read at the
first joint state on or after each point of a fixed 20 ms grid (the twist uses the actual time
since the previous read), so `/wheel/odom` is 50 Hz and the tick quantisation is one tick per window
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
adds its own error on top (depth-texture interpolation). That error depends on range, incidence
and the geometry the robot actually sees, so it is calibrated where it is used: per world,
along the scenario's trajectory.

A render table, `sim/noise/render/<world>.yaml` (tracked), holds the measured sigma per cell
(range bin x incidence bin) for that world and trajectory. The file records:
- world and scenario;
- UTC timestamp;
- Gazebo version and render engine;
- the LiDAR block of `robot.yaml` and the declared sigma;
- raw bag SHA256, the selection, the bins, and the point count of every cell.

The table is a measured property of gpu_lidar on that world and trajectory, not a tuned value.
Re-run the calibration when the world, the trajectory, the sensor (LiDAR block, declared sigma),
the render engine or the Gazebo version changes.

Procedure:

```
tools/calibrate_lidar_render.sh sim/scenarios/E-01.yaml results/<run-id>/ [--render-engine ogre2]
cp results/<run-id>/render_warehouse.yaml sim/noise/render/warehouse.yaml   # printed by the script
```

The script records the scenario once with the Gazebo LiDAR noise at 0
(`record_scenario.sh --lidar-gz-sigma 0`; same scenario, seed, robot and driver). It then runs
`tools/calibrate_lidar_render.py` on the raw bag. The raw bag has no bridge noise, and Gazebo
renders each cloud at one instant, so the residual is the renderer's error alone. The script
writes only under `results/`; installing the table is the copy shown above.

Measurement, the same in calibration and in the noise report (`noise_measure`):
- **Residual:** along-ray residual to the nearest sidecar plane.
- **Range used for binning:** the noise-free plane range (`plane_ranges`: the range from the
  LiDAR to the ray's intersection with that plane). Binning on the measured range would select
  points on their own error at a crowded bin edge.
- **Silhouette points are excluded:** a grid neighbour (±1 column, ±1 ring) lies more than
  `lidar.edge_jump_m` = 0.10 m (5 declared sigma) from the point's plane, or has no return, or
  was dropped / is below the minimum range. A dropped neighbour counts as an edge so that the
  same points are selected with dropout (report) and without it (raw calibration bag). A plain
  range jump would flag oblique planes: adjacent beams differ by r·dphi·tan(theta), which is
  0.45 m at 30 m and 60 deg. Tested at 65-70 deg: < 1 % flagged.
- **Low incidence:** points with `|n.u|` < `lidar.min_incidence_cos` = 0.3 are excluded.
- **Outliers:** cut per cell at 5 x the cell sigma. The report uses the expected sigma. The
  calibration starts at 5 declared sigma (0.10 m) and iterates once on its own estimate. This
  cut is symmetric on both sides, so it does not truncate a wide cell at a fixed 0.10 m.

Gating:
- The expected value per cell is `sqrt(sigma_declared^2 + sigma_render(cell)^2)`, gated at
  ±15 % when the cell has at least 1000 points.
- A cell without a table entry is shown but not gated.
- Without any table the render error is taken as 0 and every cell is gated.
- A row fails when no cell could be gated.
- The table is validated on load: non-empty, same world as the planes sidecar, same bins,
  cells on the grid, n > 0, finite sigma >= 0.
- `record_scenario.sh` passes the world's table when it exists. The pooled row is
  informational.

Calibration, 2026-10-27 UTC, gz-sim 8.15.0, ogre2, on the raw bags recorded for it
(`sigma_render` in m, point count in brackets):

| World (scenario) | range | 0.3-0.5 | 0.5-0.7 | 0.7-0.9 | 0.9-1.0 |
| --- | --- | --- | --- | --- | --- |
| warehouse (E-01) | 10-20 m | - | 0.0312 (7 889) | 0.0177 (143 014) | 0.0082 (406 742) |
| warehouse (E-01) | 20-40 m | - | 0.0522 (1 700) | 0.0227 (13 747) | 0.0112 (102 181) |
| corridor (E-04) | 0.5-2 m | 0.0076 (2 402) | 0.0019 (458 671) | 0.0016 (1 623 639) | 0.0008 (1 898 004) |
| corridor (E-04) | 2-5 m | 0.0151 (851 986) | 0.0054 (697 078) | 0.0037 (479 432) | 0.0019 (508 324) |
| corridor (E-04) | 5-10 m | 0.0310 (10 119) | 0.0136 (10 506) | 0.0053 (4 616) | 0.0039 (43 952) |
| corridor (E-04) | 10-20 m | - | - | - | 0.0035 (28 569) |
| corridor (E-04) | 20-40 m | - | - | - | 0.0020 (6 572) |
| outdoor (OUT-01) | 10-20 m | 0.0842 (3 069) | 0.0470 (1 239) | 0.0183 (120 593) | 0.0095 (183 993) |
| outdoor (OUT-01) | 20-40 m | 0.1227 (74 286) | 0.0561 (285 882) | 0.0304 (463 025) | 0.0160 (503 251) |
| outdoor (OUT-01) | 40-100 m | 0.1497 (13 431) | 0.0780 (75 606) | 0.0468 (72 840) | 0.0252 (36 801) |

The outdoor rows were calibrated on 2026-11-02 UTC (same procedure) along the OUT-01 route; E-07
uses the same world but another route, so its cells are only an approximation (re-run on E-07
before gating it).

That is 6 warehouse cells and 14 corridor cells. The renderer error grows toward grazing
incidence and with range, and differs between scenes at the same range and incidence. Earlier
attempts to describe it with one formula failed: a constant, `k r`, `k (1/|n.u| - 1)` and
`k tan(theta)` all left residuals of about 0.01 m against bin errors of 1e-3 m or less. Timing
was ruled out: a driven and a static run with the Gazebo noise at 0 gave the same sigma, and a
-6..+6 ms sweep of the LiDAR stamps had its minimum at 0. The calibration recordings and the
tested bags are separate Gazebo runs of the same scenario (Gazebo is not deterministic).

### Wheel contact calibration

Odometry with the true kinematics (so the believed-kinematics error is excluded), 50 Hz
encoder, against `/ground_truth`: a 10 m straight at 0.5 m/s and a spin in place of 2 turns at
0.5 rad/s, robot at the warehouse spawn, one change at a time (2026-10-27 UTC):

| Wheel collision | straight 10 m | spin in place |
| --- | --- | --- |
| cylinder 0.04 m wide (first recording) | -0.04 % | -36.25 deg/rev |
| cylinder 0.01 m wide | -0.04 % | -9.25 deg/rev |
| sphere, wheel radius (shipped) | -0.01 % | -0.08 deg/rev |

A cylinder contacts the floor at a rim, an effective track of ~0.36 m instead of 0.40 m; the
error scales with the width. A sphere of the wheel radius contacts in the wheel's mid-plane
(`robot.yaml` `wheel.collision_shape: sphere`; the visual stays a cylinder). The spheres widen
the collision footprint to y = ±0.30 m (visual ±0.22 m) and touch obstacles up to 0.20 m high;
every route keeps at least 0.5 m between this footprint and the world boxes (tested). On the E-01 lap
with the true kinematics, straight segments read 0.00 % before and after; turns read
-35.87 deg/rev before and +0.00 deg/rev after. WheelSlip (compliance 0) adds no slip, and it
can only add slip.

### Outdoor world and wheel slip

Backlog L3 / S1, ADR-011 amendment 2026-11-02. `sim/worlds/outdoor.sdf`: 120 x 120 m of flat
ground in seven non-overlapping tiles (top z = 0): concrete, a 30 x 30 m grass field
(x 10..40, y -40..-10) and a 12 x 30 m wet patch (x -20..-8, same y). Structure: three
buildings (9-15 m high), a row of eleven 4 m trunks along y = -5 (x = -40..40, every 8 m), a
100 m fence (1.2 m) at y = -50 and bollards along x = +-52. No ceiling: rays that miss everything return nothing.

Surfaces (`outdoor.planes.yaml` `surfaces`, `sim/mss_sim/surface.py`):

| Surface | mu (SDF tile) | WheelSlip compliance (long. = lat.) | Basis |
| --- | --- | --- | --- |
| concrete (default) | 0.8, quoted | 0, chosen (no added slip, as the warehouse) | dry concrete peak 0.80-0.90 [1] |
| grass | 0.5, chosen | 0.4 = 0.2 / 0.5, derived | between the two quoted ends: dry earth road 0.68 peak [1] and wet grassy field 0.20 [3] |
| wet patch | 0.3, chosen | 0.67 = 0.2 / 0.3, derived | wet grassy field 0.20 [3] and wet earth road 0.40-0.50 sliding [1] |

The friction values are those of ADR-011 section 4; only the concrete value is a quoted number,
grass and wet are choices inside the quoted ranges. The gz WheelSlip compliance is the slip ratio
per tangential / normal force ratio. It is set as the secant to the friction peak: the peak
occurs near 20 % slip [1], so `c = 0.2 / mu` (a choice; the real slip curve is not linear).
`sim/mss_sim/surface_node.py` (started by `sim.launch.py` when the sidecar has `surfaces`) feeds
the ground truth to `surface.SurfaceCommander`, which sends the compliances of the patch under
the body origin once per surface change (logged with its sim time; retried every 1 s of sim time
until acknowledged). The request goes to the `UserCommands` service
`/world/<world>/wheel_slip/blocking` through the `gz service` CLI (`mss_sim/gz_cli.py`, also used
by `live_start`), so no Gazebo Python bindings are needed. It is a service, not a topic: a
publisher on `/world/<world>/wheel_slip` has no subscriber in gz-sim 8.15.

Sources (the page numbers are as given by the secondary source; not checked against the books):
1. J. Y. Wong, *Theory of Ground Vehicles*, 2nd ed., Wiley, 1993, p. 26 (peak and sliding
   friction of tyres per surface; peak near 20 % slip), as tabulated on hpwizard.com, "Tire
   friction and rolling resistance coefficients".
2. Robert Bosch GmbH, *Automotive Handbook*, 4th ed., 1996, p. 330 (rolling resistance: car tyre
   on concrete or asphalt 0.013, unpaved road 0.05, field 0.10-0.35), same secondary source.
3. R. K. Noon, *Engineering Analysis of Vehicular Accidents*, CRC Press, 1994 (wet grassy field
   0.20), as cited by P. Cenek et al., "Frictional characteristics of roadside grass types",
   Australasian Road Safety conference paper (saferroadsconference.com).

**Limit (finding, backlog L22).** Gazebo contact has no rolling resistance and the casters are
frictionless, so a wheel carries tangential force only while the robot accelerates or turns.
Slip compliance acts on that force alone: on straight driving at constant speed it adds no slip.
ADR-011's grass target (3-6 % odometry drift) is therefore not reached. Quoted rolling
resistance [2] is 0.013 on concrete, 0.05 on an unpaved road and 0.10-0.35 in a field (no grass
value is given; a grass field would sit in the field range); modelling it needs a drag force
on the body per surface (e.g. the `ApplyLinkWrench` system), which is not added here.

OUT-01 (`sim/scenarios/OUT-01.yaml`, `config/sim/outdoor.yaml`: GNSS and wheel odometry on):
170 m at 0.8 m/s, east along y = -15 across the wet patch and the grass, back along y = -25, and
north to the spawn. Recorded 2026-11-02 UTC (seed 42, gz-sim 8.15, ogre2): 234 s sim in 125 s
wall (real-time factor 1.87), raw bag 199 MB in 10 files; a second slip recording through the
`gz service` CLI (234 s in 122 s, noise report exit 0) shows how well a slip run reproduces.
A control recording with the compliances not applied (the first run, before the service fix)
differs in the surface commands and in Gazebo's own nondeterminism. Noise report with the outdoor render table: exit 0 (12 gated LiDAR cells; GNSS sigma_h
1.016 / sigma_v 1.879 m; odometry drift 1.82 % of 168.7 m, yaw 4.60 deg/rev).

Wheel-odometry residuals against ground truth, per `/wheel/odom` message (`r_v` speed,
`r_lat` lateral body velocity the odometry assumes 0, `r_w` yaw rate; RMS, m/s and rad/s):

| Surface, phase | n | slip run 1: r_v / r_lat / r_w | slip run 2 | control |
| --- | --- | --- | --- | --- |
| concrete, straight | 5295 | 0.0128 / 0.0001 / 0.0086 | 0.0128 / 0.0001 / 0.0088 | 0.0129 / 0.0001 / 0.0033 |
| concrete, turns | 190 | 0.0126 / 0.0038 / 0.0130 | 0.0125 / 0.0038 / 0.0123 | 0.0124 / 0.0038 / 0.0138 |
| grass, straight | 3320 | 0.0131 / 0.0003 / 0.0099 | 0.0131 / 0.0003 / 0.0098 | 0.0131 / 0.0001 / 0.0072 |
| grass, turns | 372 | 0.0132 / **0.0096** / 0.0201 | 0.0132 / **0.0096** / 0.0198 | 0.0131 / 0.0038 / 0.0126 |
| wet, straight | 1498 | 0.0133 / 0.0000 / 0.0098 | 0.0133 / 0.0000 / 0.0099 | 0.0134 / 0.0000 / 0.0000 |

What the slip shows, and what is noise:
- **Lateral residual (the slip evidence).** In the grass turns it is 2.5x the concrete turns of
  the same run (0.0096 against 0.0038 m/s, max 0.016 m/s), in both slip runs, and 2.5x the
  control's grass turns.
- **Yaw rate: not evidence.** On straight driving the yaw-rate residual is the encoder
  quantisation. One tick on one wheel in a 20 ms window is 2 pi 0.101 / 4096 / 0.02 / 0.399 =
  0.0194 rad/s (believed radius 0.101 m and track 0.399 m, `odom.ticks_per_rev` 4096 and the
  50 Hz `odom.rate_hz` of `sim/noise/default.yaml`), and the ground-truth yaw rate there is exactly 0 (on the wet straight in 100 %
  of the messages: the robot drives straight with no rotation in DART). So the RMS is
  0.0194 sqrt(p), with p the share of windows where the two wheels differ by a tick: p = 0.26
  on the wet straight of run 2 gives 0.0098, as measured. In the control's wet straight p = 0,
  hence exactly 0.0000. The two slip runs reproduce each other to 2 % (grass turns 0.0201 /
  0.0198, concrete straights 0.0086 / 0.0088). The control differs even on the concrete
  straights (0.0033), where the compliance is 0 in both configurations, so the yaw-rate
  difference in the grass turns (0.020 against 0.013) cannot be attributed to grass slip; it is
  dominated by encoder quantisation.
- **Speed.** The speed residual and the travelled-distance error (+1.0 % on every surface, the
  believed wheel radius) do not change. The speed NIS (declared 0.02 m/s) stays near 0.43, with
  0.1-0.3 % of messages above the 1-DoF 0.95 bound (3.84).

The pipeline has no chi-square gate on wheel-odometry factors (only the time check), so slip
enters the graph unflagged (`n_odom_rejected` 0).

Pipeline (`run_integration.py --bridged`, then `mss_bench` with one overlay change; ATE after
SE(3) alignment, 2333 pairs; return = the start revisited after 216.7 s, relative displacement
estimate vs truth):

| Run | ATE [m] | horiz. / vert. RMS [m] | return horiz. / vert. [m] | GNSS factors | GNSS chi-square rejections | p50 / p95 / max scan [ms] |
| --- | --- | --- | --- | --- | --- | --- |
| slip, shipped | 0.375 | 0.112 / 0.358 | 0.326 / 0.368 | 959 | 80 of 1177 | 9.4 / 22.3 / 48.3 |
| slip, `imu.init_level_start` | 0.208 | 0.101 / 0.181 | 0.319 / 0.356 | 976 | 57 | 8.5 / 19.1 / 27.3 |
| slip, level start, wheel off | 0.234 | 0.092 / 0.216 | 0.313 / 0.392 | 1038 | 57 | 8.5 / 20.9 / 40.9 |
| control, shipped | 0.395 | 0.107 / 0.380 | 0.277 / 0.392 | 957 | 74 of 1167 | 8.4 / 19.0 / 30.7 |
| control, level start | 0.207 | 0.098 / 0.183 | 0.299 / 0.380 | 971 | 55 | 8.3 / 18.7 / 23.9 |
| control, level start, wheel off | 0.297 | 0.080 / 0.286 | 0.294 / 0.347 | 981 | 53 | 8.2 / 20.0 / 29.3 |

The estimator handles this much slip: horizontal ATE moves by 0.003-0.012 m. The vertical part
is the IMU-bias tilt of S2 (halved by the level start) and the GNSS vertical noise. Wheel odometry
makes the horizontal error 0.01-0.02 m larger on this route with or without slip. The scan p95
(19-22 ms) is about 2.5x the warehouse's; the bench numbers are from the desktop under WSL. The
control bench run exits 3 (3 scans dropped by the bench queue). These are not baselines.

### E-07 on the outdoor loop

Backlog C1-4, C1-14. `sim/scenarios/E-07.yaml` drives `sim/trajectories/outdoor_loop.yaml`: a
312 m lap plus a 30 m revisit (342 m in all; the polyline is 314.6 + 30 m, the pursuit cuts the
corners) around the building block (east along y = -15 across the wet patch and the
grass, north along the feature-poor x = 45, a slalom of eight turns along y = 46..50, south along
x = -45), then 30 m past the start. GNSS is off in both overlays (`config/sim/outdoor_e07.yaml`,
loops off; `outdoor_e07_loops_on.yaml`, `loop_closure.enabled` with PCM on). The profile is the
default MPU-class one (no inflated noise).

Two loop scenes, two purposes:
- **E-07** (outdoor, this section): drift in metres, so it tests that loop closure removes real
  drift.
- **E-07W** (`sim/scenarios/E-07W.yaml`): the former E-07, the warehouse lap driven three times
  (`warehouse_3laps.yaml`, overlay `warehouse_e07.yaml`, unchanged except the L2 lever arm). It
  drifts about 4 cm, so it only tests that loops are found in structured indoor space and do no
  harm. Re-recorded at main bf2f7cd (H6, "Re-recorded with the level start" below): `kf_opt`
  ATE 0.0192 m with loops off, 0.0142 m with the loops-on overlay (16 loops, none false). The
  numbers of the pre-#41 bag (ATE 0.087 m, 4.7 / 1.5 cm) are superseded.

Recorded 2026-11-02 UTC on this branch (body frame `imu_link`, after #41; seed 42, gz-sim 8.15,
ogre2): 452 s sim in 241 s wall (real-time factor 1.88), raw bag 375 MB in 19 files. Noise report
exit 0 with the outdoor render table (12 gated cells; GNSS sigma_h 1.004 / sigma_v 1.916 m;
odometry drift 0.87 % of the 342 m driven, yaw 4.51 deg/rev).

Pipeline (`mss_bench`, same bag, one overlay each; ATE after SE(3) alignment, 4512 pairs;
"return" = the start revisited at 392 s, before any loop is accepted; "final" = the last pose,
30 m later, after the loops; both are the start-to-pose displacement in the start frame,
estimate vs truth; est.tum is the online estimate; the return error is divided by the 312 m
driven up to the return. That distance is the route's arc length at the return in the
`test_pursuit` unicycle replay of the same route and seed (310.8 m at 392 s, 312.3 m at the
closest pass); the recordings were deleted after the run, so it is not the ground-truth arc
length):

| Run | ATE [m] | horiz. / vert. RMS [m] | return horiz. [m] (% of 312 m) | final horiz. / vert. [m] | searches / accepted / rejected / PCM-rejected / removed | p50 / p95 / max scan [ms] |
| --- | --- | --- | --- | --- | --- | --- |
| loops off | 1.213 | 1.035 / 0.633 | 3.68 (1.18 %) | 2.69 / 0.82 | - | 7.5 / 15.0 / 25.4 |
| loops on (PCM) | 1.190 | 1.007 / 0.634 | 3.68 | 2.25 / 0.83 | 316 / 2 / 4 / 0 / 0 | 7.4 / 15.1 / 74.5 |
| loops off, `imu.init_level_start` | 0.961 | 0.745 / 0.606 | 2.77 (0.89 %) | 1.97 / 1.04 | - | 7.2 / 15.4 / 26.9 |
| loops on, level start | 0.888 | 0.678 / 0.574 | 2.77 | 0.39 / 0.71 | 316 / 5 / 1 / 0 / 0 | 7.1 / 15.0 / 75.5 |

The drift is now in metres: 3.7 m horizontal at the revisit with loops off. Loop closure with the
shipped thresholds removes little of it. The drift at the revisit is above `max_correction_m`
(3 m), so the first candidates fail that gate; the two accepted loops carry corrections of
2.8-3.0 m, and the Cauchy kernel with the 5 cm / 1 deg floor applies only a small part of them
(C1-13). With the level start the drift is 2.8 m: five loops are accepted (corrections
0.9-2.1 m) and the final horizontal error drops from 1.97 to 0.39 m. The vertical error
(0.6-1.0 m) is the S2 tilt plus no GNSS. A loop keyframe costs up to 75 ms (C1-15).
These are not baselines.

#### Re-recorded with the level start (H6)

Both loop scenes were recorded again at main bf2f7cd (body frame `imu_link`, L2 lever arm; seed
42, gz-sim 8.15, ogre2), one at a time under the build lock: recording A on 2026-11-02, B and C
on 2026-11-03 UTC (04:40 and 06:11) for the review rounds; both overlays set `imu.init_level_start` (ADR-027; owner ruling S5: on
in the loop scenes only, `config/default.yaml` keeps it off). E-07W gained a loops-on variant
(`warehouse_e07_loops_on.yaml`). These recordings replace the H5 one above and the pre-#41 E-07W
bag. Gazebo is not deterministic, so each recording drifts differently.

| scene, recording | sim s / wall s (RTF) | raw bag | noise report |
| --- | --- | --- | --- |
| E-07, C | 454 / 235 (1.93) | 377 MB, 19 files | exit 0, 12 gated cells; GNSS sigma_h 1.004 / sigma_v 1.919 m; odometry 0.87 % of distance, 4.51 deg/rev |
| E-07W, C | 172 / 83 (2.07) | 232 MB, 8 files | exit 0, 6 gated cells; GNSS 1.023 / 1.849 m; odometry 1.74 %, 4.51 deg/rev |
| E-07, B / A | 454 / 236, 453 / 261 | 377, 376 MB | exit 0; the same odometry figures |
| E-07W, B / A | 176 / 86, 172 / 88 | 238, 231 MB | exit 0 |

E-07 and E-07W are scored on `kf_opt.tum` (`mss_bench`: the stored keyframes at their optimised
poses after the replay; owner ruling), E-01..E-03 on `est.tum`. Recording C, `tools/loop_eval.py`
on each run (commands: `docs/derivations/loop_closure.md` section 15); ATE after SE(3) alignment with its number of pairs;
return = the start revisited at 394 s; final = the last pose; horizontal errors of the
start-to-pose displacement in the start frame; loops: searches / accepted / rejected /
PCM-rejected / removed; latency per scan, desktop WSL:

| run | ATE `kf_opt` [m] (pairs) | ATE `est` [m] (pairs) | return / final h `kf_opt` [m] | loops | p50 / p95 / max [ms] |
| --- | --- | --- | --- | --- | --- |
| E-07 loops off | 1.016 (337) | 1.099 (4525) | 3.68 / 2.84 | - | 7.4 / 15.5 / 23.4 |
| E-07 shipped loop settings | 0.870 (337) | 1.070 (4525) | 3.17 / 2.44 | 315 / 2 / 3 / 0 / 0 | 7.3 / 15.1 / 52.0 |
| E-07 loops-on overlay | 0.152 (337) | 1.001 (4525) | 0.042 / 0.17 | 312 / 5 / 0 / 0 / 0 | 7.3 / 15.2 / 66.7 |
| E-07W loops off | 0.0192 (135) | 0.0163 (1708) | - / 0.009 | - | 5.6 / 9.8 / 18.2 |
| E-07W shipped loop settings | 0.0167 (135) | 0.0205 (1708) | - / 0.045 | 56 / 16 / 0 / 0 / 0 | 5.2 / 8.5 / 24.0 |
| E-07W loops-on overlay | 0.0142 (135) | 0.0151 (1708) | - / 0.021 | 56 / 16 / 0 / 0 / 0 | 5.2 / 8.6 / 22.2 |

The loops-on overlays carry the loop-scene settings of ADR-030 (floor 2 cm / 0.25 deg and kernel
release in both; place search and `max_correction_m` 6 in E-07 only). No false loop in any run
(every accepted loop within 5.5 cm and 0.23 deg of the truth). The level start does not rank
reliably: the drift at the return of E-07 with / without it is 3.1 / 4.1 m on B, 4.0 / 2.6 m on
A, 2.8 / 3.7 m on the H5 one, and 3.7 m with it on C (backlog C1-32, a multi-seed study); the
loops-on overlay closes every one measured (`kf_opt` ATE 0.152 to 0.164 m). Diagnosis, NEES, each setting
alone and the cost: `docs/derivations/loop_closure.md` sections 13 to 15. These are not
baselines. The numbers were measured on the review branch at ddc9682, before the tilt-keyed descriptor cache refresh (review N2-1); E-01..E-03 with the flags off were md5-checked against main at this branch's tip; E-07 and E-07W are to be re-run on the next recording (backlog C1-37).

### Máy bàn

Dev machine: WSL Ubuntu 24.04, 9 GB RAM, ROS 2 Jazzy, gz-sim 8.15.0 (vendor packages under
`/opt/ros/jazzy/opt`), headless `ogre2` works under WSLg (no fallback to `ogre` needed). The WSL
system Python has no `rosbags`/`shapely`/`allantools` and no `python3-venv`; without sudo they
were installed with `pip install --no-deps --target <dir> rosbags shapely allantools zstandard
lz4 ruamel.yaml backports.zstd apsw` and that dir put on `PYTHONPATH`. With sudo:
`sudo apt-get install python3.12-venv` and a venv with `--system-site-packages` (rclpy).
Gazebo, if missing: `sudo apt-get install ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap`.
Image alternative: `docker/sim.Dockerfile` (usage in its header).

Numbers of the final recording (2026-10-27 UTC, `results/<run-id>` on `/mnt/c`, seed 42, sphere
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

LiDAR range sigma per cell on the recorded bags, with the world's render table. Expected value:
`sqrt(0.02^2 + sigma_render(cell)^2)`, gated at ±15 % with at least 1000 points after the
outlier cut. Excluded shares:
- E-01: 79.6 % silhouette (mostly points on shelves, boxes and columns, which are not plane
  points) and 12.3 % low incidence.
- E-04: 14.9 % silhouette (dropped neighbours now count) and 13.7 % low incidence.

| World | range | `\|n.u\|` | points | expected | measured | result |
| --- | --- | --- | --- | --- | --- | --- |
| E-01 warehouse | 10-20 m | 0.5-0.7 | 6 178 | 0.0371 | 0.0375 (+1.3 %) | pass |
| E-01 warehouse | 10-20 m | 0.7-0.9 | 120 588 | 0.0267 | 0.0266 (-0.4 %) | pass |
| E-01 warehouse | 10-20 m | 0.9-1.0 | 352 060 | 0.0216 | 0.0216 (0.0 %) | pass |
| E-01 warehouse | 20-40 m | 0.3-0.5 | 171 | - | 0.0445 | shown (no calibrated cell) |
| E-01 warehouse | 20-40 m | 0.5-0.7 | 1 090 | 0.0559 | 0.0551 (-1.4 %) | pass |
| E-01 warehouse | 20-40 m | 0.7-0.9 | 11 675 | 0.0303 | 0.0301 (-0.6 %) | pass |
| E-01 warehouse | 20-40 m | 0.9-1.0 | 86 130 | 0.0229 | 0.0229 (-0.1 %) | pass |
| E-04 corridor | 0.5-2 m | 0.3-0.5 | 26 475 | 0.0214 | 0.0224 (+4.7 %) | pass |
| E-04 corridor | 0.5-2 m | 0.5-0.7 / 0.7-0.9 / 0.9-1.0 | 0.42-1.69 M | 0.0201 / 0.0201 / 0.0200 | +0.1 / +0.2 / 0.0 % | pass |
| E-04 corridor | 2-5 m | 0.3-0.5 / 0.5-0.7 / 0.7-0.9 / 0.9-1.0 | 0.43-0.76 M | 0.0250 / 0.0207 / 0.0203 / 0.0201 | +0.5 / +0.1 / -0.4 / -0.3 % | pass |
| E-04 corridor | 5-10 m | 0.3-0.5 / 0.5-0.7 / 0.7-0.9 / 0.9-1.0 | 4 023-38 714 | 0.0369 / 0.0242 / 0.0207 / 0.0204 | 0.0 / -0.7 / 0.0 / -0.6 % | pass |
| E-04 corridor | 10-20 m | 0.9-1.0 | 24 218 | 0.0203 | 0.0202 (-0.7 %) | pass |
| E-04 corridor | 20-40 m | 0.9-1.0 | 4 535 | 0.0201 | 0.0205 (+2.2 %) | pass |

Exit codes: E-01 0 (6 gated cells), E-04 0 (14 gated cells); every gated row passes in both. The
largest deviation is +4.7 % (E-04 0.5-2 m x 0.3-0.5). That cell read -14 % when cells were
binned on the measured range: corridor floor points crowd its 2 m edge, so the measured-range
selection truncated their error one-sidedly. `odom_errors`: `stamp_not_increasing` only (8 on
E-01, 22 on E-04), the duplicate joint-state stamps after the final pause. The odometry meets
ADR-011 section 3, with the believed kinematics (1.01 radius, 0.399 track) as the only error.

#### Pipeline on the Gazebo bag

`tools/run_integration.py --bridged <record_scenario out_dir>` replaces the synthetic data by a
recorded bridge output (ENU origin = scenario `origin_lla`); the rest of the chain
(`bag_to_mssdata.py`, `mss_bench`, `mss_eval.ate`) takes the Gazebo bag unchanged. Bench:
WSL build of this branch (`tools/wsl/build.sh --target mss_bench`), desktop, 2026-10-27 UTC.

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
from the LiDAR + IMU part. Cause (backlog S2): the IMU's horizontal accelerometer bias, read
as a tilt `b / g` by the static initialisation, so the map is built tilted until the first
turn; `imu.init_level_start` (ADR-027, default off) removes it (E-01 ATE 0.095 -> 0.015 m on
a seed-42 recording), see `docs/derivations/vertical_drift_study.md`.

Diagnostics for the pipeline follow-up (aligned error over time, RMS per window):

| Run | 0-10 s horiz. / vert. | 10-30 s | 30-50 s | 50 s-end |
| --- | --- | --- | --- | --- |
| E-01 LiDAR + IMU | 0.016 / 0.109 (mean z +0.099) | 0.021 / 0.122 (-0.037) | 0.012 / 0.070 | 0.008 / 0.055 |
| E-04 degeneracy on | 0.512 / 3.757 (mean z -3.714) | 0.207 / 2.071 (+1.054) | 0.197 / 1.856 (+1.780) | 0.406 / 1.490 (-1.062) |

In E-01 the vertical error is largest early, with an offset of +0.10 m in the first 10 s, and it
decreases later. E-04 with degeneracy on drifts vertically by metres while the horizontal error
stays near 0.2-0.5 m. The bench exports no degeneracy counters; its log flags 47 of 53 keyframes
degenerate (`add_lidar_factor ... degenerate`), without the directions. Both patterns have the
same cause, the bias read as tilt (`docs/derivations/vertical_drift_study.md` sections 2 and 5):
E-01's offset holds until the first turn, and E-04's lift with degeneracy on drops from metres
to 0.13-0.55 m with `imu.init_level_start`.

## Map demo

One command from a scenario to a top-down picture of the global map with the estimated and
true trajectories (WSL or Ubuntu with Gazebo, see "Máy bàn"):

```
tools/run_map_demo.sh sim/scenarios/E-01.yaml results/<run-id>/     --bench-cmd "env LD_LIBRARY_PATH=$HOME/opt/gtsam/lib <build>/bench/mss_bench"     [--bag results/<earlier run>/] [--z-min-m 0.3] [--z-max-m 2.0] [--res-m 0.05]
```

It chains the existing tools: `record_scenario.sh` (Gazebo, then the noise bridge) into
`<run-id>/record/`, then `run_integration.py --bridged ... --tests E-01 --map-out`
(`bag_to_mssdata.py`, then `mss_bench ... --map-out run/E-01/map.pcd`, then ATE) into
`<run-id>/run/`, then `python -m mss_eval map-png` (eval/README.md "Map picture") into
`<run-id>/map.png`. `--bag <dir>` takes an earlier `record_scenario.sh` output of the same
scenario (`bag/` + `gt.tum`) and skips the recording. `map_demo.json` holds the scenario id,
the E-01 gate verdict (`gate`), the wall time per
step, the E-01 ATE and bench counters, the bench's `map:` line and the map-png result. The
bench output sits in `run/E-01/` for every scenario: that is the harness's test slot, not the
scenario.
The picture uses the scenario's `pipeline_config` (for E-04 the degeneracy-off one). The E-01
gate inside `run_integration.py` compares against the synthetic baselines, so a Gazebo bag
"fails" it on the ATE; the demo records that in `map_demo.json` (`gate`) and still draws the
map. `gate.gate_passed` is the E-01 ATE gate only; a PF-01 latency failure is recorded apart
(`gate.pf01_passed`). A failed or timed-out bench (E-01 `bench_exit` not 0) is not accepted:
exit 2. Exit 0 = picture written, 2 = a step failed (the message names its log). Wrap it in
`flock ~/.mss-build.lock` when another heavy job may run.

Numbers (2026-10-29 UTC, desktop WSL, Release build of `feat/map-demo`, seed 42, fresh
recordings, so they differ from the tables above; Gazebo is not deterministic):

| Scenario | wall s: record / convert + bench / picture | map points (in the z cut) | keyframes | ATE RMSE [m] (pairs) |
| --- | --- | --- | --- | --- |
| E-01 warehouse | 68.3 / 13.7 / 0.7 | 48 000 (15 618) | 42 | 0.092 (609) |
| E-04 corridor | 93.9 / 16.3 / 0.6 | 30 815 (15 912) | 48 | 0.249 (737) |

The record time includes the noise report. In the E-04 picture the corridor floor climbs
through the 0.3-2.0 m cut along the corridor: the map's vertical drift (median z of the
in-corridor points 0.36 m at x 8-16 m, 1.30 m at x 40-48 m). The climb is the tilt `b / g`
of the IMU bias read at initialisation (about `abs(b) / g` per metre; the height cut clips it);
`imu.init_level_start` removes it (`docs/derivations/vertical_drift_study.md`, ADR-027).

## Live

Gazebo, the noise layer, `mss_node` and RViz in one launch (ADR-011 amendment of 2026-10-29;
desktop as in "Máy bàn", with the colcon workspace of `mss_ros2` sourced and the `sim/`
Python dependencies on `PYTHONPATH`):

```
export ROS_DOMAIN_ID=<free id>   # other Gazebo runs use the same topic names
ros2 launch mss_ros2 mss_live.launch.py scenario:=sim/scenarios/E-01.yaml rviz:=true \
    [rtf:=1.0] [headless:=false] [seed:=N] [exit_on_end:=true]
```

- Assets are generated at launch into `results/live/<id>/assets` (`assets_dir:=`).
- Gazebo starts paused and publishes the raw sensor topics under `/sim/raw`
  (`sim.launch.py raw_prefix:=/sim/raw`). `python -m mss_sim.live_node` turns them into
  `/lidar/points`, `/imu/data`, `/gnss/fix` and `/wheel/odom` with the offline bridge's models
  (`NoiseLayer`), messages kept as CDR bytes. It holds each scan until `/ground_truth` covers its
  sweep (about 0.1 s sim) and logs its counters every 10 s and at exit.
- `python -m mss_sim.live_start` waits until mss_node and the noise node subscribe, sets the
  real-time factor (`rtf:=`), unpauses, and pauses the world at `duration_s` + 2 s of sim time.
  It exits 2 (no false success) when the sim clock is not within `--max-start-clock-s` (1 s)
  of 0 once the robot exists, i.e. not this run's Gazebo, or when the clock does not advance
  for `--stall-timeout-s` (10 s) of wall time or goes back.
- Isolation and shutdown: the launch runs Gazebo transport in its own `GZ_PARTITION`
  (`mss_live_<launch pid>`), and on shutdown `sim.launch.py` stops the gz processes below its
  own launch process (SIGINT, SIGKILL after 5 s); launch alone signals only the `ruby gz`
  wrapper, whose `gz sim` server child used to survive and break the next run.
  With `exit_on_end:=true` the launch then shuts down; otherwise it stays up for viewing.
- Determinism: only the seed is fixed. Fed the same raw messages, `mss_sim.live.LiveNoise` gives
  the offline bridge's bytes (`tests/python/test_live.py`); Gazebo itself is not deterministic.
  Use bags for numbers that must reproduce.

E-01, full run, 2026-10-29 UTC, desktop WSL (12 cores, 10 GB RAM), gz-sim 8.15, ogre2,
Release colcon build with GTSAM, `rtf:=1.0`, seed 42:

| Run | real-time factor | scans / IMU / GNSS / odom received by mss_node | dropped | poses | ATE `/mss/path` vs `/ground_truth` |
| --- | --- | --- | --- | --- | --- |
| headless, no RViz | 0.983 (61.4 sim s in 62.4 wall s) | 621 / 12442 / 311 / 3110 | 0 | 601 | 0.097 m |
| Gazebo GUI + RViz | 0.934 (61.5 sim s in 65.8 wall s) | 623 / 12465 / 312 / 3116 | 0 | 602 | 0.102 m |

"Dropped" compares what mss_node received (its exit summary) with what the noise node published:
equal on every topic, and no message was skipped by time. The noise node received every raw
message (joint states at 1 kHz, IMU and ground truth at 200 Hz without gaps); its last scan
waits for ground truth past the pause. Scan processing in the noise node: p50 13 ms, p95 16 ms
(headless), 19 / 25 ms with the GUI. mss_node: 42 keyframes, 0 dropped scans, 0 match failures.
The ATE is the estimator's on Gazebo data, as on the recorded bags (0.095 m in "Pipeline on the
Gazebo bag"); Gazebo runs differ, so the numbers do not match a bag run exactly.

CPU (average over the run, % of one core) and peak RSS:

| Process | headless | GUI + RViz |
| --- | --- | --- |
| Gazebo (`gz sim`) | 99 %, 373 MB | 222 %, 881 MB |
| noise node (`live_node`) | 77 %, 111 MB | 90 %, 111 MB |
| mss_node | 15 %, 206 MB | 23 %, 207 MB |
| RViz | - | 38 %, 241 MB |
| driver, `live_start`, `tf_static`, `parameter_bridge` | 42 / 39 / 29 / 35 % | 56 / 52 / 42 / 46 % |
| system memory used (max) | 1.6 GB | 1.7 GB |

`tf_static` does nothing after start-up and still uses 29-42 % of a core: every rclpy node on
sim time receives the 1 kHz `/clock` (driver and `live_start` likewise). The machine keeps up at
`rtf` 1.0, so the real-time factor was not lowered.

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
