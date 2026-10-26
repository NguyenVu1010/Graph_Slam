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
`SeedSequence(seed).spawn(5)` streams for [lidar, gnss, odom, imu, mutate]; the same raw bag,
scenario, profile and seed produce byte-identical messages and log times (SIM-05). `--seed`
overrides the scenario seed. Record the raw bag once and re-run the bridge to vary noise.

## Mutating a bag (E-03)

`tools/mutate_bag.py` derives the robustness bag from a bridged one (packet loss and reordering
inside a time window, seeded):

```
python tools/mutate_bag.py <in_bag> <out_bag> [--drop P] [--reorder P] [--window-ns N] [--seed N]
```

E-03 bags skip the noise report: reordering violates its ordering check by design. Run the
report on the E-01 bag the E-03 bag was derived from.

Tests: `python -m pytest tests/python`; lint: `ruff check . && ruff format --check .`.
