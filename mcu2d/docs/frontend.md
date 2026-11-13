# mcu2d front end: deskew, motion prediction, scan-to-submap odometry

Implemented in `mcu2d/src/{motion,frontend}.cpp` (contracts in `mcu2d/include/mss2d/
{motion,frontend,lidar_source}.hpp`). Tested by `mcu2d/tests/{motion,frontend}_test.cpp` on
the host simulation in `mcu2d/sim/`. Builds on `docs/scan_matching.md` (matcher, submap,
degeneracy) and ADR-034.

Frames (`docs/conventions.md`): `w` map, `b` body = IMU, `l` LiDAR, `s` submap. `T_b_l` comes
from `Config::lidar`. SE(2) tangent `[x, y, yaw]`, right perturbation. The map frame `w` is
the body pose at the end of the first scan.

## 1. Data flow

`LidarSource` (RPLIDAR driver, recording or simulator) hands over one complete turn with
`t_start` and a per-point `t_offset_ns`; the main loop pushes it into the `Merger` with the
gyro and wheel samples. `FrontEnd` is the merger's `Sink`: `on_imu`/`on_wheel` store the
samples in the motion model's fixed history (256 gyro, 64 wheel samples), `on_scan` runs one
front-end step and sends a `ScanPose` (`t` = scan end = `Scan2d::key()`, `T_w_b`, status,
matcher diagnostics) to the `PoseSink`. The merger delivers IMU up to the first sample at or
after the scan end before the scan, so the gyro covers the whole turn.

## 2. Motion model

Yaw rate `w(t) = gyro_z(t) - bias`, linear between gyro samples (nearest sample outside the
stored span). Body velocity `v(t) = (wheel(t), 0)` (no side slip; the wheel speed is that of the
body origin) when a wheel sample lies within `motion.wheel_timeout_s` of `t` and
`motion.use_wheel` is set, otherwise the constant body velocity `(vx, vy)` of the last two
scan poses:

    tw = Log(T_w_b(k-1)^-1 T_w_b(k)) / dt,   (vx, vy) = tw[0..1]

The relative pose between two times is the product of constant-twist steps,

    T_b0_b1 = prod_k Exp([vx h, vy h, w h])       (midpoint of each sub-step, h <= integration_step_s)

which is exact for a constant twist (test `IntegratesTheBiasCorrectedConstantTwist`).

Gyro bias: after each scan matched with every direction constrained (status `kOk`),

    bias <- clamp(bias + gyro_bias_gain * (yaw_pred - yaw_matched) / dt, +-max_gyro_bias_rad_s)

where `yaw_pred` is the yaw change the gyro predicted between the two scan ends and
`yaw_matched` the one the matcher found. A first-order filter with gain 0.1 per scan converges
in a few seconds of driving (simulation: 0.0094 against a true 0.0100 rad/s).

## 3. Deskew

A rotating single-beam scanner samples each beam at its own time `t_i = t_start + t_offset_i`;
during a 5.5 Hz turn a robot turning at 1 rad/s rotates 10 deg, which bends the scan by up to
0.9 m at 5 m range. Every point is moved into the LiDAR frame at the scan end:

    p_le = T_le_l(t_i) p_li,   T_le_l(t) = T_l_b T_be_b(t) T_b_l,   T_be_b(t) = T_b0_be^-1 T_b0_b(t)

`T_b0_b(t)` is tabulated at `deskew_knots + 1` equally spaced times over the turn (16 by
default, from the motion model of section 2) and interpolated linearly in `(x, y, yaw)`
between knots, so the per-point cost is one interpolation and one rotation. With
`motion.deskew = false` every point is taken at the scan end. Test
`MovesEveryBeamIntoTheScanEndFrame`: noise-free scan while turning at 1 rad/s with an offset
LiDAR, every deskewed beam lands on its true world point within 5 mm (measured < 0.1 mm;
0.88 m without deskew).

## 4. Odometry step (`FrontEnd::on_scan`)

1. Prediction `T_w_b_pred = T_w_b(k-1) * T_b(k-1)_b(k)` from section 2.
2. Points: deskew (section 3) or plain polar-to-Cartesian, with the matcher's range filter.
3. Match against the older active submap from the guess `T_s_l = T_w_s^-1 T_w_b_pred T_b_l`
   (correlative search, then Gauss-Newton; `docs/scan_matching.md`).
4. Status:
   - `kPredicted`: too few usable points; the prediction is the output and the scan is not
     inserted.
   - `kDegenerate`: some Hessian directions are flagged. Gauss-Newton does not move along them,
     but the correlative stage may have; the correction `delta = Log(guess^-1 T_s_l)` loses its
     component along each flagged eigenvector `v_k` (`delta -= v_k (v_k^T delta)`), so the
     pose keeps the prediction along those directions.
   - `kOk`: the matched pose; only then is the gyro bias updated.
   - `kInitialised`: the first scan, which starts the first submap at the identity.
5. `T_w_b = T_w_s T_s_l T_l_b`; the scan is inserted into every active submap, then the submaps
   rotate (section 5) and the distance field of the matching submap is recomputed.

## 5. Submaps

At most two submaps are active. Each scan is inserted into both; the older one is matched
against. A new submap (origin at the body position, axes along `w`) starts when the newest
active one reaches half of `submap.scans_per_submap` scans or half of
`submap.max_distance_m` of body travel from its origin; the older active one is finished at the
full limit and the newer, about half full, takes over. Consecutive submaps therefore overlap by
half. Finished submaps stay in a fixed pool of `kSubmapPoolSize = 4` grids (2 active + 2
finished); when a new submap needs a slot and none is free, the oldest finished one is recycled
(`FrontEndStats::n_submaps_recycled`). The backend will take finished submaps from
`finished_submap(i)`. `Config::validate` keeps `max_distance_m` below half the grid side minus
the distance field clamp, so the robot stays well inside the grid.

## 6. Synthetic evaluation

`sim/frontend_sim.{hpp,cpp}` (host only): line-segment worlds, a piecewise constant-twist
trajectory (exact poses at any time), `SyntheticLidar` (a `LidarSource`: 720 beams per turn at
5.5 Hz, beam `i` sampled at `t_start + i / (720 * 5.5)` s from the pose at that time, 1 cm range
noise, LiDAR 0.10 m ahead of the body origin), a 200 Hz gyro with 0.01 rad/s bias and 0.005
rad/s noise, a 50 Hz wheel with +1 % scale error and 0.02 m/s noise, fixed seeds. Sensors go
through the `Merger` into the `FrontEnd` in timestamp order.

Loop scenario: a 2.5 m wide ring corridor around a 9 x 5 m block (14 x 10 m outside) with six
pillars and boxes; one counter-clockwise loop on the centre line at 0.5 m/s (37 m) with four
0.75 m radius turns (0.67 rad/s) and one full turn in place at 1 rad/s; 446 scans.
ATE: RMSE of the translation after aligning the first estimated pose to the first true pose
(SE(2), no scale). RPE: RMSE over pose pairs 10 scans (1.8 s) apart.

| run (seed 1) | ATE RMSE | ATE max | RPE trans | RPE rot |
| --- | --- | --- | --- | --- |
| deskew, wheel (default) | 0.064 m | 0.120 m | 0.009 m | 0.08 deg |
| no deskew, wheel | 0.230 m | 0.470 m | 0.042 m | 2.33 deg |
| deskew, no wheel, `degeneracy_eig_threshold` 10 | 0.084 m | 0.164 m | 0.015 m | 0.08 deg |

Seeds 1 to 5: ATE 0.053 to 0.084 m with deskew, 0.15 to 0.25 m without; RPE rotation 0.07 to
0.10 deg against 2.2 to 2.4 deg. Without deskew the matched yaw lags by about half a turn's
rotation whenever the robot turns. The tests assert ATE < 0.15 m, RPE < 0.025 m and 0.25 deg
with deskew, and ATE at least 1.5 times and RPE rotation above 1 deg without.

Degeneracy: the corridor walls are flagged along the corridor axis in about a third of the
loop scans (the default threshold 50; eigenvalues 10 to 50 there), and those scans follow the
wheel along the axis. In the featureless corridor of `corridor_scenario()` (4 m straight) the
final error is 6 mm in x and 3 cm in y with the flag; with the flag off, the scan locks onto
its own map along the axis and x stalls (ATE 2.2 m). Without a wheel the fallback is only the
constant-velocity twist, which cannot follow a speed change while the direction stays flagged
(loop with the default threshold, seeds 1 to 5: ATE 0.07 to 0.08 m for four seeds, 2.8 m for
seed 4). Without a wheel, keep
the threshold low (10 above) or accept the drift in long corridors.

Run it: `mss2d_frontend_eval [--out DIR] [--no-deskew] [--no-wheel] [--corridor] [--seed N]
[--eig THRESHOLD]` prints these numbers and, with `--out`, writes `DIR/estimate.tum` (map
frame) and `DIR/truth.tum` (world frame) as `t x y 0 0 0 qz qw`, for `eval/` or evo with
alignment.

## 7. Cost on the host

Per-scan `FrontEnd::on_scan` wall time on a desktop x86 core, Release build, loop scenario
(446 scans): median 3.1 ms, p95 3.2 to 3.4 ms, max under 8 ms. The matcher takes about 2.6 ms
of it (`HostTimingOf720BeamMatch`); deskew, two submap insertions and the distance field
recompute share the rest.

Static footprint (`sizeof`, bytes):

| object | bytes |
| --- | --- |
| `FrontEnd` (total) | 252 160 |
| of which 4 x `OccupancyGrid` (200 x 200, 5 cm) | 4 x 46 088 |
| of which `DistanceField` | 42 424 |
| of which `ScanMatcher` scratch | 8 236 |
| of which `PointCloud2d` | 8 196 |
| of which `MotionModel` (gyro and wheel history) | 7 760 |
| of which `DeskewTable` | 792 |
| `Merger` (rings: 2 scans, 512 IMU, 64 wheel) | 38 584 |
| `Scan2d` (driver buffer) | 12 304 |

About 300 KB in all, above the 280 KB SRAM first estimate of ADR-034: the two finished
submaps (92 KB) are the candidates for external memory, as are any further ones the backend
keeps.
