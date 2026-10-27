# map: voxel hash map (ADR-007)

`mss::map::VoxelMap` stores LiDAR points in the world frame `w`. No Jacobians live here; this
note fixes the rules that the implementation (`core/map/src/voxel_map.cpp`) and its tests
(`tests/unit/test_voxel_map.cpp`, `tests/component/test_voxel_map_component.cpp`) share.

## Key

For a point `p_w` (m), the voxel key per axis `a` is `key_a = floor(p_a / voxel_m)`, computed
with `std::floor` on a division (not a multiply by `1 / voxel_m`) and stored as `int32`.
`floor`, not truncation: `-0.1 m` with `voxel_m = 0.5` gives key `-1`. A point whose `floor`
value is non-finite or outside `[INT32_MIN + 1, INT32_MAX - 1]` is skipped by `insert`
(not counted as dropped), and such a query makes `knn` return 0, so `key +- 1` never
overflows.

## Rounding

`insert` computes `p_w = T_w_l * p_l` in double, rounds each coordinate to `float`, promotes
the rounded value back to double and takes the key from that. So every stored point lies
inside its own voxel, and `knn` returns the stored float coordinates promoted to double.

## Ring

A voxel holds `count <= N` points (`N = max_points_per_voxel`) in slots `0..N-1`. A new point
that does not replace a stored point (Spacing, next section; always, when
`min_point_dist_m = 0`) goes to slot `head`, then `head = (head + 1) % N` and `count = min(count + 1, N)`. When the
voxel is full the oldest point is overwritten. A new voxel when `num_voxels == max_voxels`
is not created: the point is dropped and `num_dropped` grows. Points for existing voxels are
still stored.

## Spacing (ADR-007 amendment, 2026-10-27)

`d = min_point_dist_m` (m, `>= 0`, `0` = off). Before a point `p` goes into an existing voxel
that holds `q_0..q_{count-1}`, `insert` computes `||p - q_i||^2` in double from the float
coordinates and takes the slot `i*` with the smallest value below `d^2` (strict `<`; on a tie
the lower slot). If there is one, `p` overwrites `q_{i*}` in place: slot, `count` and ring
`head` stay, `num_spacing_replaced` grows. If there is none, `p` goes into the ring as above
(a full voxel overwrites its oldest point). A point that creates a voxel is never tested.
Only the voxel's own points are compared: two points `d / 10` apart across a voxel face are
both kept.

Cost: at most `count <= N` distance evaluations (3 subtractions, 3 multiplications, 2
additions, 1 comparison each) per inserted point, no memory and no extra work when `d = 0`.
With `N = 20` that is 20 evaluations on a full voxel, against up to `27 * N = 540` per `knn`
query; and `insert` runs only on keyframe clouds (ADR-016).

Distribution. A point that enters through the ring is at least `d` from every stored point of
its voxel; a replacement moves a stored point by less than `d`. So stored points are not
guaranteed to stay `d` apart, but each place seen again within `d` keeps one slot, holding its
newest observation (freshness), and new slots are spent only on places at least `d` from
every stored point (coverage). A cube of edge `a = voxel_m` holds about `(a / d + 1)^3`
points `d` apart, a planar patch about `(a / d + 1)^2`: with `a = 0.5 m`, 36 per patch at
`d = 0.1 m` (more than `N = 20`, so the ring can still turn over), about 12 at `0.2 m`.
Points closer than `d` to each other in one insert replace one another in turn: a line of 50
points 1 cm apart leaves one point, the last (test `SpacingSweepKeepsNewest`). Scans are
downsampled at `preprocess.downsample_voxel_m` = `voxel_m` = 0.5 m before insertion, so a
single keyframe brings about one point per surface and voxel and this drift does not occur
within a keyframe.

Why spacing was tried. Without it, near-repeats of the same place from consecutive keyframes
take ring slots and evict older points; on box edges the ring can end up with two point
columns, one per face, whose 5-NN neighbourhood is exactly coplanar (`lambda_min = 0`): the
fit returns a diagonal plane that passes `plane_rms_max_m`. A first version rejected the new
point instead of replacing; it kept the first observation of each place and cost E-01 ATE
+6 / +14 / +16 % at 0.05 / 0.1 / 0.2 m. Replacing keeps the newest observation and halves
that cost (evaluation below).

## kNN and ties

`k <= 0` means `knn_k`; `k > 32` is clamped to 32. Candidates are the points of the 27 voxels
`key + (dx, dy, dz)`, `d in {-1, 0, 1}`, visited `for dz, for dy, for dx` (each from -1 to 1),
then slots `0..count-1`. They are ranked by `d2 = dx^2 + dy^2 + dz^2` in double; insertion
uses strict `<`, so on equal `d2` the earlier candidate in that visit order wins.
`dist2` is reported as `float(d2)`.

## Prune

A voxel is removed when `||(key + 0.5) * voxel_m - p_w_b||^2 > prune_radius_m^2`. The
comparison is strict: a centre exactly at the radius is kept. Its pool index goes back on the
free stack and its hash slot is emptied by backward-shift deletion (linear probing, no
tombstones), so later inserts reuse the space.

## Memory bound

Hash capacity is the smallest power of two `>= 2 * max_voxels` (load factor <= 0.5), 16 B per
slot. Points are float x, y, z, 12 B each. Total, allocated once in the constructor:

`max_voxels * N * 12 B + capacity * 16 B + max_voxels * (24 B metadata + 4 B free stack)`

With the defaults (`max_voxels = 500000`, `N = 20`, capacity `2^20`): 120 MB + 16.8 MB +
14 MB, about 151 MB. Nothing is allocated after construction.

## Đánh giá (min_point_dist_m, 2026-10-27)

Sweep of `map.min_point_dist_m` (replacement policy) with the plane-fit gates off and
everything else at `config/default.yaml` (N = 20, 5-NN, scan downsample 0.5 m). S-02: the
0.125 m room map inserted once, 10 trials, worst error (`S02_UngatedEdgeBias` setup). Box:
`PipelineComponent.BoxWorld*` (GTSAM build). Corridor: `CorridorAxisLeak` (ray-cast box
world with the x walls beyond `max_range_m`, 10 keyframes 1 m apart, scan aligned at its true
pose): share of the largest translational information along the corridor axis, `a^T S a /
lambda_max(S)` with `S = H_tt - H_tr H_rr^-1 H_rt`. E-01..E-03: `tools/run_integration.py`,
`mss_bench` in WSL (base 9ec6919).

| `min_point_dist_m` | S-02 worst error | S-02 replaced | box ATE LiDAR / GNSS+wheel | box map points | corridor axis share | E-01 / E-02 / E-03 ATE |
| --- | --- | --- | --- | --- | --- | --- |
| 0 (off) | 9.50 mm / 0.083° | 0 | 8.8 / 32.9 mm | 20 297 | 1.15 % | 0.0709 / 0.0709 / 0.0707 m |
| 0.05 m | 9.50 mm / 0.083° | 0 | 9.3 / 33.4 mm | 18 826 | 1.34 % | 0.0739 / 0.0739 / 0.0735 m |
| 0.1 m | 0.38 mm / 0.016° | 132 | 8.2 / 32.4 mm | 12 932 | 1.17 % | 0.0742 / 0.0742 / 0.0739 m |
| 0.2 m | 421 mm / 4.8° (fails S-02) | 9 360 | 6.1 / 32.9 mm | 8 105 | 0.63 % | 0.0714 / 0.0714 / 0.0717 m |

The rejecting first version gave E-01 0.0751 / 0.0805 / 0.0826 m at 0.05 / 0.1 / 0.2 m;
replacing halves or removes that cost but does not remove it at 0.05 and 0.1 m.

S-02: the room map is inserted once, so its bias is not about repeated scans. The faces at
`x = -4`, `y = -3` and `z = -1` lie on voxel boundaries; those edge voxels hold two faces (32
grid points for 20 slots). At 0.1 m the face point 0.088 m from the other face's point is
replaced (132 points) and the ring keeps different points. At 0.2 m the 0.125 m grid collapses
to one point per 0.2 m and S-02 diverges.

Corridor: spacing does not change the axis leak. The leak comes from neighbourhoods across
the wall-floor and wall-ceiling edges, which the planarity gate removes
(`docs/derivations/scan_match.md` section 3: 1.1 % -> 0.04 %).

E-01 at 0.1 m, explained. Replays of the E-01 dataset with the fusion switched off show where
the ATE comes from (ATE after SE(3) alignment, same data):

| replay | spacing off | spacing 0.1 m |
| --- | --- | --- |
| LiDAR + IMU only | 3.0 mm (first / last 10 s: 5.1 / 2.4 mm) | 3.7 mm (7.1 / 2.0 mm) |
| + wheel only | 2.8 mm | 4.0 mm |
| + GNSS only | 73.2 mm | 75.3 mm |
| + GNSS + wheel (E-01) | 70.9 mm | 74.2 mm |

Per-scan scan-matching statistics of the E-01 replay (temporary instrumentation, mean over
578 scans, off / 0.1 m): map points 20 694 / 13 583; inliers 1137.6 / 1136.4 (99.3 / 99.2 %
of the scan); nearest-neighbour distance 59.0 / 62.8 mm; 5th-neighbour distance 189 / 220 mm;
mean |point-to-plane residual| 6.3 / 5.2 mm; Gauss-Newton iterations 2.63 / 2.37. Scan matching
is not degraded: the same points match, with smaller residuals. 96 % of the E-01 ATE is the
GNSS fixes (σ 1.5 m) pulling a LiDAR-inertial trajectory that is accurate to 3 mm on its own.
Spacing changes that LiDAR trajectory by under 1 mm, mostly in the first 10 s, when the map is
built from the first keyframes (3 830 / 2 791 points; 5-NN radius 0.44 / 0.46 m). The GNSS
fusion turns this into +3.3 mm (+4.7 %); the mechanism of that amplification was not isolated.
The regression is therefore real but not a matching failure, and it is specific to a
GNSS-dominated metric.

After the rebase on 0d1df78 (#19, scenario overlay, E-01 about 2 mm): E-01 ATE at 0 / 0.05 /
0.1 / 0.2 m is 2.20 / 2.31 / 3.11 / 4.25 mm with the gates off and 1.65 / 1.48 / 1.51 / 1.96 mm
with planarity 0.05 + collinearity 0.2. With those gates, spacing 0.05 or 0.1 m improves E-01 by
about 10 %; with the gates off it still costs 5 to 93 %. The GNSS-weighting explanation above was
measured on 9ec6919, before #19 calibrated the LiDAR covariance.

With planarity 0.01 (an evaluated, off-by-default gate) E-01 ATE at 0 / 0.05 / 0.1 / 0.2 m is
0.95 / 0.87 / 0.91 / 1.27 mm.

Default: `min_point_dist_m` stays **0.0**: at the shipped config (gates off) it costs E-01
(2.20 -> 2.31 / 3.11 / 4.25 mm), its effect changes sign with the plane-fit gates, and 0.2 m
breaks S-02. It stays an evaluated memory and speed option (0.1 m: about 30 % fewer map
points, scan p50 -10 %).

Cost (E-01 scenario dataset and overlay, `mss_bench` in WSL on the desktop, 5 alternating runs,
medians; Pi5 not measured): main 0d1df78 4.27 / 5.83 ms scan p50 / p95, 2.57 s replay; shipped
defaults (spacing and gates off) 4.25 / 5.81 ms, 2.50 s, `est.tum` md5-identical to main,
40 115 map points at the end; with spacing 0.1 m 3.81 / 5.72 ms, 2.36 s, 25 781 points and
77 566 replacements. On the ov_plain replay (old E-01 data, default extrinsics): main 4.00 /
6.17 ms, 2.61 s; shipped 3.98 / 6.13 ms, 2.59 s, md5-identical; spacing 0.1 m 3.65 / 5.53 ms,
2.30 s. The distance checks cost less than the smaller map saves in `knn`.
