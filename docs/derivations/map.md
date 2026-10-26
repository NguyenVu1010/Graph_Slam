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
goes to slot `head`, then `head = (head + 1) % N` and `count = min(count + 1, N)`. When the
voxel is full the oldest point is overwritten. A new voxel when `num_voxels == max_voxels`
is not created: the point is dropped and `num_dropped` grows. Points for existing voxels are
still stored.

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
