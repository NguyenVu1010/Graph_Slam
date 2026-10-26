# preprocess: deskew and voxel downsampling

Implemented in `core/preprocess/` (`Preprocessor`, `detail::DeskewTable`,
`detail::VoxelSet`). Tested by `tests/unit/test_preprocessor.cpp`,
`tests/unit/test_preprocess_*.cpp` (P-02..P-04) and
`tests/component/test_preprocess_deskew_plane.cpp` (P-01).
Conventions: `docs/conventions.md`, `docs/derivations/lie_conventions.md`.

## 1. Deskew

A point `p_l(t)` is measured in the LiDAR frame at time `t = in.t_start + t_offset_ns`.
Deskew moves it to `l0`, the LiDAR frame at `in.t_start`:

    p_l0 = T_l0_l(t) * p_l(t),    T_l0_l(t) = T_b_l^-1 * T_b0_b(t) * T_b_l

with `T_b_l` the extrinsic (frames.T_b_l) and `T_b0_b(t)` the body motion since
`t_start` from `imu::ImuTrajectory`. This inverts the skew model the simulator injects
(ADR-011 section 3): `p_i <- T_l(t_i)^-1 * T_l(t_start) * p_i`.

## 2. Interpolation

Between two IMU samples `k`, `k+1` the trajectory is the SE(3) geodesic

    T_b0_b(t) = T_k * Exp(s * Log(T_k^-1 * T_k+1)),    s = (t - t_k) / (t_k+1 - t_k)

(trajectory.hpp). Conjugation by `C = T_b_l` commutes with it, because
`C^-1 Exp(xi) C = Exp(Ad(C^-1) xi)`:

    C^-1 (T_k Exp(s xi)) C = (C^-1 T_k C) Exp(s Ad(C^-1) xi) = A_k Exp(s xi^l_k)

with `A_k = T_l_b T_k T_b_l` and `xi^l_k = Ad(C^-1) xi = Log(A_k^-1 A_k+1)`
(since `A_k^-1 A_k+1 = C^-1 T_k^-1 T_k+1 C`). So `DeskewTable::build` precomputes `A_k`
and `xi^l_k` once per scan, and each point costs one binary search over the sample
times plus one `Exp`. Tangent order `[rho, phi]`, right perturbation.

## 3. Clamping

`t <= t_0` gives `A_0`; `t >= t_{n-1}` gives `A_{n-1}`. There is no extrapolation, and a
single-sample trajectory gives `A_0` everywhere.

## 4. Exactness rule

`A_k` is set to the exact identity when `samples[k].T_b0_b` is bitwise the identity
(`R == I`, `p == 0`); a naive `T_l_b * I * T_b_l` leaves ~1e-17 residue. When
`xi^l_k` is exactly zero, or `t == t_k`, the result is `A_k` without calling `Exp`.
Together these make static scans (P-02) and points with `t_offset_ns == 0` bit-exact.

## 5. Voxel key

`key = floor(p / downsample_voxel_m)` per axis as `int64` (floor, not truncation, so
`-0.1 m` and `+0.1 m` are in different voxels). The key is computed from the float
coordinates written to `out`, so a kept point always lies in its own voxel. The first
surviving point in input order wins. The set is open addressing with linear probing,
slot count the smallest power of two >= 2 * max_points (load factor <= 0.5), hash
`(x * 73856093) ^ (y * 19349669) ^ (z * 83492791)` (Teschner et al. 2003).

This module has no residual and no Jacobian, so there is no numerical-difference test;
deskew correctness is checked by P-01 against an analytic plane.
