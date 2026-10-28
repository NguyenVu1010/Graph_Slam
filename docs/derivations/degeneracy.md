# Degeneracy detection and handling (D-01..D-03)

Implemented in `core/degeneracy/src/degeneracy.cpp` (contract:
`core/degeneracy/include/mss/degeneracy/degeneracy.hpp`). Tested by
`tests/unit/test_degeneracy.cpp` and `tests/component/test_degeneracy_scenes.cpp`.
Design background: ADR-004 (LiDAR factor covariance = inverse Hessian, "unobserved" along
degenerate directions), ADR-007 (voxel map, source of the k-NN planes).

Conventions: tangent `xi = [rho, phi]` (translation first), right perturbation
`T <- T * Exp(xi)` (see `lie_conventions.md`). This module converts no frames.

## 1. Source of the Hessian

Scan matching minimises point-to-plane residuals, one per scan point `p_l` with map plane
point `q_w` and unit normal `n_w`:

    r = n_w . (T_w_l * p_l - q_w)

With `T_w_l <- T_w_l * Exp(xi)` the Jacobian at `xi = 0` (`scan_matcher.hpp`) is the row

    J_r_xi = [ (R_w_l^T n_w)^T ,  -(R_w_l^T n_w)^T skew(p_l) ]

and the Gauss-Newton Hessian is `H = sum_i w_i J_r_xi_i^T J_r_xi_i` (IRLS weights `w_i`).
Write `n = R_w_l^T n_w` (normal in l). Since `-n^T skew(p) = (p x n)^T`, the row is
`[n ; p x n]^T`. In particular at `R_w_l = I` it is `[n_w ; p_l x n_w]^T`.

Variable: `xi`, the right perturbation of `T_w_l`, so H lives in the tangent of l.

## 2. Eigen-analysis and flags

H is symmetric positive semi-definite, so

    H = V diag(lambda) V^T,   lambda_0 <= ... <= lambda_5,   V^T V = I.

Direction `k` (eigenvector `v_k = V.col(k)`) is degenerate iff

    enabled  and  lambda_k < eig_threshold      (strict).

`lambda` is used as is: absolute, not normalised by the number of points or by the largest
eigenvalue (header decision). Its units are mixed: the `rho-rho` block is unitless
(`J` entries for `rho` are unitless), the `rho-phi` and `phi-phi` blocks carry `m` and
`m^2` from the lever arm `p x n`. The threshold is therefore tuned for the scan matcher's typical
point count and range (`config/default.yaml`).

If every normal is orthogonal to a translation direction `t` (e.g. `t` = corridor axis,
normals on the walls and the floor), then `[t ; 0]^T J_r_xi_i^T = n_i . t = 0` for every
`i`, so `H [t ; 0] = 0` and `lambda = 0` exactly along that direction.

## 3. Projected update

Let `m_k = 0` if direction `k` is flagged, else `1`. The projected Gauss-Newton update is

    delta' = V diag(m) V^T delta = delta - sum_{k flagged} v_k (v_k^T delta).

Proof of the contract: for any `j`, `v_j^T delta' = v_j^T delta - sum_{k flagged}
(v_j^T v_k)(v_k^T delta) = v_j^T delta - [j flagged] v_j^T delta`, using `v_j^T v_k =
delta_jk`. Hence `v_j^T delta' = 0` for flagged `j` and is unchanged for the others.

Repeated eigenvalues: the flag depends only on `lambda_k`, so all eigenvectors of a
repeated eigenvalue are flagged together. `sum_{k flagged} v_k v_k^T` is then the
orthogonal projector onto the whole eigenspace, which does not depend on the basis the
solver picks inside it (example: a single floor plane, three zero eigenvalues spanning
`rho_x, rho_y, phi_z`).

## 4. Covariance inflation

Let `f = relax_factor >= 1`, `s_k = sqrt(f)` if flagged else `1`:

    A = V diag(s) V^T = I + sum_{k flagged} (sqrt(f) - 1) v_k v_k^T
    cov' = A cov A^T,  then symmetrised as 0.5 (cov' + cov'^T).

`A` is symmetric and `A v_k = s_k v_k`, so

    v_k^T cov' v_k = (A v_k)^T cov (A v_k) = s_k^2 v_k^T cov v_k,

i.e. `f v_k^T cov v_k` for flagged directions and `v_k^T cov v_k` for the others. The
symmetrisation removes rounding asymmetry only (it does not change `v^T cov' v`). With
`f = 1`, `A = I` up to rounding. `cov'` stays positive definite because `A` is invertible.

Caller: none in the pipeline since PR 4b. `scan_match::ScanMatcher::align` now applies
`unobserved_covariance` instead (section 4b), because `relax_factor` times a regularised
variance is still a confident value along a direction the scan does not observe
(`scan_match.md` section 6).

## 4b. Unobserved covariance

`unobserved_covariance(cov, report, cfg, var_trans_m2, var_rot_rad2, c)` (`var_m2` and
`var_rad2` below are `var_trans_m2` and `var_rot_rad2`): with `P = I - sum_{k flagged} v_k
v_k^T` and `D = diag(var_m2 I3, var_rad2 I3)`,

    cov' = P cov P + U',   then symmetrised,   U = P_F D P_F,   P_F = I - P,
    U' = U with each eigenvalue u on F floored at relax_factor * c / eig_threshold
         (no floor when eig_threshold <= 0),

with `c = cov_scale * s2` the scale of the matcher's regularised inverse. For unflagged
`i, j`: `v_i^T cov' v_j = v_i^T cov v_j`; the flagged subspace is decoupled from the observed
one, and `U` does not depend on the basis chosen inside a repeated flagged eigenvalue. If the
configured variances are below the floor, `U'` is the pre-PR-4b relaxed covariance. Derivation and defaults in `scan_match.md` section 6. With no flags it
returns `cov` exactly. The pipeline passes the DegeneracyReport to backend for diagnostics
only (see graph.hpp) and must not change the covariance again.

## 5. Frames

Everything here lives in the tangent of the pose the Hessian was built for: l for scan
matching. The pipeline converts afterwards: the covariance with
`transform_covariance(cov', T_b_l)` and the degenerate directions with
`se3_adj(T_b_l) * v_k` (`lie.hpp`).

## 6. Non-finite input

If any entry of the lower triangle of H is NaN or Inf (or the eigen solver fails), the
report is `lambda = 0`, `V = I` and, when enabled, every direction is flagged. Then
`delta' = delta - sum_k e_k e_k^T delta = 0` (no update from a broken linearisation) and
`A = sqrt(f) I`, so `cov' = f cov` (never NaN from H), and `unobserved_covariance` returns
`D = diag(var_m2 I3, var_rad2 I3)` floored at `relax_factor * c / eig_threshold` for any finite `cov` (exactly `D` for `c = 0`, test `UnobservedCovariance.AxisDirectionsGetConfiguredVariance`).
The scan matcher's `LLT(H_reg)` fails on a non-finite H ("singular Hessian"). When disabled
nothing is flagged and the functions are identities.

## 7. Đánh giá (evaluation, PR 4b, on main 3f90e34)

Main 3f90e34 includes the gravity-direction factor on keyframe 0 (ADR-020), the decoupled
relative factor (ADR-021: translation and rotation residuals separate, the chart of the
scan-match covariance) and the exact pose prior (#28).

Data: `tests/component/box_world.hpp`, 3 s static, 2 s ramp, 15 s on a 2 m circle at 0.5 m/s,
16 x 720 ray-cast LiDAR, IMU white noise from `config/default.yaml` (no bias in the simulation),
GTSAM backend, 180 estimates. "Box": the closed 16 x 10 x 3 m box. "Box 3x": the same with a
48 x 30 m footprint. "Corridor": the box with the x walls moved beyond
`preprocess.max_range_m`, so world x is unobserved by the LiDAR. Errors are RMS position
errors without alignment. "Degenerate" counts `Estimator::Stats::n_degenerate_keyframes` (of
22 LiDAR factors; `n_degenerate_rot_keyframes` is 0 in every run). E-01 is
`tools/run_integration.py` (scenario overlay `config/sim/warehouse.yaml`, GNSS and wheel),
run with `mss_bench` on the same mssdata and scored with `mss_eval.ate`.

Reproduce the box-world tables and the GNSS-corridor sweep (GTSAM build; about 3 min):

    MSS_DEGENERACY_STUDY=1 <build>/tests/component/test_pipeline_component \
        --gtest_filter='DegeneracyStudy.*' 2>/dev/null
    # optional: MSS_DEGENERACY_STUDY_FILTER=<text> runs only the settings whose name has it

The committed checks are `CorridorFollowsWheelAlongAxis`, `CorridorImuOnlyAlongAxisBounded`
and the corridor GNSS tests (`CorridorDriftCorrectedByGnss` and the two #19 tests after it).

### Absolute detector (eig_threshold 100) on and off

| setting | corridor + wheel: along / lateral | corridor IMU only: along | box LiDAR / GNSS + wheel ATE | box 3x LiDAR / GNSS + wheel ATE | E-01 ATE | degenerate: corridor / box / box 3x / E-01 |
| --- | --- | --- | --- | --- | --- | --- |
| off (main) | 0.296 / 0.0033 m | 6.08 m | 9.2 / 5.4 mm | 7.9 / 5.8 mm | 2.183 mm | 0 / 0 / 0 / 0 |
| absolute 100 | 0.0038 / 0.0008 m | 0.021 m | 9.7 / 6.0 mm | 7.9 / 5.8 mm | 2.450 mm | 22 / 5 and 5 / 0 / 52 |

Absolute 100 flags the corridor axis in every keyframe (3.8 mm along the axis with wheel
odometry, 0.30 m off) but regresses the box (LiDAR only 9.2 -> 9.7 mm, GNSS + wheel 5.4 ->
6.0 mm, 5 healthy keyframes flagged in each) and E-01 (2.18 -> 2.45 mm, 52 keyframes): the
absolute eigenvalue threshold depends on the inlier count and the range.

A sigma detector (posterior std along each eigen-direction against a physical tolerance) was
evaluated in PR 4b and is held, not merged: in this corridor the wall-floor edge leak (1.15 %
of the translational information, `scan_match.md` section 3) gives the axis a posterior std
of 3.8 mm, below every tolerance >= 2 cm, so it flags nothing until the map edge leak is fixed.

### Unobserved variance (absolute 100)

| `unobserved_var_m2` / `_rad2` | corridor + wheel: along | corridor IMU only: along |
| --- | --- | --- |
| 1e-4 / 1e-5 | 3.4 mm | 0.073 m |
| 0.001 / 1e-4 (default) | 3.8 mm | 0.021 m |
| 0.01 / 0.001 | 3.9 mm | 0.140 m |
| 0.1 / 0.01 | 3.9 mm | 2.24 m |
| 1.0 / 0.1 | 3.9 mm | 0.823 m |

### IMU-only corridor after ADR-020 and ADR-021

Without wheel odometry only the IMU observes the axis, over a 17 s stretch of motion (2 s ramp,
15 s cruise). Inertial dead reckoning in this simulation (accelerometer white noise
`sigma_a = 5.8e-3 m/s^2/sqrt(Hz)`, no bias, roll and pitch observed by the floor and walls)
gives a velocity random walk `sigma_a sqrt(T) = 0.024 m/s`, a position error
`sigma_a T^1.5 / sqrt(3) = 0.24 m`, and `0.5 b T^2 = 0.59 m` from the accelerometer-bias error
of the 2 s static initialisation window (`b = sigma_a / sqrt(2) = 4.1e-3 m/s^2`): about 0.6 to
0.7 m (1 sigma) at T = 17 s.

On main 94b9f16 the observed IMU-only errors were 4.3 to 48 m with 0.1 to 1 m^2 (7 to 70
sigma). PR 4d found the cause: a tilt about the horizontal axis normal to x is
indistinguishable from an acceleration along x, and the only absolute tilt anchor was the
keyframe-0 prior at 1 deg (`docs/derivations/imu_only_observability.md`, `docs/lessons.md`
section 1.1). The gravity-direction factor on keyframe 0 (ADR-020) and the decoupled relative
factor (ADR-021, which stops the SE(3) Log chart from turning a large residual along the free
axis into rotation information, `docs/lessons.md` section 1.9) bring the loose-variance rows
down to 2.2 m at 0.1 m^2 and 0.82 m at 1 m^2, about 3.4 and 1.3 sigma of dead reckoning; at
the default 0.001 m^2 the axis is held at 0.021 m (test bound 0.1 m). With degeneracy off the
error (6.1 m) is not IMU drift but the scan matcher wandering on the nearly singular H.

### #19 GNSS corridor (`CorridorDriftCorrectedByGnss`)

120 m corridor with the end walls out of range, 60 s, GNSS 0.5 m at 5 Hz, no wheel, absolute
detector, "unobserved" variance 1000 m^2 / 10 rad^2 (`kCorridorRelaxYaml`). The variance is
chosen physically: the LiDAR factor must not resist an along-axis GNSS correction as large as
the along-axis drift over the run, which reaches tens of metres (LiDAR-only ATE 8 to 67 m), so
its std must be of the order of the 30 m drive or more; 1000 m^2 is 32 m per keyframe step,
far above the 0.5 m GNSS sigma. The test asserts, over seeds 42, 43, 44: on every seed more
than 200 GNSS factors (no chi-square lock-out) and fused ATE <= 0.5 x LiDAR-only ATE, and a
median fused ATE <= 2.18 m (owner decision).

| seed | LiDAR-only ATE | fused ATE | ratio | GNSS factors |
| --- | --- | --- | --- | --- |
| 42 | 35.1 m | 0.507 m | 0.014 | 273 |
| 43 | 18.7 m | 0.589 m | 0.032 | 268 |
| 44 | 19.4 m | 1.704 m | 0.088 | 260 |
| median | | 0.589 m | | |

The variance sweep (`DegeneracyStudy.GnssCorridorVariance`, fused / LiDAR-only ATE in metres)
is kept as evidence; it is chaotic in the variance, the seed and even the rounding, so no value
is taken from it:

| `unobserved_var_m2` | seed 42 | seed 43 | seed 44 | median fused | worst fused |
| --- | --- | --- | --- | --- | --- |
| 1 | 23.9 / 40.3 | 0.50 / 12.6 | 14.4 / 51.8 | 14.4 | 23.9 |
| 10 | 0.47 / 9.1 | 0.68 / 7.9 | 26.5 / 66.8 | 0.68 | 26.5 |
| 100 | 2.69 / 38.4 | 0.59 / 10.2 | 2.94 / 33.8 | 2.69 | 2.94 |
| 1000 (used) | 0.51 / 35.1 | 0.59 / 18.7 | 1.70 / 19.4 | 0.59 | 1.70 |
| 1e4 | 0.50 / 39.8 | 0.75 / 10.1 | 3.38 / 13.7 | 0.75 | 3.38 |

A rounding-only change in the unobserved block (the basis-invariant `U = P_F D P_F`) moved
seed 42 at 1000 m^2 from 1.78 to 0.51 m.

**Open item: in this corridor fusion is not reliably better than raw GNSS alone.** Raw GNSS at
0.5 m per axis is about 0.87 m 3D RMS. On this branch the fused ATE is 0.51 / 0.59 / 1.70 m over
seeds 42, 43, 44: better than raw GNSS on seeds 42 and 43, 1.95x worse on seed 44. On main the
15-seed median is 3.47 m (4x worse, ADR-021 table). It needs a diagnosis PR
(`gnss_fusion_study.md` section 12).

### Gazebo E-04 (measurement only)

E-04 bag recorded with the #26 pipeline (not committed): `tools/record_scenario.sh
sim/scenarios/E-04.yaml results/<run-id>/` (`sim/README.md`), then `tools/run_integration.py
--bridged results/<run-id>/` or `mss_bench` on its `mssdata/` with `config/sim/corridor.yaml`
and the degeneracy settings changed, ATE after the `mss_eval.ate`
alignment, split into horizontal and vertical (metres):

| run | ATE | horizontal | vertical | keyframes (flagged) | GNSS factors | realigns |
| --- | --- | --- | --- | --- | --- | --- |
| main 3f90e34, `corridor_degeneracy_on.yaml` (relax_factor 10) | 1.688 | 0.500 | 1.612 | 55 | 224 | 0 |
| off (= main off, eig_threshold 10) | 9.973 | 9.973 | 0.058 | 24 (0) | 121 | 1 |
| absolute 100, unobserved 1e-12 m^2 (old semantics) | 2.202 | 0.875 | 2.021 | 55 (54) | 162 | 0 |
| absolute 100, 1e-4 m^2 | 3.376 | 1.058 | 3.206 | 57 (55) | 147 | 1 |
| absolute 100, 1e-3 m^2 (default) | 1.836 | 0.743 | 1.679 | 55 (54) | 174 | 0 |
| absolute 100, 0.01 m^2 | **0.409** | 0.192 | 0.361 | 51 (49) | 259 | 0 |
| absolute 100, 0.1 m^2 | **0.343** | 0.162 | 0.302 | 51 (49) | 260 | 0 |
| absolute 100, 1 m^2 | 1.260 | 0.221 | 1.240 | 52 (44) | 266 | 0 |
| eig_threshold 30, 1e-3 m^2 | 5.252 | 5.249 | 0.158 | 23 (7) | 119 | 2 |
| eig_threshold 1000, 1e-3 m^2 | 17.19 | 16.89 | 3.18 | 113 (112) | 17 | 4 |

- **At 0.001 m^2 the floor binds on this bag.** `relax_factor * c / eig_threshold` is about
  1.2e-3 m^2 with Gazebo residuals, so the default, 1e-4 and 1e-12 m^2 all give the pre-PR-4b
  relaxed covariance; the default is not "unobserved" here.
- **E-04 with degeneracy on is chaotic.** Main and the old semantics are bit-identical up to
  t = 5.7 s and first differ at the 1e-13 level (rounding of `P C P + U` against `A C A^T`);
  that difference grows to 1.69 m against 2.20 m ATE through the GNSS chi-square gate. Runs
  with the same semantics spread from 1.8 to 3.4 m. A single run cannot rank semantics.
- **Above the floor the "unobserved" semantics are the best on this bag:** 0.41 m at 0.01 m^2
  and 0.34 m at 0.1 m^2 (about 4x better than main), with 259 to 260 GNSS factors and no
  realign; 1 m^2 is worse again (1.26 m).
- **The vertical error is the S2 start-up defect, not the covariance semantics.** With
  absolute 100 the estimated z rises by about 1.1 m within 3 s of the first estimate, before
  any keyframe correction, for every variance; it stays at 0.03 m with degeneracy off and with
  `eig_threshold` 30 (7 keyframes flagged, ATE 5.25 m, vertical 0.16 m). Absolute 100 flags
  more than the corridor axis at start-up, when the map is small and the absolute eigenvalues
  are low; which direction needs a per-direction flag dump (S2).

### Decision (owner, 2026-10-28)

`degeneracy.enabled: false` and `unobserved_var_m2: 0.001` stay. The synthetic IMU-only corridor
is 0.14 m at 0.01 m^2 against its 0.1 m bound, and one chaotic bag (1 m^2 worse than 0.1 m^2)
cannot choose a value. Retune `unobserved_var_m2` over several Gazebo recordings in **S2**, with
0.01 m^2 as the candidate, together with the per-direction flag dump for the E-04 start-up
excursion.
