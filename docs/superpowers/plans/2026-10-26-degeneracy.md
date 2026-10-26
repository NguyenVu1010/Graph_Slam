# Degeneracy Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `mss::degeneracy::{analyze, project_update, relax_covariance}`: eigen-analysis of the 6x6 point-to-plane Hessian, per-direction degeneracy flags, projected Gauss-Newton update and covariance inflation along degenerate directions.

**Architecture:** Three stateless free functions in one new source file `core/degeneracy/src/degeneracy.cpp`, using only fixed-size Eigen (`Eigen::SelfAdjointEigenSolver<common::Mat6>`). `mss_degeneracy` switches from INTERFACE to STATIC. Unit tests cover each function's contract and the no-allocation rule; component tests build synthetic corridor / room / floor Hessians directly from plane normals and points (no scan matcher) for D-01..D-03.

**Tech Stack:** C++17, Eigen 3.4 (fixed-size), GoogleTest, CMake (`mss_add_test`), WSL build via `tools/wsl/build.sh`.

**Spec:** `core/degeneracy/include/mss/degeneracy/degeneracy.hpp` and `config.hpp` (frozen contract); consumer `core/scan_match/include/mss/scan_match/scan_matcher.hpp` (calls `analyze` + `project_update` every Gauss-Newton iteration when `deg_cfg.enabled`); `config/default.yaml` keys `degeneracy.*`; `docs/Test_CI_CD.md` section `### degeneracy` (D-01..D-03); ADR-004 (LiDAR factor covariance = inverse Hessian inflated along degenerate directions), ADR-007 (voxel map; source of the k-NN planes whose normals build H), ADR-003/005/006/008/009/013; `docs/conventions.md`, `docs/derivations/lie_conventions.md`; `CLAUDE.md`.

## Contract issues

None. Every public signature in `degeneracy.hpp` is implementable as declared.

## Global Constraints

- Public signatures in `core/degeneracy/include/mss/degeneracy/*.hpp` are frozen; add only private helpers in the `.cpp` (anonymous namespace).
- Tangent order `[rho, phi]` (translation first), right perturbation `T <- T * Exp(xi)`; H, delta, cov and the report share the tangent of the pose H was built for (`T_w_l` for scan matching). This module converts no frames.
- Config (from `config/default.yaml`, validated by `pipeline::load_config`): `degeneracy.enabled: false`, `degeneracy.eig_threshold: 100.0` (absolute eigenvalue of H, `Range::kNonNegative`), `degeneracy.relax_factor: 10.0` (`Range::kAtLeastOne`). No other tunables; no magic numbers in `core/`.
- Hot path (per Gauss-Newton iteration per scan): fixed-size Eigen only, no heap allocation, no logging, no exceptions, no `std::vector` / `Eigen::Dynamic` in `core/degeneracy/src/`.
- No `auto` for Eigen expressions; pass Eigen by `const&`; naming per CLAUDE.md; include order own header, std, third party, project.
- Warnings `-Wall -Wextra -Wpedantic -Werror` (build.sh sets `MSS_WERROR=ON`); files LF; clang-format-18 clean.
- Random tests use fixed seeds (`std::mt19937`, seed given per test).
- Feature flag: with `enabled = false` nothing is flagged, so `project_update` and `relax_covariance` return their input bit for bit (default behaviour unchanged).
- Build/test (the only way to compile):
  `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-degeneracy && tools/wsl/build.sh --no-gtsam --build-dir ~/mss-build/feat-degeneracy-nogtsam --target <test_binary_name>"`; full build + ctest: same without `--target`.
- Format check before each commit:
  `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-degeneracy && git ls-files -mo --exclude-standard '*.cpp' '*.hpp' | xargs -r clang-format-18 --dry-run --Werror"` -> no output, exit 0.
- Commits: Conventional Commits with trailers
  `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01DYAqFYL9xp2KFYiPfmqpc4`. Do not push.

## Review Focus

1. NaN/Inf only in the strict upper triangle of H (header: "only the lower triangle is read") -> result identical to the clean symmetric H, not the non-finite fallback. Pinned: Task 1 `Analyze.UpperTriangleIsIgnored`.
2. Eigenvalue exactly equal to `eig_threshold` -> not flagged (strict `<`); one ulp above the eigenvalue -> flagged. Pinned: Task 1 `Analyze.ThresholdIsStrict`.
3. Multi-dimensional degenerate subspace with a repeated (zero) eigenvalue, e.g. a single floor plane (3 free DoF: rho_x, rho_y, phi_z). The eigenvectors inside the subspace are arbitrary, so the projection must remove the whole subspace. Pinned: Task 3 `DegeneracyScenes.SinglePlaneThreeDirections`.
4. Non-finite H with `enabled = true` (all flagged, eigvecs = I) -> the scan matcher's update becomes exactly zero and the covariance becomes `relax_factor * cov` up to rounding, never NaN. Pinned: Task 2 `ProjectUpdate.AllFlaggedIdentityGivesZero`, `RelaxCovariance.AllFlaggedIdentityScalesCov`.
5. `relax_factor = 1.0` (allowed minimum) with flags set -> cov unchanged up to rounding and the output exactly symmetric. Pinned: Task 2 `RelaxCovariance.FactorOneKeepsCov`.

---

### Task 1: Derivation doc, STATIC library and `analyze()`

**Files:**
- Create: `docs/derivations/degeneracy.md`
- Create: `core/degeneracy/src/degeneracy.cpp`
- Modify: `core/degeneracy/CMakeLists.txt` (whole file)
- Create: `tests/unit/test_degeneracy.cpp`
- Modify: `tests/unit/CMakeLists.txt` (append one line)

**Interfaces:**
- Consumes: `common::Mat6`, `common::Vec6`, `common::PoseCovariance`, `common::DegeneracyReport` (`mss/common/types.hpp`); `degeneracy::DegeneracyConfig` (`mss/degeneracy/config.hpp`).
- Produces: `common::DegeneracyReport analyze(const common::Mat6& H, const DegeneracyConfig& cfg)` defined; target `mss::degeneracy` is STATIC; test binary `test_degeneracy` (links `alloc_counter_impl.cpp`; Task 2 extends it); helper in the test's anonymous namespace `common::Mat6 random_spd(std::uint32_t seed)`: B with entries `N(0, 1)` from `std::mt19937(seed)`, returns `B * B^T + 1e-3 * I`.

- [x] **Step 1: Write `docs/derivations/degeneracy.md`**

Sections (formulas, no code):
1. Source of H: per point `r = n_w . (T_w_l p_l - q_w)`, `J_r_xi = [ (R_w_l^T n_w)^T, -(R_w_l^T n_w)^T skew(p_l) ]` (scan_matcher.hpp), `H = sum w J^T J`. At `R_w_l = I` the row is `[n ; p x n]`. Variable: `xi = [rho, phi]`, right perturbation of `T_w_l`.
2. Eigen-analysis `H = V diag(lambda) V^T`, lambda ascending, V orthonormal; direction k is degenerate iff `enabled && lambda_k < eig_threshold`. Note that lambda has mixed units (m blocks and m/rad blocks) and is absolute, not normalised (header decision). If every normal is orthogonal to a translation direction, that direction has `lambda = 0` exactly.
3. Projection `delta' = V diag(m) V^T delta = delta - sum_{k flagged} v_k (v_k^T delta)`; proof that `v_k^T delta' = 0` for flagged k and is unchanged for the others. Repeated eigenvalues: the flag depends only on lambda, so a repeated eigenvalue is flagged as a whole subspace and the projector does not depend on which eigenvectors the solver picks inside it.
4. Inflation `A = V diag(s) V^T = I + sum_{k flagged} (sqrt(f) - 1) v_k v_k^T`, `cov' = A cov A^T`, symmetrised as `0.5 (cov' + cov'^T)`; proof that `A v_k = sqrt(f) v_k`, hence `v_k^T cov' v_k = f v_k^T cov v_k` (flagged) and `= v_k^T cov v_k` (unflagged).
5. Frames: everything is in the tangent of the pose H was built for (l). The pipeline converts afterwards (`transform_covariance(cov, T_b_l)`, `se3_adj(T_b_l) * v_k`).
6. Non-finite fallback (eigvals 0, eigvecs I, all flagged when enabled) and its effect: zero update, `cov' = f cov`.

- [x] **Step 2: Write the failing tests** in `tests/unit/test_degeneracy.cpp` (cfg is `degeneracy::DegeneracyConfig{true, 100.0, 10.0}` unless stated)

```cpp
TEST(Analyze, DiagonalSortedAscendingUnitNorm) {
  // H = diag(5, 1, 4, 2, 6, 3) -> eigvals == (1, 2, 3, 4, 5, 6) within 1e-12;
  // |eigvecs(perm[k], k)| within 1e-12 of 1, perm = {1, 3, 5, 2, 0, 4};
  // (V * eigvals.asDiagonal() * V^T - H).norm() < 1e-12.
}
TEST(Analyze, RandomSpdReconstructs) {
  // H = random_spd(7): (V diag V^T - H).norm() < 1e-9 * H.norm();
  // (V^T V - I).norm() < 1e-12; eigvals(k) <= eigvals(k + 1) for all k.
}
TEST(Analyze, FlagsBelowThresholdOnlyWhenEnabled) {
  // H = diag(50, 150, 1e3, 1e3, 1e3, 1e3): enabled -> degenerate == {1, 0, 0, 0, 0, 0};
  // enabled = false -> all false, eigvals bitwise equal to the enabled run.
}
TEST(Analyze, ThresholdIsStrict) {
  // H = random_spd(3); l0 = analyze(H, {true, 0.0, 10.0}).eigvals(0);
  // eig_threshold = l0 -> degenerate[0] false;
  // eig_threshold = std::nextafter(l0, +inf) -> degenerate[0] true.
}
TEST(Analyze, NegativeEigenvalueFlagged) {
  // H = diag(-1e-9, 1e3, 1e3, 1e3, 1e3, 1e3), cfg {true, 0.0, 10.0}:
  // degenerate[0] true, others false, eigvals all finite.
}
TEST(Analyze, NonFiniteLowerTriangleFallsBack) {
  // H = random_spd(5) with H(3, 1) = NaN; then separately with H(4, 4) = +Inf:
  // eigvals == Zero and eigvecs == Identity (bitwise); all degenerate when enabled,
  // none when enabled = false.
}
TEST(Analyze, UpperTriangleIsIgnored) {
  // H = random_spd(9); Hu = H; Hu(1, 3) = NaN; Hu(0, 5) = 1e300 (strict upper only):
  // analyze(Hu) eigvals, eigvecs and degenerate bitwise equal to analyze(H).
}
```

- [x] **Step 3: Register the test and run it to verify it fails**

Append `mss_add_test(test_degeneracy test_degeneracy.cpp alloc_counter_impl.cpp)` to `tests/unit/CMakeLists.txt`. Run the build command with `--target test_degeneracy`.
Expected: link error `undefined reference to ... mss::degeneracy::analyze`.

- [x] **Step 4: CMake + implement `analyze` in `core/degeneracy/src/degeneracy.cpp`**

`core/degeneracy/CMakeLists.txt` follows `core/io/CMakeLists.txt`: `add_library(mss_degeneracy STATIC src/degeneracy.cpp)`, alias `mss::degeneracy`, `mss_set_target_defaults(mss_degeneracy)`, PUBLIC include dir (same generator expressions as now), `target_link_libraries(mss_degeneracy PUBLIC mss::common)`.

`analyze`: check finiteness of the lower triangle only (`i >= j`, `std::isfinite`); then `Eigen::SelfAdjointEigenSolver<common::Mat6> solver(H, Eigen::ComputeEigenvectors)` (reads the lower triangle; ascending eigenvalues; unit eigenvectors). If H is non-finite or `solver.info() != Eigen::Success`, return the default `DegeneracyReport` (eigvals 0, eigvecs I) with `degenerate[k] = cfg.enabled` for all k (planner's call: solver failure is handled like non-finite input). Otherwise copy eigvals and eigvecs and set `degenerate[k] = cfg.enabled && eigvals(k) < cfg.eig_threshold`. The decomposition also runs when disabled (planner's call: the report stays informative; the scan matcher calls it only when enabled).

- [x] **Step 5: Run the test to verify it passes**

Run the build command with `--target test_degeneracy`. Expected: `[  PASSED  ] 7 tests.` and `BUILD OK`.

- [x] **Step 6: Format check, then commit**

```bash
git add docs/derivations/degeneracy.md core/degeneracy tests/unit/test_degeneracy.cpp tests/unit/CMakeLists.txt
git commit -m "feat(degeneracy): eigen-analysis of the scan-matching Hessian"
```

---

### Task 2: `project_update()`, `relax_covariance()` and the no-allocation test

**Files:**
- Modify: `core/degeneracy/src/degeneracy.cpp`
- Modify: `tests/unit/test_degeneracy.cpp`

**Interfaces:**
- Consumes: `analyze` and `random_spd` from Task 1.
- Produces: `common::Vec6 project_update(const common::Vec6& delta, const common::DegeneracyReport& report)` and `common::PoseCovariance relax_covariance(const common::PoseCovariance& cov, const common::DegeneracyReport& report, const DegeneracyConfig& cfg)` defined (used by Task 3, `scan_match` and `pipeline`).

Fixtures: `report_flags02` = `analyze(random_spd(13), {false, 0.0, 10.0})` with `degenerate[0]` and `degenerate[2]` then set to true by hand; `delta` with entries `N(0, 1)` from `std::mt19937(21)`; `cov = random_spd(11)`; `all_flagged` = default-constructed `DegeneracyReport` with every flag true.

- [x] **Step 1: Write the failing tests**

```cpp
TEST(ProjectUpdate, NoFlagsIsExactIdentity) {
  // report = analyze(random_spd(13), {false, 0.0, 10.0}): (out.array() == delta.array()).all().
}
TEST(ProjectUpdate, RemovesFlaggedKeepsOthers) {
  // report_flags02: |v_k . out| < 1e-12 * delta.norm() for k in {0, 2};
  // |v_k . (out - delta)| < 1e-12 * delta.norm() for k in {1, 3, 4, 5}.
}
TEST(ProjectUpdate, AllFlaggedIdentityGivesZero) {
  // all_flagged -> out == Vec6::Zero() bitwise.
}
TEST(RelaxCovariance, NoFlagsIsExactIdentity) {
  // report without flags, cfg {true, 100.0, 10.0} -> cov' == cov bitwise.
}
TEST(RelaxCovariance, InflatesFlaggedDirectionsByFactor) {
  // report_flags02, cfg {true, 100.0, 10.0}: for k in {0, 2}, ratio = v^T cov' v / v^T cov v:
  // EXPECT_NEAR(ratio, 10.0, 1e-10 * 10.0) and ratio >= 10.0 * (1 - 1e-12);
  // for k in {1, 3, 4, 5}: ratio within 1e-10 of 1;
  // cov'(i, j) == cov'(j, i) bitwise for all i, j; Eigen::LLT<common::Mat6>(cov').info() == Success.
}
TEST(RelaxCovariance, FactorOneKeepsCov) {
  // report_flags02, relax_factor 1.0: (cov' - cov).norm() < 1e-12 * cov.norm(); cov' exactly symmetric.
}
TEST(RelaxCovariance, AllFlaggedIdentityScalesCov) {
  // all_flagged, relax_factor 10.0: (cov' - 10 * cov).norm() < 1e-12 * 10 * cov.norm(); all finite.
}
TEST(DegeneracyHotPath, NoAllocation) {
  // Built before measuring: H = random_spd(17), cfg {true, 1e9, 10.0} (everything flagged, so every
  // branch runs), delta, cov. AllocCounter::reset(); { EigenMallocGuard guard; 100 times:
  // r = analyze(H, cfg); d = project_update(delta, r); c = relax_covariance(cov, r, cfg);
  // sink += d.sum() + c.trace(); }
  // Then EXPECT_EQ(AllocCounter::count(), 0u); EXPECT_TRUE(std::isfinite(sink)).
  // No EXPECT/ASSERT inside the guard scope (gtest may allocate).
}
```

- [x] **Step 2: Run to verify they fail**

Run the build command with `--target test_degeneracy`. Expected: link errors `undefined reference to ... project_update` / `relax_covariance`.

- [x] **Step 3: Implement both functions in `core/degeneracy/src/degeneracy.cpp`**

Private helper `bool any_flagged(const common::DegeneracyReport& report)`. Both functions return a copy of their input when nothing is flagged (bit-for-bit identity).
`project_update`: `delta - sum_{k flagged} v_k (v_k . delta)` (equal to the header's `V diag(m) V^T delta`, cheaper with few flags; planner's call).
`relax_covariance`: `A = I + sum_{k flagged} (std::sqrt(cfg.relax_factor) - 1) v_k v_k^T` as `common::Mat6`; `common::Mat6 inflated = A * cov * A.transpose()`; return `0.5 * (inflated + inflated.transpose())` evaluated into a `common::PoseCovariance`. No runtime check of `relax_factor >= 1` (precondition validated by `load_config`; no exceptions on the hot path).

- [x] **Step 4: Run to verify they pass**

Run the build command with `--target test_degeneracy`. Expected: `[  PASSED  ] 15 tests.`

- [x] **Step 5: Fixed-size-only check**

`EigenMallocGuard` sees only Eigen code instantiated in the test TU (the library is compiled without `EIGEN_RUNTIME_NO_MALLOC`), and `AllocCounter` does not see `std::malloc`. So the source is also checked mechanically:
Run: `MSYS_NO_PATHCONV=1 wsl -- bash -lc "grep -nE 'Dynamic|MatrixXd|VectorXd|std::vector|new |cout|throw' /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-degeneracy/core/degeneracy/src/degeneracy.cpp || echo CLEAN"`
Expected: `CLEAN`.

- [x] **Step 6: Format check, then commit**

```bash
git add core/degeneracy/src/degeneracy.cpp tests/unit/test_degeneracy.cpp
git commit -m "feat(degeneracy): projected update and covariance inflation"
```

---

### Task 3: Synthetic-scene component tests D-01..D-03, full build, clang-tidy

**Files:**
- Create: `tests/component/degeneracy_scenes.hpp`
- Create: `tests/component/test_degeneracy_scenes.cpp`
- Modify: `tests/component/CMakeLists.txt` (append one line, outside the `if(MSS_BUILD_BENCH)` block)

**Interfaces:**
- Consumes: `analyze`, `project_update`, `relax_covariance` (Tasks 1-2); `common::skew` (`mss/common/lie.hpp`).
- Produces (test-only, header-only, namespace `mss::test`):
  - `struct PlaneSample { common::Vec3 p_l; common::Vec3 n_l; };` (`n_l` unit; metres)
  - `std::vector<PlaneSample> make_corridor_scene(double yaw_rad);`
  - `std::vector<PlaneSample> make_room_scene();`
  - `std::vector<PlaneSample> make_floor_scene();`
  - `common::Mat6 point_to_plane_hessian(const std::vector<PlaneSample>& samples, double point_sigma_m, double normal_sigma, std::uint32_t seed);`

How the synthetic Hessians are built without a scan matcher: the sensor sits at the origin of l with `T_w_l = I`, so each sample's row of the scan matcher (scan_matcher.hpp with `R_w_l = I`) is `J = [n ; -(n^T skew(p))^T] = [n ; p x n]`, and `H = sum J J^T` (all IRLS weights 1). `point_to_plane_hessian` first perturbs each sample with `std::mt19937(seed)` and `std::normal_distribution<double>`: `p += N(0, point_sigma_m)` per axis, `n = normalize(n + N(0, normal_sigma))` per axis (models point and plane-fit noise). Scenes use 0.25 m grids at cell centres (planner's call):
- Corridor (built axis-aligned, then points and normals rotated by `R_z(yaw_rad)`): walls `y = +-1.5` (normals `-+y`), `x in {-10 + 0.25 i, i = 0..80}`, `z in {-0.875 + 0.25 j, j = 0..11}`; floor `z = -1.0` (normal `+z`), same x, `y in {-1.375 + 0.25 j, j = 0..11}`. 2916 samples. Corridor axis in l is `R_z(yaw) e_x`; only rho along it is unconstrained.
- Room 8 x 6 x 3 m: floor `z = -1` and ceiling `z = 2` over `x in {-3.875 + 0.25 i, i = 0..31}`, `y in {-2.875 + 0.25 j, j = 0..23}`; walls `x = +-4` (over that y grid) and `y = +-3` (over that x grid) with `z in {-0.875 + 0.25 k, k = 0..11}`. Normals point inward. 2880 samples.
- Floor: the room's floor samples only (768).

Reference values (numpy, different RNG, same sigmas 0.01): corridor at yaw 30 deg has eigvals ~ [0.3, 972, 1267, 3333, 3.3e4, 6.6e4] and an axis angle ~0.01 deg; the room's smallest eigenvalue is ~566. Both are far from the `eig_threshold: 100.0` boundary.

Common setup: `cfg = {true, 100.0, 10.0}` (default.yaml values with the flag on), `kYawRad` = 30 deg in radians, `point_sigma_m = 0.01`, `normal_sigma = 0.01`, seed 42, `axis6 = [R_z(kYawRad) e_x ; 0, 0, 0]`.

- [x] **Step 1: Write the scene header and the tests**

```cpp
TEST(DegeneracyScenes, D01_CorridorFlaggedAlongAxis) {
  // r = analyze(point_to_plane_hessian(make_corridor_scene(kYawRad), 0.01, 0.01, 42), cfg);
  // r.degenerate == {true, false, false, false, false, false};
  // std::acos(std::min(1.0, |r.eigvecs.col(0).dot(axis6)|)) < 5 deg in rad.   // D-01: < 5 deg
}
TEST(DegeneracyScenes, D02_RoomNotFlagged) {
  // r = analyze(point_to_plane_hessian(make_room_scene(), 0.01, 0.01, 42), cfg):
  // no flag set; r.eigvals(0) > cfg.eig_threshold.
}
TEST(DegeneracyScenes, D03_ProjectedUpdateAndRelaxedCovariance) {
  // Corridor H and r as in D-01; delta = (0.3, -0.2, 0.1, 0.01, -0.02, 0.03);
  // |v0 . project_update(delta, r)| < 1e-12 * delta.norm();                    // D-03: update = 0
  // cov = H.inverse() (fixed-size Mat6); c = relax_covariance(cov, r, cfg);
  // v0^T c v0 >= cfg.relax_factor * (1 - 1e-9) * v0^T cov v0.                   // D-03: >= k times
}
TEST(DegeneracyScenes, D03_DisabledIsIdentity) {
  // cfg.enabled = false on the corridor H: no flags; project_update and relax_covariance return
  // delta and cov bit for bit.
}
TEST(DegeneracyScenes, SinglePlaneThreeDirections) {
  // r = analyze(point_to_plane_hessian(make_floor_scene(), 0.0, 0.0, 42), cfg) (noise-free):
  // degenerate == {1, 1, 1, 0, 0, 0}; out = project_update(delta, r):
  // |out(0)|, |out(1)|, |out(5)| < 1e-9 * delta.norm() (rho_x, rho_y, phi_z removed whatever
  // basis the solver picked); |out(i) - delta(i)| < 1e-9 * delta.norm() for i in {2, 3, 4}.
}
```

- [x] **Step 2: Register and run**

Append `mss_add_test(test_degeneracy_scenes test_degeneracy_scenes.cpp)` to `tests/component/CMakeLists.txt` outside the `if(MSS_BUILD_BENCH)` block. Run the build command with `--target test_degeneracy_scenes`.
Expected: `[  PASSED  ] 5 tests.` (The functions already exist; if D-01 or D-02 fails, check the scene builder against the geometry above, do not tune the threshold.)

- [x] **Step 3: Full build and ctest**

Run the build command without `--target`. Expected: all tests pass and the last line is `BUILD OK: <home>/mss-build/feat-degeneracy-nogtsam`.

- [x] **Step 4: clang-tidy on the new source**

Run: `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-degeneracy && cmake -B ~/mss-build/feat-degeneracy-nogtsam -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && clang-tidy-18 -p ~/mss-build/feat-degeneracy-nogtsam core/degeneracy/src/degeneracy.cpp"`
Expected: no warnings in `core/degeneracy/` (fix any that appear there).

- [x] **Step 5: Format check, then commit**

```bash
git add tests/component/degeneracy_scenes.hpp tests/component/test_degeneracy_scenes.cpp tests/component/CMakeLists.txt
git commit -m "test(degeneracy): synthetic corridor, room and floor scenes for D-01..D-03"
```

PR note (for the controller): no ATE change is expected (`degeneracy.enabled` defaults to false and nothing in the pipeline calls the module yet). Bench numbers come with the `scan_match` integration, the first caller that runs.
