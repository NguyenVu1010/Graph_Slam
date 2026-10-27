# scan_match Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `mss::scan_match::point_to_plane_residual` and `ScanMatcher` (constructor + `align`): point-to-plane scan-to-map registration against the `VoxelMap`, Huber IRLS Gauss-Newton, degeneracy-aware update, 6x6 Hessian and covariance output, no allocation per scan.

**Architecture:** One STATIC library `mss_scan_match` with `src/point_to_plane.cpp` (residual + Jacobian), `src/plane_fit.{hpp,cpp}` (internal k-NN plane fit) and `src/scan_matcher.cpp` (the Gauss-Newton loop). `ScanMatcher` gains private members only (config copies, a `std::vector<common::Vec3>` holding the scan in double precision, sized `max_points` once, and a `map::NeighborBuffer`). Per iteration H and g are accumulated on the fly in scan order (single thread, so results are bit-reproducible); the degeneracy projection runs BEFORE the solve, so an exactly degenerate corridor is solved instead of failing LLT. Component tests use synthetic room and corridor maps with a known pose.

**Tech Stack:** C++17, Eigen 3.4 fixed-size (`SelfAdjointEigenSolver<Mat3>`, `LLT<Mat6>`), Sophus via `mss/common/lie.hpp`, GoogleTest, CMake `mss_add_test`, WSL `tools/wsl/build.sh`.

**Spec:** `core/scan_match/include/mss/scan_match/scan_matcher.hpp` + `config.hpp` (frozen contract); providers `core/map/include/mss/map/voxel_map.hpp` (`knn` over 27 voxels, `NeighborBuffer::kMaxNeighbors = 32`), `core/degeneracy/include/mss/degeneracy/degeneracy.hpp` + `docs/derivations/degeneracy.md`; `config/default.yaml` keys `scan_match.*`, `map.*`, `degeneracy.*`, `realtime.max_points_per_scan`; `docs/Test_CI_CD.md` section `### scan_match — point-to-plane` (S-01..S-05) and D-03 (`### degeneracy`); ADR-004 (LiDAR factor covariance = inverse Hessian times a factor, inflated along degenerate directions), ADR-007 (voxel hash map, k-NN plane, RMS rejection), ADR-003/005/006/008/009/013; `docs/conventions.md`, `docs/derivations/lie_conventions.md`; `CLAUDE.md`.

## Contract issues

1. **Covariance scale has no residual variance.** Header: `cov = cov_scale * H^-1` with `H = sum w J^T J` (J unitless or m/rad, r in m). Statistically `cov = sigma_r^2 H^-1`; with `cov_scale = 1.0` ("dimensionless", YAML) the covariance assumes sigma_r = 1 m, about 1000x too large in variance for a 3 cm LiDAR. The dispatch asked for "H^-1 scaled by the robust residual". Proposed change (header doc + config comment, no signature change): `cov = cov_scale * s2 * H^-1`, `s2 = sum(w_i r_i^2) / (n_inliers - 6)` (robust residual variance, m^2), `cov_scale` stays dimensionless. **This plan implements the header as written** (`cov_scale * H^-1`); the derivation records both forms, and switching is a one-line change once the owner agrees.
2. **Solve order with degeneracy enabled (planner's call, pre-agreed by the controller).** The header reads "Solve `H xi = -g` ... then `xi = project_update(xi, report)`" and "singular Hessian when `LLT(H)` fails in any iteration or for the final cov". Taken literally, an exactly degenerate scene (corridor, `lambda = 0`) stops with "singular Hessian" before the projection runs. This plan keeps the signature and the `project_update` call every iteration but, when `deg_cfg.enabled`, factorises `H_reg = H + sum_{k flagged} (eig_threshold - lambda_k) v_k v_k^T` (flagged eigenvalues raised to `eig_threshold`) instead of H, for both the update and the final `cov = cov_scale * H_reg^-1`. With no flags `H_reg == H` bit for bit; for an invertible H the projected update equals `project_update(-H^-1 g)` exactly (proof in the derivation). "singular Hessian" remains only when an UNflagged direction is not positive definite, or when degeneracy is disabled and `LLT(H)` fails. `MatchResult::H` stays the raw, unprojected H. Proposed header doc update: describe this order and the `H_reg` covariance.

## Global Constraints

- Public signatures in `core/scan_match/include/mss/scan_match/*.hpp` are frozen; only additive private members in `scan_matcher.hpp` and new files under `core/scan_match/src/`. Do not modify `core/map`, `core/degeneracy`, `core/common` or `tests/component/degeneracy_scenes.hpp`.
- Tangent `[rho, phi]`, right perturbation `T_w_l <- T_w_l * Exp(xi)` (`common::boxplus`); H, g, xi, cov and the degeneracy report live in the tangent of l. No frame conversion in this module.
- Config (verbatim from `config/default.yaml`; tests load it with `pipeline::load_config(MSS_REPO_DIR "/config/default.yaml")` and copy-modify fields): `max_iterations: 10`, `rot_tol_rad: 1.0e-4`, `trans_tol_m: 1.0e-3`, `max_corr_dist_m: 1.0`, `plane_rms_max_m: 0.05`, `robust_huber_delta_m: 0.1`, `min_inliers: 100`, `cov_scale: 1.0`; `map.voxel_m: 0.5`, `max_points_per_voxel: 20`, `knn_k: 5`; `degeneracy.enabled: false`, `eig_threshold: 100.0`, `relax_factor: 10.0`; `realtime.max_points_per_scan: 65536`. No other tunables; no magic numbers in `core/`.
- Error strings exactly (literals, `Result::error(const char*)`): `"scan larger than reserved max_points"`, `"non-finite initial pose"`, `"too few inliers"`, `"singular Hessian"`.
- Hot path (`align`, `point_to_plane_residual`, `fit_plane`): no heap allocation, no logging, no exceptions, fixed-size Eigen only; the scan is read through `xyz()`, never copied as a `ScanFrame`.
- No `auto` for Eigen expressions; Eigen by `const&`; members `snake_case_`; include order own header, std, third party, project.
- `-Wall -Wextra -Wpedantic -Werror` (build.sh sets `MSS_WERROR=ON`); files LF; clang-format-18 clean; random tests use fixed `std::mt19937` seeds.
- Feature flag: with `degeneracy.enabled = false` the matcher never calls `analyze`; enabled with nothing flagged gives a bitwise identical result.
- Build/test (the only way to compile):
  `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && tools/wsl/build.sh --no-gtsam --build-dir ~/mss-build/feat-scan_match-nogtsam --target <test_binary_name>"`; full build + ctest: same without `--target`.
- Format check before each commit: `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && git ls-files -mo --exclude-standard '*.cpp' '*.hpp' | xargs -r clang-format-18 --dry-run --Werror"` -> no output, exit 0.
- Commits: Conventional Commits with trailers `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01DYAqFYL9xp2KFYiPfmqpc4`. Do not push. The PR will exceed the 400-line target (new module + tests); say so in the PR description.

## Review Focus

1. Exactly degenerate corridor (`lambda = 0` along the axis) with `degeneracy.enabled = true` -> `align` returns ok (not "singular Hessian"), recovers the observable DoF and does not move along the axis; covariance along the axis is `cov_scale / eig_threshold`. Pinned: Task 4 `ScanMatchComponent.D03_CorridorProjectedNotSingular`.
2. Scan containing NaN/Inf points -> skipped (no correspondence, no NaN in H); result bitwise equal to the same scan without them. Pinned: Task 2 `ScanMatchComponent.NonFinitePointsSkipped`.
3. One matcher reused across scans of different sizes (big then small) -> result bitwise equal to a fresh matcher (no stale scratch state). Pinned: Task 2 `ScanMatchComponent.ReuseMatchesFreshMatcher`.
4. Collinear map neighbourhood (map is a single line; plane-fit RMS 0, normal arbitrary) -> no NaN anywhere: either an error, or ok with finite pose, H and cov. Pinned: Task 4 `ScanMatchComponent.CollinearMapNeverNaN`.
5. Scan of exactly `max_points` points -> accepted; `max_points + 1` -> "scan larger than reserved max_points" before any map query. Pinned: Task 2 `ScanMatcher.ScanSizeBoundary`.

---

### Task 1: Derivation, static library, residual/Jacobian and plane fit (S-01)

**Files:**
- Create: `docs/derivations/scan_match.md`
- Modify: `docs/derivations/README.md` (one index line)
- Modify: `core/scan_match/CMakeLists.txt` (whole file)
- Create: `core/scan_match/src/point_to_plane.cpp`, `core/scan_match/src/plane_fit.hpp`, `core/scan_match/src/plane_fit.cpp`, `core/scan_match/src/scan_matcher.cpp` (includes only; Task 2 fills it)
- Create: `tests/unit/test_scan_match.cpp`
- Modify: `tests/unit/CMakeLists.txt` (append two lines)

**Interfaces:**
- Consumes: `common::{Vec3, Vec6, Mat3, Pose, skew, so3_exp, boxplus}`; `map::NeighborBuffer`.
- Produces: `double point_to_plane_residual(const common::Vec3& p_l, const common::Vec3& q_w, const common::Vec3& n_w, const common::Pose& T_w_l, common::Vec6* J_r_xi)` defined; internal header `core/scan_match/src/plane_fit.hpp`, namespace `mss::scan_match::detail`:
  ```cpp
  struct PlaneFit {
    common::Vec3 q_w = common::Vec3::Zero();   // m, centroid of the neighbours, frame w
    common::Vec3 n_w = common::Vec3::UnitZ();  // unit normal (eigenvector of the smallest eigenvalue), w
    double rms_m = 0.0;                        // m, RMS point-to-plane distance = sqrt(lambda_min / n)
  };
  // nullopt when buf.size < 3. Hot path: no allocation, no logging, no exceptions.
  std::optional<PlaneFit> fit_plane(const map::NeighborBuffer& buf);
  ```
  Test binary `test_scan_match` (Task 2 extends it), with `core/scan_match/src` on its include path.

- [x] **Step 1: Write `docs/derivations/scan_match.md`** (formulas, no code), sections:
  1. Residual `r = n_w . (T_w_l p_l - q_w)`; variable `xi = [rho, phi]`, right perturbation; frames (p_l in l, q_w and n_w in w).
  2. Jacobian: `T Exp(xi) p ~ R (p + rho + phi x p) + t` gives `J_r_xi = [ (R^T n_w)^T , -(R^T n_w)^T skew(p_l) ]` = `[n_l ; p_l x n_l]^T`, `n_l = R^T n_w`; units (unitless, m/rad). A flipped normal flips r and J together, so H and g do not depend on its sign.
  3. Plane fit: centroid `q`, scatter `S = sum (x_i - q)(x_i - q)^T`, `n` = eigenvector of `lambda_min(S)`, `rms = sqrt(lambda_min / n)`; acceptance per the header (>= 3 neighbours, `dist2[0] <= max_corr_dist_m^2`, `rms <= plane_rms_max_m`). A collinear neighbourhood has `rms = 0` and an arbitrary normal; rejecting it would need a new tunable (open question, not in this plan).
  4. Huber IRLS: `w = 1 if |r| <= delta else delta / |r|`; `H = sum w J^T J`, `g = sum w J^T r`; `H xi = -g`.
  5. Degeneracy-aware solve (Contract issue 2): `H_reg = H + sum_{flagged} (tau - lambda_k) v_k v_k^T`, tau = `eig_threshold`; proof that `P H_reg^-1 (-g) = P H^-1 (-g)` for invertible H (`P` = projector of `project_update`; same eigenvectors, unflagged components identical, flagged ones removed); singular only if an unflagged `lambda_k <= 0`.
  6. Covariance `cov = cov_scale * H^-1` (`H_reg^-1` when enabled), symmetrised `0.5 (C + C^T)`; the consistent form `cov_scale * s2 * H^-1`, `s2 = sum w r^2 / (n - 6)` (Contract issue 1); ADR-004 note that Hessian covariances are over-confident.
  7. Loop: stop test `|xi_rho| < trans_tol_m && |xi_phi| < rot_tol_rad` after the update; the reported H is the last one built (before the last update); error list.

- [x] **Step 2: Write the failing tests** in `tests/unit/test_scan_match.cpp`

```cpp
// S-01: 100 trials, std::mt19937(1): p_l, q_w uniform in [-10, 10]^3 m, n_w random unit,
// T_w_l = Pose{so3_exp(phi), t}, phi ~ N(0, 1) rad per axis, t uniform in [-5, 5]^3 m.
// Numerical J: central difference, h = 1e-6, on boxplus(T_w_l, h * e_k), k = 0..5.
TEST(PointToPlane, S01_JacobianMatchesNumerical) {
  // (J_analytic - J_numeric).norm() / std::max(J_numeric.norm(), 1e-12) < 1e-6, every trial.
}
TEST(PointToPlane, ResidualIsSignedDistanceAndNullJacobianOk) {
  // p_l = (1, 2, 3), T_w_l = Pose{so3_exp((0, 0, pi/2)), (0.5, 0, 0)}, q_w = 0, n_w = UnitX:
  // r == (T_w_l * p_l).x() within 1e-12; r with J_r_xi == nullptr equals r with a J.
}
TEST(PlaneFit, ExactPlaneRecoversNormalAndZeroRms) {
  // 5 points on z = 2 ((0,0), (0.1,0), (0,0.1), (0.1,0.1), (0.05,0.05)): |n_w.z()| == 1 within
  // 1e-12, q_w == centroid within 1e-12, rms_m < 1e-12.
}
TEST(PlaneFit, RmsMatchesOffPlaneSpread) {
  // corners of a 0.2 m square at z = +0.03 and at z = -0.03 (8 points): rms_m == 0.03 within 1e-9.
}
TEST(PlaneFit, FewerThanThreeIsNullopt) {
  // buf.size = 0, 1, 2 -> nullopt; size = 3 non-collinear -> has_value.
}
```

- [x] **Step 3: Register and run to verify it fails.** Append to `tests/unit/CMakeLists.txt`:
`mss_add_test(test_scan_match test_scan_match.cpp alloc_counter_impl.cpp)` and
`target_include_directories(test_scan_match PRIVATE ${CMAKE_SOURCE_DIR}/core/scan_match/src)`.
Run the build command with `--target test_scan_match`. Expected: link error `undefined reference to mss::scan_match::point_to_plane_residual`.

- [x] **Step 4: CMake + implement.** `core/scan_match/CMakeLists.txt` follows `core/degeneracy/CMakeLists.txt`: `add_library(mss_scan_match STATIC src/point_to_plane.cpp src/plane_fit.cpp src/scan_matcher.cpp)`, alias `mss::scan_match`, `mss_set_target_defaults`, PUBLIC include dir (same generator expressions), `target_link_libraries(mss_scan_match PUBLIC mss::common mss::map mss::degeneracy)`. Residual: `n_l = R^T n_w` once, `r = n_w.dot(T_w_l * p_l - q_w)`, J per the header formula. `fit_plane`: centre first, then accumulate S; `Eigen::SelfAdjointEigenSolver<common::Mat3>::compute` (not `computeDirect`: more accurate on near-flat sets; planner's call); `rms_m = sqrt(max(lambda_0, 0) / n)`.

- [x] **Step 5: Run to verify it passes.** Build command with `--target test_scan_match`. Expected: `[  PASSED  ] 5 tests.` and `BUILD OK`.

- [x] **Step 6: Format check, then commit**
```bash
git add docs/derivations core/scan_match tests/unit/test_scan_match.cpp tests/unit/CMakeLists.txt
git commit -m "feat(scan_match): point-to-plane residual, Jacobian and k-NN plane fit"
```

---

### Task 2: `ScanMatcher` Gauss-Newton loop (degeneracy disabled), S-02, no-alloc

**Files:**
- Modify: `core/scan_match/include/mss/scan_match/scan_matcher.hpp` (add `#include <vector>` and a `private:` section only)
- Modify: `core/scan_match/src/scan_matcher.cpp`
- Modify: `tests/unit/test_scan_match.cpp`
- Create: `tests/component/scan_match_scenes.hpp`, `tests/component/test_scan_match_component.cpp`
- Modify: `tests/component/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 `point_to_plane_residual`, `detail::fit_plane`; `map::VoxelMap::knn(p_w, 0, buf)`; `common::boxplus`.
- Produces: `ScanMatcher(const ScanMatchConfig&, const degeneracy::DegeneracyConfig&, std::size_t)` and `align(...)` defined. Private members: `ScanMatchConfig cfg_; degeneracy::DegeneracyConfig deg_cfg_; std::size_t max_points_; std::vector<common::Vec3> p_l_;` (m, frame l, `resize(max_points)` in the constructor) `map::NeighborBuffer nbr_;` and private helper `int build_system(const map::VoxelMap& map, std::size_t n, const common::Pose& T_w_l, common::Mat6& H, common::Vec6& g)` returning the inlier count (one iteration's accumulation; Task 4 does not change it).
  Scene helpers in `tests/component/scan_match_scenes.hpp`, namespace `mss::test`:
  - `std::vector<common::Vec3> room_points_w(double spacing_m)`: faces of the box x in [-4, 4], y in [-3, 3], z in [-1, 2] m, cell-centred grid of `spacing_m` on each face.
  - `std::vector<common::Vec3> corridor_points_w(double spacing_m, double half_length_m)`: walls y = +-1.5 (z in [-1, 2]) and floor z = -1 (y in [-1.5, 1.5]), x in [-half_length_m, half_length_m], open ends.
  - `std::vector<common::Vec3> line_points_w(double spacing_m, double half_length_m)`: points (x, 0, 0).
  - `void insert_points(map::VoxelMap& map, const std::vector<common::Vec3>& p_w)` (one ScanFrame, `T_w_l = I`).
  - `void fill_scan(common::ScanFrame& scan, const std::vector<common::Vec3>& p_w, const common::Pose& T_w_l)`: writes `T_w_l.inverse() * p_w` as float, offsets 0, ring 0, `set_size`.
  - `common::Pose perturb(const common::Pose& T, double trans_m, double rot_rad, std::mt19937& rng)`: `Pose{T.R * so3_exp(rot_rad * u1), T.p + trans_m * u2}`, u1, u2 random unit vectors.
  - `struct PoseError { double trans_m; double rot_rad; }; PoseError pose_error(const common::Pose& est, const common::Pose& truth)` (`|p_est - p_true|`, `|so3_log(R_true^T R_est)|`).
  - Constants (planner's calls): map spacing 0.125 m; room scan spacing 0.5 m (~720 points); `kTruthRoom = Pose{so3_exp((0.02, -0.03, 0.2)), (0.4, -0.3, 0.2)}`.

- [x] **Step 1: Write the failing unit tests** (append to `tests/unit/test_scan_match.cpp`; configs from `load_config`)

```cpp
TEST(ScanMatcher, ConstructorRejectsInvalidConfig) {
  // EXPECT_THROW(std::invalid_argument) for each: max_iterations = 0, rot_tol_rad = 0,
  // trans_tol_m = -1, max_corr_dist_m = 0, plane_rms_max_m = 0, robust_huber_delta_m = 0,
  // min_inliers = 5, cov_scale = 0, max_points = 0, rot_tol_rad = NaN (planner's call: checks
  // written `!(x > 0)` so NaN is rejected). Default config + 65536 does not throw.
}
TEST(ScanMatcher, ScanSizeBoundary) {
  // max_points = 4, empty map: 5-point scan -> reason "scan larger than reserved max_points";
  // 4-point scan -> "too few inliers" (size check passed, map empty).
}
TEST(ScanMatcher, NonFiniteInitialPose) {
  // T.p.x() = NaN -> "non-finite initial pose"; T.R(1, 2) = Inf -> same.
}
TEST(ScanMatcher, EmptyScanOrMapIsTooFewInliers) {
  // empty scan + 3 x 3 grid map -> "too few inliers"; 3 x 3 grid scan + empty map -> same.
}
```

- [x] **Step 2: Write the failing component tests** in `tests/component/test_scan_match_component.cpp` (room map `room_points_w(0.125)`, room scan `room_points_w(0.5)` seen from `kTruthRoom`)

```cpp
// S-02: 10 trials, std::mt19937(2), init = perturb(truth, 0.5 m, 10 deg) (the bound = worst case).
TEST(ScanMatchComponent, S02_KnownTransformRecovered) {
  // each trial: ok; converged; iterations <= 10; pose_error < 0.01 m and < 0.1 deg (in rad);
  // inliers >= min_inliers; cov == cov.transpose() exactly; LLT(cov) succeeds.
}
TEST(ScanMatchComponent, NoAllocationInAlign) {
  // build matcher + map, one warm-up align; AllocCounter::reset(); { EigenMallocGuard g;
  // r = align(...); } AllocCounter::count() == 0 and r.is_ok(). Same for the error path
  // (empty scan -> "too few inliers"): count() == 0.
}
TEST(ScanMatchComponent, DeterministicBitIdentical) {
  // two fresh matchers, same scan, map, init: memcmp-equal T_w_l.R, T_w_l.p, H, cov; equal
  // iterations, inliers, converged.
}
TEST(ScanMatchComponent, ReuseMatchesFreshMatcher) {
  // matcher A aligns a room_points_w(0.125) scan, then the 0.5 m scan; fresh matcher B aligns
  // the 0.5 m scan only: results bitwise equal.
}
TEST(ScanMatchComponent, NonFinitePointsSkipped) {
  // 0.5 m scan with extra points NaN at index 0, +Inf in the middle, -Inf at the end:
  // result bitwise equal to the clean scan's; inliers equal.
}
```

- [x] **Step 3: Register and run to verify they fail.** Append to `tests/component/CMakeLists.txt`:
```cmake
mss_add_test(test_scan_match_component test_scan_match_component.cpp
  ${CMAKE_SOURCE_DIR}/tests/unit/alloc_counter_impl.cpp)
target_include_directories(test_scan_match_component PRIVATE ${CMAKE_SOURCE_DIR}/tests/unit)
target_compile_definitions(test_scan_match_component PRIVATE
  MSS_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
```
Run the build command with `--target test_scan_match`, then with `--target test_scan_match_component`. Expected: link errors on `ScanMatcher::ScanMatcher` / `ScanMatcher::align`.

- [x] **Step 4: Implement the constructor and `align`** in `core/scan_match/src/scan_matcher.cpp`. Constructor: validate exactly the header list (`!(x > 0)` form; `min_inliers < 6`; `max_points == 0`), copy configs, `p_l_.resize(max_points)`. `align` (Task 4 adds the enabled branch at the `[Task 4]` marks):
```
if scan.size() > max_points_            -> error "scan larger than reserved max_points"
if any entry of T_init.R, T_init.p non-finite -> error "non-finite initial pose"
n = 0; for each point of scan.xyz(): if x, y, z finite: p_l_[n++] = double(point)   // once per align
T = T_init; result = {}
for it = 1 .. max_iterations:
  inliers = build_system(map, n, T, H, g)
     // per j < n: p_w = T * p_l_[j]; k = map.knn(p_w, 0, nbr_);
     // keep iff k >= 3 && double(nbr_.dist2[0]) <= max_corr_dist_m^2 && fit = fit_plane(nbr_)
     //   && fit->rms_m <= plane_rms_max_m;
     // r = point_to_plane_residual(p_l_[j], fit->q_w, fit->n_w, T, &J); w = Huber(r);
     // H.noalias() += w * J * J^T; g.noalias() += (w * r) * J
  if inliers < min_inliers                -> error "too few inliers"
  [Task 4: when enabled, report = analyze(H); A = H_reg]   else A = H
  llt.compute(A); if llt.info() != Eigen::Success -> error "singular Hessian"
  xi = llt.solve(-g)   [Task 4: when enabled, xi = project_update(xi, report)]
  T = boxplus(T, xi); result.H = H; result.inliers = inliers; result.iterations = it
  if xi.head<3>().norm() < trans_tol_m && xi.tail<3>().norm() < rot_tol_rad: converged = true; break
result.cov = cov_scale * llt.solve(Mat6::Identity()) from the LAST factorisation, then 0.5 (C + C^T)
```
`Eigen::LLT<common::Mat6>` is a local (fixed size, no allocation).

- [x] **Step 5: Run to verify they pass.** Build command with `--target test_scan_match` (Expected: `[  PASSED  ] 9 tests.`), then with `--target test_scan_match_component` (Expected: `[  PASSED  ] 5 tests.`). If S-02 misses its iteration or accuracy bound, do not loosen it: stop and report the measured numbers (Test_CI_CD: thresholds change only in a PR with a reason).

- [x] **Step 6: Format check, then commit**
```bash
git add core/scan_match tests/unit/test_scan_match.cpp tests/component/scan_match_scenes.hpp tests/component/test_scan_match_component.cpp tests/component/CMakeLists.txt
git commit -m "feat(scan_match): Huber IRLS Gauss-Newton scan-to-map alignment"
```

---

### Task 3: Convergence basin (S-03, CSV), noise (S-04), outliers (S-05)

**Files:**
- Modify: `tests/component/test_scan_match_component.cpp`

**Interfaces:**
- Consumes: Task 2 scene helpers and `ScanMatcher`.
- Produces: CSV `${MSS_TEST_OUTPUT_DIR}/scan_match/s03_convergence_basin.csv`, i.e. `<build-dir>/test_output/scan_match/s03_convergence_basin.csv`; on the dev machine `~/mss-build/feat-scan_match-nogtsam/test_output/scan_match/s03_convergence_basin.csv`. Never written into the source tree (`MSS_REPO_DIR` is only read, for the YAML). Header row and columns: `trans_offset_m,rot_offset_deg,trials,successes,success_rate,mean_iterations`. A later `eval/` script plots it (out of scope here).

- [x] **Step 1: Write the tests**

```cpp
// S-03: grid trans_offset_m {0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0} x rot_offset_deg {0, 5, 10, 15, 20, 30}
// (planner's call: 42 cells keep the Debug + ASan CI job within budget), 20 trials per cell,
// std::mt19937(3), init = perturb(truth, t, r). Success = align ok AND pose_error < 0.01 m and
// < 0.1 deg (S-02 accuracy; planner's call, the converged flag is not required).
TEST(ScanMatchComponent, S03_ConvergenceBasin) {
  // std::filesystem::create_directories(parent); ASSERT the ofstream opened; one row per cell;
  // EXPECT success_rate at (0.3 m, 5 deg) >= 0.95 (>= 19 of 20); printf the CSV path and
  // RecordProperty("s03_csv", path).
}
// S-04: scan points + N(0, 0.02 m) per axis (std::mt19937(4)), map noise-free, 10 trials,
// init = perturb(truth, 0.3 m, 5 deg).
TEST(ScanMatchComponent, S04_GaussianNoise) {
  // each trial: ok; pose_error.trans_m < 0.03.
}
// S-05: 20 % of the scan points replaced by points uniform in the box [-4, 4] x [-3, 3] x [-1, 2] m
// (in w, then into l), std::mt19937(5), 10 trials, init = perturb(truth, 0.3 m, 5 deg).
TEST(ScanMatchComponent, S05_OutliersRobustKernel) {
  // each trial: ok; pose_error.trans_m < 0.05.
}
```

- [x] **Step 2: Run.** Build command with `--target test_scan_match_component`. Expected: `[  PASSED  ] 8 tests.`, the S-03 line printing the CSV path, and `MSYS_NO_PATHCONV=1 wsl -- bash -lc "wc -l ~/mss-build/feat-scan_match-nogtsam/test_output/scan_match/s03_convergence_basin.csv"` -> `43`. A failure here is an algorithm bug (use superpowers:systematic-debugging), not a reason to change thresholds.

- [x] **Step 3: Format check, then commit**
```bash
git add tests/component/test_scan_match_component.cpp
git commit -m "test(scan_match): S-03 convergence basin CSV, S-04 noise, S-05 outliers"
```

---

### Task 4: Degeneracy-aware solve and covariance (D-03)

**Files:**
- Modify: `core/scan_match/src/scan_matcher.cpp`
- Modify: `tests/component/test_scan_match_component.cpp`

**Interfaces:**
- Consumes: `degeneracy::analyze(const common::Mat6&, const DegeneracyConfig&)`, `degeneracy::project_update(const common::Vec6&, const common::DegeneracyReport&)`.
- Produces: `align` behaviour with `deg_cfg.enabled` per Contract issue 2.

Corridor fixture (planner's calls): map `corridor_points_w(0.125, 10.0)`; scan `corridor_points_w(0.25, 6.0)` (~1700 points) seen from `truth = Pose{I, (0, 0.1, 0)}`; init = `Pose{so3_exp((0, 0, 3 deg)), truth.p + (0.3, 0.1, 0)}` (0.3 m along the unobservable axis).

- [x] **Step 1: Write the failing tests**

```cpp
TEST(ScanMatchComponent, D03_CorridorProjectedNotSingular) {
  // deg enabled: ok (not "singular Hessian"); |p.y - truth.y| < 0.01, |p.z - truth.z| < 0.01,
  // rotation error < 0.1 deg; |p.x - init.p.x| < 0.01 (no motion along the degenerate axis; the
  // bound covers the O(|phi| |rho|) coupling of Exp); report = analyze(H, deg_cfg):
  // degenerate[0] && |report.eigvecs.col(0).head<3>().dot(R_w_l^T e_x)| > cos(5 deg);
  // v0^T cov v0 == cov_scale / eig_threshold within 1e-6 relative.
}
TEST(ScanMatchComponent, DegeneracyEnabledRoomEqualsDisabled) {
  // room scene, first S-02 init: enabled result bitwise equal to disabled (no flags -> H_reg == H).
}
TEST(ScanMatchComponent, NoAllocationWithDegeneracy) {
  // corridor, deg enabled: warm-up, reset, EigenMallocGuard, align -> count() == 0 and ok.
}
TEST(ScanMatchComponent, CollinearMapNeverNaN) {
  // map line_points_w(0.05, 10.0), scan line_points_w(0.1, 5.0) offset by (0, 0.02, 0.02),
  // deg disabled and enabled: either !ok, or ok with every entry of T_w_l, H, cov finite.
}
```

- [x] **Step 2: Run to verify D03 fails.** Build command with `--target test_scan_match_component`. Expected: `D03_CorridorProjectedNotSingular` FAILS ("singular Hessian", or x drifts / non-finite); the others may pass.

- [x] **Step 3: Implement the enabled branch** at the Task 2 marks: when `deg_cfg_.enabled`, `report = degeneracy::analyze(H, deg_cfg_)`; `H_reg = H`; for each flagged k, `const common::Vec6 v_k = report.eigvecs.col(k); H_reg.noalias() += (deg_cfg_.eig_threshold - report.eigvals(k)) * (v_k * v_k.transpose());`; factorise `H_reg`; after the solve `xi = degeneracy::project_update(xi, report)`. The final cov uses that factorisation (`cov_scale * H_reg^-1`); `result.H` stays the raw H. Disabled: unchanged, `analyze` not called.

- [x] **Step 4: Run to verify all pass.** Build command with `--target test_scan_match_component`. Expected: `[  PASSED  ] 12 tests.`

- [x] **Step 5: Format check, then commit**
```bash
git add core/scan_match/src/scan_matcher.cpp tests/component/test_scan_match_component.cpp
git commit -m "feat(scan_match): project degenerate directions before the Gauss-Newton solve"
```

---

### Task 5: Latency record, full build, clang-tidy

**Files:**
- Modify: `tests/component/test_scan_match_component.cpp`

**Interfaces:**
- Consumes: everything above.
- Produces: recorded `align` latency p50/p95 for the PR description (CLAUDE.md hot-path rule; `mss_bench` does not call this module yet).

- [x] **Step 1: Write the timing test** (recorded, never fails on time; same pattern and local `percentile` helper as `VoxelMapComponent.M03_InsertAndKnnTimingRecorded`)

```cpp
// Room map room_points_w(0.125); scan room_points_w(0.125) (~11.5k points, ADR-007 scan size);
// 50 aligns, std::mt19937(6), init = perturb(truth, 0.2 m, 2 deg); steady_clock around align only.
TEST(ScanMatchComponent, AlignTimingRecorded) {
  // printf("scan_match align_ms p50=%.2f p95=%.2f points=%zu iters_mean=%.1f\n", ...);
  // RecordProperty("align_p50_us", ...); RecordProperty("align_p95_us", ...);
  // EXPECT every align ok (correctness only).
}
```

- [x] **Step 2: Run it.** Build command with `--target test_scan_match_component`. Expected: `[  PASSED  ] 13 tests.` and the timing line. PR description: before = "n/a, new module", after = this line (desktop); Pi5: "pending pi5-bench (manual)".

- [x] **Step 3: Full build + ctest.** Build command without `--target`. Expected: all tests pass, `BUILD OK`.

- [x] **Step 4: clang-tidy on the new sources.** Run: `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && cmake -B ~/mss-build/feat-scan_match-nogtsam -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && clang-tidy-18 -p ~/mss-build/feat-scan_match-nogtsam core/scan_match/src/*.cpp"`. Expected: no warnings from `core/scan_match`; fix any and re-run Step 3.

- [x] **Step 5: Format check, then commit**
```bash
git add tests/component/test_scan_match_component.cpp core/scan_match
git commit -m "test(scan_match): record align latency p50/p95 on the synthetic room"
```

PR description states: tests S-01..S-05, D-03, no-alloc, determinism; the S-03 CSV location; the timing line; ATE "no change expected" (the pipeline does not call `ScanMatcher` yet); both Contract issues; PR size over 400 lines.
