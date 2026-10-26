# Pipeline Estimator (single-thread, deterministic) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire every merged module into `pipeline::Estimator` so that `run_once()`/`flush()` turn IMU, LiDAR, GNSS and wheel messages into an optimised trajectory, single-threaded and bit-for-bit reproducible, and fix the bench scan-drop regression.

**Architecture:** `Estimator::Impl` keeps the queues and the k-way merge it has today and hands every merged message to a new internal `detail::EstimatorCore` (`core/pipeline/src/estimator_core.{hpp,cpp}`), which owns the modules: IMU static init, an IMU ring buffer that feeds two `imu::Preintegrator`s (scan-to-scan prediction, keyframe-to-keyframe factor) cut exactly at scan `t_start` by an interpolated boundary sample, deskew trajectory building, scan-to-map matching, the keyframe decision, `backend::Graph` factors and `optimize()`, the ADR-016 keyframe store plus `VoxelMap` window cache with rebuild, GNSS gating/alignment and wheel deltas. Small pure helpers live in their own internal headers so each is unit-tested in isolation; the core is tested through the public `Estimator` on a synthetic box world with a known trajectory.

**Tech Stack:** C++17, Eigen, Sophus (via `mss/common/lie.hpp`), GTSAM only inside `backend`, GoogleTest, yaml-cpp (config parser, already PRIVATE).

**Spec:** `core/pipeline/include/mss/pipeline/{estimator,config,bounded_queue,merge_order}.hpp`; module headers `core/{imu,preprocess,scan_match,map,degeneracy,gnss,odom,backend}/include/mss/*/*.hpp`; `docs/Test_CI_CD.md` (`### core/common` C-04, C-08, C-09; `## Test tích hợp và hiệu năng` E-01..E-03; G-02); `docs/spec.md` (Dev loop); `docs/conventions.md`; `docs/derivations/{lie_conventions,imu_preintegration,scan_match,backend_factors,gnss,wheel_odom,map}.md`; ADR-003, 004, 005 (threads deferred), 008, 009, 011, 014, 015, 016; `CLAUDE.md`; dispatch brief items (1)-(7).

## Contract issues

The controller rules on these before implementation. Each task states what it does if a proposal is refused.

1. **GNSS alignment thresholds move to `GnssConfig` (gnss.hpp comment "pending", gnss plan Contract issue 2).** Proposal: add to `core/gnss/include/mss/gnss/config.hpp`
   `int align_min_pairs = 10; double align_min_spread_m = 1.0; double chi2_gate = 7.814727903251178;`
   (defaults equal today's constants so value-initialised configs in existing gnss tests keep their behaviour; `load_config` always overwrites them from YAML). `GnssAligner` gets two PRIVATE members `int min_pairs_; double min_spread_m_;` set from `cfg` in its existing constructor and used in `update()` instead of `kMinPairsForAlignment` / `kMinHorizontalSpreadM`. No public signature changes; the two `static constexpr` constants stay (documented as "default of GnssConfig::align_min_pairs / align_min_spread_m") so `test_interfaces_compile`'s `static_assert` still holds. If refused: Task 2 adds only `chi2_gate` and the aligner keeps its constants.
2. **Promote the chi-square helpers to the public gnss header (gnss plan Contract issue 1).** `gnss_residual`, `innovation_nis`, `chi_square_gate`, `kChi2Gate3Dof95` live in `core/gnss/src/innovation.hpp` (`mss::gnss::detail`, not installed); the pipeline needs them for G-02. Proposal: move the declarations to `gnss.hpp` (`namespace mss::gnss`) and leave `innovation.hpp` as `using` aliases so gnss tests compile unchanged. This plan does NOT depend on it: `mss_pipeline` adds `target_include_directories(mss_pipeline PRIVATE ${CMAKE_SOURCE_DIR}/core/gnss/src)` with a `# TODO(contract issue 2)` comment and includes `"innovation.hpp"`; if approved, swap the include and drop that line.
3. **Doc-comment-only changes in `estimator.hpp` (no signature change).** The "Current phase (Task 8)" text no longer holds. Replace it with: (a) `latest_state()`: `state.t` stays `Timestamp{}` until the IMU is initialised and the first keyframe exists; afterwards `state.t` is the `t_start` of the last scan that produced an estimate, `T_w_b`/`v_w_b`/`bias` from the front-end state (optimised keyframe state on keyframe scans, scan-matched pose plus IMU-predicted velocity otherwise), `cov` = marginal of the keyframe or the scan-match covariance in b, `keyframe_id` = last keyframe; (b) `run_once()`/`flush()`: allocation-free on non-keyframe scans; keyframe scans allocate inside `backend::Graph` (GTSAM iSAM2) and when a `KeyframeStore` slot grows; (c) class comment: modules are plugged in, `start()`/`stop()` remain no-ops (threads are a later PR, ADR-005). If refused, the code still behaves as described and the PR notes the stale comment.
4. **Optional, not needed by this plan: diagnostic counters in `Estimator::Stats`.** Proposal: append `n_keyframes, n_match_failures, n_skipped_scans, n_map_rebuilds, n_gnss_factors, n_odom_factors` (additive fields, source compatible) so bench.json can report them. This plan tests those numbers through the internal `detail::CoreCounters` instead; nothing depends on the proposal.

## Global Constraints

- C++17; builds clean with `-Wall -Wextra -Wpedantic -Werror` (MSS_WERROR=ON) on GCC 13 and Clang; `clang-format-18` clean (`.clang-format`).
- `core/` never includes ROS; only `core/backend` includes GTSAM; the pipeline talks to the backend only through `mss/backend/graph.hpp`.
- Frames/convention: `T_a_b` maps b to a; right perturbation `T <- T * Exp(xi)`, `xi = [rho, phi]`; quaternions only at I/O, Hamilton `(w, x, y, z)`, order commented at each construction.
- Time is `common::Timestamp` (int64 ns); `seconds_between` only for dt; never compare floating-point times.
- No `auto` for Eigen expressions; fixed-size Eigen in hot code; Eigen args by `const&`.
- No magic numbers: every tunable from `Config` (YAML, validated in `load_config`); unit in the name or comment.
- Per-message processing returns `Status`/`Result` or logs and skips; never throws or aborts on sensor data; constructors may throw `std::invalid_argument`.
- Hot path (per scan, per IMU sample): no heap allocation, no exception, no per-point logging; non-keyframe scans are allocation-free (tested with `AllocCounter` + `EigenMallocGuard`).
- Determinism: no wall clock and no RNG in estimation; two runs over the same input give bit-identical outputs (E-02).
- Threads: none. `start()`/`stop()` stay no-ops that log the existing warnings (ADR-005 threads are a later PR).
- Tests: fixed seeds; every analytic Jacobian vs numerical differentiation, relative error < 1e-6.
- Build/test only through `tools/wsl/build.sh` (Windows has no toolchain). No-GTSAM command:
  `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-pipeline && tools/wsl/build.sh --no-gtsam --build-dir ~/mss-build/feat-pipeline-nogtsam --target <T>"`
  GTSAM command (same, without `--no-gtsam`, `--build-dir ~/mss-build/feat-pipeline`). Files stay LF.
- Format check before each commit: `MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd /mnt/c/Users/NguyenVD/workspace/graph_slam_wt/feat-pipeline && git diff --name-only --cached --diff-filter=AM -- '*.cpp' '*.hpp' | xargs -r clang-format-18 --dry-run --Werror"` prints nothing.
- Commits: Conventional Commits, scope `pipeline` (or `gnss`/`bench` where the file lives there), the session's attribution trailers. Do not push.
- PR size: this PR exceeds the 400-line guideline (integration of eight modules); the PR description says so and points reviewers at the per-task commits.
- PR description: ATE "first real estimate, no baseline yet (Plan B adds E-01..E-03)"; bench numbers (p50/p95/CPU, desktop) from Task 6 Step 7; Pi5 numbers not required (scan matching code unchanged).

## Review Focus

1. **Scan match fails (featureless or empty scan after filtering).** Expected: no keyframe, no map insert, `latest_state()` follows the scan with the IMU-predicted pose, processing continues. Pinned: Task 5 `PipelineComponent.ScanMatchFailureFallsBackToImuPrediction`.
2. **IMU samples missing inside a scan span (E-03 drops 5 % of IMU).** Expected: the deskew trajectory bridges the gap (the preintegrator integrates over the longer dt), samples stay strictly increasing. Pinned: Task 3 `ImuTrajectory.BridgesMissingSamples`.
3. **A scan not newer than the previous one (out of order, duplicate stamp).** Expected: skipped with one log line and a counter; state untouched; no crash. Pinned: Task 5 `PipelineComponent.OldScanSkipped`.
4. **`flush()` with a scan whose end no IMU sample reaches (data ends mid-scan).** Expected: the scan is consumed (n_scans counts it), no estimate update, no deskew extrapolation, no crash. Pinned: Task 5 `PipelineComponent.UncoveredScanSkippedAtFlush`.
5. **A 5 m GNSS jump after alignment, and a NaN pose covariance (stub backend marginal).** Expected: the jump is rejected by the chi-square gate and never fed to the aligner; a NaN covariance is treated as zero, not as "reject everything" or "accept everything". Pinned: Task 4 `GnssFusion.JumpRejectedAfterAlignment`, `GnssFusion.NonFinitePoseCovTreatedAsZero`.

---

## File Structure

| File | Responsibility |
| --- | --- |
| `docs/derivations/pipeline.md` (create) | Keyframe-boundary IMU cut, deskew trajectory, l->b covariance, LiDAR factor measurement, GNSS gate and time shift, keyframe/rebuild metrics, synthetic box world |
| `docs/adr/017-single-thread-estimator-data-flow.md` (create), `docs/adr/README.md` (row) | Design choices of the data flow (below) |
| `core/gnss/include/mss/gnss/config.hpp`, `core/gnss/include/mss/gnss/gnss.hpp`, `core/gnss/src/aligner.cpp` (modify) | Contract issue 1 |
| `core/pipeline/src/config.cpp`, `config/default.yaml` (modify) | New `gnss.*` keys |
| `core/pipeline/src/imu_buffer.{hpp,cpp}` (create) | Ring of recent IMU samples, linear interpolation, range copy |
| `core/pipeline/src/imu_trajectory_builder.{hpp,cpp}` (create) | `imu::ImuTrajectory` for one scan |
| `core/pipeline/src/frames.{hpp,cpp}` (create) | `lidar_cov_in_body`, `degeneracy_in_body` |
| `core/pipeline/src/keyframe_store.{hpp,cpp}` (create) | ADR-016 keyframe store |
| `core/pipeline/src/keyframe_policy.{hpp,cpp}` (create) | `is_new_keyframe`, `pose_moved`, `MapWindow`, `odom_delta_in_time` |
| `core/pipeline/src/gnss_fusion.{hpp,cpp}` (create) | gate_fix + chi-square + aligner + pending fixes shifted to keyframe time |
| `core/pipeline/src/estimator_core.{hpp,cpp}` (create) | The per-message algorithm; owns all modules |
| `core/pipeline/src/estimator.cpp`, `core/pipeline/include/mss/pipeline/estimator.hpp` (modify) | Drain hands messages to the core; doc comments (Contract issue 3) |
| `core/pipeline/CMakeLists.txt` (modify) | New sources, PRIVATE include of `core/gnss/src` |
| `bench/main.cpp` (modify) | run_once() before every scan push |
| `tests/unit/test_pipeline_internals.cpp` + `tests/unit/test_pipeline_gnss_fusion.cpp` (create) | Unit tests of the helpers |
| `tests/component/box_world.hpp` (create) | Ray-cast box world, analytic motion, IMU/GNSS/wheel, mssdata writer |
| `tests/component/test_pipeline_component.cpp` (create) | ATE, no-alloc, robustness |
| `tests/unit/test_estimator.cpp`, `tests/unit/test_config.cpp`, `tests/unit/test_gnss_aligner.cpp`, `tests/component/test_bench_replay.cpp`, `tests/unit/CMakeLists.txt`, `tests/component/CMakeLists.txt` (modify) | Updated expectations and new tests |
| `README.md` (modify) | Bench output semantics (pose after IMU init), new gnss keys |

---

### Task 1: Derivation doc and ADR-017

**Files:**
- Create: `docs/derivations/pipeline.md`, `docs/adr/017-single-thread-estimator-data-flow.md`
- Modify: `docs/adr/README.md` (one row), `docs/derivations/README.md` (one line), `core/pipeline/include/mss/pipeline/estimator.hpp` (comments only, if Contract issue 3 is approved)

**Interfaces:**
- Produces: the formulas every later task cites by section number (`pipeline.md §N`).

- [x] **Step 1: Write `docs/derivations/pipeline.md`** with these sections (English, formulas in the style of `scan_match.md`):
  - §1 IMU cut at keyframe/scan time. Samples with `t < t_start` are integrated, then the interpolated sample `s(t_start)` (acc and gyro linear in time between the bracketing samples); the next interval starts by re-feeding `s(t_start)`. Hence `delta.dt_s * 1e9 == t_j - t_i` up to rounding, inside the backend tolerance `1.5e9 / rate_hz` ns, and `predict()` returns a state exactly at `t_start`. A raw sample at exactly `t_start` is its own interpolation.
  - §2 Deskew trajectory. `x0` = predicted state at `t_start`; preintegrator reset at `x0.bias`, fed `s(t_start)` then each sample with `t_start < t`, up to and including the first sample with `t >= t_end` (`t_end = scan_end_time`); `T_b0_b(t_k) = x0.T_w_b^-1 * predict(x0, g_w).T_w_b`. Subsampling when more than `ImuTrajectory::kMaxImuPerScan` entries: keep entry indices `round(j * (N - 1) / (kMax - 1))`, `j = 0..kMax-1` (both ends kept).
  - §3 LiDAR result to body. `T_w_b = T_w_l * T_b_l^-1`; perturbation `T_w_l Exp(xi_l) T_l_b = T_w_b Exp(Ad(T_b_l) xi_l)`, so `J_xib_xil = Ad(T_b_l)` and `cov_b = Ad(T_b_l) cov_l Ad(T_b_l)^T` (`transform_covariance(cov_l, T_b_l)`); degeneracy eigenvectors `V_b = Ad(T_b_l) V_l`, eigenvalues and flags unchanged (no re-normalisation).
  - §4 LiDAR factor measurement. `T_bi_bj = (T_w_bi^ref)^-1 * T_w_bj^match`, where `T_w_bi^ref` is the pose keyframe i had when it was inserted into the map and used as the prediction reference (optimised pose if `optimize()` succeeded, else matched pose); covariance = `cov_b` of scan j (relative-to-map uncertainty, as LIO-SAM; ADR-004 notes it is optimistic).
  - §5 GNSS. Pre-gate `gate_fix(m, max_cov_m2)`; once aligned, residual `r = p_e_g - T_e_w (T_w_b p_b_g)` and `J_r_xi` from `gnss_residual`, `S = m.cov + J_r_xi P J_r_xi^T` with `P` the latest pose covariance (`P := 0` when any entry is non-finite), reject when `NIS > gnss.chi2_gate`; only accepted fixes update the aligner. Shift to keyframe time: `p_e_g' = p_e_g + R_e_w (T_w_bk p_b_g - T_w_b(t_fix) p_b_g)` with front-end poses; `cov` unchanged.
  - §6 Metrics. Keyframe: `|p_ref - p| >= keyframe_dist_m` or `|so3_log(R_ref^T R)| >= keyframe_rot_rad`. Rebuild: oldest window keyframe moved, `|dp| > rebuild_trans_m` or `|so3_log(dR)| > rebuild_rot_rad` (strict). Odom time check: `2 |t0 - t_i| <= 3 P` and `2 |t1 - t_j| <= 3 P` in integer ns, `P` = last observed wheel interval.
  - §7 Synthetic box world (shared with Plan B). Static for `static_s`, then speed ramp over `ramp_s` with constant curvature `kappa = W / V`, `tau = t - T0 - static_s`:
    `v = 0 (tau < 0); V (1 - cos(pi tau / ramp_s)) / 2 (0 <= tau < ramp_s); V (tau >= ramp_s)`,
    `s = 0; V/2 (tau - (ramp_s/pi) sin(pi tau / ramp_s)); V ramp_s / 2 + V (tau - ramp_s)`,
    `a_t = V pi / (2 ramp_s) sin(pi tau / ramp_s)` in the ramp, else 0; `psi = kappa s`, `omega = kappa v`,
    `p_w_b = (1/kappa) [sin psi, 1 - cos psi, 0]`, `R_w_b = Rz(psi)`,
    `a_w = a_t [cos psi, sin psi, 0] + kappa v^2 [-sin psi, cos psi, 0]`,
    `acc_b = R_w_b^T (a_w - g_w)`, `gyro_b = [0, 0, omega]`. `static_s = ramp_s = 0` reduces to the constant circle of `sim/mss_sim/synth.py`.
- [ ] **Step 2: Write ADR-017** (Vietnamese like the other ADRs, template `000-template.md`, status "đề xuất"): decisions = (a) single-thread `EstimatorCore` behind the existing queues; (b) IMU cut by interpolated boundary sample (§1); (c) only keyframe clouds go into the `VoxelMap` (consistent with ADR-016 rebuild), inserted at the optimised pose; (d) LiDAR factor measurement per §4; (e) GNSS fixes attached to the next keyframe, shifted per §5, `T_e_w` = latest aligner estimate per factor; (f) no estimate before IMU init; failed scan match = IMU prediction, no keyframe; (g) stub backend (MSS_WITH_GTSAM=OFF): optimize errors fall back to the front-end state, logged once per failure streak. Alternatives table: per-scan map insertion; fixed `T_e_w` after first alignment; keyframe at every GNSS fix. Consequences: run_once allocates on keyframe scans (GTSAM); `T_e_w` refinement feeds back own trajectory.
- [x] **Step 3: Apply Contract issue 3 to `estimator.hpp` comments** (only if approved; text as in the issue). Build: GTSAM command with `--target test_estimator` still compiles. Expected: `BUILD OK`.
- [ ] **Step 4: Commit**
```bash
git add docs/derivations/pipeline.md docs/derivations/README.md docs/adr/017-single-thread-estimator-data-flow.md docs/adr/README.md core/pipeline/include/mss/pipeline/estimator.hpp
git commit -m "docs(pipeline): derivation and ADR-017 for the single-thread estimator data flow"
```

---

### Task 2: GNSS gate and alignment thresholds in config (dispatch item 5)

**Files:**
- Modify: `core/gnss/include/mss/gnss/config.hpp`, `core/gnss/include/mss/gnss/gnss.hpp` (private members + comments, Contract issue 1), `core/gnss/src/aligner.cpp`, `core/pipeline/src/config.cpp:390-393`, `config/default.yaml` (gnss block)
- Test: `tests/unit/test_config.cpp`, `tests/unit/test_gnss_aligner.cpp`

**Interfaces:**
- Produces: `GnssConfig::align_min_pairs` (int, pairs), `align_min_spread_m` (m), `chi2_gate` (NIS threshold, 3 DoF); YAML `gnss.align_min_pairs: 10`, `gnss.align_min_spread_m: 1.0`, `gnss.chi2_gate: 7.814727903251178  # chi-square 3 DoF at q = 0.95 (G-02 band 90-99 %)`.

- [x] **Step 1: Write the failing tests**
```cpp
// test_config.cpp
TEST(Config, GnssGateAndAlignmentKeysLoaded) {
  // load_config(config/default.yaml): align_min_pairs == 10, align_min_spread_m == 1.0,
  // chi2_gate == 7.814727903251178
}
TEST(Config, GnssGateKeysValidated) {
  // overlay gnss.align_min_pairs: 0 -> error line "gnss.align_min_pairs: must be >= 1 (got 0)";
  // gnss.align_min_spread_m: 0 -> "gnss.align_min_spread_m: must be > 0 (got 0)";
  // gnss.chi2_gate: -1 -> "gnss.chi2_gate: must be > 0 (got -1)"
}
// test_gnss_aligner.cpp
TEST(GnssAligner, UsesConfiguredMinPairs) {
  // cfg.align_min_pairs = 3, spread fine (pairs on a 5 m circle): update #2 -> "not enough
  // pairs", update #3 -> ok with yaw within 1e-9 of the truth (noise-free pairs)
}
TEST(GnssAligner, UsesConfiguredSpread) {
  // cfg.align_min_spread_m = 10.0, 12 pairs on a 5 m circle -> "insufficient horizontal spread"
}
```
- [x] **Step 2: Run to verify they fail** — no-GTSAM command, `--target test_config`, then `--target test_gnss`. Expected: compile errors (no such member).
- [x] **Step 3: Implement.** Fields per Contract issue 1 (defaults 10 / 1.0 / 7.814727903251178, unit comments). Parser lines: `r.field("gnss.align_min_pairs", c.gnss.align_min_pairs, Range::kAtLeastOne); r.field("gnss.align_min_spread_m", c.gnss.align_min_spread_m, Range::kPositive); r.field("gnss.chi2_gate", c.gnss.chi2_gate, Range::kPositive);`. `GnssAligner`: private `int min_pairs_; double min_spread_m_;` from cfg; `aligner.cpp` uses them; update the two constant comments ("default of GnssConfig::..."). YAML keys after `max_cov_m2` with unit comments. If Contract issue 1 is refused: only `chi2_gate` (+ its tests), drop the aligner tests.
- [x] **Step 4: Run** — `--target test_config`, `--target test_gnss`, `--target test_interfaces_compile`, then `python -m pytest tests/python/test_sim03_consistency.py -q` on Windows from the worktree (overlay keys still valid). Expected: all PASS.
- [x] **Step 5: Format check, commit**
```bash
git add core/gnss config/default.yaml core/pipeline/src/config.cpp tests/unit/test_config.cpp tests/unit/test_gnss_aligner.cpp
git commit -m "feat(gnss): alignment thresholds and chi-square gate level from GnssConfig"
```

---

### Task 3: IMU buffer, deskew trajectory builder, l->b conversion

**Files:**
- Create: `core/pipeline/src/imu_buffer.{hpp,cpp}`, `core/pipeline/src/imu_trajectory_builder.{hpp,cpp}`, `core/pipeline/src/frames.{hpp,cpp}`, `tests/unit/test_pipeline_internals.cpp`
- Modify: `core/pipeline/CMakeLists.txt` (sources), `tests/unit/CMakeLists.txt`:
  `mss_add_test(test_pipeline_internals test_pipeline_internals.cpp test_pipeline_gnss_fusion.cpp alloc_counter_impl.cpp)` + `target_include_directories(test_pipeline_internals PRIVATE ${CMAKE_SOURCE_DIR}/core/pipeline/src ${CMAKE_SOURCE_DIR}/core/gnss/src)` (the gnss fusion file arrives in Task 4; add it to the list then).

**Interfaces** (all `namespace mss::pipeline::detail`, internal, not installed):
- Produces:
  - `class ImuBuffer` — fixed ring, capacity `kMaxImuQueue` (bounded_queue.hpp), storage allocated in the constructor:
    `bool push(const common::ImuSample& s)` (false, nothing stored, when `s.t <= back().t`; when full the oldest is dropped first); `std::size_t size() const; bool empty() const; bool full() const; const common::ImuSample& at(std::size_t i) const` (0 = oldest); `const common::ImuSample& front() const; const common::ImuSample& back() const; void pop_front();`
    `std::optional<common::ImuSample> interpolate(common::Timestamp t) const` (pipeline.md §1; nullopt outside `[front().t, back().t]`);
    `std::size_t copy_range(common::Timestamp t0, common::Timestamp t1, std::array<common::ImuSample, kMaxImuQueue>& out) const` (samples with `t0 <= t <= t1`, returns count).
  - `common::Status build_imu_trajectory(const ImuBuffer& imu, const common::NavState& x0, common::Timestamp t_end, const common::Vec3& g_w, imu::Preintegrator& scratch, imu::ImuTrajectory& traj)` — `x0.t` is the scan `t_start`; pipeline.md §2. Errors (literals): `"imu does not cover scan start"`, `"imu does not cover scan end"`. No allocation.
  - `common::PoseCovariance lidar_cov_in_body(const common::PoseCovariance& cov_l, const common::Pose& T_b_l)`; `common::DegeneracyReport degeneracy_in_body(const common::DegeneracyReport& r_l, const common::Pose& T_b_l)` (pipeline.md §3).

- [x] **Step 1: Write the failing tests** (`test_pipeline_internals.cpp`; IMU at 200 Hz, `g = 9.80665`, seeds fixed)
```cpp
TEST(ImuBuffer, PushRejectsNonIncreasing)            // push t=10, t=10 -> false, size 1
TEST(ImuBuffer, FullDropsOldest)                     // kMaxImuQueue + 1 pushes -> front().t == second sample
TEST(ImuBuffer, InterpolateLinear) {
  // samples t=0 acc (0,0,0), t=10 ns acc (10,0,0): interpolate(4)->acc.x == 4.0 exactly;
  // interpolate(10) returns the sample itself; interpolate(11) and interpolate(-1) -> nullopt
}
TEST(ImuBuffer, CopyRangeInclusive)                  // t = 0..9: copy_range(2, 5) -> 4 samples, t 2..5
TEST(ImuTrajectory, StaticIsIdentity) {
  // acc = (0,0,g), gyro 0, x0: R = I, v = 0, t = t_start between two samples; t_end = t_start + 99 ms
  // traj.t_start == t_start; samples[0].t == t_start; samples[0].T_b0_b exactly identity;
  // every T_b0_b: R within 1e-9 of I, p norm < 1e-9; samples[size-1].t >= t_end; t strictly increasing
}
TEST(ImuTrajectory, ConstantYawRateMatchesExp)      // gyro (0,0,0.5): R_b0_b(t_k) vs so3_exp({0,0,0.5*dt}) < 1e-9
TEST(ImuTrajectory, ConstantVelocityTranslation)    // x0.v = (1,0,0), acc = R^T(-g_w): p(t_k) vs (dt,0,0) < 1e-9
TEST(ImuTrajectory, MoreThanCapacitySubsampledWithEnds) {
  // 2 kHz IMU over a 100 ms scan: size == ImuTrajectory::kMaxImuPerScan, samples[0].t == t_start,
  // last.t == first buffered sample with t >= t_end, strictly increasing
}
TEST(ImuTrajectory, BridgesMissingSamples)          // Review Focus 2: constant yaw rate, 3 samples
                                                     // removed mid-scan: same check as above < 1e-9
TEST(ImuTrajectory, ErrorsWhenImuMissesScan)        // buffer starts after t_start -> "imu does not cover
                                                     // scan start"; ends before t_end -> "... scan end"
TEST(ImuTrajectory, BuildDoesNotAllocate)           // AllocCounter count() == 0, EigenMallocGuard
TEST(Frames, LidarCovInBodyMatchesNumericalJacobian) {
  // seed 7, 20 random T_b_l / T_w_l: J_num(:,k) = (boxminus(T_w_l*Exp(+h e_k)*T_l_b, T_w_b)
  //   - boxminus(T_w_l*Exp(-h e_k)*T_l_b, T_w_b)) / (2h), h = 1e-6; relative Frobenius error
  //   vs se3_adj(T_b_l) < 1e-6; lidar_cov_in_body(C, T_b_l) == Ad C Ad^T within 1e-12 (C random SPD)
}
TEST(Frames, DegeneracyEigvecsRotatedToBody)        // eigvecs == se3_adj(T_b_l) * eigvecs_l (1e-12);
                                                     // eigvals and degenerate[] unchanged
```
- [x] **Step 2: Run to verify they fail** — no-GTSAM command, `--target test_pipeline_internals`. Expected: compile errors.
- [x] **Step 3: Implement** the three units per the Interfaces block and pipeline.md §1-§3. `build_imu_trajectory`: count the entries first (N = 1 + samples in `(t_start, t_last]`), precompute the kept indices, integrate every sample, store only kept ones.
- [x] **Step 4: Run** — same target. Expected: PASS.
- [x] **Step 5: Format check, commit**
```bash
git add core/pipeline/src/imu_buffer.* core/pipeline/src/imu_trajectory_builder.* core/pipeline/src/frames.* core/pipeline/CMakeLists.txt tests/unit/test_pipeline_internals.cpp tests/unit/CMakeLists.txt
git commit -m "feat(pipeline): IMU ring buffer, deskew trajectory builder and LiDAR-to-body covariance"
```

---

### Task 4: Keyframe bookkeeping: store, policy, map window, GNSS fusion

**Files:**
- Create: `core/pipeline/src/keyframe_store.{hpp,cpp}`, `core/pipeline/src/keyframe_policy.{hpp,cpp}`, `core/pipeline/src/gnss_fusion.{hpp,cpp}`, `tests/unit/test_pipeline_gnss_fusion.cpp`
- Modify: `core/pipeline/CMakeLists.txt` (sources + `target_include_directories(mss_pipeline PRIVATE ${CMAKE_SOURCE_DIR}/core/gnss/src)  # TODO(contract issue 2)`), `tests/unit/test_pipeline_internals.cpp`, `tests/unit/CMakeLists.txt`

**Interfaces** (`namespace mss::pipeline::detail`):
- Consumes: `GnssConfig::chi2_gate` (Task 2); `gnss::detail::gnss_residual`, `gnss::detail::chi_square_gate`.
- Produces:
  - `struct KeyframeEntry { common::KeyId id; common::Timestamp t; common::ScanFrame cloud_l; };`
    `class KeyframeStore { explicit KeyframeStore(std::size_t max_keyframes); /* throws invalid_argument when 0 */ common::Status add(common::KeyId id, common::Timestamp t, const common::ScanFrame& cloud_l); std::size_t size() const; std::size_t capacity() const; const KeyframeEntry& at(std::size_t i) const; /* 0 = oldest kept */ const KeyframeEntry* find(common::KeyId id) const; /* nullptr when evicted or unknown */ std::uint64_t n_evicted() const; };`
    Ring of `max_keyframes` slots allocated once; `add` copies with `copy_from`, re-reserving a slot (`clear()` then `reserve(cloud.size())`) only when its capacity is too small; error `"keyframe id not increasing"`.
  - `bool is_new_keyframe(const common::Pose& T_w_b_ref, const common::Pose& T_w_b, const backend::BackendConfig& cfg);` `bool pose_moved(const common::Pose& a, const common::Pose& b, double trans_m, double rot_rad);` (pipeline.md §6)
  - `struct WindowEntry { common::KeyId id; common::Pose T_w_b_inserted; };` `class MapWindow { explicit MapWindow(std::size_t window_keyframes); void push(common::KeyId id, const common::Pose& T_w_b); std::size_t size() const; const WindowEntry& at(std::size_t i) const; /* 0 = oldest */ void set_pose(std::size_t i, const common::Pose& T_w_b); };` (fixed ring)
  - `bool odom_delta_in_time(const common::OdomDelta& d, common::Timestamp t_i, common::Timestamp t_j, std::int64_t period_ns);`
  - `class GnssFusion { GnssFusion(const gnss::GnssConfig& cfg, const common::Vec3& p_b_g); common::Status on_fix(const common::GnssMeasurement& m, const common::Pose& T_w_b_fix, const common::PoseCovariance& P_w_b); std::size_t collect(common::Timestamp t_k, const common::Pose& T_w_bk, std::array<common::GnssMeasurement, kMaxAuxQueue>& out); bool aligned() const; const common::Pose& T_e_w() const; std::uint64_t n_pending_dropped() const; };`
    `on_fix` order (pipeline.md §5): `gate_fix` (its reason returned); if aligned: chi-square (`"chi-square gate exceeded"` and other reasons returned; the aligner is not fed); `aligner.update` (refreshes `T_e_w` on success); if aligned after that, queue `{m, T_w_b_fix}` in a fixed ring of `kMaxAuxQueue` (full: drop oldest, count) and return ok, else return `Status::error("gnss not aligned yet")`. `collect` moves every queued fix with `t <= t_k` into `out` shifted per §5, returns the count.

- [ ] **Step 1: Write the failing tests**
```cpp
// test_pipeline_internals.cpp
TEST(KeyframeStore, AddKeepsInsertionOrder)          // ids 0,1,2 -> at(i).id == i, find(1)->t correct
TEST(KeyframeStore, EvictsOldestBeyondMaxKeyframes)  // max 3, add 5 -> ids {2,3,4}, n_evicted 2, find(0) == nullptr
TEST(KeyframeStore, RejectsNonIncreasingId)          // add(1), add(1) -> "keyframe id not increasing", size 1
TEST(KeyframeStore, CloudIsCopied)                   // mutate the source after add -> stored xyz unchanged
TEST(KeyframeStore, ReusedSlotDoesNotAllocate)       // max 2, clouds of equal size: after 2 adds, 4 more adds -> AllocCounter 0
TEST(KeyframePolicy, ByDistance)                     // keyframe_dist_m 1.0: dp 0.999 -> false, 1.0 -> true
TEST(KeyframePolicy, ByRotation)                     // keyframe_rot_rad = 10 deg: 9.99 deg -> false, 10 deg -> true
TEST(KeyframePolicy, PoseMovedIsStrict)              // rebuild 0.05 m / 0.5 deg: 0.05 m -> false, 0.0501 -> true; 0.51 deg -> true
TEST(KeyframePolicy, MapWindowKeepsLastN)            // window 3, push 5 -> at(0).id == 2; set_pose updates
TEST(KeyframePolicy, OdomDeltaTimeCheck)             // period 20 ms: |t0 - t_i| == 30 ms ok; 30 ms + 1 ns rejected (both ends)
// test_pipeline_gnss_fusion.cpp (T_e_w truth: yaw 30 deg, p_e_w (5,-3,1); body on a 5 m circle;
// p_b_g = (-0.20, 0.10, 0.60); cov diag(0.01, 0.01, 0.04); noise-free unless stated)
TEST(GnssFusion, NotAlignedUntilMinPairs)            // pairs 1..9 -> "gnss not aligned yet"; #10 -> ok, aligned()
TEST(GnssFusion, CollectShiftsFixToKeyframeTime) {
  // fix at body pose A, keyframe pose B = A moved 0.5 m along x: collected p_e_g' satisfies
  // gnss_residual(p_e_g', T_e_w(), B, p_b_g) norm < 1e-9; fixes with t > t_k stay queued
}
TEST(GnssFusion, JumpRejectedAfterAlignment) {       // Review Focus 5
  // after 20 consistent fixes, a fix 5 m off in x -> reason "chi-square gate exceeded";
  // T_e_w() bitwise unchanged; next collect() does not contain it
}
TEST(GnssFusion, NonFinitePoseCovTreatedAsZero)      // P = all NaN: consistent fix ok, jump rejected
TEST(GnssFusion, GateFixReasonReturned)              // status -1 -> "no fix"
TEST(GnssFusion, PendingBoundedDropsOldest)          // kMaxAuxQueue + 5 accepted fixes, no collect:
                                                     // collect returns kMaxAuxQueue newest, n_pending_dropped 5
```
- [ ] **Step 2: Run to verify they fail** — no-GTSAM command, `--target test_pipeline_internals`. Expected: compile errors.
- [ ] **Step 3: Implement** per Interfaces and pipeline.md §5-§6.
- [ ] **Step 4: Run** — same target. Expected: PASS.
- [ ] **Step 5: Format check, commit**
```bash
git add core/pipeline/src/keyframe_store.* core/pipeline/src/keyframe_policy.* core/pipeline/src/gnss_fusion.* core/pipeline/CMakeLists.txt tests/unit/test_pipeline_internals.cpp tests/unit/test_pipeline_gnss_fusion.cpp tests/unit/CMakeLists.txt
git commit -m "feat(pipeline): ADR-016 keyframe store, keyframe and rebuild policy, GNSS fusion"
```

---

### Task 5: EstimatorCore and Estimator wiring

**Files:**
- Create: `core/pipeline/src/estimator_core.{hpp,cpp}`, `tests/component/box_world.hpp`, `tests/component/test_pipeline_component.cpp`
- Modify: `core/pipeline/src/estimator.cpp` (drain hands messages to the core; `latest = core.latest()`), `core/pipeline/CMakeLists.txt`, `tests/unit/test_estimator.cpp`, `tests/component/CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 2-4; module APIs as declared in their headers.
- Produces:
  - `struct CoreCounters { std::uint64_t n_keyframes = 0, n_match_failures = 0, n_skipped_scans = 0, n_map_rebuilds = 0, n_gnss_factors = 0, n_gnss_rejected = 0, n_odom_factors = 0, n_odom_rejected = 0, n_backend_failures = 0; };`
  - `class EstimatorCore { explicit EstimatorCore(const Config& cfg); void on_imu(const common::ImuSample& s); void on_wheel(const common::WheelSample& s); void on_gnss(const common::GnssMeasurement& m); void on_scan(const common::ScanFrame& raw_l); bool has_estimate() const; const common::Estimate& latest() const; const CoreCounters& counters() const; };`
    Constructor throws `std::invalid_argument` (message names the key) when `ceil(imu.init_window_s * imu.rate_hz) + 1 > kMaxImuQueue` (`"imu.init_window_s: window does not fit the IMU buffer"`) or `map.window_keyframes > map.max_keyframes`; builds `ImuInitializer(cfg.imu)`, two `Preintegrator(cfg.imu)` (prediction, keyframe) + one scratch for deskew, `Preprocessor(cfg.preprocess, T_b_l, max_points)`, a second `Preprocessor({min_range_m, max_range_m, map.keyframe_voxel_m, false}, T_b_l, max_points)` for keyframe clouds, `VoxelMap(cfg.map)`, `ScanMatcher(cfg.scan_match, cfg.degeneracy, max_points)`, `Graph(cfg.backend, cfg.imu, frames.p_b_g)`, `WheelIntegrator(cfg.wheel_odom, frames.T_b_o)`, `GnssFusion(cfg.gnss, frames.p_b_g)`, `KeyframeStore(map.max_keyframes)`, `MapWindow(map.window_keyframes)`; reserves every scratch `ScanFrame` for `realtime.max_points_per_scan`. `g_w = [0, 0, -imu.gravity_mps2]` (also what `InitState::g_w` returns).
  - `Estimator` public behaviour per Contract issue 3; `Stats` unchanged.

Per-message algorithm (decisions; ADR-017, pipeline.md):
- `on_imu`: `imu_.push(s)`; if the buffer was full and the estimator is initialised, the evicted oldest sample is first integrated into both preintegrators (it precedes every future `t_start`).
- `on_wheel`: record the interval to the previous sample as the observed period; append to a fixed wheel ring (`kMaxAuxQueue`; full: integrate the oldest into `WheelIntegrator` first). Ignored for factors when `!wheel_odom.enabled` (still consumed).
- `on_gnss`: ignored unless `gnss.enabled` and an estimate exists; pose at `m.t` = copy of the prediction preintegrator advanced through buffered samples `<= m.t` plus `interpolate(m.t)`, then `predict(front_end_state, g_w)`; `fusion_.on_fix(m, pose, latest.cov)`; count rejections (`chi-square gate exceeded`, gate_fix reasons), no log for "not aligned yet".
- `on_scan` (`t = raw.t_start`, `t_end = scan_end_time(raw)`):
  1. `t <= last_scan_t_` -> `n_skipped_scans`, log once per call `"scan not newer than the previous one, skipped"`, return.
  2. Not initialised: `try_init` on `imu_.copy_range(t - init_window, t)`; failure -> return (no estimate; log the reason only when it differs from the previous attempt's). Success -> front-end state `{t, R_w_b, p 0, v 0, bias}`; both preintegrators reset at `bias` and fed `interpolate(t)`; drop buffered samples older than the last one `<= t`; wheel: integrate samples `<= t`, `take_delta()` discarded.
     Initialised: integrate buffered samples with `t_s < t`, then `interpolate(t)`, into both preintegrators (pipeline.md §1); `predicted = pred.predict(front_end_state, g_w)` with `.t = t`; wheel: integrate samples `<= t`.
  3. `imu_.back().t < t_end` (only possible at `flush()`) or `build_imu_trajectory` error -> `n_skipped_scans`, front-end state := predicted, reset prediction preintegrator at `t`, no `latest` update, return (Review Focus 4).
  4. `preprocessor_.process(raw, traj_, pre_)`.
  5. No keyframe yet: keyframe 0 = `graph.add_keyframe(t, predicted)`; `optimize()` (ok -> state from result, else front-end) ; keyframe cloud via the second Preprocessor; `store.add`, `map.insert(cloud, T_w_b * T_b_l)`, `window.push`; reference pose := that pose; publish; return.
  6. `matcher_.align(pre_, map_, predicted.T_w_b * T_b_l)`; error -> `n_match_failures`, log `"scan match failed: " + reason`, front-end := predicted, publish it (cov = previous `latest.cov`), reset prediction at `t`, return (Review Focus 1).
  7. `T_w_b = match.T_w_l * T_b_l.inverse()`, `cov_b = lidar_cov_in_body(match.cov, T_b_l)`; front-end := `{t, T_w_b, predicted.v_w_b, predicted.bias}`.
  8. `is_new_keyframe(ref, T_w_b, cfg.backend)`: `j = add_keyframe(t, front_end)`; `add_imu_factor(i, j, kf_preint.delta())`; `add_lidar_factor(i, j, ref.inverse() * T_w_b, cov_b, degeneracy.enabled ? &degeneracy_in_body(analyze(match.H, deg_cfg), T_b_l) : nullptr)`; wheel: `d = take_delta()`, if `wheel_odom.enabled` and `odom_delta_in_time(d, t_i, t, period)` -> `add_odom_factor` else count + one log line when enabled; GNSS: if enabled, `collect(t, T_w_b, buf)` and `add_gnss_factor(j, buf[k], fusion.T_e_w())` each; `optimize()`: ok -> front-end := result state (`.t = t`), `latest.cov` = result cov; error -> `n_backend_failures`, log only on the first failure of a streak. Then: keyframe preintegrator reset at the front-end bias and fed `interpolate(t)`; reference := front-end pose; keyframe cloud -> `store.add(j, t, cloud)`, `map.insert(cloud, ref * T_b_l)`, `window.push(j, ref)`; if `map.num_dropped()` grew, one warning. If optimize succeeded: rebuild check on `window.at(0)` with `graph.keyframe_state(id)` and `pose_moved(..., rebuild_trans_m, rebuild_rot_rad)` -> `map.clear()`, re-insert every window keyframe still in the store at its `keyframe_state` pose (inserted pose when that call errors), `set_pose`, `n_map_rebuilds`, one info line.
  9. Prune: when `seconds_between(last_prune_t_, t) >= prune_period_s`, `map.prune(front_end.p)`.
  10. Prediction preintegrator reset at the front-end bias and fed `interpolate(t)`; publish `latest = {front_end, cov (keyframe: optimised marginal, else cov_b), last keyframe id}`; `last_scan_t_ = t`.
- `Estimator::Impl::drain`: IMU -> `core.on_imu`, wheel -> `core.on_wheel`, GNSS -> `core.on_gnss`, scan -> `core.on_scan(pool[slot])`; counters `n_*` exactly as today; `latest = core.latest()` only when `core.has_estimate()`.

- [ ] **Step 1: Write `tests/component/box_world.hpp`** (header-only, `namespace mss::test`):
  `struct BoxWorldParams { std::int64_t t0_ns = 1'000'000'000; double static_s = 3.0, ramp_s = 2.0, motion_s = 15.0, speed_mps = 0.5, yaw_rate_radps = 0.25; common::Vec3 box_half_m{8.0, 5.0, 1.5}; int rings = 16, columns = 720; double elevation_min_deg = -15.0, elevation_max_deg = 15.0, scan_period_s = 0.1; double imu_rate_hz = 200.0, wheel_rate_hz = 50.0, gnss_rate_hz = 5.0; common::Pose T_b_l; /* ADR-011: (0.10, 0, 0.50), quaternion_wxyz (0.99984770, 0, 0.01745241, 0) */ common::Vec3 p_b_g{-0.20, 0.10, 0.60}; double acc_noise_density = 5.8e-3, gyro_noise_density = 3.5e-4; double gnss_sigma_m = 0.5; double gnss_yaw_e_w_deg = 30.0; std::uint32_t seed = 42; };`
  Functions: `common::NavState truth(const BoxWorldParams&, common::Timestamp t)` (pipeline.md §7), `common::ImuSample imu_at(...)` (+ white noise `density * sqrt(rate)` from `std::mt19937` seeded `seed`), `void scan_at(const BoxWorldParams&, common::Timestamp t_start, common::ScanFrame& out)` (column `c` at offset `c * scan_period / columns` ns, ray cast from `T_w_b(t) * T_b_l` against the six planes, ring-major order, nearest positive hit, points in l at that time), `common::Status write_box_dataset(const std::filesystem::path& dir, const BoxWorldParams&)` (DatasetWriter: IMU, wheel `{v, omega}`, GNSS `p_e_g = T_e_w (p_w_b + R_w_b p_b_g)` + N(0, sigma^2) with `cov = sigma^2 I`, gt at every scan, scans at `t0 + k * scan_period` for `k >= 1`; IMU, wheel and GNSS continue one IMU period past the last scan's end so `flush()` leaves no scan uncovered), `std::string box_overlay_yaml(const BoxWorldParams&, bool gnss_and_wheel)` (`frames.T_b_l`, `frames.p_b_g_m`, and when asked `gnss.enabled: true`, `wheel_odom.enabled: true`).
- [ ] **Step 2: Write the failing tests** (`test_pipeline_component.cpp`; dataset under `MSS_TEST_OUTPUT_DIR/<test name>`; replay = `DatasetReader` in time order into `Estimator` with `run_once()` before and after every scan push, `flush()` at the end; collect `latest_state()` after each `run_once` when `state.t` changed)
```cpp
TEST(PipelineComponent, BoxWorldAteWithinTolerance) {
  // ATE = sqrt(mean |p_est - truth(t).p|^2) over all estimates (w and the truth frame coincide
  // by construction: level start, yaw 0; no alignment). Tolerance (planner's call):
  // MSS_TEST_WITH_GTSAM ? 0.05 m : 0.10 m (front-end only); max |yaw error| < 1.0 deg (GTSAM) / 2.0 deg.
  // Estimates exist for every scan with t_start >= t0 + init_window_s (scans at t >= t0 + 2.0 s).
}
TEST(PipelineComponent, BoxWorldWithGnssAndWheelWithinTolerance)  // overlay enables both; same bounds
TEST(PipelineComponent, NoEstimateBeforeImuInit)    // IMU with gyro noise std 0.1 rad/s (> init_max_gyro_std_radps):
                                                     // latest_state().state.t == Timestamp{} after 10 scans
TEST(PipelineComponent, NonKeyframeScanDoesNotAllocate) {
  // EstimatorCore directly: init + keyframe 0 from the static phase, warm-up 5 static scans,
  // then 10 static scans (no keyframe): AllocCounter count() == 0 across on_imu/on_scan, EigenMallocGuard;
  // counters().n_keyframes == 1
}
TEST(PipelineComponent, ScanMatchFailureFallsBackToImuPrediction)  // Review Focus 1: after keyframe 0,
  // 5 empty scans: latest().state.t == each t_start, n_match_failures == 5, n_keyframes == 1
TEST(PipelineComponent, OldScanSkipped)              // Review Focus 3: repeat a t_start -> n_skipped_scans 1,
                                                     // latest() bitwise unchanged
TEST(PipelineComponent, UncoveredScanSkippedAtFlush) // Review Focus 4: Estimator, IMU stops 50 ms into the
                                                     // last scan; flush(): n_scans counts it, latest t unchanged
TEST(PipelineComponent, CoreConstructorValidatesBuffers)  // init_window_s 30 (6000 samples) -> invalid_argument
                                                           // naming imu.init_window_s; window_keyframes > max_keyframes
```
  CMake: `mss_add_test(test_pipeline_component test_pipeline_component.cpp ${CMAKE_SOURCE_DIR}/tests/unit/alloc_counter_impl.cpp)`, include dirs `tests/unit` and `core/pipeline/src`, `target_compile_definitions(test_pipeline_component PRIVATE MSS_TEST_WITH_GTSAM=$<BOOL:${MSS_WITH_GTSAM}>)`.
- [ ] **Step 3: Update `tests/unit/test_estimator.cpp`.** Add helper `void feed_static_imu(Estimator&, std::int64_t t_from, std::int64_t t_to)` (200 Hz, `acc = (0, 0, 9.80665)`, `run_once()` every 100 samples). `LatestStateTimestampFollowsScans`: prefix 2.0 s of static IMU before the first scan and an IMU sample covering each scan; keep the `t` checks; replace exact identity checks with `isApprox`/`norm < 1e-6`. `ScanQueueFullDropsOldest`: same prefix plus an IMU sample covering the last scan's end, keep the `t` check. `ScanWaitsForImuCoverage` and `PoolCyclesUnderSustainedOverflow` (no IMU init): expect `latest_state().state.t == Timestamp{}`; their counter checks stay. Add `LatestStateEmptyUntilImuInitialised`.
- [ ] **Step 4: Run to verify they fail** — GTSAM command, `--target test_pipeline_component`. Expected: compile errors (no `EstimatorCore`).
- [ ] **Step 5: Implement `EstimatorCore` and the drain change** per the algorithm above.
- [ ] **Step 6: Run** — GTSAM command `--target test_pipeline_component`, `--target test_estimator`; then no-GTSAM command for both targets. Expected: all PASS in both builds.
- [ ] **Step 7: Format check, commit**
```bash
git add core/pipeline tests/component/box_world.hpp tests/component/test_pipeline_component.cpp tests/component/CMakeLists.txt tests/unit/test_estimator.cpp
git commit -m "feat(pipeline): single-thread estimator core wiring IMU init, deskew, scan matching, keyframes, backend, map store, GNSS and wheel"
```

---

### Task 6: Bench scan-drop fix, E-02 determinism, docs, verification

**Files:**
- Modify: `bench/main.cpp:213-313` (Replayer), `tests/component/test_bench_replay.cpp`, `tests/component/CMakeLists.txt` (include dir for `box_world.hpp` is the same directory), `README.md`

**Interfaces:**
- Consumes: `tests/component/box_world.hpp` (`write_box_dataset`, `box_overlay_yaml`).
- Produces: `Replayer::feed` calls `run_once()` (through `drain_if(true)`) before every scan push as well as after it; `scan_limit_` removed; header comment updated (the scan queue never holds more than the scan being waited on plus the new one).

- [ ] **Step 1: Write the failing tests** (`test_bench_replay.cpp`)
```cpp
TEST(BenchReplay, ImuStampOffsetDoesNotDropScans) {
  // write_dataset variant with imu_offset_ms = 3 (IMU at kT0 + 3 ms + k * 5 ms); the IMU sample
  // covering scan k-1 then arrives after scan k's record. Default config (scan_queue_depth 2): exit 0, "\"n_dropped_scans\": 0," and "\"n_scans\": 10,"
}
TEST(BenchReplay, E02_TwoRunsAreBitIdentical) {
  // write_box_dataset (motion_s = 5.0) + overlay file from box_overlay_yaml(params, true);
  // run mss_bench twice into out1/out2: both exit 0, est.tum non-empty and byte-identical,
  // bench.json "n_scans" equal
}
```
  Update `ReplaysSyntheticDataset`: `write_dataset(data, 2000)` (2 s static IMU lead so the IMU initialises at scan 0); expect 9 lines (the last scan ends at 2998.9 ms, after the last IMU sample at 2995 ms, so `flush()` skips it: Review Focus 4); first line starts with `"3.000000000 "` and its x, y, z parse to `|v| < 1e-6`; JSON count expectations become 600 IMU, 15 GNSS, 150 odom.
- [ ] **Step 2: Run to verify `ImuStampOffsetDoesNotDropScans` fails** — no-GTSAM command `--target test_bench_replay`. Expected: FAIL on `n_dropped_scans` (the regression), the others PASS or fail only on the updated expectations.
- [ ] **Step 3: Implement** the Replayer change.
- [ ] **Step 4: Run** — `--target test_bench_replay` in the no-GTSAM and GTSAM builds. Expected: PASS.
- [ ] **Step 5: README** — under "Latency benchmark": `est.tum` has one line per scan that produced an estimate (none before the IMU static initialisation, which needs `imu.init_window_s` of still data); config section: the three new `gnss.*` keys.
- [ ] **Step 6: Full verification** — GTSAM command without `--target` and no-GTSAM command without `--target`. Expected: last line `BUILD OK: <dir>` for both (all ctest PASS). Then `python -m pytest tests/python -q` (Windows). Expected: PASS.
- [ ] **Step 7: Bench numbers for the PR** — in WSL, run `~/mss-build/feat-pipeline/bench/mss_bench config/default.yaml <test_output>/E02_TwoRunsAreBitIdentical/data --overlay <its overlay> --out /tmp/mss-bench-a` three times under `/usr/bin/time -v`; record push_scan p50/p95 from bench.json, wall time and `Percent of CPU`. Paste into the PR description ("before" = identity stub, not comparable).
- [ ] **Step 8: Format check, commit**
```bash
git add bench/main.cpp tests/component/test_bench_replay.cpp tests/component/CMakeLists.txt README.md
git commit -m "fix(bench): drain before every scan push so a late IMU sample cannot drop a scan; E-02 determinism test"
```
