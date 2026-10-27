# preprocess Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `mss::preprocess::Preprocessor` (bad-point/range filter, IMU deskew to `t_start`, first-point voxel downsample) as an allocation-free per-scan step that passes P-01..P-04.

**Architecture:** One pass over the input scan: filter -> deskew -> voxel test -> append to `out`. Deskew uses a per-scan table (`detail::DeskewTable`) that precomputes, for every IMU sample, `A_k = T_l_b * T_b0_b_k * T_b_l` and the segment twist `xi_k = Log(A_k^-1 * A_k+1)`, so each point costs one binary search plus one `se3_exp`. Downsampling uses a reserve-once open-addressing set of integer voxel keys (`detail::VoxelSet`). Both helpers live in `detail/` headers so they can be private members of `Preprocessor` without a pimpl (no public signature changes).

**Tech Stack:** C++17, Eigen, `mss::common` (lie.hpp via Sophus), GoogleTest; build via `tools/wsl/build.sh` in WSL.

**Spec:** `core/preprocess/include/mss/preprocess/preprocessor.hpp` (frozen contract), `core/preprocess/include/mss/preprocess/config.hpp`, `config/default.yaml` (`preprocess.*`), `core/imu/include/mss/imu/trajectory.hpp`, `core/common/include/mss/common/scan_frame.hpp`, `docs/Test_CI_CD.md` section `### preprocess — deskew, voxel` (P-01..P-04), `docs/conventions.md`, `docs/derivations/lie_conventions.md`, ADR-003/004/005/006/007/008/009/011 (section 3, LiDAR skew row)/013, `CLAUDE.md`.

## Contract issues

None. Everything below is implementable with the frozen public signatures. Changes to
`preprocessor.hpp` are limited to an additive `private:` section and includes of two new
`detail/` headers.

## Global Constraints

- C++17, `-Wall -Wextra -Wpedantic`, CI `-Werror`; `.clang-format` and `.clang-tidy` clean (clang-format-18 / clang-tidy-18 in WSL).
- Naming: types `PascalCase`, functions/variables `snake_case`, members `trailing_`, constants `kPascalCase`, namespace `mss::preprocess` (helpers in `mss::preprocess::detail`).
- No `auto` for Eigen expressions; fixed-size Eigen types; Eigen objects by `const&`.
- Hot path (`process()`, `DeskewTable::build/T_l0_l`, `VoxelSet::clear/insert`): no heap allocation, no logging, no exceptions. Errors are `common::Status::error("<literal>")` with the exact literals `"output capacity too small"` and `"empty imu trajectory"` (preprocessor.hpp).
- No magic numbers in `core/`: tunables come from `PreprocessConfig` (`min_range_m`, `max_range_m`, `downsample_voxel_m`, `deskew_enabled`); structural constants (hash multipliers, load factor) are named `kPascalCase` with a comment.
- Time is `common::Timestamp` (int64 ns); convert to double only to compute an interpolation fraction. Never compare floating-point times.
- Frames: `T_b_l` = LiDAR in body; `T_b0_b(t)` from `imu::ImuTrajectory`; output in `l0` = LiDAR at `in.t_start`. Right perturbation, tangent `[rho, phi]` (lie.hpp).
- Tests: fixed seeds (`std::mt19937{42}`; 42 = `random_seed` in `config/default.yaml`); no-alloc via `AllocCounter` + `EigenMallocGuard` (`tests/unit/alloc_counter.hpp`; the binary links `alloc_counter_impl.cpp` once).
- Only files under `core/preprocess/`, `tests/unit/`, `tests/component/`, `docs/derivations/preprocess.md` change. No new dependencies. No GTSAM.
- Every commit: Conventional Commits, ends with the attribution trailers from the session's system reminder (`Co-Authored-By: ...` and `Claude-Session: https://claude.ai/code/session_01DYAqFYL9xp2KFYiPfmqpc4`). Do not push.
- All files LF.

Build/test command (the only way to compile; `<T>` = target):
```
MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && tools/wsl/build.sh --no-gtsam --build-dir ~/mss-build/feat-preprocess-nogtsam --target <T>"
```
Format/tidy check (`<FILES>` = the task's C++ files; tidy only on `.cpp` under `core/`):
```
MSYS_NO_PATHCONV=1 wsl -- bash -lc "cd <worktree> && clang-format-18 --dry-run --Werror <FILES> && clang-tidy-18 -p ~/mss-build/feat-preprocess-nogtsam --quiet <CORE_CPP_FILES>"
```

## Review Focus

1. **Point times outside the trajectory** (later than `samples[size-1].t`, or `traj.size == 1`): must clamp to the end sample, never extrapolate or read past `size`. Pinned: Task 2 `DeskewTable.ClampsBeyondLastSample`, `DeskewTable.SingleSampleIsIdentityEverywhere`.
2. **Negative coordinates in the voxel grid**: `floor`, not truncation toward zero (`-0.1 m` and `+0.1 m` with 0.5 m voxels are different voxels). Pinned: Task 3 `VoxelKey.FloorsNegativeCoordinates`.
3. **Reusing one instance across scans**: the voxel set and deskew table must not leak state from the previous scan (a larger first scan, then a smaller second one sharing voxels). Pinned: Task 4 `Preprocessor.SecondScanMatchesFreshInstance`.
4. **A point with `t_offset_ns == 0` under non-zero motion** must come out bit-for-bit unchanged (trajectory.hpp: "a point with `t_offset_ns == 0` is unchanged"); a naive `T_l_b * I * T_b_l` leaves ~1e-17 residue that turns an exact `0.0F` into a tiny non-zero. Pinned: Task 4 `Preprocessor.ZeroOffsetPointUnchangedUnderMotion`.
5. **Non-finite config values** (NaN/inf `min_range_m`, `max_range_m`, `downsample_voxel_m`): `NaN < 0` is false, so the documented checks alone accept NaN and every point would be silently dropped. Reject at construction (planner's call; stricter than, not contrary to, the header). Pinned: Task 1 `Preprocessor.RejectsNonFiniteConfig`.

---

### Task 1: Derivation doc, STATIC library, constructor validation

**Files:**
- Create: `docs/derivations/preprocess.md`
- Create: `core/preprocess/src/preprocessor.cpp`
- Modify: `core/preprocess/CMakeLists.txt` (INTERFACE -> STATIC, follow `core/io/CMakeLists.txt`)
- Modify: `core/preprocess/include/mss/preprocess/preprocessor.hpp` (add `private:` members only)
- Create: `tests/unit/test_preprocessor.cpp`
- Modify: `tests/unit/CMakeLists.txt`

**Interfaces:**
- Produces: `Preprocessor` private members `PreprocessConfig cfg_; common::Pose T_b_l_; std::size_t max_points_;` (Tasks 2-4 add `detail::DeskewTable deskew_;` and `detail::VoxelSet voxels_;`). Test target `test_preprocess`.

- [x] **Step 1: Write `docs/derivations/preprocess.md`**

Sections (content fixed here, wording free):
1. *Deskew.* Point `p_l(t)` measured at `t = in.t_start + t_offset_ns`. `p_l0 = T_l0_l(t) * p_l(t)`, `T_l0_l(t) = T_b_l^-1 * T_b0_b(t) * T_b_l` (preprocessor.hpp). Skew model it inverts: ADR-011 section 3, `p_i <- T_l(t_i)^-1 * T_l(t_start) * p_i`.
2. *Interpolation.* `T_b0_b(t) = T_k * Exp(s * Log(T_k^-1 * T_k+1))`, `s = (t - t_k) / (t_k+1 - t_k)` (trajectory.hpp). Conjugation commutes with it: with `C = T_b_l`, `C^-1 (T_k Exp(s xi)) C = A_k Exp(s Ad(C^-1) xi)` and `A_k^-1 A_k+1 = C^-1 T_k^-1 T_k+1 C`, so precomputing `A_k = T_l_b T_k T_b_l` and `xi^l_k = Log(A_k^-1 A_k+1)` per sample gives the same `T_l0_l(t)` with one `Exp` per point. Tangent `[rho, phi]`, right perturbation (lie_conventions.md).
3. *Clamping.* `t <= t_0` -> `A_0`; `t >= t_{n-1}` -> `A_{n-1}`; no extrapolation.
4. *Exactness rule (planner's call).* `A_k` is set to the exact identity when `samples[k].T_b0_b` is bitwise identity (`R == I`, `p == 0`), and when `xi^l_k` is exactly zero or `t == t_k` the result is `A_k` without calling `Exp`. This makes static scans and `t_offset_ns == 0` points bit-exact.
5. *Voxel key.* `key = floor(p / downsample_voxel_m)` per axis as `int64`, computed from the float coordinates written to `out` (so a kept point is always inside its own voxel); the first point in input order wins. No residual and no Jacobian in this module, so there is no numerical-difference test; correctness is checked by P-01 against an analytic plane.

- [x] **Step 2: Write the failing constructor tests in `tests/unit/test_preprocessor.cpp`**

```cpp
// make(cfg, max_points) constructs a Preprocessor with T_b_l = identity.
// Default cfg = {min 0.5, max 100.0, voxel 0.5, deskew true} (config/default.yaml preprocess.*)
TEST(Preprocessor, ConstructsWithDefaultYamlValues)  // no throw, max_points 65536 (realtime.max_points_per_scan)
TEST(Preprocessor, VoxelZeroIsValid)                  // downsample_voxel_m = 0 -> no throw
TEST(Preprocessor, RejectsInvalidConfig) {
  EXPECT_THROW(make({-0.1, 100.0, 0.5, true}, 10), std::invalid_argument);  // min < 0
  EXPECT_THROW(make({0.5, 0.5, 0.5, true}, 10), std::invalid_argument);     // max <= min
  EXPECT_THROW(make({0.5, 0.4, 0.5, true}, 10), std::invalid_argument);
  EXPECT_THROW(make({0.5, 100.0, -0.5, true}, 10), std::invalid_argument);  // voxel < 0
  EXPECT_THROW(make({0.5, 100.0, 0.5, true}, 0), std::invalid_argument);    // max_points == 0
}
TEST(Preprocessor, RejectsNonFiniteConfig)  // NaN and +inf in each of the three doubles -> std::invalid_argument
```

- [x] **Step 3: Register the test and run it to verify it fails**

`tests/unit/CMakeLists.txt`: `mss_add_test(test_preprocess test_preprocessor.cpp alloc_counter_impl.cpp)` (Tasks 2-3 append their test files to this same line).
Run: build command with `--target test_preprocess`. Expected: link error (undefined `Preprocessor::Preprocessor`).

- [x] **Step 4: Implement the constructor in `core/preprocess/src/preprocessor.cpp`**

CMake: `add_library(mss_preprocess STATIC src/preprocessor.cpp)`, `mss_set_target_defaults(mss_preprocess)`, `PUBLIC` include dir, `target_link_libraries(mss_preprocess PUBLIC mss::common mss::imu)`; keep the `mss::preprocess` alias. The constructor throws `std::invalid_argument` with a message naming the YAML key (e.g. `"preprocess.max_range_m must be > preprocess.min_range_m"`) for the header's four conditions plus non-finite values. `process()` returns `common::Status::error("not implemented")` for now.

- [x] **Step 5: Run to verify pass**

Run: build command with `--target test_preprocess`, then without `--target` (full build + ctest; confirms pipeline still links). Expected: all PASS.

- [x] **Step 6: Format check and commit**

```bash
git add docs/derivations/preprocess.md core/preprocess tests/unit/test_preprocessor.cpp tests/unit/CMakeLists.txt
git commit -m "feat(preprocess): derivation, static library and config validation"
```

---

### Task 2: DeskewTable (per-scan SE(3) interpolation in the LiDAR frame)

**Files:**
- Create: `core/preprocess/include/mss/preprocess/detail/deskew_table.hpp`
- Create: `core/preprocess/src/deskew_table.cpp`
- Modify: `core/preprocess/CMakeLists.txt` (add source)
- Create: `tests/unit/test_preprocess_deskew_table.cpp` (append to `test_preprocess`)

**Interfaces:**
- Consumes: `common::se3_exp`, `common::se3_log`, `common::Pose`, `imu::ImuTrajectory`.
- Produces:
```cpp
namespace mss::preprocess::detail {
// Ownership: plain value, fixed arrays (no heap); one per Preprocessor, front-end thread only.
class DeskewTable {
 public:
  // Precondition: 1 <= traj.size <= ImuTrajectory::kMaxImuPerScan. No allocation.
  void build(const imu::ImuTrajectory& traj, const common::Pose& T_b_l);
  // T_l0_l at absolute sensor time t, clamped to [samples[0].t, samples[size-1].t]. No allocation.
  common::Pose T_l0_l(common::Timestamp t) const;

 private:
  std::array<common::Timestamp, imu::ImuTrajectory::kMaxImuPerScan> t_{};
  std::array<common::Pose, imu::ImuTrajectory::kMaxImuPerScan> A_{};   // T_l0_l at t_k
  std::array<common::Vec6, imu::ImuTrajectory::kMaxImuPerScan> xi_{};  // Log(A_k^-1 A_k+1), [rho, phi]
  std::size_t size_ = 0;
};
}  // namespace mss::preprocess::detail
```

- [x] **Step 1: Write the failing tests**

Fixture: `T_b_l` = `R_y(2 deg)`, `p = [0.10, 0, 0]` m (ADR-011 section 2 rig, planner's call); random trajectory from `std::mt19937{42}`: 21 samples 5 ms apart (200 Hz, `imu.rate_hz`), `samples[0] = (t0, identity)`, others `se3_exp` of a random `[rho, phi]` with components in [-0.2, 0.2].
```cpp
TEST(DeskewTable, MatchesSamplesAtSampleTimes)       // T_l0_l(t_k) vs T_b_l^-1 * T_k * T_b_l: max |dR|, |dp| < 1e-12
TEST(DeskewTable, MatchesBodyFrameInterpolation)     // 50 random t: vs T_b_l^-1 * (T_k * se3_exp(s * se3_log(T_k^-1 * T_k+1))) * T_b_l, < 1e-12
TEST(DeskewTable, ClampsBeyondLastSample)            // t_last + 1 ms and t_last + 1 s -> element-wise EXPECT_EQ to T_l0_l(t_last)
TEST(DeskewTable, ClampsBeforeFirstSample)           // t0 - 1 ns -> bitwise identity
TEST(DeskewTable, SingleSampleIsIdentityEverywhere)  // size 1: any t -> R == I, p == 0 (EXPECT_EQ per element)
TEST(DeskewTable, IdentitySampleGivesExactIdentity)  // t == t0 under the random motion: bitwise identity
TEST(DeskewTable, StaticTrajectoryIsExactIdentity)   // all samples identity: any t -> bitwise identity
TEST(DeskewTable, BuildAndQueryDoNotAllocate)        // AllocCounter::count() == 0 under EigenMallocGuard for build + 1000 queries
```

- [x] **Step 2: Run to verify they fail**

Run: build command with `--target test_preprocess`. Expected: compile error (`deskew_table.hpp` not found).

- [x] **Step 3: Implement `build` and `T_l0_l`**

`build`: per Task 1 doc sections 2 and 4 (`A_k` exact identity when the sample pose is bitwise identity; `xi_k` for `k < size-1`). `T_l0_l`: `std::upper_bound` over `t_[0, size_)`; clamp at both ends; `s = static_cast<double>(t.ns - t_k.ns) / static_cast<double>(t_k1.ns - t_k.ns)`; return `A_k` without `Exp` when `t == t_k` or `xi_k` is exactly zero, else `A_k * se3_exp(s * xi_k)` (assign `s * xi_k` to a `common::Vec6` first).

- [x] **Step 4: Run to verify pass**

Run: build command with `--target test_preprocess`. Expected: PASS.

- [x] **Step 5: Format check and commit**

```bash
git add core/preprocess tests/unit/test_preprocess_deskew_table.cpp tests/unit/CMakeLists.txt
git commit -m "feat(preprocess): deskew table with SE(3) interpolation in the LiDAR frame"
```

---

### Task 3: VoxelSet (reserve-once voxel key set)

**Files:**
- Create: `core/preprocess/include/mss/preprocess/detail/voxel_set.hpp`
- Create: `core/preprocess/src/voxel_set.cpp`
- Modify: `core/preprocess/CMakeLists.txt` (add source)
- Create: `tests/unit/test_preprocess_voxel_set.cpp` (append to `test_preprocess`)

**Interfaces:**
- Produces:
```cpp
namespace mss::preprocess::detail {
struct VoxelKey {
  std::int64_t x = 0, y = 0, z = 0;
  bool operator==(const VoxelKey& o) const;
};
// floor(coord / voxel_m) per axis, coordinates widened to double. Precondition: voxel_m > 0.
VoxelKey voxel_key(float x, float y, float z, double voxel_m) noexcept;

// Open addressing, linear probing. Ownership: owns two vectors sized once in the ctor.
class VoxelSet {
 public:
  explicit VoxelSet(std::size_t max_keys);    // throws std::invalid_argument if 0 or > UINT32_MAX - 1; allocates
  void clear() noexcept;                      // O(slot count), no allocation
  bool insert(const VoxelKey& key) noexcept;  // true if new; precondition size() < max_keys
  std::size_t size() const noexcept;

 private:
  std::vector<std::uint32_t> slots_;  // index into keys_; kEmptySlot = UINT32_MAX
  std::vector<VoxelKey> keys_;        // sized max_keys once
  std::size_t size_ = 0;
  std::uint64_t mask_ = 0;
};
}  // namespace mss::preprocess::detail
```
Slot count = smallest power of two >= `2 * max_keys` (load factor <= 0.5, `kSlotsPerKey = 2`, planner's call). Hash: `(x * 73856093) ^ (y * 19349669) ^ (z * 83492791)` on `uint64` (Teschner et al. 2003, the iVox/Faster-LIO hash ADR-007 follows; named constants), then `& mask_`.

- [x] **Step 1: Write the failing tests**

```cpp
TEST(VoxelKey, FloorsNegativeCoordinates) {
  EXPECT_EQ(voxel_key(-0.1F, 0.1F, -0.5F, 0.5), (VoxelKey{-1, 0, -1}));
  EXPECT_EQ(voxel_key(0.5F, -0.0F, 0.49F, 0.5), (VoxelKey{1, 0, 0}));
}
TEST(VoxelSet, InsertReportsNewOnlyOnce)        // insert k -> true, again -> false, size() == 1
TEST(VoxelSet, FillsToCapacityWithDistinctKeys)  // max_keys 1000: 10x10x10 grid of keys in [-5, 4]^3 all true, size 1000
TEST(VoxelSet, ClearForgetsAllKeys)              // after clear: size 0, earlier keys insert -> true
TEST(VoxelSet, ZeroCapacityThrows)               // VoxelSet(0) -> std::invalid_argument
TEST(VoxelSet, InsertAndClearDoNotAllocate)      // AllocCounter::count() == 0 for clear + 1000 inserts
```

- [x] **Step 2: Run to verify they fail**

Run: build command with `--target test_preprocess`. Expected: compile error (`voxel_set.hpp` not found).

- [x] **Step 3: Implement `voxel_key` and `VoxelSet`**

`voxel_key` uses `std::floor(static_cast<double>(x) / voxel_m)` cast to `std::int64_t`.

- [x] **Step 4: Run to verify pass**

Run: build command with `--target test_preprocess`. Expected: PASS.

- [x] **Step 5: Format check and commit**

```bash
git add core/preprocess tests/unit/test_preprocess_voxel_set.cpp tests/unit/CMakeLists.txt
git commit -m "feat(preprocess): reserve-once voxel key set"
```

---

### Task 4: Preprocessor::process (P-02, P-03, P-04, errors, no-alloc)

**Files:**
- Modify: `core/preprocess/include/mss/preprocess/preprocessor.hpp` (private: `detail::DeskewTable deskew_; detail::VoxelSet voxels_;` and includes of the two detail headers)
- Modify: `core/preprocess/src/preprocessor.cpp`
- Modify: `tests/unit/test_preprocessor.cpp`

**Interfaces:**
- Consumes: Task 2 `DeskewTable::build/T_l0_l`; Task 3 `voxel_key`, `VoxelSet::clear/insert`.
- Produces: `common::Status Preprocessor::process(const common::ScanFrame& in, const imu::ImuTrajectory& traj, common::ScanFrame& out)` per preprocessor.hpp.

- [x] **Step 1: Write the failing tests**

Helpers: `random_scan(n, seed)` fills a `ScanFrame` with points uniform in `[-20, 20]^3` m, intensity `i`, `t_offset_ns` uniform in [0, 100 ms], ring `i % 16`; `static_traj(t_start)` = 21 identity samples 5 ms apart; `moving_traj(t_start)` = same times with `se3_exp(t * [v; w])`, `v = [1, 0, 0]` m/s, `w = [0, 0, 1]` rad/s.

```cpp
// P-02 (Test_CI_CD): omega = 0, v = 0 -> output equals input.
// cfg {0.5, 100, voxel 0 (planner's call: downsampling off so "equal" is meaningful), deskew true};
// 5000 points with range in [1, 30] m (seed 42), static_traj.
TEST(Preprocessor, P02_StaticRobotOutputEqualsInput)  // size and t_start equal; xyz, intensity, t_offset_ns, ring EXPECT_EQ element-wise

// P-03 (Test_CI_CD): random cloud, voxel 0.5 m. cfg {0.5, 100, 0.5, deskew false}; 10000 points, seed 42.
TEST(Preprocessor, P03_VoxelDownsample) {
  // (a) at most one output point per voxel_key(out xyz, 0.5)
  // (b) every output point is bitwise one input point (all attributes), relative input order kept
  // (c) set of output voxels == set of voxels of the range-filtered input
  // (d) out == reference built in the test (std::set of key tuples, first in input order wins)
  // (e) a second process() on the same instance and process() on a fresh instance give bitwise-identical out
}
TEST(Preprocessor, P03b_DeskewedVoxelOutputRespectsGrid)  // deskew on, moving_traj, voxel 0.5: (a) holds on the written float coords

// P-04 (Test_CI_CD): NaN, < 0.5 m, > max range all removed, no crash. cfg {0.5, 100, 0, false}.
TEST(Preprocessor, P04_BadPointsRemoved)
  // bad: NaN in x, y, z separately; +inf; -inf; (0.49,0,0); (0,0,0); (100.01,0,0); (60,60,60)
  // kept: (0.5,0,0) and (100,0,0) (closed interval), (3,4,0). out == exactly these 3, in input order.
TEST(Preprocessor, P04_AllPointsFilteredIsOkAndEmpty)  // only bad points -> is_ok(), out.size() == 0
TEST(Preprocessor, EmptyInputIsOk)                     // in.size() == 0 -> ok, size 0, out.t_start == in.t_start

// Errors (preprocessor.hpp)
TEST(Preprocessor, OutputCapacityTooSmallIsError)       // out pre-filled with 3 points, capacity < in.size(): reason() == "output capacity too small", out.size() == 0
TEST(Preprocessor, EmptyTrajectoryIsErrorWhenDeskewing) // traj.size == 0, deskew on: reason() == "empty imu trajectory", out.size() == 0
TEST(Preprocessor, EmptyTrajectoryIgnoredWhenDeskewOff) // traj.size == 0, deskew off: is_ok()

// Review Focus
TEST(Preprocessor, ZeroOffsetPointUnchangedUnderMotion) // T_b_l = R_y(2 deg), p [0.10,0,0]; moving_traj; points (5,0,0), (0,5,0), (0,0,5) with t_offset 0 -> bitwise unchanged
TEST(Preprocessor, SecondScanMatchesFreshInstance)      // voxel 0.5, deskew on: process 10000-pt scan A (seed 42), then 2000-pt scan B (seed 7) -> bitwise equal to a fresh instance on B

// Hot path
TEST(Preprocessor, ProcessDoesNotAllocate)  // max_points 65536 (realtime.max_points_per_scan), 65536-pt scan, default cfg (deskew on, voxel 0.5),
                                            // moving_traj; AllocCounter::reset(); under EigenMallocGuard call process twice; both ok, count() == 0
```

- [x] **Step 2: Run to verify they fail**

Run: build command with `--target test_preprocess`. Expected: FAIL (`process` returns "not implemented").

- [x] **Step 3: Implement `process`**

The constructor now also constructs `voxels_(max_points)`. Body, in this order (reads through `in.xyz()` etc. spans, writes through `out.*_mut()`):
```
out.clear();
if (out.capacity() < in.size()) return error("output capacity too small");
if (deskew_enabled && traj.size == 0) return error("empty imu trajectory");
out.t_start = in.t_start;
if deskew: deskew_.build(traj, T_b_l_);   if voxel > 0: voxels_.clear();
for each input point i:
  skip if any of x, y, z is not std::isfinite;
  r = sqrt(double(x)^2 + double(y)^2 + double(z)^2); skip if r < min_range_m or r > max_range_m;
  if deskew: p = deskew_.T_l0_l(Timestamp{in.t_start.ns + t_offset_ns[i]}) * Vec3(x, y, z); x, y, z = float(p);
  if voxel > 0 and !voxels_.insert(voxel_key(x, y, z, voxel)): skip;
  write x, y, z, intensity, original t_offset_ns, ring at index n; ++n;
out.set_size(n); return ok;
```

- [x] **Step 4: Run to verify pass**

Run: build command with `--target test_preprocess`, then without `--target` (full build + ctest). Expected: all PASS.

- [x] **Step 5: Format check and commit**

```bash
git add core/preprocess tests/unit/test_preprocessor.cpp
git commit -m "feat(preprocess): filter, deskew and voxel downsample per scan"
```

---

### Task 5: P-01 component test (deskew of a known plane)

**Files:**
- Create: `tests/component/test_preprocess_deskew_plane.cpp`
- Modify: `tests/component/CMakeLists.txt` (`mss_add_test(test_preprocess_component test_preprocess_deskew_plane.cpp)`, outside the `if(MSS_BUILD_BENCH)` block)

**Interfaces:**
- Consumes: `Preprocessor` (Task 4), `common::se3_exp`.

Synthetic setup (planner's call except where cited):
- `T_b_l`: `R_y(2 deg)`, `p = [0.10, 0, 0]` m (ADR-011 section 2 rig).
- Motion: constant body twist `xi = [v; w]`, `v = [1.0, 0, 0]` m/s, `w = [0, 0, 1.0]` rad/s (ADR-011 section 2 limits); `T_b0_b(t) = se3_exp(t * xi)`, which SE(3) geodesic interpolation reproduces exactly.
- Trajectory: 21 samples 5 ms apart (200 Hz, `imu.rate_hz`), `samples[0] = (t_start, identity)`; scan length 0.1 s (10 Hz LiDAR, ADR-011).
- Plane in `l0`: `n = normalize([1, 0.2, 0.1])`, `n . p = 5` m.
- 3600 points: point i at `t_offset = i * 100 ms / 3600`, ring `i % 16`, direction in `l(t_i)`: azimuth `-60 + 120 * i / 3600` deg, elevation `-15 + 2 * (i % 16)` deg; intersect with the plane expressed in `l(t_i)` via `T_l0_l(t_i) = T_b_l^-1 * T_b0_b(t_i) * T_b_l`; store as float.
- cfg `{0.5, 100, voxel 0, deskew true}`, `max_points` 3600.

- [x] **Step 1: Write the test**

```cpp
TEST(PreprocessComponent, P01_DeskewConstantMotionPlane) {
  // out.size() == 3600; RMS over out of (n . p_out - 5) < 1e-3 m  (Test_CI_CD P-01: RMS < 1 mm)
}
TEST(PreprocessComponent, P01_WithoutDeskewPlaneIsSmeared) {
  // same scan, deskew_enabled = false: RMS > 1e-2 m (shows the test discriminates; planner's call)
}
```

- [x] **Step 2: Run to verify**

Run: build command with `--target test_preprocess_component`. Expected: PASS (Task 4 already implements deskew). If `P01_DeskewConstantMotionPlane` fails, debug `DeskewTable` (superpowers:systematic-debugging) before touching the test.

- [x] **Step 3: Full build, ctest, format and tidy on every changed file**

Run: build command without `--target`; then the format/tidy command on `core/preprocess/src/*.cpp core/preprocess/include/mss/preprocess/*.hpp core/preprocess/include/mss/preprocess/detail/*.hpp tests/unit/test_preprocess*.cpp tests/component/test_preprocess_deskew_plane.cpp` (tidy on `core/preprocess/src/*.cpp`). Expected: ctest 100% pass, no format diff, no tidy warnings.

- [x] **Step 4: Commit**

```bash
git add tests/component
git commit -m "test(preprocess): P-01 deskew of a plane under constant motion"
```

PR note (for the controller): `bench` does not call `preprocess` yet, so this branch has no before/after bench numbers; the no-alloc test stands in for the hot-path check. ATE effect: none expected (pipeline wiring is a later phase). Target <= 400 changed lines excluding this plan.
