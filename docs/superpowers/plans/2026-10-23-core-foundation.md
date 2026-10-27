# Nền chung cho các module C++ (Giai đoạn 0) — kế hoạch thực hiện

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cố định mọi thứ các module C++ dùng chung (phép toán Lie, cấu hình YAML, kiểu dữ liệu, header interface, bộ đọc dataset cho bench, môi trường build WSL/CI) để sau đó 6 module được phát triển song song trên worktree riêng.

**Architecture:** `common` thêm `lie.hpp` (bọc Sophus), `span.hpp`, `scan_frame.hpp`, kiểu dữ liệu trao đổi; `pipeline` thành STATIC với `config.cpp` (yaml-cpp, overlay deep-merge) và `Estimator` khung (pool `ScanFrame`, hàng đợi có giới hạn, đơn luồng); module mới `io` đọc/ghi định dạng `mssdata` phẳng; `bench` phát lại dataset vào `Estimator`; 9 header interface chỉ khai báo. Mọi build/test C++ chạy trong WSL qua `tools/wsl/build.sh`; CI mỏng build Sophus từ source có cache.

**Tech Stack:** C++17, GCC 13, CMake ≥ 3.22, Eigen 3.4, Sophus 1.22.10 (`SOPHUS_USE_BASIC_LOGGING`), yaml-cpp 0.8, GTest 1.14, GTSAM 4.2.0 (chỉ test đối chiếu), Python 3.12 (`mss_sim` cho bộ chuyển đổi).

**Spec:** `docs/superpowers/specs/2026-10-23-core-foundation-design.md` (gồm mục "Quyết định bổ sung sau builder check" 1–9, ràng buộc).

## Global Constraints

- CLAUDE.md: C++17; `-Wall -Wextra -Wpedantic`, `-Werror` (`MSS_WERROR=ON` trong mọi lần build của kế hoạch này); `#pragma once`; thứ tự include own/std/third-party/project; naming `PascalCase`/`snake_case`/`member_`/`kConst`/`mss::<module>`; không `auto` cho biểu thức Eigen; Eigen cố định, truyền `const&`; không số ma thuật (mọi tham số qua config struct); init/config được throw, per-message trả `Status`/`Result`; không heap/log/exception theo điểm trên đường nóng; chỉ `backend` include GTSAM (test đối chiếu nằm trong `tests/unit/backend/`, chỉ build khi `MSS_WITH_GTSAM`); chỉ `pipeline` parse YAML; `core/` không include ROS.
- `docs/conventions.md`: `T_a_b`, nhiễu phải `T <- T * Exp(delta)`, tangent `[rho, phi]`; Jacobian `J_<residual>_<variable>`; `Timestamp` int64 ns; quaternion chỉ ở I/O, Hamilton `(w, x, y, z)` và ghi thứ tự ở nơi dựng; gravity `[0, 0, -9.80665]`.
- Spec mục "Quyết định bổ sung" 1–9 là ràng buộc nguyên văn: khoá YAML và giá trị khởi đầu (mục 1), `gnss.bin` ENU + `status int8` + `origin_lla` trong `index.json` (2), pool `scan_queue_depth + 2` và `push_scan(const ScanFrame&)` memcpy (3), `ImuTrajectory` 128 phần tử `{t, T_b0_b}` và deskew về `t_start` (4), `PreintegratedDelta::cov` thứ tự `[phi, p, v]` (5), `H` theo nhiễu phải của `T_w_l`, `eigvals` tăng dần, `transform_covariance` (6), `KeyId = uint64_t`, `Span` tự viết (7), `frames.T_b_o` (8), Sophus từ source có cache + `SOPHUS_USE_BASIC_LOGGING` (9).
- Build/test C++ chỉ trong WSL: từ Bash tool dùng `MSYS_NO_PATHCONV=1 wsl -- bash -lc "<lệnh>"`, từ PowerShell `wsl -- bash -lc "<lệnh>"`; repo tại `<repo>`; GTSAM `$HOME/opt/gtsam`, Sophus `$HOME/opt/sophus`; `--parallel 3`.
- Python: ruff + pytest xanh (`python -m pytest -q -W error`), `eval/` không import `mss_sim`.
- clang-format: `python -m clang-format -i <files>` (bản pip trên Windows, cùng `.clang-format`), chạy trước mỗi commit C++.
- Commit theo Conventional Commits, mỗi task một commit, trailer attribution của phiên.

## Review Focus

1. YAML có khoá `_deg` hoặc quaternion không chuẩn hoá (norm lệch > 1e-6): `load_config` phải trả lỗi nêu tên khoá đầy đủ, không nạp giá trị sai im lặng. Test ở Task 4.
2. `ScanFrame::push` khi đầy: trả `false`, không ném, không cấp phát; `size()` không đổi. Test ở Task 3.
3. Overlay đặt scalar vào nơi base là mapping (hoặc ngược lại): lỗi nêu tên khoá, không ghi đè im lặng. Test ở Task 4.
4. File `lidar/<seq>.bin` bị cắt cụt hoặc `n` không khớp kích thước: `DatasetReader::next` trả `Status` lỗi nêu tên file, không crash, không đọc quá bộ đệm. Test ở Task 7.
5. `Estimator::push_imu` với timestamp không tăng: trả `Status` lỗi và bỏ qua mẫu, không làm hỏng hàng đợi. Test ở Task 8.

---

### Task 1: Môi trường build WSL, Sophus trong CMake và CI

**Files:**
- Create: `tools/wsl/setup.sh`, `tools/wsl/build.sh`
- Modify: `core/CMakeLists.txt` (find_package Sophus, yaml-cpp), `core/common/CMakeLists.txt`, `cmake/mss_coreConfig.cmake.in`, `.github/workflows/ci.yml` (job `core-no-gtsam`), `docker/deps.Dockerfile`, `README.md` (mục Build)
- Test: build xanh trong WSL ở cả hai chế độ

**Interfaces:**
- Produces: `tools/wsl/build.sh [--no-gtsam] [--asan] [--build-dir DIR] [--target T]` → cmake configure + build + ctest; build dir mặc định `$HOME/mss-build/<tên-nhánh với / thành ->`; exit code khác 0 khi bất kỳ bước nào lỗi; in dòng cuối `BUILD OK: <dir>`; target CMake `Sophus::Sophus` link PUBLIC vào `mss::common` với `target_compile_definitions(mss_common PUBLIC SOPHUS_USE_BASIC_LOGGING)`; `find_package(yaml-cpp REQUIRED)` ở `core/CMakeLists.txt` (dùng từ Task 4).

- [ ] **Step 1: Viết `tools/wsl/setup.sh`**: in danh sách apt cần cài (`ninja-build ccache clang-format-18 clang-tidy-18 libeigen3-dev libgtest-dev libyaml-cpp-dev libtbb-dev libboost-all-dev`) và dừng nếu thiếu gói bắt buộc (eigen, gtest, yaml-cpp, boost, tbb) với thông điệp gợi ý `sudo apt-get install`; build Sophus 1.22.10 vào `$HOME/opt/sophus` với `-DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_USE_BASIC_LOGGING=ON`; build GTSAM 4.2.0 vào `$HOME/opt/gtsam` với đúng cờ của `docker/deps.Dockerfile` cộng `--parallel 3`; bỏ qua bước đã có (kiểm tra `*Config.cmake`). Idempotent.
- [ ] **Step 2: Viết `tools/wsl/build.sh`** theo Interfaces; `CMAKE_PREFIX_PATH="$HOME/opt/gtsam;$HOME/opt/sophus"`; `--asan` → `-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"`; `ccache` nếu có; luôn `-DMSS_WERROR=ON`.
- [ ] **Step 3: CMake**: `core/CMakeLists.txt` thêm `find_package(Sophus REQUIRED)` và `find_package(yaml-cpp REQUIRED)`; `core/common/CMakeLists.txt` link `Sophus::Sophus` PUBLIC + compile definition; `cmake/mss_coreConfig.cmake.in` thêm `find_dependency(Sophus REQUIRED)` và `find_dependency(yaml-cpp REQUIRED)`.
- [ ] **Step 4: CI**: job `core-no-gtsam` cài thêm `libyaml-cpp-dev`, bước build Sophus từ source (clone tag 1.22.10, cùng cờ, install prefix `$HOME/opt/sophus`) bọc trong `actions/cache@v4` với key `sophus-1.22.10-ubuntu-24.04`, rồi configure với `-DCMAKE_PREFIX_PATH=$HOME/opt/sophus`. `docker/deps.Dockerfile`: thêm build Sophus trước GTSAM với cùng cờ vào `/usr/local`.
- [ ] **Step 5: Chạy** `MSYS_NO_PATHCONV=1 wsl -- bash -lc "<repo>/tools/wsl/build.sh --no-gtsam"` → `BUILD OK`, 7 test pass; rồi không `--no-gtsam` → `BUILD OK`.
- [ ] **Step 6: README** mục Build: thêm "WSL (dev machine)": hai lệnh trên và ghi chú apt cần sudo.
- [ ] **Step 7: Commit** `build: WSL build scripts, Sophus and yaml-cpp in CMake and CI`

### Task 2: `lie.hpp` — Lie group bọc Sophus (C-01, C-02)

**Files:**
- Create: `core/common/include/mss/common/lie.hpp`, `core/common/src/lie.cpp`, `docs/derivations/lie_conventions.md`
- Test: `tests/unit/test_lie.cpp`, `tests/unit/backend/test_lie_gtsam.cpp`, `tests/unit/backend/CMakeLists.txt`
- Modify: `tests/unit/CMakeLists.txt`, `tests/CMakeLists.txt` (hàm `mss_add_test` nhận biến `MSS_TEST_EXTRA_LIBS`; `add_subdirectory(unit/backend)` chỉ khi `MSS_WITH_GTSAM`)

**Interfaces:**
- Produces (namespace `mss::common`, tất cả trong `lie.hpp`): `Mat3 skew(const Vec3& v)`; `Mat3 so3_exp(const Vec3& phi)`; `Vec3 so3_log(const Mat3& R)`; `Mat3 so3_jr(const Vec3& phi)`; `Mat3 so3_jr_inv(const Vec3& phi)`; `Mat3 so3_jl(const Vec3& phi)`; `Mat3 so3_jl_inv(const Vec3& phi)`; `Pose se3_exp(const Vec6& xi)`; `Vec6 se3_log(const Pose& T)`; `Mat6 se3_adj(const Pose& T)`; `Pose boxplus(const Pose& T, const Vec6& delta)`; `Vec6 boxminus(const Pose& a, const Pose& b)` (= `se3_log(b.inverse() * a)`); `PoseCovariance transform_covariance(const PoseCovariance& cov_x, const Pose& T_y_x)` (= `Ad * cov * Ad^T` với `Ad = se3_adj(T_y_x)`); `Sophus::SE3d to_sophus(const Pose&)`; `Pose from_sophus(const Sophus::SE3d&)`. Tangent luôn `[rho, phi]`. Sophus chỉ được include trong `lie.hpp`/`lie.cpp`.

- [ ] **Step 1: Viết `docs/derivations/lie_conventions.md`**: định nghĩa Exp/Log, Jr/Jl, boxplus/boxminus, adjoint theo nhiễu phải `[rho, phi]`, ghi rõ Sophus `SE3::log()` trả `[rho, phi]` trùng GTSAM `Pose3::Logmap`.
- [ ] **Step 2: Test thất bại** `tests/unit/test_lie.cpp` (seed `std::mt19937 rng(42)`):

```cpp
TEST(Lie, So3ExpLogRoundTrip)   // 10000 phi: |phi| ~ U(0, pi-1e-6), thêm 100 mẫu |phi| < 1e-10 và 100 mẫu |phi| in (pi-1e-6, pi-1e-9); EXPECT_LT((so3_log(so3_exp(phi)) - phi).norm(), 1e-9)
TEST(Lie, Se3ExpLogRoundTrip)   // 10000 xi tương tự; EXPECT_LT((se3_log(se3_exp(xi)) - xi).norm(), 1e-9)
TEST(Lie, JrMatchesNumericDiff) // 200 phi ngẫu nhiên: cột k của so3_jr(phi) ≈ (so3_log(so3_exp(phi)^T * so3_exp(phi + h e_k)) )/h, h=1e-6; sai số tương đối (frobenius) < 1e-6
TEST(Lie, JrInvIsInverseOfJr)   // so3_jr(phi)*so3_jr_inv(phi) ≈ I, 1e-9
TEST(Lie, JlEqualsJrOfNegPhi)   // so3_jl(phi) ≈ so3_jr(-phi), 1e-12
TEST(Lie, AdjointMatchesConjugation) // Ad(T)*xi ≈ se3_log(T * se3_exp(xi) * T^{-1}) với |xi| nhỏ (1e-3), sai số < 1e-9
TEST(Lie, BoxplusBoxminus)      // boxminus(boxplus(T, d), T) ≈ d, 1e-9
TEST(Lie, TransformCovarianceIsCongruence) // cov_y == Ad cov_x Ad^T và đối xứng
TEST(Lie, SophusRoundTrip)      // from_sophus(to_sophus(T)) == T, 1e-12
```

- [ ] **Step 3: Chạy** `.../build.sh --no-gtsam --target test_lie` → FAIL (header thiếu).
- [ ] **Step 4: Cài đặt** `lie.hpp`/`lie.cpp` bằng `Sophus::SO3d`/`SE3d` (`Exp`, `log`, `leftJacobian`, `leftJacobianInverse`; Jr từ Jl(−phi); `Adj()`); `so3_log` xử lý góc gần π và gần 0 qua Sophus (không tự viết).
- [ ] **Step 5: Chạy** → PASS, không warning.
- [ ] **Step 6: Test đối chiếu GTSAM** `tests/unit/backend/test_lie_gtsam.cpp`: 1000 `T` ngẫu nhiên: `se3_log(T)` ≈ `gtsam::Pose3::Logmap(gtsam::Pose3(gtsam::Rot3(T.R), T.p))` sai số < 1e-9 **với hoán vị**: GTSAM `Logmap` trả `[phi, rho]` (rot trước), còn `se3_log` trả `[rho, phi]`; test khẳng định mối quan hệ này tường minh (`xi.head<3>() == gtsam.tail<3>()` v.v.) và ghi comment, vì `backend` sẽ dựa vào đó. `tests/unit/backend/CMakeLists.txt`: `mss_add_test(test_lie_gtsam test_lie_gtsam.cpp)` + `target_link_libraries(test_lie_gtsam PRIVATE gtsam)`; `tests/CMakeLists.txt` thêm `if(MSS_WITH_GTSAM) add_subdirectory(unit/backend) endif()`.
- [ ] **Step 7: Chạy** `build.sh` (có GTSAM) → PASS cả hai test. clang-format.
- [ ] **Step 8: Commit** `feat(common): Lie group wrappers over Sophus with Jacobian tests`

### Task 3: Kiểu dữ liệu chung, `Span`, `ScanFrame`

**Files:**
- Create: `core/common/include/mss/common/span.hpp`, `core/common/include/mss/common/scan_frame.hpp`, `core/common/src/scan_frame.cpp`, `tests/unit/alloc_counter.hpp`
- Modify: `core/common/include/mss/common/types.hpp`, `core/common/CMakeLists.txt`
- Test: `tests/unit/test_scan_frame.cpp`, `tests/unit/test_types.cpp`

**Interfaces:**
- Produces (`types.hpp`): `using KeyId = std::uint64_t;` `struct ImuSample{Timestamp t; Vec3 acc_b; Vec3 gyro_b;}`; `struct ImuBias{Vec3 bg; Vec3 ba;}`; `struct NavState{Timestamp t; Pose T_w_b; Vec3 v_w_b; ImuBias bias;}`; `struct Estimate{NavState state; PoseCovariance cov; KeyId keyframe_id;}`; `struct MatchResult{Pose T_w_l; PoseCovariance cov; Mat6 H; int inliers; int iterations; bool converged;}`; `struct DegeneracyReport{Vec6 eigvals; Mat6 eigvecs; std::array<bool, 6> degenerate;}` (eigvals tăng dần, cột `eigvecs` tương ứng); `struct PreintegratedDelta{Mat3 dR; Vec3 dv; Vec3 dp; double dt_s; Eigen::Matrix<double, 9, 9> cov; Mat3 J_dR_bg, J_dv_ba, J_dv_bg, J_dp_ba, J_dp_bg; ImuBias bias_lin;}` (cov khối `[phi, p, v]`); `struct GnssMeasurement{Timestamp t; Vec3 p_e_g; Mat3 cov; std::int8_t status;}`; `struct WheelSample{Timestamp t; double v_mps; double omega_radps;}`; `struct OdomDelta{Timestamp t0; Timestamp t1; Pose T_b0_b1; PoseCovariance cov;}`; `struct LoopConstraint{KeyId i; KeyId j; Pose T_bi_bj; PoseCovariance cov;}`. Mọi struct có docstring nêu khung và đơn vị; mọi Eigen member có `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` khi chứa kiểu cố định kích thước bội 16 byte.
- `span.hpp`: `template <class T> class Span{ Span(T* data, std::size_t size); T* begin(); T* end(); std::size_t size() const; T& operator[](std::size_t) const; bool empty() const; }` + `Span<const T>` từ `std::vector`/`std::array`.
- `scan_frame.hpp`: `class ScanFrame{ ScanFrame() = default; explicit ScanFrame(std::size_t capacity); ScanFrame(const ScanFrame&) = delete; ScanFrame& operator=(const ScanFrame&) = delete; ScanFrame(ScanFrame&&) noexcept = default; ScanFrame& operator=(ScanFrame&&) noexcept = default; void reserve(std::size_t capacity); std::size_t capacity() const; std::size_t size() const; void clear(); bool push(float x, float y, float z, float intensity, std::uint32_t t_offset_ns, std::uint16_t ring); void copy_from(const ScanFrame& other); /* memcpy, yêu cầu capacity >= other.size(), trả bool */ Timestamp t_start; Span<const float> xyz() const; /* 3*size */ Span<const float> intensity() const; Span<const std::uint32_t> t_offset_ns() const; Span<const std::uint16_t> ring() const; float* xyz_mut(); ... }` SoA, bộ nhớ cấp phát duy nhất trong `reserve`.
- `tests/unit/alloc_counter.hpp`: `struct AllocCounter{ static std::size_t count(); static void reset(); }` thay `operator new/delete` toàn cục trong các test binary dùng nó (đếm `new`).

- [ ] **Step 1: Test thất bại** `test_scan_frame.cpp`: `ReserveOnceThenPushNoAlloc` (reserve 1000; `AllocCounter::reset()`; push 1000 điểm → `count() == 0`, `size()==1000`); `PushWhenFullReturnsFalse` (push thứ 1001 → false, size vẫn 1000); `CopyFromNoAlloc` (frame B reserve 1000, `copy_from(A)` → bằng từng giá trị, `count()==0`; B capacity 10 → `copy_from` trả false); `MoveKeepsData`; `ClearKeepsCapacity`. `test_types.cpp`: `static_assert(std::is_trivially_copyable_v<ImuSample>)` v.v. cho các struct POD, `sizeof` không bị padding bất thường (chỉ kiểm tra biên dịch + vài static_assert).
- [ ] **Step 2: Chạy** `build.sh --no-gtsam --target test_scan_frame` → FAIL.
- [ ] **Step 3: Cài đặt** `span.hpp`, `scan_frame.hpp/.cpp`, `types.hpp`, `alloc_counter.hpp`; thêm `src/scan_frame.cpp` vào `core/common/CMakeLists.txt`.
- [ ] **Step 4: Chạy** → PASS. clang-format. **Step 5: Commit** `feat(common): shared data types, Span and reserve-once ScanFrame`

### Task 4: Cấu hình YAML với overlay (C-04)

**Files:**
- Create: `core/<module>/include/mss/<module>/config.hpp` cho `preprocess, map, scan_match, degeneracy, imu, gnss, odom, loop, backend`; `core/pipeline/include/mss/pipeline/config.hpp`, `core/pipeline/src/config.cpp`
- Modify: `core/pipeline/CMakeLists.txt` (STATIC, link `yaml-cpp` PRIVATE), `config/default.yaml` (thêm khoá theo spec mục 1), `config/sim/warehouse.yaml` (không đổi trừ khi cần), `README.md` (bảng tham số ngắn hoặc trỏ tới YAML)
- Test: `tests/unit/test_config.cpp`; chạy lại `python -m pytest tests/python/test_sim03_consistency.py`

**Interfaces:**
- Produces: struct trong namespace của module, ví dụ `mss::preprocess::PreprocessConfig{double min_range_m; double max_range_m; double downsample_voxel_m; bool deskew_enabled;}`; `mss::map::MapConfig{double voxel_m; int max_points_per_voxel; int knn_k; double prune_radius_m; double prune_period_s; std::size_t max_voxels;}`; `mss::scan_match::ScanMatchConfig{int max_iterations; double rot_tol_rad; double trans_tol_m; double max_corr_dist_m; double plane_rms_max_m; double robust_huber_delta_m; int min_inliers; double cov_scale;}`; `mss::degeneracy::DegeneracyConfig{bool enabled; double eig_threshold; double relax_factor;}`; `mss::imu::ImuConfig{double rate_hz; double acc_noise_density; double gyro_noise_density; double acc_random_walk; double gyro_random_walk; double gravity_mps2; double init_window_s; double init_max_acc_std_mps2; double init_max_gyro_std_radps;}`; `mss::gnss::GnssConfig{bool enabled; double sigma_h_m; double sigma_v_m;}`; `mss::odom::WheelOdomConfig{bool enabled; double wheel_radius_m; double track_m; double speed_sigma_mps;}`; `mss::loop::LoopConfig{bool enabled; double search_radius_m; double min_time_gap_s;}`; `mss::backend::BackendConfig{double keyframe_dist_m; double keyframe_rot_rad; double relinearize_threshold; int relinearize_skip;}`; `mss::pipeline::FramesConfig{std::string world, body, lidar, gnss; Pose T_b_l; Pose T_b_o; Vec3 p_b_g;}`; `mss::pipeline::RealtimeConfig{int scan_queue_depth; int imu_queue_depth; int aux_queue_depth; std::size_t max_points_per_scan; bool pin_cores; int priority;}`; `mss::pipeline::LidarConfig{double range_sigma_m;}`; `mss::pipeline::Config{bool deterministic; std::uint64_t random_seed; FramesConfig frames; ImuConfig imu; LidarConfig lidar; GnssConfig gnss; WheelOdomConfig wheel_odom; PreprocessConfig preprocess; MapConfig map; ScanMatchConfig scan_match; DegeneracyConfig degeneracy; LoopConfig loop_closure; BackendConfig backend; RealtimeConfig realtime;}`.
- `Result<Config> load_config(const std::filesystem::path& default_yaml, const std::optional<std::filesystem::path>& overlay = std::nullopt)`; `Result<Config> parse_config(const std::string& default_text, const std::optional<std::string>& overlay_text)` (để test không cần file). Khoá `_deg` trong YAML → trường `_rad`. Lỗi: một thông điệp gộp, mỗi dòng `<dotted.key>: <lý do>`; khoá lạ ở base hoặc overlay là lỗi; overlay kiểu khác base (scalar vs mapping) là lỗi; quaternion norm lệch > 1e-6 là lỗi; `rate_hz > 0`, σ ≥ 0, độ sâu hàng đợi > 0, `max_points_per_scan > 0`.

- [ ] **Step 1: Cập nhật `config/default.yaml`** theo spec mục 1 (giữ cấu trúc `mss_node.ros__parameters`, đơn vị trong tên, comment). Chạy `python -m pytest tests/python/test_sim03_consistency.py -q` → vẫn pass.
- [ ] **Step 2: Test thất bại** `test_config.cpp` dùng `parse_config`: `LoadsDefaultFile` (đọc `config/default.yaml` qua `load_config`, kiểm `backend.keyframe_rot_rad ≈ 10°` rad, `map.max_voxels == 500000`, `frames.T_b_l` identity); `OverlayWarehouse` (`default.yaml` + `config/sim/warehouse.yaml`: `wheel_odom.wheel_radius_m == 0.101`, `frames.T_b_l.p.x() == 0.10`, quaternion pitch 2° → `R` khớp `so3_exp([0, 0.0349, 0])` 1e-3); `MissingKeyNamed` (xoá `imu.rate_hz` → reason chứa `"imu.rate_hz"`); `WrongTypeNamed`; `UnknownKeyNamed` (`imu.foo`); `OverlayTypeMismatch` (overlay `imu: 5` → reason chứa `"imu"`); `UnnormalisedQuaternionRejected` (`[1, 0, 0.5, 0]` → `"frames.T_b_l.quaternion_wxyz"`); `NegativeSigmaRejected`; `AllErrorsReported` (hai lỗi → cả hai tên khoá trong reason).
- [ ] **Step 3: Chạy** `build.sh --no-gtsam --target test_config` → FAIL.
- [ ] **Step 4: Cài đặt** `config.cpp`: parse yaml-cpp thành cây, deep-merge overlay (đệ quy, cùng ngữ nghĩa `sim/mss_sim/config_overlay.py`), đọc từng trường qua helper `get<T>(node, "dotted.key", errors)` gom lỗi, kiểm tra khoảng, quaternion `(w,x,y,z)` → `Mat3` (ghi thứ tự ở nơi dựng). `pipeline` thành STATIC với `src/config.cpp`.
- [ ] **Step 5: Chạy** → PASS; `build.sh` đầy đủ xanh; pytest SIM-03 xanh. clang-format.
- [ ] **Step 6: Commit** `feat(pipeline): YAML config with overlay, validation and per-module config structs`

### Task 5: Header interface của 9 module

**Files:**
- Create: `core/preprocess/include/mss/preprocess/preprocessor.hpp`; `core/imu/include/mss/imu/{trajectory.hpp,preintegrator.hpp,initializer.hpp}`; `core/map/include/mss/map/voxel_map.hpp`; `core/scan_match/include/mss/scan_match/scan_matcher.hpp`; `core/degeneracy/include/mss/degeneracy/degeneracy.hpp`; `core/gnss/include/mss/gnss/gnss.hpp`; `core/odom/include/mss/odom/wheel_integrator.hpp`; `core/loop/include/mss/loop/loop_detector.hpp`; `core/backend/include/mss/backend/graph.hpp`; `core/pipeline/include/mss/pipeline/estimator.hpp` (chỉ khai báo ở task này; thân ở Task 8)
- Modify: các `core/<module>/CMakeLists.txt` (INTERFACE target thêm `target_include_directories(... INTERFACE $<BUILD_INTERFACE:...include> $<INSTALL_INTERFACE:...>)`)
- Test: `tests/unit/test_interfaces_compile.cpp` (include mọi header, `static_assert` vài kiểu, không gọi hàm)

**Interfaces (nguyên văn, là hợp đồng cho các worktree):**
- `mss::imu::ImuTrajectory`: `static constexpr std::size_t kMaxImuPerScan = 128;` `struct Sample{Timestamp t; Pose T_b0_b;}`; `std::array<Sample, kMaxImuPerScan> samples; std::size_t size;` `Timestamp t_start;` docstring: `b0` = body tại `t_start` của scan.
- `mss::preprocess::Preprocessor{ explicit Preprocessor(const PreprocessConfig&, std::size_t max_points); Status process(const ScanFrame& in, const imu::ImuTrajectory& traj, ScanFrame& out); }` — out ở khung `l` tại `t_start`; `out` phải có capacity ≥ `in.size()`.
- `mss::map::NeighborBuffer{ static constexpr int kMaxNeighbors = 32; std::array<Vec3, kMaxNeighbors> p_w; std::array<float, kMaxNeighbors> dist2; int size; }`; `VoxelMap{ explicit VoxelMap(const MapConfig&); void insert(const ScanFrame&, const Pose& T_w_l); int knn(const Vec3& p_w, int k, NeighborBuffer&) const; void prune(const Vec3& p_w_b); std::size_t num_voxels() const; std::size_t num_points() const; }`.
- `mss::scan_match`: `double point_to_plane_residual(const Vec3& p_l, const Vec3& q_w, const Vec3& n_w, const Pose& T_w_l, Vec6* J_r_xi)` (residual `r = n_w·(T_w_l p_l − q_w)`, `J_r_xi` theo nhiễu phải `[rho, phi]` của `T_w_l`); `ScanMatcher{ explicit ScanMatcher(const ScanMatchConfig&, std::size_t max_points); Result<MatchResult> align(const ScanFrame& scan_l, const map::VoxelMap&, const Pose& T_w_l_init); }`.
- `mss::degeneracy`: `DegeneracyReport analyze(const Mat6& H, const DegeneracyConfig&)`; `Vec6 project_update(const Vec6& delta, const DegeneracyReport&)`; `PoseCovariance relax_covariance(const PoseCovariance&, const DegeneracyReport&, const DegeneracyConfig&)`.
- `mss::imu`: `Preintegrator{ explicit Preintegrator(const ImuConfig&); void reset(const ImuBias&); Status integrate(const ImuSample&); const PreintegratedDelta& delta() const; NavState predict(const NavState& x0, const Vec3& g_w) const; }`; `struct InitState{Vec3 g_w; ImuBias bias; Mat3 R_w_b;}`; `ImuInitializer{ explicit ImuInitializer(const ImuConfig&); Result<InitState> try_init(Span<const ImuSample>); }`.
- `mss::gnss`: `struct Lla{double lat_deg; double lon_deg; double alt_m;}`; `Vec3 lla_to_enu(const Lla&, const Lla& origin)`; `Lla enu_to_lla(const Vec3&, const Lla& origin)`; `Status gate_fix(const GnssMeasurement&, double max_cov_m2)`; `GnssAligner{ explicit GnssAligner(const GnssConfig&); Result<Pose> update(const Vec3& p_e_g, const Pose& T_w_b, const Vec3& p_b_g); /* trả T_e_w khi đủ cặp */ }`.
- `mss::odom::WheelIntegrator{ WheelIntegrator(const WheelOdomConfig&, const Pose& T_b_o); Status integrate(const WheelSample&); OdomDelta take_delta(); }`.
- `mss::loop::LoopDetector{ explicit LoopDetector(const LoopConfig&); /* giai đoạn 2 */ }`.
- `mss::backend::Graph{ explicit Graph(const BackendConfig&); ~Graph(); KeyId add_keyframe(Timestamp, const NavState&); void add_imu_factor(KeyId i, KeyId j, const PreintegratedDelta&); void add_lidar_factor(KeyId i, KeyId j, const Pose& T_bi_bj, const PoseCovariance&, const DegeneracyReport*); void add_gnss_factor(KeyId, const GnssMeasurement&, const Pose& T_e_w); void add_odom_factor(KeyId i, KeyId j, const OdomDelta&); void add_loop_factor(const LoopConstraint&); Result<Estimate> optimize(); PoseCovariance marginal_cov(KeyId) const; private: struct Impl; std::unique_ptr<Impl> impl_; }` (pimpl; GTSAM chỉ trong `graph.cpp` sau này).
- `mss::pipeline::Estimator{ explicit Estimator(const Config&); Status push_scan(const ScanFrame&); Status push_imu(const ImuSample&); Status push_gnss(const GnssMeasurement&); Status push_odom(const WheelSample&); Estimate latest_state() const; void run_once(); void start(); void stop(); struct Stats{...}; Stats stats() const; }`.
Mỗi hàm có docstring: khung, đơn vị, điều kiện tiên quyết, hành vi khi lỗi (trả `Status`), và tham chiếu test ID của `docs/Test_CI_CD.md`.

- [ ] **Step 1: Viết test** `test_interfaces_compile.cpp` include tất cả header trên, `static_assert(!std::is_copy_constructible_v<mss::common::ScanFrame>)`, `static_assert(sizeof(mss::imu::ImuTrajectory) > 0)`; không link gì thêm.
- [ ] **Step 2: Chạy** → FAIL (header thiếu). **Step 3: Viết 11 header + CMake include dirs.** **Step 4: Chạy** `build.sh --no-gtsam` và đầy đủ → PASS, `-Werror` sạch (unused parameter trong khai báo thuần không có thân nên không cảnh báo). clang-format.
- [ ] **Step 5: Commit** `feat(core): module interface headers (contracts for parallel development)`
- Reviewer của task này được hỏi thêm một câu duy nhất: "Có chỗ nào hai người đọc có thể hiểu khác nhau không" — mọi điểm như vậy là finding Important.

### Task 6: Bộ chuyển đổi `bag_to_mssdata.py` và ADR-014

**Files:**
- Create: `tools/bag_to_mssdata.py`, `docs/adr/014-bench-dataset-format.md`, `docs/mssdata_format.md`
- Modify: `docs/adr/README.md` (dòng 014), `sim/README.md` (một đoạn)
- Test: `tests/python/test_bag_to_mssdata.py` (fixture `bridged_bag`)

**Interfaces:**
- Produces: `convert(bag_dir: Path, out_dir: Path, *, origin_lla: tuple | None = None) -> ConvertReport(n: dict[str, int], origin_lla)`; CLI `python tools/bag_to_mssdata.py <bag_dir> <out_dir> [--origin-lla LAT LON ALT]` (mặc định gốc = fix đầu tiên). Định dạng (little-endian, ghi trong `docs/mssdata_format.md`, phiên bản 1): `index.json{version:1, topics:{imu:{file,count}, gnss:{...}, odom:{...}, gt:{...}, lidar:{dir,count}}, origin_lla:[lat,lon,alt], T_b_l:{translation_m, quaternion_wxyz}, p_b_g_m, body_frame}`; `imu.bin` bản ghi 56 B `{t int64, acc 3×f64, gyro 3×f64}`; `gnss.bin` 105 B `{t int64, p_e_g 3×f64, cov 9×f64, status int8}`; `odom.bin` 24 B `{t int64, v f64, omega f64}`; `gt.bin` 64 B `{t int64, p 3×f64, q_wxyz 4×f64}`; `lidar/NNNNNN.bin` header 12 B `{t_start int64, n uint32}` + `n` × 22 B `{xyz 3×f32, intensity f32, t_offset u32, ring u16}` (điểm NaN/inf được ghi nguyên). Bản ghi sắp theo `t` tăng dần; `gt` quaternion Hamilton `(w,x,y,z)` từ Odometry `(x,y,z,w)` (ghi thứ tự ở nơi dựng).

- [ ] **Step 1: Viết `docs/mssdata_format.md` và ADR-014** (bối cảnh: Pi5 không ROS, thư viện MCAP C++ + CDR tự viết; quyết định: chuyển đổi một lần bằng Python, C++ đọc nhị phân phẳng; hệ quả: hai định dạng, bag vẫn là nguồn gốc).
- [ ] **Step 2: Test thất bại**: `convert` trên `bridged_bag` → `index.json` đúng khoá; số bản ghi `imu.bin` = `n_msgs["/imu/data"]`; `gt.bin` 4001 bản ghi (6 s + 1); `lidar/` có 60 file, file đầu `n == 16*720`, `t_offset` tăng theo cột tại hàng 0; `gnss.bin` ENU: điểm đầu gần `[0,0,0]` (gốc = fix đầu) hoặc đúng offset khi truyền `--origin-lla`; bản ghi `t` tăng chặt; `struct.calcsize` khớp kích thước khai báo.
- [ ] **Step 3: Chạy** `python -m pytest tests/python/test_bag_to_mssdata.py -q -W error` → FAIL. **Step 4: Cài đặt** (dùng `mss_sim.bag_io`, `mss_sim.geodesy`, `numpy.tofile` với dtype cấu trúc `np.dtype([...], align=False)`). **Step 5: Chạy** → PASS; ruff sạch. **Step 6: Commit** `feat(tools): bag_to_mssdata converter and ADR-014 dataset format`

### Task 7: Module `io`: `DatasetReader`/`DatasetWriter` (C++)

**Files:**
- Create: `core/io/CMakeLists.txt`, `core/io/include/mss/io/{dataset_format.hpp,dataset_reader.hpp,dataset_writer.hpp}`, `core/io/src/{dataset_reader.cpp,dataset_writer.cpp}`
- Modify: `core/CMakeLists.txt` (`MSS_MODULES` thêm `io`), `core/pipeline/CMakeLists.txt` (không link io — io là app-level), `bench/CMakeLists.txt` (link `mss::io`), `tests/CMakeLists.txt` (`mss_add_test` link thêm `mss::io`)
- Test: `tests/unit/test_dataset_io.cpp`

**Interfaces:**
- Produces (`mss::io`): `dataset_format.hpp` với các `struct` packed (`#pragma pack(push,1)`) `ImuRecord`, `GnssRecord`, `OdomRecord`, `GtRecord`, `LidarHeader`, `LidarPoint` đúng kích thước Task 6 (`static_assert(sizeof(GnssRecord) == 105)` v.v.) và `constexpr int kFormatVersion = 1`; `struct DatasetIndex{ int version; std::size_t n_imu, n_gnss, n_odom, n_gt, n_lidar; gnss::Lla origin_lla; Pose T_b_l; Vec3 p_b_g; std::string body_frame; }` (parse JSON tối thiểu bằng tay hoặc yaml-cpp — JSON là YAML hợp lệ; dùng yaml-cpp, link PRIVATE); `enum class RecordKind{kImu, kGnss, kOdom, kLidar, kGt}`; `struct Record{ RecordKind kind; Timestamp t; ImuSample imu; GnssMeasurement gnss; WheelSample odom; NavState gt; /* lidar nạp vào frame ngoài */ }`; `DatasetReader{ static Result<DatasetReader> open(const std::filesystem::path& dir, std::size_t max_points_per_scan); const DatasetIndex& index() const; Result<bool> next(Record& out, ScanFrame& scan); /* gộp 5 luồng theo t tăng dần; với kLidar nạp vào scan; false khi hết */ }`; `DatasetWriter{ static Result<DatasetWriter> create(const std::filesystem::path& dir, const DatasetIndex& meta); Status write_imu(const ImuSample&); write_gnss; write_odom; write_gt(const NavState&); Status write_lidar(const ScanFrame&); Status finish(); /* ghi index.json */ }`.
- Lỗi: file thiếu, kích thước không chia hết cho bản ghi, `lidar` header `n` vượt `max_points_per_scan` hoặc file ngắn hơn `12 + n*22` → `Status` lỗi nêu tên file; không đọc quá bộ đệm.

- [ ] **Step 1: Test thất bại** `test_dataset_io.cpp`: `WriteThenReadRoundTrip` (writer ghi 10 imu, 2 gnss, 3 odom, 4 gt, 2 lidar 100 điểm xen kẽ thời gian; reader trả 21 bản ghi theo `t` tăng dần, giá trị khớp bit, `scan.size()==100`, `t_offset`/`ring` khớp); `RecordSizesMatchFormat` (static_assert); `TruncatedLidarFileIsStatusError` (cắt file lidar thứ 2 còn 50 B → `next` trả lỗi có tên file, không crash); `TooManyPointsRejected` (`max_points_per_scan = 50` → lỗi); `MissingIndexIsError`.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** (fstream nhị phân, `fread` theo bản ghi, buffer một bản ghi mỗi luồng cho k-way merge; lidar đọc thẳng vào mảng SoA của `ScanFrame` qua `xyz_mut()` v.v. sau khi đọc AoS vào một buffer tạm cấp phát một lần trong `open` theo `max_points_per_scan`).
- [ ] **Step 4: Chạy** → PASS; thêm test chéo: trong `tests/python/test_bag_to_mssdata.py` thêm `test_cpp_reader_agrees` chỉ chạy khi env `MSS_BENCH_BIN` trỏ tới `mss_bench` (bỏ qua nếu không) — Task 8 dùng. clang-format. **Step 5: Commit** `feat(io): mssdata dataset reader and writer`

### Task 8: `Estimator` khung và `bench` phát lại

**Files:**
- Create: `core/pipeline/src/estimator.cpp`, `core/pipeline/include/mss/pipeline/bounded_queue.hpp`
- Modify: `core/pipeline/CMakeLists.txt`, `bench/main.cpp`, `bench/CMakeLists.txt`, `core/pipeline/include/mss/pipeline/estimator.hpp` (nếu cần sửa khai báo từ Task 5, ghi rõ trong report)
- Test: `tests/unit/test_estimator.cpp`, `tests/component/CMakeLists.txt`, `tests/component/test_bench_replay.cpp`, `tests/CMakeLists.txt` (`add_subdirectory(component)`)

**Interfaces:**
- Produces: `bounded_queue.hpp`: `template <class T, std::size_t N> class BoundedQueue{ bool push(const T&); /* false + ghi đè cũ nhất khi đầy, trả false */ bool pop(T&); std::size_t size() const; }` (vòng tròn, cấp phát tĩnh, không mutex ở giai đoạn này — đơn luồng; ghi rõ TODO ADR-005 cho luồng).
- `Estimator` (đơn luồng, `deterministic` bắt buộc true ở giai đoạn này, `start()/stop()` trả về ngay và log cảnh báo nếu `deterministic == false`): pool `realtime.scan_queue_depth + 2` `ScanFrame` reserve `max_points_per_scan` trong constructor; `push_scan` copy_from vào slot trống, hàng đợi đầy → bỏ cũ nhất, trả `Status::error("scan queue full, dropped oldest")`; `push_imu` từ chối timestamp ≤ mẫu trước (`Status::error("imu timestamp not increasing")`); `push_gnss/odom` vào hàng đợi `aux_queue_depth`; `run_once()` rút hết hàng đợi, đếm vào `Stats{uint64 n_scans, n_imu, n_gnss, n_odom, n_dropped_scans, n_rejected_imu; double push_scan_us_p50, p95;}` (p50/p95 từ reservoir cố định 4096 mẫu `std::chrono::steady_clock`), cập nhật `latest_state()` = `NavState` với `t` = stamp scan cuối, pose identity (chưa có thuật toán; ghi rõ docstring).
- `bench/main.cpp`: `mss_bench <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR`: `load_config`, `DatasetReader::open`, vòng `next` → `push_*`, `run_once()` theo chính sách watermark: watermark = thời điểm kết thúc scan (`t_start + max t_offset`); rút hàng đợi khi bản ghi kế tiếp vượt watermark, rút trước khi đẩy scan nếu chưa có scan nào đang chờ, và rút khi hàng đợi phụ (GNSS/odom) hoặc hàng đợi scan sắp đầy; cuối: ghi `<out>/est.tum` (một dòng mỗi scan: `t x y z qx qy qz qw` từ `latest_state()`, quaternion TUM `(x,y,z,w)` ghi thứ tự), `<out>/bench.json` (Stats + thời gian phát lại), in bảng ra stdout; exit 0; lỗi cấu hình, dataset hoặc ghi đầu ra → in reason, exit 2; chạy xong nhưng còn bản ghi không được xử lý (bị bỏ hoặc từ chối) → cảnh báo, vẫn ghi đầu ra, exit 3.

- [ ] **Step 1: Test thất bại** `test_estimator.cpp`: `PushScanCopiesWithoutAlloc` (AllocCounter sau constructor = 0 khi push 5 scan); `ScanQueueFullDropsOldest` (depth 2: push 3 → lần 3 trả error reason chứa "dropped", `stats().n_dropped_scans == 1`, `run_once` xử lý 2); `ImuNonIncreasingRejected` (t 10, 10 → thứ 2 lỗi, `n_rejected_imu == 1`); `RunOnceDrainsQueues`; `LatestStateTimestampFollowsScans`. `tests/component/test_bench_replay.cpp`: dùng `DatasetWriter` tạo dataset nhỏ (1 s: 200 imu, 5 gnss, 50 odom, 10 lidar 1000 điểm) trong thư mục tạm, chạy `mss_bench` bằng `std::system` (đường dẫn qua macro `MSS_BENCH_PATH` do CMake truyền `$<TARGET_FILE:mss_bench>`) với `config/default.yaml` → exit 0, `est.tum` có 10 dòng, `bench.json` chứa `"n_imu": 200`.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** `bounded_queue.hpp`, `estimator.cpp`, `bench/main.cpp`. **Step 4: Chạy** `build.sh --no-gtsam` và đầy đủ → PASS; chạy tay: `python tools/bag_to_mssdata.py results/demo/E-01/bag results/demo/E-01/mssdata` rồi trong WSL `mss_bench config/default.yaml results/demo/E-01/mssdata --overlay config/sim/warehouse.yaml --out results/demo/bench` → in số bản ghi (200 lidar, 4000 imu, 100 gnss, 999 odom); ghi số vào report. clang-format.
- [ ] **Step 5: Commit** `feat(pipeline,bench): Estimator skeleton with bounded queues and dataset replay`

### Task 9: Tài liệu, trang kiến trúc, dọn

**Files:**
- Modify: `docs/adr/011-simulation.md` (§3 LiDAR: deskew về `t_start`), `docs/architecture.html` (khối `io` mới, `common` và `pipeline` cập nhật hàm thật + test, `bench` short "replay mssdata"), `CLAUDE.md` (mục "Notes for AI assistants": build/test C++ bằng `tools/wsl/build.sh`, không có toolchain trên Windows), `README.md` (bench usage), `docs/adr/README.md` (014)

- [ ] **Step 1:** Sửa ADR-011 §3 dòng "Méo chuyển động" / ScanFrame: thời điểm tham chiếu là `t_start` (= `header.stamp`).
- [ ] **Step 2:** `architecture.html` `DATA`: `common` thêm fns thật (`so3_exp`, `se3_log`, `boxplus`, `transform_covariance`, `ScanFrame::push`), tests C-01/C-02 + scan_frame; `pipeline` status `stub` với `load_config`, `Estimator::push_scan`, `run_once` thật; khối `io` (`DatasetReader::next`, `DatasetWriter`) status `done`; `bench` fns thật; `repo.commit` = HEAD trước commit này. Kiểm tra JS syntax bằng node và chụp Edge headless.
- [ ] **Step 3:** CLAUDE.md và README cập nhật; `python -m ruff check . && python -m ruff format --check . && python -m pytest -q -W error`; `build.sh` cả hai chế độ xanh; `git ls-files '*.cpp' '*.hpp' | xargs python -m clang-format --dry-run --Werror` sạch.
- [ ] **Step 4: Commit** `docs: core foundation status, mssdata, WSL build notes`; push nhánh và xác nhận CI xanh (`gh run watch`).

---

## Tự soát

- Phủ spec: Mục tiêu 1 → Task 1; 2 → Task 2; 3 → Task 4; 4 → Task 3 + 5; 5 → Task 6 + 7 + 8; 6 → Task 6 (ADR-014) + Task 9 (ADR-011 sửa). Quyết định bổ sung 1 → T4; 2 → T6/T7; 3 → T3/T8; 4 → T5; 5 → T3; 6 → T2/T3/T5; 7 → T3/T5; 8 → T4/T5; 9 → T1.
- Kiểu nhất quán: `ScanFrame` API (T3) dùng ở T5/T7/T8; `Status/Result` từ `result.hpp` hiện có; `Lla` khai báo ở T5 (`gnss.hpp`) và dùng ở T7 (`DatasetIndex`) — `io` do đó phụ thuộc header `gnss` (INTERFACE, không có source) → ghi `target_link_libraries(mss_io PUBLIC mss::common mss::gnss)`; chấp nhận vì `gnss` chưa có thân hàm.
- Review Focus 1, 3 → T4; 2 → T3; 4 → T7; 5 → T8.
- Tỷ lệ: kế hoạch dài hơn spec nhưng phần lớn là chữ ký interface, là chính nội dung hợp đồng cần cố định.
