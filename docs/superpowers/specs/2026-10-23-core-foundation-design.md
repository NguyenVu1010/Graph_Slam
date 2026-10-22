# Giai đoạn 0: nền chung cho các module C++ (thiết kế)

- Ngày: 2026-10-23
- Trạng thái: chờ duyệt
- Mục đích: sau giai đoạn này, 5–6 module (`preprocess`, `map`, `imu`, `gnss`, `odom`, `degeneracy`) có thể được phát triển **song song trên worktree riêng** mà không đụng nhau: mọi kiểu dữ liệu, chữ ký hàm, quy ước toán và cách đọc cấu hình đã cố định; mỗi module chỉ còn việc điền thân hàm và viết test theo `docs/Test_CI_CD.md`.

## Vì sao cần giai đoạn này

Hiện `core/` chỉ có `common` (Timestamp, Result, Pose, logger). Mọi module cần: phép toán Lie group với Jacobian đúng quy ước (ADR-006), struct cấu hình đọc từ YAML (ADR-008), các kiểu dữ liệu trao đổi giữa module (ScanFrame, MatchResult, PreintegratedDelta...), và `bench` cần đọc được bag mô phỏng. Nếu 6 agent cùng tự định nghĩa những thứ này sẽ sinh 6 quy ước và merge conflict ở `common`. Máy dev Windows không build được C++; WSL Ubuntu 24.04 build được (đã kiểm tra: skeleton + 7 GTest pass).

## Mục tiêu / không mục tiêu

**Mục tiêu**
1. Môi trường build trong WSL dùng được cho agent: GTSAM 4.2.0, Sophus 1.22.10 trong `~/opt`; lệnh một dòng build + test; clone ext4 `~/graph_slam` cho worktree.
2. `mss::common` có `lie.hpp`: SO(3)/SE(3) Exp/Log, Jacobian phải/trái, adjoint, chuyển đổi `Pose` ↔ `Sophus::SE3d`, tất cả theo nhiễu phải `[ρ, φ]`. Test C-01, C-02.
3. Cấu hình: `mss::pipeline::Config` (struct lồng theo module) đọc từ `config/default.yaml` cộng một overlay tuỳ chọn (`config/sim/<world>.yaml`), deep-merge với cùng ngữ nghĩa như `sim/mss_sim/config_overlay.py` (khoá lạ là lỗi), kiểm tra khoảng giá trị, thông báo lỗi nêu tên khoá đầy đủ. Test C-04.
4. Kiểu dữ liệu chung và **header interface của từng module** (khai báo lớp, chữ ký hàm, docstring nêu residual/khung/đơn vị), chưa có thân hàm. Đây là "hợp đồng" để các worktree làm song song.
5. `bench` đọc được dữ liệu mô phỏng: bộ chuyển đổi Python `tools/bag_to_mssdata.py` (bag MCAP → thư mục dữ liệu nhị phân phẳng) và bộ đọc C++ `mss::io::DatasetReader` không phụ thuộc thư viện ngoài. `bench` phát lại theo timestamp vào `Estimator` (chưa có thuật toán, chỉ đếm và đo).
6. ADR-014 ghi quyết định mục 5 và việc ghim Sophus.

**Không mục tiêu**: bất kỳ thuật toán nào (deskew, kNN, preintegration, GN, iSAM2); `ros2_node` subscription; Gazebo.

**Phản mục tiêu** (thất bại dù code chạy): một quy ước Jacobian khác với `docs/conventions.md`; một kiểu dữ liệu cấp phát heap trên đường nóng; GTSAM lọt ra ngoài `backend`; header interface mơ hồ đến mức hai agent hiểu khác nhau.

## Ràng buộc

- CLAUDE.md và ADR-003/005/006/008/013: C++17, `-Wall -Wextra -Wpedantic -Werror`, không `auto` cho Eigen, kiểu cố định, `Result`/`Status` thay exception trên đường nóng, không log/alloc/exception theo điểm, chỉ `backend` include GTSAM, chỉ `pipeline` parse YAML.
- Dependency mới: Sophus (ADR-006, ghim commit), yaml-cpp vào `pipeline` (ADR-013 đã cho phép). **Không** thêm thư viện MCAP C++ (xem Quyết định 5).
- RAM WSL 10 GB: `--parallel 3`.

## Quyết định (phần "làm thế nào" đã chốt)

### 1. Môi trường build
- `tools/wsl/setup.sh`: cài apt (ninja, clang-format-18, clang-tidy-18, ccache; cần sudo, người dùng chạy), build Sophus + GTSAM vào `~/opt` (không sudo), clone `~/graph_slam` từ GitHub.
- `tools/wsl/build.sh [--no-gtsam] [--debug-asan]`: cmake + build + ctest với `CMAKE_PREFIX_PATH=$HOME/opt/gtsam;$HOME/opt/sophus`, ccache, `-DMSS_WERROR=ON`. Đây là lệnh agent dùng trong mọi worktree.
- `docker/deps.Dockerfile` thêm Sophus cùng commit (CI phải khớp WSL).

### 2. `mss/common/lie.hpp` (bọc Sophus)
Hàm tự do trong `mss::common`, mọi vector tangent là `Vec3`/`Vec6` thứ tự `[ρ, φ]`:
- `Mat3 so3_exp(const Vec3& phi)`, `Vec3 so3_log(const Mat3& R)`
- `Pose se3_exp(const Vec6& xi)`, `Vec6 se3_log(const Pose& T)`
- `Mat3 so3_jr(const Vec3& phi)`, `Mat3 so3_jr_inv(const Vec3& phi)`, `Mat3 so3_jl(...)`, `Mat3 so3_jl_inv(...)`
- `Mat6 se3_adj(const Pose& T)`, `Mat3 skew(const Vec3& v)`
- `Pose boxplus(const Pose& T, const Vec6& delta)` = `T * se3_exp(delta)`; `Vec6 boxminus(const Pose& a, const Pose& b)` = `se3_log(b⁻¹ a)`
- Chuyển đổi: `Sophus::SE3d to_sophus(const Pose&)`, `Pose from_sophus(const Sophus::SE3d&)`.
Module khác **chỉ** gọi lớp bọc này, không include Sophus trực tiếp (clang-tidy không ép được, review ép).
Test C-01: 10 000 vector ngẫu nhiên seed cố định, gồm góc < 1e-10 và > π − 1e-6, `‖Log(Exp(ξ)) − ξ‖ < 1e-9`. C-02: `so3_jr` so với sai phân số của `so3_log(so3_exp(φ)⁻¹ so3_exp(φ + δ))`, sai số tương đối < 1e-6; thêm test đối chiếu `se3_log` với `gtsam::Pose3::Logmap` trong `tests/unit/backend/` (chỉ build khi `MSS_WITH_GTSAM`), để khoá thứ tự `[ρ, φ]`.

### 3. Cấu hình (`mss/pipeline/config.hpp`, `config.cpp`)
- Struct: `Config{ bool deterministic; uint64 random_seed; FramesConfig frames; ImuConfig imu; LidarConfig lidar; GnssConfig gnss; WheelOdomConfig wheel_odom; PreprocessConfig preprocess; MapConfig map; ScanMatchConfig scan_match; DegeneracyConfig degeneracy; LoopConfig loop_closure; BackendConfig backend; RealtimeConfig realtime; }`. Mỗi struct ở header của module tương ứng (`mss/imu/config.hpp`...), `pipeline` gom lại.
- `Result<Config> load_config(const std::filesystem::path& default_yaml, std::optional<path> overlay)`: đọc `mss_node.ros__parameters`, deep-merge overlay (khoá lạ → lỗi `imu.foo: unknown key`), kiểm tra: thiếu khoá, sai kiểu, khoảng giá trị (tần số > 0, σ ≥ 0, quaternion chuẩn hoá trong 1e-6, `_deg` → radian khi nạp). Lỗi gộp tất cả khoá sai vào một thông điệp.
- `config/default.yaml` bổ sung các khoá cho `preprocess`, `map`, `scan_match`, `degeneracy`, `backend`, `realtime` với giá trị mặc định từ ADR-004/005/007 (keyframe 1 m/10°, relinearize 0.1, voxel 0.5 m, 20 điểm/voxel, k = 5, prune 100 m, hàng đợi scan 2 / IMU 400 / khác 32, `max_points_per_scan` 65 536). SIM-03 (Python) vẫn phải pass.
- Test C-04: thiếu trường, sai kiểu, quaternion không chuẩn, overlay khoá lạ → thông điệp nêu tên khoá; file chuẩn + overlay `warehouse.yaml` nạp đúng.

### 4. Kiểu dữ liệu chung và header interface
`mss/common/types.hpp` thêm (tất cả POD/Eigen cố định, không `std::vector` trong kiểu trao đổi trên đường nóng trừ nơi ghi rõ):
- `struct ImuSample{Timestamp t; Vec3 acc_b; Vec3 gyro_b;}` (sửa kiểu hiện có: thêm `t`).
- `struct ImuBias{Vec3 bg; Vec3 ba;}`, `struct NavState{Pose T_w_b; Vec3 v_w_b; ImuBias bias; Timestamp t;}`.
- `struct Estimate{NavState state; PoseCovariance cov; uint64_t keyframe_id;}`.
- `class ScanFrame` (trong `mss/common/scan_frame.hpp`): bộ đệm cố định `reserve(max_points)` một lần; mảng SoA `xyz` (float, 3N), `t_offset_ns` (uint32), `ring` (uint16), `intensity` (float); `Timestamp t_start`; `size()`, `capacity()`, `clear()`, `push(...)` trả `bool` (false khi đầy, không ném). Không copy ngầm (xoá copy ctor, cho phép move).
- `struct MatchResult{Pose T_w_l; PoseCovariance cov; Mat6 H; int inliers; int iterations; bool converged;}`.
- `struct DegeneracyReport{Vec6 eigvals; Mat6 eigvecs; std::array<bool,6> degenerate; }`.
- `struct PreintegratedDelta{Mat3 dR; Vec3 dv; Vec3 dp; double dt_s; Eigen::Matrix<double,9,9> cov; Mat3 J_dR_bg, J_dv_ba, J_dv_bg, J_dp_ba, J_dp_bg; ImuBias bias_lin;}`.
- `struct GnssMeasurement{Timestamp t; Vec3 p_e_g; Mat3 cov; }`, `struct WheelSample{Timestamp t; double v_mps; double omega_radps;}`, `struct OdomDelta{Timestamp t0, t1; Pose T_b0_b1; PoseCovariance cov;}`, `struct LoopConstraint{uint64_t i, j; Pose T_bi_bj; PoseCovariance cov;}`.

Header interface mỗi module (chỉ khai báo + docstring nêu residual, khung, đơn vị, điều kiện tiên quyết; chữ ký lấy từ `docs/architecture.html` khối "dự kiến", được rà lại ở đây):
- `mss/preprocess/preprocessor.hpp`: `class Preprocessor{ explicit Preprocessor(const PreprocessConfig&); Status process(const ScanFrame& in, const ImuTrajectory& traj, ScanFrame& out); }` với `ImuTrajectory` = danh sách `(Timestamp, Pose)` đã tích phân trong scan (`mss/imu/trajectory.hpp`, do `imu` cung cấp).
- `mss/map/voxel_map.hpp`: `class VoxelMap{ explicit VoxelMap(const MapConfig&); void insert(const ScanFrame&, const Pose& T_w_l); int knn(const Vec3& p_w, int k, NeighborBuffer&) const; void prune(const Vec3& p_w_b); size_t num_voxels() const; }`, `struct NeighborBuffer` cố định `kMaxNeighbors = 32`.
- `mss/scan_match/scan_matcher.hpp`: `class ScanMatcher{ explicit ScanMatcher(const ScanMatchConfig&); Result<MatchResult> align(const ScanFrame&, const VoxelMap&, const Pose& T_w_l_init); }` + hàm tự do `double point_to_plane_residual(const Vec3& p_l, const Vec3& q_w, const Vec3& n_w, const Pose& T_w_l, Vec6* J_r_xi)`.
- `mss/degeneracy/degeneracy.hpp`: `DegeneracyReport analyze(const Mat6& H, const DegeneracyConfig&)`, `Vec6 project_update(const Vec6& delta, const DegeneracyReport&)`.
- `mss/imu/preintegrator.hpp`: `class Preintegrator{ explicit Preintegrator(const ImuConfig&); void reset(const ImuBias&); Status integrate(const ImuSample&); const PreintegratedDelta& delta() const; NavState predict(const NavState& x0, const Vec3& g_w) const; }`; `mss/imu/initializer.hpp`: `class ImuInitializer{ Result<InitState> try_init(span<const ImuSample>); }` với `InitState{Vec3 g_w; ImuBias bias; Mat3 R_w_b;}`.
- `mss/gnss/gnss.hpp`: `Vec3 lla_to_enu(const Lla&, const Lla& origin)`, `Lla enu_to_lla(...)`, `class GnssAligner{ Result<Pose> update(const Vec3& p_e_g, const Pose& T_w_b, const Vec3& p_b_g); }`, `Status gate_fix(const GnssMeasurement&, double max_cov_m2)`.
- `mss/odom/wheel_integrator.hpp`: `class WheelIntegrator{ Status integrate(const WheelSample&); OdomDelta take_delta(); }`.
- `mss/loop/loop_detector.hpp`: khai báo tối thiểu (giai đoạn 2).
- `mss/backend/graph.hpp`: `class Graph{ KeyId add_keyframe(Timestamp, const NavState&); void add_imu_factor(KeyId i, KeyId j, const PreintegratedDelta&); void add_lidar_factor(KeyId i, KeyId j, const Pose& T_bi_bj, const PoseCovariance&, const DegeneracyReport*); void add_gnss_factor(KeyId, const GnssMeasurement&, const Pose& T_e_w); void add_odom_factor(KeyId i, KeyId j, const OdomDelta&); void add_loop_factor(const LoopConstraint&); Result<Estimate> optimize(); PoseCovariance marginal_cov(KeyId) const; }` — GTSAM chỉ trong `graph.cpp` qua pimpl.
- `mss/pipeline/estimator.hpp`: `class Estimator{ explicit Estimator(const Config&); Status push_scan(ScanFrame&&); Status push_imu(const ImuSample&); Status push_gnss(const GnssMeasurement&); Status push_odom(const WheelSample&); Estimate latest_state() const; void run_once(); /* deterministic */ void start(); void stop(); }`.

Header interface được review bởi một agent độc lập với câu hỏi duy nhất: "có chỗ nào hai người đọc có thể hiểu khác nhau không".

### 5. Dữ liệu cho `bench` (ADR-014)
Không thêm thư viện MCAP C++. `tools/bag_to_mssdata.py` (Python, dùng `mss_sim.bag_io`) chuyển bag MCAP thành thư mục `mssdata/`: `index.json` (phiên bản, topic, số bản ghi, extrinsics `T_b_l`, `p_b_g`), và mỗi topic một file nhị phân little-endian bản ghi cố định: `imu.bin` (t int64, acc 3×f64, gyro 3×f64), `gnss.bin` (t, lat, lon, alt, cov 9×f64), `odom.bin` (t, v, ω), `gt.bin` (t, p 3, q_wxyz 4), `lidar/<seq>.bin` (header: t_start int64, n uint32; rồi N × {xyz 3×f32, intensity f32, t_offset u32, ring u16}). C++ `mss::io::DatasetReader` (`core/io/`, module mới, chỉ phụ thuộc `common`) đọc tuần tự, gộp theo timestamp, nạp mỗi scan vào `ScanFrame` không cấp phát. Lý do: Pi5 không có ROS; thư viện mcap C++ cần lz4/zstd và bộ giải mã CDR tự viết cho 6 kiểu message; bộ chuyển đổi Python chạy một lần ngoài đường nóng. Nhược điểm: hai định dạng dữ liệu; chấp nhận vì bag vẫn là nguồn gốc.
- `bench` (`bench/main.cpp`): `mss_bench <config.yaml> <mssdata-dir> [--overlay x.yaml] [--out results/<run-id>]` phát lại theo thứ tự timestamp vào `Estimator`, ghi `est.tum` (hiện là pose ground-truth passthrough hoặc identity vì chưa có thuật toán), in p50/p95 mỗi stage từ `Estimator` (stage hiện rỗng). Mục đích: khung đo đã sẵn để các module cắm vào.

### 6. Tổ chức worktree sau giai đoạn 0
- Clone ext4 `~/graph_slam`, worktree `~/graph_slam/.worktrees/feat-<module>` từ `main` sau khi PR giai đoạn 0 merge.
- Mỗi worktree: một kế hoạch `docs/superpowers/plans/<date>-<module>.md` sinh từ header interface + bảng test của module trong `docs/Test_CI_CD.md`; một agent điều phối (SDD) riêng; PR riêng, CI riêng.
- Nhóm song song 1: `preprocess`, `map`, `imu`, `gnss`, `odom`, `degeneracy`. Nhóm 2: `scan_match`, `backend`. Tuần tự cuối: `pipeline`, `bench` hoàn chỉnh, `ros2_node`, harness E-01..E-03.

## Quyết định bổ sung sau builder check (2026-10-23, đều là lựa chọn của người viết spec)

1. **Khoá YAML và trường config của từng module** (giá trị khởi đầu, đơn vị trong tên):
   - `preprocess`: `min_range_m 0.5`, `max_range_m 100.0`, `downsample_voxel_m 0.5`, `deskew_enabled true`.
   - `map`: thêm `max_voxels 500000` (ADR-007), giữ `voxel_m 0.5`, `max_points_per_voxel 20`, `knn_k 5`, `prune_radius_m 100.0`, `prune_period_s 5.0`.
   - `scan_match`: `max_iterations 10`, `rot_tol_rad 1e-4`, `trans_tol_m 1e-3`, `max_corr_dist_m 1.0`, `plane_rms_max_m 0.05`, `robust_huber_delta_m 0.1`, `min_inliers 100`, `cov_scale 1.0`.
   - `degeneracy`: `enabled false`, `eig_threshold 100.0` (trị riêng tuyệt đối của `H`), `relax_factor 10.0` (hệ số nới covariance theo hướng suy biến, D-03).
   - `imu`: thêm `init_window_s 2.0`, `init_max_acc_std_mps2 0.05`, `init_max_gyro_std_radps 0.01`.
   - `backend`: `keyframe_dist_m 1.0`, `keyframe_rot_deg 10.0`, `relinearize_threshold 0.1`, `relinearize_skip 1`.
   - `loop_closure`: `enabled false`, `search_radius_m 10.0`, `min_time_gap_s 30.0`.
   - `realtime`: `scan_queue_depth 2`, `imu_queue_depth 400`, `aux_queue_depth 32`, `max_points_per_scan 65536`, `pin_cores false`, `priority 0`.
   - `frames`: thêm `T_b_o` (khung odom bánh xe `o` trong body, mặc định đồng nhất), cùng cấu trúc `translation_m` + `quaternion_wxyz`.
2. **GNSS cho bench**: bộ chuyển đổi Python đổi sang ENU sẵn (Python đã có geodesy); `gnss.bin` = `{t int64, p_e_g 3×f64, cov 9×f64, status int8}`; `index.json` ghi `origin_lla` (độ). `GnssMeasurement` thêm `int8_t status`. `Lla{double lat_deg, lon_deg, alt_m}` trong `mss/gnss/gnss.hpp`. `io` không phụ thuộc `gnss`.
3. **Sở hữu `ScanFrame`**: `Estimator` giữ một pool cố định `scan_queue_depth + 2` khung, `reserve` một lần theo `max_points_per_scan`. `Status push_scan(const ScanFrame&)` sao chép (memcpy mảng SoA, ~250 KB/scan, không cấp phát) vào khung trống; hàng đợi đầy → bỏ scan cũ nhất và cảnh báo (ADR-005). Người gọi (`DatasetReader`, `ros2_node`) giữ bộ đệm riêng của mình, cũng reserve một lần.
4. **`ImuTrajectory`**: mảng cố định `kMaxImuPerScan = 128` phần tử `{Timestamp t; Pose T_b0_b;}` với `b0` = body tại `t_start` của scan (pose tương đối, không phải `T_w_b`). Deskew đưa mọi điểm về khung `l` tại **`t_start`** (= `header.stamp`, điểm có `t_offset = 0` không đổi, khớp cách bridge tiêm skew). ADR-011 §3 ghi "thời điểm cuối scan" là sai, sẽ sửa cùng PR này.
5. **`PreintegratedDelta::cov`** 9×9 theo thứ tự GTSAM `[φ (rot), p, v]`; các Jacobian theo bias đặt tên theo khối tương ứng. Ghi rõ trong docstring; I-03 so trực tiếp với GTSAM không cần hoán vị.
6. **Scan matching / suy biến**: `MatchResult::H` là Hessian theo nhiễu phải của `T_w_l`; `DegeneracyReport::eigvals` tăng dần, `eigvecs` cột tương ứng; `project_update` nhận `delta` theo cùng tham số hoá. `pipeline` đổi sang khung `b` bằng `T_b_l` và xoay covariance/hướng suy biến qua adjoint; thêm `PoseCovariance transform_covariance(const PoseCovariance& cov_x, const Pose& T_y_x)` vào `lie.hpp` cho việc này.
7. **Kiểu còn thiếu**: `using KeyId = std::uint64_t;` `GnssAligner::update` trả `Result<Pose>` là `T_e_w`; `ImuInitializer(const ImuConfig&)`, `WheelIntegrator(const WheelOdomConfig&, const Pose& T_b_o)`; `mss::common::Span<T>` tối thiểu (con trỏ + kích thước, `begin/end/size/operator[]`) trong `mss/common/span.hpp` vì C++17 không có `std::span`.
8. **Khung odom bánh xe**: `OdomDelta::T_b0_b1` ở khung `b`; `WheelIntegrator` nhận `T_b_o` và đổi khung trước khi trả. Mặc định `T_b_o` = đồng nhất (rig: `base_link` ≡ `imu_link`).
9. **Sophus trong CI mỏng**: job `core-no-gtsam` build Sophus 1.22.10 từ source (cache bằng `actions/cache` theo tag) với `-DSOPHUS_USE_BASIC_LOGGING=ON`; `mss_common` thêm `target_compile_definitions(PUBLIC SOPHUS_USE_BASIC_LOGGING)` để không kéo `fmt`. Cùng tag và cờ trong `docker/deps.Dockerfile` và `tools/wsl/setup.sh`.

## Để lại cho builder
- Cách chia file test (`tests/unit/common/test_lie.cpp`, `tests/unit/pipeline/test_config.cpp`, ...).
- Chi tiết `ScanFrame` SoA vs AoS, miễn giữ reserve-một-lần và không copy ngầm.
- Thứ tự kiểm tra trong `load_config` và cách gộp thông điệp lỗi.
- Bố cục `index.json`.

## Tiêu chí hoàn thành
- `tools/wsl/build.sh` chạy xanh trong WSL (có và không GTSAM), CI xanh (deps image có Sophus).
- C-01, C-02, C-04 pass; test đối chiếu `se3_log` với GTSAM pass.
- `bench` chạy trên `results/demo/E-01` đã chuyển đổi, in số bản ghi mỗi topic và thời gian phát lại.
- Header interface của 9 module tồn tại, biên dịch (include-what-you-use qua một TU test `tests/unit/test_interfaces_compile.cpp`), đã qua review "hiểu khác nhau".
- ADR-014 chấp nhận; `docs/architecture.html` cập nhật khối `io` và trạng thái `common`.
