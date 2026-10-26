# Lớp nhiễu mô phỏng offline (ADR-011 phần A) — kế hoạch thực hiện

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gói Python `mss_sim` biến một bag thô từ Gazebo (sensor sạch + ground truth) thành bag có nhiễu tái lập được từ seed, kèm `mutate_bag.py`, báo cáo nhiễu trong `eval`, và các test SIM-03..SIM-06; tất cả chạy không cần ROS hay Gazebo.

**Architecture:** `sim/mss_sim/` là thư viện thuần Python: mỗi cảm biến một module (`lidar`, `gnss`, `odom`) nhận mảng numpy và trả mảng numpy; `bridge.py` đọc/ghi MCAP qua `rosbags` và nối các module theo `scenario` + `profile`. `synth.py` sinh bag thô tổng hợp để test không cần Gazebo. `eval/mss_eval/noise_report.py` đọc bag nhiễu và đặt "khai báo" cạnh "đo được".

**Tech Stack:** Python 3.12, numpy, scipy (Rotation/Slerp), rosbags ≥ 0.11 (đọc/ghi MCAP), pyyaml, shapely, allantools, matplotlib, pytest, ruff.

**Spec:** `docs/adr/011-simulation.md` (mục 1, 3, 5, 6, 7, 8, 9). Phần B (world, SDF, driver, ghi bag thô, SIM-01 tĩnh, đóng gói) là kế hoạch riêng.

## Global Constraints

- Python 3.12, type hint đầy đủ, `ruff check` và `ruff format --check` sạch, `pytest` xanh; cấu hình trong `pyproject.toml` gốc.
- `eval/` không bao giờ import `rclpy` hay gọi vào core. `sim/mss_sim/` không import `rclpy` ở cấp module (chỉ `driver.py` của phần B, import trong hàm).
- Script nhận đường dẫn làm tham số và ghi vào thư mục do tham số chỉ định; không ghi vào cây nguồn.
- Khoá YAML có đơn vị trong tên (`_m`, `_s`, `_hz`, `_mps`, `_radps`, `_deg`); quaternion trong YAML theo `[w, x, y, z]`; file TUM theo `x y z w`.
- Mọi ngẫu nhiên lấy từ `numpy.random.Generator` sinh bởi `SeedSequence(seed).spawn(n)`, thứ tự spawn cố định: `[lidar, gnss, odom, imu, mutate]`.
- Timestamp là `int` nano giây; chỉ chuyển sang giây để tính `dt`.
- Giá trị mặc định lấy từ ADR-011 mục 3: LiDAR σ = 0.02 m, p rớt = 0.02 (0.3 khi góc tới > 80°), T_scan = 0.1 s, NaN 0.1%, điểm gần 0.1%; GNSS σ_h = 1.0 m, σ_v = 2.0 m, trôi σ = 0.5 m τ = 300 s, multipath 3–10 m giữ 2–10 s xác suất bắt đầu 0.2/s, degraded ×3; encoder 4096 xung/vòng, σ_v = 0.01 m/s, bán kính +1.0%, khoảng cách bánh −1.5%; `inflation_max` = 5; IMU N_g = 1.75e-4, N_a = 2.9e-3, K_g = 1.0e-5, K_a = 1.0e-4.
- Commit theo Conventional Commits, mỗi task một commit.

## Review Focus

1. PointCloud2 thô không có tổ chức (height = 1): azimuth phải tính từ `atan2(y, x)` chứ không từ chỉ số cột; nếu không, timestamp và skew sai mà không báo lỗi. Test ở Task 7.
2. Scan có timestamp ngoài khoảng ground truth (scan đầu tiên trước GT đầu tiên): nội suy phải clamp và ghi cảnh báo, không ném exception. Test ở Task 5.
3. Điểm `inf`/NaN trong cloud thô (Gazebo trả `inf` khi không có tia về): skew và dropout phải giữ nguyên NaN/inf, không sinh toạ độ rác. Test ở Task 7.
4. Kịch bản GNSS có thời đoạn chồng nhau hoặc đa giác vùng không hợp lệ: `load_scenario` phải báo lỗi nêu tên trường, không im lặng. Test ở Task 3.
5. `JointState` thiếu tên khớp bánh xe hoặc stamp không tăng: bộ lượng tử hoá trả `None` và đếm lỗi, không crash. Test ở Task 9.

---

### Task 1: Gói `mss_sim`, phụ thuộc, hồ sơ nhiễu

**Files:**
- Create: `sim/mss_sim/__init__.py`, `sim/mss_sim/profile.py`, `sim/noise/default.yaml`
- Modify: `pyproject.toml` (dependencies, `packages.find.where = ["eval", "sim"]`, `pytest.pythonpath`)
- Test: `tests/python/test_profile.py`

**Interfaces:**
- Produces: `load_profile(path: Path) -> NoiseProfile`; dataclasses frozen `NoiseProfile(imu: ImuNoise, gnss: GnssNoise, lidar: LidarNoise, odom: OdomNoise, clock_offset_ns: dict[str, int], inflation_max: float)`; `ImuNoise(rate_hz, gyro_noise_density, acc_noise_density, gyro_random_walk, acc_random_walk, gyro_bias_sigma_radps, acc_bias_sigma_mps2, bias_corr_time_s)`; `GnssNoise(rate_hz, sigma_h_m, sigma_v_m, drift_sigma_m, drift_tau_s, multipath_amp_min_m, multipath_amp_max_m, multipath_hold_min_s, multipath_hold_max_s, multipath_start_rate_per_s, degraded_factor)`; `LidarNoise(range_sigma_m, dropout_p, dropout_p_grazing, grazing_deg, scan_period_s, nan_fraction, near_fraction, min_range_m)`; `OdomNoise(ticks_per_rev, speed_sigma_mps, kinematics_true: Kinematics, kinematics_believed: Kinematics)`; `Kinematics(wheel_radius_m, track_m)`.
- `ProfileError(ValueError)` với thông điệp chứa đường dẫn khoá dạng `gnss.sigma_h_m`.

- [ ] **Step 1: Cập nhật `pyproject.toml`**: thêm `dependencies = ["numpy>=1.26", "scipy>=1.12", "pyyaml>=6", "rosbags>=0.11", "shapely>=2.0"]`, `eval = ["evo>=1.28", "matplotlib>=3.8", "allantools>=2024.6"]`, `[tool.setuptools.packages.find] where = ["eval", "sim"]`, `pythonpath = ["eval", "sim", "tools"]`. Chạy `python -m pip install -e ".[dev,eval]"`.

- [ ] **Step 2: Viết `sim/noise/default.yaml`** với đúng các giá trị ở Global Constraints, cấu trúc khoá trùng tên trường dataclass, cộng `clock_offset_ns: {imu: 0, lidar: 0, gnss: 0, odom: 0}`, `inflation_max: 5.0`, `odom.kinematics_true: {wheel_radius_m: 0.10, track_m: 0.40}`, `odom.kinematics_believed: {wheel_radius_m: 0.101, track_m: 0.394}`.

- [ ] **Step 3: Viết test thất bại** `tests/python/test_profile.py`:

```python
def test_default_profile_loads() -> None:
    p = load_profile(REPO / "sim/noise/default.yaml")
    assert p.lidar.range_sigma_m == 0.02
    assert p.odom.kinematics_believed.wheel_radius_m == pytest.approx(0.101)
    assert p.inflation_max == 5.0

def test_missing_field_names_key(tmp_path: Path) -> None:
    bad = yaml.safe_load((REPO / "sim/noise/default.yaml").read_text()); del bad["gnss"]["sigma_h_m"]
    f = tmp_path / "p.yaml"; f.write_text(yaml.safe_dump(bad))
    with pytest.raises(ProfileError, match="gnss.sigma_h_m"): load_profile(f)

def test_negative_sigma_rejected(tmp_path: Path) -> None: ...  # lidar.range_sigma_m = -1 -> ProfileError match "lidar.range_sigma_m"
```

- [ ] **Step 4: Chạy** `python -m pytest tests/python/test_profile.py -v` → FAIL (ImportError).
- [ ] **Step 5: Cài đặt `profile.py`**: dataclasses frozen; `load_profile` đọc YAML, dựng từng dataclass bằng một hàm `_build(cls, mapping, prefix)` báo thiếu/thừa trường và kiểm `>= 0` cho mọi `float`, `> 0` cho `rate_hz`, `ticks_per_rev`, `scan_period_s`, `inflation_max >= 1`.
- [ ] **Step 6: Chạy lại** → PASS. `ruff check . && ruff format --check .` sạch.
- [ ] **Step 7: Commit** `feat(sim): add mss_sim package and noise profile loader`

### Task 2: Geodesy LLA ↔ ENU

**Files:**
- Create: `sim/mss_sim/geodesy.py`
- Test: `tests/python/test_geodesy.py`

**Interfaces:**
- Produces: `Lla = tuple[float, float, float]` (độ, độ, m); `lla_to_ecef(lla) -> np.ndarray(3)`; `ecef_to_lla(xyz) -> Lla`; `lla_to_enu(lla, origin: Lla) -> np.ndarray(3)`; `enu_to_lla(enu: np.ndarray, origin: Lla) -> Lla`. WGS-84 a = 6378137.0, f = 1/298.257223563. Hàm vector hoá: chấp nhận mảng (N, 3).

- [ ] **Step 1: Test thất bại**:

```python
def test_round_trip_under_1mm() -> None:
    origin = (21.0278, 105.8342, 12.0)
    rng = np.random.default_rng(0)
    enu = rng.uniform(-5000, 5000, size=(1000, 3))
    back = lla_to_enu(enu_to_lla(enu, origin), origin)
    assert np.max(np.abs(back - enu)) < 1e-3

def test_east_is_longitude() -> None:  # +100 m east -> longitude increases, latitude unchanged within 1e-9 deg
```

- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** (ECEF chuẩn, `ecef_to_lla` lặp Bowring 5 vòng). **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): WGS-84 LLA/ENU conversion`

### Task 3: Kịch bản, overlay cấu hình, SIM-03

**Files:**
- Create: `sim/mss_sim/scenario.py`, `sim/mss_sim/config_overlay.py`, `sim/scenarios/E-01.yaml`, `config/sim/warehouse.yaml`
- Test: `tests/python/test_scenario.py`, `tests/python/test_sim03_consistency.py`

**Interfaces:**
- Produces: `load_scenario(path) -> Scenario(id, world, profile: Path, pipeline_config: Path, trajectory: Path, seed: int, duration_s: float, origin_lla: Lla, gnss: list[GnssSegment], gnss_zones: list[GnssZone], topics: list[str])`; `GnssSegment(mode: Literal["nominal","outage","multipath","degraded"], t_start_s, t_end_s)`; `GnssZone(mode, polygon_enu: list[tuple[float, float]])` (shapely Polygon hợp lệ, ≥ 3 đỉnh); `ScenarioError(ValueError)`.
- `merge_overlay(base: dict, overlay: dict) -> dict` (deep merge, overlay thắng, không thêm khoá mới ngoài `base` trừ khi khoá cha là `null` trong base; khoá lạ → `KeyError` nêu đường dẫn); `load_pipeline_config(default: Path, overlay: Path) -> dict` trả phần `mss_node.ros__parameters`.

- [ ] **Step 1: Viết `sim/scenarios/E-01.yaml`**: `id: E-01`, `world: warehouse`, `profile: sim/noise/default.yaml`, `pipeline_config: config/sim/warehouse.yaml`, `trajectory: sim/trajectories/warehouse_lap_60s.yaml`, `seed: 42`, `duration_s: 60`, `origin_lla: [21.0278, 105.8342, 12.0]`, `gnss: [{mode: nominal, t_start_s: 0, t_end_s: 60}]`, `gnss_zones: []`, `topics` = 7 topic ở ADR mục 6.
- [ ] **Step 2: Viết `config/sim/warehouse.yaml`**: overlay cùng cấu trúc `mss_node.ros__parameters` với `frames.T_b_l` = dịch `[0.10, 0, 0.50]`, quaternion nghiêng 2° quanh y `[0.99985, 0, 0.01745, 0]`; `frames.p_b_g_m: [-0.20, 0.10, 0.60]`; `imu` noise = sim × 2 (N_g 3.5e-4, N_a 5.8e-3, K_g 2.0e-5, K_a 2.0e-4); `gnss.enabled: true`, `gnss.sigma_h_m: 1.5`, `gnss.sigma_v_m: 3.0`; `wheel_odom.enabled: true`, `wheel_odom.wheel_radius_m: 0.101`, `wheel_odom.track_m: 0.394`, `wheel_odom.speed_sigma_mps: 0.02`; `lidar.range_sigma_m: 0.03`. Thêm các khoá tương ứng (với giá trị rig: `gnss.sigma_*`, `wheel_odom.*`, `lidar.range_sigma_m`) vào `config/default.yaml` để overlay không thêm khoá lạ.
- [ ] **Step 3: Test thất bại** `test_scenario.py`: nạp E-01 đúng trường; thời đoạn chồng nhau (`[0,30]`, `[20,60]`) → `ScenarioError` match `"gnss[1].t_start_s"`; đa giác 2 đỉnh → match `"gnss_zones[0].polygon_enu"`; `merge_overlay` với khoá lạ `"foo"` → `KeyError` match `"imu.foo"`.
- [ ] **Step 4: Test thất bại** `test_sim03_consistency.py` (chính là SIM-03, chạy trên file thật của repo):

```python
@pytest.mark.parametrize("scenario", sorted((REPO / "sim/scenarios").glob("*.yaml")))
def test_pipeline_noise_within_inflation(scenario: Path) -> None:
    sc = load_scenario(scenario); prof = load_profile(REPO / sc.profile)
    cfg = load_pipeline_config(REPO / "config/default.yaml", REPO / sc.pipeline_config)
    for key_cfg, val_sim in PAIRS(prof):   # [("imu.gyro_noise_density", prof.imu.gyro_noise_density), ... gnss.sigma_h_m, gnss.sigma_v_m, lidar.range_sigma_m, wheel_odom.speed_sigma_mps]
        ratio = get(cfg, key_cfg) / val_sim
        assert 1.0 <= ratio <= prof.inflation_max, key_cfg

def test_pipeline_uses_believed_kinematics(...):  # wheel_odom.wheel_radius_m == believed, track_m == believed, p_b_g_m == ADR values
```

- [ ] **Step 5: Chạy cả hai** → FAIL. **Step 6: Cài đặt** `scenario.py`, `config_overlay.py`. **Step 7: Chạy** → PASS, ruff sạch. **Step 8: Commit** `feat(sim): scenario loader, config overlay and SIM-03 consistency test`

### Task 4: Đọc/ghi MCAP và PointCloud2

**Files:**
- Create: `sim/mss_sim/bag_io.py`
- Test: `tests/python/test_bag_io.py`

**Interfaces:**
- Produces: `TYPESTORE = get_typestore(Stores.ROS2_JAZZY)`; `BagReader(path: Path)` context manager với `.topics() -> dict[str, str]` (topic → typename) và `.messages(topics: list[str]) -> Iterator[tuple[str, int, Any]]` (topic, log_time_ns, msg đã deserialize, theo thứ tự log_time); `BagWriter(path: Path)` với `.add_topic(name, typename)` và `.write(topic, log_time_ns, msg)`; `CLOUD_FIELDS = [("x", F32), ("y", F32), ("z", F32), ("intensity", F32), ("t", U32), ("ring", U16)]`; `cloud_to_arrays(msg) -> CloudArrays(xyz: (N,3) f32, intensity: (N,) f32, t_ns: (N,) u32 | None, ring: (N,) u16 | None, height: int, width: int)`; `arrays_to_cloud(arrays: CloudArrays, stamp_ns: int, frame_id: str) -> PointCloud2` (22 byte/điểm, `is_dense=False`); `stamp_ns(header) -> int`; `make_header(stamp_ns: int, frame_id: str)`.
- Bag ghi bằng storage MCAP (`Writer.StoragePlugin.MCAP`), nén zstd nếu rosbags hỗ trợ, `version=9`.

- [ ] **Step 1: Test thất bại**: ghi 3 message `Imu` + 1 `PointCloud2` tổ chức 16×720 với trường `t`, `ring` → đọc lại: số message, thứ tự log_time, `cloud_to_arrays` trả `height == 16`, `xyz` bằng input (`allclose`), `t_ns` nguyên vẹn; cloud input chỉ có `x,y,z` (3 trường) → `t_ns is None`.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** bằng `rosbags.rosbag2.Reader/Writer`, `numpy.frombuffer` với dtype cấu trúc theo `point_step` và `fields` của message (không giả định thứ tự trường cố định khi đọc). **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): MCAP reader/writer and PointCloud2 array helpers`

### Task 5: Ground truth và nội suy pose

**Files:**
- Create: `sim/mss_sim/gt.py`
- Test: `tests/python/test_gt.py`

**Interfaces:**
- Produces: `GroundTruth(t_ns: (N,) int64, p_w_b: (N,3) f64, q_wxyz: (N,4) f64)` với `from_odometry_msgs(msgs) -> GroundTruth`, `pose_at(t_ns: np.ndarray) -> tuple[R: (M,3,3), p: (M,3)]` (dịch tuyến tính, quay Slerp bằng `scipy.spatial.transform`), clamp ngoài khoảng và cộng dồn `self.n_clamped`, `log_warn` một lần; `to_tum(path: Path)` ghi `t x y z qx qy qz qw` giây 9 chữ số; `warnings` dùng `logging.getLogger("mss_sim")`.

- [ ] **Step 1: Test thất bại**: GT tổng hợp quay đều ω = 0.5 rad/s quanh z ở 200 Hz trong 2 s, hỏi tại t giữa hai mẫu → góc yaw khớp `ω·t` < 1e-9 rad, p khớp tuyến tính; `pose_at([-1e9])` → bằng pose đầu, `n_clamped == 1`, không exception; `to_tum` rồi `mss_eval.tum.read_tum` đọc lại đúng N dòng và quaternion thứ tự `x y z w`.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt.** **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): ground-truth interpolation and TUM export`

### Task 6: Bag thô tổng hợp

**Files:**
- Create: `sim/mss_sim/synth.py`
- Test: `tests/python/test_synth.py`, fixture dùng chung trong `tests/python/conftest.py`

**Interfaces:**
- Produces: `write_synthetic_raw_bag(out_dir: Path, *, duration_s: float = 6.0, seed: int = 0, organized: bool = True, box_half_m: tuple = (10, 6, 3), omega_radps: float = 0.3, speed_mps: float = 0.5, origin_lla: Lla) -> SynthInfo(planes: list[tuple[n, d]], t0_ns: int)`: robot đi vòng tròn bán kính `speed/omega` trong phòng hộp 6 mặt; ghi `/ground_truth` 200 Hz, `/imu/data` 200 Hz (ω và gia tốc đúng động học, không nhiễu), `/lidar/points` 10 Hz 16×720 (ray-cast lên 6 mặt phẳng, `inf` khi > 100 m, chỉ `x,y,z,intensity`, render tại một thời điểm = stamp), `/gnss/fix` 5 Hz từ GT + `p_b_g` sang LLA quanh `origin_lla`, `/joint_states` 50 Hz (tên `left_wheel_joint`, `right_wheel_joint`, `velocity` rad/s từ `kinematics_true` mặc định r = 0.10, track = 0.40), `/tf_static` với `T_b_l` identity.
- Fixture `raw_bag(tmp_path_factory) -> tuple[Path, SynthInfo]` scope session.

- [ ] **Step 1: Test thất bại**: bag có đủ 6 topic; số scan = 60 cho 6 s; mọi điểm hữu hạn nằm trên một trong 6 mặt phẳng (khoảng cách < 1e-6 m); stamp từng topic tăng chặt.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** (ray-plane intersection vector hoá cho lưới 16 × 720, góc dọc −15..15°, ngang 0..360° theo cột). **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): synthetic raw bag generator for ROS-free tests`

### Task 7: LiDAR: timestamp từng điểm, skew, rớt điểm, điểm xấu (SIM-04)

**Files:**
- Create: `sim/mss_sim/lidar.py`
- Test: `tests/python/test_lidar.py`

**Interfaces:**
- Consumes: `CloudArrays`, `GroundTruth.pose_at`, `LidarNoise`.
- Produces: `point_times_ns(xyz: (N,3), stamp_ns: int, scan_period_s: float) -> (N,) u32` (offset = `(atan2(y, x) mod 2π) / 2π · T_scan`, tính từ toạ độ, không từ cột; NaN/inf → 0); `ring_index(arrays) -> (N,) u16` (từ `height`/`width` nếu tổ chức, nếu không từ góc nâng lượng tử 16 mức); `inject_skew(xyz, t_ns, stamp_ns, gt, T_b_l: (4,4)) -> xyz'` theo `p_i ← T_l(t_i)⁻¹ · T_l(t_scan) · p_i`, `T_l(t) = T_w_b(t) · T_b_l`, giữ nguyên hàng NaN/inf; `apply_dropout(arrays, noise: LidarNoise, rng) -> mask (N,) bool` với góc tới ước lượng từ pháp tuyến cục bộ (tích có hướng của sai phân theo cột và theo hàng, chỉ khi tổ chức; không tổ chức → dùng `dropout_p` đều); `inject_bad_points(xyz, noise, rng) -> xyz'` (NaN `nan_fraction`, điểm tại `0.3 m` theo hướng ngẫu nhiên `near_fraction`); `process_scan(arrays, stamp_ns, gt, T_b_l, noise, rng, *, skew: bool = True) -> CloudArrays`.

- [ ] **Step 1: Test thất bại**:

```python
def test_static_robot_cloud_unchanged(): ...        # GT đứng yên -> inject_skew trả xyz allclose 1e-12 (SIM-04a)
def test_constant_yaw_rate_matches_analytic(): ...  # GT quay ω=1 rad/s quanh z, T_b_l = I: điểm có offset t_i phải bằng R_z(-ω t_i) p, sai số < 1e-3 m (SIM-04b)
def test_unorganized_cloud_uses_atan2(): ...        # height=1, xáo trộn thứ tự điểm -> point_times_ns giống bản tổ chức theo từng điểm
def test_nan_inf_passthrough(): ...                 # hàng inf giữ inf sau process_scan, không có warning numpy
def test_dropout_rate_and_grazing(): ...            # p=0.02 trên 100k điểm -> giữ 97.5–98.5%; mặt phẳng nhìn xiên 85° -> giữ ≤ 75%
def test_bad_points_fractions(): ...                # đúng số NaN và số điểm có range < 0.5 m (±1 điểm)
def test_deterministic_with_seed(): ...             # hai lần cùng seed -> bằng nhau từng byte
```

- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt.** **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): lidar per-point time, motion skew, dropout and bad points`

### Task 8: GNSS degrader (SIM-06)

**Files:**
- Create: `sim/mss_sim/gnss.py`
- Test: `tests/python/test_gnss.py`

**Interfaces:**
- Consumes: `GnssNoise`, `GnssSegment`, `GnssZone`, `geodesy`.
- Produces: `GnssDegrader(noise, segments, zones, origin_lla, rng)` với `.step(t_ns: int, p_e_g_true: np.ndarray(3)) -> GnssSample | None` (`None` = outage); `GnssSample(lla: Lla, cov_enu_diag: np.ndarray(3), status: int)`; `mode_at(t_s, p_enu) -> str` (vùng thắng thời đoạn; ngoài mọi thời đoạn = nominal). Trôi Gauss-Markov: `x ← e^{-dt/τ} x + σ √(1 − e^{-2dt/τ}) n`. Multipath: trong mode đó, bắt đầu sự kiện với xác suất `start_rate · dt`, biên độ `U(amp_min, amp_max)` theo hướng ngang ngẫu nhiên, giữ `U(hold_min, hold_max)` s; `cov_enu_diag` = nominal. Degraded: σ × factor và cov × factor². `cov_enu_diag` nominal = `[σ_h², σ_h², σ_v²]`.

- [ ] **Step 1: Test thất bại**: outage `[10, 20]` → `step` trả `None` đúng trong đoạn, có mẫu ngay ngoài đoạn; nominal 2000 mẫu đứng yên → σ ngang đo được trong ±10% của 1.0 m sau khi trừ trôi (dùng τ lớn 1e9 để tắt trôi); multipath với seed cố định → tồn tại đoạn liên tiếp ≥ 2 s có sai số > 3 m và `cov_enu_diag` không đổi; degraded → cov × 9; zone đa giác quanh gốc → điểm trong vùng dùng mode vùng; `mode_at` ngoài thời đoạn → `"nominal"`; hai lần cùng seed → chuỗi mẫu giống hệt.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt.** **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): GNSS degrader with outage, multipath, drift and zones`

### Task 9: Encoder và odom "robot tin"

**Files:**
- Create: `sim/mss_sim/odom.py`
- Test: `tests/python/test_odom.py`

**Interfaces:**
- Consumes: `OdomNoise`.
- Produces: `WheelQuantizer(noise, rng, left_joint="left_wheel_joint", right_joint="right_wheel_joint")` với `.step(t_ns, names: list[str], velocity_radps: list[float]) -> OdomTwist | None` (`None` khi thiếu khớp hoặc stamp không tăng, tăng `self.n_errors`); tích phân góc mỗi bánh, lượng tử `floor(angle · ticks/2π)`, vận tốc từ chênh tick / dt với `kinematics_believed`, cộng Gauss `speed_sigma_mps` lên v; `OdomTwist(v_mps, omega_radps)`; `integrate_pose(twists: list[tuple[t_ns, OdomTwist]]) -> (N,3) [x, y, yaw]`.

- [ ] **Step 1: Test thất bại**: đi thẳng v = 0.5 m/s 60 s ở 50 Hz, σ_v = 0 → quãng đường odom ≈ 30 m × 1.010 (±0.1%); quay tại chỗ ω = 1 rad/s 2π s → yaw odom = 2π × (0.10/0.101) × (0.394/0.40)⁻¹ … tức `yaw = 2π · (r_b/r_t) · (track_t/track_b)` trong ±0.5%; thiếu tên khớp → `None` và `n_errors == 1`; stamp lùi → `None`; lượng tử hoá: ở v rất nhỏ 0.001 m/s, chuỗi tick không đổi cho đến khi tích luỹ đủ 1 tick.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt.** **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(sim): encoder quantisation and believed-kinematics odometry`

### Task 10: Bridge offline, CLI, SIM-05

**Files:**
- Create: `sim/mss_sim/bridge.py`, `sim/mss_sim/__main__.py` (CLI `python -m mss_sim bridge|gt-to-tum`), `sim/README.md` (cập nhật)
- Test: `tests/python/test_bridge.py`

**Interfaces:**
- Consumes: mọi module trên.
- Produces: `run_bridge(raw_bag: Path, scenario: Scenario, profile: NoiseProfile, out_dir: Path, *, seed: int | None = None, skew: bool = True) -> BridgeReport(n_msgs: dict[str, int], n_dropped_gnss: int, n_odom_errors: int, n_gt_clamped: int)`; ghi `out_dir/bag/` (MCAP) với topic `/lidar/points` (có `t`, `ring`), `/imu/data` (giữ nguyên + `clock_offset_ns.imu`), `/gnss/fix` (từ degrader, `position_covariance_type = 2`), `/wheel/odom` (`nav_msgs/Odometry`, `child_frame_id = "base_link"`, twist từ `WheelQuantizer`, pose từ `integrate_pose`), `/ground_truth` và `/tf_static` sao chép nguyên; `out_dir/gt.tum`; `out_dir/bridge_report.json`. `T_b_l` và `p_b_g` đọc từ `load_pipeline_config(...)["frames"]`. Thứ tự ghi = thứ tự log_time của bag thô. CLI: `python -m mss_sim bridge <raw_bag> <scenario.yaml> <out_dir> [--seed N] [--no-skew]`, `python -m mss_sim gt-to-tum <bag> <out.tum>`.

- [ ] **Step 1: Test thất bại** (dùng fixture `raw_bag`, scenario E-01 với `origin_lla` của fixture và `duration_s` 6):

```python
def test_bridge_outputs_all_topics(raw_bag, tmp_path): ...   # 7 topic? (6 + /wheel/odom thay /joint_states), gt.tum tồn tại, bridge_report.json có n_msgs
def test_lidar_has_t_and_ring(...): ...                       # cloud_to_arrays(...).t_ns không None, max(t_ns) < 1e8
def test_bridge_is_deterministic(raw_bag, tmp_path): ...     # SIM-05: chạy 2 lần cùng seed -> với mỗi topic, danh sách (log_time, rawdata bytes) bằng nhau
def test_different_seed_differs(...): ...                     # ít nhất /gnss/fix khác
def test_no_skew_flag_keeps_xyz(...): ...                     # --no-skew: xyz bằng bag thô (trừ điểm bị dropout/bad: so trên mask giữ)
```

- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt** `bridge.py` + `__main__.py` (argparse). **Step 4: Chạy** → PASS; chạy CLI tay trên bag fixture để xem `bridge_report.json`. **Step 5: Cập nhật `sim/README.md`**: lệnh bridge, cấu trúc profile/scenario, lưu ý ADR-011 về tất định. **Step 6: Commit** `feat(sim): offline noise bridge CLI with deterministic output`

### Task 11: `tools/mutate_bag.py` (sinh E-03)

**Files:**
- Create: `tools/mutate_bag.py`
- Test: `tests/python/test_mutate_bag.py`

**Interfaces:**
- Consumes: `BagReader`, `BagWriter`.
- Produces: `mutate(in_bag: Path, out_bag: Path, *, drop_fraction: float = 0.05, reorder_fraction: float = 0.02, window_ns: int = 20_000_000, topics: tuple[str, ...] = ("/imu/data", "/lidar/points"), seed: int = 0) -> MutateReport(n_in, n_dropped, n_reordered)`; rớt theo Bernoulli trên message thuộc `topics`; hoán đổi: chọn message, đổi `log_time` với message kế tiếp cùng topic nếu cách < `window_ns` (header stamp giữ nguyên); CLI `python tools/mutate_bag.py <in> <out> [--drop 0.05] [--reorder 0.02] [--seed 0]`.

- [ ] **Step 1: Test thất bại**: trên bag bridge (fixture chạy Task 10 một lần, scope session): số IMU + LiDAR giảm 5% ± 1%; `/ground_truth` không đổi; tồn tại ≥ 1 cặp message cùng topic có `log_time` tăng nhưng `header.stamp` giảm; cùng seed → giống hệt.
- [ ] **Step 2: Chạy** → FAIL. **Step 3: Cài đặt.** **Step 4: Chạy** → PASS. **Step 5: Commit** `feat(tools): mutate_bag for packet-loss and reordering scenarios`

### Task 12: Báo cáo nhiễu trong `eval`

**Files:**
- Create: `eval/mss_eval/allan.py`, `eval/mss_eval/noise_report.py`, `eval/mss_eval/report_html.py`
- Test: `tests/python/test_allan.py`, `tests/python/test_noise_report.py`

**Interfaces:**
- Consumes: bag nhiễu (đọc bằng `rosbags` trực tiếp trong `eval`, **không** import `mss_sim`; sao chép tối thiểu `cloud_to_arrays` vào `eval/mss_eval/pointcloud.py`), `sim/noise/*.yaml` (đọc YAML thẳng, không import `mss_sim.profile`), tuỳ chọn `--planes planes.yaml` (danh sách `{n: [..], d: ..}`), tuỳ chọn `--pipeline-config` để in cột "pipeline tin".
- Produces: `allan.py`: `oadev(x: np.ndarray, rate_hz: float) -> (taus, adev)` (bọc `allantools.oadev`), `fit_white_and_walk(taus, adev) -> (N, K)` (N = adev tại τ = 1 s theo khớp đoạn dốc −½ trên τ ∈ [0.01, 1]; K = adev tại τ = 3 s trên đoạn dốc +½), `theory_curve(taus, N, K)`; `noise_report.py`: `run(bag_dir: Path, profile: Path, out_dir: Path, *, planes: Path | None, pipeline_config: Path | None, static_s: float = 5.0, allan_gate: bool = False) -> Report(sections: list[Section], passed: bool)`, mỗi `Section(name, rows: list[Row(name, declared, measured, tol, ok)], figures: list[bytes png])`; ghi `out_dir/noise_report.html`, `out_dir/noise_report.json`; CLI `python -m mss_eval.noise_report <bag_dir> <profile.yaml> --out <dir> [--planes ..] [--pipeline-config ..] [--allan-gate]` exit 1 nếu `passed` sai. Các mục và tiêu chí đúng bảng ADR mục 7: IMU (σ trắng từ `static_s` đầu ±15%; Allan chỉ gate khi `--allan-gate`, lệch < 10% trên τ ∈ [0.01, 100]), GNSS (σ đoạn nominal ±15%, histogram, vùng tô theo `bridge_report`/scenario nếu có), LiDAR (chỉ khi có `--planes`: σ ±15%, tỷ lệ điểm không hữu hạn so `dropout_p` ±20% tương đối), Wheel odom (trôi theo mét so GT, in số, gate theo khoảng 1–3% chỉ khi `--odom-gate`), Bag (SIM-02: mỗi topic stamp tăng chặt, có `/ground_truth`, khoảng trống tối đa < 0.5 s).
- HTML: `report_html.py` dựng trang tĩnh tự chứa, token màu và bố cục bảng giống `docs/architecture.html` (sáng/tối), hình PNG base64.

- [ ] **Step 1: Test thất bại** `test_allan.py`: sinh 2 giờ tín hiệu 200 Hz = white(N = 1.75e-4) + tích phân white(K = 1e-5), seed 0 → `fit_white_and_walk` trả N, K trong ±10%.
- [ ] **Step 2: Test thất bại** `test_noise_report.py` (bag bridge fixture, planes từ `SynthInfo.planes`): `run(...)` tạo HTML và JSON; mục Bag pass; mục GNSS có hàng `sigma_h_m` với measured trong ±15% của 1.0; mục LiDAR có `range_sigma_m` measured trong ±15% của 0.02 (bag thô tổng hợp không có nhiễu range, nên fixture bridge cho test này phải bật thêm nhiễu range trong `process_scan` — thêm tham số `range_sigma_apply: bool` vào `process_scan`, mặc định `False` vì Gazebo đã thêm; bật trong fixture test); bag bị xoá `/ground_truth` → `passed False` và JSON nêu `"ground_truth missing"`; exit code CLI = 1 khi fail.
- [ ] **Step 3: Chạy** → FAIL. **Step 4: Cài đặt** ba file (matplotlib backend `Agg`). **Step 5: Chạy** → PASS; mở `noise_report.html` sinh từ fixture để kiểm bằng mắt (Edge headless screenshot nếu không có màn hình). **Step 6: Commit** `feat(eval): noise report with Allan, GNSS, LiDAR, odom and bag checks`

### Task 13: Kết thúc: lint toàn repo, cập nhật tài liệu và CI

**Files:**
- Modify: `.github/workflows/ci.yml` (job `python` cài `.[dev,eval]`), `README.md` (mục Python tooling: bridge, mutate, noise report), `docs/architecture.html` (khối `eval` và khối mới `sim (py)`: trạng thái `done`, hàm thật, test SIM-03..06), `eval/README.md`, `tools/fetch_test_data.sh` (ghi chú manifest nhiều file, nội dung chờ phần B)

- [ ] **Step 1:** `ruff check . && ruff format --check . && python -m pytest -q` sạch toàn bộ.
- [ ] **Step 2:** Cập nhật ba file tài liệu và `architecture.html` (sửa khối `DATA`: thêm module `sim` với các hàm `load_profile`, `run_bridge`, `process_scan`, `GnssDegrader.step`, `WheelQuantizer.step`, `mutate`; `eval` thêm `noise_report.run`, `fit_white_and_walk`; test SIM-03..06 và số test pytest thực tế).
- [ ] **Step 3:** Commit `docs: document sim noise layer and update architecture status`; push và xác nhận CI xanh (`gh run watch`).

---

## Tự soát

- Phủ spec: mục 1 (kiến trúc offline, tất định) → Task 10; mục 3 (IMU pass-through + clock offset → Task 10; GNSS → 8; LiDAR → 7; odom → 9; clock offset → 10); mục 5 (scenario, overlay, E-03 mutate) → 3, 11; mục 6 (topic, MCAP, gt.tum) → 4, 5, 10; mục 7 → 12; mục 8 (SIM-03 → 3, SIM-04 → 7, SIM-05 → 10, SIM-06 → 8; SIM-01/02 kiểm trong 12, SIM-01 gate thật thuộc phần B); mục 9 phụ thuộc → 1. Không có task cho world/SDF/driver/ghi bag thô/đóng gói: thuộc phần B, cố ý.
- Kiểu nhất quán: `CloudArrays`, `GroundTruth.pose_at`, `GnssDegrader.step`, `WheelQuantizer.step`, `run_bridge`, `BagReader/BagWriter` dùng cùng tên ở mọi task.
- Review Focus: 1 và 3 → Task 7; 2 → Task 5; 4 → Task 3; 5 → Task 9.
