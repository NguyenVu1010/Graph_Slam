# ADR-011: Môi trường mô phỏng Gazebo Harmonic cho kiểm chứng 4 cảm biến

- Trạng thái: đề xuất
- Ngày: 2026-10-22, cập nhật 2026-10-23 sau builder check
- Liên quan: ADR-003 (lõi không ROS), ADR-008 (YAML là nguồn tham số duy nhất), ADR-009 và `docs/Test_CI_CD.md` (E-01..E-07, SIM-01, SIM-02), ADR-010 (MCAP, TUM, evo)

## Phạm vi

ADR này bao gồm: world, robot, mô hình nhiễu, lớp làm bẩn, ghi bag, ground truth, `mutate_bag.py`, báo cáo nhiễu, test SIM-01..SIM-06, đóng gói `testdata-vN`. **Không** bao gồm harness chạy `Estimator` trên bag và so ATE với baseline (E-01..E-07, `integration.yml`): phần đó thuộc ADR-009/010 và làm cùng với `pipeline`, vì cần pipeline tồn tại trước.

## Bối cảnh

Mô phỏng là nơi duy nhất có đủ bốn cảm biến (LiDAR, IMU, GNSS, wheel odom): rig thật không có GNSS lẫn encoder, UrbanNav không có encoder. Mọi PR phải qua E-01..E-03 trên bag mô phỏng, hằng đêm chạy E-04..E-07. Vì vậy kết quả SLAM trong sim chỉ có ý nghĩa nếu:

1. Nhiễu cảm biến có mô hình rõ, tham số khai báo được, và đo lại được từ bag (SIM-01 đòi Allan variance khớp cấu hình trong 10%).
2. Vật lý đủ thật để wheel odom sai như thật (trượt bánh, ma sát khác nhau theo bề mặt, giới hạn gia tốc), vì nếu odom hoàn hảo thì factor odom không bao giờ bị thử thách.
3. Dữ liệu LiDAR có méo chuyển động (skew), vì deskew là bước đầu của pipeline và sensor Gazebo mặc định render cả scan tại một thời điểm.
4. GNSS có mất tín hiệu, multipath và trôi chậm (E-05, E-06, G-02), trong khi sensor NavSat của Gazebo chỉ có nhiễu Gauss.
5. Bag tái lập bit-for-bit từ seed, nhỏ hơn 100 MB mỗi file, có ground truth.

Máy chạy sim: amd64, GPU 8 GB, RAM 16 GB, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. CI không có GPU và không chạy Gazebo.

## Quyết định

### 1. Kiến trúc: Gazebo cho vật lý và hình học, lớp "làm bẩn" bằng Python cho nhiễu

```
Gazebo Harmonic ──ros_gz_bridge──▶ topic thô ──▶ sim_bridge (rclpy) ──▶ topic đã nhiễu ──▶ rosbag2 MCAP
   vật lý, va chạm,                 (sạch hoặc nhiễu          skew LiDAR, rớt điểm,           + /ground_truth
   render LiDAR, IMU Gauss           Gauss đơn giản)          GNSS degrader, encoder quantize
```

- Gazebo chịu trách nhiệm: vật lý (DART, bước 1 ms), hình học world, render LiDAR GPU, nhiễu Gauss và bias động của IMU (sensor IMU của Gazebo đã mô hình hoá đủ cho SIM-01).
- Gói Python `sim/mss_sim/` (rclpy, chỉ dùng trong `sim/`) chịu trách nhiệm mọi nhiễu mà Gazebo không có: skew LiDAR, rớt điểm, GNSS degrader, lượng tử hoá encoder, lệch đồng hồ cố định. Mọi nguồn ngẫu nhiên lấy seed từ file kịch bản.
- Lý do tách: nhiễu phức tạp viết bằng Python kiểm thử được bằng pytest không cần Gazebo, tái lập được, và đổi mô hình nhiễu không phải sửa SDF hay viết plugin C++.
- `eval/` vẫn không dính ROS: đọc MCAP bằng thư viện `rosbags` (thuần Python).

**Tính tất định và nguồn dữ liệu.** Gazebo (DART, render GPU, transport) không tất định, và driver chạy trên trạng thái sim nên quỹ đạo cũng khác nhau giữa hai lần. Vì vậy:

- `sim_bridge` là một thư viện Python chạy **offline** trên bag thô (đường chính), và được bọc thành node live chỉ để xem trực tiếp. Bag thô = sensor sạch (chỉ nhiễu Gauss IMU của Gazebo) + `/ground_truth`.
- Mỗi kịch bản ghi bag thô **một lần**, đánh phiên bản và SHA256 cùng `testdata-vN`. "Tái lập từ seed" áp cho lớp nhiễu: `sim_bridge(bag thô, noise.yaml, seed)` → bag nhiễu giống hệt byte (SIM-05). Bag nhiễu E-01..E-03 cũng được đóng gói sẵn để CI không phải chạy bridge.
- "Nhiều seed" hằng đêm là nhiều seed nhiễu trên cùng bag thô. Đổi quỹ đạo hay world là ghi bag thô mới và nâng phiên bản testdata.

### 2. Robot mô phỏng

| Hạng mục | Giá trị | Ghi chú |
| --- | --- | --- |
| Đế | bánh vi sai, 2 bánh chủ động + 2 caster | khối lượng 20 kg, bán kính bánh 0.10 m, khoảng cách hai bánh 0.40 m |
| Giới hạn | v ≤ 1.0 m/s, a ≤ 0.5 m/s², ω ≤ 1.0 rad/s | đặt trong bộ điều khiển, để vận tốc thật trễ so với lệnh như thật |
| LiDAR `l` | 16 tia, 360°, dọc ±15°, 0.5–100 m, 10 Hz, 720 mẫu ngang (0.5°) | `gpu_lidar`; đặt cao 0.5 m, lệch IMU 0.10 m theo x, nghiêng 2° quanh y để `T_b_l` không tầm thường |
| IMU `b` | 200 Hz, tại gốc body | sensor `imu` của Gazebo |
| GNSS `g` | 5 Hz, anten lệch `p_b_g = [-0.20, 0.10, 0.60]` m | sensor `navsat`, gốc ENU tại điểm khởi hành của world |
| Encoder | 4096 xung/vòng mỗi bánh | lượng tử hoá trong `sim_bridge` |
| Ground truth | pose và twist của body ở 200 Hz | plugin `OdometryPublisher` trên link IMU, không nhiễu |

SDF của robot sinh từ template `sim/models/robot.sdf.j2` bằng `tools/gen_sim_assets.py`, tham số lấy từ `sim/noise/<profile>.yaml` và `config/default.yaml` (extrinsics). Không sửa tay file SDF đã sinh.

### 3. Mô hình nhiễu và tham số mặc định (`sim/noise/default.yaml`)

Mọi tham số viết theo đơn vị liên tục (continuous-time) như datasheet. Script sinh SDF đổi sang dạng rời rạc của Gazebo: `stddev_rời_rạc = N · √f` với `f` là tần số mẫu.

**IMU** (lớp MPU-9250, giá trị khởi đầu, chỉnh khi có datasheet model thật):

| Tham số | Gyro | Accel |
| --- | --- | --- |
| Mật độ nhiễu trắng N | 1.75e-4 rad/s/√Hz (0.01 °/s/√Hz) | 2.9e-3 m/s²/√Hz (300 µg/√Hz) |
| Random walk bias K | 1.0e-5 rad/s²/√Hz | 1.0e-4 m/s³/√Hz |
| Bias ban đầu (σ, rút một lần theo seed) | 0.035 rad/s (2 °/s) | 0.3 m/s² (30 mg) |
| Thời gian tương quan bias động | 1000 s | 1000 s |

Gazebo mô hình bias động bằng quá trình Gauss-Markov bậc 1, không phải random walk thuần. Với thời gian tương quan 1000 s, Allan deviation khớp random walk trên τ ∈ [0.01, 100] s; SIM-01 chỉ so trong dải này. Ghi rõ điều này trong báo cáo nhiễu.

**GNSS** (lớp thu dân dụng, không RTK):

| Chế độ | Mô hình | Tham số mặc định |
| --- | --- | --- |
| `nominal` | Gauss trắng ngang σ_h, dọc σ_v, cộng trôi chậm Gauss-Markov | σ_h = 1.0 m, σ_v = 2.0 m; trôi: σ = 0.5 m, τ = 300 s |
| `outage` | không phát message | thời đoạn trong kịch bản |
| `multipath` | cộng bias nhảy bậc ngẫu nhiên vào vị trí, giữ 2–10 s, covariance báo cáo vẫn là nominal | biên độ 3–10 m, xác suất bắt đầu 0.2/s trong vùng |
| `degraded` | σ_h, σ_v nhân hệ số, covariance báo cáo đúng | hệ số 3 |

Chế độ gắn theo thời đoạn hoặc theo vùng không gian (đa giác trong world, ví dụ cạnh toà nhà). Covariance trong message `NavSatFix` là giá trị bộ thu "tin", không phải sai số thật, để kiểm định chi-square (G-02, E-06) có ý nghĩa.

**LiDAR**:

| Hiện tượng | Mô hình | Mặc định |
| --- | --- | --- |
| Nhiễu khoảng cách | Gauss trên range, Gazebo | σ = 0.02 m |
| Rớt điểm | Bernoulli theo điểm, thêm xác suất cao hơn khi góc tới lớn | p = 0.02; p = 0.3 khi góc tới > 80° |
| Timestamp từng điểm | `t_i = t_start + (azimuth_i / 2π) · T_scan` (`t_start` = `header.stamp`), trường `t` uint32 ns offset, `ring` uint16 | T_scan = 0.1 s |
| Méo chuyển động | tiêm từ ground truth: `p_i ← T_l(t_i)⁻¹ · T_l(t_start) · p_i` với `T_l(t)` nội suy từ `/ground_truth` | luôn bật; tắt được để test P-02. Deskew đưa điểm về khung `l` tại `t_start` (không phải cuối scan); điểm có `t_offset_ns == 0` không đổi |
| Tầm tối thiểu | điểm < 0.5 m bị loại ở sensor; `sim_bridge` chèn thêm 0.1% điểm NaN và điểm "trên thân robot" | để test P-04 |

**Wheel odom**:

- Vận tốc bánh lấy từ joint thật trong Gazebo (nên trượt bánh của vật lý đi thẳng vào odom), lượng tử hoá 4096 xung/vòng, cộng nhiễu Gauss nhỏ trên tốc độ σ = 0.01 m/s.
- Sai số hiệu chuẩn có chủ đích: odom tính bằng bán kính bánh và khoảng cách hai bánh mà "robot tin" (`kinematics_believed`), lệch so với giá trị thật của SDF (`kinematics_true`); mặc định bán kính +1.0%, khoảng cách bánh −0.25%. Đây là nguồn trôi chính trên sàn khô, vì trượt vật lý thuần với µ = 0.8 và a ≤ 0.5 m/s² rất nhỏ; trượt bánh chỉ cần đủ để trôi tăng rõ trên cỏ và vũng ướt.
- Mục tiêu đo được (kiểm tra bằng báo cáo nhiễu, builder chỉnh plugin cho đạt): trôi odom thuần 1–3% quãng đường trên sàn kho, 3–6% trên cỏ; sai số yaw 4–5° mỗi vòng quay tại chỗ (≈ 4.5° từ hiệu chuẩn, hệ số (r_tin/r_thật)·(track_thật/track_tin) = 1.0125; trượt vật lý cộng thêm trên cỏ).

**Bổ sung 2026-10-27 (sai số hiệu chuẩn odom và mục tiêu yaw).** Khoảng cách bánh "tin" đổi từ −1.5% (`track_m: 0.394`) sang −0.25% (`track_m: 0.399`); bán kính giữ +1.0% (`wheel_radius_m: 0.101`); mục tiêu yaw đổi từ 0.3–1.0° sang 4–5° mỗi vòng quay tại chỗ; mục tiêu trôi quãng đường 1–3% giữ nguyên. Lý do: odom tính `ω = (v_phải − v_trái) / track_tin` với tốc độ bánh nhân bán kính tin, nên yaw bị nhân hệ số (r_tin/r_thật)·(track_thật/track_tin), không chỉ sai số khoảng cách bánh. Cặp cũ cho 1.01 × 0.40/0.394 ≈ 1.0254, tức ≈ 9.1° mỗi vòng, mâu thuẫn với mục tiêu 0.3–1.0°. Muốn dưới 1° thì hai sai số phải gần triệt tiêu nhau (triệt tiêu hoàn toàn tại khoảng cách bánh tin +1.0% = 0.404 m; dưới 1° trong khoảng +0.72…+1.28%), một sự trùng hợp gượng ép không giống robot thật. Nên giữ hai sai số độc lập và đặt mục tiêu theo phép tính: 1.01 × 0.40/0.399 ≈ 1.0125, tức ≈ 4.5° mỗi vòng từ hiệu chuẩn. Hệ quả: odom đáng tin cho tịnh tiến ngắn (trôi quãng đường ≈ 1% do bán kính) nhưng yếu về hướng; đó đúng là vai trò của factor between và factor nonholonomic của odom trong graph (`docs/derivations/wheel_odom.md`; ADR-004 chỉ quy định odom là factor riêng sau cờ YAML): hướng do LiDAR và IMU giữ, odom chủ yếu ràng buộc tịnh tiến và chuyển động không trượt ngang.

**Đồng hồ**: tất cả cảm biến dùng đồng hồ sim (`/clock`). `sim/noise/*.yaml` có `clock_offset_s` cho từng cảm biến, mặc định 0, để sau này thử nghiệm lệch đồng hồ.

### 4. Vật lý "sát tự nhiên"

| Hạng mục | Quyết định |
| --- | --- |
| Engine | DART, bước 1 ms, không yêu cầu real-time factor = 1; ghi bag theo sim time |
| Ma sát bề mặt | sàn bê tông µ = 0.8; cỏ µ = 0.5; vũng ướt / tấm kim loại µ = 0.3 (vùng riêng trong world ngoài trời) |
| Trượt bánh | plugin `WheelSlip` của Gazebo, slip dọc và ngang; giá trị chọn để đạt mục tiêu trôi odom ở mục 3 |
| Lực cản lăn | `damping` và `friction` trên joint bánh (khởi đầu 0.1 N·m·s/rad và 0.2 N·m); giới hạn moment động cơ |
| Địa hình | kho và hành lang: phẳng; ngoài trời: heightmap gồ ±5 cm, một gờ 3 cm cắt ngang đường để kích thích IMU |
| Quán tính | tensor quán tính tính từ hình hộp thực, không để mặc định |

### 5. World và kịch bản (`sim/worlds/`, `sim/scenarios/`)

| World | Kích thước | Đặc điểm | Kịch bản dùng |
| --- | --- | --- | --- |
| `warehouse` | 40 × 25 m, trần 6 m | kệ hàng, thùng, cột; đủ ràng buộc 6 mặt; có một đoạn kệ dài 15 m gần suy biến nhẹ | E-01, E-02, E-03 |
| `corridor` | 60 × 2.5 m | hai tường trơn song song, sàn, trần; cuối có phòng để kết thúc | E-04 |
| `outdoor` | 120 × 120 m | 3 toà nhà (vùng multipath), hàng cây, bãi cỏ, đường bê tông, vũng ướt; cổng hầm 25 m (vùng outage) | E-05, E-06 |
| `loop` | vòng 300 m quanh khối nhà trong `outdoor` | về đúng điểm xuất phát, có đoạn trùng đường | E-07 |

Mỗi kịch bản là một file YAML: world, profile nhiễu, quỹ đạo (waypoint), kịch bản GNSS (thời đoạn và vùng), seed, thời lượng, danh sách topic ghi. Ví dụ `sim/scenarios/E-01.yaml` = `warehouse` + `default` + `warehouse_lap_60s` + GNSS nominal + seed 42 + 60 s.

Robot chạy theo waypoint bằng bộ điều khiển pure-pursuit trong `sim/mss_sim/driver.py`, tốc độ mục tiêu có dao động ngẫu nhiên ±10% theo seed. Mọi kịch bản bắt đầu bằng 5 s đứng yên (cho IMU init của pipeline và cho mục IMU của báo cáo nhiễu). E-03 không ghi lại từ sim: sinh từ bag E-01 bằng `tools/mutate_bag.py` (rớt 5% message IMU và LiDAR, hoán đổi thứ tự 2% message trong cửa sổ 20 ms, seed cố định).

**Cấu hình pipeline cho từng kịch bản.** `config/default.yaml` là cấu hình rig thật. Mỗi kịch bản E-xx khai `pipeline_config: config/sim/<world>.yaml`, là file overlay chỉ chứa khoá khác với default (extrinsics của robot sim, GNSS/odom bật, nhiễu). Harness tích hợp (ADR-009) ghép overlay lên default; E-04 và E-07 chạy hai lần bằng hai overlay bật/tắt `degeneracy.enabled` hoặc `loop_closure.enabled`. Script sinh SDF đọc extrinsics từ overlay của world đó, không từ default.

### 6. Bag và ground truth

- Topic: `/lidar/points` (PointCloud2: x, y, z, intensity float32, t uint32, ring uint16), `/imu/data`, `/gnss/fix`, `/wheel/odom` (nav_msgs/Odometry, chỉ twist có nghĩa), `/ground_truth` (nav_msgs/Odometry, 200 Hz), `/tf_static` (`T_b_l`, `p_b_g`), `/clock`.
- Định dạng MCAP, nén zstd. Giới hạn 100 MB mỗi file; LiDAR 16 × 720 ở 10 Hz khoảng 170 MB thô mỗi 60 s nên kể cả E-01 cũng tách file theo kích thước (`--max-bag-size`), `tools/fetch_test_data.sh` tải cả bộ theo manifest. Không giảm cấu hình LiDAR để vừa 100 MB, vì đổi độ phân giải là đổi baseline ATE.
- Ground truth xuất thêm thành TUM (`gt.tum`) bằng `sim/mss_sim/gt_to_tum.py` để `eval` dùng trực tiếp; quaternion viết theo thứ tự TUM (x, y, z, w).
- Lệnh một dòng: `tools/record_scenario.sh sim/scenarios/E-01.yaml results/<run-id>/` chạy Gazebo headless, driver, bridge, ghi bag, xuất TUM, chạy báo cáo nhiễu, rồi tắt. Đóng gói bản phát hành `testdata-v1` kèm SHA256.

### 7. Báo cáo nhiễu trong `eval` (`python -m mss_eval.noise_report <bag-dir> <noise.yaml> --out <dir>`)

Một trang HTML tĩnh tự chứa, mỗi cảm biến một mục, luôn đặt "khai báo" cạnh "đo được", có bảng pass/fail:

| Cảm biến | Đồ thị | Tiêu chí pass |
| --- | --- | --- |
| IMU | Allan deviation gyro và accel (3 trục) đè đường lý thuyết từ N, K; histogram mẫu đứng yên | Trên kịch bản tĩnh `SIM-01_static_2h` (ghi riêng trên máy bàn, báo cáo đính vào release testdata): lệch < 10% trên τ ∈ [0.01, 100] s. Trên E-xx: chỉ kiểm σ trắng từ 5 s đứng yên đầu, ±15%; Allan chỉ hiển thị, không gate |
| GNSS | sai số vị trí theo thời gian tô vùng outage/multipath; histogram sai số với Gauss khai báo; tỷ lệ fix bị chi-square loại | σ đo được trong ±15% σ khai báo ở đoạn nominal |
| LiDAR | histogram khoảng cách điểm tới các mặt phẳng tham chiếu của world (tường có toạ độ biết trước); tỷ lệ điểm rớt; độ méo khi quay (so có/không skew) | σ trong ±15%; tỷ lệ rớt trong ±20% tương đối |
| Wheel odom | tích phân odom thuần so với ground truth; trôi theo mét và theo góc | trong khoảng mục tiêu ở mục 3 |
| Vật lý | vận tốc lệnh so với vận tốc thật; gia tốc thật | không vượt giới hạn khai báo; có trễ > 0 |
| Bag | timestamp tăng dần từng topic, có `/ground_truth`, khoảng trống tối đa giữa hai message | SIM-02 |

Báo cáo đọc bag bằng `rosbags`, không cần ROS. Khi chạy trong `tools/record_scenario.sh`, exit code khác 0 nếu có mục fail, để bag sai nhiễu không bao giờ được đóng gói.

### 8. Test bổ sung cho `sim/` (ngoài SIM-01, SIM-02 đã có)

| ID | Test | Tiêu chí |
| --- | --- | --- |
| SIM-03 | Tham số nhất quán giữa sim và pipeline | So overlay `config/sim/<world>.yaml` (sau khi ghép lên default) với `sim/noise/<profile>.yaml`: (a) mỗi tham số nhiễu IMU/GNSS/LiDAR của pipeline phải nằm trong `[1, inflation_max] ×` giá trị sim, `inflation_max` khai trong profile (mặc định 5), vì pipeline được phép tin nhiễu phóng đại nhưng không được tin nhiễu nhỏ hơn thật; (b) extrinsics `T_b_l`, `p_b_g` và động học odom của pipeline phải bằng `kinematics_believed` của profile, không phải giá trị thật. Test pytest, fail CI nếu lệch |
| SIM-04 | Tiêm skew đúng | robot đứng yên: cloud không đổi; robot quay ω hằng: điểm cuối scan lệch đúng ω·T_scan so với giải tích, sai số < 1 mm |
| SIM-05 | Tất định | cùng seed, hai lần chạy `sim_bridge` offline trên cùng input cho output giống hệt byte |
| SIM-06 | GNSS degrader | thời đoạn outage không có message; multipath: bias đúng biên độ, covariance báo cáo không đổi |

SIM-03..SIM-06 là test thuần Python trên dữ liệu tổng hợp, chạy trong job `python` của CI, không cần Gazebo.

### 9. Phụ thuộc mới

| Phụ thuộc | Ở đâu | Lý do |
| --- | --- | --- |
| `ros_gz_sim`, `ros_gz_bridge` (apt, Jazzy) | máy bàn | cầu nối Gazebo ↔ ROS 2 |
| `rclpy` | `sim/` | node bridge và driver; không bao giờ trong `core/` hay `eval/` |
| `rosbags` | `eval/`, `tools/` | đọc và ghi MCAP không cần ROS |
| `allantools` | `eval/` | Allan deviation |
| `jinja2`, `pyyaml`, `shapely` | `tools/`, `sim/` | sinh SDF từ template; đọc kịch bản; vùng đa giác GNSS |

Gazebo và `ros_gz` không vào image deps của CI (ADR-002): CI chỉ dùng bag đã ghi. Thêm image `docker/sim.Dockerfile` riêng cho máy bàn, tuỳ chọn.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chỉ dùng nhiễu có sẵn trong sensor Gazebo | Không viết thêm gì | Không có skew LiDAR, outage, multipath, trôi chậm, lượng tử hoá; E-05, E-06, P-01 không kiểm được |
| Viết plugin Gazebo C++ cho mọi nhiễu | Chạy trong tiến trình sim, không qua ROS | Khó test đơn vị, build chậm, đổi mô hình nhiễu phải biên dịch lại; Python đủ nhanh vì nhiễu tính ngoài vòng real-time |
| Bộ mô phỏng tổng hợp thuần Python, không Gazebo | Rất nhanh, tất định tuyệt đối | Không có vật lý, va chạm, bề mặt, trượt bánh; vẫn dùng dạng này cho dữ liệu tổng hợp trong unit/component test |
| Isaac Sim hoặc CARLA | Hình ảnh và cảm biến đẹp | Nặng cho GPU 8 GB và RAM 16 GB; không native ROS 2 Jazzy; không cần camera ở giai đoạn này |
| Skew LiDAR bằng nhiều sensor sector quay | Skew "thật" từ render | Phức tạp, dễ sai pha, tốn GPU; tiêm từ ground truth cho cùng kết quả hình học và kiểm chứng được giải tích |
| Dùng dataset công khai thay sim | Nhiễu thật | Không có bộ nào đủ 4 cảm biến với nhiễu điều khiển được; vẫn dùng UrbanNav cho PF-04 |

## Hệ quả

- Mọi kịch bản E-xx tái lập từ một file YAML và một seed; bag là sản phẩm phái sinh, không phải nguồn.
- Nhiễu "khai báo" và "đo được" luôn đi cùng nhau trong báo cáo, và SIM-03 buộc `config/default.yaml` khớp với sim, nên pipeline không bao giờ chạy với giả định nhiễu lệch thực tế mà không ai biết.
- Đổi lại: thêm một lớp Python giữa Gazebo và bag, tức thêm độ trễ ghi (không ảnh hưởng vì bag theo sim time) và thêm 6 test phải bảo trì. Bias Gauss-Markov của Gazebo chỉ xấp xỉ random walk trong dải τ hữu hạn.
- Khi có model LiDAR và MPU thật (việc còn mở trong `spec.md`), chỉ cần thêm profile `sim/noise/<model>.yaml`; không đổi kiến trúc.

## Để lại cho builder

- Giá trị cụ thể của `WheelSlip`, `damping`, moment động cơ: chỉnh cho đạt mục tiêu trôi odom ở mục 3, ghi giá trị cuối vào profile.
- Hình học chi tiết của từng world, miễn giữ đặc điểm trong bảng mục 5 và các mặt phẳng tham chiếu có toạ độ ghi trong file world để báo cáo LiDAR dùng.
- Bố cục HTML của báo cáo nhiễu; dùng cùng token màu với `docs/architecture.html`.
- Cách tách file bag và đặt tên khi kịch bản vượt 100 MB.

## Ghi chú 2026-10-27 (UTC): phần B, PR đầu (Gazebo: robot, kho, hành lang, driver, ghi bag)

**Đã giao trong PR này.** `sim/models/robot.sdf.j2` + `sim/models/robot.yaml` sinh bằng `tools/gen_sim_assets.py` (mục 2, 4); world `warehouse` và `corridor` viết tay kèm sidecar `<world>.planes.yaml` (mục 5); `sim/mss_sim/pursuit.py` + `driver.py`, quỹ đạo `warehouse_lap_60s` và `corridor_60s`, kịch bản E-04; `sim/launch/sim.launch.py`; `tools/record_scenario.sh` (mục 6); `tools/ci/check_sim_assets.sh` trong job `python` của CI (mục 9: CI không chạy Gazebo); `docker/sim.Dockerfile` (tuỳ chọn, không build trong CI). E-01 và E-04 đã ghi trên máy bàn (WSL, gz-sim 8.15, ogre2 headless, hệ số thời gian thực ≈ 2.0); E-04 đạt mọi dòng có gate của báo cáo nhiễu, E-01 cũng đạt mọi dòng có gate sau khi hiệu chuẩn sai số render LiDAR theo quỹ đạo (xem sai khác 1); con số và SHA256 trong `sim/README.md` mục "Máy bàn" (tạm thời, đến khi đóng gói `testdata-v1`). Harness tích hợp chạy được trên bag Gazebo (`tools/run_integration.py --bridged`): E-01 ATE 0.0955 m.

**Để cho PR sau.** World `outdoor` và `loop` (E-05..E-07) cùng giá trị `WheelSlip` cho cỏ và vũng ướt (plugin chỉ có thể thêm trượt; trên sàn kho để 0); hiệu chuẩn sai số render LiDAR cho các world mới (sai khác 1); đóng gói `testdata-v1`. Việc của pipeline (S2), có số liệu chẩn đoán trong `sim/README.md`: E-04 bật degeneracy trôi theo phương đứng (ATE 2.13 m, gần như toàn bộ theo z; −3.7 m trung bình trong 10 s đầu); E-01 chỉ LiDAR + IMU lệch đứng +0.10 m trong 10 s đầu rồi giảm còn 0.055 m RMS sau 50 s; bench chưa xuất bộ đếm và hướng suy biến (log chỉ báo 47/53 keyframe suy biến).

**Sai khác so với thiết kế (quyết định, lý do, hệ quả).**

1. *Sai số render LiDAR được hiệu chuẩn theo world và quỹ đạo, không bù trong SDF.* σ khai báo (`lidar.range_sigma_m`) là phần lớp nhiễu thêm vào: nhiễu Gazebo bằng đúng giá trị này. Renderer (`gpu_lidar`, nội suy texture độ sâu) thêm sai số riêng phụ thuộc tầm, góc tới và hình học robot thực sự thấy; không công thức nào mô tả được (hằng số, k·r, k·(1/|n·u| − 1), k·tanθ đều để lại phần dư ≈ 0.01 m so với sai số chuẩn ≤ 1e-3 m của các khoảng), nên nó được đo ở nơi dùng: bảng `sim/noise/render/<world>.yaml` (có trong repo) chứa σ_render cho từng ô (khoảng tầm × khoảng góc tới), kèm world, kịch bản, thời điểm UTC, phiên bản Gazebo, render engine, khối LiDAR của `robot.yaml`, σ khai báo, SHA256 bag thô và số điểm mỗi ô. Bảng là một tính chất đo được của `gpu_lidar` trên world và quỹ đạo đó, không phải giá trị chỉnh; chạy lại khi world, quỹ đạo, cảm biến, render engine hoặc phiên bản Gazebo đổi. Quy trình (`sim/README.md` "LiDAR render error"): `tools/calibrate_lidar_render.sh <kịch bản> results/<run-id>/` ghi kịch bản một lần với nhiễu LiDAR Gazebo = 0 (cùng seed, robot, driver) và đo trên bag thô (không có nhiễu của bridge; Gazebo render cả đám mây tại một thời điểm); script chỉ ghi vào `results/`, bảng được chép vào `sim/noise/render/`. Cách đo giống hệt ở hiệu chuẩn và báo cáo: chia ô theo tầm không nhiễu tới giao điểm của tia với mặt phẳng (chia theo tầm đo sẽ chọn điểm theo chính sai số của nó ở mép khoảng); loại điểm ở đường viền (láng giềng trên lưới cách mặt phẳng của điểm hơn `lidar.edge_jump_m` = 0.10 m = 5σ khai báo, không phản hồi, bị rớt hoặc dưới tầm tối thiểu) và điểm có |n·u| < `lidar.min_incidence_cos` = 0.3; cắt ngoại lai ở 5 × σ của ô (báo cáo: σ kỳ vọng; hiệu chuẩn: từ 0.10 m, lặp một lần). Báo cáo kỳ vọng `sqrt(σ_khai_báo² + σ_render(ô)²)`, gate ±15% từng ô có ≥ 1000 điểm; ô không có trong bảng chỉ hiển thị; không có bảng thì σ_render = 0 và mọi ô đều gate; một dòng trượt khi không ô nào được gate; bảng được kiểm khi nạp (không rỗng, đúng world của sidecar, đúng khoảng, ô trên lưới, n > 0, σ hữu hạn ≥ 0). Hiệu chuẩn 2026-10-27 UTC trên quỹ đạo E-01 (kho, 6 ô, σ_render 0.0082–0.0522 m) và E-04 (hành lang, 14 ô, 0.0008–0.0310 m). Bag hiệu chuẩn và bag được kiểm là hai lần chạy Gazebo riêng của cùng kịch bản (Gazebo không tất định). Không có lỗi thời gian: lần chạy di chuyển và đứng yên với nhiễu 0 cho cùng σ, quét lệch stamp LiDAR −6..+6 ms cho cực tiểu tại 0. Hệ quả: trên bag E-01 và E-04 đã ghi, mọi ô có gate đều đạt (E-01 6 ô, E-04 14 ô; lệch lớn nhất +4.7% ở hành lang 0.5–2 m × 0.3–0.5, ô này lệch −14% khi còn chia theo tầm đo), cả hai thoát mã 0.
2. *Báo cáo nhiễu: tỷ lệ góc tới lớn g tính từ pháp tuyến láng giềng trên mọi bề mặt* (mục 7), không từ cos với mặt phẳng tham chiếu gần nhất. Lý do: trong kho có kệ, thùng, cột, điểm không nằm trên mặt phẳng tham chiếu nhận cos tuỳ ý (g = 0.63 thay vì ≈ 0.13, kỳ vọng 0.197 so với đo 0.057). Điểm bị rớt được nội suy lại theo vòng quét trước khi tính pháp tuyến. Điểm có góc tới không xác định (láng giềng không có phản hồi) tính là không nghiêng, như bridge. Hệ quả: báo cáo và mô hình rớt điểm của bridge dùng cùng cách ước lượng; kho: trước sửa kỳ vọng 0.1967 (g = 0.627), sau sửa 0.0587 (g = 0.135), đo 0.0568; dữ liệu tổng hợp: trước 0.0210 (g = 0), sau 0.0228 (g = 0.0063), đo 0.0221.
3. *`/tf_static` phát bằng một node (`sim/mss_sim/tf_static.py`), một message chứa cả `T_b_l` và `p_b_g`*, không dùng hai `static_transform_publisher`. Lý do: hai publisher cho hai message cùng stamp, mục Bag của báo cáo đòi stamp tăng ngặt. Hệ quả: `tf_static.yaml` sinh ra chứa danh sách transform thay vì đối số dòng lệnh.
4. *Tham số robot không có trong profile (hình học, khối lượng, giới hạn, ma sát khớp, placeholder slip, cấu hình LiDAR) nằm ở `sim/models/robot.yaml`*; extrinsics và tên khung lấy từ overlay của kịch bản ghép lên `config/default.yaml` (đúng mục 5), không lấy từ default. Thêm giới hạn gia tốc góc 1.0 rad/s² cho `DiffDrive`, ADR không quy định.
5. *Ghi bag.* Bag thô gồm `/lidar/points`, `/imu/data`, `/gnss/fix`, `/ground_truth`, `/tf_static`, `/clock`, `/joint_states` (khớp bánh, mỗi bước vật lý = 1 kHz, giữ nguyên trong bag thô), log stamp theo sim time. Bridge đọc encoder ở `odom.rate_hz` = 50 Hz (khoá mới của profile): góc bánh tích phân giữa các joint state, số xung đọc mỗi cửa sổ 20 ms, nên `/wheel/odom` 50 Hz như nguồn tổng hợp và lượng tử hoá là một xung mỗi cửa sổ; trước đó mỗi joint state 1 kHz cho một message, nhiễu lượng tử ≈ 0.044 m/s, lớn hơn σ khai báo 0.01 và σ pipeline tin 0.02 m/s. Recorder chỉ bắt đầu khi có `/clock` mà world đang dừng không phát `/clock`, nên script khởi động Gazebo ở trạng thái dừng, bước từng 1 ms đến khi recorder đã subscribe, rồi mới chạy; tới `duration_s` + 2 s thì dừng world trước khi dừng recorder (nén file có thể mất vài giây). Tắt tiến trình bằng SIGTERM (job nền của shell không tương tác bỏ qua SIGINT).
6. *Hệ toạ độ world = ENU*: `<spherical_coordinates>` của world phải bằng `origin_lla` của kịch bản (có test); sidecar ghi `enu_origin_w_m: [0, 0, 0]`.
7. *Kịch bản*: thêm khoá tuỳ chọn `pipeline_config_variants` (E-04: overlay `corridor.yaml` tắt degeneracy và biến thể `degeneracy_on`), SIM-03 kiểm cả biến thể; quỹ đạo được nạp và kiểm khi đọc kịch bản. E-04 dài 70 s vì 51 m ở 0.9 m/s cộng 5 s đứng yên mất ≈ 66 s.
8. *Seed*: driver dùng luồng thứ sáu `SeedSequence(seed).spawn(6)[5]`; năm luồng đầu của bridge không đổi. Gazebo nhận `--seed` bằng seed kịch bản (bias IMU ban đầu), nhưng vẫn không tất định (mục 1).
9. *Tiếp xúc bánh xe là hình cầu* (`robot.yaml` `wheel.collision_shape: sphere`, hình hiển thị vẫn là trụ). Lý do: hiệu chuẩn odom với động học thật (loại sai số động học "tin"), encoder 50 Hz, so với ground truth: đi thẳng 10 m ở 0.5 m/s −0.04%, quay tại chỗ 2 vòng ở 0.5 rad/s −36.25°/vòng với bánh trụ rộng 0.04 m; bánh trụ rộng 0.01 m cho −9.25°/vòng; bánh cầu bán kính bánh cho −0.01% và −0.08°/vòng. Bánh trụ chạm sàn ở một mép, khoảng cách bánh hiệu dụng ≈ 0.36 m thay vì 0.40 m. Đây là nguyên nhân 12.5%/31°/vòng của lần ghi đầu, không phải `WheelSlip` (độ đàn hồi 0 không thêm trượt; plugin chỉ có thể thêm trượt). Hệ quả: E-01 với động học tin đo 1.67% quãng đường và 4.49°/vòng, E-04 0.92% (bag ghi cuối): đạt mục tiêu mục 3; với động học thật các đoạn thẳng 0.00% và khúc quay +0.00°/vòng trên vòng E-01.

## Sửa đổi 2026-10-29: lớp nhiễu online cho chạy live (lát cắt B3)

Mục 1 đã định `sim_bridge` "được bọc thành node live chỉ để xem trực tiếp". Sửa đổi này chốt cách bọc.

**Quyết định.**

1. *Dùng lại đúng mô hình nhiễu của bridge offline.* Phần xử lý từng message của `run_bridge` tách thành lớp `NoiseLayer` (`sim/mss_sim/bridge.py`: `imu`, `lidar`, `gnss`, `joints`); `run_bridge` gọi nó theo thứ tự log time như trước, nên bag offline giữ nguyên từng byte (SIM-05 vẫn đạt). `sim/mss_sim/live.py` (`LiveNoise`, không ROS) nhận message thô dạng CDR, gọi cùng `NoiseLayer` và trả CDR đã nhiễu; `sim/mss_sim/live_node.py` (rclpy) chỉ nối dây: subscription `raw=True`, publish bytes, không chuyển kiểu message, nên không có bản sao thứ hai của mô hình nào.
2. *Tính tất định.* Bridge offline vẫn là tham chiếu cho test và baseline. Live chỉ để xem: seed cố định (của kịch bản, hoặc `seed:=`), nhưng Gazebo không tất định (mục 1) và message đến theo thời gian thực. Mỗi cảm biến có luồng rng riêng, nên kết quả chỉ phụ thuộc thứ tự message của chính cảm biến đó: cho cùng chuỗi message thô và cùng seed, `LiveNoise` cho đúng các byte của bag offline (`tests/python/test_live.py`, cả khi các cảm biến đến xen kẽ khác và `/tf_static` đến trễ).
3. *Ground truth tăng dần.* Skew cần chuyển động trong lúc quét, nên một scan được giữ đến khi `/ground_truth` phủ `[stamp, stamp + T_scan]` (trễ ≈ 0,1 s sim), rồi xử lý trên cửa sổ 2 s ground truth gần nhất. Nội suy theo từng khoảng (vị trí tuyến tính, slerp với khoá ns nguyên), nên cửa sổ cho đúng số của toàn quỹ đạo. `t0` của các đoạn GNSS là stamp ground truth đầu tiên, `T_b_l` và tên frame thân đến từ `/tf_static` (latched), như offline; GNSS và joint state đến trước đó nằm trong hàng đợi có giới hạn (đếm khi tràn).
4. *Topic.* Gazebo phát topic thô dưới `/sim/raw` (`sim.launch.py raw_prefix:=/sim/raw`: `/lidar/points`, `/imu/data`, `/gnss/fix`, `/joint_states`, vì ba topic đầu trùng tên với topic đã nhiễu); node nhiễu phát `/lidar/points`, `/imu/data`, `/gnss/fix`, `/wheel/odom` đúng như bag đã qua bridge. `/ground_truth` và `/tf_static` không phát lại (đã có trên bus). Ghi bag (`record_scenario.sh`) không đổi: `raw_prefix` mặc định rỗng.
5. *Đồng hồ.* Mọi node `use_sim_time` theo `/clock` của Gazebo; dữ liệu vẫn mang stamp của message.
6. *Khởi động và hệ số thời gian thực.* World có `real_time_factor` 0 (không khoá) cho ghi bag. Live khởi động Gazebo ở trạng thái dừng; `sim/mss_sim/live_start.py` chờ mọi consumer subscribe (mss_node và node nhiễu, để không mất message đầu), đặt hệ số thời gian thực (`/world/<w>/set_physics`, `rtf:=`, mặc định 1,0), bước từng 1 ms đến khi có ground truth (robot đã spawn), chạy, rồi dừng world ở `duration_s` + 2 s sim và log hệ số đo được.

**Đo trên máy bàn** (WSL, 12 lõi, 10 GB RAM, gz-sim 8.15, ogre2; E-01 trọn 61 s sim, 2026-10-29): `rtf` 1,0 giữ được, đo 0,983 không GUI, 0,934 khi có GUI Gazebo và RViz; node không mất message nào trên cả bốn topic (nhận đúng số node nhiễu phát); ATE `/mss/path` so với `/ground_truth` 0,097 m (không GUI) và 0,102 m (có GUI), cùng cỡ bench trên bag Gazebo (0,095 m). Số chi tiết, CPU và RAM: `sim/README.md` mục "Live".

**Hệ quả.** Một lệnh `ros2 launch mss_ros2 mss_live.launch.py scenario:=sim/scenarios/E-01.yaml rviz:=true` chạy toàn bộ chuỗi trực tiếp. Live không thay thế bag: số dùng để so sánh và baseline vẫn lấy từ bag và bridge offline.
