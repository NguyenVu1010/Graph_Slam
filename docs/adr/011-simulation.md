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
- Sai số hiệu chuẩn có chủ đích: odom tính bằng bán kính bánh và khoảng cách hai bánh mà "robot tin" (`kinematics_believed`), lệch so với giá trị thật của SDF (`kinematics_true`); mặc định bán kính +1.0%, khoảng cách bánh −1.5%. Đây là nguồn trôi chính trên sàn khô, vì trượt vật lý thuần với µ = 0.8 và a ≤ 0.5 m/s² rất nhỏ; trượt bánh chỉ cần đủ để trôi tăng rõ trên cỏ và vũng ướt.
- Mục tiêu đo được (kiểm tra bằng báo cáo nhiễu, builder chỉnh plugin cho đạt): trôi odom thuần 1–3% quãng đường trên sàn kho, 3–6% trên cỏ; sai số yaw 0.3–1.0° mỗi vòng quay tại chỗ.

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
