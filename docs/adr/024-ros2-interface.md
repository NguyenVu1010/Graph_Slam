# ADR-024: Giao diện ROS 2 của `mss_node`: topic, frame và cây TF, một luồng executor, thời gian, lỗi

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Liên quan: ADR-003 (adapter ROS 2 mỏng), ADR-005 (luồng), ADR-008 (cấu hình YAML), ADR-016 (sửa đổi 2026-10-29: `Estimator::export_map`), ADR-017 (estimator đơn luồng)

## Bối cảnh

Đến PR này `mss_node` chỉ đọc `config_path`, cài logger rồi spin; pipeline chưa nối với topic nào. Lát cắt đầu-cuối B cần: phát lại một bag đã qua noise bridge (`sim/mss_sim/bridge.py`) vào node, và RViz thấy đường đi lớn dần, TF của robot và bản đồ đầy dần. Lõi không có ROS (ADR-003), nên mọi quyết định về message, frame ROS, QoS và executor nằm trong `ros2_node`. Không quy ước frame nào của lõi thay đổi.

`mss_bench` đã đọc cùng dữ liệu qua `tools/bag_to_mssdata.py`, nên ngữ nghĩa chuyển đổi đã có sẵn và node phải theo đúng nó, để ATE của node so được với ATE của bench.

## Quyết định

### 1. Đầu vào

| Topic (tham số ROS) | Kiểu | Sang | Ngữ nghĩa (như `bag_to_mssdata.py`) |
| --- | --- | --- | --- |
| `/lidar/points` (`topics.lidar`) | `sensor_msgs/PointCloud2` | `common::ScanFrame` | `t_start = header.stamp`; trường `x y z` float32 (m, khung `l`), `intensity` float32 (thiếu thì 0), `t` uint32 (ns tính từ `header.stamp`), `ring` uint16, đúng như bridge ghi (`CLOUD_FIELDS` trong `bag_io.py`). Đọc theo `offset` và `point_step` của chính message. Thiếu `t` hoặc `ring` là lỗi ("chạy bridge trước"). Điểm không hữu hạn được giữ, `preprocess` loại chúng như ở bench. |
| `/imu/data` (`topics.imu`) | `sensor_msgs/Imu` | `common::ImuSample` | `acc_b = linear_acceleration`, `gyro_b = angular_velocity` (khung `b`); orientation và covariance của message bị bỏ qua. |
| `/gnss/fix` (`topics.gnss`) | `sensor_msgs/NavSatFix` | `common::GnssMeasurement` | `p_e_g = gnss::lla_to_enu(lla, origin)`. Gốc ENU: tham số `gnss_origin.*` khi `gnss_origin.from_first_fix` là false, nếu không thì fix hợp lệ đầu tiên (`status >= 0`, lat/lon/alt hữu hạn), như `_gnss_records`. Covariance: `position_covariance` (ENU, hàng trước) khi kiểu khác `UNKNOWN`; `UNKNOWN` thì `diag(sigma_h², sigma_h², sigma_v²)` từ `gnss.sigma_h_m`, `gnss.sigma_v_m` của config. `status = status.status`. Fix trước khi có gốc bị bỏ và đếm. |
| `/wheel/odom` (`topics.odom`) | `nav_msgs/Odometry` | `common::WheelSample` | `v_mps = twist.twist.linear.x`, `omega_radps = twist.twist.angular.z`; pose của message bị bỏ qua. |

Tên topic là tham số ROS vì là chuyện nối dây của ROS, không phải tham số ước lượng (mục 7).

QoS đăng ký: best effort, keep last, độ sâu theo tham số (`qos_depth.*`; mặc định LiDAR 10, IMU 400 = 2 s ở 200 Hz như `realtime.imu_queue_depth`, GNSS và odom 50). Best effort khớp được cả publisher reliable lẫn best effort (driver thật, `ros2 bag play`); một subscription reliable sẽ không nối với driver best effort mà không báo gì. Độ sâu phải đủ cho khoảng thời gian một `run_once()` keyframe chiếm luồng (vài chục ms) vì message đến trong lúc đó nằm trong hàng đợi DDS.

### 2. Đầu ra

| Topic | Kiểu | Khi nào | QoS |
| --- | --- | --- | --- |
| `/mss/odometry` | `nav_msgs/Odometry` | mỗi ước lượng mới (`latest_state().state.t` đổi) | reliable, volatile, sâu 10 |
| `/mss/path` | `nav_msgs/Path` | như trên; thêm một pose rồi phát cả đường | reliable, transient local, sâu 1 |
| `/tf` | `map → base_link` | như trên | của `tf2_ros::TransformBroadcaster` |
| `/mss/map` | `sensor_msgs/PointCloud2` (`x y z` float32, khung `map`) | `Estimator::export_map()` ở tần số `map_rate_hz` (mặc định 1 Hz; 0 tắt) | reliable, transient local, sâu 1 |

- `Odometry.pose` là `T_w_b`; quaternion dựng từ `R_w_b` theo Hamilton (w, x, y, z), ghi sang trường ROS `x y z w`. Covariance: lõi cho `[rho, phi]` với nhiễu phải (`T <- T * Exp(delta)`, delta trong `b`); ROS hiểu covariance pose quanh trục của `header.frame_id`. Bậc nhất, `p' = p + R rho` và `R' = Exp(R phi) R`, nên `Sigma_ros = J Sigma J^T` với `J = blkdiag(R_w_b, R_w_b)`. Thứ tự `[x y z rotx roty rotz]` trùng `[rho, phi]`. Đây là phép đổi ở biên I/O, không đổi quy ước của lõi.
- `Odometry.twist.linear = R_w_b^T v_w_b` (twist của ROS ở `child_frame_id`); góc quay không được ước lượng, để 0; covariance twist để 0 (không biết).
- Đường đi là pose front-end lúc phát (ADR-005 mục hệ quả): các pose cũ không được sửa khi backend hiệu chỉnh. Bản đồ thì có: `export_map()` đọc pose keyframe đã tối ưu ở lúc gọi.
- Stamp của mọi đầu ra là `state.t` (đồng hồ cảm biến, tức thời gian của bag khi phát lại). Bản đồ mang stamp của ước lượng mới nhất.

### 3. Tên frame và cây TF

- `w` → `map` (tham số `tf.world_frame`), `b` → `base_link` (tham số `tf.body_frame`, là thân IMU). TF phát `map → base_link` từ `T_w_b`: `translation = p_w_b`, `rotation = q(R_w_b)` Hamilton. `T_w_b` ánh xạ điểm từ `b` sang `w`, đúng nghĩa transform `parent = map`, `child = base_link` của tf2, nên không cần đảo.
- **Không có frame `odom`.** REP-105 đặt `odom` giữa `map` và `base_link` để một nguồn odometry liên tục và một nguồn định vị có nhảy cùng tồn tại. Node là nguồn pose duy nhất và ước lượng của nó đã là pose trong `map`; một `odom` đồng nhất chỉ thêm một frame không mang thông tin, còn một `odom` thật (tích phân IMU/bánh xe tần số cao, liên tục) là việc của sau này. Khi có, cây thành `map → odom → base_link` với `T_map_odom = T_w_b * T_odom_b^-1`. Frame `odom` của `/wheel/odom` trong bag không được phát thành TF.
- LiDAR và ăng-ten: node không phát; chúng đến từ `/tf_static` của nguồn dữ liệu (bag hoặc robot). Bag Gazebo treo `lidar_link` dưới `frames.body` của config (`imu_link`); bag tổng hợp dưới `base_link`. Muốn cây liền trên bag Gazebo thì đặt `body_frame:=imu_link` khi launch (trên robot thật `base_link == imu_link`).

### 4. Luồng

- Node chạy trên `SingleThreadedExecutor`. Callback subscription chỉ chuyển đổi và gọi `push_*`. Một timer tường (`process_period_s`, mặc định 0,01 s) gọi `run_once()` rồi phát odometry, path, TF nếu có ước lượng mới. Một timer tường thứ hai (`1 / map_rate_hz`) gọi `export_map()` và phát `/mss/map`. Timer tường chứ không phải timer ROS: với `use_sim_time` timer ROS đứng khi `/clock` dừng, còn hàng đợi vẫn phải được xả.
- **Hôm nay không cần khoá**: mọi callback và timer chạy nối tiếp trên một luồng, nên đúng hợp đồng của `Estimator` (“mọi phương thức gọi từ một luồng”, ADR-017) và hợp đồng của `export_map()` (“gọi giữa hai lần `run_once()`, trên luồng đó”).
- **Khi các luồng của ADR-005 đến**: node gọi `start()`/`stop()` thay cho timer `run_once()`; `push_*` vẫn từ luồng executor (hàng đợi có giới hạn, một người ghi). `latest_state()` đọc `state_` dưới mutex của estimator. `export_map()` phải lấy khoá của chủ sở hữu map/backend, hoặc estimator trao một ảnh chụp; việc đó thuộc `pipeline`, không thuộc node (mục tồn đọng của A1). Executor của node vẫn một luồng.

### 5. Thời gian

- `Timestamp = sec * 1e9 + nanosec` từ `header.stamp` (int64 ns, đồng hồ cảm biến); không qua `double`. Ngược lại, `sec = floor(ns / 1e9)`.
- Phát lại bag: `use_sim_time:=true` và `ros2 bag play --clock`; dữ liệu vẫn được đánh thời gian bằng stamp của message, không bằng đồng hồ node.
- Mỗi topic giữ stamp lớn nhất đã nhận. Message có stamp không tăng bị bỏ, đếm, log có điều tiết (`time_jump.*`), giống `bag_to_mssdata.py` giữ bản ghi đầu khi trùng stamp. Lùi quá `time_jump_back_warn_s` (mặc định 1 s) thì log là nhảy thời gian hay lặp bag. Node không bao giờ abort; sau một vòng lặp bag, node bỏ qua dữ liệu cho đến khi stamp vượt mốc cũ (khởi động lại node để chạy lại).

### 6. Lỗi

- Mỗi `push_*` trả `Status` không ok, mỗi lỗi chuyển đổi, mỗi message bị bỏ vì thời gian đều được đếm theo loại và log cảnh báo có điều tiết (đồng hồ steady, `error_log_period_s`, mặc định 5 s), kèm lý do và tổng đếm. Không log trên đường nóng ngoài mức điều tiết đó.
- Lỗi khởi tạo (config không đọc được, tham số node sai) thì log và thoát mã 2, như `mss_bench`.
- Khi tắt, node gọi `flush()` và log các bộ đếm của estimator và của node.

### 7. Tham số

Tham số ước lượng vẫn chỉ ở `config/default.yaml` + overlay (ADR-008); node chỉ nhận đường dẫn (`config_path`, `config_overlay`). Tham số ROS của node chỉ là nối dây ROS: topic, frame, QoS, chu kỳ timer, tần số bản đồ, gốc ENU, điều tiết log. Chúng có mặc định trong một struct, đơn vị trong tên, và được kiểm tra khi khởi động với thông báo nêu tên tham số.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Gọi `run_once()` ngay trong callback scan | Trễ thấp hơn một chu kỳ timer | Callback IMU đến sau scan mới phủ được scan; timer gọn và cùng một chỗ phát. Chu kỳ 10 ms nhỏ so với 100 ms của scan |
| `MultiThreadedExecutor` | Callback không chờ `run_once()` | Vi phạm hợp đồng một luồng của `Estimator`, cần khoá mà pipeline chưa có |
| Phát `map → odom` đồng nhất + `odom → base_link` | Đúng dáng REP-105 | `odom` không mang thông tin; dễ khiến người dùng tưởng có odometry liên tục |
| Subscription reliable | Không mất message trên mạng | Không nối với publisher best effort; trên localhost best effort đủ (đo trong PR) |
| Gốc ENU luôn là fix đầu | Không cần tham số | Không so được với bench chạy theo gốc của kịch bản; giữ làm mặc định |

## Hệ quả

- `ros2 launch mss_ros2 mss_bag.launch.py bag:=<thư mục bag> rviz:=true` phát bag vào node. Đo trên bag E-01 tổng hợp (60 s, GTSAM, desktop WSL), tốc độ phát 1x và 2x: node nhận đủ 600/600 scan, 12000/12000 mẫu IMU, 300 fix, 2999 mẫu bánh xe với QoS best effort, không bỏ message nào; 579 pose phát ra, ATE 0,002184 m, bằng `mss_bench` trên cùng bag (cùng 579 pose, vị trí lệch tối đa 7e-9 m). Bag Gazebo E-01 (warehouse, 66,9 s): 669 scan, 647 pose, ATE 0,0952 m, bằng `mss_bench` (lệch 9e-9 m). Với `delay_s` 2 s, node mất giây đầu của bag Gazebo (chưa kịp subscribe), nên mặc định là 5 s.
- Đường đi phát lại toàn bộ mỗi lần có ước lượng: O(số pose); chạy dài cần giới hạn hoặc phát theo keyframe (tồn đọng).
- Bản build không GTSAM không có bản đồ (`export_map()` báo lỗi, node log có điều tiết).
- Thứ tự xử lý của node phụ thuộc thời điểm message đến so với timer, nên không tái lập bit-cho-bit như bench; `deterministic: true` vẫn là chế độ của bench và test.
