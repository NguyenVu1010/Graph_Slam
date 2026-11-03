# ADR-031: Lái tay trong Gazebo và xem đồ thị SLAM trực tiếp: `Estimator::graph_snapshot`, hiển thị và hai service của `mss_node`, gói `mss_teleop`

- Trạng thái: đề xuất
- Ngày: 2026-11-04
- Sửa đổi: ADR-024 (thêm đầu ra và service của `mss_node`, chỉ khi `viz.enabled`), ADR-011 sửa đổi live (`mss_live.launch.py` và `live_start` có chế độ không dừng)
- Liên quan: ADR-003 (lớp), ADR-005/017 (một luồng), ADR-016 (pose keyframe đã tối ưu), ADR-026/028/030 (trạng thái loop), backlog R8, R9, L25

## Bối cảnh

Muốn biết SLAM làm gì hôm nay phải chạy một kịch bản rồi đọc file kết quả. Người dùng muốn tự lái robot mô phỏng (tay cầm hoặc bàn phím), đi lại chỗ cũ, qua lối đi lặp lại, rẽ gắt, và thấy ngay trên RViz2 keyframe mọc dần, loop được nhận hay bị PCM gỡ, quỹ đạo co lại sau mỗi lần đóng loop; bấm một nút để lưu kết quả. Thiết kế chi tiết và các quyết định của controller nằm trong tài liệu thiết kế của tính năng; ADR này ghi các lựa chọn kiến trúc và phụ thuộc mới.

Ràng buộc: `core/` không có ROS; header `core/pipeline/include/mss/pipeline/*.hpp` là hợp đồng (chủ dự án đồng ý phần chỉ thêm mô tả dưới); E-01..E-03 phải giữ nguyên `est.tum` (md5) khi tắt cờ; không đụng đường nóng.

## Quyết định

### 1. API lõi chỉ thêm: `Estimator::graph_snapshot(GraphSnapshot&) const`

Header mới `mss/pipeline/graph_snapshot.hpp` (cùng kiểu `map_export.hpp`): `GraphKeyframe {id, t, T_w_b}`, `LoopState {kActive, kReleased, kRemoved}`, `GraphLoop {i, j, t_i, t_j, state}`, `GraphSnapshot {keyframes, loops}`. Hàm xoá `out` rồi điền, giữ dung lượng giữa các lần gọi.

- Keyframe: mọi keyframe backend có ước lượng (`backend::Graph::keyframe_state`, KeyId liền từ 0), kể cả keyframe đã bị đẩy khỏi kho bản đồ (`map.max_keyframes`): iSAM2 giữ mọi biến, đọc rẻ.
- Loop: với PCM là `graph_loops_` của pipeline (đã có cờ `released` của ADR-030, nên `kReleased` rẻ và được giữ); không PCM là danh sách loop đã thêm, lọc bằng `Graph::has_loop_factor`; loop bị PCM gỡ (`n_loops_removed`) được ghi vào một danh sách riêng và liệt kê `kRemoved` nếu chưa quay lại. Hai danh sách này chỉ để snapshot đọc, không bao giờ được ước lượng đọc, nên không đổi kết quả nào. Loop bị bỏ cùng một lần cập nhật lỗi chưa từng vào graph, không được liệt kê.
- Hợp đồng luồng như `visit_map()`: gọi trên luồng gọi `run_once()`, giữa hai lần gọi. O(keyframe + loop) lần đọc backend, ngoài đường nóng (khoảng 1 Hz, theo yêu cầu). Bản build không GTSAM: lỗi "no keyframe has a backend pose", snapshot rỗng.

### 2. `mss_node`: cờ `viz.enabled` (mặc định `false`)

Tắt: node y như trước, không publisher, service hay tham số `viz.*` nào ngoài cờ (test `VizOffAddsNothing`). Bật:

| Tên | Kiểu | Nội dung | QoS |
| --- | --- | --- | --- |
| `/mss/graph/keyframes` | `visualization_msgs/MarkerArray` | `DELETEALL` rồi một mũi tên mỗi keyframe tại `T_w_b` đã tối ưu | reliable, transient local, sâu 1 |
| `/mss/graph/loops` | `visualization_msgs/MarkerArray` | `DELETEALL` rồi một đoạn `i - j` mỗi loop; xanh lá `kActive`, xanh lơ `kReleased`, đỏ `kRemoved` | như trên |
| `/mss/graph/path` | `nav_msgs/Path` | các keyframe đã tối ưu, vẽ lại cả đường (backlog R9) | như trên |
| `/mss/graph/counts` | `visualization_msgs/Marker` | chữ: số keyframe, loop nhận / từ chối / PCM từ chối / gỡ, loop theo trạng thái | như trên |
| `/mss/save` | `std_srvs/Trigger` | ghi `kf_opt.tum` (mọi keyframe của snapshot), `loops.csv` (`i,j,t_i_s,t_j_s,state`), `map.pcd` (`export_map()`, `io::write_pcd`) vào `<viz.save_dir>/<run id>/`, trả về thư mục | |
| `/mss/pause` | `std_srvs/SetBool` | `true` dừng, `false` chạy tiếp cả thế giới Gazebo | |

- Timer tường `viz.rate_hz` (1 Hz) gọi `graph_snapshot()` và `stats()`; timer tường để RViz vẫn được vẽ lại khi thế giới dừng và `/clock` đứng. Chuyển đổi snapshot sang message nằm trong `conversions.cpp` (hàm thuần, có test).
- Executor vẫn một luồng; service chạy trong callback, giữa hai lần `run_once()`, nên không cần khoá (ADR-024 mục 4).
- **Tạm dừng = dừng cả thế giới Gazebo** (quyết định của chủ dự án): nếu robot vẫn chạy khi SLAM bỏ dữ liệu thì IMU có khoảng trống và scan matching bắt đầu từ tư thế cũ; repo chưa có định vị lại. Dừng thế giới giữ thời gian mô phỏng, cảm biến và SLAM nhất quán; hệ quả là robot không lái được khi dừng. Gọi qua CLI `gz service -s /world/<viz.gz_world>/control` như `sim/mss_sim/gz_cli.py` (không thêm `ros_gz_interfaces`), `posix_spawnp` không qua shell, tên thế giới chỉ gồm chữ, số, `_`, `-`, giới hạn thời gian `viz.gz_timeout_ms` + 2 s. Service trả lời sau khi `gz` trả về; lỗi thì `success=false` kèm lý do.
- `/mss/save` tự dừng thế giới khi nó đang chạy, ghi, rồi chạy lại, để không mất dữ liệu cảm biến trong lúc ghi. Không có thế giới (bag) hoặc dừng lỗi: vẫn ghi và nói rõ trong `message`. Mỗi file ghi ra `.tmp` rồi đổi tên.

### 3. Gói ROS 2 mới `mss_teleop` (ament_python, thư mục gốc `mss_teleop/` cạnh `ros2_node/`)

Giữ `ros2_node` là adapter mỏng của lõi; mọi thứ chỉ dành cho chế độ lái tay nằm trong gói riêng: `launch/teleop.launch.py` (`world:=warehouse|outdoor`, `input:=keyboard|joy`, `rviz`, `headless`, `rtf`, `repo`, `keyboard_prefix`), node `joy_commands` (nút -> `/mss/save`, `/mss/pause`), `config/joy.yaml`, `config/slam_viz.yaml` (tham số ROS thêm cho `mss_node`), `rviz/teleop.rviz`.

- Kịch bản và overlay dùng lại, không thêm file kịch bản: `warehouse` -> E-07W, `outdoor` -> E-07 (nhiễu, seed, GNSS, bánh xe, vị trí đặt); overlay SLAM là overlay `loops_on` của kịch bản đó, tham chiếu chứ không sao chép (một bản sao thứ ba sẽ trôi khỏi overlay đã đánh giá), cộng `viz.enabled` qua `mss_live.launch.py viz:=true`.
- `mss_live.launch.py` thêm `driver`, `stop_at_end`, `viz`, `node_params`, `rviz_config` (mặc định như cũ); `live_start` không có `--stop-at-s` thì thoát 0 ngay sau khi chạy thế giới, không canh đồng hồ (người dùng có thể dừng).
- Giới hạn vận tốc của cả hai đầu vào lấy từ `sim/models/robot.yaml` (`teleop_twist_joy` `scale_*` và `teleop_twist_keyboard` `speed`/`turn` đặt bằng vận tốc tối đa); plugin DiffDrive đã kẹp vận tốc và gia tốc theo cùng file, nên không thêm bộ làm mượt. Tay cầm có nút giữ an toàn (LB), không turbo.
- Bàn phím cần terminal riêng: `keyboard_prefix:="xterm -e"` mở nó, nếu không launch in lệnh cho terminal thứ hai (mặc định trên WSL, nơi tay cầm USB cần `usbipd`).

### 4. Phụ thuộc mới

| Phụ thuộc | Gói | Ghi chú |
| --- | --- | --- |
| `visualization_msgs`, `std_srvs`, `std_msgs` | `mss_ros2` (`<depend>`) | thuộc `ros_core`/`common_interfaces`, đã có trong ảnh `ros:jazzy-ros-base` của CI |
| `joy`, `teleop_twist_joy`, `teleop_twist_keyboard` | `mss_teleop` (`<exec_depend>`) | chỉ lúc chạy; thêm vào `docker/sim.Dockerfile`; CI chỉ build gói Python, không cần chúng |

## Hệ quả

- Không đổi đường nóng, không đổi kết quả với cờ tắt (md5 `est.tum` E-01..E-03 bằng main); ATE và độ trễ: không đổi.
- Hiển thị chạy trên luồng executor như `export_map()` (backlog R8): O(keyframe) message mỗi giây; ở 1000 keyframe chưa đo.
- Di chuyển robot trong Gazebo lúc SLAM đang chạy (kéo bằng GUI, đặt lại tư thế) làm hỏng ước lượng: robot bị "bắt cóc" và repo không có định vị lại hay nhiều phiên bản đồ (backlog L25).
- `kf_opt.tum` của `/mss/save` có mọi keyframe; `kf_opt.tum` của `mss_bench` chỉ có keyframe còn trong kho (`visit_map`). Cùng pose cho keyframe chung.
