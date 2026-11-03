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
| `/mss/save` | `std_srvs/Trigger` | ghi `kf_opt.tum` (mọi keyframe của snapshot, ba dòng chú thích `#` nói rõ), `loops.csv` (`i,j,t_i_s,t_j_s,state`), `map.pcd` (`export_map()`, `io::write_pcd`) vào `<viz.save_dir>/<run id>/save-<n>/` (mỗi lần lưu một thư mục mới, tạo bằng `create_directory` không đệ quy nên không bao giờ dùng lại thư mục đã có, kể cả sau khi khởi động lại với cùng `viz.run_id`; review N2-3), trả về thư mục | |
| `/mss/pause` | `std_srvs/SetBool` | `true` dừng, `false` chạy tiếp cả thế giới Gazebo | |
| `/mss/paused` | `std_msgs/Bool` | trạng thái thế giới node suy ra từ đồng hồ mô phỏng (xem dưới), phát khi đổi | reliable, transient local, sâu 1 |

- Timer tường `viz.rate_hz` (1 Hz) gọi `graph_snapshot()` và `stats()`; timer tường để RViz vẫn được vẽ lại khi thế giới dừng và `/clock` đứng. Chuyển đổi snapshot sang message nằm trong `conversions.cpp` (hàm thuần, có test). Kích thước và màu marker là hằng hiển thị trong code (`VizStyle`, `loop_color`), không phải tham số YAML: chúng không ảnh hưởng ước lượng, có đơn vị trong tên; đây là ngoại lệ có chủ ý với quy tắc cấu hình của CLAUDE.md (review N-2).
- Executor vẫn một luồng; service chạy trong callback, giữa hai lần `run_once()`, nên không cần khoá (ADR-024 mục 4).
- **Không có lệnh `gz` nào chặn executor** (review M-2): tiến trình `gz` được khởi động không chờ (`posix_spawnp`, ống `pipe2(O_CLOEXEC)`, chỉ đầu đọc của node đặt `O_NONBLOCK` để stdout của tiến trình con vẫn chặn; mọi mô tả tệp khác của node được đóng trong tiến trình con bằng `posix_spawn_file_actions_addclosefrom_np`), một timer tường mỗi 0,05 s kiểm tra nó (`waitpid(WNOHANG)`), và service trả lời sau bằng phản hồi trì hoãn của rclcpp (`create_service` với `request_header`, rồi `send_response`). Trong lúc chờ, executor vẫn nhận IMU, scan và các yêu cầu khác; mỗi lúc chỉ một yêu cầu tới thế giới, yêu cầu thứ hai nhận ngay `success=false` ("busy"). Các lời gọi `Estimator` vẫn ở luồng executor.
- **Trạng thái thế giới lấy từ đồng hồ mô phỏng, không từ những gì node đã yêu cầu** (review M-1): cùng timer tường đó (0,05 s, không phụ thuộc `viz.paused_after_s`, review N2-7) lấy mẫu đồng hồ ROS của node (thời gian mô phỏng từ `/clock`). Thế giới "chạy" khi đồng hồ đổi trong `viz.paused_after_s` gần nhất (0,5 s; trong [0,2; 5] s) và lần tiến gần nhất có hệ số thời gian thực ít nhất `viz.running_min_rtf` (0,05; trong (0; 1]); nếu không là "dừng" (các bước 1 ms của `live_start` lúc khởi động, khoảng 0,01 lần, cũng tính là dừng). Trạng thái này phát trên `/mss/paused` và `joy_commands` dùng nó.
- **Lệnh `gz` của chính node có hiệu lực ngay** (review M-3): khi `pause: true` (của `/mss/pause` hay của lần lưu) thành công, trạng thái là "dừng" ngay và giữ đến khi đồng hồ, lấy mẫu sau đó, tiến liên tục (mỗi lần đổi ≥ `running_min_rtf`) suốt trọn `viz.paused_after_s` đồng hồ tường, như khi thế giới được chạy tiếp từ ngoài; các mẫu `/clock` còn sót hay đến trễ sau lệnh dừng, dù trễ bao nhiêu, chỉ trải một khoảng ngắn nên không xoá nó, và một lần đổi chậm hơn `running_min_rtf` bắt đầu lại việc đếm (review N3-1); khi `pause: false` thành công, trạng thái là "chạy" ngay và giữ đến khi đồng hồ xác nhận hoặc `viz.paused_after_s` trôi qua không đổi. `/mss/paused` được phát lại ngay. Nhờ vậy "dừng rồi lưu ngay" không chạy lại thế giới.
- **Cần `use_sim_time`** khi có `viz.gz_world` (review N2-1): không có nó, đồng hồ của node là đồng hồ hệ thống, mọi thế giới đều trông như đang chạy và `/mss/save` sẽ chạy lại một thế giới đã dừng từ GUI. Node từ chối khởi động (`std::invalid_argument` nêu cả hai tham số). Không có `/clock` khi có thế giới: node cảnh báo mỗi 10 s (review N2-2); khi đó thế giới được coi là "dừng", nên `/mss/save` ghi mà không gửi lệnh `gz` và nói "sim clock not advancing (world paused or stalled), world left as it is".
- **Tạm dừng = dừng cả thế giới Gazebo** (quyết định của chủ dự án): nếu robot vẫn chạy khi SLAM bỏ dữ liệu thì IMU có khoảng trống và scan matching bắt đầu từ tư thế cũ; repo chưa có định vị lại. Dừng thế giới giữ thời gian mô phỏng, cảm biến và SLAM nhất quán; hệ quả là robot không lái được khi dừng. Gọi qua CLI `gz service -s /world/<viz.gz_world>/control` như `sim/mss_sim/gz_cli.py` (không thêm `ros_gz_interfaces`), `posix_spawnp` không qua shell, tên thế giới chỉ gồm chữ, số, `_`, `-`, giới hạn thời gian `viz.gz_timeout_ms` + 2 s. Service trả lời sau khi `gz` trả về; lỗi thì `success=false` kèm lý do.
- `/mss/save` tự dừng thế giới khi nó đang chạy, ghi, rồi chạy lại, để không mất dữ liệu cảm biến trong lúc ghi. Nó chỉ chạy lại thế giới mà chính lần lưu đó đã dừng: một thế giới đã dừng sẵn (bằng `/mss/pause`, GUI Gazebo hay `gz service` từ shell) được để nguyên và không có lệnh `gz` nào được gửi. Không có thế giới (bag) hoặc dừng lỗi: vẫn ghi và nói rõ trong `message`. Mỗi file ghi ra `.tmp` rồi đổi tên.

### 3. Gói ROS 2 mới `mss_teleop` (ament_python, thư mục gốc `mss_teleop/` cạnh `ros2_node/`)

Nút Start của `joy_commands` không tự nhớ trạng thái dừng: nó đọc `/mss/paused` của `mss_node` (cùng suy luận từ đồng hồ mô phỏng như `/mss/save`, mục 2), nên một lệnh dừng từ CLI, GUI hay `/mss/save` vẫn khớp, và `joy_commands` không phải nghe `/clock` 1 kHz (review N-7). Một lời gọi chưa có trả lời sau `call_timeout_s` (15 s) không còn chặn lần bấm sau (review N-6).

Giữ `ros2_node` là adapter mỏng của lõi; mọi thứ chỉ dành cho chế độ lái tay nằm trong gói riêng: `launch/teleop.launch.py` (`world:=warehouse|outdoor`, `input:=keyboard|joy`, `rviz`, `headless`, `rtf`, `repo`, `keyboard_prefix`), node `joy_commands` (nút -> `/mss/save`, `/mss/pause`), `config/joy.yaml`, `config/slam_viz.yaml` (tham số ROS thêm cho `mss_node`), `rviz/teleop.rviz`.

- Kịch bản và overlay dùng lại, không thêm file kịch bản: `warehouse` -> E-07W, `outdoor` -> E-07 (nhiễu, seed, GNSS, bánh xe, vị trí đặt); overlay SLAM là overlay `loops_on` của kịch bản đó, tham chiếu chứ không sao chép (một bản sao thứ ba sẽ trôi khỏi overlay đã đánh giá), cộng `viz.enabled` qua `mss_live.launch.py viz:=true`.
- `mss_live.launch.py` thêm `driver`, `stop_at_end`, `viz`, `node_params`, `rviz_config` (mặc định như cũ); `live_start` không có `--stop-at-s` thì thoát 0 ngay sau khi chạy thế giới, không canh đồng hồ (người dùng có thể dừng).
- Giới hạn vận tốc của cả hai đầu vào lấy từ `sim/models/robot.yaml` (`teleop_twist_joy` `scale_*` và `teleop_twist_keyboard` `speed`/`turn` đặt bằng vận tốc tối đa); plugin DiffDrive đã kẹp vận tốc và gia tốc theo cùng file, nên không thêm bộ làm mượt. Tay cầm có nút giữ an toàn (LB), không turbo.
- Bàn phím cần terminal riêng: `keyboard_prefix:="xterm -e"` mở nó, nếu không launch in lệnh cho terminal thứ hai (mặc định trên WSL, nơi tay cầm USB cần `usbipd`).

### 4. Phụ thuộc mới

| Phụ thuộc | Gói | Ghi chú |
| --- | --- | --- |
| `visualization_msgs`, `std_srvs`, `std_msgs` | `mss_ros2` (`<depend>`) | thuộc `ros_core`/`common_interfaces`, đã có trong ảnh `ros:jazzy-ros-base` của CI |
| `joy`, `teleop_twist_joy`, `teleop_twist_keyboard`, `std_msgs` | `mss_teleop` (`<exec_depend>`) | chỉ lúc chạy; ba gói đầu thêm vào `docker/sim.Dockerfile` (`std_msgs` có sẵn trong `ros_core`); CI chỉ build gói Python, không cần chúng |
| `rosgraph_msgs` | `mss_ros2` (`<test_depend>`) | test node phát `/clock`; có sẵn trong `ros_core` |
| `rviz2` | `docker/sim.Dockerfile` | đã là `<exec_depend>` của `mss_ros2`; ảnh sim trước đây không có, chế độ lái tay cần nó |

## Hệ quả

- Không đổi đường nóng, không đổi kết quả với cờ tắt (md5 `est.tum` E-01..E-03 bằng main); ATE và độ trễ: không đổi.
- Hiển thị chạy trên luồng executor như `export_map()` (backlog R8): O(keyframe) message mỗi giây; ở 1000 keyframe chưa đo.
- Di chuyển robot trong Gazebo lúc SLAM đang chạy (kéo bằng GUI, đặt lại tư thế) làm hỏng ước lượng: robot bị "bắt cóc" và repo không có định vị lại hay nhiều phiên bản đồ (backlog L25).
- `kf_opt.tum` của `/mss/save` có mọi keyframe; `kf_opt.tum` của `mss_bench` chỉ có keyframe còn trong kho (`visit_map`). Cùng pose cho keyframe chung. Tên file giữ theo tài liệu thiết kế; ba dòng chú thích `#` đầu file nói rõ khác biệt (bộ đọc TUM của `eval/` và evo bỏ qua dòng `#`).
- Hai danh sách loop của snapshot (`plain_loops_`, `removed_loops_`) được cập nhật cả khi cờ tắt: lõi không biết `viz.enabled` (tham số ROS); chỉ một phần tử thêm mỗi loop được nhận hay bị gỡ, ngoài đường nóng, không được ước lượng đọc (md5 không đổi). Thêm một cờ cấu hình lõi cho việc này sẽ thêm bề mặt cấu hình mà không đổi kết quả (review N-14).
- Lưu khi thế giới đang ở chuỗi khởi động của `live_start` (các bước 1 ms) được tính là "dừng", nên không chạy lại thế giới giữa chuỗi đó.
- Còn lại một cửa sổ: một lệnh dừng hay chạy tiếp từ GUI Gazebo hoặc từ `gz service` ngoài node chỉ được thấy qua đồng hồ, sau tối đa `viz.paused_after_s` (0,5 s). Một lần lưu trong cửa sổ đó sau khi GUI dừng thế giới sẽ gửi `pause: true` (không đổi gì) rồi `pause: false`, tức chạy lại thế giới; sau khi GUI chạy tiếp, lần lưu ghi trong lúc thế giới chạy. Lệnh qua `/mss/pause` hay nút Start không có cửa sổ này (review M-3).
- Thế giới chạy chậm hơn `viz.running_min_rtf` (ví dụ dựng hình phần mềm rất nặng) bị coi là "dừng": `/mss/save` không dừng nó và Start gửi `pause: false`. Hạ `viz.running_min_rtf` cho máy đó (review N2-2).
