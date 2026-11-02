# ADR-029: Biến thể mô phỏng cầm tay: không bánh xe, chuyển động đi bộ 6 bậc tự do

- Trạng thái: đề xuất
- Ngày: 2026-11-02
- Liên quan: ADR-011 (mô phỏng), ADR-004 (odom là factor riêng sau cờ YAML), ADR-026 (Scan Context, `sc_height_offset_m`), backlog L1, C1-6

## Bối cảnh

Chủ dự án muốn chạy pipeline cả khi rig được cầm tay (backlog L1): không có encoder, LiDAR và IMU gắn cứng trên một tay cầm, người đi bộ. Mọi kịch bản hiện có dùng robot vi sai của ADR-011 mục 2: chuyển động phẳng, thân ở độ cao trục bánh (0,10 m), roll và pitch gần 0, odom bánh xe luôn có. Cầm tay khác ở ba điểm mà pipeline phải chịu:

1. Không có wheel odom: chỉ LiDAR + IMU (+ GNSS nếu có), nên mọi ràng buộc tịnh tiến ngắn hạn từ bánh xe biến mất.
2. Chuyển động 6 bậc tự do theo nhịp bước: thân nhấp nhô theo phương đứng ở tần số bước, lắc ngang và roll ở tần số sải (nửa tần số bước), pitch ở tần số bước. IMU thấy gia tốc đứng cỡ `A (2π f)²` (≈ 3,6 m/s² với A = 0,025 m, f = 1,9 Hz), lớn hơn hẳn robot (≤ 0,5 m/s²), và deskew phải xử lý quay roll/pitch trong lúc quét.
3. Độ cao thân so với mặt đất khoảng 1,2 m thay cho 0,10 m, nên khoá phụ thuộc độ cao (Scan Context `sc_height_offset_m`, C1-6) cần giá trị riêng.

Đây là một biến thể nền tảng mới, với mô hình chuyển động mới, nên cần ADR (CONTRIBUTING.md).

## Quyết định

1. **Biến thể là cấu hình, không phải nhánh mã.** Một robot mô tả `sim/models/handheld.yaml` (thông số dáng đi, độ cao, cảm biến) và, ở bước sau, một overlay pipeline `config/sim/handheld.yaml` với `wheel_odom.enabled: false` và `loop_closure.sc_height_offset_m` theo độ cao thân. Không có cờ `platform` trong lõi: lõi chỉ thấy cảm biến nào có dữ liệu.
2. **Mô hình chuyển động (`sim/mss_sim/walking.py`, không ROS, có test).** Đường đi ngang: cùng tuyến waypoint và cùng bộ pure-pursuit với robot (`mss_sim.pursuit`, đứng yên 5 s đầu, giới hạn tốc độ, gia tốc và tốc độ quay lấy từ `handheld.yaml`), tích phân như xe một bánh. Dao động bước cộng lên, biên độ nhân với `v / v_hành_trình` (đứng yên thì không dao động):
   - pha bước `φ(t) = 2π ∫ f_bước dt`; độ cao `z = h + A_z cos φ`;
   - lắc ngang theo trục y thân `A_y sin(φ/2)`; roll `A_roll sin(φ/2)`; pitch `A_pitch sin φ`; yaw cộng `A_yaw sin(φ/2)`;
   - xoay `R_w_b = Rz(ψ + δψ) Ry(pitch) Rx(roll)`; vận tốc và vận tốc góc lấy bằng sai phân trung tâm trên lưới 1 kHz rồi lấy mẫu lại, để bước sau sinh IMU nhất quán với tư thế.
   Giá trị mặc định là của thân người khi đi bình thường, thiết bị cầm tay coi như theo thân: tốc độ 1,3 m/s, nhịp 1,9 bước/s, dịch đứng của khối tâm ≈ 5 cm đỉnh–đỉnh, dịch ngang ≈ 4 cm, roll ±2°, pitch ±1,5°, yaw ±1,5° (Perry và Burnfield, *Gait Analysis*, 2010; Saunders, Inman và Eberhart, 1953; Bohannon, 1997). Thay bằng số đo từ một bản ghi thật của rig khi có.
3. **Gazebo (bước sau, không trong PR này).** Một model không bánh, link thân tắt trọng lực, chạy bằng hệ `VelocityControl` của gz-sim (twist 6 bậc tự do trong khung thân) với bộ bám quỹ đạo của `walking.py` (truyền thẳng + phản hồi từ `/ground_truth`); IMU, LiDAR và ground truth như robot. Trước khi dùng phải kiểm IMU của Gazebo vẫn đo lực riêng có trọng lực khi link tắt trọng lực; nếu không, IMU sinh từ quỹ đạo như `synth.py`. Kịch bản đầu tiên: tuyến `outdoor_slip` đi bộ, GNSS bật, wheel odom tắt.
4. **Không đổi gì của robot bánh xe.** E-01..E-07 và OUT-01 giữ nguyên byte.

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Đặt pose model mỗi bước bằng dịch vụ `set_pose` | Một lời gọi dịch vụ mỗi 1 ms, chậm và không có gia tốc liên tục cho IMU |
| Robot chân (mô phỏng dáng đi thật) | Nặng, khó điều khiển ổn định, ngoài phạm vi SLAM |
| Chỉ dữ liệu tổng hợp (`synth.py`) | Không có hình học world thật; vẫn là đường dự phòng cho IMU ở quyết định 3 |
| Dao động ngẫu nhiên thay vì tuần hoàn | Dáng đi là tuần hoàn; nhiễu ngẫu nhiên có thể cộng sau khi có số đo thật |

## Hệ quả

- PR này chỉ có ADR, `sim/models/handheld.yaml` và bộ sinh quỹ đạo có test; model Gazebo, overlay, kịch bản và số ATE là bước sau (backlog L1 giữ mở).
- Pipeline sẽ phải chịu gia tốc đứng vài m/s² theo nhịp bước và không có odom bánh xe; cần đo ATE và độ trễ khi có kịch bản.
- `sc_height_offset_m` (C1-6) cần giá trị cho độ cao 1,2 m; chủ dự án quyết định độ cao cầm tay và cách gắn LiDAR (`T_b_l`) của rig thật.
