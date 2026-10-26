# ADR-005: Mô hình luồng và thời gian thực

- Trạng thái: đề xuất
- Ngày: 2026-10-23

## Bối cảnh

Ngân sách trên Pi5: p95 dưới 100 ms mỗi scan ở 10 Hz, CPU dưới 300% trên 4 nhân, RAM dưới 2 GB (PF-02), chạy 30 phút không tăng độ trễ (PF-03). IMU đến ở 200 Hz và không được rớt. Kết quả phải tái lập bit-for-bit để E-02 so hai lần chạy. Lõi không có ROS (ADR-003), nên không dùng executor của rclcpp.

## Quyết định

- **Chỉ `pipeline::Estimator` tạo luồng.** Module là code tuần tự, không biết luồng, không giữ mutex.
- Ba luồng cố định:

| Luồng | Việc | Đầu vào | Đầu ra |
| --- | --- | --- | --- |
| front-end | preprocess → scan_match → degeneracy → map.insert → quyết định keyframe | hàng đợi scan, bộ đệm IMU | keyframe + factor LiDAR |
| backend | gom factor (IMU preintegrated, LiDAR, GNSS, odom, loop) → iSAM2 update → cập nhật state | hàng đợi keyframe, hàng đợi GNSS/odom | `Estimate` mới nhất, pose keyframe đã hiệu chỉnh |
| loop | tìm và xác minh loop closure (giai đoạn 2, tắt mặc định) | keyframe đã có pose | `LoopConstraint` |

- `push_imu`, `push_scan`, `push_gnss`, `push_odom` chạy trên luồng của người gọi, chỉ sao chép vào hàng đợi và trả `Status` ngay; không bao giờ chặn.
- **Hàng đợi có giới hạn**, mỗi hàng đợi một chủ sở hữu ghi và một chủ sở hữu đọc:
  - scan: sâu 2; tràn thì bỏ scan cũ nhất và log cảnh báo (độ trễ quan trọng hơn đủ scan).
  - IMU: sâu đủ 2 s (400 mẫu); tràn là lỗi cấu hình, log lỗi và bỏ mẫu cũ nhất.
  - keyframe, GNSS, odom, loop: sâu 32; tràn thì bỏ cũ nhất và cảnh báo.
- Trạng thái chia sẻ chỉ có hai thứ, mỗi thứ một mutex và chủ sở hữu ghi rõ trong code: `state_` (Estimate mới nhất, backend ghi, ai cũng đọc) và `map_` (front-end ghi và đọc trên đường nóng; backend ghi pose hiệu chỉnh ngoài đường nóng qua một hàng đợi "sửa pose" để front-end áp dụng giữa hai scan, tránh khoá map trong scan matching).
- **`deterministic: true`**: một luồng duy nhất, các bước chạy theo thứ tự timestamp của message, mọi RNG seed cố định, không dùng đồng hồ tường. Đây là chế độ của test và `bench`. Chế độ nhiều luồng chỉ dùng khi chạy thật và chấp nhận sai khác nhỏ giữa hai lần chạy.
- Đường nóng (mỗi scan, mỗi mẫu IMU): không cấp phát heap, không log, không exception cho từng điểm; buffer reserve một lần trong constructor theo `max_points_per_scan` trong YAML. Vi phạm bắt bằng test có bộ đếm cấp phát (ADR-009).
- Ưu tiên luồng và ghim nhân: tuỳ chọn trong YAML (`realtime.pin_cores`, `realtime.priority`), mặc định tắt; chỉ bật trên Pi5 sau khi đo.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Một luồng cho tất cả | Đơn giản, tất định | iSAM2 update có thể vượt 100 ms khi relinearize; chặn scan tiếp theo. Giữ làm chế độ `deterministic` |
| Executor đa luồng của rclcpp | Có sẵn | Lõi không có ROS; không kiểm soát được thứ tự và hàng đợi |
| Thread pool theo tác vụ | Tận dụng 4 nhân | Thứ tự không tất định, khó tái lập; scan matching tuần tự theo bản chất |
| Hàng đợi lock-free | Ít tranh chấp | Mutex đủ nhanh ở 10 Hz và 200 Hz; xem lại nếu profiling trên Pi5 chỉ ra |

## Hệ quả

Mọi tranh chấp luồng nằm trong một file (`Estimator`), dễ soát. Test và bench tất định. Đổi lại, backend chạy trễ một keyframe so với front-end, nên pose công bố ở tần số scan là pose front-end hiệu chỉnh bởi kết quả backend gần nhất; cần ghi rõ trong README và trong message đầu ra.
