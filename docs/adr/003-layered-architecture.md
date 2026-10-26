# ADR-003: Kiến trúc phân lớp: lõi không phụ thuộc ROS

- Trạng thái: đề xuất
- Ngày: 2026-10-22

## Bối cảnh

Cùng một thuật toán phải chạy ở ba nơi: node ROS 2 (Gazebo và rig thật), chương trình đo độ trễ trên Pi5, và unit test. Ranh giới giữa các phần phải rõ để đọc và test độc lập.

## Quyết định

```
ứng dụng      ros2_node, bench                      (ROS 2 chỉ ở đây)
    │
điều phối     pipeline::Estimator                   (luồng, hàng đợi; ADR-005)
    │
module        preprocess, map, scan_match, degeneracy, imu, gnss, odom, loop, backend
    │                                               (GTSAM chỉ trong backend)
nền           common: Pose, Timestamp, Result, logger, Lie ops
```

- Bốn lớp, phụ thuộc chỉ đi từ trên xuống: ứng dụng → điều phối → module thuật toán → nền. Linker ép điều này qua target CMake theo module (ADR-013).
- ROS 2 chỉ xuất hiện ở lớp ứng dụng; `ros2_node` chỉ đổi message sang struct của core và ngược lại, và adapt logger của core sang rclcpp.
- GTSAM chỉ nằm trong module `backend`; các module front-end trả về kết quả bằng kiểu của core (`Pose`, `PoseCovariance`), không biết GTSAM.
- `Estimator` là cửa vào duy nhất của core: nhận dữ liệu cảm biến có timestamp, trả pose và bản đồ. Mô hình luồng chi tiết nằm ở ADR-005.
- `eval/` bằng Python không gọi vào core; nó chỉ đọc file quỹ đạo định dạng TUM do core xuất ra và bag MCAP qua thư viện thuần Python (ADR-010, ADR-011).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Node ROS 2 nguyên khối (kiểu LIO-SAM) | Nhanh làm ban đầu | Khó test, không chạy được benchmark trên Pi5 mà không có ROS, logic lẫn với callback |
| Nhiều node ROS 2 nhỏ (preprocess, odometry, mapping) | Tách biệt rõ | Thêm độ trễ serialize và truyền message trên Pi5; khó đồng bộ thời gian giữa node |
| Plugin cảm biến nạp động | Mở rộng linh hoạt | Quá tay cho 4 cảm biến cố định; mỗi cảm biến là một module tĩnh là đủ |

## Hệ quả

Lõi test được độc lập, benchmark trên Pi5 không kéo theo ROS, và đổi thư viện tối ưu chỉ ảnh hưởng một module. Đổi lại, phải viết và test lớp chuyển đổi message ↔ struct (R-01), và việc hiển thị RViz chỉ có khi chạy qua `ros2_node`.
