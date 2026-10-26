# ADR-010: Dữ liệu và đánh giá: rosbag2 MCAP, quỹ đạo định dạng TUM, evo

- Trạng thái: đề xuất
- Ngày: 2026-10-23

## Bối cảnh

Dữ liệu đến từ ba nguồn: mô phỏng (ADR-011), dataset công khai (UrbanNav, M2DGR), rig thật. Đánh giá phải chạy được không cần ROS và không link core (ADR-003), so sánh được với baseline khác (FAST-LIO2, LIO-SAM), và kết quả không được ghi vào cây nguồn.

## Quyết định

- **Định dạng bag**: rosbag2 MCAP, nén zstd. Mọi nguồn (sim, dataset chuyển đổi, rig) quy về cùng bộ topic và kiểu message như ADR-011 mục 6, để một launch file chạy được tất cả.
- **Quỹ đạo**: định dạng TUM `t x y z qx qy qz qw` (thời gian giây, quaternion thứ tự x y z w của file; đây là biên I/O duy nhất mà quaternion không theo Hamilton `w x y z`, ghi chú trong code). `Estimator` và `bench` xuất `est.tum`; ground truth là `gt.tum`.
- **Độ đo**: ATE RMSE sau căn chỉnh SE(3) (không scale) và RPE theo đoạn 10 m, 50 m, 100 m, tính bằng `evo`; độ trễ p50/p95 và CPU/RAM từ `bench`. Bảng ablation: bật tắt từng factor và từng cờ.
- **Thư mục kết quả**: mọi script nhận đường dẫn làm tham số và ghi vào `results/<run-id>/` (git-ignore): `est.tum`, `gt.tum`, `config.yaml` đã dùng, `metrics.json`, biểu đồ evo, báo cáo nhiễu nếu có.
- **Baseline so sánh**: FAST-LIO2 và LIO-SAM chạy trên cùng bag qua container riêng, kết quả lưu cùng định dạng; không chặn merge (PF-04).
- **Dataset công khai**: UrbanNav HK (LiDAR + IMU + GNSS, có ground truth) cho GNSS; M2DGR cho đa dạng môi trường. Chuyển đổi một lần sang bộ topic chuẩn bằng `tools/convert_<dataset>.py`; không đưa vào CI vì quá nặng.
- **Thư viện Python**: `evo` cho độ đo, `rosbags` để đọc và ghi MCAP không cần ROS, `numpy`, `matplotlib`. `eval/` không bao giờ import rclpy.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| rosbag2 SQLite3 | Mặc định cũ | Chậm, file lớn, không đọc ngoài ROS tốt bằng MCAP |
| Định dạng KITTI cho quỹ đạo | Phổ biến | Không có timestamp; evo hỗ trợ TUM đầy đủ hơn |
| Tự viết ATE/RPE | Không phụ thuộc | evo là chuẩn cộng đồng, kết quả so sánh được với bài báo |
| Đọc bag qua rclpy trong `eval` | Dùng API chính thức | Buộc cài ROS để đánh giá; `rosbags` đủ và nhẹ |

## Hệ quả

Một lệnh đánh giá chạy được trên máy bàn không có ROS, kết quả so sánh được với các hệ khác. Đổi lại, phải bảo trì script chuyển đổi cho từng dataset và giữ bộ topic chuẩn ổn định qua các phiên bản bag test.
