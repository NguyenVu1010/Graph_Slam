# ADR-004: Estimator: LiDAR factor loosely coupled trong iSAM2

- Trạng thái: đề xuất
- Ngày: 2026-10-22

## Bối cảnh

Cần hợp nhất LiDAR, IMU, GNSS và odometry, dễ thêm cảm biến, và có chỗ để cài xử lý suy biến (điểm nghiên cứu của dự án). Phải chạy được 10 Hz trên Pi5.

## Quyết định

- Scan-to-map point-to-plane (tự viết) cho ra pose tương đối giữa hai keyframe, đưa vào graph dưới dạng `BetweenFactor<Pose3>`.
- Covariance của factor đó lấy từ nghịch đảo Hessian 6×6 của scan matching, nhân hệ số hiệu chỉnh, và nới rộng theo các hướng suy biến. Đây là nơi cài điểm nghiên cứu (module `degeneracy`).
- IMU vào graph bằng `CombinedImuFactor` (preintegration), đồng thời dùng để deskew và đoán trước pose cho scan matching. GNSS, odometry bánh xe và loop closure là các factor riêng, mỗi factor sau một cờ YAML.
- Back-end iSAM2; keyframe mỗi 1 m hoặc 10°; ngưỡng relinearize 0.1.
- Điều kiện xem lại: nếu trên kịch bản hành lang mô phỏng (E-04), ATE của hệ kém FAST-LIO2 quá [x]% sau khi đã bật xử lý suy biến, thì viết ADR mới chuyển front-end sang IEKF tightly coupled.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn cho giai đoạn 1 |
| --- | --- | --- |
| IEKF tightly coupled (kiểu FAST-LIO2) | Chính xác, rất nhanh trên ARM | Khó thêm GNSS, loop closure và odometry theo kiểu cắm-rút; là bước mở rộng tự nhiên sau giai đoạn 1 |
| Mỗi residual point-to-plane là một factor trong graph | Tightly coupled đúng nghĩa | Hàng nghìn factor mỗi scan; vượt ngân sách thời gian trên Pi5 |
| Fixed-lag smoother thay cho iSAM2 | Bộ nhớ và độ trễ bị chặn | Mất khả năng loop closure toàn cục; xem lại nếu chạy quãng đường rất dài |
| EKF loosely coupled (kiểu robot_localization) | Quen thuộc, nhiều tài liệu | Không có loop closure, kém nhất quán với IMU tần số cao, ít giá trị nghiên cứu |

## Hệ quả

Kiến trúc dễ hiểu, giống LIO-SAM nên dễ so sánh và dễ trình bày. Nhược điểm cần ghi rõ trong README: covariance từ Hessian thường quá tự tin, và scan-to-map dùng bản đồ dựng từ chính các ước lượng trước đó, nên các factor LiDAR không độc lập thống kê.
