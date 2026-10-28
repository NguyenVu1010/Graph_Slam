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

## Sửa đổi 2026-10-27: "nới rộng" theo hướng suy biến nghĩa là "không quan sát"

- Theo hướng bị đánh dấu suy biến, covariance của factor LiDAR không còn là `relax_factor` nhân phương sai đã chuẩn hoá (vẫn là một giá trị tự tin). Covariance theo hướng đó là phương sai cố định `scan_match.unobserved_var_m2` / `unobserved_var_rad2` (mặc định 0.001 m² và 1e-4 rad²), không tương quan với các hướng khác, và không nhỏ hơn `relax_factor` lần giá trị đã chuẩn hoá. Các hướng quan sát được giữ nguyên. Công thức và kiểm thử: `docs/derivations/scan_match.md` mục 6.
- Đánh giá trên main 3f90e34 (`docs/derivations/degeneracy.md` mục 7): detector absolute (`eig_threshold` 100) đánh dấu trục hành lang ở mọi keyframe (dọc trục 3.8 mm khi có odometry bánh xe, 0.30 m khi tắt) nhưng làm xấu hộp (5 keyframe khoẻ bị đánh dấu) và E-01 (2.18 -> 2.45 mm). Vì vậy `degeneracy.enabled` vẫn là `false` và `unobserved_var_m2` vẫn là 0.001 m² (quyết định của chủ dự án); việc chỉnh lại `unobserved_var_m2` (ứng viên 0.01 m²) làm ở S2 trên nhiều bản ghi Gazebo. Một detector theo sigma (độ lệch chuẩn hậu nghiệm so với dung sai vật lý) đã được đánh giá trong PR 4b và đang tạm giữ: lỗ rò cạnh tường–sàn (1.15 %) cho trục hành lang độ lệch chuẩn 3.8 mm, dưới mọi dung sai ≥ 2 cm, nên nó không đánh dấu gì cho đến khi lỗ rò của bản đồ được sửa.
- Đã xử lý (không còn là việc mở): trôi dọc trục khi chỉ có IMU. Nguyên nhân là độ nghiêng không phân biệt được với gia tốc dọc trục; đã sửa bằng factor hướng trọng lực trên keyframe 0 (ADR-020, PR 4d). Factor tương đối `decoupled` (ADR-021) đọc covariance của scan matching đúng chart nó được tính (tịnh tiến và quay tách riêng); với `between`, phần dư lớn dọc trục tự do rò thông tin quay vào tịnh tiến qua Log của SE(3). Trên main 3f90e34, với detector absolute và `unobserved_var_m2` 0.001 m², sai số dọc trục khi chỉ có IMU là 0.021 m (0.16 m trên 94b9f16); với 1 m² là 0.82 m, khoảng 1.3 sigma của dead reckoning (0.6-0.7 m sau 17 s).
