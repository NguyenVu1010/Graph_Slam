# ADR-008: Cấu hình: YAML là nguồn tham số duy nhất

- Trạng thái: đề xuất
- Ngày: 2026-10-23

## Bối cảnh

Lõi không có ROS nên không dùng được ROS parameters làm nguồn gốc; `bench`, test và `ros2_node` phải chạy cùng một bộ tham số; không được có số "ma thuật" trong code; mọi tính năng mới phải bật tắt được và mặc định giữ hành vi cũ.

## Quyết định

- Một file `config/default.yaml` là nguồn duy nhất. `ros2_node` cài file này vào share của gói và nạp qua `ros__parameters`; `bench` và test đọc thẳng bằng yaml-cpp trong `pipeline`.
- Mỗi module có một struct cấu hình (`PreprocessConfig`, `ImuConfig`, ...) chỉ chứa giá trị đã kiểm tra; module không bao giờ đọc YAML, chỉ nhận struct. `pipeline` là nơi duy nhất parse.
- Quy tắc khoá: đơn vị trong tên (`_m`, `_s`, `_hz`, `_deg`, `_mps2`, `_radps`); góc trong YAML được phép bằng độ với hậu tố `_deg`, chuyển sang radian khi nạp; quaternion theo thứ tự Hamilton `[w, x, y, z]` và khoá có hậu tố `_wxyz`.
- Kiểm tra khi khởi động: thiếu trường, sai kiểu, ngoài khoảng cho phép → throw với tên trường đầy đủ (test C-04); không có giá trị mặc định ngầm cho tham số nhiễu và extrinsic.
- Cờ tính năng: `gnss.enabled`, `wheel_odom.enabled`, `loop_closure.enabled`, `degeneracy.enabled`, ... mặc định tắt cho đến khi được đánh giá; `deterministic` và `random_seed` ở cấp cao nhất.
- Hồ sơ nhiễu của mô phỏng (`sim/noise/*.yaml`) là file riêng vì mô tả "thế giới", nhưng test SIM-03 ép các tham số nhiễu trùng tên phải khớp với `config/default.yaml` (ADR-011).
- Tham số dùng trong một lần chạy được ghi lại nguyên văn vào `results/<run-id>/config.yaml` để tái lập.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chỉ ROS parameters | Chuẩn ROS, đổi lúc chạy | Lõi không có ROS; bench và test cần đọc được không qua rclcpp |
| JSON | Có schema chuẩn | Không có comment, khó đọc cho tham số có đơn vị; ROS 2 dùng YAML sẵn |
| Cờ dòng lệnh | Nhanh khi thử | Không lưu được, không tái lập; vẫn cho phép ghi đè một khoá YAML qua dòng lệnh trong `bench` |
| Hằng số `constexpr` trong code | Nhanh nhất | Đổi tham số phải build lại; không ablation được |

## Hệ quả

Một file là đủ để tái lập một lần chạy. Đổi lại, thêm tham số là phải sửa ba chỗ (YAML, struct, kiểm tra) và viết test cho kiểm tra; chấp nhận vì sai tham số im lặng là lỗi đắt nhất trong SLAM.
