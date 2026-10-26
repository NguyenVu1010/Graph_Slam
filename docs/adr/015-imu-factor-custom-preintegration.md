# ADR-015: IMU vào graph bằng factor preintegration 9x9 tự viết, loop dùng kernel Cauchy

- Trạng thái: chấp nhận
- Ngày: 2026-10-26
- Thay thế một phần: ADR-004 (câu "IMU vào graph bằng `CombinedImuFactor`")

## Bối cảnh

ADR-004 ghi IMU vào graph bằng `gtsam::CombinedImuFactor`. Module `imu` (đã merge) tự tích
phân preintegration theo sơ đồ `ManifoldPreintegration`: delta `dR, dv, dp`, Jacobian theo
bias và covariance 9x9 thứ tự `[phi, p, v]` (`docs/derivations/imu_preintegration.md` §1-§5).
GTSAM 4.2 đang dùng (bản ghi `4.2a9`, tag phát hành 4.2) được build với
`GTSAM_TANGENT_PREINTEGRATION`, nên `PreintegratedCombinedMeasurements` dẫn xuất từ
`TangentPreintegration`: covariance và Jacobian theo bias gyro nằm trong một chart khác
(`imu_preintegration.md` §6). Đưa delta của `imu` vào `CombinedImuFactor` sẽ trộn hai chart;
còn để GTSAM tích phân lại thì tích phân hai lần cùng dữ liệu và kết quả phụ thuộc cờ build
của GTSAM. Header `backend/graph.hpp` (mới hơn ADR-004) đã ghi factor 9x9 tự viết.

Loop closure cần một robust kernel để loop sai không kéo lệch đồ thị (B-03), nhưng ADR-004
không chọn kernel và `BackendConfig` chưa có tham số cho nó.

## Quyết định

- IMU vào graph bằng factor tự viết `detail::ImuPreintFactor` (5 biến `X_i, V_i, X_j, V_j,
  B_i`, residual 9 chiều `[phi, p, v]` trong chart delta của `imu`, hiệu chỉnh bias bậc một,
  nhiễu là `delta.cov` không hoán vị, Jacobian giải tích) cộng một
  `BetweenFactor<imuBias::ConstantBias>` cho random walk của bias, sigma
  `random_walk * sqrt(dt_s)`. Đạo hàm: `docs/derivations/backend_factors.md` §3-§4. Random
  walk bằng 0 bị constructor từ chối.
- Loop closure là `BetweenFactor<Pose3>` bọc kernel Cauchy, `k = 3.5485 =
  sqrt(chi2_6(0.95))` (hằng `kLoopCauchyK`; khoá YAML `backend.loop_cauchy_k` sẽ thêm cùng
  module loop). Đạo hàm: `backend_factors.md` §5.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| `CombinedImuFactor`, GTSAM tích phân lại IMU | Factor có sẵn, đã kiểm thử | Tích phân hai lần; chart covariance phụ thuộc `GTSAM_TANGENT_PREINTEGRATION`, lệch với `imu` |
| `CombinedImuFactor` nhận delta của `imu` | Không tích phân lại | Trộn chart manifold và tangent; sai khi bias gyro khác 0 |
| Kernel Huber cho loop | Lồi, hội tụ ổn định | Lực kéo không triệt tiêu: một loop lệch 5 m vẫn kéo đồ thị vài chục cm |
| Kernel Cauchy (chọn) | Trọng số về 0 với outlier lớn (~1e-4 với lệch 5 m) | Không lồi; cần đoán đầu tốt (có từ LiDAR và IMU) |

## Hệ quả

Backend chỉ phụ thuộc kiểu `Pose3`, `Vector3`, `ConstantBias` và factor nền của GTSAM, không
phụ thuộc cờ preintegration của GTSAM. Phải giữ đạo hàm và test Jacobian (B-01) của factor tự
viết. Kernel Cauchy có thể bỏ qua một loop đúng nhưng có covariance quá tự tin; cần đo trên
E-01..E-03 khi module loop vào. Không đổi ATE hay độ trễ của pipeline hiện tại (backend chưa
nối vào `Estimator`).
