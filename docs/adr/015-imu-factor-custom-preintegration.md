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

## Ghi chú (PR 4d, 2026-10-27): Jacobian gần đúng của `BetweenFactor<Pose3>` cho factor LiDAR

Chẩn đoán `docs/derivations/imu_only_observability.md` ("Exploratory controls") tìm thấy một lỗi
nhất quán của backend: với factor LiDAR không mang thông tin dọc trục hành lang (ngữ nghĩa "không
quan sát" của PR 4b, relax 1e8 hoặc bỏ hẳn) và không phản hồi backend về front end, keyframe cuối
của backend ở +5772 m trong khi sigma tự báo là 14 m. Nguyên nhân: `BetweenFactor<Pose3>` dùng
Jacobian của `between` mà bỏ `Jr^-1` của `Logmap` (GTSAM không có
`GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`), và phần dư SE(3) truyền sai số lớn dọc hướng không quan
sát sang các hướng quan sát qua `V(phi)^-1`. Một factor LiDAR với phần dư tách tịnh tiến / quay
và Jacobian đúng đưa lần chạy đó về -8,7 m với sigma 24,5 m.

Việc tiếp theo: PR 4e riêng, nhỏ (factor LiDAR với Jacobian `Jr^-1` đúng, hoặc define của
GTSAM). Tiêu chí nhận: lần đối chứng "x removed, no backend feedback" của nghiên cứu nằm trong
sigma của chính nó. PR 4b (dùng ngữ nghĩa "không quan sát") cần nó.

## Ghi chú (PR 4e, 2026-10-28): ba dạng factor tương đối cho LiDAR và loop, chờ quyết định

Trạng thái: **đang chờ quyết định của owner**. PR 4e thêm khoá `backend.relative_factor` với ba
dạng; mặc định `between` (hành vi trước PR 4e, `est.tum` của E-01 trùng md5 với main). Hai dạng
chính xác là tùy chọn đã đánh giá, chưa được chọn. Đạo hàm và số liệu đầy đủ:
`docs/derivations/backend_factors.md` §5 và §5a.

Ba dạng (cùng nhiễu `Gaussian::Covariance(P cov P^T)` và cùng kernel Cauchy cho loop):

- `between`: `gtsam::BetweenFactor<Pose3>`, phần dư `Logmap(Z^-1 T_i^-1 T_j)`. Jacobian bỏ
  `Jr^-1(e)` của `Logmap` (GTSAM 4.2 không có `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`): đúng khi
  sai số bằng 0, sai tương đối `O(|e|)` ở chỗ khác (> 0,1 ở 5 m / 30 độ).
- `se3_exact` (`detail::Se3RelativePoseFactor`): cùng phần dư SE(3), Jacobian đúng
  `J_j = Jr^-1(r)`, `J_i = -Jr^-1(r) Ad(T_j^-1 T_i)` lấy từ `odom::relative_pose_residual`.
- `decoupled` (`detail::DecoupledRelativePoseFactor`): phần dư tịnh tiến
  `R_Z^T (R_i^T (p_j - p_i) - p_Z)` và quay `Log(R_Z^T R_i^T R_j)` tách riêng, Jacobian đúng.
  Bằng phần dư SE(3) ở bậc một của nhiễu; khác ở số hạng BCH `0,5 [rho, phi]`.

Phương án `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR` (build GTSAM với cờ đó) không đưa vào đánh
giá: tính đúng khi đó phụ thuộc cờ build và bản pin trong image deps (build khác cờ sẽ lặng lẽ
quay về Jacobian gần đúng), và cờ đổi cả những chỗ GTSAM tự dùng `BetweenFactor`. Về số học
nó giống `se3_exact`.

Kết quả đối chứng ("x removed, no backend feedback", seed 42, sai số x của keyframe cuối và
sigma tự báo):

| dạng | circle | straight |
| --- | --- | --- |
| between | +5772,3 m (14,05): 411 sigma | +11,37 m (20,90): 0,54 sigma |
| se3_exact | -28,23 m (1,75): 16 sigma | +7,47 m (24,28): 0,31 sigma |
| decoupled | -8,71 m (24,46): 0,36 sigma | +4,36 m (24,48): 0,18 sigma |

`se3_exact` hết phân kỳ nhưng tự tin quá mức trên circle: phần dư SE(3) đưa sai số tịnh tiến
lớn dọc trục không quan sát vào các thành phần quan sát được qua `V(phi)^-1`. Chỉ `decoupled`
nằm trong sigma của chính nó ở cả hai cảnh. Trên test tổng hợp, mỗi dạng dùng nhiễu theo đúng
mô hình của nó: NEES theo x của `se3_exact` là 0,85, của `decoupled` là 0,92, còn `between` làm
hệ suy biến.

Cấu hình đang ship, 15 seed (42..56) ghép cặp với `between`: không dạng nào khác `between` có
ý nghĩa thống kê (mọi CI 95 % của hiệu trung vị đều chứa 0). Trung vị của `between` / `se3_exact`
/ `decoupled`:

- hành lang 60 s có GNSS: 1,85 / 2,58 / 3,37 m;
- hành lang 60 s chỉ LiDAR: 38,3 / 41,1 / 45,1 m;
- box NEES: 8,57 / 8,58 / 8,47 (cả ba trong 7,8 ± 1 ở `cov_scale` 4,7, nên không cần hiệu chỉnh lại);
- circle 20 s, sai số cuối: 1,45 / 2,60 / 2,30 m;
- straight 20 s, sai số cuối: 2,72 / 3,26 / 3,38 m.

Các cảnh này hỗn loạn: cùng dạng `decoupled`, chỉ đổi thứ tự phép nhân dấu phẩy động, cho
trung vị circle 3,30 thay vì 2,30 m. Với seed 42..44 của `CorridorImuOnlyAlongAxisWithinDeadReckoning`,
`se3_exact` vượt ngưỡng 3 sigma_DR (trung vị 7,33 m so với 2,11 m), `decoupled` thì không.
E-01..E-03 và độ trễ (trung vị 5 lần) không đổi trong phạm vi nhiễu đo với cả ba dạng.

Hệ quả, chung cho mọi lựa chọn: dạng chính xác tốn thêm một `Jr^-1` cho mỗi lần tính factor
(một lần mỗi keyframe và mỗi lần relinearize), không đo được trên E-01. Chọn `decoupled` nghĩa
là đổi mô hình lỗi của factor LiDAR, cần ghi rõ trong quyết định (`cov` hiểu theo chart
SO(3) x R3). PR 4b dùng ngữ nghĩa "không quan sát", nên phụ thuộc vào quyết định này.

Quyết định: chờ owner.
