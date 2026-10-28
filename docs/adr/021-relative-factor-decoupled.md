# ADR-021: Factor tương đối của LiDAR và loop mặc định `decoupled` (phần dư tịnh tiến và quay tách riêng)

- Trạng thái: đề xuất
- Ngày: 2026-10-28
- Bổ sung: ADR-015 (câu "loop closure là `BetweenFactor<Pose3>` bọc kernel Cauchy" và ghi chú
  PR 4e); ADR-004 (câu "đưa vào graph dưới dạng `BetweenFactor<Pose3>`")

## Bối cảnh

Factor tương đối nối hai keyframe `X_i = T_w_bi`, `X_j = T_w_bj` (`Graph::add_lidar_factor`,
`Graph::add_loop_factor`, `docs/derivations/backend_factors.md` §5):

- Đầu vào: phép đo `Z = T_bi_bj` (scan matching hoặc loop closure) và hiệp phương sai 6x6
  `cov` của scan matcher. `cov = cov_scale * max(sigma_r^2, min_residual_sigma_m^2) * A^-1`,
  với `A` là Hessian Gauss-Newton của hàm chi phí point-to-plane tại nghiệm (nới dọc hướng suy
  biến khi bật), trong tiếp tuyến `[rho, phi]` của nhiễu loạn phải `T_w_l <- T_w_l Exp(xi)`
  (`scan_matcher.hpp`, `MatchResult::cov`). Pipeline đổi `cov` từ `l` sang `b` bằng adjoint của
  `T_b_l`. Tại `xi = 0` chart SE(3) và chart SO(3) x R3 trùng nhau (`V(0) = I`), nên `cov` là
  một đại lượng bậc một tại nghiệm: phần tịnh tiến là độ dời của gốc, phần quay là góc quay,
  đọc riêng từng phần.
- Đầu ra cho iSAM2: phần dư 6 chiều theo thứ tự GTSAM `[phi, rho]`, Jacobian theo `X_i`, `X_j`
  và nhiễu `Gaussian::Covariance(P cov P^T)` (loop: bọc thêm Cauchy `k = 3,5485`).

PR 4e (#25) thêm khoá `backend.relative_factor` với ba dạng cùng nhiễu, mặc định `between` (hành
vi cũ), và đánh giá chúng (ghi chú PR 4e của ADR-015, `backend_factors.md` §5a):

- `between`: `gtsam::BetweenFactor<Pose3>`, phần dư `Log(Z^-1 T_i^-1 T_j)`, Jacobian bỏ
  `Jr^-1` của `Logmap` (GTSAM 4.2 không có `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR`).
- `se3_exact`: cùng phần dư SE(3), Jacobian đúng.
- `decoupled`: phần dư tịnh tiến `R_Z^T (R_i^T (p_j - p_i) - p_Z)` và quay
  `Log(R_Z^T R_i^T R_j)` tách riêng, Jacobian đúng.

## Vấn đề

Lần đối chứng của nghiên cứu (`PipelineComponent.RelativeFactorSeeds`, seed 42, hành lang tròn,
LiDAR không mang thông tin dọc x, không phản hồi backend về front end, factor trọng lực tắt):
sai số x của keyframe cuối và sigma biên tự báo.

| dạng | sai số x | sigma tự báo | tỉ số |
| --- | --- | --- | --- |
| `between` | +5772 m | 14 m | 411 sigma |
| `se3_exact` | -28 m | 1,75 m | 16 sigma |
| `decoupled` | -8,7 m | 24,5 m | 0,36 sigma |

Chỉ `decoupled` nằm trong sigma của chính nó (cảnh thẳng: 0,54 / 0,31 / 0,18 sigma).

## Cơ chế

Nói gọn: chart log SE(3) gắn phần quay vào phần tịnh tiến bằng một cánh tay đòn có độ dài bằng
phần dư tịnh tiến. Khi phần dư tịnh tiến dọc trục tự do lớn cỡ mét, cánh tay đòn đó biến góc
quay (được quan sát tốt) thành sai số giả ở các hàng tịnh tiến (được cân bằng sigma cỡ cm), tức
thông tin giả dọc trục tự do.

Công thức: với `E = Z^-1 T_i^-1 T_j = (R_E, t_E)`, `phi = Log(R_E)`, hàng tịnh tiến của `Log`
SE(3) là

    rho = V(phi)^-1 t ~= t - 1/2 phi x t = t + 1/2 [t]_x phi

- Với `t` cỡ mét dọc trục tự do, `1/2 [t]_x` là cánh tay đòn: nó đưa nhiễu loạn quay vào các
  hàng tịnh tiến, mà các hàng này được cân bằng sigma tịnh tiến cỡ cm. Góc quay được quan sát
  tốt trở thành thông tin giả dọc trục tịnh tiến tự do.
- `decoupled` đọc tịnh tiến và quay riêng (`r_p = t_E`, `r_phi = phi`), đúng chart mà `cov` được
  tính: hàm chi phí point-to-plane tuyến tính theo độ dời `dt` ở mọi góc quay, nên hướng tự do
  vẫn tự do ở mọi phần dư.
- `se3_exact` dùng Jacobian đúng của phần dư SE(3), nên hết phân kỳ nhưng vẫn mang cánh tay đòn:
  tự tin quá 16 sigma.
- `between` cộng thêm đạo hàm `Logmap` gần đúng (bỏ `Jr^-1(e) = I + ad(e)/2 + ...`, mà `ad(e)`
  chứa `skew(rho)` cỡ mét). Bước Gauss-Newton sai với hệ số cỡ phần dư, iSAM2 chỉ đi một bước mỗi
  keyframe nên bước sai không được lặp bỏ: vì vậy nó phân kỳ chứ không chỉ tự tin quá mức.

## Quyết định

1. `backend.relative_factor` mặc định `decoupled` trong `config/default.yaml` và trong
   `BackendConfig` (`RelativeFactor::kDecoupled`). Đây là thay đổi bổ sung của header giao diện
   `core/backend/include/mss/backend/config.hpp` (chỉ giá trị mặc định), ghi trong PR.
2. `between` và `se3_exact` vẫn chọn được để so sánh; giá trị khác là lỗi cấu hình.
3. Loop dùng cùng dạng, bọc trong kernel Cauchy như trước.

## Bằng chứng

**15 seed (42..56), cấu hình phát hành, ghép cặp với `between`** (trung vị (max) | thắng / thua
| khoảng tin cậy 95 % bootstrap của hiệu trung vị, dạng - between; nhỏ hơn là tốt hơn):

| chỉ số | between | se3_exact | decoupled |
| --- | --- | --- | --- |
| hành lang 60 s + GNSS, ATE (m) | 1,85 (18,54) | 2,58 (26,41) \| 4/11 \| [-0,06; +1,02] | 3,37 (23,56) \| 6/9 \| [-0,09; +2,03] |
| hành lang 60 s chỉ LiDAR + IMU, ATE (m) | 38,34 (205,39) | 41,14 (152,24) \| 8/7 \| [-5,88; +20,07] | 45,06 (149,52) \| 7/8 \| [-12,07; +11,67] |
| NEES factor LiDAR box (kỳ vọng 6) | 8,57 (12,47) | 8,58 (12,47) \| 5/10 \| [-0,00; +0,01] | 8,47 (12,49) \| 5/10 \| [-0,00; +0,02] |
| tròn 20 s, x tự do, \|sai số x cuối\| (m) | 1,45 (7,02) | 2,60 (13,63) \| 6/9 \| [-0,65; +1,71] | 2,30 (7,37) \| 6/9 \| [-0,21; +1,01] |
| tròn 20 s, x tự do, RMS dọc x (m) | 1,36 (2,84) | 1,33 (5,81) \| 8/7 \| [-0,91; +1,13] | 1,68 (3,25) \| 6/9 \| [-0,84; +1,35] |
| thẳng 20 s, x tự do, \|sai số x cuối\| (m) | 2,72 (6,28) | 3,26 (5,90) \| 6/9 \| [-0,39; +0,40] | 3,38 (8,18) \| 5/10 \| [-0,26; +0,82] |

Mọi khoảng tin cậy chứa 0: trên các cảnh đang có, không dạng nào khác `between` có ý nghĩa thống
kê. Các cảnh này hỗn loạn (cùng dạng `decoupled`, chỉ đổi thứ tự phép nhân dấu phẩy động, cho
trung vị tròn 3,30 thay vì 2,30 m), nên chênh lệch trung vị nhỏ không quyết định. Quyết định dựa
trên lần đối chứng (mục Vấn đề) và cơ chế, không dựa trên các trung vị này. Test tổng hợp
`BackendComponent/UnobservedAxis.*` (mỗi dạng với nhiễu trong chart của nó): NEES theo x 0,85
(`se3_exact`), 0,92 (`decoupled`); `between` làm hệ suy biến.

**`CorridorImuOnlyAlongAxisWithinDeadReckoning`** (seed 42..44, sai số x cuối, m; giới hạn không
đổi: tròn trung vị <= 3 sigma_DR = 2,11 m và RMS <= 3,0 m; thẳng trung vị <= 1,86 m và RMS
<= 1,88 m):

| dạng | tròn | thẳng |
| --- | --- | --- |
| between (PR 4e) | -1,287 / +0,384 / -1,445 | +0,930 / +0,470 / +2,718 |
| se3_exact (PR 4e) | -13,632 / -2,603 / -7,328 (trung vị 7,33 m, vượt) | +0,988 / +1,375 / +1,650 |
| decoupled (PR 4e) | +2,298 / -0,780 / -0,079 | +0,815 / +0,603 / +8,180 |
| decoupled, mặc định (PR này) | +2,298 / -0,780 / -0,079 | +0,815 / +0,603 / +8,180 |

Với mặc định `decoupled` test qua: tròn trung vị 0,780 m (giới hạn 2,113 m), RMS trung vị 0,742 m
(3,0 m); thẳng trung vị 0,815 m (1,86 m), RMS trung vị 1,430 m (1,88 m). Seed 44 của cảnh thẳng
có sai số cuối 8,18 m (11,6 sigma_DR); test giới hạn trung vị nên đuôi đó không bị chặn.

**E-01..E-03 và bench.** ATE (mm), E-01 / E-02 / E-03, PR 4e (Windows, overlay kịch bản):

| dạng | E-01 | E-02 | E-03 |
| --- | --- | --- | --- |
| main (`between`) | 2,181 | 2,181 | 2,175 |
| se3_exact | 2,189 | 2,189 | 2,176 |
| decoupled | 2,185 | 2,185 | 2,179 |
| decoupled, mặc định (PR này, `run_integration.py`) | 2,185 | 2,185 | 2,179 |

`est.tum` của E-01 và E-03 trên nhánh này trùng md5 với lần chạy `decoupled` của PR 4e.

Bench E-01 của PR 4e (desktop, WSL, trung vị 5 lần xen kẽ; scan p50 / p95 ms, wall s, CPU
user+sys s): main 4,83 / 6,57 / 4,54 / 3,05; between 4,84 / 6,35 / 4,46 / 3,03; se3_exact
4,74 / 6,50 / 4,23 / 3,01; decoupled 4,75 / 6,53 / 4,34 / 3,04. Không chênh lệch nào vượt nhiễu
giữa các lần chạy (độ trải p50 trong mỗi dạng khoảng 0,6 ms).

Bench E-01 của PR này (desktop, WSL, trung vị 5 lần xen kẽ, main `fb218b2` với `between` so với
nhánh này với `decoupled`):

| build | scan p50 (ms) | scan p95 (ms) | wall (s) | CPU user+sys (s) |
| --- | --- | --- | --- | --- |
| main | 4,400 | 5,768 | 3,666 | 2,73 |
| nhánh này | 4,389 | 5,922 | 3,598 | 2,73 |

p95 cao hơn 0,15 ms, trong độ trải giữa các lần chạy (main 5,72 đến 5,89 ms, nhánh 5,77 đến
6,05 ms); p50, wall và CPU không đổi.

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Giữ `between` | Mang mâu thuẫn 411 sigma trong cảnh có trục tự do, mà kịch bản hành lang E-04 chính là cảnh đó; Jacobian gần đúng ở phần dư cỡ mét |
| `se3_exact` | Hết phân kỳ nhưng tự tin quá 16 sigma (cánh tay đòn `1/2 [t]_x` vẫn còn); vượt giới hạn của `CorridorImuOnlyAlongAxisWithinDeadReckoning` (trung vị tròn 7,33 m so với 2,11 m) |
| Build GTSAM với `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR` | Về số học giống `se3_exact`; tính đúng phụ thuộc cờ build của image deps (ghi chú PR 4e của ADR-015) |

## Hệ quả

- `est.tum` của E-01 / E-03 không còn trùng byte với main trước ADR này (ATE E-01 2,181 lên
  khoảng 2,185 mm, trong nhiễu).
- `eval/baselines.json` vẫn là null, nên không cần PR `baseline-update` cho thay đổi này.
- PR 4b (`feat/scan-match-gate`, ngữ nghĩa "không quan sát", một trục tịnh tiến tự do) phải đánh
  giá lại trên mặc định mới.
- Mọi factor sau này phải ghi chart của hiệp phương sai của nó ở giao diện module (như
  `MatchResult::cov` trong `scan_matcher.hpp`), và được test trên một trục tự do ở phần dư cỡ
  mét, không chỉ tại phép đo.
- Loop dùng cùng dạng bên trong kernel Cauchy.
- Test `BackendIsam.B04_IsamMatchesBatchLevenbergMarquardt` dựng tham chiếu batch theo dạng
  đã cấu hình và chạy cho cả ba dạng. Với hai dạng chính xác, LM dừng ở cận trên lambda trước
  điểm dừng (độ chênh chi phí nó so sánh cỡ 1e-9, ngang nhiễu tính chi phí), nên tham chiếu
  thêm một bước Gauss-Newton sau LM; giới hạn 1e-6 không đổi.
- Chi phí: một `Jr^-1` SO(3) (3x3) mỗi lần tính factor, không đo được trên E-01.
- Với suy biến quay (không phải tịnh tiến), không chart nào chính xác; chưa có cảnh đánh giá.
