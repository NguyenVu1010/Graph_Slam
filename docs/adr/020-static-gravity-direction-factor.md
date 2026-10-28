# ADR-020: Hướng trọng lực của cửa sổ tĩnh vào keyframe 0 (factor hướng, nhiễu 2 lần), có cổng đứng yên; kiểm tra nhất quán đã đánh giá và tắt

- Trạng thái: đề xuất
- Ngày: 2026-10-27
- Bổ sung: ADR-017 quyết định 4 (cửa sổ khởi tạo IMU); thay đổi hợp đồng `imu::InitState`

## Bối cảnh

Chẩn đoán `docs/derivations/imu_only_observability.md` (mục 1 đến 8):
- Trong hành lang mà LiDAR không quan sát trục x, một độ nghiêng `theta` quanh trục ngang vuông
  góc với x không phân biệt được với gia tốc `g theta` dọc x trong các factor IMU.
- LiDAR chỉ neo độ nghiêng so với bản đồ của chính nó.
- Neo tuyệt đối duy nhất là prior quay của keyframe 0 (`backend.prior_rot_sigma_deg` 1 độ =
  17,5 mrad), lỏng hơn 42 lần so với cái cửa sổ tĩnh đo được (0,42 mrad).
- Khi LiDAR không mang thông tin dọc x (ngữ nghĩa "không quan sát" của PR 4b), độ nghiêng đạt
  150 đến 212 mrad và ước lượng trôi 224 đến 316 m trong 20 s.

Cửa sổ tĩnh đo lực riêng `f = ba - R^T g`, tức là tổ hợp nghiêng / bias ngang, không tách được
hai đại lượng. Prior nghiêng chặt sẽ ghim sai độ nghiêng khi IMU có bias thật (`b / g`).

## Quyết định

1. `imu::ImuInitializer` trả thêm trong `InitState`:
   - `acc_mean_b`: trung bình gia tốc của cửa sổ, m/s^2, trong b;
   - `acc_mean_cov`: hiệp phương sai của trung bình, mỗi trục
     `max(s_k^2, acc_noise_density^2 rate_hz) / n`.

   Thay đổi giao diện bổ sung.
2. `backend::Graph::add_static_gravity_factor(k, f_b, cov)` (bổ sung) thêm
   `detail::StaticGravityFactor` lên pose và bias của keyframe 0 (`backend_factors.md` §6c):
   - phần dư `f_b - (ba - R^T g)` chiếu lên mặt phẳng vuông góc `f_b` (2 chiều: chỉ hướng, bỏ độ
     lớn `|g|` so với `ba_z`);
   - nhiễu `static_gravity_sigma_scale^2 * cov`, chiếu cùng cách;
   - đi cùng các prior keyframe 0.
3. Pipeline thêm factor khi `backend.static_gravity_factor` bật (mặc định bật,
   `backend.static_gravity_sigma_scale` 2,0), keyframe 0 chính là scan khởi tạo, và cửa sổ qua
   **cổng đứng yên**. Không qua thì bỏ qua factor, ghi log và đếm `n_gravity_factor_gated`.
   Cổng gồm:
   - `|gyro trung bình| <= imu.init_max_mean_gyro_radps` (0,15 rad/s);
   - mọi tốc độ bánh xe trong cửa sổ `<= imu.init_max_wheel_speed_mps` (0,1 m/s, khi wheel bật);
   - `||f| - g| <= imu.init_gravity_check_sigma` (3) x sqrt(phương sai trung bình + prior bias
     acc^2).
4. **Kiểm tra nhất quán: đã đánh giá, mặc định tắt** (`backend.static_gravity_check_keyframes: 0`;
   giá trị > 0 bật nó ở keyframe đó; kiểm tra `>= 0`). Khi bật, nếu thống kê innovation của
   factor so với phần còn lại của đồ thị (`backend_factors.md` §6c: `e^T S^-1 e`,
   `S = I - J P J^T`, tính qua `S^-1 = I + J P_rest J^T`) vượt `backend.static_gravity_chi2_gate`
   (5,991, 2 bậc tự do, 0,95), factor bị gỡ khỏi iSAM2 (`removeFactorIndices`) và khỏi tập dùng
   để dựng lại, đếm `n_gravity_factor_removed` (Stats, bench.json), ghi một cảnh báo
   (`Graph::static_gravity_chi2`, `Graph::remove_static_gravity_factor`, bổ sung).
   - Không tính được thống kê (trị riêng nhỏ nhất của `S` <= 1e-9, tức factor là thông tin
     duy nhất về đại lượng nó đo): giữ factor, đếm `n_gravity_factor_unchecked`.
   - Gỡ thất bại (iSAM2 ném lỗi hoặc ước lượng không hữu hạn): iSAM2 được dựng lại, factor ở
     lại, ghi cảnh báo, không thử lại (kiểm tra chạy một lần).
   - Vòng sửa 1 dùng phần dư hậu nghiệm `|W r|^2`: kỳ vọng khi đứng yên là `2 - leverage`, nhỏ
     hơn 2, nên 5 % chỉ là cận trên của tỉ lệ gỡ nhầm và độ nhạy với cửa sổ đang chuyển động
     thấp hơn. Thống kê innovation thay thế nó.

   **Vì sao tắt** (`imu_only_observability.md` mục 10g, 12). Mọi cảnh dưới đây khởi động đứng
   yên, nên mọi lần gỡ là gỡ nhầm:

   | cảnh (15 seed, nhiễu x2, kiểm tra ở keyframe 5) | số lần gỡ |
   | --- | --- |
   | hành lang GNSS 60 s | 6/15 (thống kê 37 đến 147) |
   | hành lang 60 s chỉ LiDAR | 2/15 |
   | box | 0/15 |
   | hành lang tròn chỉ IMU | 0/15 |
   | hành lang thẳng chỉ IMU | 2/13 (2 lần chạy không tới 5 keyframe) |
   | E-01..E-03 | 0 |

   - Tổng cộng 10/73, khoảng 14 % (40 % trên hành lang GNSS), so với 5 % của chi-square(2).
   - Thống kê khi đứng yên (14 đến 147) chồng lên thống kê ở các lần khởi động đang chuyển động
     (38 đến 233): không ngưỡng nào tách được hai trường hợp.
   - Tách nguyên nhân (mục 10g): tắt GNSS ngăn 4 trong 6 lần gỡ (phía GNSS là nguyên nhân lớn
     hơn; giả thuyết chưa kiểm chứng: biến `E` còn đang căn chỉnh ở keyframe 5); tăng `scan_match.cov_scale` lên 6,7 (NEES box
     8,57 xuống 6,22) chỉ đổi 6 thành 5. Phần còn lại (2/15 khi không GNSS và NEES nhất quán)
     chưa tách được, và 2/15 ở mức 5 % không vượt ngẫu nhiên (xác suất nhị thức khoảng 0,17).
   - Một factor sai ở lần khởi động tăng tốc thẳng chỉ tốn tối đa +2,2 % ATE (đo: -1,4 % và
     +2,2 %), vì `R_w_b` của bộ khởi tạo lấy từ cùng giá trị trung bình của cửa sổ. Trong hành
     lang mà LiDAR không quan sát trục di chuyển, không thống kê nào phát hiện được trường hợp
     đó.

   Cổng đứng yên (mục 3) vẫn bật. Mã thống kê innovation, các bộ đếm và các test giữ lại như
   một tham số đã đánh giá.
5. `Graph::gnss_frame_estimate()` (bổ sung, chẩn đoán): biến `E = T_e_w` của ADR-019 sau lần
   tối ưu gần nhất.

## Bằng chứng (15 seed 42..56, ghép cặp với main, `imu_only_observability.md` mục 10e)

Mỗi ô ghi trung vị (max), thắng/thua so với main và khoảng tin cậy 95 % bootstrap của hiệu
trung vị. Số nhỏ hơn là tốt hơn. Mọi cột chạy ở cấu hình phát hành (kiểm tra nhất quán tắt);
cột in đậm là cấu hình phát hành. Các cột có kiểm tra bật ở mục 10g.

| chỉ số | main | hướng, nhiễu x1 | hướng, nhiễu x2 (chọn) | hướng, nhiễu x3 |
| --- | --- | --- | --- | --- |
| hành lang GNSS 60 s, ATE (m) | 3,03 (27,9) | 2,44, 9/6, [-20,2; +2,3] | **1,85 (18,5), 8/7, [-19,6; +1,2]** | 2,14, 8/7, [-6,8; +1,5] |
| hành lang 60 s chỉ LiDAR, ATE (m) | 24,9 | 21,4, 9/6 | **38,3 (205), 6/9, [-11,0; +30,8]** | 26,5, 8/7 |
| NEES LiDAR box | 7,49 (13,3) | 9,42, 5/10 | **8,57 (12,5), 6/9, [-0,39; +2,09]** | 7,51 |
| hành lang tròn 20 s, sai số cuối (m) | 21,7 | 2,97 | **1,45, 14/1** | 2,15 |
| hành lang tròn, RMS dọc x (m) | 11,6 | 1,31 | **1,36, 15/0** | 1,87 |
| hành lang thẳng 20 s, sai số cuối (m) | 3,56 | 1,86 | **2,72, 10/5, [-6,4; +1,2]** | 3,85 |
| hành lang thẳng, RMS dọc x (m) | 1,85 | 0,79 | **1,34, 12/3, [-2,2; -0,17]** | 1,30 |

- Trên hai hành lang 60 s, mọi khoảng tin cậy chứa 0: factor không tốt hơn cũng không kém hơn
  main trong độ phân tán của 15 seed. Việc chọn nhiễu x2 dựa trên NEES và các hành lang chỉ IMU:
  - x1 làm NEES tăng nhiều nhất;
  - x3 trả lại hành lang thẳng (trung vị 3,85 m) và nhân đôi sai số giữa đường của hành lang tròn;
  - x2 có khoảng tin cậy NEES chứa 0.
- Vòng 1 "hành lang GNSS kém đi" là do 5 seed (main có đuôi nặng, 0,54 đến 27,9 m) cộng với một
  hiệu ứng nhỏ hơn và có thật của phần dư đầy đủ:
  - quá tự tin (NEES 9,6);
  - để mode rỗng (nghiêng quanh trục ngang, `ba` dọc trục) trượt khi chạy thẳng (seed 42, vòng 1:
    -88 mrad cùng `ba_x` -0,83 m/s^2).

  Phần dư đầy đủ vẫn chạy lại được qua seam nghiên cứu (`study_seams.hpp`).
- Không có lệch độ lớn trọng lực: box world, `sim/mss_sim/synth.py` và `imu.gravity_mps2` cùng
  9,80665; `|f| - g` của cửa sổ nằm trong ±1,6 sigma.
- Với các dạng neo độ nghiêng bằng thông tin của cửa sổ (đầy đủ nhiễu x1, chỉ hướng, dạng chọn),
  sai số yaw cuối của `E` có trung vị 1,8 / 3,5 / 3,5 mrad so với 5,3 mrad của main (5 seed).
  Dạng quá lỏng hoặc quá chặt (x10, /10) thì không. Factor không đẩy sai số sang yaw của `E`.
- Bật kiểm tra nhất quán (đánh giá, mục 10g) không đổi trung vị nào trong bảng trên trừ hành lang
  chỉ LiDAR 60 s (38,3 xuống 37,5 m) và tỉ số max hành lang thẳng (6,64 lên 6,72); đuôi của
  hai hành lang 60 s ngắn lại (GNSS max 18,5 xuống 11,3 m; chỉ LiDAR 205 xuống 107 m), hai lần
  chạy hành lang thẳng kém đi 0,6 đến 1,3 m ở điểm cuối. Số lần gỡ nhầm ở quyết định 4.
- E-01 / E-03 (tạo lại bằng `run_integration.py`, main `cc642be`): 2,181 / 2,175 mm so với
  2,227 / 2,139 mm. Factor được thêm và giữ. Bench E-01 không đổi: p50 4,19 / 4,18 ms, p95
  5,61 / 5,88 ms (mục 10f).

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Siết prior nghiêng / bias keyframe 0 | Ghim sai độ nghiêng khi IMU có bias thật (hành lang tròn có bias: 7,5 m so với 5,2 m) |
| Factor đầy đủ (cả độ lớn), nhiễu cửa sổ | NEES 9,6 (khoảng tin cậy không chứa 0), hành lang GNSS trung vị 4,15 m |
| Nhiễu x3 | Hành lang thẳng kém main (3,85 m), giữa đường hành lang tròn x2 |
| Bật kiểm tra nhất quán (keyframe 5) | Gỡ nhầm khi đứng yên 14 % (40 % trên hành lang GNSS); thống kê khi đứng yên chồng lên khi đang chuyển động (quyết định 4) |
| Neo lại hướng trọng lực định kỳ khi IMU đứng yên | Hành lang thử không có đoạn đứng yên; để việc sau (mục 11) |

## Hệ quả

- **Phải khởi động khi đứng yên.** Factor coi cửa sổ khởi tạo là thân xe đứng yên. Một lần khởi
  động đang chuyển động mà bộ khởi tạo vẫn chấp nhận (mục 12):
  - quay đều 0,25 rad/s: cổng đứng yên chặn, ATE bằng khi tắt factor;
  - tăng tốc thẳng 6 s / 2 s: cổng cho qua, factor được giữ, ATE -1,4 % / +2,2 % so với khi tắt
    factor (test giới hạn +5 %);
  - với kiểm tra bật (keyframe 5, đánh giá): mở cổng gyro 106,7, tăng tốc 38,1 / 233, đều bị gỡ.
- **Giới hạn: các cổng không bảo đảm hết điều kiện đứng yên.** Khởi động khi đang tăng tốc đều
  theo đường thẳng, trong cảnh không quan sát trục di chuyển, không có odometry bánh xe, thì
  không thống kê nào phát hiện được:
  - cổng đứng yên cho qua (gyro gần 0, kiểm tra độ lớn chỉ là bậc hai);
  - độ nghiêng `a / g` mà cửa sổ đọc là đúng mode rỗng LiDAR không thấy, và bản đồ được dựng với
    chính độ nghiêng đó, nên phần còn lại của đồ thị đồng ý với nó.

  Không có factor thì lần chạy cũng bắt đầu với cùng độ nghiêng đó (`R_w_b` của bộ khởi tạo lấy
  từ cùng giá trị trung bình), nên factor không làm trường hợp này tệ hơn nhiều. "Khởi động khi
  đứng yên" vẫn là điều kiện tiên quyết mà các cổng không bảo đảm hết.
- **Factor sửa được:** trôi cuối đường trong hành lang chỉ IMU (tròn: trung vị 21,7 xuống 1,45 m;
  RMS 11,6 xuống 1,36 m).
- **Factor không sửa được:**
  - Các lệch giữa đường: tỉ số `|x err(t)| / sigma_DR(t)` lớn nhất có trung vị 7,1 và tối đa
    23,8, lúc 9 đến 12 s, cùng sai số nghiêng tạm thời 8 đến 11 mrad.
  - **Khoảng hở đã biết:** hành lang thẳng không đạt 3 sigma_DR (trung vị 2,72 m = 3,9 sigma_DR).
    Test thành phần giới hạn trung vị seed 42..44 bằng 2 lần giá trị đo (1,86 / 1,88 m); khi tắt
    factor các seed đó cho 6,90 / 3,52 m nên test thất bại.
- Độ nghiêng của dạng chọn còn 2 mrad sau 60 s trên hành lang chỉ LiDAR (5 seed, mục 11); nó bắt
  đầu trôi khi cửa sổ bản đồ quên keyframe đầu. ATE 60 s chỉ LiDAR qua 15 seed: trung vị 38,3 m so
  với 24,9 m của main, trong độ phân tán. Neo lại định kỳ là việc tiếp theo.
- Bộ chặn NEES một seed (`LidarFactorCovarianceConsistent`, [3, 12]) mong manh ở cả hai phía:
  qua 15 seed cả main (5,1 đến 13,3) và dạng chọn (4,7 đến 12,5) đều ra ngoài trên vài seed.

## Việc còn mở

- **Factor LiDAR với trục không quan sát** (ghi chú PR 4e ở ADR-015): trong lần đối chứng
  "x removed, no feedback", keyframe cuối của backend ở +5772 m với sigma tự báo 14 m. Nguyên
  nhân: Jacobian gần đúng của `BetweenFactor<Pose3>` và, chủ yếu, liên hệ `V(phi)^-1` của phần dư
  SE(3) đưa sai số tịnh tiến lớn dọc trục không quan sát vào các thành phần quan sát được. Chỉ
  sửa Jacobian (`se3_exact`) thì hết phân kỳ nhưng vẫn tự tin quá 16 sigma. PR 4e cho chọn ba
  dạng qua `backend.relative_factor`; ADR-021 chọn mặc định `decoupled` (-8,7 m, sigma 24,5 m
  trong cùng lần đối chứng). PR 4b đánh giá lại trên mặc định mới.
- Neo lại hướng trọng lực định kỳ ở các cửa sổ đứng yên sau này.
