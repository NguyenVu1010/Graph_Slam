# ADR-027: Khởi tạo IMU "đứng bằng" (`imu.init_level_start`): roll, pitch ban đầu bằng 0 và cả độ lệch của cửa sổ tĩnh là bias gia tốc

- Trạng thái: đề xuất
- Ngày: 2026-10-30
- Bổ sung: ADR-020 (cửa sổ tĩnh không tách được nghiêng và bias ngang), ADR-017 quyết định 4 (cửa sổ khởi tạo IMU); thêm vào hợp đồng `imu/config.hpp` (chỉ thêm): trường `ImuConfig::init_level_start`

## Bối cảnh

Backlog S2 / A4: trên bản ghi Gazebo, bản đồ E-04 leo khoảng 1,4 m trên 45 m; E-01 có độ lệch
đứng khoảng 9 cm chiếm gần hết ATE. Chẩn đoán: `docs/derivations/vertical_drift_study.md`.

- Cửa sổ tĩnh đo lực riêng `f = ba - R^T g`. Bộ khởi tạo đặt `ba = 0` và lấy roll, pitch từ
  `f`, nên bias ngang `b` thành độ nghiêng `b / g` (ADR-020 đã nêu). IMU Gazebo rút bias một
  lần theo seed với sigma 0,3 m/s^2 mỗi trục (hồ sơ nhiễu ADR-011). Seed 42: `b_x` 0,588 m/s^2,
  nghiêng đo được 58,5 mrad, `b_x / g` = 60,0 mrad.
- Trên đường thẳng không gì tách lại hai đại lượng. LiDAR chỉ neo nghiêng so với bản đồ của
  chính nó, nên bản đồ được dựng nghiêng và ước lượng leo `b / g` trên mỗi mét đi. Khi robot
  rẽ, bias quay theo thân còn nghiêng thì không, nên nghiêng hội tụ: E-01 về khoảng 2 mrad sau
  khúc rẽ đầu, nhưng phần bản đồ dựng trước đó vẫn lệch.
- Ablation trên bản ghi (cùng bag, chỉ đổi một thứ): trừ bias gia tốc thật khỏi IMU đưa E-04
  từ 0,955 m xuống 0,032 m sai số đứng cuối, E-01 từ ATE 0,095 m xuống 0,015 m. Chỉ trừ bias z,
  trừ bias gyro, tắt factor trọng lực hay nới prior bias không đổi kết quả; oracle sàn nhà
  (tư thế thật, `T_b_l` của cấu hình) loại sai số render LiDAR và lệch ngoại tham (độ dốc sàn
  dưới 0,3 mrad) (nghiên cứu, mục 3 và 4).

## Quyết định

1. Cờ `imu.init_level_start` (mặc định `false`, hành vi cũ). Khi bật, `ImuInitializer` coi thân
   xe đứng bằng trong cửa sổ tĩnh: `R_w_b = I` (yaw 0 như cũ) và `ba = mean(acc_b) + g_w`, tức
   cả phần lệch của lực riêng so với trọng lực là bias. `bg`, `g_w`, `acc_mean_b`,
   `acc_mean_cov` không đổi.
2. Không thêm factor mới. Prior keyframe 0 (`backend.prior_rot_sigma_deg`,
   `backend.prior_bias_acc_sigma_mps2`) đã lấy tâm tại giá trị đầu, nên nay chúng neo nghiêng
   quanh 0 và bias quanh `b`. Factor trọng lực ADR-020 khớp đúng với cặp giá trị này (phần dư 0
   tại khởi tạo) và giữ nguyên.
3. Cửa sổ có `a_z <= 0` (thân lộn ngược) trả lỗi "level start: body z not up" thay vì một bias
   cỡ 2 g.
4. **Cận cho phần ngang** (review I-1): `imu.init_level_max_bias_mps2` (m/s^2, > 0, mặc định
   0,75). Khi `hypot(a_x, a_y)` của trung bình cửa sổ vượt cận, giả thiết đứng bằng coi như
   sai (dốc, IMU gắn nghiêng): bộ khởi tạo dùng cách cũ (nghiêng từ trung bình, `ba = 0`), đặt
   `InitState::level_start_fallback` và pipeline ghi một cảnh báo (khởi tạo chỉ chạy một lần).
   - 0,75 = 2,5 lần sigma bias gia tốc mỗi trục (0,3 m/s^2, hồ sơ sim, cỡ MPU). Robot đứng bằng
     rơi vào nhánh cũ với xác suất `exp(-2,5^2 / 2)` = 4,4 % (Rayleigh). Khi bias bằng 0, cận
     cho phép dốc hay nghiêng gắn tối đa `asin(0,75 / g)` = 4,4°; dốc 5° (0,85 m/s^2) rơi về
     nhánh cũ.
   - Vì sao quay về cách cũ mà không trả lỗi thứ 7: cửa sổ trên dốc lặp lại giống hệt ở mọi scan,
     nên một lỗi sẽ chặn khởi tạo mãi và robot không có ước lượng. Cách cũ đúng với một độ nghiêng
     thật và chỉ mất phần sửa bias, tức là hành vi trước cờ; cảnh báo cho người vận hành biết giả
     thiết bị vi phạm.

Giả thiết của cờ: robot đứng trên nền bằng khi khởi tạo và IMU gắn bằng trong `b`. Độ dốc nền
hay độ nghiêng gắn IMU `theta` trở thành bias sai `g theta` và độ nghiêng sai `theta`, thay cho
`b / g` hiện nay. Nền nhà xưởng dốc 0,1 đến 0,5° (2 đến 9 mrad) nhỏ hơn nhiều so với bias MEMS
chưa hiệu chuẩn (MPU: tới 60 mg, 60 mrad).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn (ở PR này) |
| --- | --- | --- |
| Bias gia tốc hiệu chuẩn sẵn (khoá cấu hình `ba` đã biết, đo 6 mặt hay quay 180°) | Không cần giả thiết nền bằng | Cần một quy trình hiệu chuẩn và một nguồn dữ liệu cho sim (bias Gazebo đổi theo seed); quyết định của chủ dự án |
| Prior "nền bằng" trên mọi keyframe | Neo nghiêng suốt hành trình | Sai trên dốc; một mặt nghiêng bị ép thành bằng; thay đổi lớn hơn |
| Bắt đầu bằng một vòng quay tại chỗ | Tách nghiêng và bias từ dữ liệu, không giả thiết | Đổi quỹ đạo kịch bản và quy trình vận hành; quyết định của chủ dự án |
| Nới `prior_bias_acc_sigma_mps2` lên 0,3 (khớp hồ sơ sim) | Một dòng cấu hình | Đo: không đổi gì (E-04 0,955 -> 0,950 m), vì không gì quan sát được cặp nghiêng/bias trên đường thẳng |
| Sửa phía sim (giảm bias Gazebo) | E-04 hết leo | Che lỗi thay vì sửa: bias 0,3 m/s^2 là hồ sơ đã quyết (ADR-011) và cỡ của MPU thật |

## Hệ quả

Số đo: `docs/derivations/vertical_drift_study.md` mục 5.

- Cờ tắt: đầu ra không đổi (`est.tum` của E-01..E-03 trùng md5 với main 9171c48). Không đụng
  đường nóng: bộ khởi tạo chỉ chạy trước keyframe đầu.
- Cờ bật, trên bản ghi Gazebo (3 seed mỗi kịch bản): sai số đứng cuối E-04 0,30 đến 0,96 m
  xuống 0,00 đến 0,03 m; ATE E-01 0,035 đến 0,095 m xuống 0,014 đến 0,016 m; E-07 0,087 m
  xuống 0,015 m. Kết quả trùng với phép trừ bias thật (oracle) trong 1,5 cm.
- Dữ liệu tổng hợp (không có bias): 15 seed ghép cặp, mọi khoảng tin cậy chứa 0. Bốn test
  thành phần hành lang với seed cố định đổi kết quả khi cờ là mặc định (backlog S5): đó là
  hỗn loạn của hành lang suy biến, không phải tác động có hệ thống, nhưng PR đổi mặc định phải
  chọn lại seed hoặc ngưỡng của chúng.
- Hợp đồng (chỉ thêm): `ImuConfig::init_level_start`, `ImuConfig::init_level_max_bias_mps2`,
  `InitState::level_start_fallback`, khoá YAML `imu.init_level_start` và
  `imu.init_level_max_bias_mps2`, lỗi thứ 6 của `ImuInitializer::try_init`.
- Mặc định của cờ do chủ dự án quyết; bật nó đổi đầu ra E-01..E-03 nên cần PR
  `baseline-update` riêng. Trên dữ liệu thật cần đo độ nghiêng gắn IMU trước khi bật.
- Không sửa: độ nghiêng sau khởi tạo vẫn chỉ được neo bởi keyframe 0 (lessons 1.13); trôi
  ngang dọc hành lang E-04 (backlog S2, nghiên cứu mục 6) là vấn đề khác.
