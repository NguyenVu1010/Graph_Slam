# ADR-020: Hướng trọng lực của cửa sổ tĩnh vào keyframe 0 (factor hướng, nhiễu 2 lần)

- Trạng thái: đề xuất
- Ngày: 2026-10-27
- Bổ sung: ADR-017 quyết định 4 (cửa sổ khởi tạo IMU); thay đổi hợp đồng `imu::InitState`

## Bối cảnh

Chẩn đoán `docs/derivations/imu_only_observability.md` (mục 1 đến 8): trong hành lang mà LiDAR
không quan sát trục x, một độ nghiêng `theta` quanh trục ngang vuông góc với x không phân biệt
được với gia tốc `g theta` dọc x trong các factor IMU, và LiDAR chỉ neo độ nghiêng so với bản đồ
của chính nó. Neo tuyệt đối duy nhất là prior quay của keyframe 0 (`backend.prior_rot_sigma_deg`
1 độ = 17,5 mrad), lỏng hơn 42 lần so với cái cửa sổ tĩnh đo được (0,42 mrad). Khi LiDAR không
mang thông tin dọc x (ngữ nghĩa "không quan sát" của PR 4b), độ nghiêng đạt 150 đến 212 mrad và
ước lượng trôi 224 đến 316 m trong 20 s.

Cửa sổ tĩnh đo lực riêng `f = ba - R^T g`, tức là tổ hợp nghiêng / bias ngang, không tách được
hai đại lượng. Prior nghiêng chặt sẽ ghim sai độ nghiêng khi IMU có bias thật (`b / g`).

## Quyết định

1. `imu::ImuInitializer` trả thêm trong `InitState`: `acc_mean_b` (trung bình gia tốc của cửa
   sổ, m/s^2, trong b) và `acc_mean_cov` (hiệp phương sai của trung bình, mỗi trục
   `max(s_k^2, acc_noise_density^2 rate_hz) / n`). Thay đổi giao diện bổ sung.
2. `backend::Graph::add_static_gravity_factor(k, f_b, cov)` (bổ sung) thêm
   `detail::StaticGravityFactor` lên pose và bias của keyframe 0: phần dư
   `f_b - (ba - R^T g)` chiếu lên mặt phẳng vuông góc `f_b` (2 chiều: chỉ hướng, bỏ độ lớn
   `|g|` so với `ba_z`), nhiễu `static_gravity_sigma_scale^2 * cov` chiếu cùng cách
   (`backend_factors.md` §6c). Đi cùng các prior keyframe 0.
3. Pipeline thêm factor khi `backend.static_gravity_factor` bật và keyframe 0 chính là scan
   khởi tạo. Mặc định: bật, `backend.static_gravity_sigma_scale` 2,0.
4. `Graph::gnss_frame_estimate()` (bổ sung, chẩn đoán): biến `E = T_e_w` của ADR-019 sau lần
   tối ưu gần nhất.

## Bằng chứng (15 seed 42..56, `imu_only_observability.md` mục 10)

| biến thể | hành lang GNSS 60 s ATE trung vị / TB / max (m) | NEES LiDAR box trung vị / max | hành lang 20 s chỉ IMU, tròn / thẳng, sai số cuối trung vị (m) |
| --- | --- | --- | --- |
| main (tắt) | 3,03 / 9,34 / 27,9 | 7,49 / 13,3 | 21,7 / 3,56 |
| factor đầy đủ, nhiễu cửa sổ | 4,15 / 7,09 / 32,6 | 9,60 / 15,3 | 2,08 / 1,72 |
| chỉ hướng, nhiễu cửa sổ | 1,99 / 3,70 / 28,9 | 9,42 / 15,3 | 1,18 / 1,97 |
| chỉ hướng, nhiễu x2 (chọn) | 1,85 / 3,52 / 18,5 | 8,57 / 12,5 | 1,45 / 2,72 |
| chỉ hướng, nhiễu x3 | 2,14 / 7,34 / 34,8 | 7,51 / 11,5 | 2,15 / 3,85 |
| prior nghiêng 0,42 mrad, không factor | 3,00 / 5,42 / 28,2 | 5,01 / 6,84 | 1,03 / 3,01 |

- "Hành lang GNSS kém đi" ở vòng 1 phần lớn là do 5 seed: với 15 seed main có đuôi nặng
  (0,54 đến 27,9 m). Phần thật sự kém đi của factor đầy đủ: cặp (nghiêng quanh trục ngang
  vuông góc, `ba` dọc trục) là một mode rỗng của IMU khi chạy thẳng; factor chỉ cố định tổ hợp
  `g theta - ba` nên lời giải trượt dọc mode đó (seed 42: nghiêng -88 mrad cùng `ba_x`
  -0,83 m/s^2, `g * 0,088 = 0,86`), và nhiễu cửa sổ quá tự tin so với phần còn lại của graph
  (NEES LiDAR 7,5 lên 9,6).
- Không có lệch độ lớn trọng lực: box world, `sim/mss_sim/synth.py` và `imu.gravity_mps2` cùng
  9,80665; `|f| - g` của cửa sổ nằm trong ±1,6 sigma. Bỏ độ lớn vẫn cải thiện hành lang GNSS
  (trung vị 4,15 xuống 1,99 m).
- Không đẩy sai số sang yaw của `E`: sai số yaw cuối của `E` trung vị 5,3 mrad (main) so với
  1,8 đến 2,8 mrad có factor.
- Nhiễu x2 là mức nhỏ nhất giữ mọi chỉ số hành lang tốt hơn main; NEES trung vị tăng 7,5 lên
  8,6 (max giảm 13,3 xuống 12,5). x3 đưa NEES về 7,5 nhưng hành lang thẳng kém main.
- Prior nghiêng không ghép bias cho NEES thấp nhất nhưng sai với IMU có bias thật (hành lang tròn
  có bias 0,06 m/s^2: 7,5 m so với 5,2 m của factor).

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Siết prior nghiêng / bias keyframe 0 | Ghim sai độ nghiêng khi IMU có bias thật |
| Factor đầy đủ (cả độ lớn), nhiễu cửa sổ | NEES 9,6, hành lang GNSS trung vị 4,15 m |
| Nhiễu x10 | Mất phần lớn lợi ích trên hành lang chỉ IMU (tròn 8,3 m) |
| Neo lại hướng trọng lực định kỳ khi IMU đứng yên | Hành lang thử không có đoạn đứng yên; để việc sau (mục 11) |

## Hệ quả

- E-01 / E-03: 2,159 / 2,149 mm so với 2,199 / 2,105 mm của main; bench E-01 p95 tăng 5 đến 9 % (bản đồ khác 2,6 % điểm), p50 không đổi (mục 10f).
- Độ nghiêng vẫn trôi chậm qua chuỗi LiDAR trên quãng dài (hành lang 60 s, 6 đến 14 mrad sau
  60 s); neo lại định kỳ là việc tiếp theo.
