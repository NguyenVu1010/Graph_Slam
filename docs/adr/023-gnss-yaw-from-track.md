# ADR-023: Yaw của `T_e_w` lấy từ hướng vết GNSS so với hướng thân xe, chiều chạy từ bánh xe, không dùng vị trí front-end dọc trục

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Bổ sung: ADR-019 (biến khung `E`, prior của nó), ADR-022 (thiết kế 1; mặc định `gnss.prealign_max_fixes` nâng từ 128 lên 320); thêm lớp `gnss::GnssTrackYawAligner` vào hợp đồng `gnss.hpp` (chỉ thêm)

## Bối cảnh

Chẩn đoán hành lang (`docs/derivations/gnss_fusion_study.md` mục 13) cho thấy hai seed (51, 52) lật yaw ngay từ khởi tạo: tại factor GNSS đầu tiên, `T_e_w` của aligner sai -178,9° và +131,8°. `GnssAligner` (`gnss.md` mục 4) khớp vị trí ăng-ten front-end trong w với fix trong e (Procrustes 2-D). Trong hành lang, LiDAR không quan sát trục dọc, nên phần dọc trục của các vị trí đó là dead reckoning IMU.

Đo trước khi thiết kế (trạng thái front-end tại từng fix trước lần căn chỉnh đầu, seed 51 và 52): hướng thân xe (gyro + phép quay LiDAR) sai dưới 0,2°, nhưng vận tốc dọc trục đổi dấu khoảng t = 10 s và đạt -1,7 m/s, trong khi thật là +0,5 m/s. Front-end chạy lùi. Brief đề xuất giải mơ hồ 180° bằng "dấu của độ dời GNSS dọc hướng"; dữ liệu cho thấy mọi dấu lấy từ chuyển động front-end đều hỏng đúng ở các seed này.

Nền tảng có gì: IMU MPU không có từ kế đáng tin, máy thu (mô phỏng) chỉ cho vị trí (không Doppler, không hai ăng-ten), có odometry bánh xe. Vậy nguồn yaw tuyệt đối duy nhất là hướng đi trên mặt đất (course over ground) của vết GNSS, và nguồn chiều chạy là dấu tốc độ bánh xe.

Mục 13.6, mục 2: mục tiêu là không seed nào sai quá 10° tại factor đầu tiên (`ref`: 2/15).

## Quyết định

1. **Cờ** `gnss.align_yaw_from_track` (mặc định `false`, hành vi cũ). Khi bật, `GnssFusion` dùng `gnss::GnssTrackYawAligner` thay cho `GnssAligner` (cả sau realign). Cổng chi-square, phép dời fix, prior của `E` và mọi thứ phía sau không đổi. Khi tắt, mã chạy như cũ (`est.tum` của E-01..E-03 trùng md5 với main).

2. **Bộ ước lượng** (`gnss.md` mục 5). `R_e_w = Rz(yaw)` (4 bậc tự do như ADR-019); thân xe chạy dọc trục `x_b`, tới hoặc lùi.
   - Gom `gnss.align_yaw_chunk_fixes` fix (10 = 2 s ở 5 Hz) thành khối: vị trí fix trung bình `c_k` (e), hướng trung bình `psi_k` (hướng được unwrap từ fix này sang fix kia) và độ trải của nó, tay đòn trung bình `m_k` (w), phương sai của `c_k`, quãng bánh xe có dấu của từng nửa khối.
   - Đoạn `k` nối khối `k` và `k + 1`: `b_k = c_{k+1} - c_k`, `l_k = m_{k+1} - m_k`, `a_k = s_k (cos, sin)((psi_k + psi_{k+1}) / 2)`.
   - Tay đòn ăng-ten: `b'_k = b_k - Rz(yaw) l_k`; số hạng `(R_w_bj - R_w_bi) p_b_g` chỉ phụ thuộc phép quay nên biết trước. Yaw là nghiệm dạng đóng của Procrustes 2-D: với `Z = sum conj(a) b`, `L = sum conj(a) l` (số phức), `r = -Re(L) + sqrt(|Z|^2 - Im(L)^2)`, `e^{i yaw} = Z / (r + L)`. Chính xác trên cung tròn đều với mọi tay đòn; vô nghiệm (`r <= 0`) thì báo "not enough track". (Bản đầu lặp điểm bất động 4 lần, không hội tụ khi bán kính gần độ dài tay đòn: -3,96° ở R = 0,2 m với sigma khoảng 1°.)
   - Bộ nhớ O(1), không cấp phát, mọi lỗi là `Result` với lý do literal.

3. **Cổng rẽ** `gnss.align_yaw_max_turn_deg` (45°, kiểm tra trong (0, 180]): đoạn có thay đổi hướng giữa hai khối, hoặc độ trải hướng trong một khối, vượt ngưỡng thì không dùng (`a_k = 0`). Hướng unwrap được giữ chỉ để đo đúng góc rẽ (trung bình vectơ đơn vị không phân biệt được vòng 350° với 10°). Bản đầu dùng phân giác `unit(h_k + h_{k+1})`, quay ngược khi rẽ quá pi giữa hai khối: lật 180° với sigma dưới 0,5° khi quay nhanh hơn 1,57 rad/s. 45° vẫn giữ vòng 2 m của box world (29° mỗi khối) và E-01 (34°).

4. **Chiều chạy từ bánh xe.** Khi mọi fix của nửa sau khối `k` và nửa đầu khối `k + 1` có dữ liệu bánh xe, `s_k` là dấu quãng bánh xe giữa hai trọng tâm. Quãng nhỏ hơn `gnss.align_yaw_min_wheel_travel_m` (0,2 m) là đoạn đứng yên và bị bỏ: nhiễu bánh xe lúc đứng yên có dấu ngẫu nhiên, làm `a_k` đổi dấu qua lại và phá phép rút gọn nhiễu (đo được: E-01 chỉ còn 8 factor GNSS thay vì 65 trước khi có quy tắc này). Không có dữ liệu bánh xe: giả thiết chạy tới (`s_k = +1`, đếm trong `n_no_wheel_segments()`), và khi đã tin cậy, đoạn có độ dời GNSS ngược hướng quá `gnss.align_yaw_reverse_nsigma` sigma thì lấy `s_k = -1`. `EstimatorCore` giữ một đồng hồ quãng có dấu (hình thang của các `WheelSample::v_mps` hữu hạn; mẫu không hữu hạn bị bỏ qua và đếm trong `n_wheel_nonfinite`) khi `wheel_odom.enabled`, và đưa số đọc tuyệt đối vào `GnssFusion::on_fix` (NaN khi không có mẫu bánh xe nào từ fix trước). Bộ ước lượng lấy hiệu số đọc giữa các fix nó thực sự nhận, nên quãng của fix bị loại trước đó (cổng, chi-square) được cộng dồn sang fix sau chứ không mất. Hệ quả của ngưỡng đứng yên: với bánh xe, xe chậm hơn khoảng 0,1 m/s (0,2 m mỗi 2 s giữa hai tâm khối) không bao giờ căn chỉnh ở mặc định; không có bánh xe thì cùng quỹ đạo vẫn căn chỉnh.

5. **Nhiễu và điều kiện chấp nhận.** `sigma_yaw = sqrt(sum_k v_k |a_{k-1} - a_k|^2) / |Z|` với `|Z| = |r + L|` (vì `r` và `arg(r + L)` chỉ phụ thuộc `|Z|`, nên bậc nhất `delta_yaw = Im(conj(Z) delta_Z) / |Z|^2`; bản trước chia cho `r`, lạc quan khi tay đòn quay: trên vòng 1 m với tay đòn box world `|r + L| / r` khoảng 0,92), với `v_k` từ phương sai ngang khai báo của fix, kẹp dưới tại `gnss.align_yaw_min_var_m2` (0,25 m^2); fix có phương sai khai báo `<= 0` trên một trong hai trục bị loại ("non-positive covariance"). Giả thiết nhiễu trắng, độc lập, khai báo đúng. Trả về `T_e_w` khi cả ba điều kiện thoả:
   - `r >= gnss.align_yaw_min_track_m` (5 m): sai số chung 0,3 m trên 5 m vết là 3,4°;
   - `r >= gnss.align_yaw_min_coherence * sum |b_k|` (0,5): chuyển động đều cho gần 1, máy thu đứng yên mà sai số trôi cho khoảng `1 / sqrt(K)`;
   - `sigma_yaw <= gnss.align_yaw_max_sigma_deg` (3°): 3 sigma = 9° < mục tiêu 10°.

   Ba điều kiện thay cho `align_min_pairs` và `align_min_spread_m` khi cờ bật. Xe đỗ với GNSS trôi (bước ngẫu nhiên mỗi fix cộng 0,3 m trắng, khai báo 0,5 m, 5 phút, 100 seed): ở 0,05 m mỗi fix, chỉ sigma 13/100 căn chỉnh (sai tới 180°), thêm vết tối thiểu 6/100, thêm độ nhất quán 0/100. Giới hạn đo được của bản phát hành: không có bánh xe, 0,05 m mỗi fix 0/100, 0,10 m 0/100 (probe của re-review với seed khác: 3/100, sai tới 154°), 0,20 m 16/100, sai tới 176° (probe: 27/100); có bánh xe 0/100 ở mọi mức (mọi đoạn là đứng yên). Không có tín hiệu đứng yên rẻ nào cho xe đỗ không có bánh xe: bộ dò đứng yên của IMU dùng ở keyframe 0 xét độ trải gia tốc và tốc độ góc, giống hệt khi đứng yên và khi chạy đều (trong mô phỏng cả hai chỉ là nhiễu cảm biến, 0,08 m/s^2 mỗi mẫu), nên nó cũng chặn luôn đoạn chạy thẳng mà hành lang cần. Đây là giới hạn đã ghi.

6. **Roll/pitch và tịnh tiến.** `R_e_w = Rz(yaw)` như hiện nay, prior nghiêng chặt của `E` (ADR-019) không đổi. Tịnh tiến `p_e_w = mean(p_e_g) - R_e_w mean(p_w_g)` trên mọi fix như `GnssAligner`; nó vẫn mang độ lệch dọc trục của front-end, và ADR-022 neo tịnh tiến của `E` bằng đo đạc khi cờ của nó bật.

7. **Sigma yaw và prior của `E`.** Sigma dự đoán được báo cáo (`GnssFusion::align_yaw_sigma_rad()`, `CoreObserver::on_alignment`), nhưng prior yaw của `E` vẫn là `backend.gnss_frame_yaw_sigma_deg` (30°). Với thiết kế 1 và 2 không seed nào còn lật trong graph; gắn hai thứ này cần đổi `graph.hpp`.

8. **Bộ đệm của ADR-022.** Thiết kế 2 có thể căn chỉnh muộn khi fix nhiễu so với quỹ đạo, nên mặc định `gnss.prealign_max_fixes` nâng từ 128 lên 320 (64 s ở 5 Hz, khoảng 175 KB, chỉ cấp phát khi `attach_prealign_fixes` bật). Quy tắc định cỡ: tần số GNSS x (cửa sổ khởi tạo IMU + thời gian tới lần căn chỉnh đầu). E-01 với cả hai cờ có 264 fix trước lần căn chỉnh: 256 bỏ 4, 320 không bỏ fix nào (với mã của vòng sửa 1, 128 đã bỏ 102). Khi cờ tắt, đầu ra không đổi (md5 trùng main).

## Kinh nghiệm đã có (prior art)

- **Chiều chạy từ một nguồn tốc độ có dấu.** Hệ GNSS/INS cho xe mặt đất lấy chiều chạy từ odometer hoặc tốc độ bánh xe; hướng đi trên mặt đất của GNSS chỉ cho phương, không cho chiều tới/lùi so với thân xe. Ở đây: dấu tốc độ bánh xe (quyết định 4), giả thiết chạy tới chỉ khi không có bánh xe và được đếm.
- **Chỉ dùng hướng đi trên mặt đất khi đi gần thẳng.** Khởi tạo heading từ vết GNSS thường đòi xe chạy thẳng đủ nhanh và đủ xa, vì hướng vết không đáng tin khi rẽ hoặc chậm. Ở đây: cổng rẽ theo độ (quyết định 3) và vết tối thiểu (quyết định 5).
- **Kẹp dưới phương sai GNSS khai báo và chờ quãng đường.** LIO-SAM (`mapOptmization.cpp`, `addGPSFactor`) dùng `max(noise, 1.0f)` cho phương sai của factor GPS, bỏ fix có phương sai trên `gpsCovThreshold`, chỉ thêm GPS khi keyframe đầu và cuối cách nhau ít nhất 5 m và giãn các factor GPS cách nhau 5 m. Ở đây: kẹp dưới 0,25 m^2 chỉ trong bộ ước lượng yaw và vết tối thiểu 5 m (quyết định 5); factor GNSS trong graph không đổi.
- **Tay đòn ăng-ten.** Số hạng tay đòn quay theo thân xe được trừ bằng phép quay đã biết, như trong các bộ lọc GNSS/INS chuẩn; ở đây dạng đóng thay cho lặp (quyết định 2).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Lấy dấu 180° từ vận tốc hoặc độ dời front-end chiếu lên hướng | Không cần bánh xe | Đo được: front-end chạy lùi tới -1,7 m/s ở đúng seed 51, 52 trước khi căn chỉnh |
| Chỉ giả thiết chạy tới (bản đầu) | Không cần bánh xe | Mọi chuyển động lùi ròng trước khi tin cậy (không chỉ "lùi ngay từ đầu") lật 180°; giữ làm phương án dự phòng khi không có bánh xe |
| Bộ chắn độ nhất quán để tránh lật khi lùi | Rẻ | Đi tới 3 m rồi lùi 10 m cho độ nhất quán 7/13 = 0,54: chỉ làm chậm việc lật, không ngăn được; giữ độ nhất quán cho trường hợp xe đỗ |
| Chỉ bỏ đoạn rẽ quá pi (unwrap, bản sửa đầu) | Dùng mọi đoạn | Hướng vết không đáng tin khi rẽ dù chưa tới pi (dây cung lệch theo thay đổi tốc độ); cổng 45° theo thực hành chuẩn |
| Lặp điểm bất động cho tay đòn | Đơn giản | Không hội tụ khi R gần độ dài tay đòn; dạng đóng chính xác và có điều kiện thất bại rõ |
| Vận tốc GNSS (Doppler), hai ăng-ten, từ kế | Hướng trực tiếp | Không có trên nền tảng này (MPU không có từ kế tin được, máy thu chỉ cho vị trí) |
| Gắn sigma prior yaw của `E` với sigma dự đoán | Đề xuất của 13.6 | Không còn lật trong graph; cần đổi `graph.hpp` |

## Hệ quả

Số đo: `docs/derivations/gnss_fusion_study.md` mục 15 (GTSAM, desktop WSL). Hành lang không bật odometry bánh xe (dữ liệu có mẫu bánh xe nhưng `wheel_odom.enabled` false, vì bánh xe sẽ quan sát trục hành lang), nên dùng phương án dự phòng; box world "gnss + wheel" và E-01 dùng dấu bánh xe.

- Hành lang 120 m, 15 seed ghép cặp: số seed sai yaw > 10° tại factor đầu từ 2 xuống 0 (lớn nhất 2,5°). Sigma dự đoán 2,2 đến 2,6° tại lần căn chỉnh đầu; sai số bình phương chuẩn hoá trung bình 0,68. Trên đường gần thẳng `|Z| = r`, nên vòng sửa 2 (sigma chia `|Z|`) không đổi kết quả hành lang.
- Chỉ thiết kế 2: trung vị ATE hợp nhất 1,99 lên 4,66 m, 4/11 so với `ref`, CI [-8,74, +2,57] (chứa 0; lớn nhất 37,0 xuống 15,4 m); kém hơn GNSS thô có ý nghĩa (1/15, CI [+1,74, +5,94]).
- Thiết kế 1 + 2: 1,66 m trung vị, lớn nhất 3,17 m (từ 37 m), 12/3 so với `ref`, CI [-13,95, -0,14]. So với `prealign`: 8/7, CI [-1,56, +0,35] (chứa 0). Chưa tốt hơn GNSS thô (6/15). Với phép dời đúng (bộ dò) 0,69 m, 14/1 so với `ref`. Thiết kế 3 còn mở.
- Box world (vòng 2 m, 20 s), 15 seed ghép cặp: GNSS 0,01019 / 0,01017 m, CI [-0,00003, +0,00001] m; GNSS + wheel 0,00645 / 0,00641 m, CI [-0,00004, +0,00000] m. Không đổi. Căn chỉnh muộn hơn: 6 đến 17 factor GNSS mỗi lượt so với 38 đến 45. Sai số yaw ở lần căn chỉnh đầu trên 15 seed (có bánh xe): bình phương chuẩn hoá trung bình 1,16 (khoảng 95 % của chi-square 15 bậc: [0,42, 1,83]), sau khi sigma chia `|Z|`. Với `/ r` là 1,66 trong một lần chạy trước; nguyên nhân đo được là mẫu số `r` (re-review: Monte Carlo 4000 seed cùng quỹ đạo cho 1,10 với `/ r` và 1,01 với `/ |Z|`; không tay đòn, cùng pha tăng tốc cho 0,995, nên giả thuyết dây cung lệch khi tăng tốc bị bác bỏ). Sai hướng của front-end tại các fix trước căn chỉnh: lớn nhất 0,044°, RMS 0,01 đến 0,02°, nên không thêm số hạng nhiễu hướng.
- E-01..E-03 khi cờ tắt: `est.tum` trùng md5 với main 65a04e5. Khi bật: E-01 0,002210 m, 35 factor GNSS so với 223 (sigma đúng `/ |Z|` lớn hơn trên vòng tròn E-01 nên căn chỉnh muộn hơn). Cả hai cờ (bộ đệm 320): E-01 0,002199 m, 286 factor, 251 fix trước căn chỉnh được gắn, không fix nào bị bỏ.
- Bench E-01 trung vị 5 lần xen kẽ, p50 / p95: main 4,78 / 6,46 ms, tắt 4,60 / 6,10 ms, bật 4,62 / 6,24 ms, cả hai cờ 4,54 / 6,09 ms. Trong nhiễu.
- Hợp đồng (chỉ thêm): lớp `gnss::GnssTrackYawAligner` trong `gnss.hpp`; hằng `gnss::kRadPerDegree` và chín trường `GnssConfig` (`align_yaw_from_track`, `_chunk_fixes`, `_max_sigma_rad`, `_min_track_m`, `_min_coherence`, `_reverse_nsigma`, `_max_turn_rad`, `_min_var_m2`, `_min_wheel_travel_m`) cùng khoá YAML tương ứng (`_deg` trong YAML). Nội bộ pipeline: `GnssFusion::on_fix` thêm đối số `wheel_travel_m` (mặc định NaN), `track_aligner()`, `n_alignments()`, `align_yaw_sigma_rad()`, `CoreObserver::on_alignment`. Không đổi chữ ký hàm nào của main; `graph.hpp`, `estimator.hpp` không đổi.
- Giới hạn còn lại: không có bánh xe và chuyển động lùi ròng trước khi tin cậy thì lật 180° (test ghim hành vi này); không có bánh xe và xe đỗ với GNSS trôi từ 0,1 m mỗi fix trở lên thì có thể căn chỉnh theo nhiễu (16/100 ở 0,2 m); có bánh xe thì xe chậm hơn khoảng 0,1 m/s không bao giờ căn chỉnh ở mặc định; sai số GNSS có màu chỉ được chặn bởi vết tối thiểu và độ nhất quán.
- Đề xuất mặc định: giữ tắt dù hai điều kiện của brief (mục tiêu hành lang đạt, box world không đổi) đều thoả: một mình nó kém hơn GNSS thô có ý nghĩa và làm chậm căn chỉnh khi fix nhiễu so với quỹ đạo; lợi ích chỉ có khi đi cùng thiết kế 1, mà thiết kế 1 tắt đến thiết kế 3. Bật cả hai cùng thiết kế 3 sau khi đo lại cả bộ (bộ đệm 320 đã đủ cho E-01); thiết kế 3 đo với `prealign_trackyaw` làm baseline.
