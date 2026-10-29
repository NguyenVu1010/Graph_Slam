# ADR-023: Yaw của `T_e_w` lấy từ hướng vết GNSS so với hướng thân xe, chiều chạy từ bánh xe, không dùng vị trí front-end dọc trục

- Trạng thái: đề xuất
- Ngày: 2026-10-29 (bổ sung thiết kế 2b cùng ngày, xem cuối tài liệu)
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
- Hợp đồng (chỉ thêm): lớp `gnss::GnssTrackYawAligner` trong `gnss.hpp`; hằng `gnss::kRadPerDegree` và chín trường `GnssConfig` (`align_yaw_from_track`, `_chunk_fixes`, `_max_sigma_rad`, `_min_track_m`, `_min_coherence`, `_reverse_nsigma`, `_max_turn_rad`, `_min_var_m2`, `_min_wheel_travel_m`) cùng khoá YAML tương ứng (`_deg` trong YAML). Nội bộ pipeline: `GnssFusion::on_fix` thêm đối số `wheel_odometer_m` (số đọc tuyệt đối, mặc định NaN), `track_aligner()`, `n_alignments()`, `align_yaw_sigma_rad()`, `CoreObserver::on_alignment`. Không đổi chữ ký hàm nào của main; `graph.hpp`, `estimator.hpp` không đổi.
- Giới hạn còn lại: không có bánh xe và chuyển động lùi ròng trước khi tin cậy thì lật 180° (test ghim hành vi này); không có bánh xe và xe đỗ với GNSS trôi từ 0,1 m mỗi fix trở lên thì có thể căn chỉnh theo nhiễu (16/100 ở 0,2 m); có bánh xe thì xe chậm hơn khoảng 0,1 m/s không bao giờ căn chỉnh ở mặc định; sai số GNSS có màu chỉ được chặn bởi vết tối thiểu và độ nhất quán.
- Đề xuất mặc định: giữ tắt dù hai điều kiện của brief (mục tiêu hành lang đạt, box world không đổi) đều thoả: một mình nó kém hơn GNSS thô có ý nghĩa và làm chậm căn chỉnh khi fix nhiễu so với quỹ đạo; lợi ích chỉ có khi đi cùng thiết kế 1, mà thiết kế 1 tắt đến thiết kế 3. Bật cả hai cùng thiết kế 3 sau khi đo lại cả bộ (bộ đệm 320 đã đủ cho E-01); thiết kế 3 đo với `prealign_trackyaw` làm baseline.

## Bổ sung: thiết kế 2b, căn chỉnh với ít chuyển động hơn (2026-10-29)

Trạng thái của bổ sung: đề xuất; việc đổi mục tiêu ở quyết định 9 chờ chủ dự án quyết.

Bối cảnh. Chủ dự án thấy thiết kế 2 cần chạy thẳng quá lâu (vết tối thiểu 5 m, sigma 3°; E-01 chỉ có 35/223 factor GNSS, box world 6 đến 17 so với 38 đến 45). Phần cứng: GNSS thường (không RTK), 0,5 m mỗi trục trong mô phỏng, 5 Hz; tay đòn ăng-ten 0,22 m theo phương ngang (`p_b_g_m` [-0,20, 0,10, 0,60]); robot có odometry bánh xe. Chờ căn chỉnh không ảnh hưởng điều hướng cục bộ: tư thế LiDAR-inertial trong w hợp lệ từ khung đầu, chỉ đầu ra ENU phải chờ `T_e_w`. Số đo: `gnss_fusion_study.md` mục 16 (sau vòng sửa 1 của review, mục 16.9); phương pháp và mô hình sai số: `gnss.md` mục 5.1.

Đo trước khi thiết kế (mục 16.1). Trên đường thẳng `sigma_yaw = sqrt(2 v) / L`, nên với 0,5 m mỗi trục cần `L = 0,224 m / sigma_yaw` giữa tâm khối đầu và cuối: 4,3 m cho 3°, 2,1 m cho 6°. Ở 3° chính vết tối thiểu 5 m quyết định trên đường thẳng; trên vòng tròn là sigma. Vậy đòn bẩy theo thứ tự là ngưỡng sigma, vết tối thiểu, rồi các đoạn rẽ bị cổng bỏ đi.

Quyết định:

9. **Ngưỡng sigma 6°** (`gnss.align_yaw_max_sigma_deg`, từ 3°; giá trị mặc định của struct vẫn 3°). Graph còn tinh chỉnh yaw của `E` sau căn chỉnh (prior 30°), còn cổng chi-square và phép dời fix dùng `T_e_w` của aligner, được ước lượng lại ở mỗi khối. Lập luận sai số bị chặn: trên đường thẳng sai số vị trí do yaw ở khoảng cách `d` từ tâm khớp là `d delta_yaw`, với 3 sigma khoảng 0,34 m, nằm trong cổng (1,4 m); bộ chắn khoá (`max_consecutive_rejects`) chặn phần còn lại. Đo ở 3 / 4,5 / 6°: yaw lúc căn chỉnh đầu nằm trong sigma dự đoán ở mọi cảnh (NEES 0,66 đến 1,54 trên 15 seed, không seed nào quá 3 sigma), không lật `E`, số fix bị loại sau căn chỉnh không tăng, ATE hợp nhất không đổi hoặc tốt hơn; `T_e_w` đầu tiên bị xoay ép 18° vẫn hội tụ (`PipelineComponent.TrackYawConvergesFromEighteenDegrees`: prior của `E` lệch 20,8 / -15,2°, cuối còn 0,06°, không realign). Chọn theo yêu cầu yaw và số fix bị loại, không theo ATE (hỗn loạn, bài học 1.15). Đề xuất đổi mục tiêu "dưới 10° ở factor đầu" của mục 13.6 thành điều nó đại diện, "không lật, không khoá"; ở 6° sai số tới 3 sigma (15 đến 18°) có thể xảy ra. Chủ dự án quyết định việc đổi này; bằng chứng là cho nhiễu trắng, sai số có màu làm đuôi rộng hơn (quyết định 11).

10. **Đoạn rẽ từ đường đi của bánh xe** (`gnss.align_yaw_wheel_turns`, bật; `gnss.align_yaw_wheel_slip_frac` 0,05). Đồng hồ quãng đo tốc độ tiến của khung odometry o (`frames.T_b_o`, tâm trục bánh), nên đường đi `Q` của o được tích phân từ quãng bánh xe theo hướng của o (dây cung của cung tròn mỗi bước, đúng khi tốc độ và tốc độ quay không đổi giữa hai fix), và tay đòn ăng-ten tính từ o là `R_w_b (p_b_g - p_b_o)`. Đoạn bị cổng rẽ loại, khi mọi bước vào các fix của hai khối có dữ liệu bánh xe và mỗi bước quay không quá cổng rẽ, lấy hướng từ độ dời ăng-ten `g_k`. Tay đòn nằm trong `g_k`, nên dạng đóng không đổi. Mô hình sai số: trượt và trượt ngang bằng `slip` lần quãng, tương quan hoàn toàn trong mỗi khối (2 s), độc lập giữa các khối, chặn tam giác cho hai đoạn dùng chung khối; profile tốc độ chưa biết giữa hai fix, `(s dpsi)^2 / 12` mỗi bước, với trọng số đúng của cả hai đoạn; sai số hướng bỏ qua (dưới 0,05°). Không có bánh xe, hoặc thiếu một số đọc trong đoạn: cổng rẽ như cũ (kết quả giống từng bit). Một bước quay quá cổng rẽ (mất GNSS trong khi rẽ) không có hình dạng cung biết trước nên không dùng qua đường bánh xe. Đoạn qua cổng giữ hướng thân xe. Giới hạn: trượt ngang kéo dài suốt một vòng rẽ một chiều (robot skid-steer có tâm quay lệch) tương quan giữa các khối, ngoài mô hình: vòng 1 m ở 6°, 5° trượt ngang NEES 1,6, 10° NEES 4,3 với 24/200 lượt quá 3 sigma, lệch tới 24° (test ghim `GnssTrackYaw2b.PersistentSkidPinned`). IMU lệch khỏi trục bánh cũng làm lệch hướng thân xe của các đoạn qua cổng khi rẽ (có từ thiết kế 2).

11. **Vết tối thiểu khi có bánh xe** (`gnss.align_yaw_min_track_wheel_m`, 3 m, giá trị mặc định riêng). Khi mọi đoạn đang dùng lấy hướng từ bánh xe, vết tối thiểu là khoá này, ngược lại vẫn là `align_yaw_min_track_m` (5 m). Chọn từ đường bao sai số tuyệt đối ở 6° (mục 16.3): so với 0 m, 3 m bỏ các lượt quá 15° khi GNSS trôi 0,02 m mỗi fix, giảm một nửa số lượt quá 15° khi trôi 0,05 m mỗi fix (52 xuống 30 trên 600) và số lần căn chỉnh khi bánh quay mà thân đứng yên ở 0,20 m mỗi fix (37 xuống 16 trên 100), đổi lại thêm 2 s ở 0,5 m/s. NEES không phải tiêu chí: sigma lớn hơn làm NEES nhỏ đi trong khi đuôi sai số tuyệt đối lớn lên. Bản đầu dùng 0 m.

12. **Đối chiếu bánh xe với GNSS.** Phép thử độ nhất quán còn so vết GNSS với độ dời bánh xe dự đoán: `r >= align_yaw_min_coherence x max(sum |b_k|, sum |d_k|)`, với `d_k` là quãng bánh xe giữa hai tâm khối (đoạn theo hướng thân xe) hoặc `|g_k|` (đoạn theo đường bánh xe). Khi chạy, tỷ số gần 1; khi bánh quay mà thân đứng yên (kẹt, kê lên giá), `r` chỉ là độ trôi còn tổng quãng bánh tăng. Không cần chuyển động front-end, thứ sẽ hỏng trong hành lang suy biến. Đo (100 seed, 5 phút): 0,05 và 0,10 m mỗi fix không bao giờ căn chỉnh (bản đầu: 0 và 21/100); 0,20 m mỗi fix: 16/100 (bản đầu 64/100), ngang mức thiết kế 2 không bánh xe với xe đỗ (16/100), là giới hạn đã ghi.

13. **Không dùng độ dời front-end ở nơi LiDAR không suy biến.** Đo trên hành lang: khi tắt phát hiện suy biến (mặc định phát hành) sigma dọc trục của scan matching là 1,5 đến 4,6 cm trong khi độ dời 2 s của front-end sai 1,0 m; khi bật với `unobserved_var_m2` phát hành (0,001) nó báo 3,2 cm trong khi sai tới 5,5 m. Chỉ thiết lập 1000 m^2 của nghiên cứu mới đánh dấu trục mù. Một phép thử theo hiệp phương sai sẽ tin đúng độ dời đã làm lật seed 51 và 52 trong cấu hình phát hành. Không có bánh xe thì chiều chạy vẫn là giả thiết chạy tới (giới hạn của quyết định 4 giữ nguyên).

14. **Không dùng xoay tại chỗ.** Ăng-ten vẽ vòng bán kính `|l|` quanh gốc thân xe, nên `sigma_yaw = sigma / (|l| sqrt(N))`. Một vòng trong 10 s với 0,5 m: 19,5° ở 5 Hz của mô phỏng (N = 50), 12,9° ở 10 Hz (N = 100); RTK 2 cm: 0,50° ở 10 Hz; sai số tương quan trong vài giây (0,4 m, hằng số thời gian 5 s, cộng 0,3 m trắng): 46° (Monte Carlo 2000 lượt, test `GnssTrackYaw2b.InPlaceRotationYawObservability`). Công thức của brief `sigma / (|l| sqrt(N / 2))` bi quan gấp sqrt(2).

Phương án đã cân nhắc thêm:

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Khớp hình dạng đã biết (mục 4 trên đường w chính xác) | Ngắn hơn khoảng 15 % trên đường thẳng, nhanh hơn khoảng 30 % trên vòng 2 m (cận đo được) | Cần đường w đúng dọc vết; hành lang suy biến không có bánh xe không có đường đó; ước lượng mới cần mô hình sai số trôi riêng |
| Dùng đường bánh xe cho mọi đoạn có rẽ, kể cả đoạn qua cổng | Bỏ lệch dây cung nhỏ (0,02 đến 0,04° trên lối đi không nhiễu) | Số hạng trượt chỉ thêm phương sai ở đoạn mà hướng thân xe đã đúng trên cung đều |
| Từ chối `align_yaw_wheel_turns` khi `T_b_o` khác đơn vị | Đơn giản | Theo phán quyết: dùng `T_b_o` thay vì từ chối |
| Vết tối thiểu 0 hoặc 5 m khi có bánh xe | 0: sớm nhất; 5: đuôi nhỏ nhất | 0: đuôi sai số và trường hợp bánh kẹt kém (mục 16.3); 5: chậm thêm 4 s ở 0,5 m/s so với 3 m |
| Giữ 3° | Sai số ban đầu nhỏ hơn | Không có lợi đo được về lật, khoá hay số fix bị loại; chờ gấp đôi quãng đường |

Hệ quả (mục 16, sau vòng sửa 1):

- Hành lang có bánh xe, 15 seed: căn chỉnh sau 4,0 m (9 s) thay vì 6,0 m (13 s), sai số yaw trong sigma (NEES 0,88), không seed nào quá 10° ở factor đầu, không lật; ATE với thiết kế 1: 0,180 m so với 0,160 m, 10/5, CI [-0,080, +0,011] (chứa 0). Không có bánh xe: không đổi (5 m vẫn quyết định).
- Box world 15 seed: có bánh xe 9,0 s / 4,0 m thay vì 15,0 s / 7,0 m (37 factor GNSS thay vì 8); không bánh xe 13,0 s / 6,0 m; ATE không đổi (CI trong khoảng ±0,00005 m).
- E-01: căn chỉnh ở 24 s (9 m) thay vì 52 s (23 m), sai số 0,8° với sigma 5,6°; 170/223 factor thay vì 35; ATE 0,002196 m (E-03 0,002177 m). `GnssAligner` căn chỉnh ở 13 s nhưng lệch 21° mà không báo.
- Lối đi có góc 90° (mức unit, 200 seed): ở 3° từ 2/200 lượt căn chỉnh lên 200/200 (trung vị 7,9 m); ở 6° từ 8,9 m xuống 2,6 m.
- Khi cờ tắt: `est.tum` của E-01..E-03 trùng md5 với main 99bd5fd. Không đổi mã chạy mỗi scan (aligner chạy mỗi fix), nên không đo lại bench.
- Hợp đồng (chỉ thêm): hàm dựng `GnssTrackYawAligner(cfg, T_b_o)`, `n_wheel_turn_segments()`; ba trường `GnssConfig` (`align_yaw_wheel_turns`, `align_yaw_wheel_slip_frac`, `align_yaw_min_track_wheel_m`) và khoá YAML tương ứng; mặc định YAML `align_yaw_max_sigma_deg` 3 -> 6. Nội bộ pipeline: `GnssFusion` nhận `T_b_o`, seam test `gnss_first_align_yaw_offset_rad`.
- Đề xuất mặc định: `gnss.align_yaw_from_track` vẫn tắt như mục 15.4; khi bật thì dùng các mặc định mới.
