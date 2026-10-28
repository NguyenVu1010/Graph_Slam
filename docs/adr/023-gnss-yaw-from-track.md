# ADR-023: Yaw của `T_e_w` lấy từ hướng vết GNSS so với hướng thân xe, không dùng vị trí front-end dọc trục

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Bổ sung: ADR-019 (biến khung `E`, prior của nó), ADR-022 (thiết kế 1); thêm lớp `gnss::GnssTrackYawAligner` vào hợp đồng `gnss.hpp` (chỉ thêm)

## Bối cảnh

Chẩn đoán hành lang (`docs/derivations/gnss_fusion_study.md` mục 13) cho thấy hai seed (51, 52) lật yaw ngay từ khởi tạo: tại factor GNSS đầu tiên, `T_e_w` của aligner sai -178,9° và +131,8°. `GnssAligner` (`gnss.md` mục 4) khớp vị trí ăng-ten front-end trong w với fix trong e (Procrustes 2-D). Trong hành lang, LiDAR không quan sát trục dọc, nên phần dọc trục của các vị trí đó là dead reckoning IMU.

Đo trước khi thiết kế (trạng thái front-end tại từng fix trước lần căn chỉnh đầu, seed 51 và 52): hướng thân xe (gyro + phép quay LiDAR) sai dưới 0,2°, nhưng vận tốc dọc trục đổi dấu khoảng t = 10 s và đạt -1,7 m/s, trong khi thật là +0,5 m/s. Front-end chạy lùi. Brief đề xuất giải mơ hồ 180° bằng "dấu của độ dời GNSS dọc hướng"; dữ liệu cho thấy mọi dấu lấy từ chuyển động front-end đều hỏng đúng ở các seed này, nên dấu phải đến từ vết GNSS cùng một giả thiết về hướng chạy.

Mục 13.6, mục 2: ước lượng yaw từ vết GNSS so với hướng thân xe; mục tiêu là không seed nào sai quá 10° tại factor đầu tiên (`ref`: 2/15).

## Quyết định

1. **Cờ** `gnss.align_yaw_from_track` (mặc định `false`, hành vi cũ). Khi bật, `GnssFusion` dùng `gnss::GnssTrackYawAligner` thay cho `GnssAligner` (cả sau realign). Cổng chi-square, phép dời fix, prior của `E` và mọi thứ phía sau không đổi. Khi tắt, mã chạy y như cũ (`est.tum` của E-01..E-03 trùng md5 với main).

2. **Bộ ước lượng** (`gnss.md` mục 5). Mô hình: `R_e_w = Rz(yaw)` (4 bậc tự do như ADR-019); thân xe chạy dọc trục tới `x_b`.
   - Gom `m = gnss.align_yaw_chunk_fixes` fix liên tiếp (mặc định 10 = 2 s ở 5 Hz) thành một khối: vị trí fix trung bình `c_k` (e, ngang), hướng trung bình đơn vị `h_k` của `(R_w_b x_b)` ngang (w), phương sai mỗi trục `v_k` của `c_k` (trung bình phương sai ngang của các fix chia `m`).
   - Đoạn `k` nối khối `k` và `k + 1`: `b_k = c_{k+1} - c_k`, `a_k = unit(h_k + h_{k+1})`. `yaw = atan2(sum a_k x b_k, sum a_k . b_k)`: Procrustes 2-D với vectơ phía w dài đơn vị, nên mỗi đoạn có trọng số theo chiều dài GNSS của nó. Đường cong đều: dây cung giữa hai trọng tâm khối nằm dọc tiếp tuyến tại góc giữa, tức `a_k`, nên khúc cua được xử lý đúng.
   - Tay đòn ăng-ten: đoạn dùng vết ăng-ten, hướng là của thân xe. Phần quay của tay đòn `l_k` (w, biết trước) được trừ: `b'_k = b_k - Rz(yaw) l_k`, giải bằng lặp điểm bất động (4 lần) trên hai tổng chạy thêm. Không có hiệu chỉnh này, vòng tròn 2 m của box world lệch khoảng 5,7°.
   - Bộ nhớ O(1), không cấp phát, mọi lỗi là `Result` với lý do literal.

3. **Sigma và yêu cầu chuyển động.** `sigma_yaw = sqrt(sum_k v_k |a_{k-1} - a_k|^2) / |(sum a.b', sum a x b')|` (nhiễu của các trọng tâm khối, `a_{-1} = a_K = 0`; trên đường thẳng chỉ còn khối đầu và cuối: `sqrt(v_0 + v_K) / L`). Trả về `T_e_w` khi `sigma_yaw <= gnss.align_yaw_max_sigma_deg` (mặc định 3°); điều kiện này thay cho `align_min_pairs` và `align_min_spread_m` khi cờ bật. Đứng yên hoặc máy thu giữ nguyên fix cho tổng gần 0, sigma vô hạn, không có biến đổi. Chọn 3° từ mục tiêu chứ không từ ATE: 3 sigma = 9° < 10°; ở 0,5 m mỗi fix cần khoảng 4,3 m vết (2° cần 6,4 m). ATE hợp nhất không đơn điệu theo ngưỡng (mục 15.1: 2,33 / 0,60 / 0,65 m ở 2 / 3 / 4°), đó là phản ứng hỗn loạn đã thấy ở mục 13 và 14.

4. **Dấu, lùi và rẽ.** Giả thiết chạy tới cố định dấu ở đầu cửa sổ. Khi sigma đã trong ngưỡng, một đoạn mới có thành phần dọc hướng (đã quay sang e) nhỏ hơn `-gnss.align_yaw_reverse_nsigma` (mặc định 3) lần sigma của chính nó được coi là chạy lùi và vào tổng với `-a_k` (đếm trong `n_reversals()`). Trước đó, đi tới rồi lùi tự triệt tiêu trong tổng và chỉ làm chậm việc căn chỉnh. Rẽ được xử lý bằng hướng riêng của từng đoạn. Không xử lý: xe lùi ngay từ đầu sẽ căn chỉnh sai 180° (test `GnssTrackYaw.ReversalAfterAlignmentResolvedBySign` ghi lại giới hạn này); dấu tốc độ bánh xe có thể giải nó sau này.

5. **Roll/pitch và tịnh tiến.** `R_e_w = Rz(yaw)` như hiện nay, prior nghiêng chặt của `E` (ADR-019) không đổi. Tịnh tiến `p_e_w = mean(p_e_g) - R_e_w mean(p_w_g)` trên mọi fix như `GnssAligner`; nó vẫn mang độ lệch dọc trục của front-end, và ADR-022 neo tịnh tiến của `E` bằng đo đạc khi cờ của nó bật.

6. **Sigma yaw và prior của `E`.** Sigma dự đoán được báo cáo (`GnssFusion::align_yaw_sigma_rad()`, `CoreObserver::on_alignment`, dùng trong nghiên cứu), nhưng prior yaw của `E` vẫn là `backend.gnss_frame_yaw_sigma_deg` (30°). Mục 13.6 chỉ đòi gắn hai thứ này nếu còn lật trong graph; với thiết kế 1 và 2 không seed nào còn lật (`E` cuối lượt trong 0,6° trên mọi seed). Gắn nó cần truyền sigma yaw theo lần gọi qua `Graph::add_gnss_factor`, tức đổi `graph.hpp`, cần sự đồng ý của owner.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Lấy dấu 180° từ vận tốc hoặc độ dời front-end chiếu lên hướng | Xử lý được xe lùi ngay từ đầu | Đo được: front-end chạy lùi tới -1,7 m/s ở đúng seed 51, 52 trước khi căn chỉnh; dấu sai đúng nơi cần |
| Procrustes trên vị trí front-end nhưng bỏ phần dọc trục (chiếu lên hướng) | Ít thay đổi | Phần dọc trục chính là thông tin yaw trên đường thẳng; bỏ đi thì không còn gì để khớp |
| Đường thay thế từ tích phân hướng theo thời gian (tốc độ đơn vị) | Dùng mọi fix như hồi quy | Pha đứng yên và đổi tốc độ làm méo hình dạng; trên đường thẳng không hiệu quả hơn đáng kể (1,26 so với 1,41 sqrt(v)/L với 5 khối) |
| Hiệu số fix liên tiếp (không gom khối) | Đơn giản | Mỗi bước 0,1 m chìm trong nhiễu 0,7 m; tổng co về hai đầu mút với nhiễu của một fix, kém hơn trung bình khối `sqrt(m)` lần |
| Vận tốc GNSS (Doppler) | Hướng trực tiếp | `GnssMeasurement` không có vận tốc; cần đổi hợp đồng và bộ chuyển đổi |
| Ngưỡng 2° (dư biên hơn) | 3 sigma = 6° | Căn chỉnh muộn hơn (6,4 m so với 4,3 m vết), front-end trôi lâu hơn; mục tiêu 10° đã đạt ở 3° với sai lớn nhất 4,4° |
| Gắn sigma prior yaw của `E` với sigma dự đoán | Đề xuất của 13.6 | Không còn lật trong graph; cần đổi `graph.hpp` (quyết định 6) |

## Hệ quả

Số đo: `docs/derivations/gnss_fusion_study.md` mục 15 (GTSAM, desktop WSL).

- Hành lang 120 m, 15 seed ghép cặp: số seed sai yaw > 10° tại factor đầu từ 2 xuống 0 (lớn nhất 4,4°, trung vị 1,5°). Sigma dự đoán 2,55 đến 2,92° tại lần căn chỉnh đầu; sai số bình phương chuẩn hoá trung bình 0,99 (kỳ vọng 1).
- Chỉ thiết kế 2: ATE hợp nhất kém hơn `ref` (trung vị 1,99 lên 3,37 m, 5/10, CI [-9,56, +1,39]; so với GNSS thô 1/15, CI loại 0 về phía xấu). Căn chỉnh muộn hơn từ một front-end trôi lâu hơn: tịnh tiến prior của `E` sai 0,43 m tại factor đầu (`ref` 0,13 m), `E` cuối lượt lệch 2,2 m dọc trục, lỗi dời trung vị 2,0 m. Không có thiết kế 1 thì không gì neo tịnh tiến của `E` vào keyframe 0.
- Thiết kế 1 + 2: 0,60 m trung vị, lớn nhất 3,46 m (từ 37 m), 13/2 so với `ref`, CI [-13,94, -0,15]: tổ hợp đầu tiên không dùng bộ dò mà CI loại 0. So với `prealign`: 10/5, CI [-2,56, +0,03] (chạm 0). Chưa tốt hơn GNSS thô (8/15). Các seed thua `prealign` (47, 48, 54) có lỗi dời 1,6 đến 3,3 m, tức nguyên nhân 2; với phép dời đúng (bộ dò) mọi seed ≤ 2,15 m. Thiết kế 3 còn mở.
- Box world (vòng 2 m, 20 s), 15 seed ghép cặp: GNSS 0,01019 / 0,01016 m, 7/8, CI [-0,00003, +0,00001] m; GNSS + wheel 0,00645 / 0,00641 m, CI [-0,00005, +0,00000] m. Không đổi. Căn chỉnh muộn hơn: 7 đến 19 factor GNSS mỗi lượt so với 38 đến 45.
- E-01..E-03 khi cờ tắt: `est.tum` trùng md5 với main 65a04e5. Khi bật: E-01 0,002196 m (+0,013 mm), E-03 0,002169 m; chỉ 65 factor GNSS so với 223, vì fix 1,5 m trên vòng 1,67 m cần khoảng 30 s để sigma xuống 3°. Đây là cái giá của yêu cầu sigma: `GnssAligner` căn chỉnh sớm hơn nhưng với sai số không được báo cáo.
- Bench E-01 trung vị 5 lần xen kẽ, p50 / p95: main 4,37 / 5,95 ms, tắt 4,33 / 5,72 ms, bật 4,37 / 5,81 ms, cả hai cờ 4,32 / 5,96 ms. Trong nhiễu.
- Hợp đồng (chỉ thêm): lớp `gnss::GnssTrackYawAligner` trong `gnss.hpp`; bốn trường `GnssConfig` (`align_yaw_from_track`, `align_yaw_chunk_fixes`, `align_yaw_max_sigma_rad`, `align_yaw_reverse_nsigma`) và bốn khoá YAML (`..._sigma_deg` trong YAML). Nội bộ pipeline: `GnssFusion::n_alignments()`, `align_yaw_sigma_rad()`, `CoreObserver::on_alignment`. Không đổi chữ ký nào; `graph.hpp`, `estimator.hpp` không đổi.
- Đề xuất mặc định: giữ tắt dù hai điều kiện của brief (mục tiêu hành lang đạt, box world không đổi) đều thoả, vì một mình nó làm ATE hành lang kém đi và làm chậm căn chỉnh khi fix nhiễu so với quỹ đạo; lợi ích chỉ có khi đi cùng thiết kế 1, mà thiết kế 1 tắt đến thiết kế 3. Bật cả hai cùng thiết kế 3 sau khi đo lại cả bộ; thiết kế 3 đo với `prealign_trackyaw` làm baseline.
