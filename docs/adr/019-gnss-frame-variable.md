# ADR-019: `T_e_w` là một biến của graph, dùng chung cho mọi factor GNSS

- Trạng thái: đề xuất
- Ngày: 2026-10-27

## Bối cảnh

Quyết định 8 của ADR-017 gắn mỗi fix GNSS vào keyframe kế tiếp, và mỗi factor giữ cố định ước lượng `T_e_w` của aligner tại lúc nó được thêm (`Graph::add_gnss_factor`: "`T_e_w` là tham số cố định, không phải biến"). Trên box world, bật GNSS làm ATE kém đi 3,7 lần (0,0088 m lên 0,0322 m). Nghiên cứu `docs/derivations/gnss_fusion_study.md` đo nguyên nhân:

- Các factor GNSS kéo cứng cả quỹ đạo theo 4 bậc tự do gauge của w (x, y, z, yaw), mà prior keyframe 0 (0,1 m, 1°) để mềm. Lực kéo đến từ sai số của `T_e_w` bị đóng băng trong từng factor: yaw của aligner trên vòng tròn 2 m với fix 0,5 m có 1-sigma khoảng 3°, và mỗi factor giữ giá trị của thời điểm nó. Với `T_e_w` hoàn hảo, lực kéo là trung bình nhiễu fix, ATE còn tệ hơn (0,061 m).
- Khi ghim gauge bằng prior chặt (1 mm, 0,01°), toàn bộ thiệt hại biến mất, cả với aligner lẫn `T_e_w` hoàn hảo. Làm hiệp phương sai LiDAR nhất quán (`cov_scale` 5) không đổi gì; hiệp phương sai GNSS đúng (NEES 2,4 so với 3); việc dời fix về thời điểm keyframe không thêm sai số đáng kể.
- Lệch giữa bản đồ (không dựng lại dưới ngưỡng ADR-016) và graph làm sai số factor LiDAR tăng từ 4 mm lên 18 mm. Đối chứng dựng lại bản đồ ở mỗi keyframe đưa sai số đó về 4 mm nhưng biến dạng quỹ đạo vẫn còn (ATE 4-DoF 0,031 m), nên biến dạng đến từ chính lực kéo của GNSS trong khoảng lỏng của chuỗi LiDAR, không từ lệch bản đồ.

Với `T_e_w` chưa biết (4 bậc tự do), GNSS chỉ mang thông tin về hình dạng quỹ đạo. Trên box world thông tin đó yếu hơn chuỗi LiDAR 6 đến 14 lần, nên tối ưu là không làm xấu ước lượng chỉ LiDAR.

## Quyết định

1. Khi `backend.gnss_frame_variable: true` (mặc định trong `config/default.yaml`), `Graph` giữ một biến `E = T_e_w` (khoá `E(0)`, `Pose3`). Mọi factor GNSS là `detail::GnssFrameFactor` trên `(X_k, E)`, phần dư `r = R_e_w (R_w_b p_b_g + p_w_b) + p_e_w - p_e_g`, Jacobian theo `docs/derivations/backend_factors.md` mục 6b.
2. Lần `add_gnss_factor` hợp lệ đầu tiên tạo `E` với giá trị đầu là `T_e_w` truyền vào (ước lượng mới nhất của `gnss::GnssAligner`), cùng một prior duy nhất đặt tâm tại đó: roll/pitch chặt (`gnss_frame_tilt_sigma_deg` 0,01°, giữ `E` 4 bậc tự do và cùng trục z với w), yaw và vị trí lỏng (`gnss_frame_yaw_sigma_deg` 30°, `gnss_frame_pos_sigma_m` 10 m). Prior đi cùng các prior của keyframe 0 nên được giữ khi một lần `optimize()` lỗi. Các lần gọi sau bỏ qua đối số `T_e_w`.
3. Pipeline không đổi: aligner vẫn cấp `T_e_w` cho cổng chi-square và phép dời fix về keyframe (`pipeline.md` mục 5), và cho giá trị đầu của `E`.
4. `backend.gnss_frame_variable: false` giữ hành vi cũ (`detail::GnssPositionFactor` với `T_e_w` cố định); các giá trị mặc định của `BackendConfig` trong struct giữ hành vi cũ cho mã gọi `Graph` trực tiếp.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Giữ `T_e_w` cố định trong factor, chặn factor đến khi độ trải lớn hơn `k sigma_h` và 1-sigma yaw dự đoán nhỏ hơn ngưỡng (đề xuất ở "Việc còn mở" của ADR-017) | Không đổi backend | Chỉ hết thiệt hại khi cổng không bao giờ mở (box world 0,5 và 2 m); ở 0,1 m vẫn kém 2,9 lần. Trong hành lang suy biến, cổng làm chậm việc căn chỉnh đến khi quỹ đạo đã trôi vài mét, rồi cổng chi-square loại mọi fix: GNSS không còn sửa được gì (25 m so với 26 m chỉ LiDAR). Số đo ở mục 7 của nghiên cứu |
| Prior chặt trên keyframe 0 | Một dòng cấu hình | Chỉ che triệu chứng: GNSS vẫn kéo `T_e_w` cũ trong từng factor, và mất khả năng để GNSS sửa vị trí ban đầu khi nó thật sự sai |
| Cập nhật `T_e_w` trong mọi factor cũ mỗi khi aligner đổi | Không thêm biến | Phải xoá và thêm lại factor trong iSAM2 ở mỗi fix; vẫn dựa trên `T_e_w` khớp theo quỹ đạo front-end chứ không theo quỹ đạo đã tối ưu |
| `E` có 6 bậc tự do, không prior nghiêng | Đơn giản hơn | Roll/pitch của `E` không quan sát được tốt từ quỹ đạo gần phẳng, và w, e đều thẳng hàng trọng lực |

## Hệ quả

- Box world (GTSAM, cùng seed, `cov_scale` 4,7): GNSS ở 0,1, 0,5 và 2,0 m cho ATE 0,0090 m ở cả ba so với 0,0090 m chỉ LiDAR (`T_e_w` cố định: 0,0249 / 0,0326 / 0,0102 m); không còn lần dựng lại bản đồ nào. Test thành phần chặn GNSS-on <= 1,2 lần chỉ LiDAR. Ở `cov_scale` 1,0 (lúc chẩn đoán) các số là 0,0088 / 0,0089 / 0,0089 m so với 0,0088 m.
- Hành lang suy biến 60 s, relax 1e8 (`CorridorDriftCorrectedByGnss`, `cov_scale` 4,7): GNSS giảm ATE từ 15,4 m xuống 1,82 m (216 factor). Với `T_e_w` cố định là 0,67 m (ở `cov_scale` 1,0: 26,1 m, 1,93 m và 0,74 m). Biến `E` không có neo tuyệt đối dọc một trục thẳng trôi đều: một độ trôi đều dọc hành lang là phép tịnh tiến cứng mà `E` hấp thụ, nên vị trí tuyệt đối dọc hành lang chỉ đến từ keyframe 0 và chuỗi IMU trước factor GNSS đầu tiên; khung cố định, khớp sớm khi quỹ đạo còn ít trôi, đóng vai neo đó. Cần xem lại khi có dữ liệu thật dài.
- Graph có thêm một biến và một prior khi GNSS bật; factor GNSS là factor hai biến, nên mỗi fix nối keyframe với `E` (fill-in nhỏ trong iSAM2, chưa đo trên Pi5).
- `Graph::add_gnss_factor` đổi ngữ nghĩa của đối số `T_e_w` khi cờ bật (chỉ là giá trị đầu ở lần gọi đầu). Đây là thay đổi hợp đồng của `graph.hpp`, ghi trong PR.
- Lệch bản đồ/graph sau mỗi lần sửa dưới ngưỡng dựng lại là hệ quả đã ghi của ADR-016 (ghi chú 2026-10-27: lệch mỗi factor tối đa cỡ ngưỡng, đo được 4 đến 18 mm); đối chứng ngưỡng gần 0 cho thấy nó không phải nguồn của biến dạng.
- Khoá cổng chi-square: front-end trôi quá cổng (mất fix trong lúc LiDAR trôi) thì mọi fix sau đều bị loại. `gnss.max_consecutive_rejects` (mặc định 60 fix = 12 s ở 5 Hz) khởi động lại aligner trên front-end hiện tại (`pipeline.md` mục 5, test `CorridorOutageRealignsAfterRejects`). Việc khởi động lại chỉ dựng lại aligner (cổng chi-square và phép quay khi dời fix); nó không bao giờ khởi tạo lại hay neo lại `E`, biến chỉ được tạo một lần ở factor đầu tiên, nên các factor mới dùng chung `E` với factor trước khi mất fix. Đánh đổi với multipath: ADR-011 giữ bước multipath 3 đến 10 m trong 2 đến 10 s với covariance báo cáo vẫn danh định; nếu cửa sổ ngắn hơn thời gian giữ, aligner khởi động lại trên các fix lệch rồi đưa chúng vào graph. Vì vậy cửa sổ phải dài hơn thời gian giữ multipath dài nhất (60 fix = 12 s > 10 s); E-06 phải đánh giá hoặc chỉnh nó (ngưỡng theo thời gian, hoặc chỉ khởi động lại sau một khoảng trống fix). Cơ chế này cũng kích hoạt khi không mất fix: trong hành lang relax mặc định, front-end trôi dọc trục trong khi `P` lạc quan, cổng loại mọi fix và aligner khởi động lại 1 lần ở 60 (2 lần ở 25).
- Chưa đánh giá đầu cuối: multipath (E-06) và mất tín hiệu (E-05). Cơ chế mất tín hiệu chỉ được phủ bởi test thành phần hành lang. Vì vậy `gnss.enabled` vẫn tắt trong `config/default.yaml`; các kịch bản sim bật bằng overlay.
- Chi phí relinearize `E` (nối với mọi keyframe có fix): box world 10 phút, GNSS 0,5 m, khoảng 850 keyframe và 2800 factor GNSS, thời gian xử lý mỗi scan p50 / p99 / max 4,4 / 9,3 / 20,3 ms với biến `E`, so với 4,2 / 9,5 / 20,5 ms với `T_e_w` cố định (desktop): không có đỉnh, nên chưa cần ngưỡng relinearize riêng cho `E`. Chưa đo trên Pi5.
- Việc tiếp theo: đọc ước lượng `E` từ graph (đầu ra có toạ độ địa lý, cổng chi-square theo `E` và marginal của nó) cần thêm accessor trong `graph.hpp`, tức cần sự đồng ý của owner; kernel robust (Huber) cho `GnssFrameFactor`, đánh giá bằng E-06.
- Hiệp phương sai LiDAR: `scan_match.cov_scale` 4,7 (NEES box world 28 xuống 7,8; test `LidarFactorCovarianceConsistent`). Kết quả lại ở mục 11 của nghiên cứu: box world vẫn 1,00 lần ở mọi sigma, E-01 có GNSS 0,0022 m, hành lang relax 1e8 15,4 m xuống 1,82 m.
- Chưa giải quyết: hiệp phương sai theo hướng suy biến (PR 4b, `scan_match.unobserved_var_m2` sẽ thay `relax_factor: 1e8` trong overlay của test hành lang). Ở relax 1e4 biến `E` cho 27,8 m so với 15,7 m chỉ LiDAR (`T_e_w` cố định: 0,50 m). Đây là tính chất của biến khung, không chỉ của độ trôi IMU có sẵn: dọc một trục thẳng trôi đều, `E` không có neo tuyệt đối (xem trên), và chuỗi chỉ-IMU của hành lang lại trôi vượt xa mô hình nhiễu của nó (khoảng 25 m trong 20 s so với 0,3 m), nên cổng chi-square khoá và các lần khởi động lại không kéo được chuỗi về.
