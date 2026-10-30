# ADR-025: `GnssAligner` báo sigma yaw dự đoán và, khi bật cờ, chờ đến khi sigma đủ nhỏ

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Bổ sung: ADR-019 (giá trị đầu và tâm prior của `E` lấy từ aligner), ADR-023 (dùng chung ngưỡng `gnss.align_yaw_max_sigma_deg` và sàn `gnss.align_yaw_min_var_m2`); thêm vào hợp đồng `gnss.hpp` (chỉ thêm): một overload `GnssAligner::update` nhận hiệp phương sai fix và `GnssAligner::yaw_sigma_rad()`

## Bối cảnh

Backlog G1: khi các cờ của thiết kế 1/2 tắt, `GnssAligner` căn chỉnh E-01 lệch khoảng 21° ở 13 s và không báo sigma yaw. Chẩn đoán (`docs/derivations/gnss_fusion_study.md` mục 17):

- Vị trí front-end đúng: thay chúng bằng vị trí ăng-ten thật cho cùng 21,0° (lệch 0,03°). Fix không nhiễu với vị trí front-end hay vị trí thật cho 0,00°, nên tay đòn, cách ghép thời gian và khung e đều đúng.
- Nguyên nhân là nhiễu fix trên một độ trải quá ngắn. Fix E-01 có nhiễu ngang trắng 1,0 m (tự tương quan bậc 1 là 0,02) và khai báo đúng 1,0 m. Cổng độ trải cố định 1 m mở ở 13 s, khi phía w chỉ trải 1,01 m. Sigma yaw dự đoán lúc đó là 7,6°. Phần tuyến tính của nhiễu thực tế cho 19,8° trên 21,0°, tức khoảng 2,6 sigma: một lần rút không may nhưng là thật, không phải lỗi mã.
- Cổng độ trải phía e mở nhờ chính nhiễu: 1,72 m so với độ trải thật 1,01 m.

## Quyết định

1. `GnssAligner` tính sigma yaw dự đoán ở mỗi lần khớp (`gnss.md` mục 4.1): `sigma^2 = sum var_i |a_c,i|^2 / (sum |a_c,i|^2)^2`. Phía w coi là đúng. `var_i` là trị riêng lớn của khối ngang 2x2 của hiệp phương sai khai báo, kẹp dưới tại `gnss.align_yaw_min_var_m2`. Sigma được tính trên ba tổng chạy, bộ nhớ O(1) và không cấp phát. `GnssFusion` truyền `m.cov` vào aligner. `GnssFusion::align_yaw_sigma_rad()` báo sigma này khi `align_yaw_from_track` tắt (trước đây là NaN), nên `CoreObserver::on_alignment` của các study cũng nhận được nó.
2. **Cờ** `gnss.align_check_yaw_sigma` (mặc định `true` từ G13; `false` là hành vi cũ): khi bật, aligner không trả `T_e_w` khi sigma lớn hơn `gnss.align_yaw_max_sigma_deg`, và báo lỗi "yaw sigma above threshold". Cặp fix vẫn được thêm. Ngưỡng dùng chung với thiết kế 2/2b (6°), để "sigma yaw cần để căn chỉnh" chỉ có một nghĩa.
3. Sigma **không** vào prior của `E`. Prior yaw vẫn là `backend.gnss_frame_yaw_sigma_deg` (30°). Gắn hai thứ này cần đổi `graph.hpp` (như quyết định 7 của ADR-023). Cờ chỉ chờ đến khi yaw đủ căn cứ.
4. Không đổi aligner mặc định sang track-yaw ở đây: G2 (thiết kế 3) và G10 (phần mở rộng đoạn chậm của 2b) chặn việc đó.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Tăng `align_min_spread_m` | Một dòng cấu hình | Ngưỡng theo mét không co giãn theo nhiễu fix: 1 m quá ít với fix 1 m và thừa với RTK |
| Đưa sigma vào prior yaw của `E` | Graph biết độ tin của giá trị đầu | Cần đổi `graph.hpp` (cần chủ dự án đồng ý); prior 30° đã lỏng hơn 7,6° |
| Đặt track-yaw (thiết kế 2/2b) làm mặc định | E-01 căn chỉnh lệch 0,8° với sigma 5,6° | Bị G2/G10 chặn; quyết định của chủ dự án |
| Dùng `trace / 2` thay trị riêng lớn | Gần đúng hơn với nhiễu đẳng hướng | Lạc quan khi nhiễu không đẳng hướng và yaw chưa biết; với nhiễu đẳng hướng hai cách bằng nhau |

## Hệ quả

Số đo: `docs/derivations/gnss_fusion_study.md` mục 17.

- Khi cờ tắt, đầu ra không đổi: aligner tính thêm sigma nhưng không dùng nó (`est.tum` của E-01..E-03 trùng md5 với main).
- Khi cờ bật (ngưỡng 6°): E-01 căn chỉnh ở 14,8 s, sau 4,4 m (trước: 13 s, 3,5 m), yaw lệch 15,4° với sigma 5,97° (2,6 sigma, trong 3 sigma). Có 214 factor GNSS thay vì 223. ATE E-01 0,002160 m, E-03 0,002172 m (main 0,002183 m), mọi cổng đạt. Với ngưỡng 4,5° yaw lệch 9,1° ở 18 s; với 3° lệch -1,7° ở 33,6 s. Hành lang 15 seed và box world 15 seed không đổi trên mọi seed: sigma đã dưới 6° khi độ trải đạt 1 m.
- Đơn vị (400 seed, quỹ đạo kiểu E-01): NEES yaw tại lần căn chỉnh đầu 0,95 khi cờ tắt, 0,97 khi bật. Khi tắt, 6/400 lần khởi đầu lệch quá 18°; khi bật không lần nào.
- Mặc định của cờ và ngưỡng (dùng chung với thiết kế 2/2b) do chủ dự án quyết (backlog G13, 2026-10-29): cờ **bật mặc định** với ngưỡng 6°, cả trong `config/default.yaml` lẫn mặc định của struct `GnssConfig`. Đầu ra E-01..E-03 đổi, nên baseline đổi trong PR `baseline-update` riêng (`docs/baselines/2026-10-29/`). Đặt `gnss.align_check_yaw_sigma: false` để về hành vi cũ. Mục tiêu 10° ở factor đầu cần khoảng 4,5° trên E-01.
- Hợp đồng (chỉ thêm): overload `GnssAligner::update(p_e_g, cov_e, T_w_b, p_b_g)`, `GnssAligner::yaw_sigma_rad()`, trường `GnssConfig::align_check_yaw_sigma` và khoá YAML `gnss.align_check_yaw_sigma`. Overload 3 đối số giữ nguyên; nó dùng sàn phương sai cho mọi fix.
- Giới hạn: phía w coi là đúng, nên dọc trục mà LiDAR không quan sát, sigma lạc quan. Cờ không chặn được hai lần khởi đầu lệch của hành lang: seed 51 lệch 178,9° với sigma 3,36°, seed 52 lệch 131,8° với sigma 3,35°. Việc đó thuộc thiết kế 2. Nhiễu fix được giả thiết trắng và khai báo đúng.

## Bổ sung: hiệp phương sai front-end trong sigma yaw (backlog G15, 2026-10-30)

Trạng thái: đề xuất; mặc định của cờ: tắt cho tới khi bật thiết kế 2 (phán quyết của chủ dự án 2026-10-30, backlog G19).

Bối cảnh. Giới hạn ghi ở trên: phía w được coi là đúng, nên dọc trục mà LiDAR không quan sát sigma lạc quan; hai seed lật của hành lang báo khoảng 3,4° trong khi lệch 132 đến 179°.

Quyết định:

5. **Cờ** `gnss.align_yaw_sigma_frontend_cov` (mặc định `false`, hành vi cũ, cả trong YAML lẫn struct). Khi bật, `GnssAligner` nhận hiệp phương sai tư thế front-end `P_w_b` (đúng cái mà cổng chi-square dùng; không hữu hạn thì coi bằng 0) qua overload `update(p_e_g, cov_e, T_w_b, p_b_g, P_w_b)`, và phương sai yaw cộng thêm `tổng s_i^2 / tổng |a_c,i|^2`, với `s_i^2` là trị riêng lớn của khối ngang của `J P_w_b J^T`, `J = [R_w_b, -R_w_b skew(p_b_g)]` (`gnss.md` mục 4.1).
6. **Chặn cho mọi tương quan.** Sai số front-end dọc trục suy biến là dead reckoning: một sai số tương quan (front-end hành lang chạy lùi, hệ số tỷ lệ -1 dọc trục), không phải nhiễu độc lập. `Var(tổng u_i . e_i) <= (tổng |u_i| s_i)^2 <= tổng |u_i|^2 tổng s_i^2` đúng với mọi tương quan, nên số hạng không giảm theo số fix. Coi `s_i` độc lập như nhiễu fix thì nó bị trung bình hoá: với 1000 m^2 dọc một trục chạy lùi, bản đó căn chỉnh lệch 180° với sigma 6° sau 96 s (`GnssAligner.FrontEndCovCatchesADeadReckonedAxis`). `P_w_b` là tuyệt đối (gồm phần chung mà phép trừ trọng tâm loại bỏ), nên số hạng là cận trên.

Phương án đã cân nhắc thêm:

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Cộng `s_i^2` vào phương sai từng fix (độc lập) | Cùng dạng với nhiễu fix | Bị trung bình hoá theo số fix; sai số dead reckoning tương quan; căn chỉnh lệch 180° sau 96 s |
| Mô hình bước ngẫu nhiên từ phần tăng của `P_w_b` | Đúng cho dead reckoning | `P_w_b` của pipeline là hiệp phương sai scan matching ở scan thường và marginal của backend ở keyframe; phần tăng giữa hai loại vô nghĩa |
| Bật mặc định ngay | Sigma trung thực | Đổi đầu ra E-01..E-03 và hành lang không còn căn chỉnh bằng `GnssAligner`; quyết định của chủ dự án cùng PR baseline-update |

Hệ quả (`gnss_fusion_study.md` mục 18.2):
- Khi cờ tắt: đầu ra không đổi (md5 E-01..E-03 trùng main e946ee9).
- Khi cờ bật: E-01 không đổi (`est.tum` trùng; sigma 5,968 -> 5,969°); E-03 căn chỉnh muộn hơn một fix (15,0 s, lệch 14,1° với sigma 5,94°), 215 factor thay vì 216, ATE 0,002163 m thay vì 0,002174 m; box world 15 seed không đổi trên mọi seed. Hành lang 15 seed (degeneracy 1000 m^2): front-end dọc trục mang phương sai không quan sát, nên sigma không bao giờ xuống dưới 6° và `GnssAligner` không căn chỉnh (0/15, ATE hợp nhất trung vị 24 m so với 2,0 m): seed 51 và 52 không còn báo 3,4° khi lệch 132 đến 179°, nhưng cũng không có GNSS nào vào graph. Ở đó cần bộ căn chỉnh theo vết (thiết kế 2).
- Hợp đồng (chỉ thêm): overload `GnssAligner::update(p_e_g, cov_e, T_w_b, p_b_g, P_w_b)`, trường `GnssConfig::align_yaw_sigma_frontend_cov` và khoá YAML. `GnssFusion` (nội bộ pipeline) truyền `P_w_b` của mỗi fix.
- Mặc định: tắt cho tới khi bật thiết kế 2 (chủ dự án quyết 2026-10-30, backlog G19); bật thì E-03 đổi và hành lang mất GNSS khi không dùng thiết kế 2.
