# ADR-022: Fix GNSS nhận trước khi căn chỉnh thành factor sau khi căn chỉnh, fix tĩnh vào keyframe 0

- Trạng thái: đề xuất
- Ngày: 2026-10-28

## Bối cảnh

Chẩn đoán hành lang (`docs/derivations/gnss_fusion_study.md` mục 13) cho thấy nguyên nhân chính khiến GNSS hợp nhất không tốt hơn GNSS thô là biến khung `E = T_e_w` (ADR-019). Dọc trục hành lang, LiDAR không quan sát được; `E` hấp thụ mọi độ lệch hằng dọc trục, và neo tuyệt đối duy nhất là keyframe 0 cùng chuỗi chỉ-IMU đến factor GNSS đầu tiên. Tịnh tiến của `E` vì vậy lấy từ tâm prior, tức ước lượng của aligner, khớp theo vị trí front-end mà phần dọc trục là dead reckoning IMU: ở `ref`, trung vị |`E` - truth| dọc x là 0,82 m. Bộ dò khung (`frame_truth_init`) đưa nó về 0,26 m.

Trước PR này, mọi fix nhận trước khi aligner trả về `T_e_w` bị bỏ: fix trước keyframe 0 (`EstimatorCore::on_gnss` trả về khi chưa có ước lượng), kể cả các fix của cửa sổ tĩnh khởi tạo IMU, và các fix sau keyframe 0 mà aligner chưa đủ cặp hoặc độ trải (`FixOutcome::kNotAligned`). Ở hành lang đó là khoảng 50 fix (11 fix tĩnh trong cửa sổ 2 s, thêm 5 fix tĩnh sau keyframe 0, rồi các fix của pha tăng tốc), đúng những fix mà vị trí keyframe được biết tốt nhất: keyframe 0 có prior, các keyframe đầu được IMU đặt từ trạng thái nghỉ.

Mục 13.6, mục 1 đề xuất gắn chúng thành factor sau khi căn chỉnh. ADR này chốt các chi tiết chẩn đoán để mở.

## Quyết định

1. **Cờ** `gnss.attach_prealign_fixes` (mặc định `false`, hành vi cũ) và **giới hạn bộ đệm** `gnss.prealign_max_fixes` (fix, >= 1, mặc định 128 = 25,6 s ở 5 Hz), cả hai được kiểm tra khi khởi động. Khi cờ tắt không có gì thay đổi: không cấp phát thêm, cùng đường mã (`est.tum` của E-01..E-03 trùng md5 với main).
2. **Bộ đệm** (`GnssFusion`, nội bộ pipeline): `std::vector` dự trữ `prealign_max_fixes` phần tử một lần lúc khởi tạo, không bao giờ lớn hơn (không cấp phát sau khởi tạo; test `GnssFusionPrealign.BufferBoundedNoAllocation`, `PipelineComponent.PrealignBufferBoundedNoAllocation`). Bộ đệm chỉ mở trước lần căn chỉnh đầu tiên (một lần duy nhất).
   - Fix trước keyframe 0 (chưa có pose front-end) qua `gate_fix` rồi vào bộ đệm; đầy thì bỏ fix cũ nhất (nó xa cửa sổ tĩnh nhất).
   - Fix sau keyframe 0 mà aligner chưa căn chỉnh được đi như cũ (aligner vẫn nhận nó) và được giữ thêm cùng pose front-end tại thời điểm fix và hiệp phương sai `P` mới nhất; đầy thì bỏ fix mới, để các fix tĩnh ở đầu được giữ.
   - Mọi lần bỏ được đếm (`Stats::n_gnss_prealign_dropped`).
3. **Fix tĩnh.** Khi keyframe 0 được tạo, các fix trước nó có `t_k0 - imu.init_window_s <= t <= t_k0` được gắn vào keyframe 0 không dời, với pose của keyframe 0 sau tối ưu, chỉ khi keyframe 0 chính là scan khởi tạo IMU và bánh xe không chuyển động trong cửa sổ (cùng điều kiện "đứng yên" mà ADR-020 dùng cho factor trọng lực). `ImuInitializer` đã kiểm tra cửa sổ đó là tĩnh. Fix ngoài cửa sổ, hoặc khi điều kiện không đạt, bị bỏ và đếm.
4. **Keyframe gần nhất.** Mỗi khi thêm keyframe `j` (thời điểm `t_j`), mỗi fix trong bộ đệm có `t <= t_j` chưa được gán đi về keyframe gần hơn theo thời gian giữa keyframe trước `i` và `j` (bằng nhau thì về `i`). Fix tĩnh sau keyframe 0 vì vậy cũng về keyframe 0 cho đến nửa đường tới keyframe 1.
5. **Phép dời** là phép dời front-end như hiện nay (`pipeline.md` mục 5): `p_e_g' = p_e_g + R_e_w (p_w_g(keyframe) - T_w_b(t_fix) p_b_g)`, hiệp phương sai không đổi. Về keyframe trước `i`, pose keyframe là pose tham chiếu của `i` sau tối ưu (trạng thái mà dự đoán front-end đến `t_fix` xuất phát); về `j`, là pose front-end khớp được tại `j`, như `collect()`. Nguyên nhân 2 của chẩn đoán (lỗi dời dọc trục) vẫn còn với các fix không tĩnh; với fix tĩnh phép dời bằng 0.
6. **Giải phóng.** Ở keyframe đầu tiên mà aligner đã có `T_e_w` (và ở các keyframe sau, cho fix còn chờ gán), mỗi fix đã gán qua cổng chi-square tại thời điểm fix với `T_e_w` hiện tại và pose, `P` đã lưu (như mọi fix sau), rồi thành một `add_gnss_factor` trên keyframe của nó, trước các fix trong hàng đợi thường. Fix bị cổng loại được đếm (`n_gnss_prealign_rejected`) và không tính vào `max_consecutive_rejects`. Mỗi fix được giải phóng nhiều nhất một lần.
7. **Prior của `E` và aligner không đổi.** Aligner vẫn chỉ nhận các fix sau keyframe 0 như trước (fix tĩnh trước keyframe 0 không vào aligner), nên tâm prior của `E`, cổng chi-square và phép quay khi dời giống hệt main. Thay đổi duy nhất là thêm factor; prior lỏng 10 m của `E` không cần đổi vì các factor tĩnh trên keyframe 0 neo tịnh tiến bằng đo đạc.
8. **Realign.** `gnss.max_consecutive_rejects` khởi động lại aligner nhưng không mở lại bộ đệm: fix "chưa căn chỉnh" sau realign bị bỏ như trước, và fix sớm đã giải phóng không bị thêm lần nữa (test `GnssFusionPrealign.RealignDoesNotAttachTwice`). Lý do: fix trong lúc realign là fix của một front-end đã trôi khỏi cổng; gắn ngược chúng không phải là neo khởi đầu mà mục này nhắm tới.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Đưa fix tĩnh trước keyframe 0 vào aligner | Aligner có tâm tịnh tiến tốt hơn | Đổi tâm prior và cổng của mọi fix sau; tách hai tác động khó đo. Factor đã neo tịnh tiến, aligner để mục 2 của 13.6 (yaw) xử lý |
| Gắn mọi fix sớm vào keyframe kế tiếp (luật hiện tại) | Không cần pose keyframe trước | Fix tĩnh sau keyframe 0 bị dời tới keyframe 1 qua cả pha tăng tốc: đúng lỗi dời dọc trục của nguyên nhân 2 |
| Prior chặt trên tịnh tiến của `E` tại tâm aligner (`frame_tight_pos`) | Một dòng cấu hình | Tâm mang cùng độ trôi front-end: 1,73 m, vẫn 2 lần GNSS thô (mục 13.2) |
| Bộ đệm vòng bỏ fix cũ nhất khi đầy sau keyframe 0 | Đơn giản | Mất fix tĩnh, là các fix neo tốt nhất |
| Mở lại bộ đệm sau mỗi realign | Không mất fix nào | Gắn ngược fix của front-end đã trôi; phải theo dõi fix nào đã giải phóng qua nhiều lần căn chỉnh |

## Hệ quả

Số đo: `docs/derivations/gnss_fusion_study.md` mục 14 (GTSAM, desktop WSL).

- Hành lang 120 m, 15 seed ghép cặp (`GnssCorridorStudy.Diagnosis`, biến thể `prealign`): trung vị |`E` - truth| dọc trục cuối lượt từ 0,82 m xuống 0,08 m (mục tiêu 0,26 m, mức của `frame_truth_init`). Các lần lật đồ thị từ điểm khởi đầu tốt (seed 53, 54) biến mất; hai lần lật do khởi tạo aligner (51, 52) còn, để thiết kế 2. ATE hợp nhất trung vị 1,99 xuống 1,88 m, 10/5, CI [-2,78, +0,61] m: chưa có ý nghĩa. Bốn seed kém đi (44, 45, 46, 49) vì lỗi dời front-end (nguyên nhân 2): khi `E` đã neo, các factor bị dời sai kéo keyframe thay vì kéo khung. Tốt hơn GNSS thô 6/15 (`ref` 5/15).
- Box world (`GnssCorridorStudy.BoxWorldPrealignPairs`), 15 seed ghép cặp: GNSS 0,5 m trung vị 0,01019 m cả hai phía, 9/6, CI [-0,00005, +0,00002] m; GNSS + wheel 0,00645 / 0,00640 m, 9/6, CI [-0,00004, +0,00001] m. Không đổi.
- E-01..E-03 khi cờ tắt: `est.tum` trùng md5 với main d09cbd3. Khi bật (E-01 có GNSS qua `config/sim/warehouse.yaml`): E-01 0,002198 m so với 0,002183 m (ngưỡng 0,002621 m), 62 factor trước căn chỉnh, 3 bị cổng loại; E-03 0,002187 m.
- Bench E-01, trung vị 5 lần xen kẽ, p50 / p95 mỗi scan: main 4,49 / 6,08 ms, nhánh tắt 4,45 / 5,93 ms, nhánh bật 4,40 / 6,12 ms. Trong nhiễu. Bộ nhớ khi bật: bộ đệm 128 phần tử khoảng 70 KB, mảng đầu ra 256 phần tử; không cấp phát sau khởi tạo.
- Thêm một thay đổi hợp đồng có tính cộng: `gnss::GnssConfig` có hai trường mới, `Estimator::Stats` và `bench.json` có ba bộ đếm `n_gnss_prealign_attached`, `_dropped`, `_rejected`. Không đổi chữ ký hàm nào; `graph.hpp` không đổi.
- Phán quyết mặc định: hai điều kiện để bật (box world không đổi, mục tiêu khung đạt) đều thoả, nhưng cờ vẫn tắt vì ATE hợp nhất của hành lang chưa tốt hơn có ý nghĩa và E-01 kém 0,7 %. Đề xuất bật cùng thiết kế 3 (hiệp phương sai của phép dời), nhắm đúng các seed thua ở đây, rồi đo lại cả bộ.
- Factor GNSS gắn vào keyframe cũ: iSAM2 nhận factor trên biến đã commit, không có chi phí đáng kể ở đây (một lô khoảng 50 factor một lần).
