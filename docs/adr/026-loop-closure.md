# ADR-026: Loop closure: tìm ứng viên theo bán kính trên pose đã tối ưu, yaw từ Scan Context, xác minh bằng ICP point-to-plane, factor tương đối bọc Cauchy, chạy đơn luồng lúc keyframe

- Trạng thái: đề xuất
- Ngày: 2026-10-29; sửa đổi 2026-10-30 (cấu trúc lặp: kiểm tra nhập nhằng, cổng tuỳ chọn theo hiệp phương sai của graph, backlog C1-12, C1-16; xem "Sửa đổi 2026-10-30"). Việc chọn loop bền vững (PCM, gỡ loop) và việc tắt mặc định cổng tuyệt đối: ADR-028.
- Liên quan: ADR-004 (factor LiDAR), ADR-005 (luồng loop), ADR-015 (loop bọc Cauchy), ADR-016 (kho keyframe, dựng lại bản đồ sau loop), ADR-021 (dạng factor tương đối `decoupled`), ADR-024 (node ROS 2)

## Bối cảnh

Lát cắt C của giai đoạn đầu-cuối: phát hiện chỗ robot quay lại, xác minh, đưa vào graph, và bản đồ không bị "tường đôi". Trước PR này `core/loop` chỉ có hợp đồng giữ chỗ (`LoopConfig` ba khoá, constructor rỗng); backend đã có `Graph::add_loop_factor(LoopConstraint)` (factor tương đối theo `backend.relative_factor`, bọc Cauchy `k = 3,5485`) cùng các test B-02, B-03 và Jacobian (`EXPECT_CORRECT_FACTOR_JACOBIANS` trên factor bọc robust). Kho keyframe (ADR-016) giữ đám mây của mỗi keyframe trong khung `l`, và `Estimator::visit_map` đọc pose đã tối ưu lúc xuất.

Công thức và lý do chi tiết: `docs/derivations/loop_closure.md`.

## Quyết định

### 1. Tìm ứng viên: bán kính trên pose đã tối ưu, không có cơ sở dữ liệu mô tả toàn cục

Với keyframe mới nhất `q` (đã tối ưu, đã vào kho), ứng viên là mọi keyframe trong kho có `t_c <= t_q - min_time_gap_s` và `|p_w_c - p_w_q| <= search_radius_m`, theo pose của backend lúc đó, gần nhất trước, tối đa `max_candidates` (mặc định 1) được xác minh mỗi keyframe.

Có dùng mô tả nơi chốn, nhưng chỉ cục bộ: **Scan Context** (Kim & Kim 2018, `sc_rings x sc_sectors` = 20 x 60 ô trong khung thân đã cân bằng trọng lực, giá trị = độ cao lớn nhất + `sc_height_offset_m`) cho (a) một cổng rẻ trước ICP (`sc_max_distance`) và (b) yaw tương đối theo độ lệch cột, lượng tử một sector (6°). Không dùng Scan Context để tìm toàn cục (ring key + cây KD).

Lý do: L-01 ("cùng chỗ, khác hướng 90°, yaw lệch ≤ 1 sector") cần yaw không phụ thuộc vào yaw của graph; vùng hội tụ của ICP về yaw chỉ khoảng 10 đến 20°. Scan Context cho yaw đó với chi phí O(số điểm) + 72k phép nhân. Tìm toàn cục chưa cần cho các cảnh hiện có (trôi nhỏ hơn bán kính 10 m), mà thêm một cơ sở dữ liệu và một đường dẫn tới false loop khác.

**Giới hạn, nói thẳng:** trôi lớn hơn `search_radius_m` (10 m) thì không tìm được loop; trôi yaw hoặc vị trí lớn hơn cổng hiệu chỉnh (mục 2: 30°, 3 m) thì loop bị loại. Trong cổng đó loop **không** chắc được đóng: xem "Giới hạn đã đo" dưới đây (cấu trúc lặp, kernel Cauchy với sàn 5 cm). Loop chỉ hoạt động khi trôi còn nhỏ (cỡ vài chục cm). Tìm toàn cục nằm trong backlog (C1-1).

`sc_height_offset_m` (0,05 m) đặt dưới độ cao thân trên mặt đất (robot sim: trục bánh, 0,10 m) để điểm mặt đất là ô rỗng. Với offset 2 m (giá trị của bài báo, cho LiDAR cao 2 m), mặt đất có mặt ở mọi ô và mọi nơi giống nhau: L-02 nhận hai false loop.

### 2. Xác minh: ICP point-to-plane trên submap của ứng viên, bốn cổng, hiệp phương sai từ Hessian có sàn

- **Submap:** đám mây kho của ứng viên và `submap_keyframes` (2) láng giềng mỗi bên (chỉ những keyframe đủ cũ), chèn vào một `VoxelMap` riêng (`submap_max_voxels` 20000) tại pose đã tối ưu, xoá trước mỗi ứng viên.
- **ICP:** dùng lại `scan_match::ScanMatcher` (point-to-plane, Huber), với `icp_max_iterations` 30 và `icp_max_corr_dist_m` 2 m thay cho giá trị bám scan; chiếu suy biến tắt (cổng dưới đọc H). Hai hạt giống: pose graph của `q`, và nếu yaw Scan Context lệch yaw graph quá một sector thì cùng pose với yaw đó.
- **Cổng hiệu chỉnh:** kết quả hội tụ nào lệch pose graph quá `max_correction_m` (3 m) hoặc `max_correction_deg` (30°) bị bỏ. Trên dữ liệu E-01 (hộp chữ nhật), một hạt giống Scan Context sai 180° khớp hoàn hảo với hộp lật: hiệu chỉnh 4,9 m / 180° là một false loop thật, cổng này chặn nó.
- **Độ khớp** trên các điểm "cấu trúc" của `q` (trên mặt đất, cùng tập mà Scan Context thấy): inlier khi điểm submap gần nhất trong `inlier_dist_m` (0,5 m) và `knn_k` láng giềng cho một mặt phẳng; phần dư là khoảng cách tới mặt phẳng đó. Chấp nhận khi hội tụ, `ratio >= min_inlier_ratio` (0,6), RMS `<= max_rms_m` (0,08 m), trị riêng nhỏ nhất của H `>= min_hessian_eig` (20). Không dùng khoảng cách tới điểm gần nhất làm phần dư: đám mây kho cách nhau `map.keyframe_voxel_m` = 0,5 m nên khớp đúng vẫn cho RMS 0,09 đến 0,12 m. Không đếm điểm mặt đất: mặt đất khớp ở mọi nơi, một cặp sai cách 6 m vẫn có ratio 0,83.
- `min_hessian_eig` 20 thấp hơn `degeneracy.eig_threshold` (100, scan đầy đủ) vì đám mây kho ít điểm hơn; loop đúng trên E-01 có trị riêng nhỏ nhất 57 đến 183, hành lang không có tường cuối trong tầm (`CorridorRejectedAsDegenerate`) dưới 20.
- **Phép đo:** `T_bc_bq = T_w_bc^-1 * T_w_lq(ICP) * T_l_b`. **Hiệp phương sai:** `MatchResult::cov` (nghịch đảo Hessian, đã nhân `cov_scale` và sàn phần dư) chuyển từ `l` sang `b_q` bằng adjoint của `T_b_l`, rồi lấy sàn theo nghĩa PSD: `C' >= diag(cov_floor_m², cov_floor_rad²)` với `cov_floor_m` 0,05 m, `cov_floor_deg` 1° (tẩy trắng theo sàn, nâng trị riêng dưới 1 lên 1). Không có sàn, loop chỉ cỡ mm và cứng hơn cả chuỗi odometry, nên một loop sai lọt cổng sẽ thắng trước khi kernel kịp tác dụng.

### 3. Backend: giữ `add_loop_factor(LoopConstraint)` hiện có, Cauchy, thêm vòng cập nhật iSAM2

- Hợp đồng `graph.hpp` không đổi: `add_loop_factor(lc)` với `lc.i` (ứng viên, cũ hơn), `lc.j` (truy vấn), `lc.T_bi_bj`, `lc.cov` `[rho, phi]` khung `b_j` đúng như brief yêu cầu (`i, j, T_ij, cov`). Factor là dạng của `backend.relative_factor` (mặc định `decoupled`, ADR-021) bọc `Robust(Cauchy(3,5485), Gaussian(P C' P^T))`; hoán vị `[rho, phi] -> [phi, rho]` chỉ trong backend. `k` không thành khoá YAML: nó là phân vị chi-square 0,95 với 6 bậc tự do trong đơn vị đã tẩy trắng, không phải tham số tinh chỉnh.
- **Cauchy thay vì Huber:** ảnh hưởng của Huber giữ hằng số với phần dư lớn, nên một loop sai cỡ mét lọt cổng vẫn kéo đồ thị mãi; ảnh hưởng của Cauchy về 0. B-03: một loop ngoại lai 5 m / 29° kéo đồ thị vuông dưới 0,1 m (5 lần sigma vị trí cạnh 0,02 m); cùng cạnh đó không có kernel kéo hơn 0,1 m (đối chứng).
- **iSAM2 sau loop:** lần `optimize()` thêm loop là một bước Gauss-Newton trên phần hiệu chỉnh phi tuyến; pipeline chạy thêm `isam_extra_updates` (2) lần `optimize()` không có gì mới để iSAM2 relinearize (theo `backend.relinearize_threshold`) và giải lại. Sau đó trạng thái front-end, pose tham chiếu keyframe và bộ tiền tích phân keyframe lấy lại từ trạng thái đã tối ưu của `q`; bản đồ cache `VoxelMap` được dựng lại bắt buộc từ kho keyframe (ADR-016 quyết định 3, "backend vừa thêm loop factor"); bản đồ xuất ra (`visit_map`) đọc pose đã tối ưu lúc xuất nên tự theo loop. Sau một loop được nhận, `min_interval_keyframes` (5) keyframe kế tiếp không tìm loop. Khi lần cập nhật đầu (lần thêm loop) thành công thì loop đã nằm trong graph: dù một lần cập nhật thêm có lỗi, loop vẫn được đếm, front-end vẫn đồng bộ theo ước lượng tốt cuối cùng và việc tìm vẫn bị bỏ qua, nên cùng loop không bị thêm hai lần (`LoopComponent.ExtraUpdateFailureKeepsLoopCountedAndNotDuplicated`). Chỉ khi chính lần cập nhật đầu lỗi thì loop bị bỏ cùng các factor đang chờ.

### 4. Luồng: đơn luồng, chạy trong `run_once` lúc keyframe

Loop chạy trong `EstimatorCore::add_keyframe`, sau `optimize()` và sau khi keyframe vào kho, trên luồng gọi `run_once()`. Chưa tạo luồng loop của ADR-005.

Lý do: (a) `deterministic: true` (test, bench, E-02) cần một luồng, và chế độ nhiều luồng của ADR-005 chưa được cài (`start()` vẫn là no-op), nên một luồng loop hôm nay sẽ là luồng duy nhất ngoài luồng gọi, kéo theo khoá cho kho keyframe và graph; (b) chi phí đo được nhỏ so với ngân sách: E-01 (desktop, WSL, trung vị 5 lần xen kẽ, scan p50 / p95): main 4,91 / 6,67 ms, nhánh này cờ tắt 4,82 / 6,84 ms, cờ bật 4,86 / 6,99 ms (độ trải giữa các lần chạy khoảng 0,8 ms ở p95). Cờ bật: 62 lần tìm và 6 loop được nhận thêm khoảng 36 ms wall cho cả lần chạy 4,5 s, tức trung bình khoảng 0,6 ms mỗi lần tìm, gồm cả các lần `optimize()` thêm và dựng lại bản đồ của 6 loop. Chưa đo trên Pi5 (backlog C1-5). Ngân sách mỗi keyframe bị chặn bởi `max_candidates` (1 ứng viên, tối đa 2 lần ICP) và `min_interval_keyframes`. Khi ADR-005 được cài, bước này chuyển sang luồng loop nguyên vẹn: đầu vào là các `KeyframeView` (id, t, pose, con trỏ đám mây), đầu ra là `LoopConstraint`.

Không cấp phát trên scan không phải keyframe; trong bước loop: `loop_views_` dự trữ `map.max_keyframes` một lần, bộ đệm ứng viên chỉ lớn theo kho, submap và ScanMatcher dự trữ trong constructor; không log theo điểm.

### 5. Cờ và khoá YAML

`loop_closure.enabled` mặc định `false`. Mọi khoá mới dưới `loop_closure.*` có đơn vị trong tên hoặc chú thích và được `load_config` kiểm tra (khoảng giá trị ghi trong `core/loop/include/mss/loop/config.hpp`), rồi `LoopDetector` kiểm tra lại khi bật: `max_candidates`, `min_interval_keyframes`, `submap_keyframes`, `submap_max_voxels`, `sc_rings`, `sc_sectors`, `sc_max_range_m`, `sc_height_offset_m`, `sc_max_distance`, `icp_max_iterations`, `icp_max_corr_dist_m`, `inlier_dist_m`, `min_inlier_ratio`, `max_rms_m`, `min_hessian_eig`, `max_correction_m`, `max_correction_deg`, `cov_floor_m`, `cov_floor_deg`, `isam_extra_updates`.

### 6. Bản build không GTSAM

Backend stub không có pose keyframe (`keyframe_state` trả lỗi), nên không dựng được view nào, không tìm kiếm nào chạy và `n_loop_searches` giữ 0: loop closure là no-op ở đó, kể cả khi cờ bật.

## Giới hạn đã đo (review C1, 2026-10-29)

Hai giới hạn dưới đây được đo lúc review C1 và ghim bởi test `LoopKnownLimits.*` (`tests/component/test_loop_component.cpp`). Giới hạn 1 đã được sửa một phần bằng sửa đổi 2026-10-30 (phần còn lại ghim bởi `LoopKnownLimits.AliasingWhenDriftExceedsGraphCovariance`); giới hạn 2 là backlog C1-13.

1. **Cấu trúc lặp (aliasing), trước sửa đổi.** Trong hành lang có cặp cột giống hệt nhau cách đều P m (giá kệ trong kho), khi trôi của graph dọc hành lang vượt P/2 (và còn dưới cổng 3 m), ICP từ hạt giống graph bám sang cặp cột kề bên và loop sai qua mọi cổng: Scan Context 0,13, ratio 0,93 đến 0,94, RMS 3,7 đến 4,0 cm, trị riêng nhỏ nhất 305 đến 392 (cổng 20); sai số so với thực bằng khoảng một chu kỳ (1,98 m với P = 2 m). Trên một chuỗi lỏng (2 cm / 0,15° mỗi bước 0,5 m) loop sai đó kéo đồ thị 0,34 đến 0,64 m (70 đến 90 % cái nó đòi); trên chuỗi cứng (5 mm) chỉ vài cm. Cổng hiệu chỉnh cố định 3 m / 30° không chặn được trường hợp này: cần cổng theo hiệp phương sai tương đối của graph (Mahalanobis trên marginal chung của i, j) và kiểm tra nghiệm tốt thứ hai.
2. **Kernel Cauchy với sàn 5 cm / 1° không đóng được hiệu chỉnh lớn đúng.** Kernel không phân biệt hiệu chỉnh lớn đúng với loop sai: với hiệp phương sai ở sàn, một loop đúng đòi 0,5 / 1 / 2 m được áp dụng khoảng 10 / 3 / 1 % trên chuỗi cứng (5 mm mỗi bước, gần với chuỗi LiDAR thật mà nghịch đảo Hessian cỡ mm) và 92 / 60 / 13 % trên chuỗi lỏng (2 cm). Tức là một lần quay lại đầu tiên sau quãng dài với trôi 0,5 đến 2 m chỉ sửa được 3 đến 13 % trong các cấu hình đo được, dù được đếm là "accepted". Test hai vòng có trôi qua được chỉ vì loop đến khi trôi còn cỡ cm. Hướng sửa: cổng theo hiệp phương sai của graph rồi mới nới kernel (hoặc switchable constraints / GNC), hiệu chỉnh lại sàn theo NEES.

## Sửa đổi 2026-10-30: cấu trúc lặp (C1-12, C1-16)

Giới hạn đã đo 1 (review C1, I-1): trong hành lang có cặp cột giống nhau cách đều P m, khi trôi vượt P/2 thì một loop sai lệch khoảng P qua mọi cổng và kéo chuỗi lỏng 0,34 đến 0,64 m. Sửa đổi này thêm hai bước sau ICP, vẫn sau cờ `loop_closure.enabled` (mặc định tắt; tắt thì E-01..E-03 trùng md5). Công thức: `docs/derivations/loop_closure.md` mục 9 và 10. Review H3 (I-1) cho thấy bước A loại mọi loop đúng khi graph hơi tự tin quá mức; ADR-028 giữ nó làm tuỳ chọn tắt (`graph_covariance_gate: false`) và thêm PCM.

### A. Cổng hiệu chỉnh theo độ bất định của graph: tuỳ chọn, mặc định tắt (ADR-028)

- Backend thêm API bổ sung `Graph::relative_pose_cov(i, j)`: hiệp phương sai của `T_bi_bj` (`[rho, phi]`, nhiễu phải, khung `b_j`) từ marginal chung của `(X_i, X_j)` trên cây Bayes của iSAM2 (`backend_factors.md` mục 8b). GTSAM vẫn chỉ ở backend; module loop nhận nó qua giao diện `loop::RelativeCovarianceSource`.
- Khi `graph_covariance_gate` bật, với mỗi kết quả ICP hội tụ: `e = Log(Z_graph^-1 Z_fit)` (6 trục, gồm roll và pitch: C1-16), `S = Sigma_cq + C'_fit`, `chi2 = e^T S^-1 e`; `chi2 > correction_chi2_gate` (22,46) thì kết quả đó không thể là loop. Không lấy được marginal thì ứng viên bị loại (`kNoMarginal`); pipeline ghi log lý do và đếm `n_loops_no_marginal`.
- **Vì sao tắt mặc định (review H3, I-1).** Với chuỗi cứng (LiDAR cỡ mm), `S` bị sàn 5 cm / 1° chi phối, cổng chỉ cho qua khoảng 0,24 m theo một trục. Chỉ cần graph tự tin quá mức một chút là mọi loop đúng bị loại: test hai vòng với sigma odometry khai báo giảm một nửa cho 43 lần bị cổng chặn, 0 loop, sai số đầu–cuối vẫn 1,21 m (không có cổng: 0,55 m); trong sảnh cột, chuỗi cứng, loop đúng với 0,4 m trôi có chi2 33. Câu cũ "gần như không mất gì" là sai. NEES hộp của pipeline đã 7,5 đến 8,6 so với 6, nên cổng cần hiệu chỉnh hiệp phương sai trước (C1-3); khi cổng bật thì kernel rộng hơn (C1-13) cũng không cứu được hiệu chỉnh lớn.
- **Chi phí** khi bật: một marginal chung mỗi lần tìm, tăng theo độ sâu của `X_i` trên cây: 1,5 / 3,9 / 12,2 ms trung bình (max 3,5 / 9,4 / 29 ms) ở 61 / 300 / 1000 keyframe trên graph pose + vận tốc + bias (desktop). Khi tắt: 0.

### B. Kiểm tra nhập nhằng: nghiệm tốt thứ hai bằng ICP từ hạt giống dời (bật)

- Chọn ICP lại từ hạt giống dời thay vì so khoảng cách Scan Context thứ hai hay độ khớp tại pose dời không ICP: descriptor quá thô (vòng 0,5 đến 2 m) và độ khớp với dung sai inlier 0,5 m gần như phẳng trong hành lang (tường khớp ở mọi độ dời). ICP đưa mỗi phương án về cực tiểu của nó và chấm bằng cùng độ khớp như cổng nhận.
- Hướng: vectơ riêng ứng với trị riêng nhỏ nhất của khối ngang 2x2 của thông tin tịnh tiến (bù Schur, đã loại quay) của Hessian ICP, quay sang `w`. Hạt giống: nghiệm tốt nhất dời `+-k * ambiguity_seed_step_m` (1 m), `k = 1..ambiguity_seeds_per_side` (2), gần trước; bỏ hạt giống xa hơn `max_correction_m` khỏi pose graph.
- Đối thủ: hội tụ, qua cổng cố định, RMS `<= max_rms_m`, cách nghiệm tốt nhất hơn `ambiguity_min_separation_m` (0,5 m) hoặc `ambiguity_min_separation_deg` (5°). Tỉ số inlier của nó trên của nghiệm tốt nhất `>= ambiguity_max_ratio` (0,9) thì loop bị loại (`kAmbiguous`). Trong hành lang cột, bản sao cách một chu kỳ khớp ở 0,93 đến 0,94 của nghiệm đúng.
- Mặc định (cổng A tắt) mọi đối thủ đều được tính. Chỉ khi cổng A bật, graph mới được phân xử: đối thủ bị bỏ qua khi nó trượt cổng chi-square **và** tệ hơn nghiệm tốt nhất ít nhất `ambiguity_chi2_margin` (13,8), và các hạt giống mà graph đã loại thì bỏ qua (chi phí về 0 trên graph cứng).
- Chi phí: chỉ ứng viên đã qua mọi cổng khác mới chạy, tối đa 4 lần ICP thêm, dừng ở đối thủ đầu tiên; khi cổng A tắt không có hạt giống nào bị bỏ (số đo ở bench dưới).

### Kết quả (chi tiết trong `loop_closure.md` mục 12 và báo cáo PR)

- Cấu hình mặc định (PCM bật, cổng A tắt), hành lang cột P = 2 và 2,5 m, trôi 0 đến 2,5 m, chuỗi lỏng và chuỗi cứng của review: mọi trường hợp là nhập nhằng, không loop sai nào vào graph (lực kéo còn lại 0). Cái giá: hành lang lặp hoàn toàn không cho loop nào.
- Cổng A bật (tuỳ chọn): với hiệp phương sai nhất quán với trôi cỡ mét (sigma tương đối khoảng 0,6 m) không còn loop sai đến trôi 2 m; với chuỗi lỏng của review loop đúng ở trôi 0 đến 0,6 m được nhận, loop sai đầu tiên của review (trôi 1,1 m, P = 2) bị loại. **Giới hạn còn lại khi bật**: trôi vượt xa hiệp phương sai của graph thì bản sao thắng; trên chuỗi lỏng các loop sai đó (trôi 1,3 đến 2,5 m, cả hai chu kỳ) kéo đồ thị **0,34 đến 0,64 m**, trên chuỗi cứng chỉ trôi đúng một chu kỳ lọt (kéo khoảng 1,4 cm) (`LoopKnownLimits.GraphGateAliasingWhenDriftExceedsGraphCovariance`).
- Loop đúng: L-01..L-03, B-02/B-03, hai vòng tổng hợp 1,207 m tắt, 0,082 m bật (4 loop, 0 sai); hai vòng với sigma khai báo giảm một nửa 0,548 m (4 loop, PCM); hộp E-01 trong estimator 7 loop, ATE 8,6 mm.

### Bench (desktop WSL, trung vị 5 lần xen kẽ, cờ loop bật, ms mỗi scan)

| dữ liệu | bản | p50 | p95 | p99 | max | loop nhận / tìm |
| --- | --- | --- | --- | --- | --- | --- |
| E-01 tổng hợp | e946ee9, loop tắt | 7,89 | 11,4 | 14,2 | 33,5 | – |
| E-01 tổng hợp | e946ee9, loop bật | 8,24 | 12,6 | 33,4 | 45,8 | 6 / 61 |
| E-01 tổng hợp | nhánh, loop tắt | 7,58 | 11,2 | 14,6 | 32,6 | – |
| E-01 tổng hợp | nhánh, loop bật (nhập nhằng + PCM) | 7,77 | 12,3 | 28,8 | 44,8 | 6 / 61 |
| E-01 tổng hợp | nhánh, loop bật, không kiểm tra nhập nhằng | 7,51 | 11,6 | 30,2 | 43,3 | 6 / 61 |
| E-07 Gazebo (ghi lại ở nhánh, sau #41) | e946ee9, loop tắt | 13,7 | 25,9 | 35,0 | 51,2 | – |
| E-07 Gazebo | e946ee9, loop bật | 12,8 | 22,4 | 35,3 | 70,3 | 16 / 55 |
| E-07 Gazebo | nhánh, loop tắt | 13,9 | 26,1 | 34,4 | 51,6 | – |
| E-07 Gazebo | nhánh, loop bật (nhập nhằng + PCM) | 14,0 | 24,3 | 39,5 | 66,5 | 16 / 55 |
| E-07 Gazebo | nhánh, loop bật, không kiểm tra nhập nhằng | 12,7 | 23,6 | 36,1 | 73,8 | 16 / 55 |

Máy chịu tải của tác tử khác lúc đo (p50 loop tắt 7,6 ms trên E-01, so với 4,6 ms lúc yên), nên chỉ so trong cùng lượt xen kẽ. Với ba giới hạn của review H3 I-2 (lọc trước bằng độ khớp, giới hạn vòng ICP, tạm dừng sau nhập nhằng), kiểm tra nhập nhằng không còn chi phí đo được: max ở keyframe có loop 44,8 ms so với 43,3 ms khi tắt kiểm tra (E-01), 66,5 so với 73,8 ms (E-07); trước các giới hạn đó là 73 và 106 ms. Max khi bật loop so với khi tắt loop: 1,37 lần (E-01) và 1,29 lần (E-07), bằng e946ee9 (1,37 và 1,37): phần còn lại là chi phí có từ trước của một loop (dựng lại bản đồ, cập nhật iSAM2; C1-15). PCM chỉ tốn 0,08 đến 0,12 ms mỗi loop đề xuất (tối đa 0,28 ms với 64 loop, 61 đến 1000 keyframe). Cờ tắt: `est.tum` E-01..E-03 và E-07 trùng md5 với e946ee9. Cờ bật: `est.tum` E-01 và E-07 trùng md5 với e946ee9 và với nhánh khi tắt kiểm tra nhập nhằng (không loop đúng nào bị loại, PCM chọn mọi loop; E-07: 16 loop, 0 sai, sai số lớn nhất 3,5 cm / 0,11°; đầu–cuối ngang 1,2 cm). Pi 5 chưa đo (C1-5, C1-15).

### Khoá YAML mới (dưới `loop_closure.*`, được `load_config` và `LoopDetector` kiểm tra)

`graph_covariance_gate` (false), `correction_chi2_gate` (22,46), `ambiguity_seeds_per_side` (2; 0 tắt kiểm tra), `ambiguity_seed_step_m` (1,0 m), `ambiguity_min_separation_m` (0,5 m), `ambiguity_min_separation_deg` (5°), `ambiguity_max_ratio` (0,9), `ambiguity_chi2_margin` (13,8). Khoá PCM: ADR-028.

### Hợp đồng đổi (bổ sung)

- `graph.hpp`: `relative_pose_cov(i, j)`, `remove_loop_factor(i, j)` (ADR-028; stub: lỗi / false).
- `loop_detector.hpp`: `RelativeCovarianceSource`; overload `detect(query, keyframes, rel)` (overload cũ giữ nguyên, tương đương `rel = nullptr`; `rel` chỉ được dùng khi `graph_covariance_gate`); `LoopOutcome::kAmbiguous`, `kNoMarginal` (thêm cuối enum); `LoopResult::correction_chi2`, `ambiguity_ratio`, `n_icp_runs`.
- `loop_selector.hpp` (mới, ADR-028), `loop/config.hpp`: các khoá trên và khoá PCM.
- `estimator.hpp`: `Stats::n_loops_pcm_rejected`, `n_loops_removed`, `n_loops_no_marginal`; `bench.json`: các bộ đếm đó và `scan_process_ms_p99`, `scan_process_ms_max` (C1-15).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chỉ bán kính, ICP từ yaw graph | Đơn giản nhất | L-01 với trôi yaw 20° phụ thuộc vùng hội tụ của ICP; không có cổng rẻ trước ICP |
| Quét yaw nhiều hạt giống ICP (8 đến 12) | Không cần mô tả | 8 đến 12 lần ICP mỗi ứng viên |
| Scan Context toàn cục (ring key + cây KD) | Tìm được loop khi trôi lớn hơn bán kính | Thêm cơ sở dữ liệu và một đường dẫn false loop; chưa cần cho cảnh hiện có. Backlog C1-1 |
| Huber thay Cauchy | Lồi | Loop sai cỡ mét kéo với ảnh hưởng không giảm |
| Luồng loop theo ADR-005 ngay | Không chặn front-end | Các luồng chưa có; cần khoá kho keyframe và graph; mất tính tất định của bench |
| Độ khớp bằng khoảng cách điểm gần nhất | Rẻ | Bị chi phối bởi khoảng cách 0,5 m của đám mây kho |

## Hệ quả

- Với cờ tắt không có gì đổi: `LoopDetector` không kiểm tra, không dự trữ; `est.tum` của E-01..E-03 trùng md5 với main (mô tả PR).
- **E-07 chưa đạt trên Gazebo.** Tiêu chí của Test_CI_CD ("sai số đầu–cuối giảm khi bật loop") không đạt trên cảnh kho hàng: sai số đầu–cuối 0,078 m khi tắt, 0,081 m khi bật. Hiệu quả chỉ được chứng minh trên dữ liệu tổng hợp. Bag E-07 ghi trước #41 (đổi khung thân) nên cần ghi lại (backlog C1-4). Chi tiết: E-07 (Gazebo kho hàng, ba vòng 72 m, GNSS tắt, seed 42): front-end gần như không trôi theo phương ngang, nên loop có ít việc để sửa. Loop tắt / bật: ATE 0,0858 / 0,0852 m (chi phối bởi độ lệch z 8 cm đã biết, mục S2), sai số ngang ở điểm cuối 4,1 / 2,3 cm; với `map.window_keyframes` 10 (cửa sổ không phủ một vòng): 3,6 / 1,1 cm. 55 lần tìm, 16 loop nhận, 0 bị loại, 0 false loop (so với ground truth: sai số lớn nhất 3,4 cm và 0,10°). Bản đồ hai trường hợp đều không bị đôi. Trôi tổng hợp (`LoopComponent.TwoLapDriftClosedByLoops`, hành lang cột, 151 keyframe): sai số đầu-cuối 1,21 m khi tắt, 0,08 m khi bật, 4 loop, 0 false.
- Hộp E-01 tổng hợp (vòng tròn 1,67 m, 60 s), cờ bật: 6 loop, 0 false, ATE 2,183 lên 2,286 mm (không có trôi để sửa; sàn hiệp phương sai 5 cm cho phép loop kéo trong khoảng mm). E-03: 5 loop, 2,215 mm.
- Bộ đếm mới: `Estimator::Stats::n_loop_searches`, `n_loops_accepted`, `n_loops_rejected` (cũng trong `bench.json`). Mỗi loop được nhận ghi một dòng log `loop closure accepted: j .. t_ns .. i .. t_ns .. dp_m .. dyaw_rad ..`, dùng để đối chiếu với ground truth.
- Hợp đồng đổi (bổ sung): `loop_detector.hpp` (API mới; constructor giữ chỗ `LoopDetector(const LoopConfig&)` được giữ, chỉ kiểm tra cấu hình), `config.hpp` của loop (khoá mới), `estimator.hpp` (ba bộ đếm). `graph.hpp` không đổi.
- Việc còn mở: backlog mục "Loop closure (C1)".
