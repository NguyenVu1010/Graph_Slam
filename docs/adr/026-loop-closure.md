# ADR-026: Loop closure: tìm ứng viên theo bán kính trên pose đã tối ưu, yaw từ Scan Context, xác minh bằng ICP point-to-plane, factor tương đối bọc Cauchy, chạy đơn luồng lúc keyframe

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Liên quan: ADR-004 (factor LiDAR), ADR-005 (luồng loop), ADR-015 (loop bọc Cauchy), ADR-016 (kho keyframe, dựng lại bản đồ sau loop), ADR-021 (dạng factor tương đối `decoupled`), ADR-024 (node ROS 2)

## Bối cảnh

Lát cắt C của giai đoạn đầu-cuối: phát hiện chỗ robot quay lại, xác minh, đưa vào graph, và bản đồ không bị "tường đôi". Trước PR này `core/loop` chỉ có hợp đồng giữ chỗ (`LoopConfig` ba khoá, constructor rỗng); backend đã có `Graph::add_loop_factor(LoopConstraint)` (factor tương đối theo `backend.relative_factor`, bọc Cauchy `k = 3,5485`) cùng các test B-02, B-03 và Jacobian (`EXPECT_CORRECT_FACTOR_JACOBIANS` trên factor bọc robust). Kho keyframe (ADR-016) giữ đám mây của mỗi keyframe trong khung `l`, và `Estimator::visit_map` đọc pose đã tối ưu lúc xuất.

Công thức và lý do chi tiết: `docs/derivations/loop_closure.md`.

## Quyết định

### 1. Tìm ứng viên: bán kính trên pose đã tối ưu, không có cơ sở dữ liệu mô tả toàn cục

Với keyframe mới nhất `q` (đã tối ưu, đã vào kho), ứng viên là mọi keyframe trong kho có `t_c <= t_q - min_time_gap_s` và `|p_w_c - p_w_q| <= search_radius_m`, theo pose của backend lúc đó, gần nhất trước, tối đa `max_candidates` (mặc định 1) được xác minh mỗi keyframe.

Có dùng mô tả nơi chốn, nhưng chỉ cục bộ: **Scan Context** (Kim & Kim 2018, `sc_rings x sc_sectors` = 20 x 60 ô trong khung thân đã cân bằng trọng lực, giá trị = độ cao lớn nhất + `sc_height_offset_m`) cho (a) một cổng rẻ trước ICP (`sc_max_distance`) và (b) yaw tương đối theo độ lệch cột, lượng tử một sector (6°). Không dùng Scan Context để tìm toàn cục (ring key + cây KD).

Lý do: L-01 ("cùng chỗ, khác hướng 90°, yaw lệch ≤ 1 sector") cần yaw không phụ thuộc vào yaw của graph; vùng hội tụ của ICP về yaw chỉ khoảng 10 đến 20°. Scan Context cho yaw đó với chi phí O(số điểm) + 72k phép nhân. Tìm toàn cục chưa cần cho các cảnh hiện có (trôi nhỏ hơn bán kính 10 m), mà thêm một cơ sở dữ liệu và một đường dẫn tới false loop khác.

**Giới hạn, nói thẳng:** trôi lớn hơn `search_radius_m` (10 m) thì không tìm được loop; trôi yaw hoặc vị trí lớn hơn cổng hiệu chỉnh (mục 2: 30°, 3 m) thì loop bị loại. Tìm toàn cục nằm trong backlog (C1-1).

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
- **iSAM2 sau loop:** lần `optimize()` thêm loop là một bước Gauss-Newton trên phần hiệu chỉnh phi tuyến; pipeline chạy thêm `isam_extra_updates` (2) lần `optimize()` không có gì mới để iSAM2 relinearize (theo `backend.relinearize_threshold`) và giải lại. Sau đó trạng thái front-end, pose tham chiếu keyframe và bộ tiền tích phân keyframe lấy lại từ trạng thái đã tối ưu của `q`; bản đồ cache `VoxelMap` được dựng lại bắt buộc từ kho keyframe (ADR-016 quyết định 3, "backend vừa thêm loop factor"); bản đồ xuất ra (`visit_map`) đọc pose đã tối ưu lúc xuất nên tự theo loop. Sau một loop được nhận, `min_interval_keyframes` (5) keyframe kế tiếp không tìm loop.

### 4. Luồng: đơn luồng, chạy trong `run_once` lúc keyframe

Loop chạy trong `EstimatorCore::add_keyframe`, sau `optimize()` và sau khi keyframe vào kho, trên luồng gọi `run_once()`. Chưa tạo luồng loop của ADR-005.

Lý do: (a) `deterministic: true` (test, bench, E-02) cần một luồng, và chế độ nhiều luồng của ADR-005 chưa được cài (`start()` vẫn là no-op), nên một luồng loop hôm nay sẽ là luồng duy nhất ngoài luồng gọi, kéo theo khoá cho kho keyframe và graph; (b) chi phí đo được nhỏ so với ngân sách: E-01 (desktop, WSL, trung vị 5 lần xen kẽ, scan p50 / p95): main 4,91 / 6,67 ms, nhánh này cờ tắt 4,82 / 6,84 ms, cờ bật 4,86 / 6,99 ms (độ trải giữa các lần chạy khoảng 0,8 ms ở p95). Cờ bật: 62 lần tìm và 6 loop được nhận thêm khoảng 36 ms wall cho cả lần chạy 4,5 s, tức trung bình khoảng 0,6 ms mỗi lần tìm, gồm cả các lần `optimize()` thêm và dựng lại bản đồ của 6 loop. Chưa đo trên Pi5 (backlog C1-5). Ngân sách mỗi keyframe bị chặn bởi `max_candidates` (1 ứng viên, tối đa 2 lần ICP) và `min_interval_keyframes`. Khi ADR-005 được cài, bước này chuyển sang luồng loop nguyên vẹn: đầu vào là các `KeyframeView` (id, t, pose, con trỏ đám mây), đầu ra là `LoopConstraint`.

Không cấp phát trên scan không phải keyframe; trong bước loop: `loop_views_` dự trữ `map.max_keyframes` một lần, bộ đệm ứng viên chỉ lớn theo kho, submap và ScanMatcher dự trữ trong constructor; không log theo điểm.

### 5. Cờ và khoá YAML

`loop_closure.enabled` mặc định `false`. Mọi khoá mới dưới `loop_closure.*` có đơn vị trong tên hoặc chú thích và được `load_config` kiểm tra (khoảng giá trị ghi trong `core/loop/include/mss/loop/config.hpp`), rồi `LoopDetector` kiểm tra lại khi bật: `max_candidates`, `min_interval_keyframes`, `submap_keyframes`, `submap_max_voxels`, `sc_rings`, `sc_sectors`, `sc_max_range_m`, `sc_height_offset_m`, `sc_max_distance`, `icp_max_iterations`, `icp_max_corr_dist_m`, `inlier_dist_m`, `min_inlier_ratio`, `max_rms_m`, `min_hessian_eig`, `max_correction_m`, `max_correction_deg`, `cov_floor_m`, `cov_floor_deg`, `isam_extra_updates`.

### 6. Bản build không GTSAM

Backend stub không có pose keyframe (`keyframe_state` trả lỗi), nên không dựng được view nào, không tìm kiếm nào chạy và `n_loop_searches` giữ 0: loop closure là no-op ở đó, kể cả khi cờ bật.

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
- E-07 (Gazebo kho hàng, ba vòng 72 m, GNSS tắt, seed 42): front-end gần như không trôi theo phương ngang, nên loop có ít việc để sửa. Loop tắt / bật: ATE 0,0858 / 0,0852 m (chi phối bởi độ lệch z 8 cm đã biết, mục S2), sai số ngang ở điểm cuối 4,1 / 2,3 cm; với `map.window_keyframes` 10 (cửa sổ không phủ một vòng): 3,6 / 1,1 cm. 55 lần tìm, 16 loop nhận, 0 bị loại, 0 false loop (so với ground truth: sai số lớn nhất 3,4 cm và 0,10°). Bản đồ hai trường hợp đều không bị đôi. Trôi tổng hợp (`LoopComponent.TwoLapDriftClosedByLoops`, hành lang cột, 151 keyframe): sai số đầu-cuối 1,21 m khi tắt, 0,08 m khi bật, 4 loop, 0 false.
- Hộp E-01 tổng hợp (vòng tròn 1,67 m, 60 s), cờ bật: 6 loop, 0 false, ATE 2,183 lên 2,286 mm (không có trôi để sửa; sàn hiệp phương sai 5 cm cho phép loop kéo trong khoảng mm). E-03: 5 loop, 2,215 mm.
- Bộ đếm mới: `Estimator::Stats::n_loop_searches`, `n_loops_accepted`, `n_loops_rejected` (cũng trong `bench.json`). Mỗi loop được nhận ghi một dòng log `loop closure accepted: j .. t_ns .. i .. t_ns .. dp_m .. dyaw_rad ..`, dùng để đối chiếu với ground truth.
- Hợp đồng đổi (bổ sung): `loop_detector.hpp` (API mới), `config.hpp` của loop (khoá mới), `estimator.hpp` (ba bộ đếm). `graph.hpp` không đổi.
- Việc còn mở: backlog mục "Loop closure (C1)".
