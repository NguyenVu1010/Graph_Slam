# ADR-007: Cấu trúc bản đồ: voxel hash map

- Trạng thái: đề xuất
- Ngày: 2026-10-23

## Bối cảnh

Scan matching point-to-plane cần kNN cho khoảng 10.000 điểm mỗi scan ở 10 Hz trên Pi5, bản đồ lớn dần khi robot đi xa nhưng RAM dưới 2 GB (PF-02), và chèn điểm mới không được cấp phát trên đường nóng. Test M-01 đòi kNN trùng vét cạn, M-02 đòi bộ nhớ bị chặn.

## Quyết định

- Bản đồ trong khung `w` là **bảng băm voxel** (kiểu iVox của Faster-LIO): khoá là toạ độ voxel nguyên `(i, j, k) = floor(p_w / voxel_m)`, giá trị là một mảng cố định tối đa `max_points_per_voxel` điểm.
- Bảng băm địa chỉ mở, dung lượng reserve một lần theo `max_voxels` trong YAML; đầy thì chèn bị từ chối và cảnh báo, không cấp phát thêm.
- kNN: duyệt 27 voxel lân cận của điểm truy vấn, gom ứng viên, chọn k gần nhất; `k` và bán kính tối đa trong YAML. Mặt phẳng cục bộ fit từ k điểm đó, loại nếu RMS vượt ngưỡng.
- Chèn: mỗi voxel giữ tối đa N điểm, điểm mới thay điểm cũ nhất khi đầy (vòng tròn), nên bản đồ tự làm mới theo thời gian.
- Cắt tỉa: ngoài đường nóng, theo chu kỳ, xoá voxel ngoài bán kính `prune_radius_m` quanh robot. Loop closure (giai đoạn 2) dựng lại bản đồ cục bộ từ keyframe đã hiệu chỉnh thay vì dịch từng voxel.
- Mặc định khởi đầu: `voxel_m = 0.5`, `max_points_per_voxel = 20`, `k = 5`, `prune_radius_m = 100`. Chỉnh theo bench.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| KdTree (PCL/nanoflann) dựng lại mỗi scan | kNN chính xác, đơn giản | Dựng lại O(n log n) mỗi scan quá chậm trên Pi5 khi bản đồ lớn; cấp phát heap |
| ikd-Tree (FAST-LIO2) | kNN chính xác, cập nhật tăng dần | Cài đặt phức tạp, cân bằng lại cây có đỉnh độ trễ; nhiều cấp phát |
| Octree | Phân cấp, nén tốt | Truy vấn lân cận chậm hơn băm ở độ phân giải cố định |
| Giữ toàn bộ keyframe cloud, kNN trên cloud gần nhất | Đơn giản | Trùng lặp điểm, không chặn bộ nhớ |

## Hệ quả

kNN gần đúng (chỉ trong 27 voxel) nhưng đủ cho point-to-plane với voxel 0.5 m; test M-01 phải định nghĩa "trùng vét cạn" trong bán kính 1 voxel. Bộ nhớ bị chặn chặt bởi `max_voxels × max_points_per_voxel`. Đổi lại, độ phân giải bản đồ cố định theo voxel; môi trường rất rộng cần voxel to hơn và mất chi tiết.

## Bổ sung 2026-10-27: khoảng cách điểm trong voxel và hai cổng cho mặt phẳng fit

**Bối cảnh.** Ở cấu hình phát hành (`max_points_per_voxel` 20, kNN 5 điểm, scan downsample 0.5 m bằng `voxel_m`), có hai triệu chứng:
- S-02 lệch 9.5 mm / 0.08°. Ở các voxel cạnh hộp, hai mặt cùng nằm trong một voxel và vòng ghi đè giữ 20 điểm mới nhất, nên lân cận 5-NN là hai cột điểm đồng phẳng trên hai mặt. Mặt fit ra là mặt chéo.
- Trong hành lang, H nhận khoảng 1.1 % thông tin giả dọc trục, đến từ các lân cận vắt qua cạnh tường–sàn.

**Quyết định.** Thêm ba tham số, cả ba mặc định **0 (tắt)**. Hành vi phát hành giống hệt main: `est.tum` của E-01 có md5 trùng.
- `map.min_point_dist_m` (m): điểm mới cách một điểm đã lưu trong voxel ít hơn khoảng này sẽ thay điểm gần nhất tại chỗ. `Estimator::Stats` / bench.json có thêm `n_map_points` và `n_map_spacing_replaced`.
- `scan_match.plane_min_eig_ratio`, trong [0, 1): loại lân cận khi λ_min > tỉ số × λ_mid (cạnh, góc).
- `scan_match.min_mid_eig_ratio`, trong [0, 1): loại lân cận khi λ_mid ≤ tỉ số × λ_max (đường thẳng, dải mỏng).

**Phương án đã cân nhắc** (đo trên 0d1df78; bảng đầy đủ ở `docs/derivations/scan_match.md` mục 3 và `docs/derivations/map.md` mục Đánh giá):

| Phương án | Kết quả | Vì sao không bật |
| --- | --- | --- |
| Cổng phẳng 0.05 + cổng thẳng hàng 0.2 | S-02 0.28 mm; thông tin giả 0.04 % | Trượt `CorridorDriftCorrectedByGnss` và ngưỡng NEES của bản không-GTSAM; trôi hành lang hẹp ×20 |
| Cổng phẳng 0.05 | E-01 1.37 mm | Trượt `CorridorDriftCorrectedByGnss` (144 factor GNSS) |
| Cổng phẳng 0.01 | E-01 2.20 → 0.95 mm, qua mọi ngưỡng trên dữ liệu mô phỏng | Chỉ đúng với dữ liệu không nhiễu (xem bên dưới) |
| Khoảng cách 0.1 m | S-02 0.38 mm; ít hơn khoảng 30 % điểm | E-01 tăng khi các cổng tắt (2.20 → 3.11 mm) |

**Lưu ý về nhiễu.** Tỉ số λ_min/λ_mid so nhiễu range với độ trải của mảng.
- Với 5 điểm trong đĩa bán kính 0.19 m, nhiễu 1 cm, cổng 0.01 loại khoảng một nửa các mảng thật sự phẳng.
- Với nhiễu 2–3 cm, cổng loại 81–90 %.
- Mọi dữ liệu dùng để chọn đều không có nhiễu range.
- Trên lần chạy ov_plain (bản đồ không lý tưởng), cổng 0.01 cho ATE 14.8 → 24.0 mm và p95 6.1 → 11.5 ms. Tỉ lệ inlier trung bình mỗi scan giảm từ 0.99 xuống 0.73. Ở 0.05: 14.5 mm, 5.5 ms.

**Lưu ý về hành lang.** Việc giảm thông tin giả dọc trục chỉ là chuyện điều kiện số của H, không phải tăng độ chính xác. Với cổng 0.01, ATE hợp nhất trong hành lang tăng từ 1.8 lên 5.2 m. `CorridorDriftCorrectedByGnss` nay có thêm ngưỡng tuyệt đối 1.2 × 1.820 m.

**Hướng tiếp theo.** Nếu muốn bật cổng phẳng trên dữ liệu thật, cần dạng có tính đến nhiễu, ví dụ λ_min > ρ·λ_mid + n·(k·σ_range)². Việc đó cần một ADR mới.
