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
