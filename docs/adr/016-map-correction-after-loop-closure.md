# ADR-016: Sửa bản đồ sau loop closure: kho keyframe là nguồn sự thật, voxel map là cache dựng lại được

- Trạng thái: chấp nhận
- Ngày: 2026-10-27

## Bối cảnh

`map::VoxelMap` được chèn bằng `T_w_l` từ `scan_match`, tức pose odometry chưa tối ưu. `backend` chỉ sửa quỹ đạo (các biến keyframe) khi có loop closure hoặc GNSS; các điểm đã chèn vào voxel map không đổi. Để nguyên như vậy thì bản đồ "ghi nhớ" sai số trôi: sau loop closure quỹ đạo về đúng nhưng chỗ gặp nhau có tường đôi, và scan matching về sau khớp vào bản đồ sai.

ADR-007 đã ghi "loop closure dựng lại bản đồ cục bộ từ keyframe đã hiệu chỉnh thay vì dịch từng voxel" nhưng chưa nói ai giữ keyframe, dựng lại khi nào, và giới hạn bộ nhớ ra sao. `backend::Graph` đã có `keyframe_state(KeyId)` để đọc pose đã tối ưu của từng keyframe. `VoxelMap` chưa có API xoá toàn bộ.

## Quyết định

1. **Kho keyframe là nguồn sự thật của bản đồ.** `pipeline` giữ `KeyframeStore`: với mỗi keyframe `k` lưu `KeyId`, `Timestamp`, và đám mây keyframe trong khung `l` (bản `ScanFrame` sau `preprocess`, giảm mẫu thêm theo `map.keyframe_voxel_m`), cùng `T_b_l` dùng chung. Pose keyframe không lưu trong kho; luôn đọc từ `backend::Graph::keyframe_state(k)`.
2. **`VoxelMap` là cache phục vụ scan matching**, chứa cửa sổ `map.window_keyframes` keyframe gần nhất, dựng từ kho keyframe bằng `insert(cloud_k, T_w_b(k) * T_b_l)` với pose mới nhất của backend. Nó không còn là nơi duy nhất giữ điểm.
3. **Dựng lại khi quỹ đạo bị sửa.** Sau mỗi `optimize()`, `pipeline` so pose mới của keyframe cũ nhất trong cửa sổ với pose đã dùng để chèn; nếu lệch quá `map.rebuild_trans_m` hoặc `map.rebuild_rot_deg` (hoặc backend báo vừa thêm loop factor) thì gọi `VoxelMap::clear()` rồi chèn lại cả cửa sổ. Chạy trên luồng front-end, giữa hai scan; không chạy trên đường nóng theo điểm. Với odometry thuần (không loop, không GNSS) điều kiện này không bao giờ kích hoạt, nên hành vi hiện tại giữ nguyên.
4. **API bổ sung cho `map`:** `void VoxelMap::clear()` đưa bảng băm, pool điểm và các bộ đếm về trạng thái ngay sau constructor, không cấp phát và không giải phóng bộ nhớ. `insert` không đổi.
5. **Khoá YAML mới dưới `map.*`:** `keyframe_voxel_m` (giảm mẫu đám mây keyframe lưu trong kho, mặc định 0.5), `max_keyframes` (giới hạn kho; đầy thì bỏ keyframe cũ nhất khỏi kho, keyframe vẫn ở trong graph), `window_keyframes` (số keyframe trong cache, mặc định 50), `rebuild_trans_m` (mặc định 0.05), `rebuild_rot_deg` (mặc định 0.5). `pipeline` kiểm tra tất cả > 0 khi khởi động.
6. **Bản đồ xuất ra** (`out_map`, PCD/PointCloud2) dựng từ toàn bộ kho keyframe với pose đã tối ưu tại thời điểm xuất, không lấy từ cache.
7. Sơ đồ `docs/architecture.html` thêm cạnh `backend → map` (qua pipeline).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Gắn `KeyId` vào từng điểm trong voxel map, dịch điểm tại chỗ theo `ΔT_k` | Không cần kho riêng | Điểm đổi voxel nên vẫn phải băm lại toàn bộ; thêm 4 B/điểm; không rẻ hơn dựng lại |
| Không sửa bản đồ, chỉ sửa quỹ đạo | Đơn giản nhất | Tường đôi sau loop closure, scan matching khớp vào bản đồ sai; chỉ đúng khi tắt loop |
| Submap theo kiểu Cartographer (nhiều bản đồ con, mỗi cái gắn một pose) | Sửa bằng cách đổi pose submap, không chèn lại | Cần kNN qua nhiều submap trên đường nóng, phức tạp hơn nhiều so với một bảng băm |

## Hệ quả

- Chi phí dựng lại theo số đo M-03 (desktop): chèn 10k điểm ≈ 0,7 ms, nên cửa sổ 50 keyframe ≈ 35 ms mỗi lần loop closure; trên Pi5 ước gấp 3 đến 4 lần. Chỉ xảy ra khi quỹ đạo bị sửa.
- Bộ nhớ kho keyframe bị chặn bởi `max_keyframes × điểm/keyframe × 12 B`; với `keyframe_voxel_m = 0.5` một keyframe còn khoảng 3k điểm, 1000 keyframe ≈ 36 MB. `VoxelMap` giữ nguyên giới hạn của ADR-007.
- Scan matching chỉ nhìn thấy cửa sổ `window_keyframes`, không phải toàn bộ bản đồ; với odometry thuần, `prune_radius_m` vẫn áp dụng trong cửa sổ.
- `loop` (giai đoạn sau) đọc đám mây ứng viên từ kho keyframe, đúng như sơ đồ "Keyframe + cloud cho loop".
- Ghi chú 2026-10-27 (PR 4a, `docs/derivations/gnss_fusion_study.md` mục 3 và 4): đo LiDAR của keyframe j lấy `ref` từ graph và `match` từ bản đồ; khi graph được sửa (GNSS, loop) mà bản đồ chưa dựng lại vì độ lệch dưới `map.rebuild_trans_m` / `rebuild_rot_deg`, mỗi factor LiDAR bị lệch tối đa cỡ hai ngưỡng đó. Đo trên box world với GNSS và `T_e_w` cố định: sai số factor LiDAR tăng từ 4 mm lên 10 đến 18 mm (NEES 28 lên 160 đến 530). Đây là hệ quả có chủ đích của việc chọn ngưỡng, không phải lỗi. Đối chứng ngưỡng gần 0 (dựng lại ở mỗi keyframe, 22 lần) đưa NEES về 20 nhưng không bớt biến dạng của quỹ đạo, nên chi phí dựng lại thường xuyên không đáng.
- Việc còn mở: ngưỡng dựng lại và kích thước cửa sổ chỉnh theo bench E-02/E-03; cân nhắc dựng lại từng phần (chỉ keyframe có pose lệch) nếu 35 ms gây rớt scan trên Pi5.
