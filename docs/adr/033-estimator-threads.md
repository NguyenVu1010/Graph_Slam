# ADR-033: Luồng của Estimator: front-end, backend, loop; snapshot bản đồ bất biến

- Trạng thái: đề xuất
- Ngày: 2026-11-12
- Liên quan: ADR-005 (mô hình luồng, chi tiết hoá và sửa một phần), ADR-009 (đường nóng không cấp phát), ADR-016 (bản đồ toàn cục), ADR-026 và ADR-028 (loop closure, PCM)

## Bối cảnh

`Estimator` hiện chạy mọi bước trên luồng gọi `run_once()`: `start()` và `stop()` là no-op, `BoundedQueue` không khoá. Một keyframe chạy iSAM2 update, dựng lại `VoxelMap` và tìm loop ngay trong scan đó, nên độ trễ của scan có keyframe cao hơn hẳn scan thường, và loop closure chặn front-end. Máy nhúng đích có 4 nhân. ADR-005 đã chọn ba luồng nhưng để `map_` dưới một mutex mà front-end giữ trong scan matching; như vậy backend dựng lại bản đồ vẫn chặn front-end. Kết quả tất định từng bit (E-02) phải giữ.

## Quyết định

1. **Ba luồng do `Estimator::start()` tạo, `stop()` join**, chỉ khi `deterministic: false`:

| Luồng | Sở hữu (chỉ luồng này ghi) | Nhận | Gửi |
| --- | --- | --- | --- |
| front-end | hàng đợi cảm biến (đầu đọc), `front_`, preintegrator dự đoán và deskew, `MapWindow` đang dùng | IMU, scan, odom, GNSS từ `push_*`; snapshot bản đồ; hiệu chỉnh `T_corr` | `Estimate` mới nhất; `KeyframeMsg` |
| backend | `Graph`, `KeyframeStore`, `GnssFusion`, `LoopSelector` | `KeyframeMsg`, `LoopCandidate` | snapshot bản đồ, `T_corr`, yêu cầu tìm loop |
| loop | `LoopDetector` và bản sao view keyframe | yêu cầu tìm loop | `LoopCandidate` |

2. **Hàng đợi một ghi một đọc (SPSC) không khoá**, dung lượng cố định lúc biên dịch, không cấp phát; tràn thì bỏ phần tử cũ nhất và đếm như ADR-005. Đầu ghi chỉ chạm chỉ số ghi, đầu đọc chỉ chạm chỉ số đọc (`std::atomic`, acquire/release). Thay mục "hàng đợi lock-free: không chọn" của ADR-005: mutex mà front-end có thể phải chờ là nguồn đảo ngược ưu tiên.
3. **Bản đồ front-end là snapshot bất biến.** Backend dựng `VoxelMap` mới ngoài đường nóng rồi công bố bằng một con trỏ dùng chung đổi nguyên tử; front-end lấy snapshot ở đầu mỗi scan và giữ nó tới hết scan. Front-end không bao giờ chờ khoá. Thay mục "`map_` dưới mutex" của ADR-005. Bản đồ là dữ liệu duy nhất dùng `std::shared_ptr` (chia sẻ thật giữa hai luồng); bộ nhớ được cấp sẵn theo `map.max_keyframes`, backend dùng luân phiên một số bộ đệm cố định nên không cấp phát lúc chạy.
4. **Hiệu chỉnh của backend** tới front-end dưới dạng `T_corr` (pose keyframe tối ưu so với pose front-end lúc tạo keyframe), áp tại biên scan kế tiếp. Pose công bố ở tần số scan là pose front-end đã áp hiệu chỉnh gần nhất.
5. **Chế độ tất định giữ nguyên v1.** `deterministic: true`: không tạo luồng; `run_once()` chạy front-end, rồi backend, rồi loop, cùng thứ tự như hiện nay, qua cùng các hàng đợi và message. Kết quả giống từng bit với tag `v1.0.0` trên E-02. Đây là chế độ của test và `mss_bench`.
6. **Lập lịch** (YAML `realtime.*`, mặc định tắt như ADR-005): `pin_cores` ghim front-end, backend, loop vào nhân 1, 2, 3; `priority > 0` đặt `SCHED_FIFO` cho front-end, backend và loop dùng bộ lập lịch thường. Không có quyền (`CAP_SYS_NICE`) thì log cảnh báo và chạy tiếp.
7. **Đo**: `Stats` thêm độ trễ front-end p50/p95/max, độ trễ hiệu chỉnh backend, độ sâu lớn nhất của từng hàng đợi và số lần bỏ của từng hàng đợi.

## Kiểm thử

- Hàng đợi SPSC: unit test một luồng; stress hai luồng với ThreadSanitizer.
- Build ThreadSanitizer trong CI chạy test pipeline.
- E-02 ở chế độ tất định giống `v1.0.0` từng bit.
- Backend giả chậm (500 ms mỗi keyframe): p95 front-end không đổi, không mất scan ở tải danh định.
- Cùng bản ghi, chế độ đa luồng so với tất định: ATE lệch không quá 5%.

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Giữ mutex trên `map_` (ADR-005) | Backend dựng lại bản đồ giữ khoá; front-end chờ, độ trễ scan phụ thuộc backend |
| Hàng đợi có mutex | Đủ nhanh, nhưng front-end có thể chờ luồng ưu tiên thấp |
| Thread pool theo tác vụ | Thứ tự không tất định; scan matching tuần tự theo bản chất |
| Sao chép bản đồ cho front-end mỗi lần cập nhật bằng memcpy dưới khoá | Vẫn có khoá; tốn băng thông bộ nhớ ở mỗi keyframe |

## Hệ quả

- Hàng đợi, snapshot và message nằm trong `core/pipeline`; module vẫn không biết luồng.
- Front-end dùng bản đồ trễ tối đa một keyframe so với backend; scan matching vẫn đúng vì bản đồ cục bộ thay đổi chậm.
- Hai lần chạy đa luồng có thể khác nhau nhỏ do thời điểm áp `T_corr`; mọi so sánh baseline dùng chế độ tất định.
