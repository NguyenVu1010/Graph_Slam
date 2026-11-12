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

2. **Hàng đợi một ghi một đọc (SPSC) không khoá** (`SpscQueue`), dung lượng cố định lúc biên dịch, không cấp phát. Đầu ghi chỉ ghi chỉ số ghi, đầu đọc chỉ ghi chỉ số đọc (`std::atomic`, acquire/release). Đầy thì từ chối phần tử mới và đếm (đầu ghi không thể bỏ phần tử cũ nhất mà không ghi chỉ số của đầu đọc); với scan, front-end rút hết hàng đợi và chỉ xử lý scan mới nhất khi tụt lại, đếm số scan bỏ, giữ nguyên ưu tiên "độ trễ hơn đủ scan" của ADR-005. Thay mục "hàng đợi lock-free: không chọn" của ADR-005: mutex mà front-end có thể phải chờ là nguồn đảo ngược ưu tiên.
3. **Bản đồ front-end là snapshot bất biến** (`SnapshotSlots`): một số bộ đệm cố định cấp một lần lúc khởi động (3 khi chỉ front-end giữ snapshot). Backend dựng `VoxelMap` mới vào một bộ đệm rảnh ngoài đường nóng rồi công bố bằng một chỉ số nguyên tử; front-end lấy snapshot ở đầu mỗi scan và giữ nó tới hết scan (đếm người đọc nguyên tử trên từng bộ đệm). Front-end không bao giờ chờ khoá, backend không bao giờ chờ front-end (không có bộ đệm rảnh thì bỏ lần công bố đó và đếm). Không dùng `std::shared_ptr` và không cấp phát lúc chạy. Thay mục "`map_` dưới mutex" của ADR-005.
4. **Hiệu chỉnh của backend** tới front-end dưới dạng `T_corr` (pose keyframe tối ưu so với pose front-end lúc tạo keyframe), áp tại biên scan kế tiếp. Pose công bố ở tần số scan là pose front-end đã áp hiệu chỉnh gần nhất.
5. **Chế độ tất định giữ nguyên v1.** `deterministic: true`: không tạo luồng; `run_once()` chạy front-end, rồi backend, rồi loop, cùng thứ tự như hiện nay, qua cùng các hàng đợi và message. Kết quả giống từng bit với tag `v1.0.0` trên E-02. Đây là chế độ của test và `mss_bench`.
6. **Lập lịch** (YAML `realtime.*`, mặc định tắt như ADR-005): `pin_cores` ghim front-end, backend, loop vào nhân 1, 2, 3; `priority > 0` đặt `SCHED_FIFO` cho front-end, backend và loop dùng bộ lập lịch thường. Không có quyền (`CAP_SYS_NICE`) thì log cảnh báo và chạy tiếp.
7. **Đo**: `Stats` thêm độ trễ front-end p50/p95/max, độ trễ hiệu chỉnh backend, độ sâu lớn nhất của từng hàng đợi và số lần bỏ của từng hàng đợi.

## Kiểm thử

- Hàng đợi SPSC và snapshot: unit test một luồng; stress hai luồng (đúng thứ tự, không đọc bộ đệm dở dang) với ThreadSanitizer.
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
