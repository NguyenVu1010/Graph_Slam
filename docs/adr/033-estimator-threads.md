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
| front-end | hàng đợi cảm biến (đầu đọc), `front_`, preintegrator dự đoán và deskew, snapshot bản đồ đang giữ | IMU, scan, odom, GNSS từ `push_*`; snapshot bản đồ; hiệu chỉnh `T_corr` | `Estimate` mới nhất; `KeyframeMsg`; yêu cầu prune bản đồ |
| backend | `Graph`, `KeyframeStore`, `GnssFusion`, `LoopSelector`, `MapWindow`, bộ đệm ghi bản đồ | `KeyframeMsg`, `LoopCandidate`, yêu cầu prune | snapshot bản đồ, `T_corr`, yêu cầu tìm loop |
| loop | `LoopDetector` và bản sao view keyframe | yêu cầu tìm loop | `LoopCandidate` |

2. **Hàng đợi một ghi một đọc (SPSC) không khoá** (`SpscQueue`), dung lượng cố định lúc biên dịch, không cấp phát. Đầu ghi chỉ ghi chỉ số ghi, đầu đọc chỉ ghi chỉ số đọc (`std::atomic`, acquire/release). Đầy thì từ chối phần tử mới và đếm (đầu ghi không thể bỏ phần tử cũ nhất mà không ghi chỉ số của đầu đọc); với scan, front-end rút hết hàng đợi và chỉ xử lý scan mới nhất khi tụt lại, đếm số scan bỏ, giữ nguyên ưu tiên "độ trễ hơn đủ scan" của ADR-005. Thay mục "hàng đợi lock-free: không chọn" của ADR-005: mutex mà front-end có thể phải chờ là nguồn đảo ngược ưu tiên.
3. **Bản đồ front-end là snapshot bất biến** (`SnapshotSlots`): một số bộ đệm cố định cấp một lần lúc khởi động (3 khi chỉ front-end giữ snapshot). Backend dựng `VoxelMap` mới vào một bộ đệm rảnh ngoài đường nóng rồi công bố bằng một chỉ số nguyên tử; bộ đệm rảnh được đưa về hiện trạng bằng cách phát lại các thao tác (chèn, prune, clear) đã ghi nhật ký mà nó còn thiếu, sao chép nguyên bộ đệm đã công bố chỉ khi nhật ký không còn đủ; front-end lấy snapshot ở đầu mỗi scan và giữ nó tới hết scan (đếm người đọc nguyên tử trên từng bộ đệm). Front-end không bao giờ chờ khoá, backend không bao giờ chờ front-end (không có bộ đệm rảnh thì bỏ lần công bố đó và đếm). Không dùng `std::shared_ptr` và không cấp phát lúc chạy. Thay mục "`map_` dưới mutex" của ADR-005.
4. **Hiệu chỉnh của backend** tới front-end dưới dạng `T_corr` (pose keyframe tối ưu so với pose front-end lúc tạo keyframe), áp tại biên scan kế tiếp. Pose công bố ở tần số scan là pose front-end đã áp hiệu chỉnh gần nhất.
5. **Chế độ tất định giữ nguyên v1.** `deterministic: true`: không tạo luồng; `run_once()` chạy front-end, rồi backend, rồi loop, cùng thứ tự như hiện nay, qua cùng các hàng đợi và message. Kết quả giống từng bit với tag `v1.0.0` trên E-02. Đây là chế độ của test và `mss_bench`.
6. **Lập lịch** (YAML `realtime.*`, mặc định tắt như ADR-005): `pin_cores` ghim front-end, backend, loop vào nhân 1, 2, 3; `priority > 0` đặt `SCHED_FIFO` cho front-end, backend và loop dùng bộ lập lịch thường. Không có quyền (`CAP_SYS_NICE`) thì log cảnh báo và chạy tiếp.
7. **Đo**: `Stats` thêm độ trễ front-end p50/p95/max, độ trễ hiệu chỉnh backend, độ sâu lớn nhất của từng hàng đợi và số lần bỏ của từng hàng đợi.
8. **Luồng backend, cách cài** (luồng loop riêng chưa có: tìm loop vẫn chạy trong backend). `start()` chỉ tạo luồng backend; front-end ở lại luồng gọi `run_once()` / `flush()`, nên app không đổi cách gọi.
   - **Một hàng đợi vào cho mọi loại message.** `KeyframeMsg`, `GnssFixMsg` và yêu cầu prune đi chung một `SpscQueue<BackendMsg, 64>` để giữ thứ tự gửi: fix gửi trước keyframe phải được gộp vào keyframe đó như ở chế độ tuần tự. `BackendUpdate` về front-end qua một `SpscQueue` thứ hai (64).
   - **Đám mây keyframe** nằm trong 4 bộ đệm `ScanFrame` cấp một lần (`realtime.max_points_per_scan` điểm mỗi bộ); message chỉ mang chỉ số bộ đệm. Front-end ghi vào bộ đệm có cờ nguyên tử rảnh; backend xoá cờ sau khi `KeyframeStore` đã sao chép.
   - **Đầy thì từ chối và đếm, front-end không chờ.** Keyframe cần một chỗ trong hàng đợi và một bộ đệm đám mây rảnh, kiểm trước khi đổi trạng thái nào: thiếu thì scan đó không thành keyframe (`n_backend_keyframes_dropped`), scan sau thử lại vì quy tắc keyframe vẫn đúng; tích phân IMU và odometry tiếp tục từ keyframe trước nên không mất factor nào. Fix GNSS bị từ chối thì bỏ (`n_backend_fixes_dropped`); yêu cầu prune bị từ chối thì scan sau gửi lại. Backend không gửi được update (hàng đợi về đầy) thì đếm `n_backend_updates_dropped`; front-end bỏ qua keyframe đó.
   - **Đánh thức.** Front-end `notify_one()` một condition variable sau mỗi push, không giữ mutex (nên không bao giờ chờ khoá). Backend chờ trên condition variable với hạn 5 ms: hạn này bắt một notify rơi vào giữa lúc backend thấy hàng rỗng và lúc nó bắt đầu chờ; không quay vòng bận. Không chọn semaphore C++20 (dự án dùng C++17) hay futex (riêng Linux).
   - **Áp `T_corr` ở biên scan, đồng bộ với snapshot bản đồ.** Front-end gửi keyframe với trạng thái của nó (`x0`) rồi chạy tiếp từ trạng thái đó; KeyId được đánh số dày từ 0 nên front-end tự biết id. Backend đẩy `BackendUpdate` của keyframe j trước, rồi mới công bố snapshot chứa j (snapshot mang `n_keyframes`). Ở đầu mỗi scan front-end lấy snapshot rồi áp, theo thứ tự, các update có id nhỏ hơn `n_keyframes` của snapshot đó: `T_corr = T_w_b(tối ưu, sau loop nếu có) * T_w_b(x0)^-1`, nhân trái vào pose front-end, `ref_`, và `x0` của các keyframe đã gửi chưa có update; vận tốc quay theo và bớt sai số vận tốc của backend tại keyframe đó (front-end chỉ tích phân vận tốc, so khớp scan không sửa nó), bias lấy của backend. Sau mỗi lần nhân, ma trận quay được chiếu lại lên SO(3) (SVD): nhân liên tiếp với `R^T` làm nghịch đảo khuếch đại phần không trực chuẩn khoảng 3 lần mỗi keyframe, sau vài chục keyframe so khớp scan hỏng. Nhờ vậy pose front-end và bản đồ nó so khớp luôn mang cùng các hiệu chỉnh, kể cả khi loop closure dựng lại bản đồ. Factor LiDAR vẫn là `ref_^-1 * T_w_b` của front-end nên không đổi khi áp hiệu chỉnh.
   - **Truy vấn đọc.** Front-end không hỏi backend gì ngoài snapshot bản đồ và các update. `stats()` đọc bản sao bộ đếm backend công bố sau mỗi message (mutex chỉ giữ trong lúc sao chép). `visit_map()`, `export_map()`, `graph_snapshot()` giữ mutex mà backend giữ trong lúc xử lý một message: gọi ngoài nhịp scan (app, khoảng 1 Hz); trên đường tuần tự chúng không khoá.
   - **Dừng.** `stop()` (và `flush()`, destructor) báo dừng, backend xử lý hết message đã nhận rồi thoát, `join`, sau đó front-end áp mọi update còn lại; kf_opt, loops và bản đồ đầy đủ khi `flush()` trả về. Sau `stop()` estimator chạy tuần tự; `start()` có thể gọi lại.
   - `Stats` thêm `n_map_publish_skipped`, `n_map_full_copies`, `n_backend_keyframes_dropped`, `n_backend_fixes_dropped`, `n_backend_prunes_dropped`, `n_backend_updates_dropped`, `n_backend_corrections`. Độ trễ front-end đo bằng `mss_bench` (`scan_process_ms_*`) với `--pace 1` (phát lại theo nhịp cảm biến: phát nhanh hơn thì front-end vượt backend, so khớp với bản đồ cũ nhiều keyframe và trôi); độ sâu hàng đợi, độ trễ hiệu chỉnh, ghim nhân và `SCHED_FIFO` (quyết định 6) chưa cài.

## Kiểm thử

- Hàng đợi SPSC và snapshot: unit test một luồng; stress hai luồng (đúng thứ tự, không đọc bộ đệm dở dang) với ThreadSanitizer.
- Build ThreadSanitizer trong CI chạy test pipeline.
- E-02 ở chế độ tất định giống `v1.0.0` từng bit.
- Backend giả chậm (500 ms mỗi keyframe): p95 front-end không đổi, không mất scan ở tải danh định. Cài trong `test_estimator_threads` (hook `before_backend_keyframe` chỉ có ở test): backend chậm 400 ms mỗi keyframe trên bản phát lại có nhịp, và backend dừng hẳn sau keyframe 2 tới hết dữ liệu; front-end vẫn ra pose cho mọi scan, keyframe bị từ chối được đếm, `flush()` không kẹt. Các test này chạy dưới ThreadSanitizer trong CI.
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
