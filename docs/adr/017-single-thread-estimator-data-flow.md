# ADR-017: Luồng dữ liệu của estimator đơn luồng: `EstimatorCore` sau các hàng đợi, cắt IMU bằng mẫu nội suy, chỉ keyframe vào bản đồ

- Trạng thái: đề xuất
- Ngày: 2026-10-27

## Bối cảnh

Sau các PR module của giai đoạn 1, mọi module (`imu`, `preprocess`, `scan_match`, `degeneracy`, `map`, `backend`, `gnss`, `odom`) đã có, nhưng `pipeline::Estimator` vẫn chỉ đếm message và trả pose đơn vị. ADR-005 hoãn các luồng; `deterministic: true` cần chạy đơn luồng và tái lập bit-cho-bit (E-02). Cần chốt cách nối các module: ai giữ trạng thái, cắt IMU ở biên keyframe ra sao, cái gì vào bản đồ, đo LiDAR vào factor thế nào, GNSS gắn vào keyframe nào. Công thức: `docs/derivations/pipeline.md`.

## Quyết định

1. **`EstimatorCore` đơn luồng sau các hàng đợi hiện có.** `Estimator::Impl` giữ nguyên hàng đợi có giới hạn và phép trộn k-way theo khoá; mỗi message đã trộn được chuyển cho `detail::EstimatorCore` (`core/pipeline/src/estimator_core.{hpp,cpp}`), lớp sở hữu mọi module. `start()`/`stop()` vẫn là no-op (luồng là PR sau, ADR-005).
2. **Chuyển trước mẫu IMU phủ cuối scan.** Khoá của scan là thời điểm kết thúc, nên mẫu IMU đầu tiên có `t >= t_end` (cần cho deskew) xếp sau scan. Khi lấy một scan ra, `drain` chuyển cho core mọi mẫu IMU đang chờ, đến và gồm cả mẫu đó, rồi mới chuyển scan. Chỉ IMU được đưa lên trước; thứ tự wheel và GNSS không đổi (`pipeline.md` §8).
3. **Cắt IMU bằng mẫu biên nội suy.** Tại `t_start` của mỗi scan, preintegrator nhận mọi mẫu `t < t_start`, rồi mẫu nội suy tuyến tính `s(t_start)`; khoảng sau bắt đầu lại bằng chính mẫu đó. `delta.dt_s` khớp `t_j - t_i` và `predict()` cho trạng thái đúng tại `t_start` (`pipeline.md` §1).
4. **Cửa sổ khởi tạo IMU** chạy từ mẫu cuối cùng tại hoặc trước `t - init_window_s` đến mẫu đầu tiên tại hoặc sau `t` (thời điểm bắt đầu scan), nên luôn dài ít nhất `init_window_s` bất kể pha lấy mẫu. Cửa sổ kết thúc đúng tại `t` sẽ ngắn hơn `init_window_s` khi `t` rơi giữa hai mẫu, và khởi tạo có thể không bao giờ qua.
5. **Chưa khởi tạo IMU thì không có ước lượng**: `latest_state().state.t == Timestamp{}`. Scan khớp thất bại thì đi theo dự đoán IMU, không tạo keyframe, không chèn bản đồ.
6. **Chỉ đám mây keyframe vào `VoxelMap`**, chèn tại pose đã tối ưu (nếu `optimize()` thành công), khớp với cơ chế dựng lại của ADR-016.
7. **Đo LiDAR vào factor**: `T_bi_bj = (T_w_bi^ref)^-1 * T_w_bj^match`, hiệp phương sai là `cov_b` của scan j (như LIO-SAM; ADR-004 ghi nhận là lạc quan) (`pipeline.md` §4).
8. **GNSS gắn vào keyframe kế tiếp**, dời về thời điểm keyframe theo `pipeline.md` §5; mỗi factor dùng ước lượng `T_e_w` mới nhất của aligner (khi `backend.gnss_frame_variable` tắt; khi bật, mọi factor dùng chung biến `T_e_w` của graph, xem ADR-019). Cổng chi-square chạy trước aligner, nên bước nhảy bị loại không làm lệch `T_e_w`. Pose tại thời điểm fix được nội suy đúng tại `m.t`: fix được giữ trong một vòng đệm cố định (32 phần tử) cho đến khi có mẫu IMU `t >= m.t`, rồi dự đoán từ trạng thái front-end ở đầu scan trước qua các mẫu IMU và mẫu nội suy tại `m.t`. Fix cũ hơn đầu scan đã xử lý gần nhất bị loại và đếm (`n_gnss_late`).
9. **Backend stub (`MSS_WITH_GTSAM=OFF`)**: lỗi `optimize()` thì dùng trạng thái front-end, chỉ ghi log ở lần lỗi đầu của một chuỗi.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chèn mọi scan vào bản đồ | Bản đồ dày hơn | Không dựng lại được từ kho keyframe (ADR-016); bản đồ mang sai số trôi của từng scan |
| Cố định `T_e_w` sau lần căn chỉnh đầu | `T_e_w` không tự phản hồi theo quỹ đạo | Căn chỉnh đầu dựa trên ít cặp, sai yaw lớn, không cải thiện về sau |
| Tạo keyframe tại mỗi fix GNSS | Không phải dời fix | Keyframe dày gấp nhiều lần, graph và kho keyframe phình ra; dời theo front-end chính xác hơn sai số GNSS |
| Cửa sổ khởi tạo kết thúc đúng tại `t` (bản kế hoạch) | Đơn giản | Ngắn hơn `init_window_s` khi lệch pha lấy mẫu, có thể không bao giờ khởi tạo |
| Để mẫu IMU phủ cuối scan đi đúng thứ tự khoá | Không đổi thứ tự trộn | Core không có IMU đến cuối scan, mọi scan bị bỏ qua |

## Hệ quả

- `run_once()` không cấp phát trên scan không phải keyframe (test `NonKeyframeScanDoesNotAllocate`, `PushScanCopiesWithoutAlloc`); scan keyframe cấp phát bên trong GTSAM iSAM2 và khi một ô `KeyframeStore` lớn thêm.
- Việc tinh chỉnh `T_e_w` dùng chính quỹ đạo ước lượng, nên sai số quỹ đạo và `T_e_w` phản hồi lẫn nhau; theo dõi ATE trên E-01.
- GTSAM cài trong môi trường dev có TBB: iSAM2 dùng luồng worker (khoảng 125 % CPU trong bench) dù estimator đơn luồng. E-02 vẫn tái lập bit-cho-bit trong các lần chạy đã thử; nếu cần đơn luồng thật hoặc ngân sách Pi5, giới hạn TBB trong `backend` là quyết định riêng.
- Bản build không GTSAM không có hiệu chỉnh vận tốc: vận tốc chỉ từ tích phân IMU giữa các scan. Đủ cho box world (ATE khoảng 1 cm), chưa kiểm trên quãng dài.
- Đo trên desktop (box world, 199 scan, GTSAM): `push_scan` p50 khoảng 10 us, p95 15 đến 19 us, khoảng 3,7 ms mỗi scan từ đầu đến cuối.
- Bật GNSS làm kết quả box world kém đi rõ rệt (mss_bench trên cùng bộ dữ liệu, GTSAM, ATE so với quỹ đạo giải tích):

  | Biến thể | ATE | Sai số yaw lớn nhất | Ghi chú |
  | --- | --- | --- | --- |
  | LiDAR + IMU | 0,0088 m | 0,027° | |
  | + wheel | 0,0079 m | 0,024° | |
  | + GNSS | 0,0322 m | 0,304° | 42 factor, 4 lần dựng lại bản đồ |
  | + cả hai | 0,0329 m | 0,298° | |

  Nguyên nhân có lẽ là quyết định 8: `T_e_w` được khớp theo chính quỹ đạo front-end rồi cố định trong mỗi factor, nên fix (sigma 0,5 m) chủ yếu thêm nhiễu và ghép yaw, và kích hoạt dựng lại bản đồ. Chấp nhận được vì `gnss.enabled` mặc định tắt; test thành phần giữ ngưỡng tuyệt đối 0,05 m và thêm điều kiện tương đối ATE có GNSS <= 5 lần ATE chỉ LiDAR trên cùng dữ liệu.
- Mẫu IMU không hữu hạn bị core bỏ và đếm (`n_nonfinite_imu`); cổng phủ IMU của `Estimator` chỉ tính mẫu hữu hạn, nên một mẫu hỏng không làm mất scan.

## Việc còn mở

- Đã giải quyết (PR 4a), xem `docs/derivations/gnss_fusion_study.md` và ADR-019: GNSS làm xấu box world vì mỗi factor đóng băng `T_e_w` của aligner và kéo gauge 4 bậc tự do của w. Đề xuất giữ lại factor đến khi độ trải đủ lớn đã được đo và bị loại (chỉ có tác dụng khi cổng không bao giờ mở, và chặn việc sửa trôi trong hành lang). Thay vào đó `T_e_w` là một biến của graph (`backend.gnss_frame_variable`); quyết định 8 vẫn đúng về việc gắn fix vào keyframe, phần "mỗi factor dùng ước lượng `T_e_w` mới nhất của aligner" chỉ còn áp dụng khi cờ tắt. Box world có GNSS 0,5 m: 0,0089 m so với 0,0088 m chỉ LiDAR.
- Giới hạn TBB của `backend` khi `deterministic: true`: đã quyết định trong ADR-018 (`BackendConfig::single_thread`, `tbb::global_control` trong `Graph`).

## Bổ sung (PR 4d, 2026-10-27): thông tin của cửa sổ tĩnh vào keyframe 0

Quyết định 4 được mở rộng bởi ADR-020: `ImuInitializer` trả thêm trung bình gia tốc của cửa sổ và
hiệp phương sai của trung bình đó (`InitState::acc_mean_b`, `acc_mean_cov`), và pipeline thêm
factor hướng trọng lực lên keyframe 0 (`backend.static_gravity_factor`, mặc định bật, nhiễu x2)
khi keyframe 0 chính là scan khởi tạo. Lý do và số liệu: ADR-020,
`docs/derivations/imu_only_observability.md`.

## Bổ sung (H2, 2026-10-30): khởi tạo "đứng bằng"

Quyết định 4 được mở rộng bởi ADR-027: với `imu.init_level_start` (mặc định tắt),
`ImuInitializer` đặt `R_w_b = I` và đưa trung bình lực riêng trừ trọng lực vào bias gia tốc,
trừ khi phần ngang vượt `imu.init_level_max_bias_mps2` (khi đó dùng cách khởi tạo cũ và ghi cảnh
báo). Lý do và số liệu: ADR-027, `docs/derivations/vertical_drift_study.md`.
