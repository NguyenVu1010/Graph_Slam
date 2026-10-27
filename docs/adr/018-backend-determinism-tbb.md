# ADR-018: Chế độ deterministic giới hạn TBB của backend về một luồng

- Trạng thái: đề xuất
- Ngày: 2026-10-27

## Bối cảnh

`config/default.yaml` đặt `deterministic: true` (trường `pipeline::Config::deterministic`) và CLAUDE.md hứa chế độ này chạy đơn luồng, seed cố định, tái lập bit-cho-bit. Pipeline đã đơn luồng ở chế độ này (ADR-017), nhưng GTSAM trong `~/opt/gtsam` và trong image deps của CI được build với `GTSAM_USE_TBB` (`gtsam/config.h`), nên `ISAM2::update` có thể linear hoá và khử biến trên các luồng worker của TBB. Nếu một phép cộng dồn số thực chạy theo thứ tự phụ thuộc lịch luồng, kết quả có thể lệch giữa các lần chạy. E-02 (`est.tum` giống từng byte) đang qua trên một graph nhỏ; đó là quan sát, không phải bảo đảm. Mục "Việc còn mở" của ADR-017 ghi việc này.

Đo trên desktop (WSL, 12 luồng, GTSAM 4.2 có TBB), `test_backend_determinism`: 300 keyframe, mỗi keyframe có factor IMU và LiDAR, cứ 50 keyframe thêm một fix GNSS và một loop (k - 40, k), `optimize()` sau mỗi keyframe, so 21 số double mỗi keyframe sau lần tối ưu cuối. Mỗi chế độ chạy trong một tiến trình riêng (`--gtest_filter=*MultiThreadRunsReported` và `--gtest_filter=*SingleThreadRunsAreBitIdentical`, xen kẽ, 5 cặp tiến trình), nên luồng worker của TBB không chạy song song với các lần đo đơn luồng; tiến trình đơn luồng có tối đa 1 luồng trong suốt lần chạy (đếm `/proc/<pid>/task`).

| Chế độ | Lần chạy khác lần chạy tham chiếu | Sai khác tuyệt đối lớn nhất | `optimize()` mỗi lần chạy (trung vị 5 tiến trình) |
| --- | --- | --- | --- |
| `single_thread = true`, so với lần chạy 1 | 0 / 5 | 0 | 0,195 s |
| `single_thread = false`, so với lần chạy nhiều luồng 1 | 0 / 5 | 0 | 0,262 s |
| `single_thread = false`, so với một lần chạy đơn luồng | 0 / 5 | 0 | |
| `false`, relinearize mọi biến mỗi lần (thử riêng, không commit) | 0 / 3 | 0 | (không dùng: đo trước khi tách tiến trình) |

Trên các graph này GTSAM nhiều luồng vẫn cho kết quả giống từng byte (có lẽ vì phép duyệt cây song song của GTSAM ghi kết quả mỗi nhánh vào ô theo chỉ số chứ không theo thứ tự hoàn thành; chưa đọc mã để xác nhận). Nhưng đó là chi tiết cài đặt của GTSAM và TBB, không phải hợp đồng; phiên bản GTSAM khác, số lõi khác (Pi5 có 4) hoặc graph lớn hơn có thể khác. Ở cỡ graph này TBB không làm `optimize()` nhanh hơn mà chậm hơn khoảng 25 % (chi phí điều phối luồng lớn hơn phần việc chia được).

## Quyết định

- `BackendConfig` thêm `bool single_thread = false` (giới hạn TBB về một luồng cho phần còn lại của tiến trình). Đây không phải khoá YAML riêng: `pipeline::load_config` gán `backend.single_thread = deterministic` sau khi đọc (ADR-008: một nguồn sự thật). `validate.cpp` không đổi.
- `Graph` đầu tiên có `single_thread == true` tạo một `tbb::global_control(max_allowed_parallelism, 1)` trước iSAM2, là biến `static` cục bộ trong hàm (cấp phát một lần, không bao giờ giải phóng), trong `#ifdef GTSAM_USE_TBB` (`<gtsam/config.h>`, header `<tbb/global_control.h>` dùng được cả với TBB 2020 lẫn oneTBB). Giới hạn giữ đến hết tiến trình: nếu nó được giải phóng rồi tạo lại, các worker TBB đã thức dậy sẽ quay vòng chiếm trọn các lõi (người review đo được 11 lõi ở 100 %), còn khi giữ mãi thì TBB không bao giờ tạo worker trong một tiến trình deterministic. Lần đầu ghi một dòng info "backend: TBB limited to 1 thread (deterministic)". GTSAM không có TBB thì cờ là no-op và một dòng info nói vậy. Bản stub (`graph_stub.cpp`, `MSS_WITH_GTSAM=OFF`) bỏ qua cờ.
- `core/backend/CMakeLists.txt`: khi `GTSAM_USE_TBB` (biến CMake do `gtsam_extra.cmake` trong cấu hình CMake của GTSAM đặt), `find_package(TBB REQUIRED)` và `target_link_libraries(mss_backend PRIVATE TBB::tbb)`, chỉ trong nhánh `MSS_WITH_GTSAM`. Không thêm phụ thuộc mới: GTSAM đã link TBB, `libtbb-dev` đã có trong `docker/deps.Dockerfile` và trong WSL. Link là PRIVATE nên không có kiểu TBB trong header công khai, nhưng `mss_backend` là thư viện tĩnh nên bản export vẫn mang `$<LINK_ONLY:TBB::tbb>`: `cmake/mss_coreConfig.cmake.in` gọi `find_dependency(TBB)` khi `GTSAM_USE_TBB` (sau `find_dependency(GTSAM)`), để một consumer của `mss_core` đã cài (như `ros2_node`) cấu hình được.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Build GTSAM không TBB cho cấu hình Pi5/deterministic | Đơn luồng chắc chắn, không có trạng thái toàn tiến trình | Phải duy trì hai bản build GTSAM và hai image deps |
| Tắt `GTSAM_USE_TBB` trong image deps cho mọi người | Một bản build, đơn giản | Mất thông lượng trên desktop cho graph lớn và chế độ không deterministic |
| `tbb::task_arena(1, 1)` và `arena.execute(...)` quanh mỗi lời gọi GTSAM | Phạm vi hẹp, không ảnh hưởng toàn tiến trình; mọi thuật toán TBB bắt đầu bên trong `execute` đều chạy trong arena đó, nên ràng buộc được GTSAM mà GTSAM không cần lộ gì | Phải bọc mọi điểm vào GTSAM có thể dùng TBB (`update`, `calculateEstimate`, marginal, dựng lại iSAM2 sau `optimize()` lỗi, và các điểm thêm sau này); sót một chỗ là mất bảo đảm mà không có lỗi nào báo; thêm chi phí arena mỗi lời gọi |
| `global_control` theo đời của từng `Graph` (bản đầu của PR này) | Trả lại song song khi `Graph` bị huỷ | Giải phóng rồi tạo lại để worker quay vòng chiếm mọi lõi (xem Quyết định) |
| `global_control` giữ đến hết tiến trình (chọn) | Một nguồn sự thật (`deterministic`), không đổi build, không có worker | Ảnh hưởng toàn tiến trình và không đảo ngược được |

## Hệ quả

- `deterministic: true` không còn dựa vào chi tiết cài đặt của GTSAM: iSAM2 chạy một luồng. Bảo đảm này do `BackendDeterminism.SingleThreadLimitIsProcessLifetime` kiểm: `global_control::active_value` bằng 1 khi có `Graph` đơn luồng (kể cả sau khi move), vẫn bằng 1 sau khi nó bị huỷ, và một `Graph` nhiều luồng tạo sau vẫn bị giới hạn và ghi đúng một cảnh báo. `BackendDeterminism.SingleThreadRunsAreBitIdentical` so 5 lần chạy bằng `std::memcmp`, nhưng với GTSAM hiện tại test này không thể thất bại kể cả khi không có giới hạn (các lần chạy nhiều luồng cũng giống từng byte), nên nó không chứng minh giới hạn có tác dụng. E-02 không đổi.
- Ảnh hưởng toàn tiến trình và giữ mãi: sau khi một `Graph` đơn luồng được tạo, mọi arena TBB trong tiến trình bị giới hạn một luồng cho đến khi tiến trình kết thúc. Một `Graph` tạo sau với `single_thread == false` trong cùng tiến trình vẫn bị giới hạn và ghi một cảnh báo (một lần) "backend: TBB already limited to 1 thread by an earlier deterministic Graph". Thực tế mỗi tiến trình có một `Graph` (do `pipeline::Estimator` sở hữu). Dưới ctest mỗi test là một tiến trình (`gtest_discover_tests`); khi chạy cả binary `test_backend_determinism` trong một tiến trình, test nhiều luồng chạy trước và tự bỏ qua nếu TBB đã bị giới hạn.
- Chi phí: trên graph 300 keyframe ở trên, đo trong tiến trình riêng như mô tả ở Bối cảnh, `optimize()` đơn luồng nhanh hơn: 0,195 s so với 0,262 s mỗi lần chạy. Trên bộ dữ liệu E-01 của lần chạy tích hợp cục bộ (600 scan, 90 keyframe), `mss_bench` với `config/default.yaml` (`deterministic: true`), trung vị của 4 lần chạy mỗi bản trong WSL (đo với bản đầu của PR; bench chỉ tạo một `Graph` trước mọi lần dùng TBB nên hành vi giống bản hiện tại):

  | Bản build | `push_scan` p50 / p95 | xử lý scan p50 / p95 | `replay_wall_s` | CPU |
  | --- | --- | --- | --- | --- |
  | main (7a88c22) | 15,5 / 33,6 us | 4,45 / 6,66 ms | 4,57 s | 67 % |
  | nhánh này | 17,7 / 37,5 us | 4,62 / 6,62 ms | 5,20 s | 55 % |

  Chênh lệch nằm trong độ dao động giữa các lần chạy (`replay_wall_s` từng lần: main 4,36 đến 5,59 s, nhánh này 4,01 đến 6,16 s); `push_scan` không chạm backend. CPU thấp hơn, phù hợp với việc iSAM2 không còn dùng luồng worker (tiến trình dưới 100 % CPU vì đọc dữ liệu từ ổ Windows). `est.tum` giống từng byte giữa hai bản và giữa mọi lần chạy.
- Chế độ thường (`deterministic: false`) không đổi.
- Nếu một ngày chế độ thường cũng cần tái lập (ví dụ debug trên Pi5), chỉ cần bật `deterministic`; không cần khoá YAML mới.
