# ADR-018: Chế độ deterministic giới hạn TBB của backend về một luồng

- Trạng thái: đề xuất
- Ngày: 2026-10-27

## Bối cảnh

`config/default.yaml` đặt `deterministic: true` (trường `pipeline::Config::deterministic`) và CLAUDE.md hứa chế độ này chạy đơn luồng, seed cố định, tái lập bit-cho-bit. Pipeline đã đơn luồng ở chế độ này (ADR-017), nhưng GTSAM trong `~/opt/gtsam` và trong image deps của CI được build với `GTSAM_USE_TBB` (`gtsam/config.h`), nên `ISAM2::update` có thể linear hoá và khử biến trên các luồng worker của TBB. Nếu một phép cộng dồn số thực chạy theo thứ tự phụ thuộc lịch luồng, kết quả có thể lệch giữa các lần chạy. E-02 (`est.tum` giống từng byte) đang qua trên một graph nhỏ; đó là quan sát, không phải bảo đảm. Mục "Việc còn mở" của ADR-017 ghi việc này.

Đo trên desktop (WSL, 12 luồng, GTSAM 4.2 có TBB), test `BackendDeterminism.SingleThreadRunsAreBitIdenticalMultiThreadReported`: 300 keyframe, mỗi keyframe có factor IMU và LiDAR, cứ 50 keyframe thêm một fix GNSS và một loop (k - 40, k), `optimize()` sau mỗi keyframe, so 21 số double mỗi keyframe sau lần tối ưu cuối.

| Chế độ | Lần chạy khác lần 1 của chế độ đơn luồng | Sai khác tuyệt đối lớn nhất | `optimize()` mỗi lần chạy |
| --- | --- | --- | --- |
| `single_thread = true` | 0 / 5 | 0 | 0,291 s |
| `single_thread = false` | 0 / 5 | 0 | 0,262 s |
| `false`, relinearize mọi biến mỗi lần (thử riêng, không commit) | 0 / 3 | 0 | 1,54 s (đơn luồng 1,44 s) |

Trên các graph này GTSAM nhiều luồng vẫn cho kết quả giống từng byte (có lẽ vì phép duyệt cây song song của GTSAM ghi kết quả mỗi nhánh vào ô theo chỉ số chứ không theo thứ tự hoàn thành; chưa đọc mã để xác nhận). Nhưng đó là chi tiết cài đặt của GTSAM và TBB, không phải hợp đồng; phiên bản GTSAM khác, số lõi khác (Pi5 có 4) hoặc graph lớn hơn có thể khác. Ở cỡ graph này TBB cũng gần như không làm `optimize()` nhanh hơn.

## Quyết định

- `BackendConfig` thêm `bool single_thread = false` (giới hạn TBB về một luồng trong suốt đời của `Graph`). Đây không phải khoá YAML riêng: `pipeline::load_config` gán `backend.single_thread = deterministic` sau khi đọc (ADR-008: một nguồn sự thật). `validate.cpp` không đổi.
- `Graph::Impl` giữ một `std::optional<tbb::global_control>` (`max_allowed_parallelism = 1`), tạo trong constructor trước iSAM2 khi `single_thread` là true, trong `#ifdef GTSAM_USE_TBB` (`<gtsam/config.h>`). Nó là thành viên đầu tiên của `Impl` nên sống lâu hơn iSAM2. Constructor ghi một dòng info "backend: TBB limited to 1 thread (deterministic)". GTSAM không có TBB thì cờ là no-op và một dòng info nói vậy. Bản stub (`graph_stub.cpp`, `MSS_WITH_GTSAM=OFF`) bỏ qua cờ.
- `core/backend/CMakeLists.txt`: `find_package(TBB REQUIRED)` và `target_link_libraries(mss_backend PRIVATE TBB::tbb)` chỉ trong nhánh `MSS_WITH_GTSAM`. Không thêm phụ thuộc mới: GTSAM đã link TBB, `libtbb-dev` đã có trong `docker/deps.Dockerfile` và trong WSL; link tường minh chỉ để dùng header `global_control`, PRIVATE nên không lộ ra header công khai.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Build GTSAM không TBB cho cấu hình Pi5/deterministic | Đơn luồng chắc chắn, không có trạng thái toàn tiến trình | Phải duy trì hai bản build GTSAM và hai image deps |
| Tắt `GTSAM_USE_TBB` trong image deps cho mọi người | Một bản build, đơn giản | Mất thông lượng trên desktop cho graph lớn và chế độ không deterministic |
| `tbb::task_arena(1)` quanh mỗi lời gọi | Phạm vi hẹp, không ảnh hưởng toàn tiến trình | GTSAM không cho truyền arena; các `parallel_for` bên trong vẫn có thể dùng arena khác |
| `global_control` trong `Graph` (chọn) | Một nguồn sự thật (`deterministic`), không đổi build | Ảnh hưởng toàn tiến trình khi `Graph` còn sống |

## Hệ quả

- `deterministic: true` không còn dựa vào chi tiết cài đặt của GTSAM: iSAM2 chạy một luồng. Test `BackendDeterminism.SingleThreadRunsAreBitIdenticalMultiThreadReported` kiểm 5 lần chạy giống từng byte (`std::memcmp`); `BackendDeterminism.SingleThreadGraphLimitsTbbForItsLifetime` kiểm `global_control::active_value` bằng 1 khi `Graph` sống (kể cả sau khi move) và trở lại giá trị cũ sau khi huỷ. E-02 không đổi.
- Ảnh hưởng toàn tiến trình: khi một `Graph` đơn luồng còn sống, mọi arena TBB trong tiến trình bị giới hạn một luồng. Thực tế mỗi tiến trình có một `Graph` (do `pipeline::Estimator` sở hữu). Hai `Graph` sống cùng lúc với cờ khác nhau là trường hợp không định nghĩa: constructor đếm số `Graph` mỗi loại và ghi một cảnh báo; giới hạn vẫn áp dụng cho tới khi `Graph` đơn luồng cuối cùng bị huỷ.
- Chi phí: trên graph 300 keyframe ở trên, `optimize()` chậm hơn khoảng 10 % (0,291 s so với 0,262 s mỗi lần chạy); khi relinearize mọi biến, bản đơn luồng còn nhanh hơn. Trên bộ dữ liệu E-01 của lần chạy tích hợp cục bộ (600 scan, 90 keyframe), `mss_bench` với `config/default.yaml` (`deterministic: true`), trung vị của 4 lần chạy mỗi bản trong WSL:

  | Bản build | `push_scan` p50 / p95 | xử lý scan p50 / p95 | `replay_wall_s` | CPU |
  | --- | --- | --- | --- | --- |
  | main (7a88c22) | 15,5 / 33,6 us | 4,45 / 6,66 ms | 4,57 s | 67 % |
  | nhánh này | 17,7 / 37,5 us | 4,62 / 6,62 ms | 5,20 s | 55 % |

  Chênh lệch nằm trong độ dao động giữa các lần chạy (`replay_wall_s` từng lần: main 4,36 đến 5,59 s, nhánh này 4,01 đến 6,16 s); `push_scan` không chạm backend. CPU thấp hơn, phù hợp với việc iSAM2 không còn dùng luồng worker (tiến trình dưới 100 % CPU vì đọc dữ liệu từ ổ Windows). `est.tum` giống từng byte giữa hai bản và giữa mọi lần chạy.
- Chế độ thường (`deterministic: false`) không đổi.
- Nếu một ngày chế độ thường cũng cần tái lập (ví dụ debug trên Pi5), chỉ cần bật `deterministic`; không cần khoá YAML mới.
