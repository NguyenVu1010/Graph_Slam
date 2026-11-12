# ADR-034: SLAM 2D cho MCU một nhân không OS (`mcu2d/`)

- Trạng thái: đề xuất
- Ngày: 2026-11-12
- Liên quan: ADR-005 và ADR-033 (luật trộn hàng đợi, tất định), ADR-009 (không cấp phát), ADR-026 và ADR-028 (loop closure, PCM), ADR-010 (đánh giá)

## Bối cảnh

Hệ LiDAR-inertial 3D trong `core/` cần vài chục MB RAM, GTSAM và một hệ điều hành. Một robot mặt đất trong nhà có thể định vị bằng LiDAR 2D, gyro z và bánh xe trên một vi điều khiển một nhân, không OS, không heap. Bài toán, thuật toán và giới hạn tài nguyên khác hẳn `core/`, nhưng quy ước khung, thời gian, cách kiểm thử và đánh giá dùng lại được.

## Quyết định

1. **Phạm vi**: SLAM phẳng SE(2) (x, y, yaw) với LiDAR 2D quay (360–8000 điểm mỗi vòng, 5–10 Hz), gyro z của IMU, bánh xe tuỳ chọn. Không GNSS, không camera, không độ cao. Bản đồ lưới chiếm chỗ theo submap, cỡ một tầng nhà (khoảng 50 × 50 m). Đầu ra: pose ở tần số scan, dead reckoning ở tần số IMU, bản đồ khi được yêu cầu, qua UART hoặc CAN.
2. **Cảm biến LiDAR mục tiêu: RPLIDAR A1/A2** (UART, giao thức gói của hãng). Driver nằm sau một giao diện `LidarSource` trả từng vòng quét đã gắn timestamp; LiDAR khác chỉ cần thêm driver. Sim Gazebo dùng một LiDAR 2D chung với cùng số tia và tần số.
3. **Thư mục riêng `mcu2d/`** trong repo này, không link với `core/` và không include GTSAM, Eigen động hay ROS. Dùng lại có chủ đích (sao chép hoặc header chung nhỏ, ghi rõ trong code): `Timestamp` int64 ns, `Result`, quy ước khung và tên biến của `docs/conventions.md`, luật trộn theo timestamp và cổng phủ IMU của `Estimator`, chính sách keyframe, ý tưởng phân tích suy biến và PCM.
4. **Kiến trúc**: ISR chỉ chép dữ liệu vào ring SPSC (scan 2 slot, IMU 2 s, bánh xe). Superloop một luồng gọi `run_once()`: front-end mỗi scan (lọc, deskew theo gyro z, dự đoán, scan matching correlative thô rồi Gauss–Newton tinh trên distance field, phân tích suy biến 3×3, chèn vào submap đang mở); backend chạy trong thời gian rảnh, chia lát có giới hạn thời gian và lưu trạng thái dở dang giữa các lát (đóng submap, pose graph SE(2), loop closure branch-and-bound, tối ưu Gauss–Newton/Cholesky thưa kích thước cố định). Front-end luôn chạy xong trước lát backend kế tiếp.
5. **Ngôn ngữ và ràng buộc**: C++17 tập con, `-fno-exceptions -fno-rtti`; không cấp phát heap sau khởi động (bắt bằng linker wrap `malloc`/`new` và test); không `std::string`; log bằng mã số vào ring log; `float` cho dữ liệu, ma trận nhỏ kích thước cố định; mọi tham số trong một struct cấu hình kiểm tra lúc khởi động như `core/`.
6. **Hai đích build từ cùng mã nguồn**: `host` (x86, test, chạy bản ghi, CI) và `firmware` (`arm-none-eabi`, toolchain file CMake). **MCU chưa chọn**: phần thuật toán làm và kiểm trên host trước; MCU được chốt bằng một bổ sung cho ADR này trước khi port, với ngân sách SRAM và flash cụ thể. Ước lượng ban đầu cho Cortex-M7 khoảng 480 MHz: SRAM bắt buộc khoảng 280 KB, front-end p95 ≤ 40 ms ở 10 Hz; bản đồ cỡ một tầng nhà cần SDRAM ngoài.
7. **Kiểm thử và đánh giá** như `core/`: unit test, test thành phần có đáp án biết trước, tất định từng bit trên host, bản ghi sim 2D đánh giá bằng evo với baseline riêng `eval/baselines_2d.json` (cùng luật chặn PR khi ATE tệ hơn 20%). Khi có firmware: CI báo kích thước (`size`, linker map) và chặn PR vượt ngân sách; đo chu kỳ bằng `DWT->CYCCNT` trên board (chạy tay).

## Phương án đã cân nhắc

| Phương án | Vì sao không chọn |
| --- | --- |
| Port `core/` xuống MCU | GTSAM, Eigen động và bản đồ voxel 3D không vừa SRAM; cần OS |
| RTOS (FreeRTOS) với nhiều task | Thêm phụ thuộc và rủi ro race; superloop chia lát đủ cho một front-end và một backend |
| Repo riêng | Phải sao chép quy ước, sim, eval và CI |
| Bản đồ điểm thay lưới chiếm chỗ | Bộ nhớ tăng theo số scan; tìm loop bằng branch-and-bound cần lưới |

## Hệ quả

- `core/` không đổi; CI thêm job build và test `mcu2d` trên host.
- Cần một world Gazebo và bản ghi với LiDAR 2D cho dữ liệu kiểm thử.
- Ngân sách bộ nhớ và thời gian trong mục 6 là ước lượng; con số chặn PR chỉ có sau khi chốt MCU.
