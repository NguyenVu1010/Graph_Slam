# ADR-002: Build: CMake + colcon, Docker đa kiến trúc

- Trạng thái: đề xuất
- Ngày: 2026-10-22
- Chi tiết hoá bởi: ADR-013 (bố cục target và option CMake đã scaffold)

## Bối cảnh

Có hai đích build: máy bàn amd64 (phát triển, Gazebo, đánh giá) và Pi5 arm64 (chạy thực địa). Lõi phải build được không cần ROS (ADR-003). Máy bàn chỉ có 16 GB RAM, trong khi GTSAM build rất tốn RAM.

## Quyết định

- `core/` là project CMake thuần (≥ 3.22), xuất target namespace `mss::` qua `find_package(mss_core)`; không dùng ament trong lõi.
- `ros2_node/` là gói `ament_cmake` phụ thuộc `mss_core`; cả workspace build bằng `colcon`.
- Phụ thuộc hệ thống qua apt: Eigen 3.4, yaml-cpp, Sophus, GoogleTest. GTSAM ghim đúng một phiên bản 4.2.x, build từ source trong Docker với `GTSAM_USE_SYSTEM_EIGEN=ON` và `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`. PCL không là phụ thuộc của lõi (xem ADR-006).
- Không dùng `-march=native` trong image dùng chung; Pi5 dùng `-mcpu=cortex-a76`. Bản Release để đo độ trễ, RelWithDebInfo để profiling.
- Một Dockerfile multi-stage (deps → build → runtime), build hai kiến trúc bằng `docker buildx` cho `linux/amd64` và `linux/arm64`; thêm dev container cho VS Code.
- `ccache` và `compile_commands.json` bật mặc định. Trên máy 16 GB: `colcon build --parallel-workers 2` với `MAKEFLAGS=-j4`.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Toàn bộ là gói ament | Đơn giản, đúng kiểu ROS | Lõi bị buộc vào ROS, khó chạy dataset offline và unit test |
| Conan hoặc vcpkg | Ghim phiên bản chặt | ROS 2 đã dựa vào apt; thêm một hệ quản lý gói nữa là quá tay cho quy mô dự án |
| Bazel | Build tái lập tốt | Ma sát lớn với ROS 2 và GTSAM |
| Cross-compile bằng toolchain ARM | Nhanh hơn QEMU | Cấu hình sysroot phức tạp; xem lại nếu build arm64 qua buildx quá chậm |
| GTSAM từ apt | Cài nhanh | Khó kiểm soát cờ build; lệch Eigen và `march` gây crash do alignment, lỗi rất hay gặp |

## Hệ quả

Cùng một image chạy được trên máy bàn và Pi5, người xem repo tái lập kết quả bằng một lệnh. Đổi lại, build arm64 qua QEMU chậm, nên CI chỉ chạy nó hằng đêm (ADR-009); khi phát triển trên Pi5 thì build native.
