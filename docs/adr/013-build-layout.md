# ADR-013: Bố cục build cụ thể: target theo module, GTSAM chỉ trong backend, option CMake

- Trạng thái: chấp nhận (đã scaffold, CI xanh tại commit `d3cd71f`)
- Ngày: 2026-10-22
- Chi tiết hoá: ADR-002, ADR-003

## Bối cảnh

ADR-002 và ADR-003 chốt CMake + colcon và lõi không ROS, nhưng chưa nói target nào, option nào, và làm sao để linker ép chiều phụ thuộc. Máy dev chỉ có 16 GB RAM; CI không có GTSAM sẵn.

## Quyết định

- Hai gói colcon: `core/` (`mss_core`, CMake thuần, xuất target `mss::*` qua `find_package(mss_core)`) và `ros2_node/` (`mss_ros2`, ament_cmake, phụ thuộc `mss_core`).
- Root `CMakeLists.txt` là superbuild cho dùng không ROS: core + `tests/` + `bench/`.
- **Mỗi module là một target `mss::<module>`**, không gộp thành một `mss::core`. Module chưa có source là `INTERFACE` target, nên chiều phụ thuộc pipeline → module → common được linker ép từ ngày đầu; app (`bench`, `ros2_node`) chỉ link `mss::pipeline`. Thêm source cho module: đổi `INTERFACE` thành `STATIC` trong `CMakeLists.txt` của module đó.
- `MSS_WITH_GTSAM` (mặc định ON) chỉ link GTSAM vào `mss::backend`. OFF build phần còn lại không cần GTSAM để unit test và lint chạy trên runner CI mỏng.
- Cờ biên dịch chung trong `cmake/mss_warnings.cmake`: `-Wall -Wextra -Wpedantic`; `-Werror` qua `MSS_WERROR` (CI bật).
- Cấu hình: một file `config/default.yaml`, cài vào share của gói ROS và đọc thẳng bởi bench (ADR-008).
- Phụ thuộc hiện tại: Eigen3 ≥ 3.4, GTSAM (ghim trong `docker/deps.Dockerfile`), GTest. yaml-cpp thêm vào `pipeline` khi nạp cấu hình, Sophus vào `common` khi có `lie.hpp` (ADR-006), đều dưới ADR này không cần ADR mới.
- Python: một `pyproject.toml` ở gốc (`ruff`, `pytest`), gói dưới `eval/` (và `sim/`, `tools/` khi có); `eval` không import core.
- Pi5: build root CMake với `-DMSS_BUILD_TESTS=OFF` để tiết kiệm RAM.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Một target `mss::core` duy nhất | Ít CMake | Không ép được chiều phụ thuộc; một module include nhầm GTSAM sẽ không bị bắt |
| GTSAM bắt buộc | Đơn giản | CI mỏng phải build GTSAM 20 phút; unit test của common không cần |
| Tests trong `core/` thay vì `tests/` ở gốc | colcon chạy test sẵn | Giữ layout theo `CLAUDE.md`; colcon dùng cho ROS, ctest dùng qua root CMake |

## Hệ quả

Mọi vi phạm phân lớp là lỗi link. Thêm module là ba dòng CMake. Đổi lại, có hai cách build (root CMake và colcon) phải giữ đồng bộ, và `core/CMakeLists.txt` phải tự đứng được như một project.

Trong job CI `docker-full`, colcon coi `CMakeLists.txt` gốc là package `multi_sensor_slam` (thư mục CMake thường cũng là package colcon), nên build cả superbuild, kể cả test. `docker/Dockerfile` chạy `colcon test --packages-select multi_sensor_slam` rồi `colcon test-result` trong bước build image, nên các test liên kết GTSAM chạy trên CI và làm build image thất bại khi có lỗi. Image deps chạy `ldconfig` sau khi cài GTSAM vào `/usr/local/lib`.

**Bổ sung 2026-10-27 (ba gói colcon trong Docker).** Trước đây `colcon build` từ `/ws` chỉ thấy package `multi_sensor_slam`: colcon dừng tìm sâu khi gặp `CMakeLists.txt` gốc, nên `mss_core` và `mss_ros2` không bao giờ được build và `CMD ros2 launch mss_ros2 mss.launch.py` của image runtime không chạy được.

- `docker/Dockerfile` liệt kê tường minh ba đường dẫn bằng `--paths`: gốc repo (`multi_sensor_slam`), `core/` (`mss_core`) và `ros2_node/` (`mss_ros2`); `colcon list --paths ...` in danh sách gói vào log CI. `mss_ros2` tìm `mss_core` qua prefix path colcon đặt cho gói phụ thuộc (khai báo trong `package.xml`); GTSAM và Sophus ở `/usr/local`.
- `core/CMakeLists.txt` đặt `EXPORT_NAME` của từng target thành tên module, nên `find_package(mss_core)` cho đúng `mss::<module>` như trong cây (trước đó export ra `mss::mss_<module>` và `mss_ros2` sẽ không link được `mss::pipeline`).
- Đường dẫn bench không đổi: `/ws/install/multi_sensor_slam/bin/mss_bench`. Test vẫn chạy bằng `colcon test --packages-select multi_sensor_slam` rồi `colcon test-result`.
- Source của core được biên dịch hai lần (trong superbuild và trong `mss_core`). Đo trong WSL: build riêng `core/` (36 file, Release, `-j3`) mất khoảng 1 phút; trên runner CI 4 nhân là vài phút, nhỏ so với bước build GTSAM của image deps. Hai worker chạy song song với `MAKEFLAGS=-j2` để giữ khoảng bốn trình biên dịch trên runner 16 GB.
- Vì sao chấp nhận: đặt `COLCON_IGNORE` ở gốc sẽ ẩn luôn hai gói con nằm bên dưới; bỏ superbuild khỏi Docker thì mất các test liên kết GTSAM (và bench) trong CI; cho `mss_core` tự build test thì phải chuyển `tests/` vào `core/`, đi ngược bố cục ở trên. Chi phí build lặp rẻ hơn các phương án đó.
