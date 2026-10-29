# Architecture Decision Records

Mỗi ADR ghi một quyết định kỹ thuật: bối cảnh, lựa chọn, phương án đã loại và hệ quả. Một file một quyết định, đánh số tăng dần, viết theo `000-template.md`. Khi đổi ý, viết ADR mới thay thế ADR cũ và đổi trạng thái của ADR cũ thành "thay thế bởi ADR-xxx", không sửa nội dung cũ.

Bắt buộc có ADR trước khi: thêm phụ thuộc, đổi build system, đổi quy ước khung toạ độ hoặc nhiễu, thêm module mới, hoặc đi ngược một ADR đã chấp nhận (xem `CLAUDE.md`).

## Danh mục

| ADR | Quyết định | Trạng thái |
| --- | --- | --- |
| [001](001-language.md) | Ngôn ngữ: C++17 cho lõi, Python cho công cụ và đánh giá | Đề xuất |
| [002](002-build-and-docker.md) | Build: CMake + colcon, Docker đa kiến trúc amd64/arm64 | Đề xuất |
| [003](003-layered-architecture.md) | Kiến trúc phân lớp: lõi C++ không phụ thuộc ROS, adapter ROS 2 mỏng | Đề xuất |
| [004](004-estimator-loosely-coupled-isam2.md) | Estimator: LiDAR factor loosely coupled trong iSAM2 cho giai đoạn 1 | Đề xuất |
| [005](005-threading-and-realtime.md) | Mô hình luồng và thời gian thực | Đề xuất |
| [006](006-math-libraries.md) | Thư viện toán và tối ưu: Eigen, Sophus, GTSAM 4.2 | Đề xuất |
| [007](007-voxel-hash-map.md) | Cấu trúc bản đồ: voxel hash map | Đề xuất |
| [008](008-yaml-config.md) | Cấu hình: YAML là nguồn tham số duy nhất | Đề xuất |
| [009](009-testing-and-ci.md) | Kiểm thử và CI: GoogleTest, test Jacobian bằng sai phân, regression ATE | Đề xuất |
| [010](010-data-and-evaluation.md) | Dữ liệu và đánh giá: rosbag2 MCAP, quỹ đạo định dạng TUM, evo | Đề xuất |
| [011](011-simulation.md) | Mô phỏng: Gazebo Harmonic + ROS 2 Jazzy, mô hình nhiễu và báo cáo nhiễu | Đề xuất |
| [012](012-fpga-time-sync.md) | Đồng bộ thời gian phần cứng bằng FPGA (chờ thông tin phần cứng) | Đề xuất |
| [013](013-build-layout.md) | Bố cục build cụ thể: target theo module, GTSAM chỉ trong backend, option CMake | Chấp nhận |
| [014](014-bench-dataset-format.md) | Định dạng dữ liệu cho bench: nhị phân phẳng `mssdata` chuyển từ bag MCAP | Chấp nhận |
| [015](015-imu-factor-custom-preintegration.md) | IMU vào graph bằng factor preintegration 9x9 tự viết (thay câu `CombinedImuFactor` của ADR-004), loop dùng kernel Cauchy | Chấp nhận |
| [016](016-map-correction-after-loop-closure.md) | Sửa bản đồ sau loop closure: kho keyframe là nguồn sự thật, `VoxelMap` là cache dựng lại được (`clear()`, khoá `map.window_keyframes`…) | Chấp nhận |
| [017](017-single-thread-estimator-data-flow.md) | Luồng dữ liệu estimator đơn luồng: `EstimatorCore` sau hàng đợi, cắt IMU bằng mẫu nội suy, chuyển trước mẫu IMU phủ cuối scan, chỉ keyframe vào bản đồ | Đề xuất |
| [018](018-backend-determinism-tbb.md) | `deterministic: true` giới hạn TBB của backend về một luồng (`BackendConfig::single_thread`, `tbb::global_control` giữ đến hết tiến trình) | Đề xuất |
| [019](019-gnss-frame-variable.md) | `T_e_w` là một biến của graph dùng chung cho mọi factor GNSS (`backend.gnss_frame_variable`, prior nghiêng chặt), thay cho `T_e_w` cố định trong từng factor | Đề xuất |
| [020](020-static-gravity-direction-factor.md) | Hướng trọng lực của cửa sổ tĩnh vào keyframe 0: factor chỉ hướng (`backend.static_gravity_factor`, nhiễu x2), cổng đứng yên (bật); kiểm tra nhất quán innovation đã đánh giá và tắt, `InitState` thêm thống kê cửa sổ | Đề xuất |
| [021](021-relative-factor-decoupled.md) | Factor tương đối LiDAR và loop mặc định `decoupled` (`backend.relative_factor`): phần dư tịnh tiến và quay tách riêng, đúng chart của hiệp phương sai scan matching; `between`, `se3_exact` giữ để so sánh | Đề xuất |
| [022](022-gnss-prealign-fixes.md) | Fix GNSS nhận trước khi căn chỉnh (cả fix tĩnh của cửa sổ khởi tạo IMU, vào keyframe 0) thành factor trên keyframe gần nhất sau khi căn chỉnh (`gnss.attach_prealign_fixes`, mặc định tắt) | Đề xuất |
| [023](023-gnss-yaw-from-track.md) | Yaw của `T_e_w` từ hướng vết GNSS so với hướng thân xe (`gnss.align_yaw_from_track`, mặc định tắt), chiều chạy từ dấu tốc độ bánh xe, cổng rẽ 45°, kẹp dưới phương sai fix; căn chỉnh khi sigma yaw ≤ 3°, vết ≥ 5 m, độ nhất quán ≥ 0,5. Bổ sung 2b: sigma ≤ 6°, đoạn rẽ lấy hướng từ đường đi của bánh xe (theo `T_b_o`), đối chiếu bánh xe với GNSS, vết tối thiểu 3 m khi mọi đoạn có bánh xe | Đề xuất |

Trạng thái: Đề xuất → Chấp nhận → Thay thế. ADR-013 ghi lại bố cục đã scaffold và đang chạy trong CI, nên ở trạng thái chấp nhận; ADR-001 đến 004 cần duyệt lại để chuyển sang chấp nhận (việc còn mở trong `docs/spec.md`).
