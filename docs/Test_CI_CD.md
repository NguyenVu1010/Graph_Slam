# Test & CI/CD

Mỗi module có test với tiêu chí đạt bằng con số; mỗi PR phải qua lint, unit test và một test tích hợp ngắn trên bag mô phỏng trước khi merge. Ngưỡng ATE và độ trễ ghi trong `eval/baselines.json` và chỉ đổi bằng PR riêng. Đây là nội dung chi tiết của ADR-009.

## Chiến lược kiểm thử

Năm tầng, càng lên cao càng ít test, càng chậm và càng gần thực tế. Ba tầng dưới chạy trên mọi PR.

| Tầng | Phạm vi | Công cụ | Chạy khi | Ngân sách |
| --- | --- | --- | --- | --- |
| 1. Unit | Một hàm hoặc lớp: Lie group, Jacobian, chuyển đổi tọa độ | GoogleTest, ctest | Mọi push | < 2 phút |
| 2. Component | Một module trên dữ liệu tổng hợp có đáp án biết trước | GoogleTest | Mọi push | < 3 phút |
| 3. Integration ngắn | Cả pipeline trên bag mô phỏng 60 giây, so ATE với baseline | ctest + evo, launch\_testing | Mọi PR vào `main` | < 10 phút |
| 4. Integration đầy đủ | Mọi kịch bản mô phỏng, nhiều seed, ablation | Script `eval/` | Hằng đêm | < 60 phút |
| 5. Hiệu năng và thực địa | Độ trễ trên Pi5, dataset UrbanNav, rig cầm tay | `bench`, `eval/` | Thủ công trước mỗi release | Không giới hạn |

Quy tắc chung: mọi factor và mọi hàm có Jacobian giải tích đều có test so với sai phân số; mọi test dùng seed ngẫu nhiên cố định; bản Debug chạy kèm AddressSanitizer và UndefinedBehaviorSanitizer.

## Test case theo module

Mỗi bảng ghi đầu vào và tiêu chí đạt. Các ngưỡng là giá trị khởi đầu; khi có số đo thật thì chỉnh lại trong PR và ghi lý do.

### core/common — Lie group, thời gian

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| C-01 | Exp/Log khứ hồi SO(3), SE(3) | 10.000 vector ngẫu nhiên, gồm cả góc gần 0 và gần π | ‖Log(Exp(ξ)) − ξ‖ < 1e-9 |
| C-02 | Jacobian trái/phải và adjoint | Điểm ngẫu nhiên | Sai lệch tương đối so với sai phân số < 1e-6 |
| C-03 | Timestamp nano giây | Giá trị biên, chuyển đổi qua lại với giây | Không mất độ chính xác, thứ tự giữ nguyên |
| C-04 | Đọc cấu hình YAML | File thiếu trường, sai kiểu | Báo lỗi rõ tên trường, không dùng giá trị mặc định âm thầm |
| C-05 | `lie.hpp` Exp/Log/Jacobian/adjoint (`tests/unit/test_lie.cpp`) | Vector ngẫu nhiên seed cố định, gồm góc gần 0 và gần π | Khứ hồi Exp/Log < 1e-9; Jr/Jl so với sai phân số < 1e-6 (sai số Frobenius tương đối); Jr·Jr⁻¹ = I (1e-9); Ad khớp phép liên hợp; `boxplus/boxminus` và `transform_covariance` (P Σ Pᵀ) đúng |
| C-06 | `ScanFrame` reserve một lần, không cấp phát (`tests/unit/test_scan_frame.cpp`) | `reserve` rồi `push`/`copy_from`/di chuyển với bộ đếm cấp phát | 0 lần cấp phát sau `reserve`; `push` khi đầy trả `false`; `clear` giữ capacity; `scan_end_time` = `t_start` + max `t_offset_ns` |
| C-07 | Bộ đọc/ghi mssdata (`tests/unit/test_dataset_io.cpp`) | Ghi rồi đọc lại dataset nhỏ; file lidar bị cắt; index thiếu hoặc sai số lượng; thời gian lùi | Khứ hồi từng bản ghi giống hệt; thứ tự khi trùng timestamp ổn định (imu, odom, gnss, gt, lidar); `next` không cấp phát; mọi lỗi trả `Status` kèm lý do, không ném ngoại lệ |
| C-08 | `Estimator` hàng đợi và thứ tự (`tests/unit/test_estimator.cpp`) | Đẩy scan/IMU/GNSS/odom, hàng đợi đầy, IMU lùi thời gian | `push_scan` không cấp phát sau khi dựng; đầy thì bỏ scan cũ nhất và đếm vào `n_dropped_scans`; IMU không tăng bị từ chối; `run_once` xử lý theo khóa thời gian với thứ tự IMU, odom, GNSS, scan khi trùng |
| C-09 | `mss_bench` phát lại (`tests/component/test_bench_replay.cpp`) | Dataset tổng hợp 1 s (200 imu, 5 gnss, 50 odom, 10 scan) chạy `mss_bench` qua `std::system` | Exit 0; `est.tum` đúng 10 dòng; `bench.json` có `n_imu` 200, `n_gnss` 5, `n_odom` 50, `n_scans` 10 và bộ đếm bỏ = 0; cảm biến đi trước lidar không bị bỏ; còn bản ghi chưa xử lý thì exit 3; dataset thiếu thì exit 2 và không ghi gì |

### preprocess — deskew, voxel

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| P-01 | Deskew với chuyển động đều | Scan mặt phẳng tổng hợp, ω và v biết trước, không nhiễu | RMS khoảng cách tới mặt phẳng sau deskew < 1 mm |
| P-02 | Robot đứng yên | ω = 0, v = 0 | Đám mây đầu ra trùng đầu vào |
| P-03 | Voxel downsample | Đám mây ngẫu nhiên, voxel 0.5 m | Mỗi voxel ≤ 1 điểm, điểm nằm trong voxel của nó, kết quả giống nhau giữa hai lần chạy |
| P-04 | Lọc điểm xấu | Có NaN, điểm < 0.5 m, điểm > tầm tối đa | Tất cả bị loại, không crash |

### map — voxel hash map

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| M-01 | kNN đúng | 100.000 điểm, 1.000 truy vấn | Kết quả trùng tìm kiếm vét cạn giới hạn trong lân cận 27 voxel (`knn` chỉ tìm trong 27 voxel quanh điểm truy vấn, nên so với vét cạn trên đúng tập điểm đó, theo `voxel_map.hpp`) |
| M-02 | Giới hạn bộ nhớ | Chèn liên tục, robot đi xa | Số điểm mỗi voxel ≤ N; voxel ngoài bán kính bị xóa |
| M-03 | Tốc độ (ghi lại, không chặn) | Benchmark chèn và truy vấn | Lưu số đo để so qua các commit |

### scan\_match — point-to-plane

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| S-01 | Jacobian residual | Điểm và mặt phẳng ngẫu nhiên | Sai lệch so với sai phân số < 1e-6 |
| S-02 | Khôi phục phép biến đổi biết trước | Phòng tổng hợp 6 mặt, lệch khởi tạo ≤ 0.5 m và 10° | Sai số < 1 cm và 0.1°, ≤ 10 vòng lặp |
| S-03 | Vùng hội tụ | Quét sai lệch khởi tạo từ 0 đến 1 m, 0 đến 30° | Tỷ lệ hội tụ ≥ 95% tại 0.3 m và 5°; xuất biểu đồ |
| S-04 | Nhiễu đo | Nhiễu Gauss σ = 2 cm | Sai số vị trí < 3 cm |
| S-05 | Outlier | 20% điểm ngẫu nhiên | Robust kernel giữ sai số < 5 cm |

### degeneracy

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| D-01 | Phát hiện hành lang | Hai tường song song + sàn | Bị gắn cờ suy biến; hướng suy biến lệch trục hành lang < 5° |
| D-02 | Không báo nhầm | Phòng đủ 6 mặt | Không bị gắn cờ |
| D-03 | Cập nhật có chiếu | Trường hợp D-01 | Bước cập nhật theo hướng suy biến = 0; covariance theo hướng đó được nới ≥ k lần |

### imu — preintegration

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| I-01 | Đứng yên | Chỉ có trọng lực, bias = 0 | ΔR = I, Δv và Δp sau khi trừ trọng lực = 0 (sai số < 1e-9) |
| I-02 | Quay đều | ω hằng | ΔR khớp Exp(ωΔt) |
| I-03 | Đối chiếu GTSAM | Cùng chuỗi IMU ngẫu nhiên | Khớp `PreintegratedCombinedMeasurements` < 1e-9 |
| I-04 | Jacobian theo bias | Bias ngẫu nhiên | Sai lệch so với sai phân số < 1e-6 |
| I-05 | Covariance | 1.000 lần Monte Carlo | Covariance lan truyền khớp hiệp phương sai mẫu trong 10% |

### gnss

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| G-01 | LLA ↔ ENU | Điểm tham chiếu biết trước, khứ hồi | Sai số < 1 mm |
| G-02 | Kiểm định chi-square | Nhiễu bình thường + một cú nhảy 5 m | Cú nhảy bị loại; tỷ lệ chấp nhận đo bình thường 90–99% |
| G-03 | Căn chỉnh yaw | Quỹ đạo xoay 30° so với ENU | Yaw ước lượng lệch < 0.5° sau 30 m |
| G-04 | Lever arm | Anten lệch 0.5 m, robot quay tại chỗ | Vị trí thân robot không đổi |

### odom

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| O-01 | Residual và Jacobian | Chuyển động nhất quán với odometry | Residual = 0; Jacobian khớp sai phân số |
| O-02 | Ràng buộc non-holonomic | Pose có vận tốc ngang | Residual tỷ lệ với vận tốc ngang |

### loop

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| L-01 | Cùng chỗ, khác hướng | Hai scan mô phỏng lệch yaw 90° | Khớp được; yaw ước lượng lệch ≤ 1 sector |
| L-02 | Khác chỗ | Bộ cặp scan ở các vị trí khác nhau | Precision = 100% (không có loop giả) |
| L-03 | Xác nhận bằng ICP | Ứng viên sai cố ý | Bị loại theo ngưỡng fitness |

### backend — GTSAM

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| B-01 | Jacobian factor tự viết | Mọi factor custom | Qua `EXPECT_CORRECT_FACTOR_JACOBIANS` của GTSAM |
| B-02 | Pose graph vòng vuông | 4 cạnh + 1 loop, nhiễu biết trước | Hội tụ về nghiệm đúng |
| B-03 | Loop sai | Thêm một loop outlier | Robust kernel giữ đồ thị không bị kéo lệch quá \[x\] m |
| B-04 | iSAM2 so với batch | Cùng một đồ thị | Kết quả iSAM2 khớp nghiệm batch (LM rồi Gauss-Newton tới điểm cố định, gradient < 1e-5) < 1e-6, mỗi dạng factor tương đối |

### ros2\_node

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| R-01 | Chuyển đổi message | PointCloud2, Imu, NavSatFix mẫu | Khứ hồi sang struct của core không mất dữ liệu |
| R-02 | Khởi động node | `launch_testing` với bag ngắn | Phát `/odometry` trong vòng 2 giây |
| R-03 | Tham số | File YAML mẫu | Giá trị trong node khớp file |

### sim và eval

| ID | Test | Đầu vào | Tiêu chí đạt |
| --- | --- | --- | --- |
| SIM-01 | Mô hình nhiễu IMU | 2 giờ IMU mô phỏng đứng yên | Allan variance khớp tham số cấu hình trong 10% |
| SIM-02 | Bag ghi đúng | Bag từ mỗi kịch bản | Có topic ground truth, timestamp tăng dần |
| EV-01 | Kiểm tra pipeline evo | Quỹ đạo so với chính nó | ATE = 0 |

## Test tích hợp và hiệu năng

Test tích hợp chạy cả `Estimator` trên bag mô phỏng. Ngưỡng là mức tương đối so với baseline đã lưu, vì ATE tuyệt đối chỉ biết được sau lần chạy đầu.

| ID | Kịch bản | Kiểm tra | Tiêu chí đạt | Chạy khi |
| --- | --- | --- | --- | --- |
| E-01 | Kho hàng, 60 giây | ATE RMSE so với baseline | Không tệ hơn baseline quá 20% | Mọi PR |
| E-02 | Kho hàng, chế độ một luồng | Tính tất định | Hai lần chạy cho quỹ đạo giống hệt | Mọi PR |
| E-03 | Kho hàng, có mất gói và message sai thứ tự | Độ bền | Không crash, có cảnh báo, ATE trong ngưỡng E-01 | Mọi PR |
| E-04 | Hành lang dài | Xử lý suy biến | ATE khi bật suy biến nhỏ hơn khi tắt | Hằng đêm |
| E-05 | Ngoài trời, mất GNSS 60 giây | Hồi phục sau mất tín hiệu | Trôi trong vùng mất < \[x\] m; hội tụ lại trong \[y\] giây khi có GNSS | Hằng đêm |
| E-06 | Ngoài trời, multipath | Loại outlier GNSS | Không có bước nhảy pose > \[z\] m | Hằng đêm |
| E-07 | Vòng khép kín dài | Loop closure | Sai số đầu–cuối giảm so với khi tắt loop | Hằng đêm |

**Hiệu năng**

| ID | Máy | Thang đo | Tiêu chí | Chạy khi |
| --- | --- | --- | --- | --- |
| PF-01 | Runner GitHub amd64 | Độ trễ mỗi scan p95 | Chỉ cảnh báo nếu chậm hơn baseline 20%, vì runner dùng chung nên số dao động | Mọi PR |
| PF-02 | Pi5 8 GB | Độ trễ p95, CPU, RAM | p95 < 100 ms ở LiDAR 10 Hz; CPU < 300% (trên 400%); RAM < 2 GB | Trước mỗi release |
| PF-03 | Pi5 8 GB | Chạy dài 30 phút | Không rò bộ nhớ, độ trễ không tăng theo thời gian | Trước mỗi release |
| PF-04 | Máy bàn | UrbanNav HK và so baseline FAST-LIO2, LIO-SAM | Ghi bảng kết quả, không chặn merge | Thủ công |

## Pipeline CI/CD

&#91;embedded content: pipeline CI/CD · 4 sự kiện kích hoạt\]

Mọi PR đi qua đủ năm bước của hàng trên cùng mới được merge. Kiểm tra nặng chạy hằng đêm, còn số liệu hiệu năng thật trên Pi5 chỉ đi vào báo cáo release qua lần chạy bấm tay.

## Workflow, dữ liệu test và quy tắc nhánh

**Workflow GitHub Actions** (thư mục `.github/workflows/`)

| File | Kích hoạt | Việc chính | Thời gian | Chặn merge |
| --- | --- | --- | --- | --- |
| `ci.yml` | Mọi push và PR | Lint (clang-format, clang-tidy trên file đổi, ruff); build Release và Debug + ASan/UBSan trong image deps; ctest unit + component; báo cáo coverage bằng gcovr | \~10 phút | Có |
| `integration.yml` | PR vào `main`; hằng đêm | colcon build; tải bag test (có cache); E-01 đến E-03 và PF-01 trên PR, E-04 đến E-07 hằng đêm; đính biểu đồ evo và bình luận kết quả lên PR | \~10 phút / \~60 phút | E-01–E-03 |
| `arm64.yml` | Hằng đêm, tag | `docker buildx` cho linux/arm64 qua QEMU, chạy unit test | \~60 phút | Không |
| `deps-image.yml` | Thay đổi trong `docker/` | Build lại image phụ thuộc (GTSAM ghim phiên bản), đẩy lên GHCR theo hash | \~40 phút | Không |
| `release.yml` | Tag `v*` | Image runtime đa kiến trúc lên GHCR; GitHub Release kèm báo cáo benchmark và changelog | \~60 phút | — |
| `pi5-bench.yml` | Chỉ bấm tay (`workflow_dispatch`) | Runner self-hosted trên Pi5 chạy PF-02 và PF-03 | \~40 phút | Không |

Runner self-hosted trên repo công khai có rủi ro bị chạy code lạ từ PR của người khác. Vì vậy `pi5-bench.yml` chỉ chạy khi bạn bấm tay, không bao giờ chạy tự động trên PR.

**Dữ liệu test**

- Bag mô phỏng ngắn (mỗi file < 100 MB), sinh bằng seed cố định, đánh phiên bản `testdata-v1`, lưu làm asset của GitHub Release kèm SHA256.
- `tools/fetch_test_data.sh` tải và kiểm checksum; CI cache theo phiên bản.
- UrbanNav và M2DGR không đưa vào CI vì quá nặng; chạy bằng script `eval/` trên máy bàn.

**Baseline**

- `eval/baselines.json` lưu ATE và độ trễ theo từng kịch bản.
- Chỉ cập nhật bằng PR riêng gắn nhãn `baseline-update`, kèm biểu đồ trước/sau và lý do.

**Quy tắc nhánh và chất lượng**

- Trunk-based: `main` được bảo vệ, mọi thay đổi qua PR, squash merge.
- Check bắt buộc: lint, unit + component, E-01 đến E-03.
- Commit theo Conventional Commits để tự sinh changelog.
- `pre-commit` chạy clang-format, ruff và kiểm tra file lớn trước khi commit.
- Mục tiêu coverage: ≥ 80% dòng cho `core/` (không tính `ros2_node/`).
