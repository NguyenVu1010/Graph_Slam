# Tổng quan phương án

Chốt ngày 2026-10-22: hệ LiDAR-inertial tự viết trên factor graph, có GNSS và odometry, kiểm chứng bằng mô phỏng và dataset công khai; chạy thật trên Pi5 với LiDAR và MPU đọc qua FPGA. File này là bản tóm tắt; chi tiết nằm ở `docs/adr/` và `docs/Test_CI_CD.md`.

## Quyết định

| Hạng mục | Chốt | Chi tiết ở |
| --- | --- | --- |
| Phạm vi giai đoạn 1 | LIO tự viết (deskew, preintegration, point-to-plane, iSAM2) + GNSS factor + xử lý suy biến + bộ đánh giá | ADR-004 |
| Giai đoạn 2 | Loop closure, odometry bánh xe, tăng tốc kernel trên FPGA, camera | ADR-004, ADR-012 |
| Máy phát triển | amd64, GPU 8 GB, RAM 16 GB, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic | ADR-011 |
| Edge | Pi5 + LiDAR + MPU đọc qua FPGA; rig không có GNSS, không có encoder | Việc còn mở |
| Vai trò FPGA | Giai đoạn 1: đọc IMU, gắn timestamp phần cứng, đồng bộ PPS với LiDAR. Giai đoạn 2: tăng tốc cộng dồn JᵀJ, Jᵀr của point-to-plane | ADR-012 |
| Kiểm chứng | Mô phỏng: đủ 4 cảm biến. UrbanNav: LiDAR + IMU + GNSS có ground truth. Rig thật: LIO với kịch bản F-01 đến F-09 (sai số đầu–cuối, mốc đo tay, độ chính xác bản đồ) | Test_CI_CD |
| Ngôn ngữ, build | C++17 lõi, Python công cụ; CMake + colcon; Docker amd64/arm64 | ADR-001, ADR-002 |
| Kiến trúc | Lõi không phụ thuộc ROS; GTSAM chỉ trong `backend`; LiDAR factor loosely coupled trong iSAM2 | ADR-003, ADR-004 |
| Chất lượng | ~45 test case, 6 workflow GitHub Actions, baseline ATE và độ trễ trong repo | Test_CI_CD, ADR-009 |

## Việc còn mở

- [ ] Model LiDAR và LiDAR có nhận PPS/NMEA không — quyết định cách đồng bộ
- [ ] Model MPU chính xác (6050 chỉ có I2C; 6500/9250 có SPI)
- [ ] Model board FPGA và cổng giao tiếp với Pi5 (SPI, UART, Ethernet)
- [ ] RAM của Pi5 (cần 8 GB)
- [ ] Nguồn ground truth cho rig thật (motion capture hoặc robot tham chiếu)
- [ ] Thước laser và đánh dấu sàn cho kịch bản F-03, F-04
- [ ] Chuyển trạng thái ADR-001 đến 004 và 012 sang "Chấp nhận" sau khi duyệt

## Quy tắc coding

Bản đầy đủ nằm trong `CLAUDE.md` ở gốc repo (dùng cho cả người và trợ lý AI khi code), kèm `.clang-format` và `.clang-tidy`. Dưới đây là các quy tắc quan trọng nhất.

**Toán và hệ tọa độ** (lỗi hay gặp nhất trong SLAM)

- Đặt tên biến đổi theo khung: `T_w_b` biến đổi điểm từ khung `b` sang khung `w`, nên `p_w = T_w_b * p_b`. Các khung: `w` world, `b` body/IMU, `l` LiDAR, `e` ENU.
- Một quy ước nhiễu duy nhất: nhiễu phải trên SE(3), `T ← T · Exp(δ)`, thứ tự `[ρ, φ]` giống GTSAM. Ghi trong `docs/conventions.md`.
- Đơn vị SI; góc trong code là radian; quaternion chỉ xuất hiện ở lớp vào/ra.
- Thời gian nội bộ là `int64` nano giây theo đồng hồ cảm biến; chỉ đổi sang giây dạng `double` khi tính `dt`.
- Mọi Jacobian giải tích phải có test so với sai phân số và có suy dẫn trong `docs/`.

**C++**

- C++17, style dựa trên Google, 100 cột; class `PascalCase`, hàm và biến `snake_case`, thành viên có hậu tố `_`, hằng số `kPascalCase`.
- Không dùng `auto` cho biểu thức Eigen (tránh lỗi expression template); ưu tiên kiểu kích thước cố định.
- Không số "ma thuật": mọi tham số nằm trong struct cấu hình đọc từ YAML và được kiểm tra khi khởi động.
- Exception chỉ khi khởi tạo và đọc cấu hình; trong vòng xử lý thì trả về kết quả có trạng thái.
- Core không include header ROS; module ngoài `backend` không include GTSAM.

**Hiệu năng trên đường nóng** (code chạy mỗi scan)

- Không cấp phát bộ nhớ, không log, không exception cho từng điểm; cấp phát trước và dùng lại buffer.
- Mọi thay đổi ở đường nóng phải kèm số đo trước/sau từ `bench`.
- Luồng do `Estimator` sở hữu; module trao đổi qua hàng đợi có giới hạn; ghi rõ luồng nào sở hữu dữ liệu nào.
- Có chế độ một luồng, seed cố định để kết quả tái lập được.

**Python và Git**

- Python 3.12, `ruff` (lint + format), type hint, `pytest`.
- Conventional Commits; PR nhỏ (≤ 400 dòng thay đổi); mỗi PR có test và ghi ảnh hưởng lên ATE và độ trễ.
- Quyết định kiến trúc mới cần một ADR trước khi code.

## Dev loop

Mỗi tính năng (một factor, một module, một tối ưu) đi qua các bước sau. Một vòng nên gói gọn trong 1–3 ngày.

1. **Toán trước:** viết suy dẫn residual và Jacobian vào `docs/`; nếu có lựa chọn kiến trúc thì viết ADR.
2. **Test trước code:** khung unit test với test Jacobian bằng sai phân số và một component test có đáp án biết trước.
3. **Cài đặt** trong `core/`, bật tắt bằng cờ trong YAML.
4. **Chạy local:** unit + component, rồi test tích hợp E-01 đến E-03 trên bag mô phỏng và so với baseline.
5. **Nếu chạm đường nóng:** build arm64, đưa lên Pi5, chạy `bench`, ghi p50/p95, CPU, RAM.
6. **PR:** CI xanh, mô tả ảnh hưởng lên ATE và độ trễ, cập nhật baseline nếu có lý do.

**Skill hỗ trợ dev loop** (dùng với trợ lý AI)

| Skill | Dùng cho bước | Làm gì |
| --- | --- | --- |
| `slam-factor-dev` | 1–4 | Dẫn quy trình thêm một factor/module: suy dẫn, test Jacobian, cài đặt theo quy ước khung tọa độ, nối vào `Estimator` sau cờ YAML |
| `slam-eval-regression` | 4, 6 | Chạy pipeline trên bag hoặc dataset, tính ATE/RPE bằng evo, so `eval/baselines.json`, xuất bảng ablation và kết luận có regression hay không |
| `pi5-bench-deploy` | 5 | Build arm64, đưa lên Pi5, chạy `bench` và PF-02/PF-03, so với ngân sách 100 ms p95 |
