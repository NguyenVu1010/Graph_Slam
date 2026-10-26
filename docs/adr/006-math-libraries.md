# ADR-006: Thư viện toán và tối ưu: Eigen, Sophus, GTSAM 4.2

- Trạng thái: đề xuất
- Ngày: 2026-10-23

## Bối cảnh

Cần đại số tuyến tính cỡ nhỏ cố định (3×3, 6×6) trên đường nóng, phép toán Lie group SO(3)/SE(3) có Jacobian đúng theo một quy ước duy nhất (`docs/conventions.md`: nhiễu phải, thứ tự `[ρ, φ]`), và một bộ tối ưu đồ thị tăng dần. Test C-01, C-02 đòi Exp/Log khứ hồi dưới 1e-9 và Jacobian khớp sai phân số dưới 1e-6.

## Quyết định

| Thư viện | Phiên bản | Dùng ở | Quy tắc |
| --- | --- | --- | --- |
| Eigen | 3.4 (apt) | mọi module | kiểu kích thước cố định trên đường nóng; không `auto` cho biểu thức Eigen; truyền `const&`; không `-march=native` trong image chung để tránh lệch alignment với GTSAM |
| Sophus | header-only, ghim commit trong `docker/deps.Dockerfile` | `common` | bọc trong `mss/common/lie.hpp` (`so3_exp`, `so3_log`, `se3_exp`, `se3_log`, Jacobian trái/phải); module khác gọi qua lớp bọc này để quy ước `[ρ, φ]` và nhiễu phải có một chỗ duy nhất |
| GTSAM | 4.2.x, build từ source | chỉ `backend` | chuyển `Pose` ↔ `gtsam::Pose3` tại biên `backend`; không type GTSAM nào thoát ra ngoài |
| PCL | không dùng | — | chuyển `PointCloud2` → span trong `ros2_node` bằng đọc trực tiếp buffer; kNN do `map` tự làm (ADR-007). Thêm PCL khi có nhu cầu thật, bằng ADR mới |

- Thứ tự tangent của Sophus `SE3::log()` là `[ρ, φ]`, là thứ tự của dự án; GTSAM `Pose3` dùng `[φ, ρ]`, nên chỉ `backend` hoán đổi vector (`P`) và hiệp phương sai (`P Σ Pᵀ`) tại biên (`docs/derivations/lie_conventions.md`); test `tests/unit/backend/test_lie_gtsam.cpp` đối chiếu với `gtsam::Pose3::Logmap`.
- Mọi Jacobian giải tích do dự án viết (point-to-plane, preintegration theo bias, odom) có file suy dẫn trong `docs/derivations/` và test sai phân số.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Tự viết Lie ops trong `common` | Không thêm phụ thuộc | Dễ sai ở góc gần 0 và gần π; Sophus đã xử lý và được dùng rộng rãi; vẫn có lớp bọc để đổi sau nếu muốn |
| Dùng `gtsam::Pose3` làm kiểu pose chung | Một kiểu duy nhất | Kéo GTSAM vào mọi module, vi phạm ADR-003; build chậm |
| Ceres thay GTSAM | Quen thuộc | Không có iSAM2 và preintegration IMU sẵn; phải viết nhiều hơn |
| g2o | Nhẹ | Không có IMU preintegration chuẩn; cộng đồng nhỏ hơn |
| PCL cho kNN và downsample | Có sẵn | Cấp phát heap nhiều, KdTree dựng lại mỗi scan chậm trên Pi5; kéo Boost và thời gian build |

## Hệ quả

Ba phụ thuộc toán, mỗi cái một vai trò rõ. Đổi lại, Sophus cần ghim commit vì không có bản phát hành ổn định qua apt, và lớp bọc `lie.hpp` là chỗ phải viết test kỹ nhất vì mọi Jacobian khác dựa vào nó.
