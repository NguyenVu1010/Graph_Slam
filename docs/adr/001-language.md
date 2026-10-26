# ADR-001: Ngôn ngữ: C++17 cho lõi, Python cho công cụ

- Trạng thái: đề xuất
- Ngày: 2026-10-22

## Bối cảnh

Lõi SLAM phải chạy thời gian thực trên Pi5 (4 nhân Cortex-A76, không GPU) trong ngân sách dưới 100 ms mỗi scan LiDAR 10 Hz. Các thư viện cốt lõi (GTSAM, Eigen, PCL) đều là C++. Mục tiêu dài hạn là tối ưu để triển khai trên phần cứng chuyên biệt.

## Quyết định

- C++17 cho thư viện lõi và node ROS 2; trình biên dịch GCC 13 của Ubuntu 24.04, cả trên máy bàn lẫn trên Pi5.
- Python 3.12 cho script mô phỏng, chuyển đổi dữ liệu, đánh giá (evo) và vẽ biểu đồ; không bao giờ nằm trên đường xử lý thời gian thực.
- Quy ước code: style dựa trên Google, `clang-format` và `clang-tidy` trong repo, `-Wall -Wextra -Wpedantic` mọi lúc và `-Werror` khi chạy CI.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| C++20 | `std::span`, concepts | GTSAM 4.2 và ROS 2 Jazzy nhắm C++17; lợi ích nhỏ hơn rủi ro lỗi header và ABI. Nâng sau bằng một ADR mới nếu cần |
| Rust | An toàn bộ nhớ | Không có GTSAM, hệ sinh thái ROS 2 cho Rust còn non |
| Python cho cả lõi | Viết nhanh | Scan matching và kNN quá chậm trên Pi5; không chứng minh được năng lực tối ưu |
| CUDA cho scan matching | Rất nhanh trên GPU | Pi5 không có GPU NVIDIA; để dành cho bản Jetson mở rộng |

## Hệ quả

Chạy native trên ARM, số liệu độ trễ phản ánh đúng phần cứng đích. Đổi lại, thời gian build dài hơn và lỗi Jacobian khó bắt hơn, nên ADR-009 bắt buộc test Jacobian bằng sai phân số cho mọi factor.
