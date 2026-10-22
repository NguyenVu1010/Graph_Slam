# ADR-014: Định dạng dữ liệu cho bench: nhị phân phẳng `mssdata`

- Trạng thái: chấp nhận
- Ngày: 2026-10-23
- Chi tiết hoá: ADR-010

## Bối cảnh

`bench` chạy trên Pi5 không có ROS (ADR-003, ADR-013) và cần đọc cùng dữ liệu mô phỏng với CI. ADR-010 chọn rosbag2 MCAP làm nguồn dữ liệu, nhưng đọc MCAP + CDR trong C++ thì cần hoặc ROS, hoặc thư viện MCAP cộng bộ giải CDR tự viết (thêm phụ thuộc và phải kiểm chứng). Đo độ trễ cần đọc dữ liệu rẻ và lặp lại được.

## Quyết định

Chuyển một lần bằng Python (`tools/bag_to_mssdata.py`) từ bag đã qua cầu nối sang thư mục nhị phân phẳng, little-endian, packed, có `index.json` (đặc tả trong `docs/mssdata_format.md`, phiên bản 1). C++ đọc bằng `fread` và `static_assert` kích thước bản ghi: IMU 56 B, GNSS 105 B, odom 24 B, gt 64 B, lidar header 12 B + 22 B/điểm. Bộ chuyển đổi làm các việc của adapter: sắp theo `header.stamp`, khử trùng, đổi ENU từ LLA, điền hiệp phương sai GNSS từ sigma khi bag không có. Bag vẫn là nguồn gốc; thư mục `mssdata` là sản phẩm phái sinh có thể tạo lại.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Thư viện `mcap` C++ + giải CDR tự viết | Đọc thẳng bag, một định dạng | Thêm phụ thuộc, phải viết và kiểm thử bộ giải CDR cho 5 kiểu tin; không giảm rủi ro so với chuyển đổi |
| `rosbag2_cpp` | Chuẩn, đầy đủ | Cần cài ROS 2 trên Pi5, trái ADR-003 cho bench |
| HDF5 | Có schema, đọc theo lát | Phụ thuộc thư viện lớn trên Pi5 |
| Parquet | Cột, nén | Cần Arrow C++, quá nặng cho bản ghi định kích thước |
| `npz` | Dễ trong Python | C++ không đọc được nếu không tự viết bộ giải zip |
| Nhị phân phẳng (chọn) | Không phụ thuộc, đọc `fread`, kích thước cố định kiểm bằng `static_assert` | Hai định dạng phải giữ đồng bộ; phải có version |

## Hệ quả

Bench không cần ROS và không cần thư viện bên thứ ba để đọc dữ liệu. Đổi lại có hai định dạng: bag (nguồn) và `mssdata` (phái sinh); đổi layout phải tăng `version` và cập nhật cả bộ chuyển đổi, bộ đọc C++ và `docs/mssdata_format.md`. Bộ chuyển đổi giữ trách nhiệm điền hiệp phương sai GNSS (ADR-011), nên bench luôn thấy `cov` hợp lệ.
