# Định dạng dữ liệu `mssdata` (phiên bản 1)

Thư mục phẳng mà `bench` đọc không cần ROS hay thư viện MCAP. Sinh bởi `tools/bag_to_mssdata.py` từ bag đã qua cầu nối (`sim/mss_sim/bridge.py`). Quyết định: ADR-014.

```
<out>/
  index.json
  imu.bin  gnss.bin  odom.bin  gt.bin
  lidar/000000.bin  lidar/000001.bin  ...
```

## Quy ước chung

- Little-endian, không căn lề (packed): kích thước bản ghi là tổng kích thước trường, không có padding. C++ dùng `static_assert(sizeof(...) == N)` trên struct `#pragma pack(1)` hoặc đọc từng trường.
- Đơn vị SI, góc radian, thời gian `t` là int64 nano giây lấy từ `header.stamp` (đồng hồ cảm biến), không phải log time của bag.
- Mỗi file `*.bin` là mảng liên tiếp các bản ghi cùng kích thước, không header; số bản ghi = kích thước file / kích thước bản ghi, và phải khớp `count` trong `index.json`.
- Thứ tự và khử trùng: bản ghi sắp theo `t` tăng chặt (sắp ổn định theo `t`; nếu nhiều bản ghi trùng `t` thì giữ bản đầu tiên theo thứ tự log time, bỏ phần còn lại). Bag đã đảo thứ tự (E-03) vì thế vẫn cho file đơn điệu. `count` là số sau khi khử trùng. Lidar áp dụng cùng quy tắc theo `t_start`, rồi đánh số từ 0.
- Quaternion chỉ ở biên I/O, thứ tự Hamilton `(w, x, y, z)`. Bag (ROS) lưu `(x, y, z, w)`; bộ chuyển đổi đổi thứ tự.

## `imu.bin` (56 B/bản ghi)

| Offset | Kiểu | Trường | Ghi chú |
| --- | --- | --- | --- |
| 0 | int64 | `t` | ns |
| 8 | f64 x3 | `acc` | gia tốc đo, m/s^2, khung `b` (IMU) |
| 32 | f64 x3 | `gyro` | tốc độ góc đo, rad/s, khung `b` |

## `gnss.bin` (105 B/bản ghi)

| Offset | Kiểu | Trường | Ghi chú |
| --- | --- | --- | --- |
| 0 | int64 | `t` | ns |
| 8 | f64 x3 | `p_e_g` | vị trí anten trong ENU quanh `origin_lla`, m |
| 32 | f64 x9 | `cov` | hiệp phương sai 3x3 theo hàng, m^2, khung `e` |
| 104 | int8 | `status` | `NavSatStatus.status` (-1 không fix, 0 fix, 1 SBAS, 2 GBAS) |

`cov` lấy nguyên `position_covariance` khi `position_covariance_type != 0`. Khi bằng 0 (không biết), bộ chuyển đổi điền `diag(sigma_h^2, sigma_h^2, sigma_v^2)` từ `gnss.sigma_h_m`, `gnss.sigma_v_m` trong file cấu hình pipeline (`--pipeline-config`) và đặt `gnss_cov_filled_from_sigma = true`; thiếu cấu hình thì báo lỗi. `gnss_cov_filled_from_sigma` là `true` nếu có ÍT NHẤT MỘT dòng được điền.

## `odom.bin` (24 B/bản ghi)

| Offset | Kiểu | Trường | Ghi chú |
| --- | --- | --- | --- |
| 0 | int64 | `t` | ns |
| 8 | f64 | `v` | `twist.linear.x`, m/s |
| 16 | f64 | `omega` | `twist.angular.z`, rad/s |

## `gt.bin` (64 B/bản ghi)

| Offset | Kiểu | Trường | Ghi chú |
| --- | --- | --- | --- |
| 0 | int64 | `t` | ns |
| 8 | f64 x3 | `p` | `p_w_b`, m |
| 32 | f64 x4 | `q_wxyz` | `R_w_b` dạng Hamilton `(w, x, y, z)` |

Chỉ dùng cho đánh giá; không đưa vào estimator.

## `lidar/NNNNNN.bin` (header 12 B + n x 22 B)

Một file một quét, đánh số từ 0 theo thời gian.

| Offset | Kiểu | Trường | Ghi chú |
| --- | --- | --- | --- |
| 0 | int64 | `t_start` | ns, `header.stamp` của quét |
| 8 | uint32 | `n` | số điểm |
| 12 + 22 i | f32 x3 | `xyz` | m, khung `l` (LiDAR) |
| 24 + 22 i | f32 | `intensity` | |
| 28 + 22 i | uint32 | `t_offset` | ns so với `t_start`, phải < 2^32 ns (khoảng 4,29 s) |
| 32 + 22 i | uint16 | `ring` | chỉ số tia |

Điểm bị rơi giữ nguyên NaN/inf (lõi lọc ở preprocess). Điểm theo thứ tự hàng-trước của cloud có tổ chức. Bag thô không có `t`/`ring` bị từ chối.

## `index.json`

```json
{
  "version": 1,
  "topics": {
    "imu":   {"file": "imu.bin",  "count": 1200},
    "gnss":  {"file": "gnss.bin", "count": 30},
    "odom":  {"file": "odom.bin", "count": 300},
    "gt":    {"file": "gt.bin",   "count": 1201},
    "lidar": {"dir": "lidar",     "count": 60}
  },
  "origin_lla": [21.0278, 105.8342, 12.0],
  "T_b_l": {"translation_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
  "p_b_g_m": [0, 0, 0],
  "body_frame": "base_link",
  "lidar_frame": "lidar_link",
  "gnss_cov_filled_from_sigma": false
}
```

- `origin_lla`: gốc ENU `[lat_deg, lon_deg, alt_m]`, là `--origin-lla` hoặc fix đầu tiên.
- `T_b_l`: từ `/tf_static` (khung con `lidar_link`), `p_b = R * p_l + t`.
- `p_b_g_m`: cánh tay đòn anten trong `b`, m; không có trong bag, lấy từ `frames.p_b_g_m` của file cấu hình (mặc định `[0,0,0]` kèm cảnh báo).
- `body_frame`: khung cha của `lidar_link` trong `/tf_static`.
