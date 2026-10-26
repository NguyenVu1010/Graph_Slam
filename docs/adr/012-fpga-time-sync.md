# ADR-012: Đồng bộ thời gian phần cứng bằng FPGA

- Trạng thái: đề xuất, chờ thông tin phần cứng (model LiDAR, MPU, board FPGA, cổng nối Pi5; xem "Việc còn mở" trong `docs/spec.md`)
- Ngày: 2026-10-23

## Bối cảnh

Trên rig thật, Pi5 đọc IMU qua Linux không có timestamp phần cứng: jitter của I2C/SPI qua kernel ở mức vài ms, trong khi deskew và preintegration cần sai số thời gian dưới 1 ms giữa IMU và LiDAR. LiDAR có đồng hồ riêng. Mục tiêu giai đoạn 2 là chuyển một phần tính toán point-to-plane sang FPGA.

## Quyết định

**Giai đoạn 1: FPGA là "đồng hồ chung" và bộ đọc IMU.**

- FPGA đọc MPU qua SPI ở 200 Hz (cần MPU-6500/9250; MPU-6050 chỉ có I2C và không đạt), gắn timestamp từ bộ đếm riêng tại thời điểm data-ready, đóng khung `{seq, t_fpga_ns, acc[3], gyro[3], crc}` và đẩy sang Pi5.
- Cùng bộ đếm đó phát PPS (và NMEA nếu LiDAR cần) cho LiDAR, nên LiDAR và IMU chung một miền thời gian; nếu LiDAR không nhận PPS thì FPGA ghi lại thời điểm xung đồng bộ của LiDAR để Pi5 hiệu chỉnh.
- Driver trên Pi5 chỉ chuyển `t_fpga_ns` thành `Timestamp` của core (int64 ns), không gắn thời gian của hệ điều hành; phát hiện mất khung bằng `seq`, báo `Status` và bỏ qua, không abort (quy tắc trong `CLAUDE.md`).
- Cổng nối Pi5 ↔ FPGA: SPI hoặc UART là đủ cho 200 Hz × 32 byte; Ethernet nếu board có sẵn. Chốt khi biết board.

**Giai đoạn 2: tăng tốc cộng dồn `JᵀJ` và `Jᵀr` của point-to-plane.** Pi5 gửi cặp (điểm, mặt phẳng) đã tìm được; FPGA trả về `H` 6×6 và `b` 6×1 mỗi vòng lặp Gauss-Newton. Chỉ làm sau khi `bench` chỉ ra bước này chiếm phần lớn thời gian; giao diện giữ nguyên (`solve_gauss_newton` nhận `H`, `b`), nên core không đổi.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Pi5 đọc IMU trực tiếp, timestamp phần mềm | Không cần FPGA | Jitter vài ms làm hỏng deskew; không có PPS cho LiDAR |
| Dùng GNSS làm nguồn PPS | Chuẩn công nghiệp | Rig không có GNSS; trong nhà không có tín hiệu |
| Vi điều khiển (ESP32/STM32) thay FPGA | Rẻ, dễ lập trình | Đủ cho giai đoạn 1, nhưng không có đường lên giai đoạn 2 |
| Toàn bộ front-end trên FPGA | Nhanh nhất | Vượt phạm vi; giao diện dữ liệu phức tạp; giữ làm hướng mở rộng |

## Hệ quả

Sai số thời gian IMU-LiDAR dưới 1 ms mà core không biết gì về phần cứng. Đổi lại, cần driver và test cho giao thức khung (mất khung, sai CRC, tràn bộ đếm), và ADR này phải cập nhật khi có model phần cứng cụ thể.
