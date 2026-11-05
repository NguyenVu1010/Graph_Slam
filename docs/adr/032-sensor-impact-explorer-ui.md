# ADR-032: Trình khám phá ảnh hưởng cảm biến: chạy lại offline, giao diện Streamlit + Plotly (extra `explorer`)

- Trạng thái: đề xuất
- Ngày: 2026-11-05
- Liên quan: ADR-010 (đánh giá, bảng ablation), ADR-011 (bản ghi sạch, noise bridge offline), ADR-031 (xem trực tiếp trong RViz2)

## Bối cảnh

Muốn biết một cảm biến góp bao nhiêu vào kết quả SLAM, hoặc một sự cố cảm biến (mất GNSS, LiDAR tắt vài giây, bias IMU nhảy, trượt bánh) làm sai số tăng bao nhiêu và hồi phục ra sao, hiện phải tự sửa overlay, chạy lại đường ống bằng tay rồi so file. Người dùng cần bật tắt factor, chỉnh nhiễu và đặt sự cố ngay trên giao diện rồi so các lần chạy cạnh nhau.

## Quyết định

1. **Offline, không live.** Mỗi lần chạy dùng lại một bản ghi sạch (`tools/record_scenario.sh`): noise bridge với profile đã nhân bội số (`mss_sim.noise_scale`), tiêm sự cố có thời gian (`mss_sim.faults`), `bag_to_mssdata.py`, `mss_bench` với overlay sinh ra. Cùng spec và cùng commit cho cùng `est.tum`. Bật tắt trong Gazebo đang chạy bị loại: sai số tích luỹ làm khó diễn giải, đổi nhiễu giữa chừng phải sửa plugin, mỗi lần thử không lặp lại được.
2. **LiDAR và IMU không tắt được** (hệ là LiDAR-inertial); chỉ `gnss.enabled`, `wheel_odom.enabled`, `loop_closure.enabled` được bật tắt. Bội số nhiễu chỉ đổi cảm biến mô phỏng, không đổi nhiễu mà pipeline tin (cảm biến xuống cấp mà bộ ước lượng không biết): sigma trong overlay và covariance của bản tin GNSS giữ theo profile chưa nhân (`run_bridge(believed_gnss=...)`). Nhiễu IMU và nhiễu khoảng cách LiDAR của Gazebo đã nằm trong bản ghi, nên hai bội số này từ 1 trở lên: bridge cộng thêm nhiễu `sigma * sqrt(k^2 - 1)`; nhiễu LiDAR thêm dùng luồng seed riêng, nên dropout và điểm hỏng không đổi. GNSS và odom do bridge sinh hoàn toàn, nên bội số từ 0.
5. **Bench ghi thêm `loops.csv`** (`t_i_ns,t_j_ns,state` từ `Estimator::graph_snapshot` sau khi replay, ADR-031) để trang vẽ loop trên quỹ đạo; `est.tum` và `bench.json` không đổi.
3. **Lõi không phụ thuộc giao diện** (`tools/mss_explorer`: spec, runner, metrics, CLI `python -m mss_explorer`), có cache theo hash của spec và commit dưới `results/explorer/runs/<key>/`; lần chạy lỗi (`failed`) không được cache, lần chạy mất dấu (`lost`) được ghi rõ chứ không hiện như một con số.
4. **Giao diện: Streamlit + Plotly**, extra tuỳ chọn `explorer` trong `pyproject.toml` (cùng matplotlib cho ảnh PNG khi xuất báo cáo), chỉ phục vụ localhost. Phương án bị loại: FastAPI + JS tự viết (nhiều mã hơn cho cùng biểu mẫu và biểu đồ), Dash (nặng hơn, ít hợp với biểu mẫu nhập liệu). Plotly được import trong hàm, nên `import mss_explorer` và CI không cần extra; test biểu đồ tự bỏ qua khi thiếu.

## Hệ quả

- Phụ thuộc mới chỉ trong extra `explorer`; core, `ros2_node` và CI không đổi. Trên máy dev, extra nên được cài vào một thư mục riêng (`pip install --target`, numpy giữ đúng bản hệ thống), không vào user site, vì numpy 2 trong user site làm hỏng scipy và matplotlib của hệ thống.
- Mỗi lần chạy mất khoảng thời gian một lần bench trên bản ghi; tối đa hai lần chạy song song (WSL khoảng 10 GB RAM).
- Hai scenario dùng cùng profile và overlay (E-07W, ROOM-01) không phân biệt được qua bản ghi; người dùng chọn đúng thư mục bản ghi.
