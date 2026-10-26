# ADR-009: Kiểm thử và CI: GoogleTest, test Jacobian bằng sai phân, regression ATE

- Trạng thái: đề xuất
- Ngày: 2026-10-23
- Chi tiết: `docs/Test_CI_CD.md` (danh sách test case theo module, workflow, dữ liệu test, quy tắc nhánh)

## Bối cảnh

Lỗi điển hình của SLAM là Jacobian sai dấu hoặc sai quy ước, nhiễu cấu hình lệch thực tế, và regression ATE không ai thấy cho đến khi chạy thật. Repo có hai đích (amd64, arm64) và CI không có GPU.

## Quyết định

- **Năm tầng**: unit (GoogleTest) → component trên dữ liệu tổng hợp có đáp án → integration ngắn E-01..E-03 trên bag mô phỏng 60 s → integration đầy đủ E-04..E-07 hằng đêm → hiệu năng và thực địa (PF-xx) thủ công. Ba tầng dưới chạy trên mọi PR và chặn merge.
- **Quy tắc bắt buộc**:
  - mọi Jacobian giải tích có test sai phân số, sai lệch tương đối dưới 1e-6; factor GTSAM qua `EXPECT_CORRECT_FACTOR_JACOBIANS`;
  - mọi test ngẫu nhiên dùng seed cố định;
  - bản Debug chạy kèm AddressSanitizer và UndefinedBehaviorSanitizer;
  - đường nóng có test đếm cấp phát (hook `operator new` trong test) để bắt vi phạm ADR-005;
  - coverage dòng cho `core/` tối thiểu 80%.
- **Regression**: `eval/baselines.json` lưu ATE và độ trễ theo kịch bản; PR fail nếu ATE của E-01 tệ hơn baseline quá 20%; baseline chỉ đổi bằng PR gắn nhãn `baseline-update` kèm biểu đồ trước/sau.
- **Workflow** (`.github/workflows/`): `ci.yml` (lint, build, unit + component), `integration.yml` (E-01..E-03 và PF-01 trên PR; E-04..E-07 hằng đêm), `arm64.yml` (hằng đêm, QEMU), `deps-image.yml`, `release.yml`, `pi5-bench.yml` (chỉ bấm tay; runner self-hosted không bao giờ chạy tự động trên PR vì rủi ro code lạ).
- **Dữ liệu test**: bag mô phỏng dưới 100 MB mỗi file, sinh từ seed theo ADR-011, đánh phiên bản `testdata-vN`, lưu làm asset của GitHub Release kèm SHA256; `tools/fetch_test_data.sh` tải và kiểm checksum.
- Trạng thái hiện tại: `ci.yml` đã có 4 job (lint, core không GTSAM, python, docker đầy đủ) và xanh; các workflow còn lại thêm khi có bag E-01.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chỉ test tích hợp trên dataset thật | Gần thực tế | Chậm, không có đáp án đúng cho từng module, không chỉ ra Jacobian nào sai |
| Catch2 thay GoogleTest | Cú pháp gọn | GTSAM và ROS 2 đã dùng GoogleTest; `EXPECT_CORRECT_FACTOR_JACOBIANS` là macro GTest |
| Chạy Gazebo trong CI | Test từ nguồn | Runner không có GPU; bag đã ghi từ seed là đủ tái lập |
| Ngưỡng ATE tuyệt đối | Rõ ràng | Chỉ biết sau lần chạy đầu; dùng ngưỡng tương đối so với baseline |

## Hệ quả

Mọi PR có bằng chứng số về ATE và độ trễ. Đổi lại, thêm một factor là thêm ít nhất ba test (Jacobian, component, tích hợp), và bag test phải được bảo trì theo phiên bản khi mô hình nhiễu đổi.
