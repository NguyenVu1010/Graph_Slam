# ADR-028: Chọn loop bền vững: PCM ở front-end loop, gỡ factor loop khỏi iSAM2 khi mất nhất quán, Cauchy giữ làm lớp cuối; cổng Mahalanobis tuyệt đối thành tuỳ chọn tắt

- Trạng thái: đề xuất
- Ngày: 2026-10-30
- Liên quan: ADR-004 (iSAM2), ADR-005 (luồng), ADR-015 (Cauchy cho loop), ADR-016 (dựng lại bản đồ sau loop), ADR-026 (loop closure và sửa đổi 2026-10-30), backlog C1-3, C1-12, C1-13, C1-16, C1-23

## Bối cảnh

Sửa đổi đầu của ADR-026 (H3 lần 1) chặn loop sai trong cấu trúc lặp bằng một cổng Mahalanobis **tuyệt đối**: hiệu chỉnh mà loop đòi phải hợp với hiệp phương sai tương đối của graph giữa hai keyframe. Review H3 (I-1) cho thấy cổng này loại **mọi** loop đúng khi graph tự tin quá mức một chút. Trên test hai vòng, chỉ cần hạ sigma odometry khai báo xuống một nửa (nhiễu thật giữ nguyên), cổng loại 43 lần tìm và không nhận loop nào. Sai số đầu–cuối vẫn là 1,21 m, trong khi không có cổng thì 4 loop đưa nó xuống 0,55 m. Hiệp phương sai LiDAR của pipeline cỡ mm và NEES của hộp đã là 7,5 đến 8,6 so với 6, nên graph thật đúng là tự tin quá mức. Vì vậy cổng tuyệt đối biến loop closure thành vô dụng đúng ở nơi cần nó.

Chủ dự án chọn hướng 4: chọn loop bền vững, **không tin hiệp phương sai tuyệt đối** của graph. Các ứng viên:

1. **PCM** (pairwise consistency maximisation, Mangelson và cộng sự 2018) ở front-end loop. Hai loop đúng thì phải nhất quán **với nhau** qua odometry nối chúng, và độ trôi chung của hai lần đi qua bị triệt tiêu trong chu trình. Chọn tập con nhất quán lớn nhất (max clique).
2. **GNC** (graduated non-convexity, Yang và cộng sự 2020). `gtsam::GncOptimizer` chạy batch trên cả graph.
3. **Switchable constraints** (Sünderhauf 2012) hoặc **DCS** (Agarwal 2013) trong backend. Mỗi loop có một biến công tắc `s ∈ [0, 1]` với prior `s = 1`; DCS là dạng đóng tương đương, tức một M-estimator.
4. Kết hợp.

Ràng buộc:
- GTSAM chỉ ở `backend`.
- Đơn luồng, chạy lúc keyframe, không cấp phát theo điểm.
- Chi phí trên Pi 5 phải nhỏ.
- iSAM2 là tăng dần.

## Quyết định

### 1. PCM ở front-end loop (`loop::LoopSelector`), không có GTSAM

**Lưu trữ.**
- Tập loop đã qua xác minh hình học (`LoopDetector`: Scan Context, ICP, các cổng độ khớp, kiểm tra nhập nhằng) được giữ trong tối đa `pcm_max_loops` (≤ 64) ô.
- Ma trận nhất quán cặp là một bitset 64 bit cho mỗi ô.

**Odometry.**
- Pipeline đưa vào mỗi keyframe phép đo LiDAR `T_{k-1,k}` cùng hiệp phương sai của nó, đúng cặp đã vào graph.
- Selector giữ hai tổng tiền tố: pose `T_0k`, và hiệp phương sai trong tiếp tuyến trái của gốc, `S_k = S_{k-1} + Ad(T_0k) (s² C_k) Ad(T_0k)^T`, với `s = pcm_odometry_sigma_scale` (2,0): hiệp phương sai odometry được nới trong phép thử cặp (graph không đổi), vì front-end có thể khai sigma nhỏ hơn thật tới `s` lần (review H3: sigma khai báo ×0,5 vẫn phải đóng được loop).
- Odometry giữa `a < b` vì thế là O(1): `T_ab = T_0a^-1 T_0b`, `C_ab = Ad(T_0b^-1) (S_b - S_a) Ad(T_0b^-1)^T` (derivation `loop_closure.md` mục 11).
- Đây là odometry thuần, không phải marginal của graph: marginal đã chứa chính các loop đang bị xét và mọi loop khác, nên không còn là phép thử độc lập.

**Phép thử cặp.**
- Hai loop `a`, `b` tạo một chu trình `X = Z_a^-1 · O(i_a → i_b) · Z_b · O(j_b → j_a)`. Hiệp phương sai được ghép theo nhiễu phải từ bốn đoạn, coi chúng độc lập.
- Nhất quán khi `e^T Σ_X^-1 e ≤ pcm_chi2_gate` (22,46, phân vị 0,999 với 6 bậc tự do), với `e = Log(X)`.
- Độ trôi tích luỹ giữa vòng 1 và vòng 2 xuất hiện ở cả hai loop nên bị triệt tiêu **khi hai đoạn odometry của chu trình ngắn và tương ứng nhau** (hai loop cùng một lần quay lại). Khi đó chu trình chỉ chứa hai đoạn odometry ngắn và hai sàn hiệp phương sai loop (5 cm, 1°), và graph tự tin quá mức ảnh hưởng ít hơn nhiều so với cổng tuyệt đối. Đây là lý do chọn PCM.
- **Nói thẳng: PCM vẫn tin hiệp phương sai odometry.** Khi chu trình chứa một đoạn odometry dài không bị triệt tiêu (ví dụ loop `(i, i+N)` với loop `(i, i+2N)`: cả một vòng odometry), một độ lệch hệ thống không có trong hiệp phương sai (bias yaw 0,1° mỗi bước của test) làm chi2 lớn lên theo độ dài chu trình, và một hệ số `s` cố định chỉ che được chu trình tới một độ dài nhất định. Review H3 (B-1): ba vòng, bán kính 6 đến 20 m, loop đúng của lần quay lại đầu bị gỡ khi lần quay lại thứ hai lớn hơn; hai vòng ở seed khác cũng mất một loop đúng. Cách xử lý: sửa đổi C dưới đây.

**Chọn tập.**
- Max clique chính xác: Bron–Kerbosch có pivot trên bitset, giới hạn `pcm_max_clique_steps` bước gọi đệ quy.
- Hoà về kích thước thì ưu tiên clique giữ nhiều loop đang được chọn nhất (ổn định, không lật qua lật lại khi bằng chứng ngang nhau), rồi đến clique có loop cũ nhất.
- Tập mới so với tập cũ cho ra hai danh sách: loop cần **thêm** vào graph (gồm loop mới, và loop cũ quay lại), loop cần **gỡ**.
- Cả hai danh sách được dự trữ trong constructor, nên không cấp phát mỗi lần gọi.

**Loop đầu tiên và khi có ít loop.**
- PCM cần ít nhất hai loop để có bằng chứng. Loop đầu tiên tạo clique cỡ 1 và **được thêm ngay**, như trước ADR này, với Cauchy làm lớp bảo vệ.
- Chờ loop thứ hai thì mất recall ở những lần quay lại chỉ cho một loop. Trên hai vòng tổng hợp, loop đầu đến ở keyframe 72, loop thứ hai ở keyframe 97, và sai số hiệu chỉnh trong 25 keyframe đó sẽ ở lại trong bản đồ.
- Khi loop thứ hai mâu thuẫn với loop đầu, hai clique cỡ 1 hoà nhau, và loop đang được chọn được giữ. Loop thứ ba nhất quán với một trong hai sẽ phân xử.
- Nói thẳng: với hai loop mâu thuẫn và không có loop thứ ba, PCM không biết loop nào đúng.

**Khi đầy.** Khi đủ `pcm_max_loops` ô, loop cũ nhất rời PCM:
- nếu nó đang được chọn thì nó **đóng băng** trong graph: vẫn ở đó, nhưng không còn được xét lại;
- nếu không thì nó bị bỏ.

Chi phí mỗi loop mới là 64 phép thử cặp (ma trận 6x6) và một max clique trên tối đa 64 đỉnh, cỡ micro giây đến dưới 1 ms.

### 2. Loop mất nhất quán muộn: gỡ factor khỏi iSAM2 (`Graph::remove_loop_factor`), không dùng biến công tắc

- API bổ sung trong backend: `remove_loop_factor(i, j)`.
  - Factor còn chờ thì bị bỏ khỏi tập chờ.
  - Factor đã commit được tìm theo con trỏ trong `getFactorsUnsafe()`; chỉ số của nó được xếp hàng và gửi làm `removeFactorIndices` trong lần `optimize()` kế tiếp (cùng lần cập nhật đó, không tốn thêm một lần cập nhật riêng). Nó rời `committed_factors` ngay, nên nếu lần cập nhật đó lỗi thì iSAM2 dựng lại từ `committed_factors` không còn nó: việc gỡ vẫn có hiệu lực.
  - `has_loop_factor(i, j)` cho biết một loop còn trong graph không (để pipeline đồng bộ selector với graph sau lỗi).
- **Chi phí:** không có lần cập nhật riêng; lần `optimize()` sau lỗi kế tiếp tốn như sau một loop mới (đo: một `optimize()` 0,8 / 2,2 / 8,4 ms ở 61 / 300 / 1000 keyframe). Chỉ khi PCM đổi ý. Sau đó pipeline tối ưu thêm `isam_extra_updates` lần, dựng lại bản đồ và đồng bộ front-end như sau một loop mới (ADR-016).
- **Không dùng switchable constraints / DCS:**
  - (a) Biến công tắc làm bài toán không lồi, và một loop sai có thể giữ `s` lớn khi nó gần với graph đã trôi: đúng trường hợp aliasing mà PCM và kiểm tra nhập nhằng xử lý.
  - (b) DCS là một kernel, nên nó có cùng giới hạn với Cauchy (C1-13): hiệu chỉnh lớn đúng bị coi như ngoại lai.
  - (c) Mỗi loop thêm một biến và một factor tự viết cần test Jacobian, tức thêm biến cho iSAM2 mà không có lợi so với gỡ factor có chủ đích.
- **Không dùng GNC:**
  - `GncOptimizer` là batch: mỗi lần chạy giải lại cả graph nhiều vòng, cỡ trăm ms đến giây ở 1000 keyframe trên desktop và chậm hơn nhiều lần trên Pi 5.
  - Để dùng với iSAM2 phải chạy định kỳ ngoài luồng ước lượng (ADR-005 chưa có), rồi dựng lại iSAM2 từ kết quả.
  - GNC cũng dựa vào phần dư so với graph, tức lại tin hiệp phương sai tuyệt đối.

### 3. Cauchy giữ nguyên làm lớp cuối

Mỗi factor loop vẫn bọc Cauchy (ADR-015, ADR-026). PCM chọn tập; Cauchy giới hạn thiệt hại của một loop sai lọt qua cả PCM. Một loop sai lọt qua PCM khi nó là loop đầu tiên hoặc khi nó nhất quán với các loop sai khác (aliasing nhất quán, xem mục 5). Giới hạn của Cauchy với hiệu chỉnh lớn đúng (C1-13) không đổi trong ADR này.

### 4. Cổng Mahalanobis tuyệt đối: giữ làm tuỳ chọn, mặc định tắt (`loop_closure.graph_covariance_gate: false`)

Giữ, không xoá, vì ba lý do:
- (a) Khi hiệp phương sai đã hiệu chỉnh theo NEES (C1-3), cổng sẽ chặn sớm và rẻ những loop mà graph thật sự loại trừ.
- (b) Cùng cờ đó bật việc graph phân xử trong kiểm tra nhập nhằng và việc bỏ các hạt giống mà graph đã loại. Cả hai giảm chi phí kiểm tra nhập nhằng về 0 trên graph cứng.
- (c) Mã và test đã có (`relative_pose_cov`, M-2).

Tắt mặc định, vì với graph tự tin quá mức nó loại hết loop đúng (review I-1, có test ghim). Khi tắt, `relative_pose_cov` không được gọi, nên không có chi phí. Khi bật mà marginal lỗi, ứng viên bị loại (`kNoMarginal`): pipeline ghi log lý do và đếm `n_loops_no_marginal` (review M-5).

### 5. Kiểm tra nhập nhằng giữ, đứng một mình

Kiểm tra nhập nhằng (ICP từ hạt giống dời, ADR-026 sửa đổi B) giữ nguyên, nhưng khi cổng tắt nó không có graph phân xử: có đối thủ là loại. Đây là lớp chặn aliasing. Ở cấu trúc lặp hoàn toàn, mọi lần dò đều cho hai nghiệm gần như nhau, và PCM không giúp được: các loop sai cùng một bản sao lệch đều một chu kỳ, nên nhất quán với nhau. Cái giá là:
- không còn loop nào trong hành lang lặp đúng chu kỳ (precision hơn recall);
- tối đa 4 lần ICP thêm cho mỗi ứng viên sắp được nhận, khoảng +30 ms ở keyframe đó trên desktop (review H3, I-2). Ba biện pháp không cần cổng graph giới hạn chi phí đó: lọc trước mỗi hạt giống dời bằng một lượt độ khớp rẻ (`ambiguity_prescreen_ratio`), giới hạn số vòng ICP của hạt giống dời (`ambiguity_icp_max_iterations`), và tạm dừng tìm loop sau một kết quả nhập nhằng (`ambiguity_backoff_keyframes`). Số đo E-01, E-07 trong `loop_closure.md` mục 12.

### 6. Cờ và khoá YAML (dưới `loop_closure.*`)

| Khoá | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `pcm_enabled` | `true` | Chỉ có tác dụng khi `enabled`. Tắt thì mọi loop qua xác minh vào thẳng graph, như trước ADR này. |
| `pcm_chi2_gate` | 22,46 | Ngưỡng chi-square của phép thử cặp. |
| `pcm_max_loops` | 64 | Từ 1 đến 64. |
| `pcm_odometry_sigma_scale` | 1,5 | ≥ 1. Hệ số nới sigma odometry (phần nhiễu trắng) trong phép thử cặp (mục 1). Sau sửa đổi C chỉ che nhiễu trắng khai báo nhỏ hơn thật tới 1,5 lần. **Giới hạn thật (review H3 I-1):** sigma khai báo ×0,5 (nhỏ hơn 2 lần) chỉ đóng được với chu trình tới khoảng 125 m; vòng 250 m (bán kính 40 m) mất 1 đến 3 loop đúng ở 3 trên 7 seed (chi2 tới 38). Với 1,0 thì đã mất một loop ở vòng 57 m (bán kính 9 m). Chủ dự án chọn 1,5 hay 2 sau một lần thử tay; chưa hiệu chỉnh (C1-27). |
| `pcm_yaw_drift_deg_per_m` | 0,2 | ≥ 0. Một sigma của tốc độ trôi yaw không biết của odometry, độ trên mét đi được (sửa đổi C). Mặc định theo mô hình odometry của chính repo (bias yaw 0,1° mỗi bước 0,5 m của odometry tổng hợp); lớn so với LiDAR thật, thay bằng giá trị đo khi hiệu chỉnh (C1-27). |
| `pcm_max_clique_steps` | 100000 | Giới hạn số bước tìm max clique; tìm bị cắt thì bắt đầu từ tập đang chọn, nên không bao giờ tệ hơn. |
| `graph_covariance_gate` | `false` | Bật cổng tuyệt đối (mục 4). |
| `correction_chi2_gate` | 22,46 | Ngưỡng của cổng tuyệt đối (chỉ khi bật). |
| `ambiguity_seeds_per_side`, `ambiguity_seed_step_m`, `ambiguity_min_separation_m`, `ambiguity_min_separation_deg`, `ambiguity_max_ratio` | 2; 1,0 m; 0,5 m; 5°; 0,9 | Kiểm tra nhập nhằng (mục 5, ADR-026 sửa đổi B). |
| `ambiguity_chi2_margin` | 13,8 | Graph phân xử đối thủ (chỉ khi cổng tuyệt đối bật). |
| `ambiguity_backoff_keyframes` | 5 | Sau một kết quả nhập nhằng, bỏ tìm loop chừng ấy keyframe (lối đi lặp: không trả 1 + 4 lần ICP mỗi keyframe). |
| `ambiguity_icp_max_iterations` | 10 | Số vòng ICP tối đa của hạt giống dời: đối thủ chỉ cần tới lưu vực của bản sao. |
| `ambiguity_prescreen_ratio` | 0,8 | Trước mỗi ICP của hạt giống dời, một lượt độ khớp (không ICP) tại pose dời; chỉ chạy ICP khi tỉ số inlier ≥ giá trị này × của nghiệm tốt nhất. |

## Sửa đổi C (2026-10-30): trôi yaw trong hiệp phương sai odometry của PCM (review H3, B-1)

Chủ dự án chọn phương án C: thêm vào hiệp phương sai odometry mà PCM dùng một số hạng trôi yaw tăng theo quãng đường, gồm tốc độ trôi yaw không biết, hằng trên đoạn (`pcm_yaw_drift_deg_per_m`, mặc định 0,2°/m theo bias của odometry tổng hợp của repo), và sai số ngang mà nó gây ra (bằng quãng đường nhân sai số yaw). Lý do: đây là sửa đúng nguyên nhân (một sai số odometry có thật nhưng thiếu trong mô hình), và nó giữ cho PCM có hiệu lực với các loop xa nhau: chu trình dài vẫn là bằng chứng, chỉ với hiệp phương sai đúng hơn. Công thức, test sai phân số và số đo: `loop_closure.md` mục 11b. Hệ số `s` giảm từ 2 xuống 1,5.

Giới hạn: số hạng trôi là bậc nhất theo `b`. Khi trôi thật lớn hơn 1,5 đến 2 lần `sigma_b` trên vòng từ 125 m trở lên, yaw tích luỹ trên một vòng (khoảng 50 đến 100°) vượt xa vùng tuyến tính và loop đúng lại bị loại hoặc gỡ (tới 9 trên 12 ở 2 sigma, bán kính 40 m, ×0,5). Mô hình đúng khi `sigma_b` chặn được trôi thật; hiệu chỉnh `sigma_b` (C1-27) quyết định điều đó. Lần quay lại theo chiều ngược làm trôi chung cộng dồn giữa hai đoạn (phép thử chặt hơn), nhưng ở mặc định chưa ảnh hưởng (chi2 ≤ 8,4 tới 100 m).

Các phương án bị loại:
- **A, chu trình dài không phải bằng chứng:** bỏ đi đúng loại bằng chứng giúp loại alias khi không có loop đúng ở gần, và ngưỡng độ dài lại là một tham số không có cơ sở vật lý.
- **B, chỉ gỡ loop khi có bằng chứng chu trình ngắn:** vẫn từ chối loop đúng mới trên chu trình dài (chỉ tránh gỡ loop cũ), nên recall vẫn mất, và quy tắc thêm một trường hợp đặc biệt vào việc chọn tập.

## Hệ quả

- Cờ loop tắt (mặc định): E-01..E-03 trùng md5. Số đo và các test có đáp án biết trước nằm trong PR B và trong `docs/derivations/loop_closure.md` mục 11 đến 12.
- **Có thể làm được:**
  - Loop đúng đóng được trên graph tự tin quá mức.
  - Một loop sai đã commit được gỡ ngay khi các loop sau mâu thuẫn với nó.
- **Không làm được:**
  - Hai loop mâu thuẫn mà không có loop thứ ba: giữ loop đến trước.
  - Một chuỗi loop sai nhất quán với nhau, tức aliasing nhất quán, nếu kiểm tra nhập nhằng không bắt được.
  - Hiệu chỉnh lớn đúng vẫn bị Cauchy giảm (C1-13).
- **Hợp đồng đổi (bổ sung):**
  - `graph.hpp`: `relative_pose_cov`, `remove_loop_factor` (PR A).
  - `loop_selector.hpp` mới.
  - `LoopConfig`: khoá mới.
  - `Estimator::Stats`: `n_loops_pcm_rejected`, `n_loops_removed`, `n_loops_no_marginal` (PR B).
- Việc còn mở: hiệu chỉnh NEES (C1-3) rồi xét bật `graph_covariance_gate`; kernel cho hiệu chỉnh lớn (C1-13); đo trên Pi 5 (C1-5, C1-15).
