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
- Selector giữ hai tổng tiền tố: pose `T_0k`, và hiệp phương sai trong tiếp tuyến trái của gốc, `S_k = S_{k-1} + Ad(T_0k) C_k Ad(T_0k)^T`.
- Odometry giữa `a < b` vì thế là O(1): `T_ab = T_0a^-1 T_0b`, `C_ab = Ad(T_0b^-1) (S_b - S_a) Ad(T_0b^-1)^T` (derivation `loop_closure.md` mục 11).
- Đây là odometry thuần, không phải marginal của graph: marginal đã chứa chính các loop đang bị xét và mọi loop khác, nên không còn là phép thử độc lập.

**Phép thử cặp.**
- Hai loop `a`, `b` tạo một chu trình `X = Z_a^-1 · O(i_a → i_b) · Z_b · O(j_b → j_a)`. Hiệp phương sai được ghép theo nhiễu phải từ bốn đoạn, coi chúng độc lập.
- Nhất quán khi `e^T Σ_X^-1 e ≤ pcm_chi2_gate` (22,46, phân vị 0,999 với 6 bậc tự do), với `e = Log(X)`.
- Độ trôi tích luỹ giữa vòng 1 và vòng 2 xuất hiện ở cả hai loop nên bị triệt tiêu. Chu trình chỉ chứa hai đoạn odometry ngắn giữa hai loop và hai sàn hiệp phương sai loop (5 cm, 1°).
- Graph tự tin quá mức vì vậy ảnh hưởng ít hơn nhiều so với cổng tuyệt đối. Đây là lý do chọn PCM.

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
  - Factor đã commit được tìm theo con trỏ trong `getFactorsUnsafe()` và gỡ bằng một `update` với `removeFactorIndices`, giống cách đã gỡ factor trọng lực (ADR-020). Nó cũng bị bỏ khỏi `committed_factors`, để lần dựng lại sau lỗi không đưa nó trở lại.
  - Nếu gỡ lỗi thì iSAM2 được dựng lại và factor ở lại, có ghi log.
- **Chi phí:** một lần cập nhật iSAM2, chỉ chạy khi PCM đổi ý, tức hiếm hơn nhiều so với số loop. Sau đó pipeline tối ưu thêm `isam_extra_updates` lần, dựng lại bản đồ và đồng bộ front-end như sau một loop mới (ADR-016).
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
- tối đa 4 lần ICP thêm cho mỗi ứng viên sắp được nhận, khoảng +30 ms ở keyframe đó trên desktop; số đo ở dưới.

### 6. Cờ và khoá YAML (dưới `loop_closure.*`)

| Khoá | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `pcm_enabled` | `true` | Chỉ có tác dụng khi `enabled`. Tắt thì mọi loop qua xác minh vào thẳng graph, như trước ADR này. |
| `pcm_chi2_gate` | 22,46 | Ngưỡng chi-square của phép thử cặp. |
| `pcm_max_loops` | 64 | Từ 1 đến 64. |
| `pcm_max_clique_steps` | 100000 | Giới hạn số bước tìm max clique. |
| `graph_covariance_gate` | `false` | Bật cổng tuyệt đối. |

Các khoá cũ của cổng (`correction_chi2_gate`, `ambiguity_chi2_margin`) chỉ được dùng khi `graph_covariance_gate` bật.

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
