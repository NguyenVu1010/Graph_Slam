# ADR-030: Loop closure với trôi cỡ mét: bỏ kernel Cauchy chỉ khi có bằng chứng từ hai nơi, tìm nơi chốn trong bán kính có giới hạn

- Trạng thái: đề xuất
- Ngày: 2026-11-03; sửa vòng review 1 (B-1, B-2, B-3) cùng ngày
- Sửa đổi: ADR-028 quyết định 3 ("Cauchy giữ nguyên làm lớp cuối") và ADR-026 mục 1 (ứng viên gần nhất trước) cùng câu về ngân sách mỗi keyframe, chỉ khi bật cờ mới
- Liên quan: ADR-015 (Cauchy cho loop), ADR-016 (dựng lại bản đồ), ADR-026 (loop closure), ADR-028 (PCM), backlog C1-1, C1-13, C1-31, C1-34; chẩn đoán và số đo: `docs/derivations/loop_closure.md` mục 13 đến 15

## Bối cảnh

Chẩn đoán E-07 (vòng ngoài trời 312 m, `loop_closure.md` mục 13) cho hai chỗ chặn:

- **Chọn ứng viên.** Với trôi 3 đến 4 m ở lần quay lại, keyframe gần nhất trên ước lượng không phải nơi quay lại (cách 2,6 đến 3,4 m thật), trong khi nơi thật (0,0 đến 0,5 m) vẫn nằm trong bán kính 10 m nhưng không bao giờ được thử.
- **Kernel.** Loop nào qua được đều đúng (vài cm, dưới 0,1°), nhưng Cauchy với sàn 5 cm chỉ áp dụng 5 đến 34 % hiệu chỉnh nó đòi. Lực của Cauchy ở phần dư lớn xấp xỉ `k² / r`, không phụ thuộc sàn; chuỗi odometry chỉ nhường khi sigma tương đối của nó cỡ `c / k`. Chỉ nhìn phần dư thì một loop sai 3 m và một loop đúng 3 m giống hệt nhau.

ADR-028 giữ Cauchy làm lớp cuối chính vì trường hợp loop sai lọt PCM: loop đầu tiên, hoặc loop **nhất quán với các loop sai khác** (aliasing nhất quán). Review vòng 1 (B-1) chỉ ra rằng bản đầu của ADR này bỏ kernel đúng ở chỗ đó: hai loop của cùng một chỗ giống nhau, từ hai truy vấn gần nhau, có cùng độ lệch sai, nên chu trình của chúng triệt tiêu và PCM cho qua.

## Quyết định

### 1. Bỏ kernel chỉ với một "bạn" ở nơi khác (`loop_closure.kernel_release`, mặc định tắt)

Một loop được PCM chọn vào graph **không có kernel** (Gaussian thuần, hiệp phương sai có sàn như cũ) chỉ khi có một loop khác trong graph, gọi là **bạn**, thoả cả bốn điều kiện:

1. **Cửa sổ:** `i` và `j` của hai loop cách nhau không quá `kernel_release_window_keyframes` (20) keyframe. Cả hai đang được PCM giữ (xem 4), nên đã nhất quán từng cặp qua một chu trình ngắn.
2. **Hai nơi:** hai ứng viên cách nhau ít nhất `kernel_release_min_separation_m` (5 m) trên graph, và hai truy vấn cũng vậy. Đây là bằng chứng mà một chỗ giống nhau (aliasing) không tạo được: hai truy vấn gần nhau nhìn cùng một cấu trúc lặp và submap chồng nhau cho hai loop sai cùng độ lệch, nhưng các ứng viên của chúng nằm trong phạm vi của submap (`submap_keyframes` 2 mỗi bên) cộng tầm tìm nhập nhằng (2 m). 5 m vượt cả hai. Để hai loop sai nhất quán ở hai nơi cách nhau 5 m, cả hai nơi phải bị lặp cùng một độ lệch: đó là hành lang lặp dài, việc của kiểm tra nhập nhằng.
3. **Trôi hợp lý:** hiệu chỉnh mỗi loop đòi (`|p(Z) - p(T_w_bi^-1 T_w_bj)|` trên graph lúc loop vào) không vượt `kernel_release_drift_ratio` (0,02) lần quãng đường đã đi từ `i` đến `j`. Một loop sai cỡ mét trên đoạn ngắn không được bỏ kernel. Đây là cổng theo quãng đường mà C1-1 cần, ở đây chỉ dùng cho quyết định bỏ kernel.
   - **Cơ sở của 2 % (review N-2), là giá trị của cảnh, không phải hiệu chuẩn:** trôi đo được của pipeline ở lần quay lại E-07 là 2,6 đến 4,1 m trên 312 m (0,8 đến 1,3 %) trên ba bản ghi; odometry bánh xe thô 0,87 % (E-07) và 1,74 % (E-07W) theo báo cáo nhiễu. 2 % là khoảng 1,5 lần trôi lớn nhất của pipeline đã đo, để một loop đúng của E-07 qua với dư. Cần đo lại trên dữ liệu thật.
   - **Trên vòng ngắn, đây là lớp bảo vệ thực sự:** với `min_interval_keyframes` 5 và keyframe 1 m, các loop kế nhau đã cách nhau khoảng 5 m khi đi thẳng, nên điều kiện tách nơi ít khi chặn (E-07W: 15 trên 16 loop được bỏ kernel, đều đúng). Cái chặn một cặp loop sai cỡ một chu kỳ (3 đến 5 m, kiểu C1-36) trên E-07W là tỉ số trôi: 2 % của một vòng 24 m là 0,48 m, của hai vòng 0,96 m, nhỏ hơn mọi hiệu chỉnh cỡ chu kỳ.
4. **PCM còn giữ cả hai:** `load_config` đòi `pcm_max_loops > kernel_release_window_keyframes + 1` (một loop bị PCM từ chối không bỏ qua keyframe nào, nên tới cửa sổ + 1 loop có thể rơi vào một cửa sổ), nên mọi loop trong cửa sổ còn được PCM xét, không phải loop đã "đóng băng" (review M-1, N-3).

Loop không có bạn giữ Cauchy như trước: loop đầu tiên, loop đơn lẻ, cặp loop từ một chỗ. Mỗi lần PCM đổi tập, mọi loop trong graph được xét lại: loop có bạn được gỡ rồi thêm lại không kernel trong cùng lần cập nhật, loop mất bạn được thêm lại có Cauchy (`loop closure kernel released|restored` trong log, `Stats::n_loops_released` đếm khi lần cập nhật thành công). Nếu lần cập nhật đó lỗi, việc gỡ vẫn có hiệu lực còn factor mới bị bỏ, nên loop rời graph và selector quên nó (`graph.hpp`; test `KernelReleaseFailedUpdateDropsTheReleasedPair`).

**Quyết định của chủ dự án cần ghi rõ:** giả thiết của ADR-028 ("Cauchy là lớp cuối trước aliasing nhất quán") được giữ cho cặp loop từ một chỗ và mọi loop không có bạn ở nơi khác. Nó chỉ được bỏ cho cặp loop ở hai nơi cách nhau từ 5 m, với hiệu chỉnh hợp lý theo quãng đường. Phần rủi ro còn lại (hai nơi cùng bị lặp một độ lệch, lọt kiểm tra nhập nhằng) chấp nhận khi bật cờ.

Test có đáp án: `LoopKnownLimits.KernelReleaseConsistentWrongPair` (chỉ cho **một chỗ**: hai loop sai cùng độ lệch 1 m và 3 m, cách 5 keyframe, ứng viên cách nhau 1,7 m: không loop nào được bỏ kernel, chạy trùng bit với cờ tắt, cặp kéo 0,131 / 0,046 m như hai loop Cauchy, so với 0,942 / 2,935 m nếu bỏ kernel không xét tách nơi), `LoopKernelRelease.PartnerRuleNeedsTwoPlacesAndPlausibleDrift` (gồm cặp loop sai nhất quán ở hai nơi cách 6 m: là bạn, giới hạn chấp nhận; và cặp sai cỡ chu kỳ sau một vòng 24 m: tỉ số trôi chặn), `LoopComponent.KernelRelease*`.

Backend thêm overload `Graph::add_loop_factor(lc, LoopKernel)` (`kCauchy`, `kNone`); overload cũ là `kCauchy`. GTSAM vẫn chỉ ở backend.

### 2. Tìm nơi chốn trong bán kính (`loop_closure.place_search`, mặc định tắt)

- **Xếp hạng có giới hạn (B-3):** chỉ `place_search_max_ranked` (20, kiểm tra `>= max_candidates`) ứng viên gần nhất trong `search_radius_m` được tính Scan Context và xếp theo nó. Mỗi descriptor tính một lần trong mỗi lần tìm (ứng viên được xác minh dùng lại khoảng cách và độ lệch cột đã có). Câu "ngân sách mỗi keyframe bị chặn bởi `max_candidates`" của ADR-026 thành: chặn bởi `max_candidates` lần xác minh và, khi cờ bật, `place_search_max_ranked` descriptor. **Bộ nhớ đệm descriptor Scan Context** (review N-5, N2-1): với `place_search`, descriptor của mỗi keyframe được tính lần đầu cần đến và giữ trong `map.max_keyframes` ô (khoảng 4,8 MB ở 1000 keyframe, cộng một ma trận 3x3 mỗi ô), cấp phát trong constructor, không cấp phát khi tìm. Mỗi ô giữ roll và pitch đã dùng để cân bằng descriptor; khi roll và pitch hiện tại của keyframe lệch quá `place_search_cache_tilt_deg` (0,1°, kiểm tra > 0) thì descriptor được tính lại, nên một hiệu chỉnh graph làm nghiêng keyframe không để lại descriptor cũ. Không thêm API: việc làm mới nằm trong detector. Test: `LoopDetector.PlaceSearchCacheRefreshesOnTiltChange`, `PlaceSearchCostIsBoundedOnADenseRevisit` (đếm số descriptor tính lại qua bộ đếm nội bộ `loop_detector_internal.hpp`).
- **Nơi phải khác biệt (B-2):** xếp hạng và hạt giống nơi chốn chỉ dùng khi ứng viên giống nhất **khác biệt**: khoảng cách Scan Context của nó không quá `place_search_sc_ratio` (0,8) lần khoảng cách tốt nhất trong các ứng viên cách nó ít nhất `place_search_min_separation_m` (2 m, ngoài các keyframe kề của cùng một nơi). Nếu không (một bản sao ở nơi khác giống ngang), tìm kiếm quay về gần nhất trước và chỉ hạt giống graph, như khi tắt cờ.
- **Hạt giống nơi chốn:** truy vấn đặt tại vị trí ứng viên với yaw Scan Context. Kết quả của nó chỉ là loop khi hiệu chỉnh không vượt `icp_max_corr_dist_m` cộng `place_search_drift_ratio` (2 %) lần quãng đường từ ứng viên đến truy vấn (review N-4: một bản sao duy nhất được lưu, cách vài mét theo đường đi ngắn, cho 7 loop sai trong 30 trường hợp khi thiếu điều này, 0 khi có). Kết quả của nó mà chỉ vị trí vượt `max_correction_m` (yaw trong cổng) không thành loop được nhưng vẫn là **đối thủ** trong kiểm tra nhập nhằng; thiếu điều này E-07 với cổng 3 m nhận một loop sai 2,7 m.
- Test: `PlaceSearchFixture.*`, `LoopAliasing.NoFalseLoopAtAnyDriftWithPlaceSearch`, `LoopAliasing.LongPeriodCopyRankedFirstNoFalseLoopWithLoopSceneSettings` (chu kỳ 3, 4, 5 m, bản sao xếp trước: 0 loop sai trong 30 trường hợp; bộ dò mặc định có 7, giới hạn có từ trước, C1-36), `LoopFixture.L02_DifferentPlacesNoFalseLoopWithPlaceSearch`, `LoopDetector.PlaceSearchCostIsBoundedOnADenseRevisit`.

### 3. Cổng hiệu chỉnh của cảnh loop

`max_correction_m` mặc định giữ 3 m. Chỉ overlay E-07 đặt 6 m cùng `place_search`. Overlay E-07W không bật `place_search` cũng không nới cổng (B-2: trôi vài cm, và các dãy kệ là nơi dễ có chỗ giống nhau nhất); nó giữ sàn mới và `kernel_release`.

### 4. Đo đạc (không đổi ước lượng)

- `LoopResult::correction_m`, `correction_yaw_rad`: hiệu chỉnh mà nghiệm được báo đòi so với pose graph của truy vấn; khi không giữ nghiệm nào, của hạt giống hội tụ cuối cùng (có thể là hạt giống nơi chốn).
- Pipeline ghi một dòng debug mỗi lần tìm có ứng viên (`loop search: ..`) và một dòng `loop applied: ..` cho mỗi loop vào graph.
- `mss_bench` ghi thêm `kf_opt.tum`: các keyframe còn trong kho tại pose đã tối ưu sau khi phát lại (quỹ đạo đã làm mượt). Theo quyết định của chủ dự án, E-07 và E-07W chấm trên `kf_opt.tum`; E-01..E-03 vẫn chấm trên `est.tum`.
- `tools/loop_eval.py` tính ATE, sai số quay lại và cuối, NEES của loop so với ground truth từ các file đó (`results/<run-id>/loop_eval.json`).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Bỏ kernel khi có bạn trong cửa sổ, không xét tách nơi (bản đầu) | Đơn giản | Review B-1: cặp loop sai từ một chỗ có đúng chữ ký đó; test: kéo 0,94 / 2,94 m |
| Chỉ cổng trôi theo quãng đường | Không tham số tách nơi | Sau một vòng dài cổng cho cả độ lệch của aliasing (2 % của 300 m là 6 m) |
| Bạn phải từ một lần quay lại khác (cách nhau theo thời gian) | Độc lập mạnh hơn | E-07 chỉ có một lần quay lại: không bao giờ bỏ kernel, E-07 không đóng |
| Nới kernel theo số bạn độc lập | Từ từ | Thêm tham số; một bạn ở nơi khác đã là bằng chứng chính |
| Độ rộng kernel theo hiệp phương sai đổi mới (graph + loop) | Có cơ sở thống kê | Graph tự tin quá mức (ADR-028) |
| Switchable constraints / DCS, GNC | Chuẩn trong tài liệu | Như ADR-028: cùng giới hạn của kernel, hoặc batch |
| Xếp mọi ứng viên trong bán kính theo Scan Context | Không cần tham số | Không chặn chi phí: 300 ứng viên 126 ms mỗi lần tìm |
| Tỉ số khoảng cách Scan Context tốt nhất / tốt nhì không xét khoảng cách | Đơn giản | Keyframe kề của chính nơi đó luôn là tốt nhì |

## Hệ quả

Số đo: `loop_closure.md` mục 15 và báo cáo PR.

- Cờ tắt (mặc định): không có gì đổi. `est.tum` của E-01..E-03 trùng md5 với main.
- E-07 (bản ghi C, trôi 3,7 m ở lần quay lại) với overlay cảnh loop: 5 loop, 0 sai, cả 5 được bỏ kernel; ATE của `kf_opt.tum` 1,016 xuống 0,152 m, sai số ngang ở lần quay lại 3,68 xuống 0,042 m. Cài đặt loop gốc: 2 loop, 0,870 m. Bản ghi B: 0,885 xuống 0,161 m. E-07W: 16 loop, 0 sai, ATE `kf_opt.tum` 0,0192 xuống 0,0142 m, ATE `est.tum` 0,0163 và 0,0151 m. Lệnh tái tạo: `loop_closure.md` mục 15.
- **Rủi ro còn lại, nói thẳng:**
  - Hai loop sai nhất quán ở hai nơi cách nhau từ 5 m (hành lang lặp dài mà kiểm tra nhập nhằng bỏ sót) vẫn được bỏ kernel (C1-34). Test ghim: `PartnerRuleNeedsTwoPlacesAndPlausibleDrift` (cặp sai ở hai nơi cách 6 m: là bạn).
  - **Vòng trong nhà lớn:** trên một vòng 200 m trở lên, 2 % là 4 m trở lên, nên một cặp loop sai cỡ chu kỳ từ bộ dò mặc định ở dãy kệ dài (C1-36, chu kỳ 3 đến 5 m ngoài tầm tìm nhập nhằng) qua được cả tỉ số trôi và điều kiện tách nơi, và kéo graph đủ. Chưa được bật ở cảnh nào như vậy; trước khi bật `kernel_release` trên một vòng trong nhà lớn cần sửa C1-36 hoặc C1-24 (C1-34).
  - Loop đầu tiên của một lần quay lại giữ kernel và chỉ áp dụng vài phần trăm cho đến khi bạn của nó đến.
  - Một loop bỏ kernel với hiệu chỉnh vài mét làm iSAM2 tuyến tính hoá lại gần như cả graph: keyframe đó tốn thêm (C1-35).
  - `max_correction_m` 6 m chỉ ở overlay E-07; một cổng phát hiện theo quãng đường là việc còn mở (C1-1).
  - Phép thử bạn dùng lại phép thử cặp của PCM; với `pcm_odometry_sigma_scale` 2 một loop sai có thể không bị mâu thuẫn. Chủ dự án giữ 1,5 (C1-27 đóng).
- Hợp đồng đổi (bổ sung, chủ dự án đã đồng ý): `graph.hpp` (`LoopKernel`, overload `add_loop_factor`), `loop/config.hpp` (`kernel_release`, `kernel_release_window_keyframes`, `kernel_release_min_separation_m`, `kernel_release_drift_ratio`, `place_search`, `place_search_max_ranked`, `place_search_sc_ratio`, `place_search_min_separation_m`, `place_search_drift_ratio`, `place_search_cache_tilt_deg`), `loop_detector.hpp` (`LoopResult::correction_m`, `correction_yaw_rad`), `estimator.hpp` (`Stats::n_loops_released`), `bench.json` (`n_loops_released`), `kf_opt.tum` của `mss_bench`.
