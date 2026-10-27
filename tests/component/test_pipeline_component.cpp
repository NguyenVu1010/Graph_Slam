// Component tests of the pipeline estimator on the synthetic box world
// (tests/component/box_world.hpp, docs/derivations/pipeline.md section 7): trajectory
// accuracy, allocation-free non-keyframe scans and the robustness cases of the plan's
// Review Focus list.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_reader.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"

#include "alloc_counter.hpp"
#include "box_world.hpp"
#include "estimator_core.hpp"

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Estimate;
using common::ImuSample;
using common::ScanFrame;
using common::Timestamp;
using common::Vec3;
using test::BoxWorldParams;

const fs::path kDefault = fs::path{MSS_REPO_DIR} / "config/default.yaml";
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr bool kWithGtsam = MSS_TEST_WITH_GTSAM != 0;

fs::path test_dir() {
  const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
  return fs::path{MSS_TEST_OUTPUT_DIR} / ("pipeline_" + name);
}

// config/default.yaml + the box-world overlay (written next to the dataset).
Config box_config(const BoxWorldParams& p, bool gnss_and_wheel, const fs::path& dir) {
  fs::create_directories(dir);
  const fs::path overlay = dir / "overlay.yaml";
  {
    std::ofstream f(overlay);
    f << test::box_overlay_yaml(p, gnss_and_wheel);
  }
  const auto cfg = load_config(kDefault, overlay);
  EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
  return cfg.value();
}

struct ReplayResult {
  std::vector<Estimate> estimates;  // one per new latest_state().state.t
  Estimator::Stats stats;
};

// Replays an mssdata directory in time order, run_once() after every pushed record
// (so before and after every scan push), flush() at the end. `after_scan(est, k)` (if set)
// runs after the k-th scan record (k from 1) has been pushed and drained.
ReplayResult replay(const fs::path& data,
                    const Config& cfg,
                    const std::function<void(Estimator&, int)>& after_scan = {}) {
  ReplayResult out;
  auto opened = io::DatasetReader::open(data, cfg.realtime.max_points_per_scan);
  EXPECT_TRUE(opened.is_ok()) << opened.reason();
  if (!opened.is_ok()) {
    return out;
  }
  io::DatasetReader reader = std::move(opened).value();
  Estimator est(cfg);
  ScanFrame scan(cfg.realtime.max_points_per_scan);
  io::Record rec;
  Timestamp last{};
  int n_scan_records = 0;
  const auto collect = [&]() {
    const Estimate e = est.latest_state();
    if (e.state.t != Timestamp{} && e.state.t != last) {
      out.estimates.push_back(e);
      last = e.state.t;
    }
  };
  for (;;) {
    const auto more = reader.next(rec, scan);
    EXPECT_TRUE(more.is_ok()) << more.reason();
    if (!more.is_ok() || !more.value()) {
      break;
    }
    switch (rec.kind) {
      case io::RecordKind::kImu:
        (void)est.push_imu(rec.imu);
        break;
      case io::RecordKind::kGnss:
        (void)est.push_gnss(rec.gnss);
        break;
      case io::RecordKind::kOdom:
        (void)est.push_odom(rec.odom);
        break;
      case io::RecordKind::kLidar:
        est.run_once();
        collect();
        (void)est.push_scan(scan);
        ++n_scan_records;
        break;
      case io::RecordKind::kGt:
        break;
    }
    est.run_once();
    collect();
    if (rec.kind == io::RecordKind::kLidar && after_scan) {
      after_scan(est, n_scan_records);
      est.run_once();
      collect();
    }
  }
  est.flush();
  collect();
  out.stats = est.stats();
  return out;
}

struct Errors {
  double ate_m = 0.0;
  double max_yaw_deg = 0.0;
};

Errors trajectory_errors(const BoxWorldParams& p, const std::vector<Estimate>& est) {
  Errors e;
  double sum_sq = 0.0;
  for (const Estimate& x : est) {
    const common::NavState truth = test::box_truth(p, x.state.t);
    sum_sq += (x.state.T_w_b.p - truth.T_w_b.p).squaredNorm();
    const common::Mat3 dR = truth.T_w_b.R.transpose() * x.state.T_w_b.R;
    const double yaw_deg = std::abs(std::atan2(dR(1, 0), dR(0, 0))) * kRadToDeg;
    e.max_yaw_deg = std::max(e.max_yaw_deg, yaw_deg);
  }
  e.ate_m = est.empty() ? 1e9 : std::sqrt(sum_sq / static_cast<double>(est.size()));
  return e;
}

// Scans with t_start >= t0 + init_window_s produce an estimate.
std::size_t expected_estimates(const BoxWorldParams& p, const Config& cfg) {
  std::size_t n = 0;
  for (int k = 1; k <= test::box_num_scans(p); ++k) {
    if (test::box_scan_time(p, k).ns >= p.t0_ns + test::box_ns(cfg.imu.init_window_s)) {
      ++n;
    }
  }
  return n;
}

// Replays the box world; with `gnss_and_wheel` also replays the same dataset LiDAR-only
// and checks GNSS-on ATE <= 1.2 x LiDAR-only ATE: on a 2 m circle GNSS at 0.5 m cannot
// improve on the LiDAR chain, so it must not degrade it (gnss_fusion_study.md section 5;
// before ADR-019 the ratio was 3.7). `after_scan` as in replay().
ReplayResult run_box_world(bool gnss_and_wheel,
                           const BoxWorldParams& p = BoxWorldParams{},
                           const std::function<void(Estimator&, int)>& after_scan = {}) {
  const fs::path dir = test_dir();
  const Config cfg = box_config(p, gnss_and_wheel, dir);
  const auto written = test::write_box_dataset(dir / "data", p);
  EXPECT_TRUE(written.is_ok()) << written.reason();
  const ReplayResult r = replay(dir / "data", cfg, after_scan);
  EXPECT_EQ(r.estimates.size(), expected_estimates(p, cfg));
  EXPECT_EQ(r.stats.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_EQ(r.stats.n_uncovered_scans, 0U);
  EXPECT_GT(r.stats.n_keyframes, 5U);
  const Errors e = trajectory_errors(p, r.estimates);
  const double ate_tol_m = kWithGtsam ? 0.05 : 0.10;
  const double yaw_tol_deg = kWithGtsam ? 1.0 : 2.0;
  EXPECT_LT(e.ate_m, ate_tol_m);
  EXPECT_LT(e.max_yaw_deg, yaw_tol_deg);
  std::printf(
      "[ box world ] gtsam %d gnss+wheel %d: ATE %.4f m, max yaw %.3f deg, %zu estimates, "
      "%llu keyframes, %llu match failures, %llu gnss factors, %llu odom factors, "
      "%llu rebuilds\n",
      kWithGtsam ? 1 : 0,
      gnss_and_wheel ? 1 : 0,
      e.ate_m,
      e.max_yaw_deg,
      r.estimates.size(),
      static_cast<unsigned long long>(r.stats.n_keyframes),
      static_cast<unsigned long long>(r.stats.n_match_failures),
      static_cast<unsigned long long>(r.stats.n_gnss_factors),
      static_cast<unsigned long long>(r.stats.n_odom_factors),
      static_cast<unsigned long long>(r.stats.n_map_rebuilds));
  if (gnss_and_wheel) {
    EXPECT_GT(r.stats.n_odom_factors, 0U);
    EXPECT_GT(r.stats.n_gnss_factors, 0U);
    const Config lidar_only = box_config(p, false, dir / "lidar_only");
    const Errors base = trajectory_errors(p, replay(dir / "data", lidar_only).estimates);
    std::printf("[ box world ] LiDAR-only ATE on the same data %.4f m (ratio %.2f)\n",
                base.ate_m,
                e.ate_m / base.ate_m);
    EXPECT_LE(e.ate_m, 1.2 * base.ate_m);
  }
  return r;
}

// GNSS must help where the LiDAR drifts (gnss_fusion_study.md section 8): an almost
// straight 30 m drive in a 120 m corridor whose end walls are out of range, so the LiDAR
// has no information along it and degeneracy relaxes that direction (relax_factor 1e8,
// scenario value). LiDAR + IMU drifts by metres; GNSS (0.5 m, 5 Hz) must halve the ATE.
TEST(PipelineComponent, CorridorDriftCorrectedByGnss) {
  BoxWorldParams p;
  p.box_half_m = Vec3(60.0, 2.5, 1.5);
  p.yaw_rate_radps = 0.0005;  // rad/s: 0.45 m lateral offset over the drive
  p.motion_s = 60.0;
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const auto run = [&](bool gnss) {
    const fs::path sub = dir / (gnss ? "gnss" : "lidar_only");
    fs::create_directories(sub);
    {
      std::ofstream f(sub / "overlay.yaml");
      f << test::box_overlay_yaml(p, false)
        << "    preprocess:\n      max_range_m: 10.0\n"
           "    degeneracy:\n      enabled: true\n      relax_factor: 1.0e8\n"
           "    gnss:\n      enabled: "
        << (gnss ? "true" : "false") << "\n";
    }
    const auto cfg = load_config(kDefault, sub / "overlay.yaml");
    EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
    const ReplayResult r = replay(dir / "data", cfg.value());
    const Errors e = trajectory_errors(p, r.estimates);
    std::printf("[ corridor ] gnss %d: ATE %.3f m, %llu keyframes, %llu gnss factors\n",
                gnss ? 1 : 0,
                e.ate_m,
                static_cast<unsigned long long>(r.stats.n_keyframes),
                static_cast<unsigned long long>(r.stats.n_gnss_factors));
    return e.ate_m;
  };
  const double lidar_only_m = run(false);
  const double with_gnss_m = run(true);
  EXPECT_GE(lidar_only_m, 0.3);
  if (kWithGtsam) {  // the stub backend adds no factor: nothing to correct with
    EXPECT_LE(with_gnss_m, 0.5 * lidar_only_m);
  }
}

TEST(PipelineComponent, BoxWorldAteWithinTolerance) {
  run_box_world(false);
}

TEST(PipelineComponent, BoxWorldWithGnssAndWheelWithinTolerance) {
  run_box_world(true);
}

// I-1: GNSS stamps 2.5 ms off the 5 ms IMU grid are deferred until the IMU covers them and
// fused at the interpolated pose; a fix older than the last processed scan is rejected.
TEST(PipelineComponent, GnssOffGridFusedAndLateFixRejected) {
  BoxWorldParams p;
  p.gnss_offset_ns = 2'500'000;
  int late_pushed = 0;
  const ReplayResult r = run_box_world(true, p, [&](Estimator& est, int k) {
    if (k == 120) {  // well after alignment: a fix stamped 1 s in the past
      common::GnssMeasurement m;
      m.t = Timestamp{test::box_scan_time(p, k).ns - 1'000'000'000};
      m.p_e_g = test::box_T_e_w(p) * (test::box_truth(p, m.t).T_w_b * p.p_b_g);
      m.cov = common::Mat3::Identity() * (p.gnss_sigma_m * p.gnss_sigma_m);
      (void)est.push_gnss(m);
      ++late_pushed;
    }
  });
  EXPECT_EQ(late_pushed, 1);
  EXPECT_EQ(r.stats.n_gnss_late, 1U);
}

TEST(PipelineComponent, NoEstimateBeforeImuInit) {
  BoxWorldParams p;
  p.static_s = 4.0;
  p.ramp_s = 0.0;
  p.motion_s = 0.0;
  p.gyro_noise_density = 0.1 / std::sqrt(p.imu_rate_hz);  // std 0.1 rad/s per sample
  const fs::path dir = test_dir();
  const Config cfg = box_config(p, false, dir);
  ASSERT_GT(0.1, cfg.imu.init_max_gyro_std_radps);
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const ReplayResult r = replay(dir / "data", cfg);
  EXPECT_TRUE(r.estimates.empty());
  EXPECT_EQ(r.stats.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_EQ(r.stats.n_keyframes, 0U);
}

// Drives EstimatorCore directly in a static box world (noise-free IMU).
class StaticCore {
 public:
  // `aux`: gnss.enabled and wheel_odom.enabled, with GNSS (5 Hz, 1.234 ms off the IMU grid)
  // and wheel (50 Hz) messages merged into the stream by time.
  explicit StaticCore(bool aux = false)
      : aux_(aux), cfg_(make_cfg(aux)), core_(std::make_unique<detail::EstimatorCore>(cfg_)) {
    scan_.reserve(static_cast<std::size_t>(p_.rings * p_.columns));
    test::box_scan(p_, Timestamp{p_.t0_ns}, scan_);  // static: identical at every time
    empty_.reserve(1);
  }

  static Config make_cfg(bool aux = false) {
    const auto loaded = load_config(kDefault);
    EXPECT_TRUE(loaded.is_ok()) << loaded.reason();
    Config c = loaded.value();
    c.frames.T_b_l = test::box_default_T_b_l();
    c.frames.p_b_g = BoxWorldParams{}.p_b_g;
    c.gnss.enabled = aux;
    c.wheel_odom.enabled = aux;
    return c;
  }

  // IMU samples (static, exact) up to and including t; a sample whose time is in `nan_`
  // has NaN acc. With `aux`, wheel and GNSS messages earlier than each IMU sample first.
  void imu_until(std::int64_t t) {
    for (; next_imu_ <= t; next_imu_ += period_) {
      if (aux_) {
        for (; next_wheel_ < next_imu_; next_wheel_ += 20'000'000) {
          core_->on_wheel(test::box_wheel(p_, Timestamp{next_wheel_}));
        }
        for (; next_gnss_ < next_imu_; next_gnss_ += 200'000'000) {
          common::GnssMeasurement m;
          m.t = Timestamp{next_gnss_};
          m.p_e_g = test::box_T_e_w(p_) * (test::box_truth(p_, m.t).T_w_b * p_.p_b_g);
          m.cov = common::Mat3::Identity() * 0.25;
          core_->on_gnss(m);
        }
      }
      ImuSample s = test::box_imu(p_, Timestamp{next_imu_}, nullptr);
      for (const std::int64_t bad : nan_) {
        if (bad == next_imu_) {
          s.acc_b.x() = std::numeric_limits<double>::quiet_NaN();
        }
      }
      core_->on_imu(s);
    }
  }

  void set_nan_times(std::vector<std::int64_t> times) { nan_ = std::move(times); }

  // Scan at t (covered by IMU first).
  void scan_at(std::int64_t t) {
    scan_.t_start = Timestamp{t};
    // Two periods: a dropped (non-finite) sample right after the end cannot cover it,
    // as in Estimator's coverage gate, which counts finite samples only.
    imu_until(common::scan_end_time(scan_).ns + 2 * period_);
    core_->on_scan(scan_);
  }

  void empty_scan_at(std::int64_t t) {
    empty_.t_start = Timestamp{t};
    imu_until(t + period_);
    core_->on_scan(empty_);
  }

  std::int64_t t_scan(int k) const { return p_.t0_ns + 2'000'000'000 + k * 100'000'000; }

  detail::EstimatorCore& core() { return *core_; }
  std::int64_t period() const { return period_; }

 private:
  BoxWorldParams p_;
  bool aux_ = false;
  Config cfg_;
  std::unique_ptr<detail::EstimatorCore> core_;
  ScanFrame scan_;
  ScanFrame empty_;
  std::int64_t period_ = 5'000'000;
  std::int64_t next_imu_ = 1'000'000'000;
  std::int64_t next_wheel_ = 1'000'000'000;
  std::int64_t next_gnss_ = 1'001'234'000;
  std::vector<std::int64_t> nan_;
};

// I-3: also with gnss.enabled and wheel_odom.enabled and their messages on the path.
TEST(PipelineComponent, NonKeyframeScanDoesNotAllocate) {
  StaticCore s(true);
  s.scan_at(s.t_scan(0));  // initialisation + keyframe 0
  ASSERT_TRUE(s.core().has_estimate());
  for (int k = 1; k <= 5; ++k) {
    s.scan_at(s.t_scan(k));  // warm-up
  }
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 6; k < 16; ++k) {
      s.scan_at(s.t_scan(k));
    }
  }
  // This thread only: a TBB-enabled GTSAM starts worker threads after the first optimize()
  // that allocate asynchronously on their own threads.
  const std::size_t allocs = AllocCounter::count_this_thread();
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(s.core().counters().n_keyframes, 1U);
  EXPECT_EQ(s.core().counters().n_match_failures, 0U);
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(15)});
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 0.02);  // static: matched near origin
}

// I-2: a NaN IMU sample inside a scan span and one exactly at a scan start are dropped;
// the neighbours bridge them and no scan is skipped.
TEST(PipelineComponent, NonFiniteImuSamplesDropped) {
  StaticCore s;
  s.set_nan_times({s.t_scan(1) + 50'000'000, s.t_scan(2)});
  s.scan_at(s.t_scan(0));
  ASSERT_TRUE(s.core().has_estimate());
  const std::uint64_t skipped = s.core().counters().n_skipped_scans;
  for (int k = 1; k <= 3; ++k) {
    s.scan_at(s.t_scan(k));
    EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(k)}) << k;
  }
  EXPECT_EQ(s.core().counters().n_skipped_scans, skipped);
  EXPECT_EQ(s.core().counters().n_nonfinite_imu, 2U);
  EXPECT_EQ(s.core().counters().n_match_failures, 0U);
  EXPECT_TRUE(s.core().latest().state.T_w_b.p.allFinite());
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 0.02);
}

TEST(PipelineComponent, ScanMatchFailureFallsBackToImuPrediction) {  // Review Focus 1
  StaticCore s;
  s.scan_at(s.t_scan(0));
  ASSERT_TRUE(s.core().has_estimate());
  for (int k = 1; k <= 5; ++k) {
    s.empty_scan_at(s.t_scan(k));
    EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(k)}) << k;
  }
  EXPECT_EQ(s.core().counters().n_match_failures, 5U);
  EXPECT_EQ(s.core().counters().n_keyframes, 1U);
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 1e-6);  // static IMU prediction
  s.scan_at(s.t_scan(6));                                   // a good scan matches again
  EXPECT_EQ(s.core().counters().n_match_failures, 5U);
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(6)});
}

// N-1: more than kMaxDeferredGnss fixes waiting for IMU overflow the ring (counted).
TEST(PipelineComponent, GnssRingOverflowCounted) {
  Config cfg = StaticCore::make_cfg(true);
  detail::EstimatorCore core(cfg);
  BoxWorldParams p;
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  const std::int64_t t_scan = p.t0_ns + 2'000'000'000;
  scan.t_start = Timestamp{t_scan};
  std::int64_t t = p.t0_ns;
  for (; t <= common::scan_end_time(scan).ns + 5'000'000; t += 5'000'000) {
    core.on_imu(test::box_imu(p, Timestamp{t}, nullptr));
  }
  core.on_scan(scan);
  ASSERT_TRUE(core.has_estimate());
  // Fixes after the last IMU sample: none can be fused yet.
  for (std::size_t k = 0; k < detail::kMaxDeferredGnss + 3; ++k) {
    common::GnssMeasurement m;
    m.t = Timestamp{t + static_cast<std::int64_t>(k) * 1'000'000};
    m.cov = common::Mat3::Identity() * 0.25;
    core.on_gnss(m);
  }
  EXPECT_EQ(core.counters().n_gnss_ring_overflow, 3U);
  core.on_end_of_data();
  EXPECT_EQ(core.counters().n_gnss_unfused_eof, detail::kMaxDeferredGnss);
}

// N-2: a fix stamped after the last IMU sample is still waiting at flush(): counted, so
// n_gnss = fused + rejected + late + overflow + unfused.
TEST(PipelineComponent, TrailingGnssFixCountedAtFlush) {
  BoxWorldParams p;
  Estimator est(StaticCore::make_cfg(true));
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  scan.t_start = Timestamp{p.t0_ns + 2'000'000'000};
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  std::int64_t t = p.t0_ns;
  for (; t <= common::scan_end_time(scan).ns + 5'000'000; t += 5'000'000) {
    ASSERT_TRUE(est.push_imu(test::box_imu(p, Timestamp{t}, nullptr)).is_ok());
    est.run_once();
  }
  ASSERT_NE(est.latest_state().state.t, Timestamp{});
  common::GnssMeasurement m;
  m.t = Timestamp{t + 1'000'000};  // after the last IMU sample
  m.cov = common::Mat3::Identity() * 0.25;
  ASSERT_TRUE(est.push_gnss(m).is_ok());
  est.flush();
  const Estimator::Stats st = est.stats();
  EXPECT_EQ(st.n_gnss, 1U);
  EXPECT_EQ(st.n_gnss_unfused_eof, 1U);
  EXPECT_EQ(st.n_gnss_ring_overflow, 0U);
  EXPECT_EQ(st.n_gnss_late, 0U);
}

TEST(PipelineComponent, OldScanSkipped) {  // Review Focus 3
  StaticCore s;
  s.scan_at(s.t_scan(0));
  s.scan_at(s.t_scan(1));
  const Estimate before = s.core().latest();
  const std::uint64_t skipped = s.core().counters().n_skipped_scans;
  s.scan_at(s.t_scan(1));  // duplicate t_start
  s.scan_at(s.t_scan(0));  // older
  const Estimate after = s.core().latest();
  EXPECT_EQ(s.core().counters().n_skipped_scans, skipped + 2);
  EXPECT_EQ(after.state.t, before.state.t);
  EXPECT_TRUE(after.state.T_w_b.R == before.state.T_w_b.R);
  EXPECT_TRUE(after.state.T_w_b.p == before.state.T_w_b.p);
  EXPECT_TRUE(after.state.v_w_b == before.state.v_w_b);
  EXPECT_TRUE(after.cov == before.cov);
  EXPECT_EQ(after.keyframe_id, before.keyframe_id);
  s.scan_at(s.t_scan(2));  // processing continues
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(2)});
}

TEST(PipelineComponent, UncoveredScanSkippedAtFlush) {  // Review Focus 4
  BoxWorldParams p;
  Config cfg = StaticCore::make_cfg();
  Estimator est(cfg);
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  const std::int64_t period = 5'000'000;
  std::int64_t t_imu = p.t0_ns;
  const auto imu_until = [&](std::int64_t t) {
    for (; t_imu <= t; t_imu += period) {
      ASSERT_TRUE(est.push_imu(test::box_imu(p, Timestamp{t_imu}, nullptr)).is_ok());
      est.run_once();  // keep the IMU queue (realtime.imu_queue_depth) from overflowing
    }
  };
  const std::int64_t t0 = p.t0_ns + 2'000'000'000;
  for (int k = 0; k < 2; ++k) {
    scan.t_start = Timestamp{t0 + k * 100'000'000};
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    imu_until(common::scan_end_time(scan).ns + period);  // covers the scan end
    est.run_once();
  }
  ASSERT_EQ(est.latest_state().state.t, Timestamp{t0 + 100'000'000});
  scan.t_start = Timestamp{t0 + 200'000'000};
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  imu_until(t0 + 250'000'000);  // IMU stops 50 ms into the last scan
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 2U);  // gated
  est.flush();
  const Estimator::Stats st = est.stats();
  EXPECT_EQ(st.n_scans, 3U);
  EXPECT_EQ(st.n_uncovered_scans, 1U);
  EXPECT_EQ(st.n_skipped_scans, 1U);
  EXPECT_EQ(est.latest_state().state.t, Timestamp{t0 + 100'000'000});
}

TEST(PipelineComponent, CoreConstructorValidatesBuffers) {
  Config cfg = StaticCore::make_cfg();
  cfg.imu.init_window_s = 30.0;  // 6000 samples at 200 Hz
  try {
    detail::EstimatorCore core(cfg);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("imu.init_window_s"), std::string::npos) << e.what();
  }
  Config cfg2 = StaticCore::make_cfg();
  cfg2.map.window_keyframes = static_cast<int>(cfg2.map.max_keyframes) + 1;
  try {
    detail::EstimatorCore core(cfg2);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("map.window_keyframes"), std::string::npos) << e.what();
  }
}

}  // namespace
}  // namespace mss::pipeline
