#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/bounded_queue.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"
#include "mss/pipeline/merge_order.hpp"

#include "alloc_counter.hpp"

namespace mss::pipeline {
namespace {

using common::GnssMeasurement;
using common::ImuSample;
using common::ScanFrame;
using common::Timestamp;
using common::WheelSample;

constexpr std::size_t kPoints = 1000;  // points per test scan

const std::filesystem::path kDefault = std::filesystem::path{MSS_REPO_DIR} / "config/default.yaml";

// config/default.yaml with a small scan capacity so the tests stay light.
Config make_config(int scan_depth = 2) {
  const auto loaded = load_config(kDefault);
  EXPECT_TRUE(loaded.is_ok()) << loaded.reason();
  Config cfg = loaded.value();
  cfg.realtime.scan_queue_depth = scan_depth;
  cfg.realtime.max_points_per_scan = 2 * kPoints;
  return cfg;
}

// Scan with kPoints points; per-point offsets 0, 100, ..., so its key (end time) is
// t_start + (kPoints - 1) * 100 ns.
void fill_scan(ScanFrame& scan, std::int64_t t_start) {
  scan.reserve(kPoints);
  scan.clear();
  scan.t_start = Timestamp{t_start};
  for (std::size_t i = 0; i < kPoints; ++i) {
    const float f = static_cast<float>(i);
    ASSERT_TRUE(scan.push(f, -f, 1.0F, 5.0F, static_cast<std::uint32_t>(i * 100U), 0));
  }
}

constexpr std::int64_t kScanSpanNs = static_cast<std::int64_t>(kPoints - 1) * 100;

ImuSample imu_at(std::int64_t t) {
  ImuSample s;
  s.t = Timestamp{t};
  return s;
}

// Still IMU sample: specific force straight up (gravity 9.80665 m/s^2), no rotation.
ImuSample static_imu_at(std::int64_t t) {
  ImuSample s = imu_at(t);
  s.acc_b = common::Vec3(0.0, 0.0, 9.80665);
  return s;
}

constexpr std::int64_t kImuPeriodNs = 5000000;    // 200 Hz
constexpr std::int64_t kStillEndNs = 2000000000;  // 2 s of still IMU = imu.init_window_s

// Still IMU at 200 Hz over [t_from, t_to], run_once() every 100 samples so the IMU queue
// (realtime.imu_queue_depth) never overflows. Enough for the static initialisation when
// it spans imu.init_window_s before the first scan.
void feed_static_imu(Estimator& est, std::int64_t t_from, std::int64_t t_to) {
  int n = 0;
  for (std::int64_t t = t_from; t <= t_to; t += kImuPeriodNs) {
    ASSERT_TRUE(est.push_imu(static_imu_at(t)).is_ok());
    if (++n % 100 == 0) {
      est.run_once();
    }
  }
  est.run_once();
}

TEST(BoundedQueue, PushPopOverwriteOldest) {
  BoundedQueue<int, 3> q;
  EXPECT_EQ(q.capacity(), 3U);
  EXPECT_TRUE(q.empty());
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_TRUE(q.push(3));
  EXPECT_FALSE(q.push(4));  // full: 1 is overwritten
  EXPECT_EQ(q.size(), 3U);
  EXPECT_EQ(q.front(), 2);
  int v = 0;
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 2);
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 3);
  ASSERT_TRUE(q.pop(v));
  EXPECT_EQ(v, 4);
  EXPECT_FALSE(q.pop(v));
}

TEST(Estimator, PushScanCopiesWithoutAlloc) {
  Estimator est(make_config());
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, 0, kStillEndNs));
  ScanFrame scan;
  fill_scan(scan, kStillEndNs + 1000);
  const Timestamp t0 = scan.t_start;
  // One scan pushed, covered and processed. Its points lie on a line, so scan matching
  // fails and the estimate follows the IMU prediction (no keyframe after keyframe 0).
  const auto step = [&](int k, bool& ok) {
    const Timestamp t{t0.ns + k * 100000000LL};
    scan.t_start = t;
    ok = ok && est.push_scan(scan).is_ok();
    scan.t_start = Timestamp{-1};  // the estimator holds its own copy
    // Covers the scan (key = t + kScanSpanNs), so run_once() processes it.
    ok = ok && est.push_imu(static_imu_at(t.ns + kScanSpanNs)).is_ok();
    est.run_once();
    ok = ok && est.latest_state().state.t == t;
  };
  bool all_ok = true;
  step(0, all_ok);  // warm-up: IMU initialisation and keyframe 0 (the backend allocates)
  step(1, all_ok);  // warm-up: the first scan-match failure is logged once
  AllocCounter::reset();
  std::size_t allocs = 0;
  {
    const EigenMallocGuard no_eigen_malloc;
    for (int k = 2; k < 7; ++k) {
      step(k, all_ok);
    }
    // This thread only: a TBB-enabled GTSAM starts worker threads after keyframe 0's
    // optimize() that allocate asynchronously on their own threads.
    allocs = AllocCounter::count_this_thread();  // read before gtest macros allocate
  }
  EXPECT_TRUE(all_ok);
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(est.stats().n_scans, 7U);
  EXPECT_EQ(est.stats().n_keyframes, 1U);
}

TEST(Estimator, ScanQueueFullDropsOldest) {
  Estimator est(make_config(2));
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, 0, kStillEndNs));
  ScanFrame scan;
  fill_scan(scan, 0);
  const std::int64_t t0 = kStillEndNs + 1000;
  for (int k = 0; k < 3; ++k) {
    scan.t_start = Timestamp{t0 + k * 100000000LL};
    const auto st = est.push_scan(scan);
    if (k < 2) {
      EXPECT_TRUE(st.is_ok()) << st.reason();
    } else {
      ASSERT_FALSE(st.is_ok());
      EXPECT_NE(st.reason().find("dropped"), std::string::npos) << st.reason();
    }
  }
  EXPECT_EQ(est.stats().n_dropped_scans, 1U);
  // One still IMU sample covering the last scan's end.
  ASSERT_TRUE(est.push_imu(static_imu_at(t0 + 2 * 100000000LL + kScanSpanNs)).is_ok());
  est.flush();
  EXPECT_EQ(est.stats().n_scans, 2U);
  EXPECT_EQ(est.stats().n_uncovered_scans, 0U);
  // The oldest (k = 0) was dropped, so the last processed scan is k = 2.
  EXPECT_EQ(est.latest_state().state.t, (Timestamp{t0 + 2 * 100000000LL}));
}

TEST(Estimator, OversizedScanRejected) {
  Config cfg = make_config();
  cfg.realtime.max_points_per_scan = kPoints - 1;
  Estimator est(cfg);
  ScanFrame scan;
  fill_scan(scan, 1000);
  const auto st = est.push_scan(scan);
  ASSERT_FALSE(st.is_ok());
  EXPECT_NE(st.reason().find("max_points_per_scan"), std::string::npos) << st.reason();
  EXPECT_EQ(est.stats().n_dropped_scans, 1U);
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 0U);
}

TEST(Estimator, ImuNonIncreasingRejected) {
  Estimator est(make_config());
  EXPECT_TRUE(est.push_imu(imu_at(10)).is_ok());
  const auto st = est.push_imu(imu_at(10));
  ASSERT_FALSE(st.is_ok());
  EXPECT_EQ(st.reason(), "imu timestamp not increasing");
  EXPECT_FALSE(est.push_imu(imu_at(9)).is_ok());
  EXPECT_TRUE(est.push_imu(imu_at(11)).is_ok());
  EXPECT_EQ(est.stats().n_rejected_imu, 2U);
  est.run_once();
  EXPECT_EQ(est.stats().n_imu, 2U);
}

TEST(Estimator, RunOnceDrainsQueues) {
  Estimator est(make_config());
  ScanFrame scan;
  fill_scan(scan, 5000);
  for (int k = 0; k < 20; ++k) {
    EXPECT_TRUE(est.push_imu(imu_at(100 + k * 10)).is_ok());
  }
  for (int k = 0; k < 3; ++k) {
    GnssMeasurement m;
    m.t = Timestamp{150 + k};
    EXPECT_TRUE(est.push_gnss(m).is_ok());
  }
  for (int k = 0; k < 4; ++k) {
    WheelSample w;
    w.t = Timestamp{120 + k};
    EXPECT_TRUE(est.push_odom(w).is_ok());
  }
  EXPECT_TRUE(est.push_scan(scan).is_ok());
  EXPECT_TRUE(est.push_imu(imu_at(5000 + kScanSpanNs)).is_ok());  // covers the scan
  est.run_once();
  Estimator::Stats s = est.stats();
  EXPECT_EQ(s.n_imu, 21U);
  EXPECT_EQ(s.n_gnss, 3U);
  EXPECT_EQ(s.n_odom, 4U);
  EXPECT_EQ(s.n_scans, 1U);
  EXPECT_GT(s.push_scan_us_p50, 0.0);
  EXPECT_GE(s.push_scan_us_p95, s.push_scan_us_p50);
  est.run_once();  // nothing left: counters unchanged
  s = est.stats();
  EXPECT_EQ(s.n_imu, 21U);
  EXPECT_EQ(s.n_scans, 1U);
}

TEST(Estimator, LatestStateTimestampFollowsScans) {
  Estimator est(make_config());
  EXPECT_EQ(est.latest_state().state.t, Timestamp{});
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, 0, kStillEndNs));
  ScanFrame scan;
  fill_scan(scan, 0);
  for (std::int64_t dt : {1000000LL, 2000000LL, 3000000LL}) {
    const std::int64_t t = kStillEndNs + dt;
    scan.t_start = Timestamp{t};
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    ASSERT_TRUE(est.push_imu(static_imu_at(t + kScanSpanNs)).is_ok());
    est.run_once();
    const common::Estimate e = est.latest_state();
    EXPECT_EQ(e.state.t, Timestamp{t});
    // Still IMU, level: the estimate stays at the initial pose.
    EXPECT_TRUE(e.state.T_w_b.R.isApprox(common::Mat3::Identity(), 1e-6));
    EXPECT_LT(e.state.T_w_b.p.norm(), 1e-6);
    EXPECT_LT(e.state.v_w_b.norm(), 1e-6);
  }
}

// A non-finite IMU sample is dropped by the core, so it cannot cover a scan: the scan
// waits for the next finite sample and is then processed normally.
TEST(Estimator, NonFiniteImuDoesNotCoverScan) {
  Estimator est(make_config());
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, 0, kStillEndNs));
  ScanFrame scan;
  fill_scan(scan, kStillEndNs + 1000);
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  ImuSample bad = static_imu_at(scan.t_start.ns + kScanSpanNs);
  bad.gyro_b.z() = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(est.push_imu(bad).is_ok());
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 0U);  // still waiting for a finite covering sample
  ASSERT_TRUE(est.push_imu(static_imu_at(scan.t_start.ns + kScanSpanNs + kImuPeriodNs)).is_ok());
  est.run_once();
  const Estimator::Stats st = est.stats();
  EXPECT_EQ(st.n_scans, 1U);
  EXPECT_EQ(st.n_skipped_scans, 0U);
  EXPECT_EQ(st.n_nonfinite_imu, 1U);
  EXPECT_EQ(est.latest_state().state.t, scan.t_start);
}

TEST(Estimator, LatestStateEmptyUntilImuInitialised) {
  Estimator est(make_config());
  // 1 s of still IMU only: shorter than imu.init_window_s (2 s).
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, 0, kStillEndNs / 2));
  ScanFrame scan;
  fill_scan(scan, 0);
  std::int64_t t = kStillEndNs / 2 + 1000;
  for (int k = 0; k < 3; ++k, t += 100000000LL) {
    scan.t_start = Timestamp{t};
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, t + 1000, t + 100000000LL - 1000));
    EXPECT_EQ(est.latest_state().state.t, Timestamp{}) << k;
  }
  EXPECT_EQ(est.stats().n_scans, 3U);
  // Once 2 s of still IMU precede a scan, the estimate appears at that scan.
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, t + 1000, t + kStillEndNs));
  t += kStillEndNs + 100000000LL;
  scan.t_start = Timestamp{t};
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  ASSERT_NO_FATAL_FAILURE(feed_static_imu(est, t + 1000, t + 100000000LL));
  EXPECT_EQ(est.latest_state().state.t, Timestamp{t});
}

// k-way merge step: smallest key first; ties IMU, wheel, GNSS, scan.
TEST(Estimator, RunOnceOrdersByKeyWithTieOrder) {
  const std::int64_t key = 1000 + kScanSpanNs;  // end time of the scan below
  std::array<QueueHead, kNumSources> heads{};
  heads[static_cast<std::size_t>(Source::kScan)] = {true, Timestamp{key}};
  heads[static_cast<std::size_t>(Source::kImu)] = {true, Timestamp{1500}};
  EXPECT_EQ(select_next(heads), Source::kImu);  // earlier IMU before the later scan end
  heads[static_cast<std::size_t>(Source::kImu)] = {true, Timestamp{key}};
  EXPECT_EQ(select_next(heads), Source::kImu);  // equal keys: IMU before scan
  heads[static_cast<std::size_t>(Source::kImu)] = {true, Timestamp{key + 1}};
  EXPECT_EQ(select_next(heads), Source::kScan);
  heads[static_cast<std::size_t>(Source::kGnss)] = {true, Timestamp{key}};
  EXPECT_EQ(select_next(heads), Source::kGnss);  // GNSS before scan on a tie
  heads[static_cast<std::size_t>(Source::kWheel)] = {true, Timestamp{key}};
  EXPECT_EQ(select_next(heads), Source::kWheel);  // wheel before GNSS on a tie
  EXPECT_EQ(select_next(std::array<QueueHead, kNumSources>{}), Source::kNone);

  // Through the estimator: the scan is keyed by its END time, so a scan pushed before IMU
  // samples inside its span is still processed, and so are all IMU samples. The IMU at
  // exactly `key` sorts before the scan and is the sample that covers it.
  Estimator est(make_config());
  ScanFrame scan;
  fill_scan(scan, 1000);
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  ASSERT_TRUE(est.push_imu(imu_at(1500)).is_ok());
  ASSERT_TRUE(est.push_imu(imu_at(key)).is_ok());
  ASSERT_TRUE(est.push_imu(imu_at(key + 1)).is_ok());
  est.run_once();
  EXPECT_EQ(est.stats().n_imu, 3U);
  EXPECT_EQ(est.stats().n_scans, 1U);
  // No still IMU window before the scan: no estimate (the IMU is not initialised).
  EXPECT_EQ(est.latest_state().state.t, Timestamp{});
}

// 100 pushes into a depth-2 queue cycle every pool slot many times without leaking one.
TEST(Estimator, PoolCyclesUnderSustainedOverflow) {
  Estimator est(make_config(2));
  ScanFrame scan;
  fill_scan(scan, 0);
  std::int64_t t = 1000;
  for (int round = 1; round <= 2; ++round) {
    for (int k = 0; k < 100; ++k) {
      scan.t_start = Timestamp{t};
      t += 100000000LL;
      (void)est.push_scan(scan);
    }
    EXPECT_EQ(est.stats().n_dropped_scans, 98U * static_cast<std::uint64_t>(round));
    est.flush();
    EXPECT_EQ(est.stats().n_scans, 2U * static_cast<std::uint64_t>(round));
    // No IMU at all: the scans are consumed without an estimate.
    EXPECT_EQ(est.latest_state().state.t, Timestamp{});
    EXPECT_EQ(est.stats().n_skipped_scans, 2U * static_cast<std::uint64_t>(round));
  }
}

TEST(Estimator, QueueOverflowCountsDrops) {
  Config cfg = make_config();
  cfg.realtime.imu_queue_depth = 4;
  cfg.realtime.aux_queue_depth = 2;
  Estimator est(cfg);
  for (int k = 0; k < 6; ++k) {
    const auto st = est.push_imu(imu_at(100 + k));
    EXPECT_EQ(st.is_ok(), k < 4) << k;
    if (k >= 4) {
      EXPECT_EQ(st.reason(), "imu queue full, dropped oldest");
    }
  }
  for (int k = 0; k < 3; ++k) {
    GnssMeasurement m;
    m.t = Timestamp{200 + k};
    EXPECT_EQ(est.push_gnss(m).is_ok(), k < 2) << k;
    WheelSample w;
    w.t = Timestamp{200 + k};
    EXPECT_EQ(est.push_odom(w).is_ok(), k < 2) << k;
  }
  Estimator::Stats s = est.stats();
  EXPECT_EQ(s.n_dropped_imu, 2U);
  EXPECT_EQ(s.n_dropped_aux, 2U);
  EXPECT_EQ(s.n_rejected_imu, 0U);
  est.run_once();
  s = est.stats();
  EXPECT_EQ(s.n_imu, 4U);
  EXPECT_EQ(s.n_gnss, 2U);
  EXPECT_EQ(s.n_odom, 2U);
}

TEST(Estimator, ConfigDepthAboveCompileTimeMaxThrows) {
  Config cfg = make_config();
  cfg.realtime.imu_queue_depth = static_cast<int>(kMaxImuQueue) + 1;
  try {
    Estimator est(cfg);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("realtime.imu_queue_depth"), std::string::npos)
        << e.what();
  }
  Config cfg2 = make_config(static_cast<int>(kMaxScanQueue) + 1);
  EXPECT_THROW(Estimator{cfg2}, std::invalid_argument);
  Config cfg3 = make_config();
  cfg3.realtime.aux_queue_depth = static_cast<int>(kMaxAuxQueue) + 1;
  EXPECT_THROW(Estimator{cfg3}, std::invalid_argument);
}

TEST(Estimator, StartStopAreNoOps) {
  Estimator est(make_config());
  est.start();
  est.stop();
  ScanFrame scan;
  fill_scan(scan, 1000);
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  est.flush();
  EXPECT_EQ(est.stats().n_scans, 1U);
}

// One-point scan with key (end time) 100 ns.
void fill_scan_key_100(ScanFrame& scan) {
  scan.reserve(1);
  scan.clear();
  scan.t_start = Timestamp{40};
  ASSERT_TRUE(scan.push(1.0F, 0.0F, 0.0F, 1.0F, 60U, 0));
}

TEST(Estimator, ScanWaitsForImuCoverage) {
  ScanFrame scan;
  ASSERT_NO_FATAL_FAILURE(fill_scan_key_100(scan));
  ASSERT_EQ(common::scan_end_time(scan), Timestamp{100});

  {  // IMU at 50 and at the key 100: covered, processed by run_once().
    Estimator est(make_config());
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    ASSERT_TRUE(est.push_imu(imu_at(50)).is_ok());
    ASSERT_TRUE(est.push_imu(imu_at(100)).is_ok());
    est.run_once();
    EXPECT_EQ(est.stats().n_imu, 2U);
    EXPECT_EQ(est.stats().n_scans, 1U);
    EXPECT_EQ(est.latest_state().state.t, Timestamp{});  // IMU not initialised
  }
  {  // IMU only at 50: run_once() stops at the scan; later messages stay queued too.
    Estimator est(make_config());
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    ASSERT_TRUE(est.push_imu(imu_at(50)).is_ok());
    GnssMeasurement m;
    m.t = Timestamp{150};  // after the scan key
    ASSERT_TRUE(est.push_gnss(m).is_ok());
    est.run_once();
    EXPECT_EQ(est.stats().n_imu, 1U);
    EXPECT_EQ(est.stats().n_scans, 0U);
    EXPECT_EQ(est.stats().n_gnss, 0U);
    EXPECT_EQ(est.latest_state().state.t, Timestamp{});
    est.run_once();  // still uncovered
    EXPECT_EQ(est.stats().n_scans, 0U);
    est.flush();  // end of data: processed regardless of coverage
    EXPECT_EQ(est.stats().n_scans, 1U);
    EXPECT_EQ(est.stats().n_gnss, 1U);
    EXPECT_EQ(est.stats().n_uncovered_scans, 1U);
    EXPECT_EQ(est.latest_state().state.t, Timestamp{});
  }
}

}  // namespace
}  // namespace mss::pipeline
