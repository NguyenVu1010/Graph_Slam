#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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
  ScanFrame scan;
  fill_scan(scan, 1000);
  const Timestamp t0 = scan.t_start;
  AllocCounter::reset();
  for (int k = 0; k < 5; ++k) {
    const Timestamp t{t0.ns + k * 100000000LL};
    scan.t_start = t;
    const auto st = est.push_scan(scan);
    EXPECT_TRUE(st.is_ok()) << st.reason();
    scan.t_start = Timestamp{-1};  // the estimator holds its own copy
    EXPECT_TRUE(est.push_imu(imu_at(t.ns + 1)).is_ok());
    est.run_once();
    EXPECT_EQ(est.latest_state().state.t, t);
  }
  EXPECT_EQ(AllocCounter::count(), 0U);
  EXPECT_EQ(est.stats().n_scans, 5U);
}

TEST(Estimator, ScanQueueFullDropsOldest) {
  Estimator est(make_config(2));
  ScanFrame scan;
  fill_scan(scan, 0);
  for (int k = 0; k < 3; ++k) {
    scan.t_start = Timestamp{1000 + k * 100000000LL};
    const auto st = est.push_scan(scan);
    if (k < 2) {
      EXPECT_TRUE(st.is_ok()) << st.reason();
    } else {
      ASSERT_FALSE(st.is_ok());
      EXPECT_NE(st.reason().find("dropped"), std::string::npos) << st.reason();
    }
  }
  EXPECT_EQ(est.stats().n_dropped_scans, 1U);
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 2U);
  // The oldest (k = 0) was dropped, so the last processed scan is k = 2.
  EXPECT_EQ(est.latest_state().state.t, (Timestamp{1000 + 2 * 100000000LL}));
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
  est.run_once();
  Estimator::Stats s = est.stats();
  EXPECT_EQ(s.n_imu, 20U);
  EXPECT_EQ(s.n_gnss, 3U);
  EXPECT_EQ(s.n_odom, 4U);
  EXPECT_EQ(s.n_scans, 1U);
  EXPECT_GT(s.push_scan_us_p50, 0.0);
  EXPECT_GE(s.push_scan_us_p95, s.push_scan_us_p50);
  est.run_once();  // nothing left: counters unchanged
  s = est.stats();
  EXPECT_EQ(s.n_imu, 20U);
  EXPECT_EQ(s.n_scans, 1U);
}

TEST(Estimator, LatestStateTimestampFollowsScans) {
  Estimator est(make_config());
  EXPECT_EQ(est.latest_state().state.t, Timestamp{});
  ScanFrame scan;
  fill_scan(scan, 0);
  for (std::int64_t t : {1000000LL, 2000000LL, 3000000LL}) {
    scan.t_start = Timestamp{t};
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    est.run_once();
    const common::Estimate e = est.latest_state();
    EXPECT_EQ(e.state.t, Timestamp{t});
    EXPECT_TRUE(e.state.T_w_b.R.isIdentity(0.0));
    EXPECT_TRUE(e.state.T_w_b.p.isZero(0.0));
    EXPECT_TRUE(e.state.v_w_b.isZero(0.0));
  }
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
  // samples inside its span is still processed, and so are all IMU samples.
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
  EXPECT_EQ(est.latest_state().state.t, Timestamp{1000});
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
    est.run_once();
    EXPECT_EQ(est.stats().n_scans, 2U * static_cast<std::uint64_t>(round));
    EXPECT_EQ(est.latest_state().state.t, (Timestamp{t - 100000000LL}));
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
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 1U);
}

}  // namespace
}  // namespace mss::pipeline
