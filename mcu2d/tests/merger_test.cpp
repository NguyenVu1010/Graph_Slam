#include "mss2d/merger.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace mss2d {
namespace {

struct Event {
  char kind;       // 'i', 'w', 's'
  std::int64_t t;  // sample time, or scan key
};

struct Recorder : Sink {
  std::vector<Event> ev;
  void on_imu(const ImuSample2d& s) override { ev.push_back({'i', s.t.ns}); }
  void on_wheel(const WheelSample& s) override { ev.push_back({'w', s.t.ns}); }
  void on_scan(const Scan2d& s) override { ev.push_back({'s', s.key().ns}); }
};

ImuSample2d imu(std::int64_t t) {
  ImuSample2d s;
  s.t = Timestamp{t};
  return s;
}
WheelSample wheel(std::int64_t t) {
  WheelSample s;
  s.t = Timestamp{t};
  return s;
}
// Scan with key = t_start + 100 (a 100 ns turn, one point at the end).
std::unique_ptr<Scan2d> scan(std::int64_t t_start, std::int32_t dur = 100) {
  auto s = std::make_unique<Scan2d>();
  s->t_start = Timestamp{t_start};
  s->count = 2;
  s->points[0].t_offset_ns = 0;
  s->points[1].t_offset_ns = dur;
  return s;
}

std::string str(const std::vector<Event>& v) {
  std::string out;
  for (const Event& e : v) {
    out += e.kind;
    out += std::to_string(e.t);
    out += ' ';
  }
  return out;
}

class MergerTest : public ::testing::Test {
 protected:
  MergerTest() : m(std::make_unique<Merger>(Config{})) {}
  std::unique_ptr<Merger> m;
  Recorder r;
};

TEST_F(MergerTest, InterleavedInputsAreDeliveredInTimeOrder) {
  // Wheel arrives before older IMU in push order; merge sorts by key.
  ASSERT_TRUE(m->push_wheel(wheel(15)).is_ok());
  ASSERT_TRUE(m->push_wheel(wheel(35)).is_ok());
  for (std::int64_t t : {10, 20, 30, 40, 250}) {
    ASSERT_TRUE(m->push_imu(imu(t)).is_ok());
  }
  m->run_once(r);
  EXPECT_EQ(str(r.ev), "i10 w15 i20 i30 w35 i40 i250 ") << "order mismatch";
}

TEST_F(MergerTest, TiesGoImuWheelScan) {
  ASSERT_TRUE(m->push_scan(*scan(0)).is_ok());  // key 100
  ASSERT_TRUE(m->push_wheel(wheel(100)).is_ok());
  ASSERT_TRUE(m->push_imu(imu(100)).is_ok());
  ASSERT_TRUE(m->push_imu(imu(200)).is_ok());
  m->run_once(r);
  EXPECT_EQ(str(r.ev), "i100 w100 s100 i200 ");
}

TEST_F(MergerTest, ScanWaitsForImuCoverageAndFlushIgnoresTheGate) {
  ASSERT_TRUE(m->push_imu(imu(50)).is_ok());
  ASSERT_TRUE(m->push_scan(*scan(0)).is_ok());  // key 100, IMU only up to 50
  ASSERT_TRUE(m->push_wheel(wheel(120)).is_ok());
  EXPECT_EQ(m->run_once(r), 1U);  // only i50; scan and the later wheel stay queued
  EXPECT_EQ(str(r.ev), "i50 ");
  // Early calls change nothing.
  EXPECT_EQ(m->run_once(r), 0U);
  // IMU at/after the key covers the scan.
  ASSERT_TRUE(m->push_imu(imu(100)).is_ok());
  m->run_once(r);
  EXPECT_EQ(str(r.ev), "i50 i100 s100 w120 ");

  // flush: a scan with no covering IMU is delivered anyway.
  ASSERT_TRUE(m->push_scan(*scan(1000)).is_ok());  // key 1100
  EXPECT_EQ(m->run_once(r), 0U);
  EXPECT_EQ(m->flush(r), 1U);
  EXPECT_EQ(r.ev.back().kind, 's');
  EXPECT_EQ(r.ev.back().t, 1100);
}

TEST_F(MergerTest, ImuAheadOfScanUpToFirstSampleAtOrAfterKey) {
  // IMU at 90 and 130 (none at 100): both sort around the scan key 100; the core needs the
  // first sample after the key before the scan, so i130 is delivered before s100.
  ASSERT_TRUE(m->push_imu(imu(90)).is_ok());
  ASSERT_TRUE(m->push_imu(imu(130)).is_ok());
  ASSERT_TRUE(m->push_imu(imu(160)).is_ok());
  ASSERT_TRUE(m->push_scan(*scan(0)).is_ok());
  m->run_once(r);
  EXPECT_EQ(str(r.ev), "i90 i130 s100 i160 ");
}

TEST_F(MergerTest, OutOfOrderMessagesAreRejectedAndCounted) {
  ASSERT_TRUE(m->push_imu(imu(10)).is_ok());
  EXPECT_FALSE(m->push_imu(imu(10)).is_ok());
  EXPECT_FALSE(m->push_imu(imu(5)).is_ok());
  ASSERT_TRUE(m->push_wheel(wheel(10)).is_ok());
  EXPECT_FALSE(m->push_wheel(wheel(9)).is_ok());
  ASSERT_TRUE(m->push_scan(*scan(0)).is_ok());
  EXPECT_FALSE(m->push_scan(*scan(-50)).is_ok());  // key 50 < 100
  const MergerStats st = m->stats();
  EXPECT_EQ(st.n_out_of_order_imu, 2U);
  EXPECT_EQ(st.n_out_of_order_wheel, 1U);
  EXPECT_EQ(st.n_out_of_order_scan, 1U);
}

TEST_F(MergerTest, FullRingsDropNewMessagesAndCount) {
  EXPECT_TRUE(m->push_scan(*scan(0)).is_ok());
  EXPECT_TRUE(m->push_scan(*scan(1000)).is_ok());
  EXPECT_FALSE(m->push_scan(*scan(2000)).is_ok());  // 2 slots
  for (int i = 0; i < 64; ++i) {
    EXPECT_TRUE(m->push_wheel(wheel(i + 1)).is_ok());
  }
  EXPECT_FALSE(m->push_wheel(wheel(1000)).is_ok());
  const MergerStats st = m->stats();
  EXPECT_EQ(st.n_dropped_scan, 1U);
  EXPECT_EQ(st.n_dropped_wheel, 1U);
  EXPECT_EQ(st.n_dropped_imu, 0U);
}

TEST_F(MergerTest, ScanWithTooManyPointsIsRejected) {
  Config c;
  c.max_scan_points = 100;
  Merger small(c);
  auto s = scan(0);
  s->count = 101;
  EXPECT_FALSE(small.push_scan(*s).is_ok());
}

TEST_F(MergerTest, DeliveredCountersAdvance) {
  ASSERT_TRUE(m->push_imu(imu(1)).is_ok());
  ASSERT_TRUE(m->push_wheel(wheel(2)).is_ok());
  ASSERT_TRUE(m->push_scan(*scan(0)).is_ok());
  ASSERT_TRUE(m->push_imu(imu(200)).is_ok());
  m->run_once(r);
  const MergerStats st = m->stats();
  EXPECT_EQ(st.n_imu, 2U);
  EXPECT_EQ(st.n_wheel, 1U);
  EXPECT_EQ(st.n_scans, 1U);
}

}  // namespace
}  // namespace mss2d
