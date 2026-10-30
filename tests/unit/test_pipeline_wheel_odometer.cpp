// Unit tests of the pipeline's wheel odometer (backlog G9, H4 review I-1; gnss.md section 5.3):
// the trapezoid reading, the measured nominal period and the gap bound, at 50, 10 and 5 Hz with
// +-10 ms of jitter. Fixed seeds.
#include <cstdint>
#include <random>

#include <gtest/gtest.h>

#include "mss/common/timestamp.hpp"

#include "wheel_odometer.hpp"

namespace {

using mss::common::Timestamp;
using mss::pipeline::detail::WheelOdometer;

constexpr std::int64_t kMs = 1'000'000;  // ns

// Constant 0.8 m/s at each rate, intervals jittered by +-10 ms (uniform, 1 ms steps): the reading
// is the travel, no interval is a gap (3 nominal periods), and the nominal period is the mean
// interval. Then the driver is silent for 1 s at 0.8 m/s: one gap, bound 0.8 x (1 - nominal).
TEST(WheelOdometer, KnownAnswerAtEachRate) {
  for (const std::int64_t period_ms : {20, 100, 200}) {
    WheelOdometer odo(3.0);
    std::mt19937 rng(5U);
    std::uniform_int_distribution<std::int64_t> jitter(-10, 10);
    std::int64_t t_ms = 0;
    std::int64_t sum_ms = 0;
    constexpr int kSamples = 400;
    for (int i = 0; i < kSamples; ++i) {
      EXPECT_FALSE(odo.push(Timestamp{t_ms * kMs}, 0.8)) << period_ms << " " << i;
      if (i + 1 < kSamples) {
        const std::int64_t dt_ms = period_ms + jitter(rng);
        t_ms += dt_ms;
        sum_ms += dt_ms;
      }
    }
    const double nominal_s = 1e-3 * static_cast<double>(sum_ms) / (kSamples - 1);
    EXPECT_NEAR(odo.nominal_period_s(), nominal_s, 1e-12) << period_ms;
    EXPECT_NEAR(odo.reading_m(), 0.8 * 1e-3 * static_cast<double>(t_ms), 1e-9) << period_ms;
    EXPECT_EQ(odo.gap_bound_m(), 0.0) << period_ms;
    EXPECT_EQ(odo.n_gaps(), 0U) << period_ms;
    t_ms += 1000;
    EXPECT_TRUE(odo.push(Timestamp{t_ms * kMs}, 0.8)) << period_ms;
    EXPECT_NEAR(odo.gap_bound_m(), 0.8 * (1.0 - nominal_s), 1e-12) << period_ms;
    EXPECT_EQ(odo.n_gaps(), 1U) << period_ms;
    EXPECT_NEAR(odo.reading_m(), 0.8 * 1e-3 * static_cast<double>(t_ms), 1e-9) << period_ms;
    // The gap did not move the nominal period.
    EXPECT_NEAR(odo.nominal_period_s(), nominal_s, 1e-12) << period_ms;
  }
}

// The threshold is gap_periods nominal periods, and the bound takes the larger end speed: at a
// nominal 0.1 s, 0.29 s is regular and 0.31 s a gap (0.21 s of excess at max(0.2, 0.6) = 0.6).
TEST(WheelOdometer, ThresholdAndLargerEndSpeed) {
  WheelOdometer odo(3.0);
  for (int i = 0; i <= 10; ++i) {
    (void)odo.push(Timestamp{i * 100 * kMs}, 0.2);
  }
  EXPECT_NEAR(odo.nominal_period_s(), 0.1, 1e-15);
  EXPECT_FALSE(odo.push(Timestamp{(1000 + 290) * kMs}, 0.2));
  EXPECT_EQ(odo.gap_bound_m(), 0.0);
  WheelOdometer odo2(3.0);
  for (int i = 0; i <= 10; ++i) {
    (void)odo2.push(Timestamp{i * 100 * kMs}, 0.2);
  }
  EXPECT_TRUE(odo2.push(Timestamp{(1000 + 310) * kMs}, -0.6));
  EXPECT_NEAR(odo2.gap_bound_m(), 0.6 * (0.31 - 0.1), 1e-12);
  EXPECT_NEAR(odo2.reading_m(), 0.2 * 1.0 + 0.5 * (0.2 - 0.6) * 0.31, 1e-12);
}

// Before a regular interval there is no nominal period, so nothing is a gap; +inf (the struct
// default) never bounds.
TEST(WheelOdometer, NoGapBeforeANominalPeriodOrWithInfinity) {
  WheelOdometer odo(3.0);
  EXPECT_FALSE(odo.has_reading());
  EXPECT_FALSE(odo.push(Timestamp{0}, 1.0));
  EXPECT_FALSE(odo.push(Timestamp{5000 * kMs}, 1.0));  // the first interval starts the period
  EXPECT_EQ(odo.gap_bound_m(), 0.0);
  WheelOdometer never;
  (void)never.push(Timestamp{0}, 1.0);
  (void)never.push(Timestamp{20 * kMs}, 1.0);
  EXPECT_FALSE(never.push(Timestamp{5000 * kMs}, 1.0));
  EXPECT_EQ(never.gap_bound_m(), 0.0);
  EXPECT_TRUE(never.has_reading());
  EXPECT_EQ(never.last_t(), Timestamp{5000 * kMs});
}

}  // namespace
