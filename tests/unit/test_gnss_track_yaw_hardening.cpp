// Unit tests of the design-2b hardening (docs/derivations/gnss.md section 5.3), one section per
// backlog item. The aligner is driven directly (front-end pose = truth unless stated). Fixed
// seeds, 100 runs per point; the numbers in the comments are those of these seeds.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"

#include "wheel_odometer.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Timestamp;
using mss::common::Vec3;
using mss::gnss::GnssConfig;
using mss::gnss::GnssTrackYawAligner;
using mss::gnss::TrackYawReadings;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr int kRuns = 100;
const Vec3 kPbg(-0.20, 0.10, 0.60);  // m, antenna lever arm in b (sim configs)
const double kNaN = std::numeric_limits<double>::quiet_NaN();

double wrap_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

const Pose kTew = yaw_pose(-75.0 * kDeg, Vec3(-40.0, 25.0, 3.0));  // true T_e_w

// The design-2b keys of config/default.yaml before the hardening (study section 16.13).
GnssConfig cfg_2b() {
  GnssConfig c;
  c.enabled = true;
  c.align_yaw_from_track = true;
  c.align_yaw_wheel_turns = true;
  c.align_yaw_max_sigma_rad = 6.0 * kDeg;
  c.align_yaw_min_track_wheel_m = 3.0;
  c.align_yaw_wheel_max_gap_fixes = 4;
  c.align_yaw_merge_max_chunks = 10;
  c.align_yaw_frontend_travel = true;
  return c;
}

// With the wheel-sample gap key of config/default.yaml (G9).
GnssConfig cfg_gap() {
  GnssConfig c = cfg_2b();
  c.align_yaw_wheel_gap_periods = 3.0;
  return c;
}

// With every hardening key of config/default.yaml.
GnssConfig cfg_hardened() {
  GnssConfig c = cfg_gap();
  c.align_yaw_drift_var_m2_per_s = 0.0017;
  return c;
}

// kAisles: 4 s straight at speed_mps, then a 90 deg turn in 1.6 s at 0.3 m/s, alternately left
// and right (the review probe's warehouse aisles). kStuck: the body never moves while its wheels
// turn at speed_mps (a robot against a kerb).
enum class Shape { kStraight, kParked, kSlowS, kAisles, kStuck };

struct Scene {
  Shape shape = Shape::kStraight;
  double speed_mps = 0.5;
  double yaw_rate_radps = 0.0;  // kSlowS: 5 s left, 5 s right, 10 s straight, repeated
  double rest_s = 2.0;
  double tmax_s = 300.0;
  double white_m = 0.5;
  double declared_m = 0.5;
  double drift_m = 0.0;  // m per fix, GNSS random walk per axis
  bool wheel = true;
  bool frontend = false;         // the LiDAR observes the travel axis (count 0)
  bool frozen_frontend = false;  // the front-end pose never moves (passes the gate)
  // Stop-and-go with a wheel driver that goes silent while stopped: the pipeline's trapezoid
  // odometer (pipeline::detail::WheelOdometer) bridges the stop; `gap_bound` passes its bound
  // (G9). `pipe_odometer` reads that odometer also without stops.
  double stall_every_s = 0.0;
  double stall_s = 0.0;
  bool gap_bound = true;
  bool pipe_odometer = false;
  int wheel_period_ms = 20;  // wheel sample period
  int wheel_jitter_ms = 0;   // uniform jitter of each sample interval, +-
  bool time = true;          // pass fix times (the drift term)
};

struct RunResult {
  bool aligned = false;
  double err_rad = 0.0;
  double sigma_rad = 0.0;
  double driven_m = 0.0;
  int fe_disagree = 0;
};

// 1 kHz kinematics of the body, fixes at 5 Hz.
RunResult run_scene(const Scene& sc, const GnssConfig& c, std::uint32_t seed) {
  std::mt19937 rng(31000U + seed);  // the review probe's seeds (rr4)
  std::normal_distribution<double> n01(0.0, 1.0);
  GnssTrackYawAligner a(c);
  double x = 0.0;
  double y = 0.0;
  double psi = 0.91 * seed;
  double odo = 0.0;
  double turn = 0.0;
  double driven = 0.0;
  Vec3 walk = Vec3::Zero();
  mss::pipeline::detail::WheelOdometer odo_pipe(c.align_yaw_wheel_gap_periods);
  std::mt19937 rng_jitter(77U + seed);
  std::uniform_int_distribution<int> jitter_ms(-sc.wheel_jitter_ms, sc.wheel_jitter_ms);
  long next_sample_ms = 0;
  const bool pipe = sc.pipe_odometer || sc.stall_every_s > 0.0;
  bool silent = false;
  constexpr double kDt = 1e-3;
  const auto n_steps = static_cast<long>(sc.tmax_s / kDt);
  const Pose frozen = yaw_pose(psi, Vec3::Zero());
  for (long k = 0; k < n_steps; ++k) {
    const double t = static_cast<double>(k) * kDt;
    double v = 0.0;
    double w = 0.0;
    const bool stalled = sc.stall_every_s > 0.0 && t >= sc.rest_s &&
                         std::fmod(t - sc.rest_s, sc.stall_every_s) > sc.stall_every_s - sc.stall_s;
    double wheel_v = 0.0;
    if (t >= sc.rest_s && sc.shape != Shape::kParked) {
      v = stalled ? 0.0 : sc.speed_mps;
      if (sc.shape == Shape::kSlowS) {
        const double tt = std::fmod(t - sc.rest_s, 20.0);
        w = tt < 5.0 ? sc.yaw_rate_radps : (tt < 10.0 ? -sc.yaw_rate_radps : 0.0);
      }
      if (sc.shape == Shape::kAisles) {
        const double tm = t - sc.rest_s;
        const auto leg = static_cast<long>(tm / 5.6);
        if (std::fmod(tm, 5.6) >= 4.0) {
          v = 0.3;
          w = (leg % 2 != 0 ? -1.0 : 1.0) * (0.5 * kPi) / 1.6;
        }
      }
      wheel_v = v;
      if (sc.shape == Shape::kStuck) {
        v = 0.0;
      }
    }
    if (pipe && k == next_sample_ms) {  // wheel samples, none while stalled
      if (!stalled) {
        (void)odo_pipe.push(Timestamp{static_cast<std::int64_t>(k) * 1'000'000}, wheel_v);
        silent = false;
      } else {
        silent = true;
      }
      next_sample_ms += sc.wheel_period_ms + jitter_ms(rng_jitter);
    }
    if (k % 200 == 0) {
      for (int i = 0; i < 2; ++i) {
        walk(i) += sc.drift_m * n01(rng);
      }
      const Pose b = yaw_pose(psi, Vec3(x, y, 0.0));
      Vec3 p = kTew * (b * kPbg) + walk;
      p.x() += sc.white_m * n01(rng);
      p.y() += sc.white_m * n01(rng);
      TrackYawReadings in;
      if (sc.wheel) {
        in.wheel_odometer_m = pipe ? (silent ? kNaN : odo_pipe.reading_m()) : odo;
      }
      in.heading_variation_rad = turn;
      in.travel_unobserved_scans = sc.frontend ? 0.0 : kNaN;
      if (pipe && sc.gap_bound) {
        in.wheel_gap_bound_m = odo_pipe.gap_bound_m();
      }
      if (sc.time) {
        in.t = Timestamp{static_cast<std::int64_t>(k) * 1'000'000};
        in.has_time = true;
      }
      Pose fe = b;
      if (sc.frozen_frontend) {
        fe.p = frozen.p;
      }
      const auto r = a.update(p, Mat3::Identity() * sc.declared_m * sc.declared_m, fe, kPbg, in);
      if (r.is_ok()) {
        RunResult out;
        out.aligned = true;
        out.err_rad = wrap_pi(std::atan2(r.value().R(1, 0), r.value().R(0, 0)) + 75.0 * kDeg);
        out.sigma_rad = a.yaw_sigma_rad();
        out.driven_m = driven;
        out.fe_disagree = a.n_frontend_wheel_disagreements();
        return out;
      }
    }
    x += v * std::cos(psi + 0.5 * w * kDt) * kDt;
    y += v * std::sin(psi + 0.5 * w * kDt) * kDt;
    driven += std::abs(v) * kDt;
    psi += w * kDt;
    turn += std::abs(w) * kDt;
    odo += wheel_v * kDt;
  }
  RunResult out;
  out.fe_disagree = a.n_frontend_wheel_disagreements();
  return out;
}

struct Stats {
  int aligned = 0;
  int beyond_3s = 0;
  int within_10m = 0;
  int fe_disagree_runs = 0;
  double nees = 0.0;
  double max_err_deg = 0.0;
  double mean_err_deg = 0.0;
  std::vector<double> driven;
  [[nodiscard]] double median_driven() const {
    if (driven.empty()) {
      return kNaN;
    }
    std::vector<double> d = driven;
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
  }
};

Stats sweep(const Scene& sc, const GnssConfig& c, int runs = kRuns) {
  Stats s;
  for (int seed = 1; seed <= runs; ++seed) {
    const RunResult r = run_scene(sc, c, static_cast<std::uint32_t>(seed));
    s.fe_disagree_runs += r.fe_disagree > 0 ? 1 : 0;
    if (!r.aligned) {
      continue;
    }
    ++s.aligned;
    const double z = r.err_rad / r.sigma_rad;
    s.nees += z * z;
    s.beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
    s.max_err_deg = std::max(s.max_err_deg, std::abs(r.err_rad) / kDeg);
    s.mean_err_deg += r.err_rad / kDeg;
    s.driven.push_back(r.driven_m);
    s.within_10m += r.driven_m <= 10.0 ? 1 : 0;
  }
  if (s.aligned > 0) {
    s.nees /= s.aligned;
    s.mean_err_deg /= s.aligned;
  }
  return s;
}

// Feeds a noise-free straight track along x, the body at x_of(k) at fix k, with the readings of
// `fix_readings(k)` (fixes declared at 0.5 m), and returns the aligner.
template <typename X, typename F>
GnssTrackYawAligner straight(const GnssConfig& c, int n_fixes, X&& x_of, F&& fix_readings) {
  GnssTrackYawAligner a(c);
  for (int k = 0; k < n_fixes; ++k) {
    const Pose b = yaw_pose(0.0, Vec3(x_of(k), 0.0, 0.0));
    const TrackYawReadings in = fix_readings(k);
    (void)a.update(kTew * (b * kPbg), Mat3::Identity() * 0.25, b, kPbg, in);
  }
  return a;
}

// 0.25 m/s for chunks 0 and 1 (moving segments), then 0.04 m/s (0.04 m per chunk half, so a
// segment between adjacent chunks is stationary and extended while the front end observes).
double fast_then_slow(int k) {
  return k <= 20 ? 0.05 * k : 1.0 + 0.008 * (k - 20);
}

// ---------------------------------------------------------------------------------------------
// G9: gap bridging with the pipeline's speed-integrating odometer.

// Review F1: stop-and-go (0.7 s stop every 4 s at 0.5 m/s) with a wheel driver silent while
// stopped: the trapezoid across each stop over-reads by 0.35 m (21 %), which tripped the
// wheel/GNSS mismatch, and the 10 m restart kept resetting: 85 of 100 aligned within 300 s (max
// 95 m driven) with the 2b keys (87 with these seeds), against 99 before bridging. With the gap
// bound in the tolerance: 100 of 100, 2 beyond 3 sigma (NEES 1.2, as before bridging).
TEST(GnssTrackYawHardening, SilentWheelDriverStopsAreBounded) {
  Scene sc;
  sc.stall_every_s = 4.0;
  sc.stall_s = 0.7;
  const Stats with = sweep(sc, cfg_gap());
  sc.gap_bound = false;
  const Stats without = sweep(sc, cfg_2b());
  std::printf(
      "[ hardening ] stop-and-go, silent wheel driver: %d of 100 aligned with the 2b keys, %d "
      "with the gap bound (median %.1f m, NEES %.2f, %d beyond 3 sigma)\n",
      without.aligned,
      with.aligned,
      with.median_driven(),
      with.nees,
      with.beyond_3s);
  EXPECT_GE(with.aligned, 99);
  EXPECT_GE(with.within_10m, 98);
  EXPECT_LE(with.beyond_3s, 2);
  EXPECT_LE(without.aligned, 90);  // negative control
}

// H4 review I-1: the gap rule is relative to the measured wheel period, so at 10 Hz or 5 Hz
// (+-10 ms jitter) an ordinary sample interval adds no bound and the protection is that of 50 Hz.
// A body still against a kerb, its wheels at 0.5 m/s, the receiver drifting 0.2 m per fix (white
// 0.3 m): 4 of 100 align at 50, 10 and 5 Hz alike, no ordinary interval being a gap (the review
// probe, with the old absolute 0.1 s threshold: 97 of 100
// at 10 Hz). Warehouse aisles align after a median 4.5 m at every rate (11.6 m before).
TEST(GnssTrackYawHardening, WheelRateKeepsTheProtectionAndTheAisles) {
  struct Rate {
    int period_ms;
    Stats stuck;
    Stats aisles;
  };
  std::vector<Rate> rates = {{20, {}, {}}, {100, {}, {}}, {200, {}, {}}};
  for (Rate& r : rates) {
    Scene sc;
    sc.pipe_odometer = true;
    sc.wheel_period_ms = r.period_ms;
    sc.wheel_jitter_ms = 10;
    Scene stuck = sc;
    stuck.shape = Shape::kStuck;
    stuck.white_m = 0.3;
    stuck.declared_m = 0.3;
    stuck.drift_m = 0.2;
    r.stuck = sweep(stuck, cfg_gap());
    Scene aisles = sc;
    aisles.shape = Shape::kAisles;
    r.aisles = sweep(aisles, cfg_gap());
    std::printf(
        "[ hardening ] wheel at %d ms +-10 ms: stuck wheels %d of 100 aligned (%d beyond 3 "
        "sigma), aisles %d aligned, median %.1f m, %d beyond 3 sigma\n",
        r.period_ms,
        r.stuck.aligned,
        r.stuck.beyond_3s,
        r.aisles.aligned,
        r.aisles.median_driven(),
        r.aisles.beyond_3s);
  }
  for (const Rate& r : rates) {
    EXPECT_LE(r.stuck.aligned, rates[0].stuck.aligned + 1) << r.period_ms;
    EXPECT_LE(r.stuck.aligned, 5) << r.period_ms;
    EXPECT_GE(r.aisles.aligned, 99) << r.period_ms;
    EXPECT_NEAR(r.aisles.median_driven(), rates[0].aisles.median_driven(), 0.5) << r.period_ms;
    EXPECT_LE(r.aisles.median_driven(), 5.5) << r.period_ms;
  }
}

// H4 review m-9: the gap bound widens the consistency tolerance by exactly its size. Noise-free
// straight track at 0.1 m per fix, declared at 1 mm, no slip: from fix 45 the odometer reads
// 1 m more than the travel. With a bound of 1.05 m the window agrees; with 0.95 m it mismatches.
TEST(GnssTrackYawHardening, GapBoundWidensTheToleranceByItsSize) {
  GnssConfig c = cfg_gap();
  c.align_yaw_min_var_m2 = 1e-6;
  c.align_yaw_wheel_slip_frac = 0.0;
  for (const double factor : {1.05, 0.95}) {
    GnssTrackYawAligner a(c);
    for (int k = 0; k < 100; ++k) {
      const Pose b = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
      TrackYawReadings in;
      in.wheel_odometer_m = 0.1 * k + (k >= 45 ? 1.0 : 0.0);
      in.wheel_gap_bound_m = k >= 45 ? factor : 0.0;
      in.heading_variation_rad = 0.0;
      (void)a.update(kTew * (b * kPbg), Mat3::Identity() * 1e-6, b, kPbg, in);
    }
    if (factor > 1.0) {
      EXPECT_EQ(a.n_wheel_mismatches(), 0);
    } else {
      EXPECT_GE(a.n_wheel_mismatches(), 1);
    }
  }
}

// H4 review m-9: a bound enters the segments as its step does. The step into fix 7 of chunk 3
// moves the centroid of chunk 3 by 3/10 of it and that of chunk 4 by all of it, so an over-read
// of 1.5 m there adds 0.45 m to segment 2-3 and 1.05 m to segment 3-4; its 1.5 m bound, split the
// same way, keeps both within tolerance. Bounding segment 3-4 only (the half chunk of the fix, as
// in the first H4 version) leaves 0.45 m unbounded on 2-3: a mismatch. Without the bound: two.
TEST(GnssTrackYawHardening, GapBoundEntersTheSegmentsAsItsStep) {
  GnssConfig c = cfg_gap();
  c.align_yaw_min_var_m2 = 1e-6;
  for (const bool with_bound : {true, false}) {
    GnssTrackYawAligner a(c);
    for (int k = 0; k < 100; ++k) {
      const Pose b = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
      TrackYawReadings in;
      in.wheel_odometer_m = 0.1 * k + (k >= 37 ? 1.5 : 0.0);
      in.wheel_gap_bound_m = with_bound && k >= 37 ? 1.5 : 0.0;
      in.heading_variation_rad = 0.0;
      (void)a.update(kTew * (b * kPbg), Mat3::Identity() * 1e-6, b, kPbg, in);
    }
    EXPECT_EQ(a.n_wheel_mismatches(), with_bound ? 0 : 2) << with_bound;
  }
}

// The bound enters both segments with the step's weights, so a bound short of the excess shows in
// both: a 1 m over-read at fix 7 of chunk 3 (0.3 m to segment 2-3, 0.7 m to 3-4) with a 0.95 m
// bound (0.285 / 0.665), noise-free, no slip: a mismatch on 2-3, the window restarts, and a second
// one on 3-4. Putting all of the bound on the later segment would hide the second.
TEST(GnssTrackYawHardening, GapBoundShortfallShowsInBothSegments) {
  GnssConfig c = cfg_gap();
  c.align_yaw_min_var_m2 = 1e-6;
  c.align_yaw_wheel_slip_frac = 0.0;
  GnssTrackYawAligner a(c);
  for (int k = 0; k < 100; ++k) {
    const Pose b = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
    TrackYawReadings in;
    in.wheel_odometer_m = 0.1 * k + (k >= 37 ? 1.0 : 0.0);
    in.wheel_gap_bound_m = k >= 37 ? 0.95 : 0.0;
    in.heading_variation_rad = 0.0;
    (void)a.update(kTew * (b * kPbg), Mat3::Identity() * 1e-6, b, kPbg, in);
  }
  EXPECT_EQ(a.n_wheel_mismatches(), 2);
}

// The bound's growth goes with the reading whose step it bounds: the reading of fix 35 is missing
// (bridged by fix 36, which carries a 1 m over-read), while the bound grows to 1.05 m at fix 35.
// The growth lands on fix 36's step: no mismatch. Taken at fix 35, it would be lost.
TEST(GnssTrackYawHardening, GapBoundGoesWithTheBridgedReading) {
  GnssConfig c = cfg_gap();
  c.align_yaw_min_var_m2 = 1e-6;
  c.align_yaw_wheel_slip_frac = 0.0;
  ASSERT_GE(c.align_yaw_wheel_max_gap_fixes, 1);
  GnssTrackYawAligner a(c);
  for (int k = 0; k < 100; ++k) {
    const Pose b = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
    TrackYawReadings in;
    in.wheel_odometer_m = k == 35 ? kNaN : 0.1 * k + (k >= 36 ? 1.0 : 0.0);
    in.wheel_gap_bound_m = k >= 35 ? 1.05 : 0.0;
    in.heading_variation_rad = 0.0;
    (void)a.update(kTew * (b * kPbg), Mat3::Identity() * 1e-6, b, kPbg, in);
  }
  EXPECT_EQ(a.n_wheel_mismatches(), 0);
}

// A step whose odometer reading holds an interpolated gap is not used in the wheel path (its
// length is off by up to the bound): noise-free 1 m circle, every segment through the wheel path;
// the reading at fix 40 over-reads by 0.3 m with a bound of 0.3 m. The yaw stays exact; through
// the path it would bend by the misplaced step.
TEST(GnssTrackYawHardening, GapStepIsNotInTheWheelPath) {
  for (const bool with_bound : {true, false}) {
    GnssConfig c = cfg_gap();
    c.align_yaw_min_var_m2 = 1e-6;
    GnssTrackYawAligner a(c);
    double psi = 0.0;
    Vec3 p = Vec3::Zero();
    double odo = 0.0;
    double phantom = 0.0;
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (int k = 0; k < 200; ++k) {
      if (k > 0) {
        const double h = 0.05;  // 0.1 m and 0.1 rad per fix: 1 m radius
        const double chord = 0.1 * std::sin(h) / h;
        p.x() += chord * std::cos(psi + h);
        p.y() += chord * std::sin(psi + h);
        psi += 2.0 * h;
        odo += 0.1;
      }
      if (k == 40) {
        phantom = 0.3;
      }
      const Pose b = yaw_pose(psi, p);
      TrackYawReadings in;
      in.wheel_odometer_m = odo + phantom;
      in.heading_variation_rad = psi;
      in.wheel_gap_bound_m = with_bound ? phantom : kNaN;
      r = a.update(kTew * (b * kPbg), Mat3::Identity() * 1e-6, b, kPbg, in);
    }
    ASSERT_TRUE(r.is_ok()) << r.reason();
    const double err =
        std::abs(wrap_pi(std::atan2(r.value().R(1, 0), r.value().R(0, 0)) + 75.0 * kDeg));
    if (with_bound) {
      EXPECT_LT(err, 1e-9);
    } else {
      EXPECT_GT(err, 1e-4);  // negative control: the phantom step bends the path
    }
  }
}

// ---------------------------------------------------------------------------------------------
// G10: the coloured-drift term.

// Known answer: on a noise-free straight track (lever arm fixed in w, so |Z| = r) every segment
// adds rate x dt, so sigma^2 grows by rate x (time between the first and last chunk centres) /
// r^2. Without fix times the term is off.
TEST(GnssTrackYawHardening, DriftTermIsRateTimesTime) {
  constexpr double kRate = 0.003;  // m^2/s
  GnssConfig off = cfg_hardened();
  off.align_yaw_drift_var_m2_per_s = 0.0;
  GnssConfig on = cfg_hardened();
  on.align_yaw_drift_var_m2_per_s = kRate;
  const auto timed = [](int k) {
    TrackYawReadings in;
    in.wheel_odometer_m = 0.1 * k;
    in.t = Timestamp{static_cast<std::int64_t>(k) * 200'000'000};  // 5 Hz
    in.has_time = true;
    return in;
  };
  constexpr int kFixes = 100;  // 10 chunks: centres from 0.9 s to 18.9 s
  const auto x = [](int k) { return 0.1 * k; };
  const GnssTrackYawAligner a_off = straight(off, kFixes, x, timed);
  const GnssTrackYawAligner a_on = straight(on, kFixes, x, timed);
  const double r = a_on.track_m();
  ASSERT_GT(r, 5.0);
  EXPECT_NEAR(a_on.track_m(), a_off.track_m(), 1e-12);
  const double expected = kRate * 18.0 / (r * r);
  EXPECT_NEAR(
      a_on.yaw_sigma_rad() * a_on.yaw_sigma_rad() - a_off.yaw_sigma_rad() * a_off.yaw_sigma_rad(),
      expected,
      1e-9 * expected + 1e-15);
  // No fix times: identical to the rate 0 bit for bit.
  const auto untimed = [](int k) {
    TrackYawReadings in;
    in.wheel_odometer_m = 0.1 * k;
    return in;
  };
  const GnssTrackYawAligner b_on = straight(on, kFixes, x, untimed);
  EXPECT_EQ(b_on.yaw_sigma_rad(), a_off.yaw_sigma_rad());
  // A chunk without fix times (chunk 3) takes no drift for the two segments that touch it: 14 s of
  // the 18 s.
  const auto gap3 = [&timed](int k) {
    TrackYawReadings in = timed(k);
    in.has_time = k < 30 || k >= 40;
    return in;
  };
  const GnssTrackYawAligner c_on = straight(on, kFixes, x, gap3);
  const double expected_14 = kRate * 14.0 / (r * r);
  EXPECT_NEAR(
      c_on.yaw_sigma_rad() * c_on.yaw_sigma_rad() - a_off.yaw_sigma_rad() * a_off.yaw_sigma_rad(),
      expected_14,
      1e-9 * expected_14 + 1e-15);
}

// The stop-and-go scene of SilentWheelDriverStopsAreBounded with the drift term as well: 100 of
// 100, median 3.3 m, none beyond 3 sigma (NEES 0.93, against 2 beyond and 1.2 with the bound
// alone). The term widens the window too: without the bound 99 of 100 align.
TEST(GnssTrackYawHardening, DriftTermWithTheGapBound) {
  Scene sc;
  sc.stall_every_s = 4.0;
  sc.stall_s = 0.7;
  const Stats with = sweep(sc, cfg_hardened());
  std::printf(
      "[ hardening ] stop-and-go with the drift term: %d of 100 aligned (median %.1f m, NEES "
      "%.2f, %d beyond 3 sigma)\n",
      with.aligned,
      with.median_driven(),
      with.nees,
      with.beyond_3s);
  EXPECT_GE(with.aligned, 99);
  EXPECT_GE(with.within_10m, 98);
  EXPECT_LE(with.beyond_3s, 1);
}

// Only the segments that are used take the drift term: 5 moving chunks, a 30 s rest (15 chunks,
// no extension), 5 moving chunks. The 10 moving segments (both edges of the rest included) add
// rate x 2 s each; the 14 stationary ones (a_k = 0) add nothing.
TEST(GnssTrackYawHardening, DriftTermOnlyOnUsedSegments) {
  constexpr double kRate = 0.003;  // m^2/s
  GnssConfig off = cfg_hardened();
  off.align_yaw_merge_max_chunks = 0;
  off.align_yaw_drift_var_m2_per_s = 0.0;
  GnssConfig on = off;
  on.align_yaw_drift_var_m2_per_s = kRate;
  const auto x = [](int k) { return 0.1 * (k < 50 ? k : (k < 200 ? 50 : k - 150)); };
  const auto timed = [&x](int k) {
    TrackYawReadings in;
    in.wheel_odometer_m = x(k);
    in.t = Timestamp{static_cast<std::int64_t>(k) * 200'000'000};
    in.has_time = true;
    return in;
  };
  const GnssTrackYawAligner a_off = straight(off, 250, x, timed);
  const GnssTrackYawAligner a_on = straight(on, 250, x, timed);
  const double r = a_on.track_m();
  ASSERT_GT(r, 5.0);
  EXPECT_GE(a_on.n_stationary_segments(), 14);
  const double expected = kRate * 20.0 / (r * r);
  EXPECT_NEAR(
      a_on.yaw_sigma_rad() * a_on.yaw_sigma_rad() - a_off.yaw_sigma_rad() * a_off.yaw_sigma_rad(),
      expected,
      1e-9 * expected + 1e-15);
}

// Review F2: the slow-segment extension aligned a slow robot under coloured drift confidently
// wrong (0.05 m/s at 0.02 m per fix: 10 of 100 beyond 3 sigma on the review's seeds; 0.30 m white,
// declared 0.5 m). With the drift term at the drift's rate (0.02^2 x 5 Hz = 0.002 m^2/s) the
// sigma covers it: none beyond 3 sigma, every run aligned, within 10 m; at the simulator rate
// (0.0017) the same. Without it (control): 10 beyond 3 sigma.
TEST(GnssTrackYawHardening, DriftTermKeepsSlowRobotsWithinSigma) {
  Scene sc;
  sc.speed_mps = 0.05;
  sc.white_m = 0.3;
  sc.drift_m = 0.02;
  sc.tmax_s = 600.0;
  GnssConfig matched = cfg_hardened();
  matched.align_yaw_drift_var_m2_per_s = 0.002;
  GnssConfig none = cfg_hardened();
  none.align_yaw_drift_var_m2_per_s = 0.0;
  const Stats s_matched = sweep(sc, matched);
  const Stats s_default = sweep(sc, cfg_hardened());
  const Stats s_none = sweep(sc, none);
  std::printf(
      "[ hardening ] 0.05 m/s, 0.02 m per fix of drift: beyond 3 sigma %d (no drift term), %d "
      "(rate 0.0017), %d (rate 0.002); aligned %d / %d / %d; median %.1f / %.1f / %.1f m\n",
      s_none.beyond_3s,
      s_default.beyond_3s,
      s_matched.beyond_3s,
      s_none.aligned,
      s_default.aligned,
      s_matched.aligned,
      s_none.median_driven(),
      s_default.median_driven(),
      s_matched.median_driven());
  for (const Stats* s : {&s_matched, &s_default}) {
    EXPECT_EQ(s->aligned, kRuns);
    EXPECT_LE(s->beyond_3s, 1);
    EXPECT_GE(s->within_10m, 95);
  }
  EXPECT_GE(s_none.beyond_3s, 5);  // negative control
}

// The term keeps the alignment the extension bought: white fixes only (0.5 m), 0.03 m/s with
// wheel data, the simulator rate: every run aligns, later (median 6.0 m instead of 3.1 m) and
// within its sigma.
TEST(GnssTrackYawHardening, DriftTermKeepsTheSlowRobotAlignment) {
  Scene sc;
  sc.speed_mps = 0.03;
  sc.tmax_s = 900.0;
  const Stats s = sweep(sc, cfg_hardened());
  std::printf("[ hardening ] 0.03 m/s, white fixes: %d aligned, median %.1f m, NEES %.2f\n",
              s.aligned,
              s.median_driven(),
              s.nees);
  EXPECT_EQ(s.aligned, kRuns);
  EXPECT_LT(s.median_driven(), 10.0);
  EXPECT_LE(s.beyond_3s, 1);
}

// ---------------------------------------------------------------------------------------------
// G3: front-end/wheel disagreement counter.

// Review F4: a front end that passes the gate but does not move (its LiDAR sees surroundings that
// move with the robot) holds every segment still: the robot driving at 0.5 m/s with wheel data
// never aligns, which used to be silent. The counter now shows it in every run. An exact front
// end, also at 0.05 m/s where its displacements are near the 0.2 m threshold, is never counted.
TEST(GnssTrackYawHardening, FrozenFrontEndIsCounted) {
  Scene sc;
  sc.frontend = true;
  sc.frozen_frontend = true;
  const Stats frozen = sweep(sc, cfg_gap());
  EXPECT_EQ(frozen.aligned, 0);
  EXPECT_EQ(frozen.fe_disagree_runs, kRuns);
  sc.frozen_frontend = false;
  const Stats exact = sweep(sc, cfg_gap());
  EXPECT_EQ(exact.aligned, kRuns);
  EXPECT_EQ(exact.fe_disagree_runs, 0);
  sc.speed_mps = 0.05;
  sc.tmax_s = 400.0;
  const Stats slow = sweep(sc, cfg_gap());
  EXPECT_EQ(slow.aligned, kRuns);
  EXPECT_EQ(slow.fe_disagree_runs, 0);
}

// ---------------------------------------------------------------------------------------------
// G12: the four mutants of the 2b review that survived.

// Mutant `ext_path_ok` (the wheel path of an extended segment without carry_.path_ok): the
// segment from chunk 2 is held still by the front end and extended over chunks 3 and 4; chunk 3
// holds a bridged reading (fix 34), so the extended span has no wheel path; the front end stops
// observing in chunk 5, where the travel reaches 0.24 m. The segment is then skipped, not taken
// along a wheel path with a hole.
TEST(GnssTrackYawHardening, ExtendedSegmentWithAHoleInItsPathIsSkipped) {
  const GnssTrackYawAligner a = straight(cfg_gap(), 60, fast_then_slow, [](int k) {
    TrackYawReadings in;
    in.wheel_odometer_m = k == 33 ? kNaN : fast_then_slow(k);
    in.heading_variation_rad = 0.0;
    in.travel_unobserved_scans = k >= 53 ? 1.0 : 0.0;
    return in;
  });
  EXPECT_EQ(a.n_stationary_segments(), 2);  // extended over chunks 3 and 4
  EXPECT_EQ(a.n_skipped_turns(), 1);
  EXPECT_EQ(a.n_wheel_turn_segments(), 0);
}

// Mutant `carried wheel_ok` (carry_.wheel_ok = true): with a gap longer than
// align_yaw_wheel_max_gap_fixes (1 here) in the first half of chunk 3, the segment from chunk 2 has
// no wheel data; the front end holds it still and it is extended over chunk 3. When the front end
// stops observing (chunk 4) the extended segment still has no wheel data (the gap's travel is
// missing), so the wheels cannot call it stationary, and without a wheel path it is skipped.
TEST(GnssTrackYawHardening, ExtendedSegmentKeepsItsMissingReading) {
  GnssConfig c = cfg_gap();
  c.align_yaw_wheel_max_gap_fixes = 1;
  const GnssTrackYawAligner a = straight(c, 50, fast_then_slow, [](int k) {
    TrackYawReadings in;
    in.wheel_odometer_m = (k == 31 || k == 32) ? kNaN : fast_then_slow(k);
    in.heading_variation_rad = 0.0;
    in.travel_unobserved_scans = k >= 45 ? 1.0 : 0.0;
    return in;
  });
  EXPECT_EQ(a.n_stationary_segments(), 1);  // the extension over chunk 3 only
  EXPECT_EQ(a.n_skipped_turns(), 1);
}

// Mutant `rr4_po_body` (the front-end position of b instead of o): turning in place about the
// axle centre o with the IMU 0.3 m ahead of it (p_b_o = (-0.3, 0, 0)), 90 deg per chunk, the
// travel axis observed: o stays still, so every segment is stationary and none takes the
// front-end path (b moves about 0.38 m between chunk means).
TEST(GnssTrackYawHardening, FrontEndPositionIsOfTheOdometryFrame) {
  GnssConfig c = cfg_gap();
  Pose T_b_o;
  T_b_o.p = Vec3(-0.3, 0.0, 0.0);
  GnssTrackYawAligner a(c, T_b_o);
  std::mt19937 rng(5);
  std::normal_distribution<double> n01(0.0, 1.0);
  int aligned = 0;
  for (int k = 0; k < 300; ++k) {
    const double psi = kPi / 20.0 * k;  // 90 deg per 2 s chunk
    Pose b = yaw_pose(psi, Vec3::Zero());
    b.p = -(b.R * T_b_o.p);  // o fixed at the origin: p_w_o = p_w_b + R_w_b p_b_o = 0
    Vec3 p = kTew * (b * kPbg);
    p.x() += 0.3 * n01(rng);
    p.y() += 0.3 * n01(rng);
    TrackYawReadings in;
    in.wheel_odometer_m = 0.0;
    in.heading_variation_rad = psi;
    in.travel_unobserved_scans = 0.0;
    aligned += a.update(p, Mat3::Identity() * 0.25, b, kPbg, in).is_ok() ? 1 : 0;
  }
  EXPECT_EQ(aligned, 0);
  EXPECT_EQ(a.n_wheel_turn_segments(), 0);
  EXPECT_GT(a.n_stationary_segments(), 20);
}

// The readings overload equals the positional one bit for bit when it carries the same readings.
TEST(GnssTrackYawHardening, ReadingsOverloadMatchesPositional) {
  GnssConfig c = cfg_gap();
  GnssTrackYawAligner a(c);
  GnssTrackYawAligner b(c);
  std::mt19937 rng(11);
  std::normal_distribution<double> n01(0.0, 1.0);
  for (int k = 0; k < 400; ++k) {
    const double psi = 0.02 * k;
    const Pose body =
        yaw_pose(psi, Vec3(0.1 * k * std::cos(0.01 * k), 0.1 * k * std::sin(0.01 * k), 0.0));
    Vec3 p = kTew * (body * kPbg);
    p.x() += 0.5 * n01(rng);
    p.y() += 0.5 * n01(rng);
    const double odo = k % 37 == 5 ? kNaN : 0.1 * k;
    const double count = static_cast<double>(k / 50);
    TrackYawReadings in;
    in.wheel_odometer_m = odo;
    in.heading_variation_rad = std::abs(psi);
    in.travel_unobserved_scans = count;
    const auto ra = a.update(p, Mat3::Identity() * 0.25, body, kPbg, in);
    const auto rb = b.update(p, Mat3::Identity() * 0.25, body, kPbg, odo, std::abs(psi), count);
    ASSERT_EQ(ra.is_ok(), rb.is_ok()) << k;
    if (ra.is_ok()) {
      EXPECT_TRUE((ra.value().R.array() == rb.value().R.array()).all()) << k;
    }
    EXPECT_EQ(a.yaw_sigma_rad(), b.yaw_sigma_rad()) << k;
  }
}

}  // namespace
