// Unit tests of the design-2b additions to gnss::GnssTrackYawAligner (ADR-023 amendment,
// docs/derivations/gnss.md section 5.1): turn-gated segments with their direction from the wheel
// path, the minimum track with wheel data, and the observability of yaw from an in-place
// rotation (a Monte Carlo of the manoeuvre, no production code). Fixed seeds.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"

#include "alloc_counter.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Vec3;
using mss::gnss::GnssConfig;
using mss::gnss::GnssTrackYawAligner;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kSigmaM = 0.5;      // m, fix noise per axis (simulator working value)
const Vec3 kPbg(-0.20, 0.10, 0.60);  // m, lever arm in b (sim configs, 0.22 m horizontal)
const double kNaN = std::numeric_limits<double>::quiet_NaN();

double wrap_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

double yaw_of(const Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

Pose yaw_pose(double yaw_rad, const Vec3& p) {
  Pose T;
  T.R = Eigen::AngleAxisd(yaw_rad, Vec3::UnitZ()).toRotationMatrix();
  T.p = p;
  return T;
}

const Pose kTew = yaw_pose(30.0 * kDeg, Vec3(12.0, -7.0, 1.0));  // true T_e_w

Mat3 fix_cov(double sigma_m) {
  return Mat3::Identity() * sigma_m * sigma_m;
}

GnssConfig cfg_2b(bool wheel_turns) {
  GnssConfig c;
  c.enabled = true;
  c.align_yaw_from_track = true;
  c.align_yaw_wheel_turns = wheel_turns;
  c.align_yaw_min_track_wheel_m = 0.0;
  return c;
}

// A path sampled at the fixes: per fix interval a constant speed (m per fix) and yaw rate (rad per
// fix), so the body moves on an exact arc between fixes. `odo` is the signed wheel odometer.
struct Step {
  double ds_m;
  double dpsi_rad;
};
struct PathFix {
  Pose body;
  double odo_m;
};
std::vector<PathFix> integrate(const std::vector<Step>& steps) {
  std::vector<PathFix> out;
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double odo = 0.0;
  out.push_back({yaw_pose(psi, Vec3(x, y, 0.0)), odo});
  for (const Step& s : steps) {
    const double h = 0.5 * s.dpsi_rad;
    const double chord = std::abs(h) > 1e-12 ? s.ds_m * std::sin(h) / h : s.ds_m;
    x += chord * std::cos(psi + h);
    y += chord * std::sin(psi + h);
    psi += s.dpsi_rad;
    odo += s.ds_m;
    out.push_back({yaw_pose(psi, Vec3(x, y, 0.0)), odo});
  }
  return out;
}

// Warehouse aisles at 5 Hz: `straight_fixes` at 0.1 m per fix (0.5 m/s), then a 90 deg turn in 8
// fixes at 0.06 m per fix (0.3 m/s, 0.98 rad/s, radius 0.31 m), alternating left and right.
std::vector<Step> aisles(int straight_fixes, int n_turns) {
  std::vector<Step> s(10, Step{0.0, 0.0});  // 2 s at rest
  for (int k = 0; k < n_turns; ++k) {
    for (int i = 0; i < straight_fixes; ++i) {
      s.push_back({0.1, 0.0});
    }
    const double sign = k % 2 == 0 ? 1.0 : -1.0;
    for (int i = 0; i < 8; ++i) {
      s.push_back({0.06, sign * 0.5 * kPi / 8.0});
    }
  }
  return s;
}

struct FirstAlignment {
  int fix = -1;  // -1: never
  double err_rad = 0.0;
  double sigma_rad = 0.0;
  double odo_m = 0.0;
};

// Feeds the path with 0.5 m noise (rng) or none; odometer NaN when `wheel` is false; returns the
// first transform.
FirstAlignment first_alignment(GnssTrackYawAligner& a,
                               const std::vector<PathFix>& path,
                               std::mt19937* rng,
                               bool wheel) {
  std::normal_distribution<double> n01(0.0, 1.0);
  for (std::size_t k = 0; k < path.size(); ++k) {
    Vec3 p = kTew * (path[k].body * kPbg);
    if (rng != nullptr) {
      p.x() += kSigmaM * n01(*rng);
      p.y() += kSigmaM * n01(*rng);
    }
    const auto r = a.update(p, fix_cov(kSigmaM), path[k].body, kPbg, wheel ? path[k].odo_m : kNaN);
    if (r.is_ok()) {
      return {static_cast<int>(k),
              wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)),
              a.yaw_sigma_rad(),
              path[k].odo_m};
    }
  }
  return {};
}

// Noise-free 1 m circle (57 deg per chunk, every segment turn-gated) with the box-world lever arm:
// through the wheel path the yaw is exact (the steps are exact arcs, the lever arm is inside the
// displacement). Noise-free aisles: the turn-gated segments are used, and the yaw is within
// 0.1 deg; the residual (0.02 deg here, 0.04 deg with the flag off) comes from the gate-passing
// segments whose chunks hold part of a turn, where the mean heading is not the chord direction.
TEST(GnssTrackYaw2b, WheelTurnsNoiseFreeExact) {
  {
    std::vector<Step> steps(10, Step{0.0, 0.0});
    for (int i = 0; i < 300; ++i) {
      steps.push_back({0.1, 0.1});
    }
    const std::vector<PathFix> circle = integrate(steps);
    GnssConfig c = cfg_2b(true);
    c.align_yaw_min_var_m2 = 1e-9;
    GnssTrackYawAligner a(c);
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (const PathFix& f : circle) {
      r = a.update(kTew * (f.body * kPbg), fix_cov(0.001), f.body, kPbg, f.odo_m);
    }
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 1e-9);
    EXPECT_GT(a.n_wheel_turn_segments(), 25);
  }
  const std::vector<PathFix> path = integrate(aisles(20, 12));
  for (const bool on : {false, true}) {
    GnssConfig c = cfg_2b(on);
    c.align_yaw_min_var_m2 = 1e-9;
    GnssTrackYawAligner a(c);
    mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
    for (const PathFix& f : path) {
      r = a.update(kTew * (f.body * kPbg), fix_cov(0.001), f.body, kPbg, f.odo_m);
    }
    ASSERT_TRUE(r.is_ok()) << r.reason();
    EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 0.1 * kDeg) << on;
    if (on) {
      EXPECT_GT(a.n_wheel_turn_segments(), 10);
      EXPECT_EQ(a.n_skipped_turns(), 0);
    } else {
      EXPECT_EQ(a.n_wheel_turn_segments(), 0);
      EXPECT_GT(a.n_skipped_turns(), 10);
    }
  }
}

// 0.5 m fixes on aisles with 2 m straights, 200 seeds, 3 and 6 deg: with the wheel path every run
// aligns, earlier than without it, and the yaw error stays within the predicted sigma.
// Measured (median distance driven at the first transform): 3 deg 33.7 m (2 of 200 runs aligned)
// -> 7.9 m; 6 deg 8.9 -> 2.6 m.
TEST(GnssTrackYaw2b, WheelTurnsAlignSoonerOnAisles) {
  const std::vector<PathFix> path = integrate(aisles(20, 14));
  for (const double sigma_deg : {3.0, 6.0}) {
    std::vector<double> dist_on;
    std::vector<double> dist_off;
    double nees = 0.0;
    int beyond_3s = 0;
    int aligned_off = 0;
    for (std::uint32_t seed = 1; seed <= 200; ++seed) {
      for (const bool on : {false, true}) {
        GnssConfig c = cfg_2b(on);
        c.align_yaw_max_sigma_rad = sigma_deg * kDeg;
        GnssTrackYawAligner a(c);
        std::mt19937 rng(seed);
        const FirstAlignment f = first_alignment(a, path, &rng, true);
        if (!on) {
          aligned_off += f.fix >= 0 ? 1 : 0;
          if (f.fix >= 0) {
            dist_off.push_back(f.odo_m);
          }
          continue;
        }
        ASSERT_GE(f.fix, 0) << seed;
        dist_on.push_back(f.odo_m);
        const double z = f.err_rad / f.sigma_rad;
        nees += z * z;
        beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
      }
    }
    std::sort(dist_on.begin(), dist_on.end());
    std::sort(dist_off.begin(), dist_off.end());
    const double med_on = dist_on[dist_on.size() / 2];
    const double med_off = dist_off.empty() ? kNaN : dist_off[dist_off.size() / 2];
    std::printf(
        "[ track yaw 2b ] aisles, %.0f deg: wheel turns median %.1f m driven (yaw NEES %.2f, %d "
        "beyond 3 sigma); without: %d of 200 aligned, median %.1f m\n",
        sigma_deg,
        med_on,
        nees / 200.0,
        beyond_3s,
        aligned_off,
        med_off);
    EXPECT_GT(nees / 200.0, 0.7);
    EXPECT_LT(nees / 200.0, 1.3);
    EXPECT_LE(beyond_3s, 3);
    if (aligned_off > 0) {
      EXPECT_LT(med_on, med_off);
    }
  }
}

// Wheel slip and skid in each turn (odometer scale error and a heading offset of the motion, both
// N(0, 0.05) per turn, the configured align_yaw_wheel_slip_frac): the predicted sigma still covers
// the error.
TEST(GnssTrackYaw2b, WheelTurnsSlipWithinSigma) {
  double nees = 0.0;
  int beyond_3s = 0;
  constexpr int kRuns = 200;
  for (std::uint32_t seed = 1; seed <= kRuns; ++seed) {
    std::mt19937 rng(1000U + seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    // Truth path with skid: in a turn the body moves at a heading offset beta from x_b and the
    // odometer reads (1 + eps) of the travel.
    std::vector<PathFix> path;
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double odo = 0.0;
    path.push_back({yaw_pose(psi, Vec3(x, y, 0.0)), odo});
    double eps = 0.0;
    double beta = 0.0;
    int in_turn = 0;
    for (const Step& s : aisles(20, 14)) {
      const bool turn = std::abs(s.dpsi_rad) > 0.0;
      if (turn && in_turn == 0) {
        eps = 0.05 * n01(rng);
        beta = 0.05 * n01(rng);
      }
      in_turn = turn ? in_turn + 1 : 0;
      const double h = 0.5 * s.dpsi_rad;
      const double chord = std::abs(h) > 1e-12 ? s.ds_m * std::sin(h) / h : s.ds_m;
      const double off = turn ? beta : 0.0;
      x += chord * std::cos(psi + h + off);
      y += chord * std::sin(psi + h + off);
      psi += s.dpsi_rad;
      odo += s.ds_m * (1.0 + (turn ? eps : 0.0));
      path.push_back({yaw_pose(psi, Vec3(x, y, 0.0)), odo});
    }
    GnssTrackYawAligner a(cfg_2b(true));
    const FirstAlignment f = first_alignment(a, path, &rng, true);
    ASSERT_GE(f.fix, 0) << seed;
    const double z = f.err_rad / f.sigma_rad;
    nees += z * z;
    beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
  }
  std::printf(
      "[ track yaw 2b ] aisles with 5 %% slip and skid per turn: yaw NEES %.2f, %d of %d "
      "beyond 3 sigma\n",
      nees / kRuns,
      beyond_3s,
      kRuns);
  EXPECT_LT(nees / kRuns, 1.3);
  EXPECT_LE(beyond_3s, 3);
}

// A circle of 1 m at 0.5 m/s turns 57 deg per chunk: every segment is turn-gated, so design 2
// never aligns; with the wheel path it does, within the predicted sigma (100 seeds).
TEST(GnssTrackYaw2b, WheelTurnsTightCircle) {
  std::vector<Step> steps(10, Step{0.0, 0.0});
  for (int i = 0; i < 600; ++i) {
    steps.push_back({0.1, 0.1});  // 0.5 m/s, 0.5 rad/s
  }
  const std::vector<PathFix> path = integrate(steps);
  double nees = 0.0;
  int beyond_3s = 0;
  for (std::uint32_t seed = 1; seed <= 100; ++seed) {
    for (const bool on : {false, true}) {
      GnssTrackYawAligner a(cfg_2b(on));
      std::mt19937 rng(seed);
      const FirstAlignment f = first_alignment(a, path, &rng, true);
      if (!on) {
        EXPECT_LT(f.fix, 0) << seed;
        continue;
      }
      ASSERT_GE(f.fix, 0) << seed;
      const double z = f.err_rad / f.sigma_rad;
      nees += z * z;
      beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
    }
  }
  EXPECT_LT(nees / 100.0, 1.3);
  EXPECT_LE(beyond_3s, 2);
}

// Spinning in place (no wheel travel) gives segments with no direction of travel: stationary,
// never a transform, never a flip.
TEST(GnssTrackYaw2b, WheelTurnsSpinInPlaceNeverAligns) {
  std::vector<Step> steps(600, Step{0.0, 0.2});  // 1 rad/s, no travel
  const std::vector<PathFix> path = integrate(steps);
  std::mt19937 rng(4U);
  GnssTrackYawAligner a(cfg_2b(true));
  const FirstAlignment f = first_alignment(a, path, &rng, true);
  EXPECT_LT(f.fix, 0);
  EXPECT_EQ(a.n_wheel_turn_segments(), 0);
  EXPECT_GT(a.n_stationary_segments(), 50);
}

// Without wheel data (NaN readings) the flag changes nothing: same yaw and sigma, bit for bit, as
// with the flag off, and the turns stay gated. One NaN reading inside a turn gates only the
// segments whose chunks it touches.
TEST(GnssTrackYaw2b, WheelTurnsNeedWheelData) {
  const std::vector<PathFix> path = integrate(aisles(20, 12));
  std::mt19937 rng_a(6U);
  std::mt19937 rng_b(6U);
  GnssTrackYawAligner off(cfg_2b(false));
  GnssTrackYawAligner on(cfg_2b(true));
  const FirstAlignment fa = first_alignment(off, path, &rng_a, false);
  const FirstAlignment fb = first_alignment(on, path, &rng_b, false);
  EXPECT_EQ(fa.fix, fb.fix);
  EXPECT_EQ(fa.err_rad, fb.err_rad);
  EXPECT_EQ(fa.sigma_rad, fb.sigma_rad);
  EXPECT_EQ(on.n_wheel_turn_segments(), 0);

  // One missing reading in the first turn (fixes 30..37 turn).
  GnssConfig c = cfg_2b(true);
  c.align_yaw_min_var_m2 = 1e-9;
  GnssTrackYawAligner a(c);
  mss::common::Result<Pose> r = mss::common::Result<Pose>::error("none");
  for (std::size_t k = 0; k < path.size(); ++k) {
    const double w = k == 33 ? kNaN : path[k].odo_m;
    r = a.update(kTew * (path[k].body * kPbg), fix_cov(0.001), path[k].body, kPbg, w);
  }
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))), 0.1 * kDeg);
  EXPECT_GE(a.n_skipped_turns(), 1);
  EXPECT_LE(a.n_skipped_turns(), 3);
}

// Minimum track with wheel data (align_yaw_min_track_wheel_m): on a straight 0.5 m/s line with
// noise-free fixes the transform comes at the second chunk with 0 m, at 5 m with the design-2
// minimum, and at 5 m whenever a segment used came without wheel data.
TEST(GnssTrackYaw2b, MinTrackWheel) {
  std::vector<Step> steps(10, Step{0.0, 0.0});
  for (int i = 0; i < 200; ++i) {
    steps.push_back({0.1, 0.0});
  }
  const std::vector<PathFix> path = integrate(steps);
  const auto first_fix = [&](double min_track_wheel_m, int nan_at) {
    GnssConfig c = cfg_2b(true);
    c.align_yaw_min_track_wheel_m = min_track_wheel_m;
    c.align_yaw_min_var_m2 = 1e-9;
    GnssTrackYawAligner a(c);
    for (std::size_t k = 0; k < path.size(); ++k) {
      const double w = static_cast<int>(k) == nan_at ? kNaN : path[k].odo_m;
      if (a.update(kTew * (path[k].body * kPbg), fix_cov(0.001), path[k].body, kPbg, w).is_ok()) {
        return static_cast<int>(k);
      }
    }
    return -1;
  };
  const int no_min = first_fix(0.0, -1);
  const int five = first_fix(5.0, -1);
  const int fallback = first_fix(0.0, 5);  // a no-wheel segment: align_yaw_min_track_m (5 m)
  EXPECT_EQ(no_min, 19);                   // the first chunk pair (rest, then 0.4 m of travel)
  EXPECT_GT(five, no_min);
  EXPECT_EQ(fallback, five);
}

// The design-2b keys as shipped in config/default.yaml (6 deg, turns from the wheel path, 3 m
// minimum track with wheel data).
GnssConfig cfg_shipped() {
  GnssConfig c = cfg_2b(true);
  c.align_yaw_max_sigma_rad = 6.0 * kDeg;
  c.align_yaw_min_track_wheel_m = 3.0;
  return c;
}

// Summary of the first alignments of a Monte Carlo.
struct Envelope {
  int runs = 0;
  int aligned = 0;
  int beyond_15deg = 0;
  int beyond_30deg = 0;
  int beyond_3s = 0;
  double max_deg = 0.0;
  void add(double err_rad, double sigma_rad) {
    ++aligned;
    const double deg = std::abs(err_rad) / kDeg;
    max_deg = std::max(max_deg, deg);
    beyond_15deg += deg > 15.0 ? 1 : 0;
    beyond_30deg += deg > 30.0 ? 1 : 0;
    beyond_3s += std::abs(err_rad) > 3.0 * sigma_rad ? 1 : 0;
  }
  void print(const char* what) const {
    std::printf(
        "[ track yaw 2b ] %s: %d of %d aligned, max %.1f deg, > 15 deg %d, > 30 deg %d, "
        "beyond 3 sigma %d\n",
        what,
        aligned,
        runs,
        max_deg,
        beyond_15deg,
        beyond_30deg,
        beyond_3s);
  }
};

// Review I1 of design 2b: a robot whose wheels turn (0.5 m/s on the odometer) while the body
// does not move (stuck against an obstacle, or lifted on a stand), GNSS 0.3 m white plus a random
// walk, declared 0.5 m, 5 min, 100 seeds, shipped keys. Without the wheel/GNSS consistency test
// (r >= align_yaw_min_coherence x the wheel-predicted displacement) and with no minimum track this
// aligned on 21 / 64 of 100 runs at 0.10 / 0.20 m per fix, about 90 deg off. Now:
// 0.05 and 0.10 m per fix never align; 0.20 m per fix aligns on 19 of 100 here (16 with the probe
// seeds, the level of design 2 without wheel data for a parked robot, 16 of 100), all far off:
// a documented limit.
TEST(GnssTrackYaw2b, StuckWheelsDriftingGnssEnvelope) {
  for (const double slip : {0.05, 0.5}) {
    for (const double step_m : {0.05, 0.10, 0.20}) {
      if (slip > 0.1 && step_m != 0.10) {
        continue;
      }
      GnssConfig c = cfg_shipped();
      c.align_yaw_wheel_slip_frac = slip;
      Envelope env;
      for (std::uint32_t seed = 0; seed < 100; ++seed) {
        ++env.runs;
        std::mt19937 rng(5000U + seed);
        std::normal_distribution<double> n01(0.0, 1.0);
        GnssTrackYawAligner a(c);
        const Pose b = yaw_pose(0.1 * seed, Vec3(3.0, -2.0, 0.0));
        Vec3 walk = Vec3::Zero();
        for (int k = 0; k < 1500; ++k) {
          Vec3 p = kTew * (b * kPbg);
          for (int i = 0; i < 2; ++i) {
            walk(i) += step_m * n01(rng);
            p(i) += walk(i) + 0.3 * n01(rng);
          }
          const auto r = a.update(p, fix_cov(kSigmaM), b, kPbg, 0.1 * k);  // wheels at 0.5 m/s
          if (r.is_ok()) {
            env.add(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)), a.yaw_sigma_rad());
            break;
          }
        }
      }
      char what[96];
      std::snprintf(what,
                    sizeof(what),
                    "stuck wheels, GNSS drift %.2f m per fix, slip fraction %.2f",
                    step_m,
                    slip);
      env.print(what);
      if (step_m <= 0.10) {
        EXPECT_EQ(env.aligned, 0) << step_m << " " << slip;
      } else {
        EXPECT_LE(env.aligned, 10) << step_m;  // documented limit (measured 2)
      }
    }
  }
}

// Review I2 of design 2b: the minimum track with wheel data, chosen from the absolute-error
// envelope at the shipped 6 deg (gnss_fusion_study.md section 16.3). A parked robot with wheel
// data (odometer constant) never aligns. A straight run at 0.12 / 0.20 / 0.50 m/s with 0.3 m white
// plus a random walk, 200 seeds per case: at 0.02 m per fix no run is beyond 30 deg with or
// without the minimum; at 0.05 m per fix the 3 m minimum cuts the runs beyond 15 deg (summed
// over the three speeds, measured 65 -> 40 here; 52 -> 30 with the study probe seeds).
TEST(GnssTrackYaw2b, MinTrackWheelEnvelope) {
  for (const double step_m : {0.05, 0.10, 0.20}) {
    int n_aligned = 0;
    for (std::uint32_t seed = 1; seed <= 100; ++seed) {
      std::mt19937 rng(seed);
      std::normal_distribution<double> n01(0.0, 1.0);
      GnssTrackYawAligner a(cfg_shipped());
      const Pose b = yaw_pose(0.1 * seed, Vec3(3.0, -2.0, 0.0));
      Vec3 walk = Vec3::Zero();
      for (int k = 0; k < 1500; ++k) {
        Vec3 p = kTew * (b * kPbg);
        for (int i = 0; i < 2; ++i) {
          walk(i) += step_m * n01(rng);
          p(i) += walk(i) + 0.3 * n01(rng);
        }
        if (a.update(p, fix_cov(kSigmaM), b, kPbg, 0.0).is_ok()) {
          ++n_aligned;
          break;
        }
      }
    }
    EXPECT_EQ(n_aligned, 0) << step_m;
  }
  for (const double drift_m : {0.02, 0.05}) {
    std::array<int, 2> beyond_15deg{0, 0};  // summed over the speeds, min track 0 / 3 m
    for (const double speed_mps : {0.12, 0.20, 0.50}) {
      std::vector<Step> steps(10, Step{0.0, 0.0});
      for (int i = 0; i < 1500; ++i) {
        steps.push_back({speed_mps / 5.0, 0.0});
      }
      const std::vector<PathFix> path = integrate(steps);
      for (const double min_track_m : {0.0, 3.0}) {
        GnssConfig c = cfg_shipped();
        c.align_yaw_min_track_wheel_m = min_track_m;
        Envelope env;
        for (std::uint32_t seed = 1; seed <= 200; ++seed) {
          ++env.runs;
          std::mt19937 rng(seed);
          std::normal_distribution<double> n01(0.0, 1.0);
          GnssTrackYawAligner a(c);
          Vec3 walk = Vec3::Zero();
          for (const PathFix& f : path) {
            Vec3 p = kTew * (f.body * kPbg);
            for (int i = 0; i < 2; ++i) {
              walk(i) += drift_m * n01(rng);
              p(i) += walk(i) + 0.3 * n01(rng);
            }
            const auto r = a.update(p, fix_cov(kSigmaM), f.body, kPbg, f.odo_m);
            if (r.is_ok()) {
              env.add(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)), a.yaw_sigma_rad());
              break;
            }
          }
        }
        char what[96];
        std::snprintf(what,
                      sizeof(what),
                      "%.2f m/s, drift %.2f m per fix, min track with wheel %.0f m",
                      speed_mps,
                      drift_m,
                      min_track_m);
        env.print(what);
        // With 0.05 m per fix a few slow runs keep failing the wheel/GNSS consistency test and
        // wait instead of aligning on drift (measured 194 of 200 at 0.12 m/s with 3 m).
        EXPECT_GE(env.aligned, drift_m < 0.03 ? 200 : 190) << speed_mps << " " << min_track_m;
        beyond_15deg[min_track_m > 0.0 ? 1 : 0] += env.beyond_15deg;
        if (drift_m < 0.03) {
          EXPECT_EQ(env.beyond_30deg, 0) << speed_mps << " " << min_track_m;
        }
      }
    }
    if (drift_m > 0.03) {
      EXPECT_LT(beyond_15deg[1], beyond_15deg[0]);
    }
  }
}

// A 1 m circle at 0.5 m/s simulated at 1 kHz: the odometry frame o (axle centre) moves along its
// heading plus `skid_rad` (persistent sideslip, the same in every segment of the turn); the body
// (IMU) origin sits `x_ahead_m` ahead of o; 0.5 m fixes at 5 Hz; returns the first alignment.
FirstAlignment circle_run(const GnssConfig& c,
                          const Pose& T_b_o_given,
                          double x_ahead_m,
                          double skid_rad,
                          std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  GnssTrackYawAligner a(c, T_b_o_given);
  double ox = 0.0;
  double oy = 0.0;
  double psi = 0.0;
  double odo = 0.0;
  constexpr double kDt = 1e-3;
  for (int k = 0; k < 240000; ++k) {
    const double t = k * kDt;
    const double v = t < 2.0 ? 0.0 : 0.5;
    const double w = t < 2.0 ? 0.0 : 0.5;
    if (k % 200 == 0) {
      const Pose b =
          yaw_pose(psi, Vec3(ox + x_ahead_m * std::cos(psi), oy + x_ahead_m * std::sin(psi), 0.0));
      Vec3 p = kTew * (b * kPbg);
      p.x() += kSigmaM * n01(rng);
      p.y() += kSigmaM * n01(rng);
      const auto r = a.update(p, fix_cov(kSigmaM), b, kPbg, odo);
      if (r.is_ok()) {
        return {k / 200, wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)), a.yaw_sigma_rad(), odo};
      }
    }
    const double off = w != 0.0 ? skid_rad : 0.0;
    ox += v * std::cos(psi + off) * kDt;
    oy += v * std::sin(psi + off) * kDt;
    psi += w * kDt;
    odo += v * kDt;
  }
  return {};
}

// Review I3 of design 2b: the odometer measures the axle centre (frames.T_b_o), not the IMU.
// IMU 0.2 m ahead of the axle on a 1 m circle (every segment through the wheel path), 200 seeds,
// 6 deg: with T_b_o the yaw NEES is about 1 (measured 0.87); integrating the odometer as the
// body path (negative control) gave a median error of 12 deg, NEES 5.5.
TEST(GnssTrackYaw2b, WheelPathUsesOdometryFrame) {
  Pose T_b_o;
  T_b_o.p = Vec3(-0.2, 0.0, 0.0);  // axle centre 0.2 m behind the IMU
  for (const bool given : {true, false}) {
    double nees = 0.0;
    int beyond_3s = 0;
    for (std::uint32_t seed = 1; seed <= 200; ++seed) {
      const FirstAlignment f = circle_run(cfg_shipped(), given ? T_b_o : Pose{}, 0.2, 0.0, seed);
      ASSERT_GE(f.fix, 0) << seed;
      const double z = f.err_rad / f.sigma_rad;
      nees += z * z;
      beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
    }
    std::printf(
        "[ track yaw 2b ] IMU 0.2 m ahead of the axle, 1 m circle, T_b_o given %d: yaw "
        "NEES %.2f, %d of 200 beyond 3 sigma\n",
        given ? 1 : 0,
        nees / 200.0,
        beyond_3s);
    if (given) {
      EXPECT_GT(nees / 200.0, 0.7);
      EXPECT_LT(nees / 200.0, 1.3);
      EXPECT_LE(beyond_3s, 3);
    } else {
      EXPECT_GT(nees / 200.0, 3.0);  // negative control
    }
  }
}

// Review M1 of design 2b, pinned limit: a skid that persists through a whole turn (here a
// constant sideslip in every segment of a 1 m circle, as a skid-steer robot's offset centre of
// rotation produces) is correlated between segments, outside the error model (gnss.md 5.1).
// Measured at 6 deg, 200 seeds: 5 deg of skid, NEES 1.6 and 1 run beyond 3 sigma; 10 deg, NEES
// 4.3, 24 runs beyond 3 sigma, up to 24 deg off. Alternating turns (aisles) cancel it.
TEST(GnssTrackYaw2b, PersistentSkidPinned) {
  for (const double skid_deg : {5.0, 10.0}) {
    Envelope env;
    double nees = 0.0;
    for (std::uint32_t seed = 1; seed <= 200; ++seed) {
      ++env.runs;
      const FirstAlignment f = circle_run(cfg_shipped(), Pose{}, 0.0, skid_deg * kDeg, seed);
      ASSERT_GE(f.fix, 0) << seed;
      env.add(f.err_rad, f.sigma_rad);
      nees += (f.err_rad / f.sigma_rad) * (f.err_rad / f.sigma_rad);
    }
    char what[96];
    std::snprintf(what,
                  sizeof(what),
                  "1 m circle, persistent skid %.0f deg, NEES %.2f",
                  skid_deg,
                  nees / 200.0);
    env.print(what);
    if (skid_deg <= 5.0) {
      EXPECT_LE(env.beyond_3s, 5);
    } else {
      EXPECT_LE(env.beyond_3s, 40);  // the documented limit
      EXPECT_EQ(env.beyond_30deg, 0);
    }
  }
}

// Review M3 of design 2b: fixes that do not reach the aligner during a turn make one long step.
// Its arc shape is unknown, so a step turning more than the turn gate is not used through the
// wheel path (before: (s dpsi)^2 / 12 of that step stayed in the sigma for good, and a 9 s outage
// in a 1 rad/s turn delayed the alignment from 36 s to 137 s). 1 rad/s, R 0.5 m, 6 deg, 100 seeds.
TEST(GnssTrackYaw2b, OutageInTurnDoesNotStall) {
  std::vector<double> t_free;
  std::vector<double> t_gap;
  for (std::uint32_t seed = 1; seed <= 100; ++seed) {
    for (const bool gap : {false, true}) {
      std::mt19937 rng(seed);
      std::normal_distribution<double> n01(0.0, 1.0);
      GnssTrackYawAligner a(cfg_shipped());
      double x = 0.0;
      double y = 0.0;
      double psi = 0.0;
      double odo = 0.0;
      constexpr double kDt = 1e-3;
      for (int k = 0; k < 240000; ++k) {
        const double t = k * kDt;
        const double v = t < 2.0 ? 0.0 : 0.5;
        const double w = t < 2.0 ? 0.0 : 1.0;
        const bool in_gap = gap && t > 4.0 && t < 13.0;
        if (k % 200 == 0 && !in_gap) {
          const Pose b = yaw_pose(psi, Vec3(x, y, 0.0));
          Vec3 p = kTew * (b * kPbg);
          p.x() += kSigmaM * n01(rng);
          p.y() += kSigmaM * n01(rng);
          const auto r = a.update(p, fix_cov(kSigmaM), b, kPbg, odo);
          if (r.is_ok()) {
            EXPECT_LT(std::abs(wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R))),
                      4.0 * a.yaw_sigma_rad())
                << seed;
            (gap ? t_gap : t_free).push_back(t);
            break;
          }
        }
        const double h = 0.5 * w * kDt;
        x += v * kDt * std::cos(psi + h);
        y += v * kDt * std::sin(psi + h);
        psi += w * kDt;
        odo += v * kDt;
      }
    }
  }
  ASSERT_EQ(t_free.size(), 100U);
  ASSERT_EQ(t_gap.size(), 100U);
  std::sort(t_free.begin(), t_free.end());
  std::sort(t_gap.begin(), t_gap.end());
  const double med_free = t_free[50];
  const double med_gap = t_gap[50];
  std::printf(
      "[ track yaw 2b ] 1 rad/s turn, 9 s outage: median time to align %.1f s (no outage "
      "%.1f s)\n",
      med_gap,
      med_free);
  EXPECT_LT(med_gap, med_free + 9.0 + 10.0);  // the outage itself, plus at most 5 chunks
}

// A scene for the round-2 tests, simulated at 1 kHz: the odometry frame o moves along its own x
// axis (heading psi_o); the body is To * T_b_o^-1; 0.5 m white fixes (plus a random walk of
// `drift_m` per fix while the wheels turn on a still o, if any) at 5 Hz, none in [gap0_s, gap1_s);
// the odometer reads the wheel travel (NaN at every fix without wheel data); the heading-variation
// reading is the integral of |yaw rate| (as the pipeline sums |heading change| scan to scan).
// shape: 0 straight at 0.5 m/s, 1 aisles (4 s straight at 0.5 m/s, 90 deg turns in 1.6 s at
// 0.3 m/s, alternating), 2 circle of 1 m at 0.5 m/s. stuck_s: after the 2 s rest, the wheels turn
// at 0.5 m/s for stuck_s while o does not move. Returns the first alignment and the distance o
// drove before it.
struct Scene2 {
  int shape = 0;
  Pose T_b_o;
  double gap0_s = -1.0;
  double gap1_s = -1.0;
  bool wheel = true;
  bool heading_readings = true;
  double stuck_s = 0.0;
  double drift_m = 0.0;
  double tmax_s = 300.0;
};
struct Run2 {
  FirstAlignment f;
  double driven_m = 0.0;
};
Run2 run_scene(const Scene2& sc, const GnssConfig& c, std::uint32_t seed) {
  std::mt19937 rng(7000U + seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  GnssTrackYawAligner a(c, sc.T_b_o);
  const Pose T_o_b = sc.T_b_o.inverse();
  double ox = 0.0;
  double oy = 0.0;
  double psi = 0.3 * seed;
  double odo = 0.0;
  double turn = 0.0;
  double driven = 0.0;
  Vec3 walk = Vec3::Zero();
  constexpr double kDt = 1e-3;
  const auto n_steps = static_cast<long>(sc.tmax_s / kDt);
  for (long k = 0; k < n_steps; ++k) {
    const double t = static_cast<double>(k) * kDt;
    const double tm = t - 2.0 - sc.stuck_s;
    double v = 0.0;
    double w = 0.0;
    bool stuck = false;
    if (t < 2.0) {
    } else if (t < 2.0 + sc.stuck_s) {
      v = 0.5;
      stuck = true;
    } else if (sc.shape == 0) {
      v = 0.5;
    } else if (sc.shape == 2) {
      v = 0.5;
      w = 0.5;
    } else {
      const double period = 4.0 + 1.6;
      const double tt = std::fmod(tm, period);
      const int leg = static_cast<int>(tm / period);
      if (tt < 4.0) {
        v = 0.5;
      } else {
        v = 0.3;
        w = (leg % 2 != 0 ? -1.0 : 1.0) * (0.5 * kPi) / 1.6;
      }
    }
    if (k % 200 == 0) {
      if (stuck) {
        walk.x() += sc.drift_m * n01(rng);
        walk.y() += sc.drift_m * n01(rng);
      }
      const Pose b = yaw_pose(psi, Vec3(ox, oy, 0.0)) * T_o_b;
      Vec3 p = kTew * (b * kPbg) + walk;
      p.x() += kSigmaM * n01(rng);
      p.y() += kSigmaM * n01(rng);
      if (!(t >= sc.gap0_s && t < sc.gap1_s)) {
        const auto r = a.update(
            p, fix_cov(kSigmaM), b, kPbg, sc.wheel ? odo : kNaN, sc.heading_readings ? turn : kNaN);
        if (r.is_ok()) {
          return {{static_cast<int>(k / 200),
                   wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R)),
                   a.yaw_sigma_rad(),
                   odo},
                  driven};
        }
      }
    }
    if (!stuck) {
      ox += v * std::cos(psi + 0.5 * w * kDt) * kDt;
      oy += v * std::sin(psi + 0.5 * w * kDt) * kDt;
      driven += v * kDt;
    }
    psi += w * kDt;
    turn += std::abs(w) * kDt;
    odo += v * kDt;
  }
  return {};
}

// Review N1 of design 2b round 1: the direction of travel is the odometry frame's x axis, for the
// heading segments and the wheel path alike. Mount yaw of 5, 90 and 180 deg (T_b_o given), straight
// track (heading segments) and 1 m circle (wheel path only), 100 seeds, shipped keys: the yaw stays
// within its sigma. Before, the heading segments used x_b: 5 deg gave NEES 2.8 on a straight track,
// 90 deg 90 deg off, 180 deg a silent flip.
TEST(GnssTrackYaw2b, MountYawSetsTheTravelAxis) {
  for (const double mount_deg : {5.0, 90.0, 180.0}) {
    for (const int shape : {0, 2}) {
      Scene2 sc;
      sc.shape = shape;
      sc.T_b_o = yaw_pose(mount_deg * kDeg, Vec3(-0.1, 0.05, 0.0));
      double nees = 0.0;
      int beyond_3s = 0;
      int aligned = 0;
      for (std::uint32_t seed = 1; seed <= 100; ++seed) {
        const Run2 r = run_scene(sc, cfg_shipped(), seed);
        if (r.f.fix < 0) {
          continue;
        }
        ++aligned;
        const double z = r.f.err_rad / r.f.sigma_rad;
        nees += z * z;
        beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
      }
      std::printf(
          "[ track yaw 2b ] mount yaw %.0f deg, %s: %d of 100 aligned, yaw NEES %.2f, %d "
          "beyond 3 sigma\n",
          mount_deg,
          shape == 0 ? "straight" : "1 m circle",
          aligned,
          aligned > 0 ? nees / aligned : kNaN,
          beyond_3s);
      EXPECT_EQ(aligned, 100) << mount_deg << " " << shape;
      EXPECT_LT(nees / aligned, 1.4) << mount_deg << " " << shape;
      EXPECT_LE(beyond_3s, 3) << mount_deg << " " << shape;
    }
  }
}

// Review N2 of design 2b round 1 (pre-existing in design 2): a GNSS outage over an S-jog (a turn
// and its opposite) cancels in the net heading change, and the turn gate passed it: aisles with an
// outage over 6-16 s aligned 100 of 100 runs beyond 3 sigma (median 26 deg off). The gate now uses
// the total heading variation between fixes. Outages over 6-16, 8-18 and 10-20 s, with and
// without wheel data, 100 seeds: at most the nominal rate of misses beyond 3 sigma; without the
// heading readings (negative control) 6-16 s still fails.
TEST(GnssTrackYaw2b, OutageOverAnSJogIsGated) {
  for (const double gap0_s : {6.0, 8.0, 10.0}) {
    for (const bool wheel : {true, false}) {
      for (const bool readings : {true, false}) {
        if (!readings && !(gap0_s == 6.0 && !wheel)) {
          continue;
        }
        Scene2 sc;
        sc.shape = 1;
        sc.gap0_s = gap0_s;
        sc.gap1_s = gap0_s + 10.0;
        sc.wheel = wheel;
        sc.heading_readings = readings;
        double nees = 0.0;
        int beyond_3s = 0;
        int aligned = 0;
        std::vector<double> err_deg;
        for (std::uint32_t seed = 1; seed <= 100; ++seed) {
          const Run2 r = run_scene(sc, cfg_shipped(), seed);
          if (r.f.fix < 0) {
            continue;
          }
          ++aligned;
          const double z = r.f.err_rad / r.f.sigma_rad;
          nees += z * z;
          beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
          err_deg.push_back(std::abs(r.f.err_rad) / kDeg);
        }
        std::sort(err_deg.begin(), err_deg.end());
        std::printf(
            "[ track yaw 2b ] aisles, outage %.0f-%.0f s, wheel %d, heading readings %d: "
            "%d of 100 aligned, median error %.1f deg, NEES %.2f, %d beyond 3 sigma\n",
            gap0_s,
            gap0_s + 10.0,
            wheel ? 1 : 0,
            readings ? 1 : 0,
            aligned,
            err_deg.empty() ? kNaN : err_deg[err_deg.size() / 2],
            aligned > 0 ? nees / aligned : kNaN,
            beyond_3s);
        ASSERT_EQ(aligned, 100);
        if (readings) {
          EXPECT_LE(beyond_3s, 3) << gap0_s << " " << wheel;
          EXPECT_LT(nees / aligned, 1.5) << gap0_s << " " << wheel;
        } else {
          EXPECT_GT(beyond_3s, 50);  // negative control: the net change alone is fooled
        }
      }
    }
  }
}

// Review N3 of design 2b round 1: wheels that turn on a still body for 30, 60 and 300 s, then the
// robot drives straight; GNSS white, or drifting by 0.05 m per fix while stuck. Before, the
// alignment waited for about the spun distance (300 s: 150 m) and the stuck drift stayed in Z
// (NEES up to 135). Now the segments since the last mismatch replace them after
// align_yaw_wheel_restart_m (10 m) of agreeing travel: aligned after at most about 16 m of
// driving (4.9 m without the stuck phase), yaw within its sigma; never while stuck.
TEST(GnssTrackYaw2b, StuckThenDriveAlignsAfterBoundedTravel) {
  for (const double stuck_s : {30.0, 60.0, 300.0}) {
    for (const double drift_m : {0.0, 0.05}) {
      Scene2 sc;
      sc.shape = 0;
      sc.stuck_s = stuck_s;
      sc.drift_m = drift_m;
      sc.tmax_s = stuck_s + 120.0;
      double nees = 0.0;
      int beyond_3s = 0;
      std::vector<double> driven;
      for (std::uint32_t seed = 1; seed <= 100; ++seed) {
        const Run2 r = run_scene(sc, cfg_shipped(), seed);
        ASSERT_GE(r.f.fix, 0) << stuck_s << " " << seed;
        EXPECT_GT(r.driven_m, 0.0) << "aligned while stuck";
        driven.push_back(r.driven_m);
        const double z = r.f.err_rad / r.f.sigma_rad;
        nees += z * z;
        beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
      }
      std::sort(driven.begin(), driven.end());
      std::printf(
          "[ track yaw 2b ] wheels spin %.0f s on a still body (drift %.2f m per fix), then "
          "drive: aligned after a median %.1f m (max %.1f m), NEES %.2f, %d beyond 3 "
          "sigma\n",
          stuck_s,
          drift_m,
          driven[50],
          driven.back(),
          nees / 100.0,
          beyond_3s);
      EXPECT_LT(driven.back(), 20.0);
      EXPECT_LT(nees / 100.0, 1.5);
      EXPECT_LE(beyond_3s, 3);
    }
  }
}

// Review m5 of design 2b round 1: the speed-profile variance of the steps shared by two adjacent
// wheel-path segments, sum var |w_f c_before + w_r c_after|^2 (gnss.md section 5.1), including its
// cross term 2 P_x c_before . c_after. A circle at 0.3 rad per fix (every segment through the
// wheel path) where the profile term dominates: fixes almost noise-free (1 cm, floor 1e-6 m^2),
// no slip (fraction 1e-6), and in every step the robot travels at a direction drawn uniformly in
// +-dpsi / 2 of the mid heading (the modelled unknown speed profile). The yaw NEES over 2000 runs
// is about 1; dropping the cross term makes the sigma optimistic.
TEST(GnssTrackYaw2b, ProfileVarianceOfSharedStepsIsConsistent) {
  GnssConfig c = cfg_shipped();
  c.align_yaw_wheel_slip_frac = 1e-6;
  c.align_yaw_min_var_m2 = 1e-6;
  c.align_yaw_min_track_wheel_m = 0.5;
  constexpr int kRuns = 2000;
  constexpr double kStepRad = 0.3;  // rad per fix
  constexpr double kStepM = 0.1;    // m per fix
  constexpr double kFixSigmaM = 0.01;
  std::mt19937 rng(31U);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(-0.5, 0.5);
  double nees = 0.0;
  int n = 0;
  for (int run = 0; run < kRuns; ++run) {
    GnssTrackYawAligner a(c);
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double odo = 0.0;
    for (int k = 0; k < 400; ++k) {
      const Pose b = yaw_pose(psi, Vec3(x, y, 0.0));
      Vec3 p = kTew * (b * kPbg);
      p.x() += kFixSigmaM * n01(rng);
      p.y() += kFixSigmaM * n01(rng);
      const auto r = a.update(p, fix_cov(kFixSigmaM), b, kPbg, odo, kStepRad * k);
      if (r.is_ok()) {
        const double err = wrap_pi(yaw_of(r.value().R) - yaw_of(kTew.R));
        nees += err * err / (a.yaw_sigma_rad() * a.yaw_sigma_rad());
        ++n;
        break;
      }
      const double dir = psi + 0.5 * kStepRad + kStepRad * u01(rng);
      x += kStepM * std::cos(dir);
      y += kStepM * std::sin(dir);
      psi += kStepRad;
      odo += kStepM;
    }
  }
  ASSERT_GT(n, kRuns * 9 / 10);
  std::printf("[ track yaw 2b ] speed-profile term, 0.3 rad per fix: yaw NEES %.2f over %d runs\n",
              nees / n,
              n);
  EXPECT_GT(nees / n, 0.8);
  EXPECT_LT(nees / n, 1.2);
}

// Item 5 of the design-2b brief: yaw from rotating in place, a Monte Carlo of the manoeuvre (no
// production code). The antenna (lever arm l, 0.22 m horizontal) circles the body origin once in
// 10 s; the yaw is the 2-D Procrustes fit of the known antenna circle in w to the fixes. First
// order: sigma_yaw = sigma / (|l| sqrt(N)) (the perpendicular noise over sum |q - mean|^2 = N
// |l|^2); the brief's sigma / (|l| sqrt(N / 2)) overstates it by sqrt(2). Measured RMS (2000 runs):
// 10 Hz, 0.5 m white: 12.9 deg (12.8 predicted); 5 Hz: 19.5 deg (18.1); RTK 0.02 m at 10 Hz:
// 0.50 deg; 0.3 m white + 0.4 m with a 5 s time constant: 46 deg.
TEST(GnssTrackYaw2b, InPlaceRotationYawObservability) {
  const double l_m = std::hypot(kPbg.x(), kPbg.y());
  const double l0 = std::atan2(kPbg.y(), kPbg.x());
  const double y0 = 0.5;  // rad, true yaw
  struct Case {
    const char* name;
    double rate_hz;
    double white_m;
    double gm_m;   // Gauss-Markov (first-order) error, steady 1-sigma per axis
    double tau_s;  // its time constant
  };
  constexpr int kRuns = 2000;
  for (const Case& c : {Case{"10 Hz, 0.5 m white", 10.0, 0.5, 0.0, 1.0},
                        Case{"5 Hz, 0.5 m white", 5.0, 0.5, 0.0, 1.0},
                        Case{"10 Hz, RTK 0.02 m white", 10.0, 0.02, 0.0, 1.0},
                        Case{"10 Hz, 0.3 m white + 0.4 m tau 5 s", 10.0, 0.3, 0.4, 5.0}}) {
    const int n = static_cast<int>(std::lround(10.0 * c.rate_hz));
    const double dt = 1.0 / c.rate_hz;
    const double phi = std::exp(-dt / c.tau_s);
    std::mt19937 rng(77U);
    std::normal_distribution<double> n01(0.0, 1.0);
    double sq = 0.0;
    for (int run = 0; run < kRuns; ++run) {
      std::vector<Eigen::Vector2d> q(static_cast<std::size_t>(n));
      std::vector<Eigen::Vector2d> p(static_cast<std::size_t>(n));
      Eigen::Vector2d gm(c.gm_m * n01(rng), c.gm_m * n01(rng));
      Eigen::Vector2d mq = Eigen::Vector2d::Zero();
      Eigen::Vector2d mp = Eigen::Vector2d::Zero();
      for (int i = 0; i < n; ++i) {
        const double psi = 2.0 * kPi * i / n + l0;
        const auto idx = static_cast<std::size_t>(i);
        q[idx] = Eigen::Vector2d(l_m * std::cos(psi), l_m * std::sin(psi));
        gm = phi * gm + c.gm_m * std::sqrt(1.0 - phi * phi) * Eigen::Vector2d(n01(rng), n01(rng));
        const Eigen::Vector2d rot(std::cos(y0) * q[idx].x() - std::sin(y0) * q[idx].y(),
                                  std::sin(y0) * q[idx].x() + std::cos(y0) * q[idx].y());
        p[idx] =
            rot + Eigen::Vector2d(5.0, -3.0) + gm + c.white_m * Eigen::Vector2d(n01(rng), n01(rng));
        mq += q[idx];
        mp += p[idx];
      }
      mq /= n;
      mp /= n;
      double sd = 0.0;
      double sc = 0.0;
      for (int i = 0; i < n; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const Eigen::Vector2d a = q[idx] - mq;
        const Eigen::Vector2d b = p[idx] - mp;
        sd += a.dot(b);
        sc += a.x() * b.y() - a.y() * b.x();
      }
      const double e = wrap_pi(std::atan2(sc, sd) - y0);
      sq += e * e;
    }
    const double rms_deg = std::sqrt(sq / kRuns) / kDeg;
    const double total_m = std::hypot(c.white_m, c.gm_m);
    const double pred_deg = total_m / (l_m * std::sqrt(static_cast<double>(n))) / kDeg;
    std::printf(
        "[ in-place rotation ] %s, %d fixes: RMS yaw error %.2f deg, white first order "
        "%.2f deg\n",
        c.name,
        n,
        rms_deg,
        pred_deg);
    if (c.gm_m == 0.0) {
      EXPECT_NEAR(rms_deg / pred_deg, 1.0, 0.15) << c.name;
    } else {
      EXPECT_GT(rms_deg, 2.0 * pred_deg) << c.name;
    }
  }
}

TEST(GnssTrackYaw2b, NoAllocationWithWheelTurns) {
  const std::vector<PathFix> path = integrate(aisles(20, 6));
  GnssTrackYawAligner a(cfg_2b(true));
  std::mt19937 rng(5U);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::vector<Vec3> fixes;
  for (const PathFix& f : path) {
    fixes.push_back(kTew * (f.body * kPbg) + kSigmaM * Vec3(n01(rng), n01(rng), 0.0));
  }
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (std::size_t k = 0; k < path.size(); ++k) {
      (void)a.update(fixes[k], fix_cov(kSigmaM), path[k].body, kPbg, path[k].odo_m);
    }
  }
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
  EXPECT_GT(a.n_wheel_turn_segments(), 0);
}

}  // namespace
