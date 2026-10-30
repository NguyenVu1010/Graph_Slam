// Root-cause study of the design-2b residual limits (gnss_fusion_study.md section 16.13): sweeps
// of each noise source against the failure rate and the distance to the first alignment, oracle
// ablations, the fixes, and a GNSS Doppler velocity estimate (study only, no core code). The
// aligner is driven directly (front-end pose = truth). The sweeps run only with
// MSS_GNSS_ROOTCAUSE_STUDY=1 and write results/<MSS_GNSS_ROOTCAUSE_RUN_ID>/gnss_rootcause.md; the
// GnssTrackYawRootCause tests of the fixes always run. Fixed seeds, 100 runs per point.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/types.hpp"
#include "mss/gnss/config.hpp"
#include "mss/gnss/gnss.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Pose;
using mss::common::Vec3;
using mss::gnss::GnssConfig;
using mss::gnss::GnssTrackYawAligner;

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

const Pose kTew = yaw_pose(30.0 * kDeg, Vec3(12.0, -7.0, 1.0));  // true T_e_w

// The design-2b keys as shipped before this study, plus the study's keys (off by default here).
GnssConfig cfg_base() {
  GnssConfig c;
  c.enabled = true;
  c.align_yaw_from_track = true;
  c.align_yaw_wheel_turns = true;
  c.align_yaw_max_sigma_rad = 6.0 * kDeg;
  c.align_yaw_min_track_wheel_m = 3.0;
  return c;
}

enum class Shape { kStraight, kCircle, kAisles, kBackward, kParked, kSlowS, kSpin };

struct Scene {
  Shape shape = Shape::kStraight;
  double speed_mps = 0.5;     // m/s while driving
  double yaw_rate_radps = 0;  // rad/s for kCircle
  double stuck_s = 0.0;       // s of wheels turning at speed_mps on a still body (1e9: for ever)
  double rest_s = 2.0;        // s at rest at the start
  double tmax_s = 300.0;      // s
  double white_m = 0.5;       // m, white GNSS noise per axis
  double declared_m = 0.5;    // m, declared sigma
  double drift_m = 0.0;       // m per fix, GNSS random walk per axis (whole run)
  bool wheel = true;
  double dropout = 0.0;  // probability of a fix without an odometer reading
  double dropout_after_s = 0.0;
  double skid_rad = 0.0;           // persistent sideslip while turning
  bool frontend = false;           // the LiDAR observes the travel axis (count passed as 0)
  std::vector<int> nan_fixes;      // fix indices without an odometer reading
  double doppler_sigma_mps = 0.0;  // > 0: also run the Doppler yaw estimate
};

struct RunResult {
  bool aligned = false;
  bool while_still = false;  // aligned while the body had not moved
  double err_rad = 0.0;
  double sigma_rad = 0.0;
  double driven_m = 0.0;  // m driven since the motion started, at the alignment
  double t_s = 0.0;
  int wheel_turn_segments = 0;  // at the alignment, or at the end
  int skipped_turns = 0;
};

// 1 kHz kinematics of the odometry frame (= body), fixes at 5 Hz.
RunResult run_scene(const Scene& sc, const GnssConfig& c, std::uint32_t seed) {
  std::mt19937 rng(9000U + seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  GnssTrackYawAligner a(c);
  double x = 0.0;
  double y = 0.0;
  double psi = 0.37 * seed;
  double odo = 0.0;
  double turn = 0.0;
  double driven = 0.0;
  Vec3 walk = Vec3::Zero();
  constexpr double kDt = 1e-3;
  constexpr int kPerFix = 200;
  const auto n_steps = static_cast<long>(sc.tmax_s / kDt);
  const double t_move = sc.rest_s + sc.stuck_s;
  for (long k = 0; k < n_steps; ++k) {
    const double t = static_cast<double>(k) * kDt;
    double v = 0.0;
    double w = 0.0;
    bool still = true;
    double wheel_v = 0.0;
    if (t >= sc.rest_s && t < t_move) {
      wheel_v = sc.speed_mps;  // wheels turn, body still
    } else if (t >= t_move && sc.shape != Shape::kParked) {
      still = false;
      v = sc.shape == Shape::kBackward ? -sc.speed_mps : sc.speed_mps;
      if (sc.shape == Shape::kCircle) {
        w = sc.yaw_rate_radps;
      } else if (sc.shape == Shape::kSlowS) {
        // Every 20 s: 5 s turning left, 5 s turning right, 10 s straight (net heading 0).
        const double tt = std::fmod(t - t_move, 20.0);
        w = tt < 5.0 ? sc.yaw_rate_radps : (tt < 10.0 ? -sc.yaw_rate_radps : 0.0);
      } else if (sc.shape == Shape::kSpin) {
        v = 0.0;  // turning in place: the wheels turn opposite, the odometer stays
        w = sc.yaw_rate_radps;
      } else if (sc.shape == Shape::kAisles) {
        const double tm = t - t_move;
        const double period = 4.0 + 1.6;
        const double tt = std::fmod(tm, period);
        const int leg = static_cast<int>(tm / period);
        if (tt >= 4.0) {
          v = 0.3;
          w = (leg % 2 != 0 ? -1.0 : 1.0) * (0.5 * kPi) / 1.6;
        }
      }
      wheel_v = v;
    }
    if (k % kPerFix == 0) {
      for (int i = 0; i < 2; ++i) {
        walk(i) += sc.drift_m * n01(rng);
      }
      const Pose b = yaw_pose(psi, Vec3(x, y, 0.0));
      Vec3 p = kTew * (b * kPbg) + walk;
      p.x() += sc.white_m * n01(rng);
      p.y() += sc.white_m * n01(rng);
      double reading = sc.wheel ? odo : kNaN;
      if (t >= sc.dropout_after_s && u01(rng) < sc.dropout) {
        reading = kNaN;
      }
      const int fix = static_cast<int>(k / kPerFix);
      if (std::find(sc.nan_fixes.begin(), sc.nan_fixes.end(), fix) != sc.nan_fixes.end()) {
        reading = kNaN;
      }
      const auto r = a.update(p,
                              Mat3::Identity() * sc.declared_m * sc.declared_m,
                              b,
                              kPbg,
                              reading,
                              turn,
                              sc.frontend ? 0.0 : kNaN);
      if (r.is_ok()) {
        RunResult out;
        out.aligned = true;
        out.while_still = still && !(driven > 0.0);
        const double yaw = std::atan2(r.value().R(1, 0), r.value().R(0, 0));
        out.err_rad = wrap_pi(yaw - 30.0 * kDeg);
        out.sigma_rad = a.yaw_sigma_rad();
        out.driven_m = driven;
        out.t_s = t;
        out.wheel_turn_segments = a.n_wheel_turn_segments();
        out.skipped_turns = a.n_skipped_turns();
        return out;
      }
    }
    const double off = w != 0.0 ? sc.skid_rad : 0.0;
    x += v * std::cos(psi + 0.5 * w * kDt + off) * kDt;
    y += v * std::sin(psi + 0.5 * w * kDt + off) * kDt;
    driven += std::abs(v) * kDt;
    psi += w * kDt;
    turn += std::abs(w) * kDt;
    odo += wheel_v * kDt;
  }
  RunResult out;
  out.wheel_turn_segments = a.n_wheel_turn_segments();
  out.skipped_turns = a.n_skipped_turns();
  return out;
}

struct Stats {
  int runs = 0;
  int aligned = 0;
  int while_still = 0;
  int beyond_3s = 0;
  int beyond_15deg = 0;
  int flips = 0;       // more than 90 deg off
  int within_10m = 0;  // aligned within 10 m of driving
  double nees = 0.0;
  double max_err_deg = 0.0;
  std::vector<double> driven;
  double median_driven() const {
    if (driven.empty()) {
      return kNaN;
    }
    std::vector<double> d = driven;
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
  }
  double max_driven() const {
    return driven.empty() ? kNaN : *std::max_element(driven.begin(), driven.end());
  }
};

Stats sweep(const Scene& sc, const GnssConfig& c) {
  Stats s;
  for (std::uint32_t seed = 1; seed <= kRuns; ++seed) {
    ++s.runs;
    const RunResult r = run_scene(sc, c, seed);
    if (!r.aligned) {
      continue;
    }
    ++s.aligned;
    s.while_still += r.while_still ? 1 : 0;
    const double z = r.err_rad / r.sigma_rad;
    s.nees += z * z;
    s.beyond_3s += std::abs(z) > 3.0 ? 1 : 0;
    s.beyond_15deg += std::abs(r.err_rad) > 15.0 * kDeg ? 1 : 0;
    s.flips += std::abs(r.err_rad) > 0.5 * kPi ? 1 : 0;
    s.max_err_deg = std::max(s.max_err_deg, std::abs(r.err_rad) / kDeg);
    if (!r.while_still) {
      s.driven.push_back(r.driven_m);
      s.within_10m += r.driven_m <= 10.0 ? 1 : 0;
    }
  }
  if (s.aligned > 0) {
    s.nees /= s.aligned;
  }
  return s;
}

class Report {
 public:
  explicit Report(bool append = false) {
    const char* id = std::getenv("MSS_GNSS_ROOTCAUSE_RUN_ID");
    const std::filesystem::path dir =
        std::filesystem::path{MSS_REPO_DIR} / "results" / (id != nullptr ? id : "gnss-rootcause");
    std::filesystem::create_directories(dir);
    out_.open(dir / "gnss_rootcause.md", append ? std::ios::app : std::ios::trunc);
  }
  void line(const std::string& s) {
    out_ << s << "\n";
    std::printf("%s\n", s.c_str());
    std::fflush(stdout);
  }
  void header() {
    line(
        "| point | variant | aligned | on a still body | beyond 3 sigma | > 15 deg | flips | "
        "NEES | driven median (max), m | within 10 m |");
    line("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |");
  }
  void row(const std::string& point, const std::string& variant, const Stats& s) {
    char buf[320];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %s | %d | %d | %d | %d | %d | %.2f | %.1f (%.1f) | %d |",
                  point.c_str(),
                  variant.c_str(),
                  s.aligned,
                  s.while_still,
                  s.beyond_3s,
                  s.beyond_15deg,
                  s.flips,
                  s.nees,
                  s.median_driven(),
                  s.max_driven(),
                  s.within_10m);
    line(buf);
  }

 private:
  std::ofstream out_;
};

std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

bool study_enabled() {
  const char* on = std::getenv("MSS_GNSS_ROOTCAUSE_STUDY");
  return on != nullptr && std::string(on) == "1";
}

// The study's fixes on top of the base keys.
GnssConfig cfg_fixed(int gap, int merge, bool frontend) {
  GnssConfig c = cfg_base();
  c.align_yaw_wheel_max_gap_fixes = gap;
  c.align_yaw_merge_max_chunks = merge;
  c.align_yaw_frontend_travel = frontend;
  return c;
}

// The study's choice (config/default.yaml): bridge 4 fixes, extend over 10 chunks; the front end
// as given.
GnssConfig cfg_after(bool frontend) {
  return cfg_fixed(4, 10, frontend);
}

TEST(GnssRootCauseStudy, Sweeps) {
  if (!study_enabled()) {
    GTEST_SKIP() << "set MSS_GNSS_ROOTCAUSE_STUDY=1";
  }
  Report rep;
  rep.line("# GNSS track-yaw root-cause study (gnss_fusion_study.md section 16.13)");
  rep.line("");
  rep.line("100 seeds per point; 5 Hz fixes; lever arm 0.22 m; front-end pose = truth.");

  // 1. Stuck wheels: wheels at 0.5 m/s on a still body for ever, 0.3 m white + drift.
  rep.line("");
  rep.line("## 1. Stuck wheels, 300 s, 0.3 m white, declared 0.5 m");
  rep.header();
  for (const double drift : {0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.50}) {
    Scene sc;
    sc.shape = Shape::kParked;
    sc.stuck_s = 1e9;
    sc.white_m = 0.3;
    sc.drift_m = drift;
    const std::string pt = fmt("drift %.2f m/fix", drift);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    rep.row(pt, "after, axis not observed", sweep(sc, cfg_after(true)));
    sc.frontend = true;
    rep.row(pt, "after, axis observed", sweep(sc, cfg_after(true)));
  }

  // 2. Missing wheel readings after a mismatch: stuck 60 s, then straight at 0.5 m/s.
  rep.line("");
  rep.line("## 2. Spin 60 s, then drive; readings missing at random after the drive-off");
  rep.header();
  for (const double p : {0.0, 0.01, 0.02, 0.05, 0.10, 0.20}) {
    Scene sc;
    sc.stuck_s = 60.0;
    sc.tmax_s = 700.0;
    sc.dropout = p;
    sc.dropout_after_s = 62.0;
    const std::string pt = fmt("%.0f %% missing", 100.0 * p);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    for (const int gap : {1, 2, 4}) {
      rep.row(pt, fmt("bridge %.0f", gap), sweep(sc, cfg_fixed(gap, 0, false)));
    }
  }
  rep.line("");
  rep.line(
      "Protection: wheels spinning for ever, 0.20 m per fix of drift, readings missing from "
      "the start");
  rep.header();
  for (const double p : {0.0, 0.02, 0.05, 0.10, 0.20}) {
    Scene sc;
    sc.shape = Shape::kParked;
    sc.stuck_s = 1e9;
    sc.white_m = 0.3;
    sc.drift_m = 0.20;
    sc.dropout = p;
    const std::string pt = fmt("%.0f %% missing", 100.0 * p);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    for (const int gap : {1, 2, 4}) {
      rep.row(pt, fmt("bridge %.0f", gap), sweep(sc, cfg_fixed(gap, 0, false)));
    }
  }

  // 3. Slow robot with wheel data, straight, 0.5 m white.
  rep.line("");
  rep.line("## 3. Slow robot with wheel data, straight, 0.5 m white, 900 s");
  rep.header();
  for (const double v : {0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.50}) {
    Scene sc;
    sc.speed_mps = v;
    sc.tmax_s = 900.0;
    const std::string pt = fmt("%.2f m/s", v);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    GnssConfig oracle = cfg_base();
    oracle.align_yaw_min_wheel_travel_m = 1e-6;  // the true travel sign, no stationary skip
    rep.row(pt, "oracle: true travel", sweep(sc, oracle));
    for (const int merge : {5, 10, 20}) {
      rep.row(pt, fmt("extend %.0f chunks", merge), sweep(sc, cfg_fixed(0, merge, false)));
    }
  }
  rep.line("");
  rep.line("Slow robot with GNSS drift 0.02 m per fix (coloured error over the long run):");
  rep.header();
  for (const double v : {0.03, 0.05, 0.10}) {
    Scene sc;
    sc.speed_mps = v;
    sc.tmax_s = 900.0;
    sc.white_m = 0.3;
    sc.drift_m = 0.02;
    const std::string pt = fmt("%.2f m/s, drift 0.02", v);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    rep.row(pt, "extend 10 chunks", sweep(sc, cfg_fixed(0, 10, false)));
  }

  rep.line("");
  rep.line("Speed x GNSS drift grid (0.3 m white, declared 0.5 m), after:");
  rep.header();
  for (const double v : {0.03, 0.05, 0.10, 0.20, 0.50}) {
    for (const double drift : {0.0, 0.005, 0.01, 0.02, 0.05}) {
      Scene sc;
      sc.speed_mps = v;
      sc.tmax_s = 900.0;
      sc.white_m = 0.3;
      sc.drift_m = drift;
      rep.row(
          fmt("%.2f m/s, ", v) + fmt("drift %.3f", drift), "after", sweep(sc, cfg_after(false)));
    }
  }

  // 4. Without wheel data.
  rep.line("");
  rep.line("## 4a. No wheel data, driving backwards at 0.5 m/s, 0.5 m white");
  rep.header();
  {
    Scene sc;
    sc.shape = Shape::kBackward;
    sc.wheel = false;
    rep.row("backwards", "before", sweep(sc, cfg_base()));
    rep.row("backwards", "after, axis not observed", sweep(sc, cfg_after(true)));
    sc.frontend = true;
    rep.row("backwards", "after, axis observed", sweep(sc, cfg_after(true)));
  }
  rep.line("");
  rep.line("## 4b. No wheel data, parked, 0.3 m white, 300 s");
  rep.header();
  for (const double drift : {0.05, 0.10, 0.15, 0.20, 0.30}) {
    Scene sc;
    sc.shape = Shape::kParked;
    sc.wheel = false;
    sc.white_m = 0.3;
    sc.drift_m = drift;
    const std::string pt = fmt("drift %.2f m/fix", drift);
    rep.row(pt, "before", sweep(sc, cfg_base()));
    rep.row(pt, "after, axis not observed", sweep(sc, cfg_after(true)));
    sc.frontend = true;
    rep.row(pt, "after, axis observed", sweep(sc, cfg_after(true)));
  }

  // 5. Persistent skid on a 1 m circle (0.5 m/s, 0.5 rad/s).
  rep.line("");
  rep.line("## 5. Persistent skid while turning, 0.5 m white");
  rep.header();
  for (const double skid_deg : {0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0}) {
    Scene sc;
    sc.shape = Shape::kCircle;
    sc.yaw_rate_radps = 0.5;
    sc.skid_rad = skid_deg * kDeg;
    rep.row(fmt("circle, skid %.0f deg", skid_deg), "before", sweep(sc, cfg_base()));
    rep.row(fmt("circle, skid %.0f deg", skid_deg),
            "after, axis not observed",
            sweep(sc, cfg_after(true)));
    sc.frontend = true;
    rep.row(
        fmt("circle, skid %.0f deg", skid_deg), "after, axis observed", sweep(sc, cfg_after(true)));
    sc.frontend = false;
    sc.shape = Shape::kAisles;
    rep.row(fmt("aisles, skid %.0f deg", skid_deg), "before", sweep(sc, cfg_base()));
    rep.row(fmt("aisles, skid %.0f deg", skid_deg),
            "after, axis not observed",
            sweep(sc, cfg_after(true)));
  }

  // 6. White GNSS sigma and driving drift (envelope), straight 0.5 m/s with wheel data.
  rep.line("");
  rep.line("## 6. White GNSS sigma (declared = true), straight 0.5 m/s with wheel data");
  rep.header();
  for (const double sigma : {0.1, 0.3, 0.5, 1.0, 1.5, 2.0, 3.0}) {
    Scene sc;
    sc.white_m = sigma;
    sc.declared_m = sigma;
    rep.row(fmt("sigma %.1f m", sigma), "before", sweep(sc, cfg_base()));
    rep.row(fmt("sigma %.1f m", sigma), "after", sweep(sc, cfg_after(false)));
  }
  rep.line("");
  rep.line("Coloured drift while driving (0.3 m white, declared 0.5 m), straight 0.5 m/s:");
  rep.header();
  for (const double drift : {0.0, 0.01, 0.02, 0.05, 0.10}) {
    Scene sc;
    sc.white_m = 0.3;
    sc.drift_m = drift;
    rep.row(fmt("drift %.2f m/fix", drift), "before", sweep(sc, cfg_base()));
    rep.row(fmt("drift %.2f m/fix", drift), "after", sweep(sc, cfg_after(false)));
  }
}

// GNSS Doppler velocity (study only): per fix the antenna velocity in e with white noise
// sigma_d per axis. Yaw from Z_D = sum conj(u_k) v_k (u_k the travel-axis unit in w times the
// wheel sign, v_k the Doppler velocity in e, complex notation), sigma_yaw = sigma_d sqrt(N) / |Z_D|
// at first order; aligned when sigma_yaw <= 6 deg. Stationarity: a chunk mean speed below
// 3 sigma_d / sqrt(10) is still.
TEST(GnssRootCauseStudy, Doppler) {
  if (!study_enabled()) {
    GTEST_SKIP() << "set MSS_GNSS_ROOTCAUSE_STUDY=1";
  }
  Report rep(true);
  rep.line("");
  rep.line("## 7. GNSS Doppler velocity, 0.05 m/s per axis, 5 Hz, straight, forward");
  rep.line(
      "| speed, m/s | aligned | fixes to 6 deg, median | distance, median m | beyond 3 sigma "
      "| NEES |");
  rep.line("| --- | --- | --- | --- | --- | --- |");
  constexpr double kSigmaD = 0.05;
  for (const double v : {0.02, 0.05, 0.10, 0.20, 0.50}) {
    int aligned = 0;
    int beyond = 0;
    double nees = 0.0;
    std::vector<double> fixes;
    for (std::uint32_t seed = 1; seed <= kRuns; ++seed) {
      std::mt19937 rng(11000U + seed);
      std::normal_distribution<double> n01(0.0, 1.0);
      const double psi = 0.37 * seed;
      double zr = 0.0;
      double zi = 0.0;
      for (int k = 1; k <= 20000; ++k) {
        // velocity in e: R_e_w (v u_w), u_w = (cos psi, sin psi)
        const double ve_x = v * std::cos(psi + 30.0 * kDeg) + kSigmaD * n01(rng);
        const double ve_y = v * std::sin(psi + 30.0 * kDeg) + kSigmaD * n01(rng);
        // conj(u) * v_e
        zr += std::cos(psi) * ve_x + std::sin(psi) * ve_y;
        zi += std::cos(psi) * ve_y - std::sin(psi) * ve_x;
        const double z = std::hypot(zr, zi);
        const double sigma = kSigmaD * std::sqrt(static_cast<double>(k)) / z;
        if (k >= 10 && sigma <= 6.0 * kDeg) {
          ++aligned;
          const double err = wrap_pi(std::atan2(zi, zr) - 30.0 * kDeg);
          nees += (err / sigma) * (err / sigma);
          beyond += std::abs(err) > 3.0 * sigma ? 1 : 0;
          fixes.push_back(k);
          break;
        }
      }
    }
    std::sort(fixes.begin(), fixes.end());
    const double med = fixes.empty() ? kNaN : fixes[fixes.size() / 2];
    char buf[200];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %.2f | %d | %.0f | %.2f | %d | %.2f |",
                  v,
                  aligned,
                  med,
                  med * v / 5.0,
                  beyond,
                  aligned > 0 ? nees / aligned : kNaN);
    rep.line(buf);
  }
  rep.line("");
  rep.line(
      "Stationarity from Doppler (chunk mean of 10 fixes, still below 3 sigma of the mean "
      "speed): a still body is flagged moving in this fraction of chunks; a moving body "
      "is flagged still in this fraction:");
  {
    const double thr = 3.0 * kSigmaD / std::sqrt(10.0) * std::sqrt(2.0);
    for (const double v : {0.0, 0.02, 0.05, 0.10}) {
      std::mt19937 rng(12000U);
      std::normal_distribution<double> n01(0.0, 1.0);
      int moving = 0;
      constexpr int kChunks = 10000;
      for (int c = 0; c < kChunks; ++c) {
        double sx = 0.0;
        double sy = 0.0;
        for (int i = 0; i < 10; ++i) {
          sx += v + kSigmaD * n01(rng);
          sy += kSigmaD * n01(rng);
        }
        moving += std::hypot(sx / 10.0, sy / 10.0) > thr ? 1 : 0;
      }
      rep.line(fmt("- %.2f m/s: ", v) +
               fmt("%.4f of chunks flagged moving", static_cast<double>(moving) / kChunks) +
               fmt(" (threshold %.3f m/s)", thr));
    }
  }
}

// ---------------------------------------------------------------------------------------------
// Unit tests of the study's fixes (always run; gnss_fusion_study.md section 16.13). The numbers in
// the comments are the study's, 100 seeds.

// Limit 2: after wheels spun on a still body (a wheel/GNSS mismatch), readings missing at random
// while driving broke the window at every gap (2 %: median 36 m, max 311 m; 10 %: none of 100
// aligned in 640 s). Bridging gaps of up to 4 fixes: median 10.9 m, max 16.9 / 15.9 m.
TEST(GnssTrackYawRootCause, BridgedReadingsRestartAfterBoundedTravel) {
  for (const double p : {0.02, 0.10}) {
    Scene sc;
    sc.stuck_s = 60.0;
    sc.tmax_s = 400.0;
    sc.dropout = p;
    sc.dropout_after_s = 62.0;
    const Stats before = sweep(sc, cfg_base());
    const Stats after = sweep(sc, cfg_fixed(4, 0, false));
    std::printf(
        "[ root cause ] spin 60 s, %.0f %% readings missing: before %d aligned, median "
        "%.1f m; bridged %d aligned, median %.1f m (max %.1f m), NEES %.2f, %d beyond 3 "
        "sigma\n",
        100.0 * p,
        before.aligned,
        before.median_driven(),
        after.aligned,
        after.median_driven(),
        after.max_driven(),
        after.nees,
        after.beyond_3s);
    EXPECT_EQ(after.aligned, kRuns) << p;
    EXPECT_LT(after.median_driven(), 12.0) << p;
    EXPECT_LT(after.max_driven(), 25.0) << p;
    EXPECT_LT(after.nees, 1.5) << p;
    EXPECT_LE(after.beyond_3s, 2) << p;
    // negative control: before, the window broke at every gap
    EXPECT_TRUE(before.aligned < kRuns || before.median_driven() > 20.0) << p;
  }
}

// Limit 2, protection: bridging does not hide wheels spinning on a still body. Wheels for ever,
// 0.20 m per fix of GNSS drift: 6 of 100 align on noise with every reading, and the same 6 with
// 10 % missing when bridged (before, missing readings raised it to 11); 0.10 m per fix: none.
TEST(GnssTrackYawRootCause, BridgingKeepsTheStuckCheck) {
  for (const double drift : {0.10, 0.20}) {
    Scene sc;
    sc.shape = Shape::kParked;
    sc.stuck_s = 1e9;
    sc.white_m = 0.3;
    sc.drift_m = drift;
    const Stats all = sweep(sc, cfg_fixed(4, 0, false));
    sc.dropout = 0.10;
    const Stats missing = sweep(sc, cfg_fixed(4, 0, false));
    std::printf(
        "[ root cause ] wheels spin for ever, drift %.2f m per fix: %d aligned with every "
        "reading, %d with 10 %% missing (bridged)\n",
        drift,
        all.aligned,
        missing.aligned);
    EXPECT_LE(missing.aligned, all.aligned) << drift;
    if (drift < 0.15) {
      EXPECT_EQ(missing.aligned, 0);
    }
  }
}

// Bridging is bounded: a run of align_yaw_wheel_max_gap_fixes (4) fixes without a reading keeps
// the wheel data of every segment, also across a chunk boundary; a longer run leaves every chunk
// half it touched without wheel data, so exactly the segments using them take the forward
// assumption (chunks of 10 fixes, segment c joins the second half of chunk c and the first half
// of chunk c + 1): fixes 23-27, both halves of chunk 2: 2 segments; fixes 26-34, the second half
// of chunk 2 and the whole of chunk 3: 2 segments; fixes 27-31, from the second half of chunk 2
// into the first half of chunk 3 (overflowing there): only segment 2 (review F6, cross-chunk half
// marking).
TEST(GnssTrackYawRootCause, GapLongerThanTheBoundIsNotBridged) {
  struct Case {
    int start;
    int len;
    int no_wheel;
  };
  for (const Case& cs :
       {Case{23, 4, 0}, Case{27, 4, 0}, Case{23, 5, 2}, Case{26, 9, 2}, Case{27, 5, 1}}) {
    {
      const int start = cs.start;
      const int len = cs.len;
      GnssConfig c = cfg_fixed(4, 0, false);
      GnssTrackYawAligner a(c);
      double odo = 0.0;
      for (int k = 0; k < 120; ++k) {
        const Pose b = yaw_pose(0.0, Vec3(0.1 * k, 0.0, 0.0));
        const bool missing = k >= start && k < start + len;
        (void)a.update(
            kTew * (b * kPbg), Mat3::Identity() * 0.25, b, kPbg, missing ? kNaN : odo, 0.0);
        odo += 0.1;
      }
      EXPECT_EQ(a.n_no_wheel_segments(), cs.no_wheel) << start << " " << len;
    }
  }
}

// Limit 3: with wheel data a robot slower than about 0.1 m/s never aligned: each 2 s segment had
// less than align_yaw_min_wheel_travel_m of travel and was skipped, and with the true travel sign
// (oracle) the short noisy segments failed the coherence test. Extending a stationary segment over
// up to 10 chunks: 0.03 / 0.05 m/s align after a median 3.1 / 3.3 m, within their sigma.
TEST(GnssTrackYawRootCause, ExtendedSegmentsAlignASlowRobot) {
  for (const double v : {0.03, 0.05}) {
    Scene sc;
    sc.speed_mps = v;
    sc.tmax_s = 400.0;
    const Stats before = sweep(sc, cfg_base());
    const Stats after = sweep(sc, cfg_fixed(0, 10, false));
    std::printf(
        "[ root cause ] %.2f m/s: before %d aligned; extended %d aligned, median %.1f m "
        "(max %.1f m), NEES %.2f, %d beyond 3 sigma\n",
        v,
        before.aligned,
        after.aligned,
        after.median_driven(),
        after.max_driven(),
        after.nees,
        after.beyond_3s);
    EXPECT_EQ(before.aligned, 0) << v;  // negative control
    EXPECT_EQ(after.aligned, kRuns) << v;
    EXPECT_GE(after.within_10m, kRuns - 1) << v;
    EXPECT_LT(after.nees, 1.5) << v;
    EXPECT_LE(after.beyond_3s, 2) << v;
  }
}

// The extension is bounded and only joins stationary segments: a parked robot with wheel data
// (odometer constant) and 0.30 m per fix of GNSS drift never aligns in 300 s.
TEST(GnssTrackYawRootCause, ExtensionDoesNotAlignAParkedRobot) {
  Scene sc;
  sc.shape = Shape::kParked;
  sc.white_m = 0.3;
  sc.drift_m = 0.30;
  const Stats s = sweep(sc, cfg_fixed(0, 10, false));
  EXPECT_EQ(s.aligned, 0);
}

// Limits 1 and 4b: a still body (wheels spinning on it, or no wheel data at all) aligned on GNSS
// drift (wheels: 6 / 12 of 100 at 0.20 / 0.50 m per fix; no wheel data: 25 / 44 at 0.20 /
// 0.30). Where the LiDAR observes the travel axis the front-end displacement makes the segments
// stationary: none of 100. Where it does not (no count), the old rates stay (control).
TEST(GnssTrackYawRootCause, FrontEndStillBodyDoesNotAlign) {
  for (const bool wheel : {true, false}) {
    for (const double drift : {0.20, 0.50}) {
      Scene sc;
      sc.shape = Shape::kParked;
      sc.stuck_s = wheel ? 1e9 : 0.0;
      sc.wheel = wheel;
      sc.white_m = 0.3;
      sc.drift_m = drift;
      const Stats blind = sweep(sc, cfg_after(true));
      sc.frontend = true;
      const Stats seen = sweep(sc, cfg_after(true));
      std::printf(
          "[ root cause ] still body, wheel data %d, drift %.2f m per fix: %d of 100 "
          "aligned on noise without the front end, %d with it\n",
          wheel ? 1 : 0,
          drift,
          blind.aligned,
          seen.aligned);
      EXPECT_EQ(seen.aligned, 0) << wheel << " " << drift;
      EXPECT_GT(blind.aligned, 0) << wheel << " " << drift;  // negative control
    }
  }
}

// A step whose count of unobserved scans grew does not observe the travel axis: with a count that
// grows at every fix the front end is not used (the same as no count).
TEST(GnssTrackYawRootCause, UnobservedStepsDoNotUseTheFrontEnd) {
  GnssConfig c = cfg_after(true);
  int aligned_growing = 0;
  int aligned_none = 0;
  for (std::uint32_t seed = 1; seed <= kRuns; ++seed) {
    for (const bool growing : {true, false}) {
      std::mt19937 rng(9000U + seed);
      std::normal_distribution<double> n01(0.0, 1.0);
      GnssTrackYawAligner a(c);
      const Pose b = yaw_pose(0.37 * seed, Vec3(3.0, -2.0, 0.0));
      Vec3 walk = Vec3::Zero();
      for (int k = 0; k < 1500; ++k) {
        Vec3 p = kTew * (b * kPbg);
        for (int i = 0; i < 2; ++i) {
          walk(i) += 0.30 * n01(rng);
          p(i) += walk(i) + 0.3 * n01(rng);
        }
        const double count = growing ? static_cast<double>(k) : kNaN;
        if (a.update(p, Mat3::Identity() * 0.25, b, kPbg, kNaN, 0.0, count).is_ok()) {
          (growing ? aligned_growing : aligned_none) += 1;
          break;
        }
      }
    }
  }
  EXPECT_EQ(aligned_growing, aligned_none);
  EXPECT_GT(aligned_none, 0);
}

// Limit 4a: without wheel data, net backward motion flipped the yaw (100 of 100 at 0.5 m/s).
// With the travel axis observed the front-end displacement gives the sign: no flip.
TEST(GnssTrackYawRootCause, FrontEndGivesTheTravelSign) {
  Scene sc;
  sc.shape = Shape::kBackward;
  sc.wheel = false;
  const Stats blind = sweep(sc, cfg_after(true));
  sc.frontend = true;
  const Stats seen = sweep(sc, cfg_after(true));
  std::printf(
      "[ root cause ] backwards without wheel data: %d flips without the front end, %d "
      "with it (NEES %.2f)\n",
      blind.flips,
      seen.flips,
      seen.nees);
  EXPECT_EQ(blind.flips, kRuns);  // negative control: the forward assumption
  EXPECT_EQ(seen.aligned, kRuns);
  EXPECT_EQ(seen.flips, 0);
  EXPECT_LT(seen.nees, 1.5);
}

// Limit 5: a skid held through a one-way turn is outside the wheel model (gyro and odometer do
// not see the sideslip): 10 deg on a 1 m circle, 9 of 100 beyond 3 sigma. With the travel axis
// observed the turn-gated segments take the front-end path: none.
TEST(GnssTrackYawRootCause, FrontEndPathInTurnsBoundsTheSkid) {
  Scene sc;
  sc.shape = Shape::kCircle;
  sc.yaw_rate_radps = 0.5;
  sc.skid_rad = 10.0 * kDeg;
  const Stats blind = sweep(sc, cfg_after(true));
  sc.frontend = true;
  const Stats seen = sweep(sc, cfg_after(true));
  std::printf(
      "[ root cause ] 1 m circle, 10 deg skid: %d beyond 3 sigma with the wheel path, %d "
      "with the front-end path (NEES %.2f)\n",
      blind.beyond_3s,
      seen.beyond_3s,
      seen.nees);
  EXPECT_GE(blind.beyond_3s, 5);  // negative control
  EXPECT_EQ(seen.aligned, kRuns);
  EXPECT_LE(seen.beyond_3s, 1);
  EXPECT_LT(seen.nees, 1.5);
}

// A bridged step is not used in the wheel path: its arc spans the gap, and the step lands on one
// fix. Noise-free 1 m circle (every segment through the wheel path), readings missing at the
// first fix of three chunks (the next reading bridges them): the yaw stays exact because the
// chunks holding a gap take no path; through the path it would be off by the step misplacement.
TEST(GnssTrackYawRootCause, BridgedStepIsNotInTheWheelPath) {
  Scene sc;
  sc.shape = Shape::kCircle;
  sc.yaw_rate_radps = 0.5;
  sc.white_m = 0.0;
  sc.declared_m = 0.5;
  sc.nan_fixes = {30, 50, 70};
  const RunResult r = run_scene(sc, cfg_after(false), 1);
  ASSERT_TRUE(r.aligned);
  EXPECT_LT(std::abs(r.err_rad), 1e-6);
}

// An extended segment takes its direction from the path of o between the chunk means, not from
// the mean heading of its two end chunks, which a turn at one end biases: slow S-jogs (57 deg
// each way at 0.2 rad/s and 0.03 m/s, net heading 0), noise-free fixes declared at 1 cm (floor
// lowered): the first yaw is exact (from the mean heading it was 4.8 deg off).
TEST(GnssTrackYawRootCause, ExtendedSegmentTakesThePathDirection) {
  Scene sc;
  sc.shape = Shape::kSlowS;
  sc.speed_mps = 0.03;
  sc.yaw_rate_radps = 0.2;
  sc.white_m = 0.0;
  sc.declared_m = 0.01;
  sc.tmax_s = 600.0;
  GnssConfig c = cfg_after(false);
  c.align_yaw_min_var_m2 = 1e-4;
  const RunResult r = run_scene(sc, c, 1);
  std::printf("[ root cause ] slow S-jogs at 0.03 m/s, noise-free: aligned %d, error %.4f deg\n",
              r.aligned ? 1 : 0,
              r.err_rad / kDeg);
  ASSERT_TRUE(r.aligned);
  EXPECT_LT(std::abs(r.err_rad), 0.01 * kDeg);
  EXPECT_GT(r.wheel_turn_segments, 0);
  // Readings missing inside the spans (bridged): an extended segment whose chunks hold a gap has no
  // wheel path (the path skips the gap) and is skipped, so the yaw stays exact.
  sc.nan_fixes = {103, 157, 211, 265, 319, 373, 427};
  const RunResult g = run_scene(sc, c, 1);
  std::printf("[ root cause ] the same with missing readings: aligned %d, error %.4f deg\n",
              g.aligned ? 1 : 0,
              g.err_rad / kDeg);
  ASSERT_TRUE(g.aligned);
  EXPECT_LT(std::abs(g.err_rad), 0.01 * kDeg);
}

// A stationary segment is extended only while a path of o will give its direction: with readings
// missing in every chunk of a 10 s rest (no wheel path there) and no front end, it is added as a
// stationary segment as before, so the run equals the one without the extension (E-01 with the flag
// on lost a segment of track before this rule).
TEST(GnssTrackYawRootCause, NoExtensionWithoutAPath) {
  Scene sc;
  sc.rest_s = 10.0;
  sc.nan_fixes = {3, 13, 23, 33, 43};
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    const RunResult with = run_scene(sc, cfg_after(false), seed);
    const RunResult without = run_scene(sc, cfg_fixed(4, 0, false), seed);
    ASSERT_TRUE(with.aligned && without.aligned) << seed;
    EXPECT_EQ(with.t_s, without.t_s) << seed;
    EXPECT_EQ(with.err_rad, without.err_rad) << seed;
  }
}

// Without wheel data, where the LiDAR observes the travel axis, the front end both extends a slow
// robot's segments (its displacement is below the travel threshold) and gives their direction
// (its path): 0.03 m/s straight, no wheel data, 0.5 m white: every run aligns within its sigma.
TEST(GnssTrackYawRootCause, FrontEndExtendsASlowRobotWithoutWheels) {
  Scene sc;
  sc.speed_mps = 0.03;
  sc.wheel = false;
  sc.frontend = true;
  sc.tmax_s = 600.0;
  const Stats s = sweep(sc, cfg_after(true));
  std::printf(
      "[ root cause ] 0.03 m/s without wheel data, front end observed: %d aligned, median "
      "%.1f m, NEES %.2f, %d beyond 3 sigma\n",
      s.aligned,
      s.median_driven(),
      s.nees,
      s.beyond_3s);
  EXPECT_EQ(s.aligned, kRuns);
  EXPECT_LT(s.nees, 1.5);
  EXPECT_LE(s.beyond_3s, 2);
}

// The front end is used only when every step of both chunks observed the travel axis: an
// unobserved jump of the front end inside the earlier chunk (a degenerate scan) must not make
// the segment look like travel. Parked, no wheel data, exact GNSS; the front end jumps 1 m at fix
// 13 (first half of chunk 1) in an unobserved step: segments 0-1 and 1-2 are not observed and
// take the forward assumption (2), every later one is observed and stationary.
TEST(GnssTrackYawRootCause, FrontEndNeedsBothChunksObserved) {
  GnssTrackYawAligner a(cfg_after(true));
  double unobserved = 0.0;
  for (int k = 0; k < 60; ++k) {
    const Pose b = yaw_pose(0.3, Vec3(k >= 13 ? 1.0 : 0.0, 0.0, 0.0));
    if (k == 13) {
      unobserved += 1.0;
    }
    (void)a.update(kTew * (yaw_pose(0.3, Vec3::Zero()) * kPbg),
                   Mat3::Identity() * 0.25,
                   b,
                   kPbg,
                   kNaN,
                   0.0,
                   unobserved);
  }
  EXPECT_EQ(a.n_no_wheel_segments(), 2);
  EXPECT_EQ(a.n_stationary_segments(), 3);
}

// Backlog G3 (review F4): a front end that observes the travel axis but does not move (its LiDAR
// sees surroundings moving with the robot) holds every segment still, so a robot driving at
// 0.5 m/s with wheel data never aligns; each such segment is counted in
// n_frontend_wheel_disagreements(). The exact front end at 0.5 m/s, and at 0.06 m/s where its
// displacement between chunk centres sits at the 0.2 m threshold, is never counted.
TEST(GnssTrackYawRootCause, FrontEndWheelDisagreementCounted) {
  struct Case {
    double step_m;  // m per fix
    bool frozen;
  };
  for (const Case c : {Case{0.1, true}, Case{0.1, false}, Case{0.012, false}}) {
    GnssTrackYawAligner a(cfg_after(true));
    int aligned = 0;
    for (int k = 0; k < 400; ++k) {
      const Pose b =
          yaw_pose(0.4, Vec3(c.step_m * k * std::cos(0.4), c.step_m * k * std::sin(0.4), 0.0));
      const Pose fe = c.frozen ? yaw_pose(0.4, Vec3::Zero()) : b;
      aligned +=
          a.update(kTew * (b * kPbg), Mat3::Identity() * 0.25, fe, kPbg, c.step_m * k, 0.0, 0.0)
                  .is_ok()
              ? 1
              : 0;
    }
    if (c.frozen) {
      EXPECT_EQ(aligned, 0);
      EXPECT_GT(a.n_frontend_wheel_disagreements(), 10);
    } else {
      EXPECT_GT(aligned, 0) << c.step_m;
      EXPECT_EQ(a.n_frontend_wheel_disagreements(), 0) << c.step_m;
    }
  }
}

// Turning in place with the travel axis observed: the front-end displacement of o stays below
// the travel threshold, so the segments are stationary although the antenna circles the body
// (its displacement is the lever-arm change only). Never aligns.
TEST(GnssTrackYawRootCause, FrontEndTurningInPlaceNeverAligns) {
  Scene sc;
  sc.shape = Shape::kSpin;
  sc.yaw_rate_radps = 0.5;
  sc.frontend = true;
  sc.tmax_s = 120.0;
  const Stats s = sweep(sc, cfg_after(true));
  EXPECT_EQ(s.aligned, 0);
}

}  // namespace
