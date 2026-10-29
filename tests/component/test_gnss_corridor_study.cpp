// Corridor fusion diagnosis (docs/derivations/gnss_fusion_study.md section 13), skipped unless
// MSS_GNSS_CORRIDOR_STUDY=1 (GTSAM build). The 120 m corridor of CorridorDriftCorrectedByGnss
// (60 s, GNSS 0.5 m at 5 Hz, no wheel) over seeds 42..56: per seed the fused error split along
// the corridor axis (w x) / lateral (w y) / vertical (w z), raw GNSS RMS, LiDAR-only ATE, GNSS
// factor / rejection / realign counts, the GNSS frame variable E against its prior and the
// truth, the along-axis marginal sigma and NEES of the optimised keyframes, and the along-axis
// RMS per 10 s window; then ablations (single switches, plus the oracles of the test seams
// force_T_e_w and gnss_shift_oracle and their combinations), paired against the reference
// variant and the shift oracles against the same frame without them (bootstrap 95 % CI of the
// median per-seed difference, fixed seed). The `prealign` variant is `ref` with
// gnss.attach_prealign_fixes (ADR-022, section 14); `trackyaw` is `ref` with
// gnss.align_yaw_from_track (ADR-023, section 15), `prealign_trackyaw` has both; the `_s45` /
// `_s60` suffixes set gnss.align_yaw_max_sigma_deg, `_wheel` runs with wheel odometry and `_2b`
// adds the design-2b keys (section 16); `aligncheck` is `ref` with gnss.align_check_yaw_sigma
// (section 17). Writes
// results/<MSS_GNSS_CORRIDOR_RUN_ID or "gnss-corridor-study">/gnss_corridor_study.md.
// MSS_GCS_VARIANTS: comma-separated variant names to run (default all; the reference first is
// always run); MSS_GCS_FIRST_SEED / MSS_GCS_LAST_SEED: seed range (default 42..56).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
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

#include "box_world.hpp"
#include "estimator_core.hpp"
#include "pipeline_test_support.hpp"

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline {
namespace {

using namespace ct;  // NOLINT(google-build-using-namespace): shared test helpers

constexpr int kWindows = 7;           // 10 s windows from t0 (the run lasts 65 s)
constexpr double kWindowS = 10.0;     // s
constexpr double kNees3SigmaX = 9.0;  // along-axis NEES (1 DoF) above 3 sigma
constexpr double kShiftBigM = 0.1;    // m, a keyframe-shift error counted as large

// Median of the finite values (NaN marks "not measured"; sorting a mix with NaN is undefined).
double median_of(const std::vector<double>& in) {
  std::vector<double> v;
  for (const double x : in) {
    if (std::isfinite(x)) {
      v.push_back(x);
    }
  }
  std::sort(v.begin(), v.end());
  return v.empty() ? std::nan("") : v[v.size() / 2];
}

// Records the optimised keyframes, the frame variable E and the GNSS factors.
class CorridorObserver : public detail::CoreObserver {
 public:
  explicit CorridorObserver(const BoxWorldParams& p) : p_(p) {}
  void on_keyframe_optimized(Timestamp t,
                             const common::NavState& x,
                             const common::PoseCovariance& cov) override {
    const common::NavState truth = test::box_truth(p_, t);
    const double ex = x.T_w_b.p.x() - truth.T_w_b.p.x();
    // Right perturbation: delta p_w = R_w_b rho, so cov_p in w = R cov_rho R^T.
    const common::Mat3 C_w = x.T_w_b.R * cov.topLeftCorner<3, 3>() * x.T_w_b.R.transpose();
    kf_.push_back({t, ex, std::sqrt(std::max(C_w(0, 0), 0.0)), n_gnss_});
  }
  void on_frame_estimate(Timestamp /*t*/, const common::Pose& T_e_w) override {
    if (!have_e_) {
      E_first_ = T_e_w;
      have_e_ = true;
    }
    E_last_ = T_e_w;
  }
  void on_alignment(Timestamp t,
                    const common::Pose& T_e_w,
                    double yaw_sigma_rad,
                    const gnss::GnssTrackYawAligner& /*track*/) override {
    if (n_alignments_ == 0) {
      align_T_ = T_e_w;
      align_sigma_rad_ = yaw_sigma_rad;
      align_t_ = t;
    }
    ++n_alignments_;
  }
  void on_fix_accepted(Timestamp t, const common::Pose& /*T_e_w*/) override {
    if (n_accepted_ == 0) {
      t_first_accepted_ = t;
    }
    ++n_accepted_;
  }
  void on_gnss_factor(Timestamp t_k,
                      const common::GnssMeasurement& m,
                      const common::Pose& T_e_w) override {
    if (n_gnss_ == 0) {
      E_prior_ = T_e_w;
    }
    aligner_last_ = T_e_w;
    ++n_gnss_;
    const common::Pose T_e_w_true = test::box_T_e_w(p_);
    const Vec3 r = m.p_e_g - T_e_w_true * (test::box_truth(p_, t_k).T_w_b * p_.p_b_g);
    nees_gnss_sum_ += r.dot(m.cov.ldlt().solve(r));
    // Error of the shift to the keyframe time (pipeline.md section 5): the factor's residual
    // against the truth at t_k minus the raw fix noise at m.t, along w x.
    const auto raw = raw_fix_.find(m.t.ns);
    if (raw != raw_fix_.end()) {
      const Vec3 noise = raw->second - T_e_w_true * (test::box_truth(p_, m.t).T_w_b * p_.p_b_g);
      const double sx = (T_e_w_true.R.transpose() * (r - noise)).x();
      shift_sq_ += sx * sx;
      shift_max_ = std::max(shift_max_, std::abs(sx));
      n_shift_big_ += std::abs(sx) > kShiftBigM ? 1U : 0U;
      ++n_shift_;
      const double dt_s = static_cast<double>(t_k.ns - m.t.ns) * 1e-9;
      const double since_prev_s =
          kf_.empty() ? -1.0 : static_cast<double>(m.t.ns - kf_.back().t.ns) * 1e-9;
      rows_.push_back({static_cast<double>(t_k.ns - p_.t0_ns) * 1e-9, dt_s, since_prev_s, sx});
    }
  }
  struct Row {
    double t_k_s, dt_s, since_prev_kf_s, shift_err_x;
  };
  std::vector<Row> rows_;

  struct Kf {
    Timestamp t;
    double ex;       // m, along-axis error of the optimised newest keyframe
    double sigma_x;  // m, its marginal along-axis sigma
    std::uint64_t n_gnss_before;
  };
  const BoxWorldParams& p_;
  std::vector<Kf> kf_;
  bool have_e_ = false;
  common::Pose E_first_;  // E after the first optimize with it
  common::Pose E_last_;
  common::Pose E_prior_;       // the aligner T_e_w given with the first GNSS factor (prior centre)
  common::Pose aligner_last_;  // the aligner T_e_w given with the last GNSS factor
  std::uint64_t n_gnss_ = 0;
  std::uint64_t n_accepted_ = 0;
  std::uint64_t n_alignments_ = 0;
  common::Pose align_T_;                   // aligner T_e_w at the first alignment
  Timestamp align_t_{};                    // time of the first alignment
  double align_sigma_rad_ = std::nan("");  // its predicted yaw sigma (ADR-023 only)
  std::map<std::int64_t, Vec3> raw_fix_;   // dataset fixes by t (ns)
  double shift_sq_ = 0.0;
  double shift_max_ = 0.0;
  std::uint64_t n_shift_big_ = 0;
  std::uint64_t n_shift_ = 0;
  Timestamp t_first_accepted_{};
  double nees_gnss_sum_ = 0.0;
};

struct Variant {
  std::string name;
  std::string what;           // the single switch, for the report
  std::string degeneracy;     // degeneracy + scan_match sections ("" = shipped, off)
  std::string extra;          // more sections (backend ...)
  std::string gnss_keys;      // more lines in the gnss section
  bool force_truth = false;   // CoreTestHooks::force_T_e_w with the true T_e_w
  std::string lidar_ref;      // key of the LiDAR-only run sharing the degeneracy settings
  bool oracle_shift = false;  // CoreTestHooks::gnss_shift_oracle with the true displacement
  bool wheel = false;         // wheel_odom.enabled (it observes the corridor axis)
};

struct SeedResult {
  double ate = 0.0, ate_x = 0.0, ate_y = 0.0, ate_z = 0.0;
  std::array<double, kWindows> win_x{};
  double lidar_ate = 0.0, lidar_ate_x = 0.0;
  double raw3d = 0.0, raw_x = 0.0;
  std::uint64_t factors = 0, rejected = 0, realign = 0, keyframes = 0, degenerate = 0;
  double end_ex = 0.0, end_sigma_x = 0.0, nees_x_mean = 0.0, frac_nees_x_3s = 0.0;
  double kf_rms_x = 0.0;  // RMS along-axis error of the optimised keyframes after GNSS starts
  // NaN = not measured (no frame variable E, or no GNSS factor).
  double E_prior_dx = std::nan(""), E_prior_dyaw_deg = std::nan("");  // E_last vs prior centre
  double E_true_dx = std::nan(""), E_true_dyaw_deg = std::nan("");    // E_last vs true T_e_w
  // The aligner T_e_w given with the first / last GNSS factor vs the true T_e_w, in w: the E
  // prior centre, or with gnss_frame_variable false the frame the factors use.
  double align_first_dx = std::nan(""), align_first_dyaw_deg = std::nan("");
  double align_last_dx = std::nan(""), align_last_dyaw_deg = std::nan("");
  std::uint64_t match_failures = 0;             // Stats::n_match_failures
  double nees_gnss = 0.0;                       // mean GNSS factor NEES vs truth (exp. 3)
  double shift_rms_x = 0.0, shift_max_x = 0.0;  // m, keyframe-shift error along w x
  std::uint64_t n_shift_big = 0;                // factors with |shift error| > kShiftBigM
  // First alignment (ADR-023): aligner yaw error vs truth and its predicted 1-sigma (deg).
  double align0_dyaw_deg = std::nan(""), align0_sigma_deg = std::nan("");
  // Section 16: time since the motion started (s) and distance driven (m) at the first
  // alignment; NaN when it never aligned.
  double align0_t_s = std::nan(""), align0_dist_m = std::nan("");
};

// Errors of published estimates (estimate - truth) in w: RMS per axis and per 10 s window.
void published_errors(const BoxWorldParams& p,
                      const std::vector<common::Estimate>& est,
                      SeedResult& r) {
  Vec3 sq = Vec3::Zero();
  std::array<double, kWindows> wsq{};
  std::array<int, kWindows> wn{};
  for (const common::Estimate& x : est) {
    const Vec3 e = x.state.T_w_b.p - test::box_truth(p, x.state.t).T_w_b.p;
    sq += e.cwiseProduct(e);
    const double ts = static_cast<double>(x.state.t.ns - p.t0_ns) * 1e-9;
    const int w = std::min(kWindows - 1, static_cast<int>(ts / kWindowS));
    wsq[static_cast<std::size_t>(w)] += e.x() * e.x();
    ++wn[static_cast<std::size_t>(w)];
  }
  const double n = std::max<double>(1.0, static_cast<double>(est.size()));
  r.ate = std::sqrt(sq.sum() / n);
  r.ate_x = std::sqrt(sq.x() / n);
  r.ate_y = std::sqrt(sq.y() / n);
  r.ate_z = std::sqrt(sq.z() / n);
  for (std::size_t w = 0; w < kWindows; ++w) {
    r.win_x[w] = wn[w] > 0 ? std::sqrt(wsq[w] / wn[w]) : std::nan("");
  }
}

// Raw GNSS error in the estimate's time span (after the IMU initialisation): 3-D RMS and the
// RMS of its component along w x (rotated to w with the true T_e_w).
void raw_gnss(const BoxWorldParams& p,
              const fs::path& data,
              double t_from_s,
              SeedResult& r,
              std::map<std::int64_t, Vec3>& fixes) {
  auto opened = io::DatasetReader::open(data, 65536);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  io::DatasetReader reader = std::move(opened).value();
  ScanFrame scan(65536);
  io::Record rec;
  const common::Pose T_e_w = test::box_T_e_w(p);
  double s3 = 0.0;
  double sx = 0.0;
  int n = 0;
  for (;;) {
    const auto more = reader.next(rec, scan);
    if (!more.is_ok() || !more.value()) {
      break;
    }
    if (rec.kind == io::RecordKind::kGnss) {
      fixes[rec.gnss.t.ns] = rec.gnss.p_e_g;
    }
    if (rec.kind != io::RecordKind::kGnss ||
        static_cast<double>(rec.gnss.t.ns - p.t0_ns) * 1e-9 < t_from_s) {
      continue;
    }
    const Vec3 r_e = rec.gnss.p_e_g - T_e_w * (test::box_truth(p, rec.gnss.t).T_w_b * p.p_b_g);
    const Vec3 r_w = T_e_w.R.transpose() * r_e;
    s3 += r_e.squaredNorm();
    sx += r_w.x() * r_w.x();
    ++n;
  }
  r.raw3d = std::sqrt(s3 / std::max(1, n));
  r.raw_x = std::sqrt(sx / std::max(1, n));
}

Config corridor_cfg(const BoxWorldParams& p, const fs::path& dir, bool gnss, const Variant& v) {
  return box_config(p,
                    gnss,
                    v.wheel,
                    dir,
                    "    preprocess:\n      max_range_m: 10.0\n" + v.degeneracy + v.extra,
                    v.gnss_keys);
}

void fused_run(const BoxWorldParams& p,
               const fs::path& data,
               const fs::path& dir,
               const Variant& v,
               const std::map<std::int64_t, Vec3>& fixes,
               SeedResult& r) {
  const Config cfg = corridor_cfg(p, dir, true, v);
  CorridorObserver obs(p);
  obs.raw_fix_ = fixes;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  hooks.force_T_e_w = v.force_truth;
  hooks.T_e_w = test::box_T_e_w(p);
  if (v.oracle_shift) {
    hooks.gnss_shift_oracle = [&p](Timestamp t_fix, Timestamp t_k) -> Vec3 {
      return Vec3(test::box_truth(p, t_k).T_w_b * p.p_b_g -
                  test::box_truth(p, t_fix).T_w_b * p.p_b_g);
    };
  }
  ReplayResult rr;
  {
    const HooksScope scope(&hooks);
    rr = replay(data, cfg);
  }
  published_errors(p, rr.estimates, r);
  r.factors = rr.stats.n_gnss_factors;
  r.realign = rr.stats.n_gnss_realign;
  r.keyframes = rr.stats.n_keyframes;
  r.degenerate = rr.stats.n_degenerate_keyframes;
  r.match_failures = rr.stats.n_match_failures;
  // Fixes not accepted after the first accepted one (chi-square rejections and the
  // not-aligned fixes after a realign); the box world writes one fix per 1 / gnss_rate_hz.
  if (obs.n_accepted_ > 0) {
    const double span_s =
        static_cast<double>(test::box_data_end(p).ns - obs.t_first_accepted_.ns) * 1e-9;
    const auto n_fixes = static_cast<std::uint64_t>(std::floor(span_s * p.gnss_rate_hz)) + 1U;
    r.rejected = n_fixes > obs.n_accepted_ ? n_fixes - obs.n_accepted_ : 0U;
  }
  r.shift_rms_x =
      obs.n_shift_ > 0 ? std::sqrt(obs.shift_sq_ / static_cast<double>(obs.n_shift_)) : 0.0;
  r.shift_max_x = obs.shift_max_;
  {
    std::ofstream f(dir / "gnss_factors.csv");
    f << "t_k_s,dt_s,since_prev_kf_s,shift_err_x_m\n";
    for (const auto& row : obs.rows_) {
      f << row.t_k_s << "," << row.dt_s << "," << row.since_prev_kf_s << "," << row.shift_err_x
        << "\n";
    }
    std::ofstream g(dir / "keyframes.csv");
    g << "t_s,ex_m,sigma_x_m,n_gnss_before\n";
    for (const auto& k : obs.kf_) {
      g << static_cast<double>(k.t.ns - p.t0_ns) * 1e-9 << "," << k.ex << "," << k.sigma_x << ","
        << k.n_gnss_before << "\n";
    }
  }
  r.n_shift_big = obs.n_shift_big_;
  r.nees_gnss = obs.n_gnss_ > 0 ? obs.nees_gnss_sum_ / static_cast<double>(obs.n_gnss_) : 0.0;
  double nees_sum = 0.0;
  double sq = 0.0;
  int n_nees = 0;
  int n_3s = 0;
  for (const CorridorObserver::Kf& k : obs.kf_) {
    if (k.n_gnss_before == 0 || !(k.sigma_x > 0.0)) {
      continue;
    }
    const double nees = k.ex * k.ex / (k.sigma_x * k.sigma_x);
    nees_sum += nees;
    sq += k.ex * k.ex;
    n_3s += nees > kNees3SigmaX ? 1 : 0;
    ++n_nees;
  }
  r.nees_x_mean = n_nees > 0 ? nees_sum / n_nees : std::nan("");
  r.frac_nees_x_3s = n_nees > 0 ? static_cast<double>(n_3s) / n_nees : std::nan("");
  r.kf_rms_x = n_nees > 0 ? std::sqrt(sq / n_nees) : std::nan("");
  if (!obs.kf_.empty()) {
    r.end_ex = obs.kf_.back().ex;
    r.end_sigma_x = obs.kf_.back().sigma_x;
  }
  // D = A^-1 B maps the w of B to the w of A; its x translation and yaw.
  const auto in_w = [](const common::Pose& A, const common::Pose& B, double& dx, double& dyaw) {
    const common::Pose D = A.inverse() * B;
    dx = D.p.x();
    dyaw = wrap_deg(yaw_of(D.R));
  };
  const common::Pose T_true = test::box_T_e_w(p);
  if (obs.have_e_) {
    in_w(obs.E_prior_, obs.E_last_, r.E_prior_dx, r.E_prior_dyaw_deg);
    in_w(T_true, obs.E_last_, r.E_true_dx, r.E_true_dyaw_deg);
  }
  if (obs.n_alignments_ > 0) {
    double dx = 0.0;
    in_w(T_true, obs.align_T_, dx, r.align0_dyaw_deg);
    r.align0_sigma_deg = obs.align_sigma_rad_ * kRadToDeg;
    r.align0_t_s = static_cast<double>(obs.align_t_.ns - p.t0_ns) * 1e-9 - p.static_s;
    r.align0_dist_m = test::box_path(p, obs.align_t_).s_m;
  }
  if (obs.n_gnss_ > 0) {
    in_w(T_true, obs.E_prior_, r.align_first_dx, r.align_first_dyaw_deg);
    in_w(T_true, obs.aligner_last_, r.align_last_dx, r.align_last_dyaw_deg);
  }
}

struct LidarRun {
  double ate = 0.0, ate_x = 0.0;
};

LidarRun lidar_run(const BoxWorldParams& p,
                   const fs::path& data,
                   const fs::path& dir,
                   const Variant& v) {
  const Config cfg = corridor_cfg(p, dir, false, v);
  const ReplayResult rr = replay(data, cfg);
  SeedResult r;
  published_errors(p, rr.estimates, r);
  return {r.ate, r.ate_x};
}

constexpr int kBootstrapResamples = 20000;
constexpr std::uint32_t kBootstrapSeed = 7U;

// 95 % bootstrap CI of median(a_i - b_i) over paired seeds (kBootstrapResamples resamples,
// std::mt19937 seeded kBootstrapSeed, index = rng() % n so the CI does not depend on the
// standard library's distribution implementation). Pairs with a non-finite value are dropped.
std::pair<double, double> bootstrap_median_diff(const std::vector<double>& a,
                                                const std::vector<double>& b) {
  std::mt19937 rng(kBootstrapSeed);
  const std::size_t n = a.size();
  std::vector<double> medians;
  medians.reserve(kBootstrapResamples);
  std::vector<double> d(n);
  for (int k = 0; k < kBootstrapResamples; ++k) {
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t j = static_cast<std::size_t>(rng()) % n;
      d[i] = a[j] - b[j];
    }
    medians.push_back(median_of(d));
  }
  std::sort(medians.begin(), medians.end());
  const auto lo = static_cast<std::size_t>(kBootstrapResamples / 40);  // 2.5 %
  return {medians[lo], medians[medians.size() - 1 - lo]};
}

std::string paired(const std::vector<double>& a_in, const std::vector<double>& b_in) {
  std::vector<double> a;
  std::vector<double> b;
  for (std::size_t i = 0; i < a_in.size(); ++i) {
    if (std::isfinite(a_in[i]) && std::isfinite(b_in[i])) {
      a.push_back(a_in[i]);
      b.push_back(b_in[i]);
    }
  }
  if (a.empty()) {
    return "n/a | n/a | n/a";
  }
  int wins = 0;
  int losses = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    wins += a[i] < b[i] ? 1 : 0;
    losses += a[i] > b[i] ? 1 : 0;
  }
  const std::pair<double, double> ci = bootstrap_median_diff(a, b);
  char buf[160];
  std::snprintf(buf,
                sizeof(buf),
                "%.3f (max %.2f) | %d/%d | [%+.3f, %+.3f]",
                median_of(a),
                *std::max_element(a.begin(), a.end()),
                wins,
                losses,
                ci.first,
                ci.second);
  return buf;
}

// "n/a" for a value that was not measured (NaN).
std::string fmt(const char* f, double x) {
  if (!std::isfinite(x)) {
    return "n/a";
  }
  char b[48];
  std::snprintf(b, sizeof(b), f, x);
  return b;
}

TEST(GnssCorridorStudy, Diagnosis) {
  const char* on = std::getenv("MSS_GNSS_CORRIDOR_STUDY");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_CORRIDOR_STUDY=1 (GTSAM build)";
  }
  StudyReport rep("gnss_corridor_study.md", "MSS_GNSS_CORRIDOR_RUN_ID", "gnss-corridor-study");
  const std::string deg_off;                      // shipped: degeneracy.enabled false
  const std::string deg_on = kCorridorRelaxYaml;  // #29 settings: absolute detector, 1000 m^2
  const std::string deg_huge =
      "    degeneracy:\n      enabled: true\n"
      "    scan_match:\n      unobserved_var_m2: 1.0e6\n      unobserved_var_rad2: 10.0\n";
  const std::string deg_shipped_var = "    degeneracy:\n      enabled: true\n";
  const std::string gate_off = "      chi2_gate: 1.0e12\n";
  // clang-format off
  std::vector<Variant> variants = {
      {"ref", "degeneracy on, 1000 m^2 (CorridorDriftCorrectedByGnss)", deg_on, "", "", false, "on"},
      {"deg_off", "degeneracy off (shipped)", deg_off, "", "", false, "off"},
      {"deg_huge", "degeneracy on, 1e6 m^2", deg_huge, "", "", false, "huge"},
      {"deg_var_shipped", "degeneracy on, shipped 0.001 m^2", deg_shipped_var, "", "", false, "shipped"},
      {"gate_off", "chi-square gate off (chi2_gate 1e12)", deg_on, "", gate_off, false, "on"},
      {"frame_fixed", "gnss_frame_variable false (aligner T_e_w frozen per factor)", deg_on,
       "    backend:\n      gnss_frame_variable: false\n", "", false, "on"},
      {"frame_tight_pos", "E translation prior 0.01 m (centred on the aligner's first T_e_w)", deg_on,
       "    backend:\n      gnss_frame_pos_sigma_m: 0.01\n", "", false, "on"},
      {"frame_truth_init", "aligner-frame oracle (force_T_e_w: E prior centre, gate and shift rotation "
       "at the true T_e_w, aligner off), loose E prior", deg_on, "", "", true, "on"},
      {"frame_truth_fixed", "aligner-frame oracle + gnss_frame_variable false (two switches: the true "
       "T_e_w frozen in every factor)", deg_on,
       "    backend:\n      gnss_frame_variable: false\n", "", true, "on"},
      {"oracle_shift", "true displacement in w for the shift to the keyframe (gnss_shift_oracle; still "
       "rotated into e with the aligner's R_e_w)", deg_on,
       "", "", false, "on", true},
      {"oracle_shift_truth_fixed", "oracle shift + frame_truth_fixed", deg_on,
       "    backend:\n      gnss_frame_variable: false\n", "", true, "on", true},
      {"oracle_shift_truth_init", "oracle shift + frame_truth_init",
       deg_on, "", "", true, "on", true},
      {"prealign", "gnss.attach_prealign_fixes (ADR-022): fixes before the first alignment, the "
       "static ones on keyframe 0, added as factors once aligned", deg_on, "",
       "      attach_prealign_fixes: true\n", false, "on"},
      {"prealign_oracle_shift", "prealign + shift-displacement oracle (gnss_shift_oracle, also for "
       "the pre-alignment fixes)", deg_on, "", "      attach_prealign_fixes: true\n", false, "on",
       true},
      {"trackyaw", "gnss.align_yaw_from_track (ADR-023): T_e_w yaw from the GNSS track direction "
       "against the body heading", deg_on, "", "      align_yaw_from_track: true\n", false, "on"},
      {"prealign_trackyaw", "prealign + trackyaw (designs 1 and 2)", deg_on, "",
       "      attach_prealign_fixes: true\n      align_yaw_from_track: true\n", false, "on"},
      {"prealign_trackyaw_oracle_shift", "prealign + trackyaw + shift-displacement oracle", deg_on,
       "", "      attach_prealign_fixes: true\n      align_yaw_from_track: true\n", false, "on",
       true},
  };
  // Section 16 (design 2b): the alignment yaw-sigma threshold, wheel odometry, and the 2b keys
  // (turns from the wheel path, minimum track with wheel data).
  const std::string d2 = "      align_yaw_from_track: true\n"
                         "      align_yaw_wheel_turns: false\n"
                         "      align_yaw_min_track_wheel_m: 5.0\n";
  const std::string pre = "      attach_prealign_fixes: true\n";
  const auto sig = [](const char* deg) {
    return std::string("      align_yaw_max_sigma_deg: ") + deg + "\n";
  };
  const std::string b2 = "      align_yaw_from_track: true\n"
                         "      align_yaw_wheel_turns: true\n"
                         "      align_yaw_min_track_wheel_m: 3.0\n";
  for (const auto& [suffix, deg] : {std::pair<const char*, const char*>{"", "3.0"},
                                    std::pair<const char*, const char*>{"_s45", "4.5"},
                                    std::pair<const char*, const char*>{"_s60", "6.0"}}) {
    const std::string s = suffix;
    variants.push_back({"trackyaw2" + s, std::string("trackyaw, yaw sigma ") + deg + " deg", deg_on,
                        "", d2 + sig(deg), false, "on"});
    variants.push_back({"prealign_trackyaw2" + s,
                        std::string("prealign + trackyaw, yaw sigma ") + deg + " deg", deg_on, "",
                        pre + d2 + sig(deg), false, "on"});
    variants.push_back({"trackyaw_wheel_2b" + s,
                        std::string("wheel + trackyaw + 2b keys, yaw sigma ") + deg + " deg",
                        deg_on, "", b2 + sig(deg), false, "on_wheel", false, true});
    variants.push_back({"prealign_trackyaw_wheel_2b" + s,
                        std::string("wheel + prealign + trackyaw + 2b keys, yaw sigma ") + deg +
                            " deg",
                        deg_on, "", pre + b2 + sig(deg), false, "on_wheel", false, true});
  }
  variants.push_back({"ref_wheel", "ref + wheel odometry", deg_on, "", "", false, "on_wheel",
                      false, true});
  // Section 17 (backlog G1): GnssAligner waits for its predicted yaw sigma.
  variants.push_back({"aligncheck", "gnss.align_check_yaw_sigma (GnssAligner yaw sigma <= 6 deg)",
                      deg_on, "", "      align_check_yaw_sigma: true\n", false, "on"});
  variants.push_back({"prealign_wheel", "prealign + wheel odometry", deg_on, "", pre, false,
                      "on_wheel", false, true});
  variants.push_back({"trackyaw_wheel", "wheel + trackyaw (design-2 keys)", deg_on, "", d2 + sig("3.0"), false,
                      "on_wheel", false, true});
  variants.push_back({"prealign_trackyaw_wheel", "wheel + prealign + trackyaw (design-2 keys)",
                      deg_on, "", pre + d2 + sig("3.0"), false, "on_wheel", false, true});
  // clang-format on
  if (const char* sel = std::getenv("MSS_GCS_VARIANTS"); sel != nullptr) {
    const std::string s = std::string(",") + sel + ",";
    std::vector<Variant> kept;
    for (const Variant& v : variants) {
      if (v.name == "ref" || s.find("," + v.name + ",") != std::string::npos) {
        kept.push_back(v);
      }
    }
    variants = kept;
  }
  std::uint32_t first = 42;
  std::uint32_t last = 56;
  if (const char* f = std::getenv("MSS_GCS_FIRST_SEED"); f != nullptr) {
    first = static_cast<std::uint32_t>(std::atoi(f));
  }
  if (const char* l = std::getenv("MSS_GCS_LAST_SEED"); l != nullptr) {
    last = static_cast<std::uint32_t>(std::atoi(l));
  }
  rep.line("# Corridor fusion diagnosis (generated by GnssCorridorStudy.Diagnosis)");
  rep.line("");
  std::vector<std::vector<SeedResult>> res(variants.size());
  std::vector<std::uint32_t> seeds;
  for (std::uint32_t seed = first; seed <= last; ++seed) {
    seeds.push_back(seed);
    BoxWorldParams p = corridor_params();
    p.seed = seed;
    const fs::path dir = rep.dir() / ("seed" + std::to_string(seed));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    SeedResult raw;
    std::map<std::int64_t, Vec3> fixes;
    ASSERT_NO_FATAL_FAILURE(raw_gnss(p, dir / "data", 2.0, raw, fixes));
    std::map<std::string, LidarRun> lidar;
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
      const Variant& v = variants[vi];
      if (lidar.count(v.lidar_ref) == 0) {
        lidar[v.lidar_ref] = lidar_run(p, dir / "data", dir / ("lidar_" + v.lidar_ref), v);
      }
      SeedResult r;
      fused_run(p, dir / "data", dir / v.name, v, fixes, r);
      r.lidar_ate = lidar[v.lidar_ref].ate;
      r.lidar_ate_x = lidar[v.lidar_ref].ate_x;
      r.raw3d = raw.raw3d;
      r.raw_x = raw.raw_x;
      std::printf(
          "PERSEED %s seed %u ate %.6f x %.6f y %.6f z %.6f lidar %.6f lidar_x %.6f raw3d %.6f "
          "raw_x %.6f factors %llu rejected %llu realign %llu kf %llu deg %llu end_ex %.4f end_sx "
          "%.4f "
          "nees_x %.3f frac3s %.3f kf_rms_x %.4f Eprior_dx %.4f Eprior_dyaw %.4f Etrue_dx %.4f "
          "Etrue_dyaw %.4f align_first_dx %.4f align_first_dyaw %.4f align_last_dx %.4f "
          "align_last_dyaw %.4f match_fail %llu nees_gnss %.3f shift_rms %.4f shift_max %.4f "
          "shift_big %llu align0_dyaw %.3f align0_sigma %.3f align0_t %.2f align0_dist %.2f "
          "win",
          v.name.c_str(),
          seed,
          r.ate,
          r.ate_x,
          r.ate_y,
          r.ate_z,
          r.lidar_ate,
          r.lidar_ate_x,
          r.raw3d,
          r.raw_x,
          static_cast<unsigned long long>(r.factors),
          static_cast<unsigned long long>(r.rejected),
          static_cast<unsigned long long>(r.realign),
          static_cast<unsigned long long>(r.keyframes),
          static_cast<unsigned long long>(r.degenerate),
          r.end_ex,
          r.end_sigma_x,
          r.nees_x_mean,
          r.frac_nees_x_3s,
          r.kf_rms_x,
          r.E_prior_dx,
          r.E_prior_dyaw_deg,
          r.E_true_dx,
          r.E_true_dyaw_deg,
          r.align_first_dx,
          r.align_first_dyaw_deg,
          r.align_last_dx,
          r.align_last_dyaw_deg,
          static_cast<unsigned long long>(r.match_failures),
          r.nees_gnss,
          r.shift_rms_x,
          r.shift_max_x,
          static_cast<unsigned long long>(r.n_shift_big),
          r.align0_dyaw_deg,
          r.align0_sigma_deg,
          r.align0_t_s,
          r.align0_dist_m);
      for (const double w : r.win_x) {
        std::printf(" %.4f", w);
      }
      std::printf("\n");
      std::fflush(stdout);
      res[vi].push_back(r);
    }
  }

  // Per-seed table of every variant.
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    rep.line("");
    rep.line("## " + variants[vi].name + ": " + variants[vi].what);
    rep.line("");
    rep.line(
        "| seed | ATE | along x | lateral y | vertical z | raw GNSS 3D / x | LiDAR-only ATE / x | "
        "GNSS factors | not accepted | realign | keyframes (flagged) | kf along-x RMS | end kf x "
        "err / sigma "
        "| kf NEES x mean (> 3 sigma) | E - prior: x, yaw deg | E - truth: x, yaw deg | aligner "
        "T_e_w - truth, first / last factor: yaw deg | match failures | GNSS NEES vs truth |");
    rep.line(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
        "--- | --- | --- | --- | --- |");
    for (std::size_t si = 0; si < seeds.size(); ++si) {
      const SeedResult& r = res[vi][si];
      char b[640];
      std::snprintf(
          b,
          sizeof(b),
          "| %u | %.3f | %.3f | %.4f | %.4f | %.3f / %.3f | %.2f / %.2f | %llu | %llu | %llu | "
          "%llu (%llu) | %.3f | %+.3f / %.3f | %.1f (%.0f %%) | %s, %s | %s, %s | %s / %s | "
          "%llu | %.2f |",
          seeds[si],
          r.ate,
          r.ate_x,
          r.ate_y,
          r.ate_z,
          r.raw3d,
          r.raw_x,
          r.lidar_ate,
          r.lidar_ate_x,
          static_cast<unsigned long long>(r.factors),
          static_cast<unsigned long long>(r.rejected),
          static_cast<unsigned long long>(r.realign),
          static_cast<unsigned long long>(r.keyframes),
          static_cast<unsigned long long>(r.degenerate),
          r.kf_rms_x,
          r.end_ex,
          r.end_sigma_x,
          r.nees_x_mean,
          100.0 * r.frac_nees_x_3s,
          fmt("%+.2f", r.E_prior_dx).c_str(),
          fmt("%+.2f", r.E_prior_dyaw_deg).c_str(),
          fmt("%+.2f", r.E_true_dx).c_str(),
          fmt("%+.2f", r.E_true_dyaw_deg).c_str(),
          fmt("%+.1f", r.align_first_dyaw_deg).c_str(),
          fmt("%+.1f", r.align_last_dyaw_deg).c_str(),
          static_cast<unsigned long long>(r.match_failures),
          r.nees_gnss);
      rep.line(b);
    }
    std::string win = "Along-x RMS per 10 s window (median over seeds, m):";
    for (std::size_t w = 0; w < kWindows; ++w) {
      std::vector<double> v;
      for (const SeedResult& r : res[vi]) {
        v.push_back(r.win_x[w]);
      }
      win += fmt(" %.3f", median_of(v));
    }
    rep.line("");
    rep.line(win);
  }

  // Section 16: the first alignment per variant (time since the motion started, distance
  // driven, yaw error against its predicted sigma) and what follows it.
  rep.line("");
  rep.line("## First alignment (section 16)");
  rep.line("");
  rep.line(
      "| variant | aligned | time median (max), s | distance median (max), m | abs yaw err median "
      "(max), deg | predicted sigma median, deg | yaw NEES mean | beyond 3 sigma | > 10 deg at "
      "first factor | E yaw > 10 deg at end | not accepted after, median (max) | GNSS factors "
      "median |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    std::vector<double> t;
    std::vector<double> d;
    std::vector<double> e;
    std::vector<double> sg;
    std::vector<double> rej;
    std::vector<double> fac;
    double nees = 0.0;
    int n_al = 0;
    int b3 = 0;
    int first10 = 0;
    int end10 = 0;
    for (const SeedResult& r : res[vi]) {
      rej.push_back(static_cast<double>(r.rejected));
      fac.push_back(static_cast<double>(r.factors));
      first10 += std::abs(r.align_first_dyaw_deg) > 10.0 ? 1 : 0;
      end10 += std::abs(r.E_true_dyaw_deg) > 10.0 ? 1 : 0;
      if (!std::isfinite(r.align0_t_s)) {
        continue;
      }
      ++n_al;
      t.push_back(r.align0_t_s);
      d.push_back(r.align0_dist_m);
      e.push_back(std::abs(r.align0_dyaw_deg));
      if (std::isfinite(r.align0_sigma_deg) && r.align0_sigma_deg > 0.0) {
        sg.push_back(r.align0_sigma_deg);
        const double z = r.align0_dyaw_deg / r.align0_sigma_deg;
        nees += z * z;
        b3 += std::abs(z) > 3.0 ? 1 : 0;
      }
    }
    const auto mx = [](const std::vector<double>& v) {
      return v.empty() ? std::nan("") : *std::max_element(v.begin(), v.end());
    };
    char b[512];
    std::snprintf(
        b,
        sizeof(b),
        "| %s | %d/%zu | %s (%s) | %s (%s) | %s (%s) | %s | %s | %d | %d | %d | %s (%s) | "
        "%s |",
        variants[vi].name.c_str(),
        n_al,
        res[vi].size(),
        fmt("%.1f", median_of(t)).c_str(),
        fmt("%.1f", mx(t)).c_str(),
        fmt("%.2f", median_of(d)).c_str(),
        fmt("%.2f", mx(d)).c_str(),
        fmt("%.2f", median_of(e)).c_str(),
        fmt("%.2f", mx(e)).c_str(),
        fmt("%.2f", median_of(sg)).c_str(),
        sg.empty() ? "n/a" : fmt("%.2f", nees / static_cast<double>(sg.size())).c_str(),
        b3,
        first10,
        end10,
        fmt("%.0f", median_of(rej)).c_str(),
        fmt("%.0f", mx(rej)).c_str(),
        fmt("%.0f", median_of(fac)).c_str());
    rep.line(b);
  }

  // Paired comparison against the reference (the first variant).
  rep.line("");
  rep.line("## Ablations paired against " + variants[0].name);
  rep.line("");
  rep.line(
      "Cells: median (max) | wins / losses against the reference over the paired seeds | 95 % "
      "bootstrap CI of the median per-seed difference (variant - reference). Lower is better.");
  rep.line("");
  rep.line("| variant | fused ATE (m) | along-x RMS (m) | kf NEES x mean |");
  rep.line("| --- | --- | --- | --- |");
  const auto col = [&](std::size_t vi, auto get) {
    std::vector<double> v;
    for (const SeedResult& r : res[vi]) {
      v.push_back(get(r));
    }
    return v;
  };
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    const auto ate = [](const SeedResult& r) { return r.ate; };
    const auto ax = [](const SeedResult& r) { return r.ate_x; };
    const auto nx = [](const SeedResult& r) { return r.nees_x_mean; };
    rep.line("| " + variants[vi].name + " | " + paired(col(vi, ate), col(0, ate)) + " | " +
             paired(col(vi, ax), col(0, ax)) + " | " + paired(col(vi, nx), col(0, nx)) + " |");
  }
  // Against raw GNSS (fused - raw 3D) for every variant.
  rep.line("");
  rep.line("| variant | fused ATE vs raw GNSS 3D RMS | along-x vs raw GNSS along x |");
  rep.line("| --- | --- | --- |");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    rep.line("| " + variants[vi].name + " | " +
             paired(col(vi, [](const SeedResult& r) { return r.ate; }),
                    col(vi, [](const SeedResult& r) { return r.raw3d; })) +
             " | " +
             paired(col(vi, [](const SeedResult& r) { return r.ate_x; }),
                    col(vi, [](const SeedResult& r) { return r.raw_x; })) +
             " |");
  }
  // Interaction: the shift oracle against the same frame without it (section 13.3, H5).
  rep.line("");
  rep.line("## Selected pairs: shift oracle, designs 1 and 2 (fused ATE, m)");
  rep.line("");
  rep.line("| variant | against | fused ATE |");
  rep.line("| --- | --- | --- |");
  const auto index_of = [&](const std::string& name) -> int {
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
      if (variants[vi].name == name) {
        return static_cast<int>(vi);
      }
    }
    return -1;
  };
  for (const auto& [with, without] :
       {std::pair<const char*, const char*>{"oracle_shift", "ref"},
        std::pair<const char*, const char*>{"prealign_oracle_shift", "oracle_shift"},
        std::pair<const char*, const char*>{"prealign_oracle_shift", "prealign"},
        std::pair<const char*, const char*>{"trackyaw", "ref"},
        std::pair<const char*, const char*>{"prealign_trackyaw", "prealign"},
        std::pair<const char*, const char*>{"prealign_trackyaw", "trackyaw"},
        std::pair<const char*, const char*>{"prealign_trackyaw_oracle_shift",
                                            "prealign_oracle_shift"},
        std::pair<const char*, const char*>{"prealign_trackyaw2_s45", "prealign_trackyaw2"},
        std::pair<const char*, const char*>{"prealign_trackyaw2_s60", "prealign_trackyaw2"},
        std::pair<const char*, const char*>{"trackyaw2_s45", "trackyaw2"},
        std::pair<const char*, const char*>{"trackyaw2_s60", "trackyaw2"},
        std::pair<const char*, const char*>{"prealign_trackyaw_wheel_2b",
                                            "prealign_trackyaw_wheel"},
        std::pair<const char*, const char*>{"prealign_trackyaw_wheel_2b_s45",
                                            "prealign_trackyaw_wheel"},
        std::pair<const char*, const char*>{"prealign_trackyaw_wheel_2b_s60",
                                            "prealign_trackyaw_wheel"},
        std::pair<const char*, const char*>{"prealign_trackyaw_wheel_2b_s60",
                                            "prealign_trackyaw_wheel_2b"},
        std::pair<const char*, const char*>{"trackyaw_wheel_2b_s60", "trackyaw_wheel"},
        std::pair<const char*, const char*>{"prealign_trackyaw_wheel", "prealign_wheel"},
        std::pair<const char*, const char*>{"prealign_wheel", "ref_wheel"},
        std::pair<const char*, const char*>{"oracle_shift_truth_init", "frame_truth_init"},
        std::pair<const char*, const char*>{"oracle_shift_truth_fixed", "frame_truth_fixed"}}) {
    const int a = index_of(with);
    const int b = index_of(without);
    if (a < 0 || b < 0) {
      continue;
    }
    const auto ate = [](const SeedResult& r) { return r.ate; };
    rep.line(std::string("| ") + with + " | " + without + " | " +
             paired(col(static_cast<std::size_t>(a), ate), col(static_cast<std::size_t>(b), ate)) +
             " |");
  }
}

// Box world with GNSS (gnss_fusion_study.md sections 9 and 11, default box world: 2 m circle,
// GNSS 0.5 m at 5 Hz), seeds 42..56, a GNSS switch off against on, paired. `on_keys`: the
// gnss-section lines of "on"; `file`, `default_id`: report file and default run id. About 4 min.
void box_world_pairs(const char* title,
                     const std::string& on_keys,
                     const char* file,
                     const char* default_id) {
  StudyReport rep(file, "MSS_GNSS_CORRIDOR_RUN_ID", default_id);
  rep.line(std::string("# Box world with GNSS, ") + title + " off / on (GnssCorridorStudy)");
  rep.line("");
  rep.line(
      "| case | seed | ATE off (m) | ATE on (m) | GNSS factors off / on | attached | dropped |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- |");
  struct Case {
    const char* name;
    bool wheel;
  };
  for (const Case c : {Case{"gnss", false}, Case{"gnss_wheel", true}}) {
    std::vector<double> off;
    std::vector<double> with;
    for (std::uint32_t seed = 42; seed <= 56; ++seed) {
      BoxWorldParams p;
      p.seed = seed;
      const fs::path dir = rep.dir() / c.name / ("seed" + std::to_string(seed));
      ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
      const ReplayResult a = replay(dir / "data", box_config(p, true, c.wheel, dir / "off"));
      const ReplayResult b =
          replay(dir / "data", box_config(p, true, c.wheel, dir / "on", "", on_keys));
      off.push_back(trajectory_errors(p, a.estimates).ate_m);
      with.push_back(trajectory_errors(p, b.estimates).ate_m);
      char buf[256];
      std::snprintf(buf,
                    sizeof(buf),
                    "| %s | %u | %.5f | %.5f | %llu / %llu | %llu | %llu |",
                    c.name,
                    seed,
                    off.back(),
                    with.back(),
                    static_cast<unsigned long long>(a.stats.n_gnss_factors),
                    static_cast<unsigned long long>(b.stats.n_gnss_factors),
                    static_cast<unsigned long long>(b.stats.n_gnss_prealign_attached),
                    static_cast<unsigned long long>(b.stats.n_gnss_prealign_dropped));
      rep.line(buf);
    }
    rep.line(std::string("| ") + c.name + " on vs off | " + paired(with, off) + " | | | |");
  }
}

// gnss.attach_prealign_fixes (section 14). Skipped unless MSS_GNSS_BOX_PAIRS=1 (GTSAM build).
// Writes results/<MSS_GNSS_CORRIDOR_RUN_ID or "gnss-box-pairs">/gnss_box_pairs.md.
TEST(GnssCorridorStudy, BoxWorldPrealignPairs) {
  const char* on = std::getenv("MSS_GNSS_BOX_PAIRS");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_BOX_PAIRS=1 (GTSAM build)";
  }
  box_world_pairs("gnss.attach_prealign_fixes",
                  "      attach_prealign_fixes: true\n",
                  "gnss_box_pairs.md",
                  "gnss-box-pairs");
}

// gnss.align_yaw_from_track (section 15; the 2 m circle exercises the lever-arm correction of
// gnss.md section 5). Skipped unless MSS_GNSS_BOX_TRACKYAW=1 (GTSAM build). Writes
// results/<MSS_GNSS_CORRIDOR_RUN_ID or "gnss-box-trackyaw">/gnss_box_trackyaw.md.
TEST(GnssCorridorStudy, BoxWorldTrackYawPairs) {
  const char* on = std::getenv("MSS_GNSS_BOX_TRACKYAW");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_BOX_TRACKYAW=1 (GTSAM build)";
  }
  box_world_pairs("gnss.align_yaw_from_track",
                  "      align_yaw_from_track: true\n",
                  "gnss_box_trackyaw.md",
                  "gnss-box-trackyaw");
}

// gnss.align_check_yaw_sigma (section 17, backlog G1). Skipped unless MSS_GNSS_BOX_ALIGNCHECK=1
// (GTSAM build). Writes results/<MSS_GNSS_CORRIDOR_RUN_ID or "gnss-box-aligncheck">/
// gnss_box_aligncheck.md.
TEST(GnssCorridorStudy, BoxWorldAlignCheckPairs) {
  const char* on = std::getenv("MSS_GNSS_BOX_ALIGNCHECK");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_BOX_ALIGNCHECK=1 (GTSAM build)";
  }
  box_world_pairs("gnss.align_check_yaw_sigma",
                  "      align_check_yaw_sigma: true\n",
                  "gnss_box_aligncheck.md",
                  "gnss-box-aligncheck");
}

// Section 16 (design 2b): the first alignment of a replay (time, predicted yaw sigma, aligner
// T_e_w, turn segments taken from the wheel path).
class AlignObserver : public detail::CoreObserver {
 public:
  void on_alignment(Timestamp t,
                    const common::Pose& T_e_w,
                    double yaw_sigma_rad,
                    const gnss::GnssTrackYawAligner& track) override {
    if (n_ == 0) {
      t_ = t;
      T_ = T_e_w;
      sigma_rad_ = yaw_sigma_rad;
      wheel_turns_ = track.n_wheel_turn_segments();
    }
    ++n_;
  }
  int n_ = 0;
  Timestamp t_{};
  common::Pose T_;
  double sigma_rad_ = std::nan("");
  int wheel_turns_ = 0;
};

// Box world (2 m circle, 20 s, GNSS 0.5 m at 5 Hz), seeds 42..56, without and with wheel
// odometry: GnssAligner, design 2 (track yaw, 3 deg, 5 m) and design 2b at 3 / 4.5 / 6 deg
// (turns from the wheel path, 3 m minimum track with wheel data): fused ATE paired, GNSS
// factors, and the first alignment (time since the motion started, distance driven, yaw error
// against the predicted sigma). Skipped unless MSS_GNSS_BOX_2B=1 (GTSAM build). Writes
// results/<MSS_GNSS_CORRIDOR_RUN_ID or "gnss-box-2b">/gnss_box_2b.md. About 8 min.
TEST(GnssCorridorStudy, BoxWorldAlignment2b) {
  const char* on = std::getenv("MSS_GNSS_BOX_2B");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_BOX_2B=1 (GTSAM build)";
  }
  StudyReport rep("gnss_box_2b.md", "MSS_GNSS_CORRIDOR_RUN_ID", "gnss-box-2b");
  const std::string d2 =
      "      align_yaw_from_track: true\n      align_yaw_wheel_turns: false\n"
      "      align_yaw_min_track_wheel_m: 5.0\n";
  const std::string b2 =
      "      align_yaw_from_track: true\n      align_yaw_wheel_turns: true\n"
      "      align_yaw_min_track_wheel_m: 3.0\n";
  const auto sig = [](const char* deg) {
    return std::string("      align_yaw_max_sigma_deg: ") + deg + "\n";
  };
  const std::vector<std::pair<std::string, std::string>> configs = {{"off", ""},
                                                                    {"d2", d2 + sig("3.0")},
                                                                    {"2b_s30", b2 + sig("3.0")},
                                                                    {"2b_s45", b2 + sig("4.5")},
                                                                    {"2b_s60", b2 + sig("6.0")}};
  rep.line("# Box world, first alignment and fused ATE (GnssCorridorStudy.BoxWorldAlignment2b)");
  struct Row {
    double ate = 0.0, t = std::nan(""), d = std::nan(""), e = std::nan(""), s = std::nan("");
    std::uint64_t factors = 0;
  };
  for (const bool wheel : {false, true}) {
    const char* name = wheel ? "gnss_wheel" : "gnss";
    std::vector<std::vector<Row>> rows(configs.size());
    for (std::uint32_t seed = 42; seed <= 56; ++seed) {
      BoxWorldParams p;
      p.seed = seed;
      const fs::path dir = rep.dir() / name / ("seed" + std::to_string(seed));
      ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
      for (std::size_t ci = 0; ci < configs.size(); ++ci) {
        AlignObserver obs;
        detail::CoreTestHooks hooks;
        hooks.observer = &obs;
        ReplayResult rr;
        {
          const HooksScope scope(&hooks);
          rr = replay(dir / "data",
                      box_config(p, true, wheel, dir / configs[ci].first, "", configs[ci].second));
        }
        Row r;
        r.ate = trajectory_errors(p, rr.estimates).ate_m;
        r.factors = rr.stats.n_gnss_factors;
        if (obs.n_ > 0) {
          r.t = static_cast<double>(obs.t_.ns - p.t0_ns) * 1e-9 - p.static_s;
          r.d = test::box_path(p, obs.t_).s_m;
          const common::Pose D = test::box_T_e_w(p).inverse() * obs.T_;
          r.e = wrap_deg(yaw_of(D.R));
          r.s = obs.sigma_rad_ * kRadToDeg;
        }
        std::printf(
            "BOX2B %s seed %u %s ate %.5f factors %llu align_t %.2f dist %.2f yaw_err %.3f "
            "sigma %.3f wheel_turns %d\n",
            name,
            seed,
            configs[ci].first.c_str(),
            r.ate,
            static_cast<unsigned long long>(r.factors),
            r.t,
            r.d,
            r.e,
            r.s,
            obs.wheel_turns_);
        std::fflush(stdout);
        rows[ci].push_back(r);
      }
    }
    rep.line("");
    rep.line(std::string("## ") + name);
    rep.line("");
    rep.line(
        "| config | fused ATE (m) | vs off: wins-losses | CI | vs d2: ATE | wins-losses | CI | "
        "GNSS factors median "
        "(min) | time to align median (max), s | distance median (max), m | abs yaw err median "
        "(max), deg | sigma median, deg | yaw NEES | beyond 3 sigma |");
    rep.line(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |");
    const auto col = [&](std::size_t ci, auto get) {
      std::vector<double> v;
      for (const Row& r : rows[ci]) {
        v.push_back(get(r));
      }
      return v;
    };
    const auto mx = [](const std::vector<double>& v) {
      double m = std::nan("");
      for (const double x : v) {
        m = std::isfinite(x) && !(x <= m) ? x : m;
      }
      return m;
    };
    for (std::size_t ci = 0; ci < configs.size(); ++ci) {
      const auto ate = [](const Row& r) { return r.ate; };
      std::vector<double> fac =
          col(ci, [](const Row& r) { return static_cast<double>(r.factors); });
      std::vector<double> t = col(ci, [](const Row& r) { return r.t; });
      std::vector<double> d = col(ci, [](const Row& r) { return r.d; });
      std::vector<double> e = col(ci, [](const Row& r) { return std::abs(r.e); });
      std::vector<double> s = col(ci, [](const Row& r) { return r.s; });
      double nees = 0.0;
      int n = 0;
      int b3 = 0;
      for (const Row& r : rows[ci]) {
        if (std::isfinite(r.e) && std::isfinite(r.s) && r.s > 0.0) {
          const double z = r.e / r.s;
          nees += z * z;
          b3 += std::abs(z) > 3.0 ? 1 : 0;
          ++n;
        }
      }
      const double fac_min = *std::min_element(fac.begin(), fac.end());
      rep.line("| " + configs[ci].first + " | " + paired(col(ci, ate), col(0, ate)) + " | " +
               (ci >= 2 ? paired(col(ci, ate), col(1, ate)) : std::string(" | | ")) + " | " +
               fmt("%.0f", median_of(fac)) + " (" + fmt("%.0f", fac_min) + ") | " +
               fmt("%.1f", median_of(t)) + " (" + fmt("%.1f", mx(t)) + ") | " +
               fmt("%.2f", median_of(d)) + " (" + fmt("%.2f", mx(d)) + ") | " +
               fmt("%.2f", median_of(e)) + " (" + fmt("%.2f", mx(e)) + ") | " +
               fmt("%.2f", median_of(s)) + " | " + (n > 0 ? fmt("%.2f", nees / n) : "n/a") + " | " +
               std::to_string(b3) + " |");
    }
  }
}

// Replays one dataset (MSS_GNSS_REPLAY_DATA, an mssdata directory) with config/default.yaml and
// the overlay MSS_GNSS_REPLAY_OVERLAY, and prints the first alignment: time since the first
// record, distance driven along the ground truth by then, predicted yaw sigma, and the aligner
// yaw against the yaw of a 2-D Procrustes fit of the published positions (w) to the ground truth
// (its frame), plus the fused ATE against the ground truth after that fit (section 16, E-01).
// Skipped unless both are set (GTSAM build).
TEST(GnssCorridorStudy, ReplayAlignment) {
  const char* data_env = std::getenv("MSS_GNSS_REPLAY_DATA");
  const char* overlay_env = std::getenv("MSS_GNSS_REPLAY_OVERLAY");
  if (data_env == nullptr || overlay_env == nullptr || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_GNSS_REPLAY_DATA and MSS_GNSS_REPLAY_OVERLAY (GTSAM build)";
  }
  const fs::path data = data_env;
  const auto cfg = load_config(kDefault, fs::path(overlay_env));
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  AlignObserver obs;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  ReplayResult rr;
  {
    const HooksScope scope(&hooks);
    rr = replay(data, cfg.value());
  }
  // Ground truth positions by time, the first record time and the path length.
  std::map<std::int64_t, Vec3> gt;
  Timestamp t_first{};
  bool have_first = false;
  {
    auto opened = io::DatasetReader::open(data, 65536);
    ASSERT_TRUE(opened.is_ok()) << opened.reason();
    io::DatasetReader reader = std::move(opened).value();
    ScanFrame scan(65536);
    io::Record rec;
    for (;;) {
      const auto more = reader.next(rec, scan);
      if (!more.is_ok() || !more.value()) {
        break;
      }
      if (!have_first) {
        t_first = rec.t;
        have_first = true;
      }
      if (rec.kind == io::RecordKind::kGt) {
        gt[rec.t.ns] = rec.gt.T_w_b.p;
      }
    }
  }
  ASSERT_FALSE(gt.empty());
  double dist_m = 0.0;
  if (obs.n_ > 0) {
    const Vec3* prev = nullptr;
    for (const auto& [t_ns, p] : gt) {
      if (t_ns > obs.t_.ns) {
        break;
      }
      if (prev != nullptr) {
        dist_m += (p - *prev).head<2>().norm();
      }
      prev = &p;
    }
  }
  // 2-D Procrustes of the published positions (w) onto the ground truth at the nearest time.
  std::vector<std::pair<Vec3, Vec3>> pairs;
  for (const common::Estimate& e : rr.estimates) {
    const auto it = gt.lower_bound(e.state.t.ns);
    if (it != gt.end() && it->first - e.state.t.ns < 5'000'000) {
      pairs.emplace_back(e.state.T_w_b.p, it->second);
    }
  }
  ASSERT_GT(pairs.size(), 10U);
  Vec3 mw = Vec3::Zero();
  Vec3 mg = Vec3::Zero();
  for (const auto& [w, g] : pairs) {
    mw += w;
    mg += g;
  }
  mw /= static_cast<double>(pairs.size());
  mg /= static_cast<double>(pairs.size());
  double sd = 0.0;
  double sc = 0.0;
  for (const auto& [w, g] : pairs) {
    const Vec3 a = w - mw;
    const Vec3 b = g - mg;
    sd += a.x() * b.x() + a.y() * b.y();
    sc += a.x() * b.y() - a.y() * b.x();
  }
  const double fit_yaw = std::atan2(sc, sd);
  double sq = 0.0;
  for (const auto& [w, g] : pairs) {
    const Vec3 a = w - mw;
    const Vec3 r(std::cos(fit_yaw) * a.x() - std::sin(fit_yaw) * a.y() - (g - mg).x(),
                 std::sin(fit_yaw) * a.x() + std::cos(fit_yaw) * a.y() - (g - mg).y(),
                 a.z() - (g - mg).z());
    sq += r.squaredNorm();
  }
  const double align_yaw = obs.n_ > 0 ? yaw_of(obs.T_.R) : std::nan("");
  std::printf(
      "REPLAY aligned %d t_since_first %.2f dist %.2f sigma_deg %.3f align_yaw_deg %.3f "
      "fit_yaw_deg %.3f err_deg %.3f wheel_turns %d factors %llu fit_rmse %.5f\n",
      obs.n_,
      obs.n_ > 0 ? static_cast<double>(obs.t_.ns - t_first.ns) * 1e-9 : std::nan(""),
      dist_m,
      obs.sigma_rad_ * kRadToDeg,
      align_yaw * kRadToDeg,
      fit_yaw * kRadToDeg,
      wrap_deg(align_yaw - fit_yaw),
      obs.wheel_turns_,
      static_cast<unsigned long long>(rr.stats.n_gnss_factors),
      std::sqrt(sq / static_cast<double>(pairs.size())));
}

}  // namespace
}  // namespace mss::pipeline
