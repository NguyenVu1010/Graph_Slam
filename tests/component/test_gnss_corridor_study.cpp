// Corridor fusion diagnosis (docs/derivations/gnss_fusion_study.md section 13), skipped unless
// MSS_GNSS_CORRIDOR_STUDY=1 (GTSAM build). The 120 m corridor of CorridorDriftCorrectedByGnss
// (60 s, GNSS 0.5 m at 5 Hz, no wheel) over seeds 42..56: per seed the fused error split along
// the corridor axis (w x) / lateral (w y) / vertical (w z), raw GNSS RMS, LiDAR-only ATE, GNSS
// factor / rejection / realign counts, the GNSS frame variable E against its prior and the
// truth, the along-axis marginal sigma and NEES of the optimised keyframes, and the along-axis
// RMS per 10 s window; then single-switch ablations (and two combined oracles, test seams
// force_T_e_w and gnss_shift_oracle), paired against the reference variant
// (bootstrap 95 % CI of the median per-seed difference). Writes
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

double median_of(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v[v.size() / 2];
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
  common::Pose E_prior_;  // the aligner T_e_w given with the first GNSS factor (prior centre)
  std::uint64_t n_gnss_ = 0;
  std::uint64_t n_accepted_ = 0;
  std::map<std::int64_t, Vec3> raw_fix_;  // dataset fixes by t (ns)
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
};

struct SeedResult {
  double ate = 0.0, ate_x = 0.0, ate_y = 0.0, ate_z = 0.0;
  std::array<double, kWindows> win_x{};
  double lidar_ate = 0.0, lidar_ate_x = 0.0;
  double raw3d = 0.0, raw_x = 0.0;
  std::uint64_t factors = 0, rejected = 0, realign = 0, keyframes = 0, degenerate = 0;
  double end_ex = 0.0, end_sigma_x = 0.0, nees_x_mean = 0.0, frac_nees_x_3s = 0.0;
  double kf_rms_x = 0.0;  // RMS along-axis error of the optimised keyframes after GNSS starts
  double E_prior_dx = 0.0, E_prior_dyaw_deg = 0.0;  // E_last vs its prior centre, in w
  double E_true_dx = 0.0, E_true_dyaw_deg = 0.0;    // E_last vs the true T_e_w, in w
  double nees_gnss = 0.0;                           // mean GNSS factor NEES vs truth (exp. 3)
  double shift_rms_x = 0.0, shift_max_x = 0.0;      // m, keyframe-shift error along w x
  std::uint64_t n_shift_big = 0;                    // factors with |shift error| > kShiftBigM
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
                    false,
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
  if (obs.have_e_) {
    // D = A^-1 E_last maps the estimated w to the w of A; its x translation and yaw.
    const auto in_w = [&](const common::Pose& A, double& dx, double& dyaw_deg) {
      const common::Pose D = A.inverse() * obs.E_last_;
      dx = D.p.x();
      dyaw_deg = wrap_deg(yaw_of(D.R));
    };
    in_w(obs.E_prior_, r.E_prior_dx, r.E_prior_dyaw_deg);
    in_w(test::box_T_e_w(p), r.E_true_dx, r.E_true_dyaw_deg);
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

// 95 % bootstrap CI of median(a_i - b_i) over paired seeds (2000 resamples, seed 7), as
// RelativeFactorSeeds.
std::pair<double, double> bootstrap_median_diff(const std::vector<double>& a,
                                                const std::vector<double>& b) {
  std::mt19937 rng(7U);
  std::uniform_int_distribution<std::size_t> pick(0, a.size() - 1);
  std::vector<double> medians;
  std::vector<double> d(a.size());
  for (int k = 0; k < 2000; ++k) {
    for (std::size_t i = 0; i < a.size(); ++i) {
      const std::size_t j = pick(rng);
      d[i] = a[j] - b[j];
    }
    medians.push_back(median_of(d));
  }
  std::sort(medians.begin(), medians.end());
  return {medians[50], medians[1949]};
}

std::string paired(const std::vector<double>& a, const std::vector<double>& b) {
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

std::string fmt(const char* f, double x) {
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
      {"frame_truth_init", "E initialised at the true T_e_w (force_T_e_w), loose prior", deg_on, "", "",
       true, "on"},
      {"frame_truth_fixed", "true T_e_w frozen (force_T_e_w, gnss_frame_variable false)", deg_on,
       "    backend:\n      gnss_frame_variable: false\n", "", true, "on"},
      {"oracle_shift", "fix shifted to its keyframe with the true displacement (test seam)", deg_on,
       "", "", false, "on", true},
      {"oracle_shift_truth_fixed", "oracle shift and true T_e_w frozen (both test seams)", deg_on,
       "    backend:\n      gnss_frame_variable: false\n", "", true, "on", true},
      {"oracle_shift_truth_init", "oracle shift and E initialised at the true T_e_w (loose prior)",
       deg_on, "", "", true, "on", true},
  };
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
          "Etrue_dyaw %.4f nees_gnss %.3f shift_rms %.4f shift_max %.4f shift_big %llu win",
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
          r.nees_gnss,
          r.shift_rms_x,
          r.shift_max_x,
          static_cast<unsigned long long>(r.n_shift_big));
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
        "| kf NEES x mean (> 3 sigma) | E - prior: x, yaw deg | E - truth: x, yaw deg | GNSS "
        "NEES vs truth |");
    rep.line(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
        "--- | --- | --- |");
    for (std::size_t si = 0; si < seeds.size(); ++si) {
      const SeedResult& r = res[vi][si];
      char b[640];
      std::snprintf(
          b,
          sizeof(b),
          "| %u | %.3f | %.3f | %.4f | %.4f | %.3f / %.3f | %.2f / %.2f | %llu | %llu | %llu | "
          "%llu (%llu) | %.3f | %+.3f / %.3f | %.1f (%.0f %%) | %+.2f, %+.2f | %+.2f, "
          "%+.2f | %.2f |",
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
          r.E_prior_dx,
          r.E_prior_dyaw_deg,
          r.E_true_dx,
          r.E_true_dyaw_deg,
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
}

}  // namespace
}  // namespace mss::pipeline
