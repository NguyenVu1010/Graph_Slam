#pragma once

// Shared helpers of the pipeline component tests (test_pipeline_component.cpp) and the IMU
// observability study (test_imu_only_study.cpp): config overlays, mssdata replay, trajectory
// errors, the corridor scenes, the test-hook scope and the study report writer.
// Header-only, test code only.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
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

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline::ct {

namespace fs = std::filesystem;
using common::Estimate;
using common::ImuSample;
using common::ScanFrame;
using common::Timestamp;
using common::Vec3;
using test::BoxWorldParams;

inline const fs::path kDefault = fs::path{MSS_REPO_DIR} / "config/default.yaml";
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr bool kWithGtsam = MSS_TEST_WITH_GTSAM != 0;

inline fs::path test_dir() {
  const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
  return fs::path{MSS_TEST_OUTPUT_DIR} / ("pipeline_" + name);
}

// config/default.yaml + the box-world overlay (written next to the dataset); `extra_yaml`:
// more sections under mss_node.ros__parameters; `gnss_keys`: more lines in the gnss section.
inline Config box_config(const BoxWorldParams& p,
                         bool gnss,
                         bool wheel,
                         const fs::path& dir,
                         const std::string& extra_yaml = "",
                         const std::string& gnss_keys = "") {
  fs::create_directories(dir);
  const fs::path overlay = dir / "overlay.yaml";
  {
    std::ofstream f(overlay);
    f << test::box_overlay_yaml(p, gnss, wheel, gnss_keys) << extra_yaml;
  }
  const auto cfg = load_config(kDefault, overlay);
  EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
  return cfg.value();
}

inline Config box_config(const BoxWorldParams& p, bool gnss_and_wheel, const fs::path& dir) {
  return box_config(p, gnss_and_wheel, gnss_and_wheel, dir);
}

struct ReplayResult {
  std::vector<Estimate> estimates;  // one per new latest_state().state.t
  Estimator::Stats stats;
};

// Replays an mssdata directory in time order, run_once() after every pushed record
// (so before and after every scan push), flush() at the end. `after_scan(est, k)` (if set)
// runs after the k-th scan record (k from 1) has been pushed and drained.
inline ReplayResult replay(const fs::path& data,
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

inline Errors trajectory_errors(const BoxWorldParams& p, const std::vector<Estimate>& est) {
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
inline std::size_t expected_estimates(const BoxWorldParams& p, const Config& cfg) {
  std::size_t n = 0;
  for (int k = 1; k <= test::box_num_scans(p); ++k) {
    if (test::box_scan_time(p, k).ns >= p.t0_ns + test::box_ns(cfg.imu.init_window_s)) {
      ++n;
    }
  }
  return n;
}

// Replays the box world; with `gnss_and_wheel` also replays the same dataset with the wheel
// only (GNSS off) and checks GNSS + wheel ATE <= 1.2 x wheel-only ATE: on a 2 m circle GNSS
// at 0.5 m cannot improve on the LiDAR chain, so it must not degrade it
// (gnss_fusion_study.md section 5; before ADR-019 the ratio was 4.2). `after_scan` as in
// replay().
inline ReplayResult run_box_world(bool gnss_and_wheel,
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
      "%llu rebuilds, %llu map points, %llu spacing replaced\n",
      kWithGtsam ? 1 : 0,
      gnss_and_wheel ? 1 : 0,
      e.ate_m,
      e.max_yaw_deg,
      r.estimates.size(),
      static_cast<unsigned long long>(r.stats.n_keyframes),
      static_cast<unsigned long long>(r.stats.n_match_failures),
      static_cast<unsigned long long>(r.stats.n_gnss_factors),
      static_cast<unsigned long long>(r.stats.n_odom_factors),
      static_cast<unsigned long long>(r.stats.n_map_rebuilds),
      static_cast<unsigned long long>(r.stats.n_map_points),
      static_cast<unsigned long long>(r.stats.n_map_spacing_replaced));
  EXPECT_GT(r.stats.n_map_points, 0U);
  if (gnss_and_wheel) {
    EXPECT_GT(r.stats.n_odom_factors, 0U);
    EXPECT_GT(r.stats.n_gnss_factors, 0U);
    const Config wheel_only = box_config(p, false, true, dir / "wheel_only");
    const Errors base = trajectory_errors(p, replay(dir / "data", wheel_only).estimates);
    std::printf("[ box world ] LiDAR + wheel (GNSS off) on the same data %.4f m (ratio %.2f)\n",
                base.ate_m,
                e.ate_m / base.ate_m);
    EXPECT_LE(e.ate_m, 1.2 * base.ate_m);
  }
  return r;
}

// Corridor drift scenario (gnss_fusion_study.md section 8): an almost straight 30 m drive
// in a 120 m corridor whose end walls are out of range, so the LiDAR has no information
// along it and degeneracy relaxes that direction (relax_factor 1e8, scenario value).
inline BoxWorldParams corridor_params() {
  BoxWorldParams p;
  p.box_half_m = Vec3(60.0, 2.5, 1.5);
  p.yaw_rate_radps = 0.0005;  // rad/s: 0.45 m lateral offset over the drive
  p.motion_s = 60.0;
  return p;
}

struct CorridorRun {
  double ate_m = 0.0;
  Estimator::Stats stats;
  std::uint64_t gnss_factors_after_outage = 0;
};

// Scenario degeneracy overlay: the LiDAR covariance along the unobservable corridor axis
// must be really large for GNSS to correct it (study section 8). PR 4b replaces this with
// its scan_match.unobserved_var_m2 / unobserved_var_rad2 keys when it rebases on this PR.
inline const char* const kCorridorRelaxYaml =
    "    degeneracy:\n      enabled: true\n      relax_factor: 1.0e8\n";

// Replays `data` with GNSS (no wheel: wheel speed would observe the corridor axis) on or off;
// `gnss_keys`: more lines in the gnss section; `degeneracy_yaml`: the degeneracy section
// ("" = shipped defaults).
inline CorridorRun run_corridor(const BoxWorldParams& p,
                                const fs::path& data,
                                const fs::path& dir,
                                bool gnss,
                                const std::string& gnss_keys = "",
                                const std::string& degeneracy_yaml = kCorridorRelaxYaml) {
  const Config cfg = box_config(p,
                                gnss,
                                false,
                                dir,
                                "    preprocess:\n      max_range_m: 10.0\n" + degeneracy_yaml,
                                gnss_keys);
  CorridorRun out;
  std::uint64_t at_outage_end = 0;
  const std::int64_t outage_end_ns =
      p.t0_ns + test::box_ns(p.gnss_outage_start_s + p.gnss_outage_s);
  const ReplayResult r = replay(data, cfg, [&](Estimator& est, int k) {
    if (test::box_scan_time(p, k).ns <= outage_end_ns) {
      at_outage_end = est.stats().n_gnss_factors;
    }
  });
  out.ate_m = trajectory_errors(p, r.estimates).ate_m;
  out.stats = r.stats;
  out.gnss_factors_after_outage = r.stats.n_gnss_factors - at_outage_end;
  std::printf(
      "[ corridor ] %s gnss %d: ATE %.3f m, %llu keyframes, %llu gnss factors (%llu after "
      "the outage), %llu realign\n",
      dir.filename().string().c_str(),
      gnss ? 1 : 0,
      out.ate_m,
      static_cast<unsigned long long>(r.stats.n_keyframes),
      static_cast<unsigned long long>(r.stats.n_gnss_factors),
      static_cast<unsigned long long>(out.gnss_factors_after_outage),
      static_cast<unsigned long long>(r.stats.n_gnss_realign));
  return out;
}

inline double wrap_deg(double a_rad) {
  return std::remainder(a_rad, 2.0 * 3.14159265358979323846) * kRadToDeg;
}

inline double yaw_of(const common::Mat3& R) {
  return std::atan2(R(1, 0), R(0, 0));
}

// Installs the test hooks for its lifetime, so a throw cannot leave a dangling pointer in
// the process-wide hook for later tests of this binary.
class HooksScope {
 public:
  explicit HooksScope(const detail::CoreTestHooks* hooks) { detail::set_core_test_hooks(hooks); }
  ~HooksScope() { detail::set_core_test_hooks(nullptr); }
  HooksScope(const HooksScope&) = delete;
  HooksScope& operator=(const HooksScope&) = delete;
};

class StudyReport {
 public:
  explicit StudyReport(const std::string& file,
                       const char* run_id_env = "MSS_GNSS_STUDY_RUN_ID",
                       const char* default_id = "gnss-study") {
    const char* id = std::getenv(run_id_env);
    dir_ = fs::path{MSS_REPO_DIR} / "results" / (id != nullptr ? id : default_id);
    fs::create_directories(dir_);
    out_.open(dir_ / file);
  }
  [[nodiscard]] const fs::path& dir() const { return dir_; }
  void line(const std::string& s) {
    out_ << s << "\n";
    std::printf("%s\n", s.c_str());
  }

 private:
  fs::path dir_;
  std::ofstream out_;
};

// The box-world circle (PR 4b's corridor scene) or an almost straight run, both 20 s.
inline BoxWorldParams imu_corridor_params(bool straight) {
  BoxWorldParams p;
  p.box_half_m.x() = 1000.0;  // m, beyond preprocess.max_range_m (100 m): x walls never seen
  if (straight) {
    p.box_half_m.y() = 2.5;     // m, as corridor_params()
    p.yaw_rate_radps = 0.0005;  // rad/s, as corridor_params()
  }
  return p;
}

// Analytic 1-sigma, per horizontal axis, of inertial dead reckoning from rest after `T_s`
// (imu_only_observability.md section 1): velocity random walk q_a T^1.5 / sqrt(3), and the
// static-initialisation tilt, a horizontal acceleration error q_a / sqrt(T_init) (mean of
// the init window over gravity), giving 0.5 q_a / sqrt(T_init) T^2.
inline double dr_sigma_vrw_m(double q_a, double T_s) {
  return q_a * std::pow(T_s, 1.5) / std::sqrt(3.0);
}
inline double dr_sigma_tilt_m(double q_a, double T_init_s, double T_s) {
  return 0.5 * q_a / std::sqrt(T_init_s) * T_s * T_s;
}
inline double dr_sigma_m(double q_a, double T_init_s, double T_s) {
  return std::hypot(dr_sigma_vrw_m(q_a, T_s), dr_sigma_tilt_m(q_a, T_init_s, T_s));
}
// Free-inertial only (attitude from the gyro): the static-init gyro bias error
// q_g / sqrt(T_init) tilts the attitude linearly, g (q_g / sqrt(T_init)) T^3 / 6, and the
// angle random walk gives g q_g T^2.5 / sqrt(20).
inline double dr_sigma_gyro_m(double q_g, double T_init_s, double T_s) {
  return test::kBoxGravity * std::hypot(q_g / std::sqrt(T_init_s) * std::pow(T_s, 3) / 6.0,
                                        q_g * std::pow(T_s, 2.5) / std::sqrt(20.0));
}

// ---------------------------------------------------------------------------------------
// PR 4d (imu_only_observability.md section 9): the corridor of the study (box-world circle,
// x walls out of range), LiDAR + IMU only, with the LiDAR factor carrying no information
// along the corridor axis (degeneracy on, relax_factor 1e8: the "unobserved" semantics), so
// only the IMU carries world x.

struct AxisRun {
  double end_x_m = 0.0;      // published estimate at the last scan, x error
  double along_rms_m = 0.0;  // RMS x error of every published estimate
  double lateral_rms_m = 0.0;
  double T_s = 0.0;            // s, time of the last estimate after the static initialisation
  double init_window_s = 0.0;  // s, imu.init_window_s of the run's config
  // max over estimates with T >= 2 s of |x err(t)| / sigma_DR(t) (dr_sigma_m), its time after
  // the initialisation and the published tilt error then (about w x / y, norm, rad).
  double max_ratio = 0.0;
  double t_at_max_s = 0.0;
  double tilt_at_max_rad = 0.0;
  std::uint64_t keyframes = 0;
  std::uint64_t gravity_removed = 0;    // n_gravity_factor_removed
  std::uint64_t gravity_unchecked = 0;  // n_gravity_factor_unchecked
};

inline AxisRun corridor_axis_run(const BoxWorldParams& p,
                                 const fs::path& data,
                                 const fs::path& dir,
                                 bool wheel,
                                 const std::string& extra_yaml) {
  const Config cfg =
      box_config(p,
                 false,
                 wheel,
                 dir,
                 "    degeneracy:\n      enabled: true\n      relax_factor: 1.0e8\n" + extra_yaml);
  const ReplayResult r = replay(data, cfg);
  AxisRun out;
  out.init_window_s = cfg.imu.init_window_s;
  double sx = 0.0;
  double sy = 0.0;
  for (const Estimate& x : r.estimates) {
    const common::NavState truth = test::box_truth(p, x.state.t);
    const Vec3 d = x.state.T_w_b.p - truth.T_w_b.p;
    sx += d.x() * d.x();
    sy += d.y() * d.y();
    const double T = static_cast<double>(x.state.t.ns - p.t0_ns) * 1e-9 - cfg.imu.init_window_s;
    if (T >= 2.0) {
      const double ratio =
          std::abs(d.x()) / dr_sigma_m(p.acc_noise_density, cfg.imu.init_window_s, T);
      if (ratio > out.max_ratio) {
        out.max_ratio = ratio;
        out.t_at_max_s = T;
        out.tilt_at_max_rad =
            common::so3_log(x.state.T_w_b.R * truth.T_w_b.R.transpose()).head<2>().norm();
      }
    }
  }
  const double n = std::max(1.0, static_cast<double>(r.estimates.size()));
  out.along_rms_m = std::sqrt(sx / n);
  out.lateral_rms_m = std::sqrt(sy / n);
  if (!r.estimates.empty()) {
    const Estimate& e = r.estimates.back();
    out.end_x_m = e.state.T_w_b.p.x() - test::box_truth(p, e.state.t).T_w_b.p.x();
    out.T_s = static_cast<double>(e.state.t.ns - p.t0_ns) * 1e-9 - cfg.imu.init_window_s;
  }
  out.keyframes = r.stats.n_keyframes;
  out.gravity_removed = r.stats.n_gravity_factor_removed;
  out.gravity_unchecked = r.stats.n_gravity_factor_unchecked;
  std::printf(
      "[ corridor axis ] %s: end x %+.3f m, along-x RMS %.4f m, lateral RMS %.4f m, max "
      "|x|/sigma_DR %.1f at T %.1f s (tilt %.2f mrad), %llu keyframes\n",
      dir.filename().string().c_str(),
      out.end_x_m,
      out.along_rms_m,
      out.lateral_rms_m,
      out.max_ratio,
      out.t_at_max_s,
      out.tilt_at_max_rad * 1e3,
      static_cast<unsigned long long>(out.keyframes));
  return out;
}

}  // namespace mss::pipeline::ct
