// Component tests of the pipeline estimator on the synthetic box world
// (tests/component/box_world.hpp, docs/derivations/pipeline.md section 7): trajectory
// accuracy, allocation-free non-keyframe scans and the robustness cases of the plan's
// Review Focus list.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
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

#include "alloc_counter.hpp"
#include "box_world.hpp"
#include "estimator_core.hpp"

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Estimate;
using common::ImuSample;
using common::ScanFrame;
using common::Timestamp;
using common::Vec3;
using test::BoxWorldParams;

const fs::path kDefault = fs::path{MSS_REPO_DIR} / "config/default.yaml";
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr bool kWithGtsam = MSS_TEST_WITH_GTSAM != 0;

fs::path test_dir() {
  const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
  return fs::path{MSS_TEST_OUTPUT_DIR} / ("pipeline_" + name);
}

// config/default.yaml + the box-world overlay (written next to the dataset); `extra_yaml`:
// more sections under mss_node.ros__parameters; `gnss_keys`: more lines in the gnss section.
Config box_config(const BoxWorldParams& p,
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

Config box_config(const BoxWorldParams& p, bool gnss_and_wheel, const fs::path& dir) {
  return box_config(p, gnss_and_wheel, gnss_and_wheel, dir);
}

struct ReplayResult {
  std::vector<Estimate> estimates;  // one per new latest_state().state.t
  Estimator::Stats stats;
};

// Replays an mssdata directory in time order, run_once() after every pushed record
// (so before and after every scan push), flush() at the end. `after_scan(est, k)` (if set)
// runs after the k-th scan record (k from 1) has been pushed and drained.
ReplayResult replay(const fs::path& data,
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

Errors trajectory_errors(const BoxWorldParams& p, const std::vector<Estimate>& est) {
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
std::size_t expected_estimates(const BoxWorldParams& p, const Config& cfg) {
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
ReplayResult run_box_world(bool gnss_and_wheel,
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
      "%llu rebuilds\n",
      kWithGtsam ? 1 : 0,
      gnss_and_wheel ? 1 : 0,
      e.ate_m,
      e.max_yaw_deg,
      r.estimates.size(),
      static_cast<unsigned long long>(r.stats.n_keyframes),
      static_cast<unsigned long long>(r.stats.n_match_failures),
      static_cast<unsigned long long>(r.stats.n_gnss_factors),
      static_cast<unsigned long long>(r.stats.n_odom_factors),
      static_cast<unsigned long long>(r.stats.n_map_rebuilds));
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
BoxWorldParams corridor_params() {
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
const char* const kCorridorRelaxYaml =
    "    degeneracy:\n      enabled: true\n      relax_factor: 1.0e8\n";

// Replays `data` with GNSS (no wheel: wheel speed would observe the corridor axis) on or off;
// `gnss_keys`: more lines in the gnss section; `degeneracy_yaml`: the degeneracy section
// ("" = shipped defaults).
CorridorRun run_corridor(const BoxWorldParams& p,
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

// GNSS must help where the LiDAR drifts: LiDAR + IMU drifts by metres; GNSS (0.5 m, 5 Hz)
// must halve the ATE. The relative bound loosens when the LiDAR-only run gets worse, so the
// fused ATE also has an absolute guard: <= 1.2 x the 1.820 m measured on main 0d1df78
// (with scan_match.plane_min_eig_ratio 0.01 it was 5.2 m, map.md / scan_match.md).
TEST(PipelineComponent, CorridorDriftCorrectedByGnss) {
  const BoxWorldParams p = corridor_params();
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const CorridorRun lidar_only = run_corridor(p, dir / "data", dir / "lidar_only", false);
  const CorridorRun with_gnss = run_corridor(p, dir / "data", dir / "gnss", true);
  EXPECT_GE(lidar_only.ate_m, 0.3);
  if (kWithGtsam) {  // the stub backend adds no factor: nothing to correct with
    EXPECT_LE(with_gnss.ate_m, 0.5 * lidar_only.ate_m);
    constexpr double kMainFusedAteM = 1.820;  // m, CorridorDriftCorrectedByGnss on 0d1df78
    EXPECT_LE(with_gnss.ate_m, 1.2 * kMainFusedAteM);
    // About 300 fixes after the first estimate; a chi-square lock-out shows up here.
    EXPECT_GT(with_gnss.stats.n_gnss_factors, 200U);
  }
}

// Degeneracy at its shipped relax_factor. With the shipped degeneracy.enabled false this
// geometry gives a singular Hessian at every scan (no keyframe, nothing to fuse), so the
// run turns degeneracy on as PR 4b does by default.
const char* const kCorridorDefaultDegeneracyYaml = "    degeneracy:\n      enabled: true\n";

// Review I-1: the corridor with the shipped degeneracy covariance. The LiDAR factor claims
// millimetres along the unobservable axis, so GNSS cannot correct it (study section 8), but
// it must not make it worse: GNSS-on ATE <= 1.2 x LiDAR-only. The lock-out guard restarts
// the aligner here without any outage (1 restart at gnss.max_consecutive_rejects 60, 2 at
// 25): the front end drifts along the axis while its pose covariance P stays optimistic, so
// the chi-square gate rejects every fix and the guard fires as after an outage.
TEST(PipelineComponent, CorridorDefaultDegeneracyGnssDoesNotDegrade) {
  const BoxWorldParams p = corridor_params();
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const CorridorRun lidar_only =
      run_corridor(p, dir / "data", dir / "lidar_only", false, "", kCorridorDefaultDegeneracyYaml);
  const CorridorRun with_gnss =
      run_corridor(p, dir / "data", dir / "gnss", true, "", kCorridorDefaultDegeneracyYaml);
  EXPECT_GT(with_gnss.stats.n_gnss_factors, 0U);
  EXPECT_LE(with_gnss.ate_m, 1.2 * lidar_only.ate_m);
}

// Review B-1: a 20 s fix outage (20 s to 40 s) in the corridor with the shipped degeneracy
// covariance; 20 s exceeds the 12 s guard window (gnss.max_consecutive_rejects 60). The
// front end drifts along the axis and the chi-square gate rejects every fix after the
// outage; without the guard no factor is added again (lock-out), with it the aligner
// restarts and factors flow again (11 after the outage). These factor counts are the
// assertions. The ATEs are only printed (12.85 m vs 12.97 m): the margin is under 1 %
// because the IMU-only chain of this corridor drifts by metres along the axis (11.5 m ATE
// with LiDAR + IMU over the run), far beyond its own noise model (about 0.3 m in 20 s), a
// pre-existing defect recorded as a follow-up that a few fixes cannot bend back.
TEST(PipelineComponent, CorridorOutageRealignsAfterRejects) {
  BoxWorldParams p = corridor_params();
  p.gnss_outage_start_s = 20.0;
  p.gnss_outage_s = 20.0;
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const CorridorRun reset =
      run_corridor(p, dir / "data", dir / "reset", true, "", kCorridorDefaultDegeneracyYaml);
  const CorridorRun no_reset = run_corridor(p,
                                            dir / "data",
                                            dir / "no_reset",
                                            true,
                                            "      max_consecutive_rejects: 1000000000\n",
                                            kCorridorDefaultDegeneracyYaml);
  if (kWithGtsam) {
    EXPECT_GE(reset.stats.n_gnss_realign, 1U);
    EXPECT_GT(reset.gnss_factors_after_outage, 0U);
    EXPECT_EQ(no_reset.stats.n_gnss_realign, 0U);
    EXPECT_EQ(no_reset.gnss_factors_after_outage, 0U);  // locked out for good
    // ATEs are printed by run_corridor, not compared: the margin is under 1 % (see above).
  }
}

TEST(PipelineComponent, BoxWorldAteWithinTolerance) {
  run_box_world(false);
}

TEST(PipelineComponent, BoxWorldWithGnssAndWheelWithinTolerance) {
  run_box_world(true);
}

// Review I-3: GNSS alone (no wheel) at sigma 0.1 / 0.5 / 2.0 m must not degrade the LiDAR
// chain on the box world: GNSS-on ATE <= 1.2 x LiDAR-only ATE on the same data (study
// section 9: 1.00 to 1.01; with the fixed T_e_w 1.15 to 3.7).
TEST(PipelineComponent, BoxWorldGnssOnlyDoesNotDegrade) {
  for (const double sigma_m : {0.1, 0.5, 2.0}) {
    BoxWorldParams p;
    p.gnss_sigma_m = sigma_m;
    const fs::path dir = test_dir() / ("sigma_" + std::to_string(sigma_m));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const ReplayResult on = replay(dir / "data", box_config(p, true, false, dir / "gnss"));
    const ReplayResult off = replay(dir / "data", box_config(p, false, false, dir / "lidar"));
    const double ate_on_m = trajectory_errors(p, on.estimates).ate_m;
    const double ate_off_m = trajectory_errors(p, off.estimates).ate_m;
    std::printf(
        "[ box world ] GNSS only sigma %.1f m: ATE %.4f m, LiDAR only %.4f m (ratio "
        "%.2f), %llu gnss factors\n",
        sigma_m,
        ate_on_m,
        ate_off_m,
        ate_on_m / ate_off_m,
        static_cast<unsigned long long>(on.stats.n_gnss_factors));
    EXPECT_GT(on.stats.n_gnss_factors, 0U) << sigma_m;
    EXPECT_LE(ate_on_m, 1.2 * ate_off_m) << sigma_m;
  }
}

// I-1: GNSS stamps 2.5 ms off the 5 ms IMU grid are deferred until the IMU covers them and
// fused at the interpolated pose; a fix older than the last processed scan is rejected.
TEST(PipelineComponent, GnssOffGridFusedAndLateFixRejected) {
  BoxWorldParams p;
  p.gnss_offset_ns = 2'500'000;
  int late_pushed = 0;
  const ReplayResult r = run_box_world(true, p, [&](Estimator& est, int k) {
    if (k == 120) {  // well after alignment: a fix stamped 1 s in the past
      common::GnssMeasurement m;
      m.t = Timestamp{test::box_scan_time(p, k).ns - 1'000'000'000};
      m.p_e_g = test::box_T_e_w(p) * (test::box_truth(p, m.t).T_w_b * p.p_b_g);
      m.cov = common::Mat3::Identity() * (p.gnss_sigma_m * p.gnss_sigma_m);
      (void)est.push_gnss(m);
      ++late_pushed;
    }
  });
  EXPECT_EQ(late_pushed, 1);
  EXPECT_EQ(r.stats.n_gnss_late, 1U);
}

TEST(PipelineComponent, NoEstimateBeforeImuInit) {
  BoxWorldParams p;
  p.static_s = 4.0;
  p.ramp_s = 0.0;
  p.motion_s = 0.0;
  p.gyro_noise_density = 0.1 / std::sqrt(p.imu_rate_hz);  // std 0.1 rad/s per sample
  const fs::path dir = test_dir();
  const Config cfg = box_config(p, false, dir);
  ASSERT_GT(0.1, cfg.imu.init_max_gyro_std_radps);
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const ReplayResult r = replay(dir / "data", cfg);
  EXPECT_TRUE(r.estimates.empty());
  EXPECT_EQ(r.stats.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_EQ(r.stats.n_keyframes, 0U);
}

// Drives EstimatorCore directly in a static box world (noise-free IMU).
class StaticCore {
 public:
  // `aux`: gnss.enabled and wheel_odom.enabled, with GNSS (5 Hz, 1.234 ms off the IMU grid)
  // and wheel (50 Hz) messages merged into the stream by time.
  explicit StaticCore(bool aux = false)
      : aux_(aux), cfg_(make_cfg(aux)), core_(std::make_unique<detail::EstimatorCore>(cfg_)) {
    scan_.reserve(static_cast<std::size_t>(p_.rings * p_.columns));
    test::box_scan(p_, Timestamp{p_.t0_ns}, scan_);  // static: identical at every time
    empty_.reserve(1);
  }

  static Config make_cfg(bool aux = false) {
    const auto loaded = load_config(kDefault);
    EXPECT_TRUE(loaded.is_ok()) << loaded.reason();
    Config c = loaded.value();
    c.frames.T_b_l = test::box_default_T_b_l();
    c.frames.p_b_g = BoxWorldParams{}.p_b_g;
    c.gnss.enabled = aux;
    c.wheel_odom.enabled = aux;
    return c;
  }

  // IMU samples (static, exact) up to and including t; a sample whose time is in `nan_`
  // has NaN acc. With `aux`, wheel and GNSS messages earlier than each IMU sample first.
  void imu_until(std::int64_t t) {
    for (; next_imu_ <= t; next_imu_ += period_) {
      if (aux_) {
        for (; next_wheel_ < next_imu_; next_wheel_ += 20'000'000) {
          core_->on_wheel(test::box_wheel(p_, Timestamp{next_wheel_}));
        }
        for (; next_gnss_ < next_imu_; next_gnss_ += 200'000'000) {
          common::GnssMeasurement m;
          m.t = Timestamp{next_gnss_};
          m.p_e_g = test::box_T_e_w(p_) * (test::box_truth(p_, m.t).T_w_b * p_.p_b_g);
          m.cov = common::Mat3::Identity() * 0.25;
          core_->on_gnss(m);
        }
      }
      ImuSample s = test::box_imu(p_, Timestamp{next_imu_}, nullptr);
      for (const std::int64_t bad : nan_) {
        if (bad == next_imu_) {
          s.acc_b.x() = std::numeric_limits<double>::quiet_NaN();
        }
      }
      core_->on_imu(s);
    }
  }

  void set_nan_times(std::vector<std::int64_t> times) { nan_ = std::move(times); }

  // Scan at t (covered by IMU first).
  void scan_at(std::int64_t t) {
    scan_.t_start = Timestamp{t};
    // Two periods: a dropped (non-finite) sample right after the end cannot cover it,
    // as in Estimator's coverage gate, which counts finite samples only.
    imu_until(common::scan_end_time(scan_).ns + 2 * period_);
    core_->on_scan(scan_);
  }

  void empty_scan_at(std::int64_t t) {
    empty_.t_start = Timestamp{t};
    imu_until(t + period_);
    core_->on_scan(empty_);
  }

  std::int64_t t_scan(int k) const { return p_.t0_ns + 2'000'000'000 + k * 100'000'000; }

  detail::EstimatorCore& core() { return *core_; }
  std::int64_t period() const { return period_; }

 private:
  BoxWorldParams p_;
  bool aux_ = false;
  Config cfg_;
  std::unique_ptr<detail::EstimatorCore> core_;
  ScanFrame scan_;
  ScanFrame empty_;
  std::int64_t period_ = 5'000'000;
  std::int64_t next_imu_ = 1'000'000'000;
  std::int64_t next_wheel_ = 1'000'000'000;
  std::int64_t next_gnss_ = 1'001'234'000;
  std::vector<std::int64_t> nan_;
};

// I-3: also with gnss.enabled and wheel_odom.enabled and their messages on the path.
TEST(PipelineComponent, NonKeyframeScanDoesNotAllocate) {
  StaticCore s(true);
  s.scan_at(s.t_scan(0));  // initialisation + keyframe 0
  ASSERT_TRUE(s.core().has_estimate());
  for (int k = 1; k <= 5; ++k) {
    s.scan_at(s.t_scan(k));  // warm-up
  }
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 6; k < 16; ++k) {
      s.scan_at(s.t_scan(k));
    }
  }
  // This thread only: a TBB-enabled GTSAM starts worker threads after the first optimize()
  // that allocate asynchronously on their own threads.
  const std::size_t allocs = AllocCounter::count_this_thread();
  EXPECT_EQ(allocs, 0U);
  EXPECT_EQ(s.core().counters().n_keyframes, 1U);
  EXPECT_EQ(s.core().counters().n_match_failures, 0U);
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(15)});
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 0.02);  // static: matched near origin
}

// I-2: a NaN IMU sample inside a scan span and one exactly at a scan start are dropped;
// the neighbours bridge them and no scan is skipped.
TEST(PipelineComponent, NonFiniteImuSamplesDropped) {
  StaticCore s;
  s.set_nan_times({s.t_scan(1) + 50'000'000, s.t_scan(2)});
  s.scan_at(s.t_scan(0));
  ASSERT_TRUE(s.core().has_estimate());
  const std::uint64_t skipped = s.core().counters().n_skipped_scans;
  for (int k = 1; k <= 3; ++k) {
    s.scan_at(s.t_scan(k));
    EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(k)}) << k;
  }
  EXPECT_EQ(s.core().counters().n_skipped_scans, skipped);
  EXPECT_EQ(s.core().counters().n_nonfinite_imu, 2U);
  EXPECT_EQ(s.core().counters().n_match_failures, 0U);
  EXPECT_TRUE(s.core().latest().state.T_w_b.p.allFinite());
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 0.02);
}

TEST(PipelineComponent, ScanMatchFailureFallsBackToImuPrediction) {  // Review Focus 1
  StaticCore s;
  s.scan_at(s.t_scan(0));
  ASSERT_TRUE(s.core().has_estimate());
  for (int k = 1; k <= 5; ++k) {
    s.empty_scan_at(s.t_scan(k));
    EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(k)}) << k;
  }
  EXPECT_EQ(s.core().counters().n_match_failures, 5U);
  EXPECT_EQ(s.core().counters().n_keyframes, 1U);
  EXPECT_LT(s.core().latest().state.T_w_b.p.norm(), 1e-6);  // static IMU prediction
  s.scan_at(s.t_scan(6));                                   // a good scan matches again
  EXPECT_EQ(s.core().counters().n_match_failures, 5U);
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(6)});
}

// N-1: more than kMaxDeferredGnss fixes waiting for IMU overflow the ring (counted).
TEST(PipelineComponent, GnssRingOverflowCounted) {
  Config cfg = StaticCore::make_cfg(true);
  detail::EstimatorCore core(cfg);
  BoxWorldParams p;
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  const std::int64_t t_scan = p.t0_ns + 2'000'000'000;
  scan.t_start = Timestamp{t_scan};
  std::int64_t t = p.t0_ns;
  for (; t <= common::scan_end_time(scan).ns + 5'000'000; t += 5'000'000) {
    core.on_imu(test::box_imu(p, Timestamp{t}, nullptr));
  }
  core.on_scan(scan);
  ASSERT_TRUE(core.has_estimate());
  // Fixes after the last IMU sample: none can be fused yet.
  for (std::size_t k = 0; k < detail::kMaxDeferredGnss + 3; ++k) {
    common::GnssMeasurement m;
    m.t = Timestamp{t + static_cast<std::int64_t>(k) * 1'000'000};
    m.cov = common::Mat3::Identity() * 0.25;
    core.on_gnss(m);
  }
  EXPECT_EQ(core.counters().n_gnss_ring_overflow, 3U);
  core.on_end_of_data();
  EXPECT_EQ(core.counters().n_gnss_unfused_eof, detail::kMaxDeferredGnss);
}

// N-2: a fix stamped after the last IMU sample is still waiting at flush(): counted, so
// n_gnss = fused + rejected + late + overflow + unfused.
TEST(PipelineComponent, TrailingGnssFixCountedAtFlush) {
  BoxWorldParams p;
  Estimator est(StaticCore::make_cfg(true));
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  scan.t_start = Timestamp{p.t0_ns + 2'000'000'000};
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  std::int64_t t = p.t0_ns;
  for (; t <= common::scan_end_time(scan).ns + 5'000'000; t += 5'000'000) {
    ASSERT_TRUE(est.push_imu(test::box_imu(p, Timestamp{t}, nullptr)).is_ok());
    est.run_once();
  }
  ASSERT_NE(est.latest_state().state.t, Timestamp{});
  common::GnssMeasurement m;
  m.t = Timestamp{t + 1'000'000};  // after the last IMU sample
  m.cov = common::Mat3::Identity() * 0.25;
  ASSERT_TRUE(est.push_gnss(m).is_ok());
  est.flush();
  const Estimator::Stats st = est.stats();
  EXPECT_EQ(st.n_gnss, 1U);
  EXPECT_EQ(st.n_gnss_unfused_eof, 1U);
  EXPECT_EQ(st.n_gnss_ring_overflow, 0U);
  EXPECT_EQ(st.n_gnss_late, 0U);
}

TEST(PipelineComponent, OldScanSkipped) {  // Review Focus 3
  StaticCore s;
  s.scan_at(s.t_scan(0));
  s.scan_at(s.t_scan(1));
  const Estimate before = s.core().latest();
  const std::uint64_t skipped = s.core().counters().n_skipped_scans;
  s.scan_at(s.t_scan(1));  // duplicate t_start
  s.scan_at(s.t_scan(0));  // older
  const Estimate after = s.core().latest();
  EXPECT_EQ(s.core().counters().n_skipped_scans, skipped + 2);
  EXPECT_EQ(after.state.t, before.state.t);
  EXPECT_TRUE(after.state.T_w_b.R == before.state.T_w_b.R);
  EXPECT_TRUE(after.state.T_w_b.p == before.state.T_w_b.p);
  EXPECT_TRUE(after.state.v_w_b == before.state.v_w_b);
  EXPECT_TRUE(after.cov == before.cov);
  EXPECT_EQ(after.keyframe_id, before.keyframe_id);
  s.scan_at(s.t_scan(2));  // processing continues
  EXPECT_EQ(s.core().latest().state.t, Timestamp{s.t_scan(2)});
}

TEST(PipelineComponent, UncoveredScanSkippedAtFlush) {  // Review Focus 4
  BoxWorldParams p;
  Config cfg = StaticCore::make_cfg();
  Estimator est(cfg);
  ScanFrame scan(static_cast<std::size_t>(p.rings * p.columns));
  test::box_scan(p, Timestamp{p.t0_ns}, scan);
  const std::int64_t period = 5'000'000;
  std::int64_t t_imu = p.t0_ns;
  const auto imu_until = [&](std::int64_t t) {
    for (; t_imu <= t; t_imu += period) {
      ASSERT_TRUE(est.push_imu(test::box_imu(p, Timestamp{t_imu}, nullptr)).is_ok());
      est.run_once();  // keep the IMU queue (realtime.imu_queue_depth) from overflowing
    }
  };
  const std::int64_t t0 = p.t0_ns + 2'000'000'000;
  for (int k = 0; k < 2; ++k) {
    scan.t_start = Timestamp{t0 + k * 100'000'000};
    ASSERT_TRUE(est.push_scan(scan).is_ok());
    imu_until(common::scan_end_time(scan).ns + period);  // covers the scan end
    est.run_once();
  }
  ASSERT_EQ(est.latest_state().state.t, Timestamp{t0 + 100'000'000});
  scan.t_start = Timestamp{t0 + 200'000'000};
  ASSERT_TRUE(est.push_scan(scan).is_ok());
  imu_until(t0 + 250'000'000);  // IMU stops 50 ms into the last scan
  est.run_once();
  EXPECT_EQ(est.stats().n_scans, 2U);  // gated
  est.flush();
  const Estimator::Stats st = est.stats();
  EXPECT_EQ(st.n_scans, 3U);
  EXPECT_EQ(st.n_uncovered_scans, 1U);
  EXPECT_EQ(st.n_skipped_scans, 1U);
  EXPECT_EQ(est.latest_state().state.t, Timestamp{t0 + 100'000'000});
}

TEST(PipelineComponent, CoreConstructorValidatesBuffers) {
  Config cfg = StaticCore::make_cfg();
  cfg.imu.init_window_s = 30.0;  // 6000 samples at 200 Hz
  try {
    detail::EstimatorCore core(cfg);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("imu.init_window_s"), std::string::npos) << e.what();
  }
  Config cfg2 = StaticCore::make_cfg();
  cfg2.map.window_keyframes = static_cast<int>(cfg2.map.max_keyframes) + 1;
  try {
    detail::EstimatorCore core(cfg2);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("map.window_keyframes"), std::string::npos) << e.what();
  }
}

// ---------------------------------------------------------------------------------------
// GNSS fusion study (docs/derivations/gnss_fusion_study.md), skipped unless
// MSS_GNSS_STUDY=1 (MSS_GNSS_STUDY_LONG=1 for the 10-minute latency run). Writes
// results/<run-id>/gnss_study.md (gnss_long_run.md) under the repository; run id from
// MSS_GNSS_STUDY_RUN_ID (default "gnss-study"). Uses the test-only CoreTestHooks seam: a perfect
// T_e_w and the factor observer for the NEES and aligner tables.

double wrap_deg(double a_rad) {
  return std::remainder(a_rad, 2.0 * 3.14159265358979323846) * kRadToDeg;
}

double yaw_of(const common::Mat3& R) {
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

// Collects the factor statistics of one run against the box-world truth.
class StudyObserver final : public detail::CoreObserver {
 public:
  explicit StudyObserver(const BoxWorldParams& p) : p_(p), T_e_w_true_(test::box_T_e_w(p)) {}

  void on_lidar_factor(Timestamp t_i,
                       Timestamp t_j,
                       const common::Pose& T_bi_bj,
                       const common::PoseCovariance& cov_b) override {
    const common::Pose truth =
        test::box_truth(p_, t_i).T_w_b.inverse() * test::box_truth(p_, t_j).T_w_b;
    const common::Vec6 e = common::boxminus(T_bi_bj, truth);
    lidar_nees_ += e.dot(cov_b.ldlt().solve(e));
    lidar_err_p2_ += e.head<3>().squaredNorm();
    lidar_tr_p_ += cov_b.topLeftCorner<3, 3>().trace();
    ++n_lidar_;
  }

  void on_gnss_factor(Timestamp t_k,
                      const common::GnssMeasurement& m,
                      const common::Pose& T_e_w) override {
    const Vec3 p_w_g = test::box_truth(p_, t_k).T_w_b * p_.p_b_g;
    const Vec3 r_used = m.p_e_g - T_e_w * p_w_g;
    const Vec3 r_true = m.p_e_g - T_e_w_true_ * p_w_g;
    gnss_nees_used_ += r_used.dot(m.cov.ldlt().solve(r_used));
    gnss_nees_true_ += r_true.dot(m.cov.ldlt().solve(r_true));
    const double dyaw = wrap_deg(yaw_of(T_e_w.R) - yaw_of(T_e_w_true_.R));
    frozen_yaw2_ += dyaw * dyaw;
    ++n_gnss_;
  }

  void on_fix_accepted(Timestamp t, const common::Pose& T_e_w) override {
    const double dyaw = wrap_deg(yaw_of(T_e_w.R) - yaw_of(T_e_w_true_.R));
    const double dp = (T_e_w.p - T_e_w_true_.p).norm();
    if (!aligned_) {
      aligned_ = true;
      first_s_ = static_cast<double>(t.ns - p_.t0_ns) * 1e-9;
      first_yaw_deg_ = dyaw;
      first_p_m_ = dp;
    }
    last_yaw_deg_ = dyaw;
    last_p_m_ = dp;
  }

  [[nodiscard]] std::string summary() const {
    char buf[512];
    const double nl = n_lidar_ > 0 ? static_cast<double>(n_lidar_) : 1.0;
    const double ng = n_gnss_ > 0 ? static_cast<double>(n_gnss_) : 1.0;
    std::snprintf(buf,
                  sizeof(buf),
                  "%.2f | %.4f / %.4f | %.2f / %.2f | %s",
                  lidar_nees_ / nl,
                  std::sqrt(lidar_err_p2_ / nl),
                  std::sqrt(lidar_tr_p_ / nl),
                  gnss_nees_used_ / ng,
                  gnss_nees_true_ / ng,
                  aligned_ ? (std::to_string(first_s_).substr(0, 5) +
                              " s: " + std::to_string(first_yaw_deg_).substr(0, 6) + " deg, " +
                              std::to_string(first_p_m_).substr(0, 5) + " m; end " +
                              std::to_string(last_yaw_deg_).substr(0, 6) + " deg, " +
                              std::to_string(last_p_m_).substr(0, 5) + " m; frozen yaw RMS " +
                              std::to_string(std::sqrt(frozen_yaw2_ / ng)).substr(0, 5) + " deg")
                                 .c_str()
                           : "-");
    return buf;
  }

  [[nodiscard]] double lidar_nees() const {
    return n_lidar_ > 0 ? lidar_nees_ / static_cast<double>(n_lidar_) : 0.0;
  }

 private:
  BoxWorldParams p_;
  common::Pose T_e_w_true_;
  double lidar_nees_ = 0.0;
  double lidar_err_p2_ = 0.0;
  double lidar_tr_p_ = 0.0;
  std::size_t n_lidar_ = 0;
  double gnss_nees_used_ = 0.0;
  double gnss_nees_true_ = 0.0;
  double frozen_yaw2_ = 0.0;
  std::size_t n_gnss_ = 0;
  bool aligned_ = false;
  double first_s_ = 0.0;
  double first_yaw_deg_ = 0.0;
  double first_p_m_ = 0.0;
  double last_yaw_deg_ = 0.0;
  double last_p_m_ = 0.0;
};

// RMS position error after the best yaw + translation fit of the estimate to the truth
// (the aligner's closed form, gnss.md section 4): deformation only.
double ate_4dof(const BoxWorldParams& p, const std::vector<Estimate>& est) {
  if (est.empty()) {
    return 1e9;
  }
  Vec3 mean_a = Vec3::Zero();
  Vec3 mean_b = Vec3::Zero();
  for (const Estimate& x : est) {
    mean_a += x.state.T_w_b.p;
    mean_b += test::box_truth(p, x.state.t).T_w_b.p;
  }
  const double n = static_cast<double>(est.size());
  mean_a /= n;
  mean_b /= n;
  Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
  for (const Estimate& x : est) {
    const Vec3 a = x.state.T_w_b.p - mean_a;
    const Vec3 b = test::box_truth(p, x.state.t).T_w_b.p - mean_b;
    C += a.head<2>() * b.head<2>().transpose();
  }
  const common::Mat3 R = test::box_rz(std::atan2(C(0, 1) - C(1, 0), C(0, 0) + C(1, 1)));
  double sum_sq = 0.0;
  for (const Estimate& x : est) {
    const Vec3 a = x.state.T_w_b.p - mean_a;
    const Vec3 b = test::box_truth(p, x.state.t).T_w_b.p - mean_b;
    sum_sq += (R * a - b).squaredNorm();
  }
  return std::sqrt(sum_sq / n);
}

struct StudyCase {
  std::string name;
  bool gnss = false;
  bool wheel = false;
  bool perfect = false;         // force the true T_e_w (CoreTestHooks)
  bool frame_variable = false;  // backend.gnss_frame_variable
  std::string extra_yaml;       // more sections (not gnss, wheel_odom, backend)
  std::string backend_keys;     // more lines in the backend section
};

struct StudyResult {
  double ate_m = 0.0;
  double ate4_m = 0.0;
  Estimator::Stats stats;
  double lidar_nees = 0.0;
  std::string factors;
};

StudyResult study_run(const BoxWorldParams& p,
                      const fs::path& data,
                      const fs::path& dir,
                      const StudyCase& c) {
  const std::string backend = std::string("    backend:\n      gnss_frame_variable: ") +
                              (c.frame_variable ? "true" : "false") + "\n" + c.backend_keys;
  const Config cfg = box_config(p, c.gnss, c.wheel, dir / c.name, backend + c.extra_yaml);
  StudyObserver obs(p);
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  hooks.force_T_e_w = c.perfect;
  hooks.T_e_w = test::box_T_e_w(p);
  const HooksScope scope(&hooks);
  const ReplayResult r = replay(data, cfg);
  StudyResult out;
  out.ate_m = trajectory_errors(p, r.estimates).ate_m;
  out.ate4_m = ate_4dof(p, r.estimates);
  out.stats = r.stats;
  out.lidar_nees = obs.lidar_nees();
  out.factors = obs.summary();
  return out;
}

class StudyReport {
 public:
  explicit StudyReport(const std::string& file) {
    const char* id = std::getenv("MSS_GNSS_STUDY_RUN_ID");
    dir_ = fs::path{MSS_REPO_DIR} / "results" / (id != nullptr ? id : "gnss-study");
    fs::create_directories(dir_);
    out_.open(dir_ / file);
  }
  [[nodiscard]] const fs::path& dir() const { return dir_; }
  void line(const std::string& s) {
    out_ << s << "\n";
    std::printf("%s\n", s.c_str());
  }
  void header(const std::string& title) {
    line("");
    line("## " + title);
    line("");
    line(
        "| case | GNSS factors | ATE (m) | ATE 4-DoF (m) | rebuilds | LiDAR NEES (exp. 6) | "
        "LiDAR err / sqrt(tr cov_p) (m) | GNSS NEES used / true (exp. 3) | aligner |");
    line("| --- | --- | --- | --- | --- | --- | --- | --- | --- |");
  }
  void row(const std::string& name, const StudyResult& r) {
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %llu | %.4f | %.4f | %llu | ",
                  name.c_str(),
                  static_cast<unsigned long long>(r.stats.n_gnss_factors),
                  r.ate_m,
                  r.ate4_m,
                  static_cast<unsigned long long>(r.stats.n_map_rebuilds));
    line(std::string(buf) + r.factors + " |");
  }

 private:
  fs::path dir_;
  std::ofstream out_;
};

TEST(PipelineComponent, GnssFusionStudy) {
  const char* on = std::getenv("MSS_GNSS_STUDY");
  if (on == nullptr || std::string(on) != "1") {
    GTEST_SKIP() << "set MSS_GNSS_STUDY=1";
  }
  StudyReport rep("gnss_study.md");
  rep.line("# GNSS fusion study (generated by PipelineComponent.GnssFusionStudy)");
  const std::string tight = "      prior_pos_sigma_m: 0.001\n      prior_rot_sigma_deg: 0.01\n";
  const std::string cov5 = "    scan_match:\n      cov_scale: 5.0\n";
  const std::string rebuild0 =
      "    map:\n      rebuild_trans_m: 1.0e-6\n      rebuild_rot_deg: 1.0e-6\n";

  rep.header("Box world (sections 1 to 4, 9), fixed T_e_w unless 'frame variable'");
  for (const double sigma_m : {0.1, 0.5, 2.0}) {
    BoxWorldParams p;
    p.gnss_sigma_m = sigma_m;
    const std::string tag = "s" + std::to_string(sigma_m).substr(0, 3);
    const fs::path dir = rep.dir() / ("box_" + tag);
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    std::vector<StudyCase> cases;
    if (sigma_m == 0.5) {
      cases.push_back({"LiDAR only", false, false, false, false, "", ""});
      cases.push_back({"LiDAR + wheel", false, true, false, false, "", ""});
      cases.push_back({"GNSS + wheel, aligner", true, true, false, false, "", ""});
      cases.push_back({"GNSS + wheel, frame variable", true, true, false, true, "", ""});
      cases.push_back({"tight prior, LiDAR only", false, false, false, false, "", tight});
      cases.push_back({"tight prior, aligner", true, false, false, false, "", tight});
      cases.push_back({"tight prior, perfect", true, false, true, false, "", tight});
      cases.push_back({"cov_scale 5, LiDAR only", false, false, false, false, cov5, ""});
      cases.push_back({"cov_scale 5, aligner", true, false, false, false, cov5, ""});
      cases.push_back({"cov_scale 5, perfect", true, false, true, false, cov5, ""});
      cases.push_back(
          {"cov_scale 5 + tight prior, aligner", true, false, false, false, cov5, tight});
      cases.push_back({"rebuild thresholds ~0, aligner", true, false, false, false, rebuild0, ""});
      cases.push_back({"rebuild thresholds ~0, perfect", true, false, true, false, rebuild0, ""});
    }
    cases.push_back({"GNSS " + tag + ", aligner", true, false, false, false, "", ""});
    cases.push_back({"GNSS " + tag + ", perfect", true, false, true, false, "", ""});
    cases.push_back({"GNSS " + tag + ", frame variable", true, false, false, true, "", ""});
    for (StudyCase& c : cases) {
      const std::string label = c.name;
      c.name = tag + "_" + std::to_string(&c - cases.data());
      rep.row(label + " (" + tag + ")", study_run(p, dir / "data", dir, c));
    }
  }

  rep.header("Degenerate corridor (sections 8, 9)");
  for (const double motion_s : {20.0, 30.0, 60.0}) {
    BoxWorldParams p = corridor_params();
    p.motion_s = motion_s;
    const std::string tag = std::to_string(static_cast<int>(motion_s)) + " s";
    const fs::path dir = rep.dir() / ("corridor_" + std::to_string(static_cast<int>(motion_s)));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    for (const char* relax : {"10.0", "1.0e4", "1.0e8"}) {
      if (motion_s != 60.0 && std::string(relax) != "1.0e8") {
        continue;
      }
      const std::string yaml =
          std::string("    preprocess:\n      max_range_m: 10.0\n") +
          "    degeneracy:\n      enabled: true\n      relax_factor: " + relax + "\n";
      const std::vector<StudyCase> cases = {
          {"lidar", false, false, false, false, yaml, ""},
          {"aligner", true, false, false, false, yaml, ""},
          {"perfect", true, false, true, false, yaml, ""},
          {"framevar", true, false, false, true, yaml, ""},
      };
      for (StudyCase c : cases) {
        const std::string label = tag + ", relax " + relax + ", " + c.name;
        c.name = std::string("r") + relax + "_" + c.name;
        rep.row(label, study_run(p, dir / "data", dir, c));
      }
    }
  }
}

// Review I-7: optimize() cost with the frame variable E connected to every GNSS keyframe,
// on a 10-minute box-world drive. Per-scan wall time of run_once() (the call that processes
// a scan; keyframe scans include optimize()).
TEST(PipelineComponent, GnssFusionStudyLongRun) {
  const char* on = std::getenv("MSS_GNSS_STUDY_LONG");
  if (on == nullptr || std::string(on) != "1") {
    GTEST_SKIP() << "set MSS_GNSS_STUDY_LONG=1";
  }
  StudyReport rep("gnss_long_run.md");
  rep.line("# GNSS long run (generated by PipelineComponent.GnssFusionStudyLongRun)");
  rep.line("");
  rep.line(
      "| case | scans | keyframes | GNSS factors | ATE (m) | p50 (ms) | p99 (ms) | max (ms) |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- | --- |");
  BoxWorldParams p;
  p.motion_s = 600.0;
  const fs::path dir = rep.dir() / "long";
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  for (const bool frame_variable : {false, true}) {
    const std::string backend = std::string("    backend:\n      gnss_frame_variable: ") +
                                (frame_variable ? "true" : "false") + "\n";
    const Config cfg =
        box_config(p, true, false, dir / (frame_variable ? "framevar" : "fixed"), backend);
    auto opened = io::DatasetReader::open(dir / "data", cfg.realtime.max_points_per_scan);
    ASSERT_TRUE(opened.is_ok()) << opened.reason();
    io::DatasetReader reader = std::move(opened).value();
    Estimator est(cfg);
    ScanFrame scan(cfg.realtime.max_points_per_scan);
    io::Record rec;
    std::vector<double> ms;
    std::vector<Estimate> estimates;
    Timestamp last{};
    std::uint64_t n_scans = 0;
    for (;;) {
      const auto more = reader.next(rec, scan);
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
          (void)est.push_scan(scan);
          break;
        case io::RecordKind::kGt:
          break;
      }
      const auto t0 = std::chrono::steady_clock::now();
      est.run_once();
      const auto t1 = std::chrono::steady_clock::now();
      const Estimate e = est.latest_state();
      if (e.state.t != Timestamp{} && e.state.t != last) {
        estimates.push_back(e);
        last = e.state.t;
      }
      const std::uint64_t n = est.stats().n_scans;
      if (n != n_scans) {
        n_scans = n;
        ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      }
    }
    est.flush();
    std::sort(ms.begin(), ms.end());
    const auto pct = [&](double q) {
      return ms[std::min(ms.size() - 1,
                         static_cast<std::size_t>(q * static_cast<double>(ms.size())))];
    };
    const Estimator::Stats st = est.stats();
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %zu | %llu | %llu | %.4f | %.3f | %.3f | %.3f |",
                  frame_variable ? "frame variable" : "fixed T_e_w",
                  ms.size(),
                  static_cast<unsigned long long>(st.n_keyframes),
                  static_cast<unsigned long long>(st.n_gnss_factors),
                  trajectory_errors(p, estimates).ate_m,
                  pct(0.5),
                  pct(0.99),
                  ms.back());
    rep.line(buf);
  }
}

// Review I-5: with scan_match.cov_scale 4.7 (study section 3) the LiDAR factor covariance
// is consistent on the box world: mean NEES of the LiDAR between factors against the truth
// within [0.5, 2] x 6 (it was 28 with cov_scale 1).
TEST(PipelineComponent, LidarFactorCovarianceConsistent) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  StudyObserver obs(p);
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  const HooksScope scope(&hooks);
  const ReplayResult r = replay(dir / "data", box_config(p, false, dir / "cfg"));
  std::printf("[ box world ] LiDAR factor NEES %.2f over %llu keyframes\n",
              obs.lidar_nees(),
              static_cast<unsigned long long>(r.stats.n_keyframes));
  EXPECT_GE(obs.lidar_nees(), 3.0);
  EXPECT_LE(obs.lidar_nees(), 12.0);
}

}  // namespace
}  // namespace mss::pipeline
