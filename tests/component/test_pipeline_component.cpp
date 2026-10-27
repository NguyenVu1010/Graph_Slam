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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/initializer.hpp"
#include "mss/imu/preintegrator.hpp"
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

// Stats map counters: spacing off never replaces; spacing on (0.1 m) refreshes map points
// from later keyframes, so the count grows, and the map holds fewer points.
TEST(PipelineComponent, MapSpacingCountersInStats) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  Config cfg = box_config(p, false, false, dir);
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  cfg.map.min_point_dist_m = 0.0;
  const Estimator::Stats off = replay(dir / "data", cfg).stats;
  cfg.map.min_point_dist_m = 0.1;
  const Estimator::Stats on = replay(dir / "data", cfg).stats;
  std::printf(
      "[ map counters ] off: %llu points, %llu replaced; 0.1 m: %llu points, %llu replaced\n",
      static_cast<unsigned long long>(off.n_map_points),
      static_cast<unsigned long long>(off.n_map_spacing_replaced),
      static_cast<unsigned long long>(on.n_map_points),
      static_cast<unsigned long long>(on.n_map_spacing_replaced));
  EXPECT_GT(off.n_map_points, 0U);
  EXPECT_EQ(off.n_map_spacing_replaced, 0U);
  EXPECT_GT(on.n_map_spacing_replaced, 0U);
  EXPECT_LT(on.n_map_points, off.n_map_points);
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

// ---------------------------------------------------------------------------------------
// IMU-only observability study (docs/derivations/imu_only_observability.md), skipped unless
// MSS_IMU_STUDY=1 (GTSAM build). Writes results/<run-id>/imu_study.md under the repository
// (run id from MSS_IMU_STUDY_RUN_ID, default "imu-study"). Corridor: the box world with its
// x walls beyond preprocess.max_range_m, LiDAR + IMU only (no wheel, no GNSS), so only the
// IMU carries the world x axis. Uses the test-only CoreTestHooks seam (keyframe period,
// front end without backend feedback, per-scan and per-keyframe observer).

// The box-world circle (PR 4b's corridor scene) or an almost straight run, both 20 s.
BoxWorldParams imu_corridor_params(bool straight) {
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
double dr_sigma_vrw_m(double q_a, double T_s) {
  return q_a * std::pow(T_s, 1.5) / std::sqrt(3.0);
}
double dr_sigma_tilt_m(double q_a, double T_init_s, double T_s) {
  return 0.5 * q_a / std::sqrt(T_init_s) * T_s * T_s;
}
double dr_sigma_m(double q_a, double T_init_s, double T_s) {
  return std::hypot(dr_sigma_vrw_m(q_a, T_s), dr_sigma_tilt_m(q_a, T_init_s, T_s));
}
// Free-inertial only (attitude from the gyro): the static-init gyro bias error
// q_g / sqrt(T_init) tilts the attitude linearly, g (q_g / sqrt(T_init)) T^3 / 6, and the
// angle random walk gives g q_g T^2.5 / sqrt(20).
double dr_sigma_gyro_m(double q_g, double T_init_s, double T_s) {
  return test::kBoxGravity * std::hypot(q_g / std::sqrt(T_init_s) * std::pow(T_s, 3) / 6.0,
                                        q_g * std::pow(T_s, 2.5) / std::sqrt(20.0));
}

// Pure inertial dead reckoning of the box-world IMU stream of seed `seed`: the pipeline's
// ImuInitializer on [t0, t0 + init_window_s], then integration from rest (p = 0, v = 0, bias
// from the initializer) with one Preintegrator; world error (estimate - truth) at
// `times_s` after t0 + init_window_s (multiples of the IMU period). With `true_attitude`
// the attitude is the truth at every sample (what LiDAR aiding gives, section 1): Euler
// integration of a_w = R_w_b(t_k) (acc_b - ba) + g_w held over [t_{k-1}, t_k].
std::vector<Vec3> dead_reckoning_errors(BoxWorldParams p,
                                        const Config& cfg,
                                        std::uint32_t seed,
                                        const std::vector<double>& times_s,
                                        bool true_attitude = false) {
  p.seed = seed;
  std::mt19937 rng(p.seed);
  const std::int64_t period_ns = test::box_ns(1.0 / p.imu_rate_hz);
  const std::int64_t t_init_ns = p.t0_ns + test::box_ns(cfg.imu.init_window_s);
  std::vector<ImuSample> window;
  std::int64_t t = p.t0_ns;
  for (; t <= t_init_ns; t += period_ns) {
    window.push_back(test::box_imu(p, Timestamp{t}, &rng));
  }
  imu::ImuInitializer init(cfg.imu);
  const auto st = init.try_init(common::Span<const ImuSample>(window.data(), window.size()));
  EXPECT_TRUE(st.is_ok()) << st.reason();
  common::NavState x0;
  x0.t = window.back().t;
  x0.T_w_b.R = st.value().R_w_b;
  x0.bias = st.value().bias;
  if (true_attitude) {
    Vec3 v = Vec3::Zero();
    Vec3 pos = Vec3::Zero();
    const double dt = 1.0 / p.imu_rate_hz;
    std::vector<Vec3> out;
    for (const double ts : times_s) {
      const std::int64_t target = x0.t.ns + test::box_ns(ts);
      for (; t <= target; t += period_ns) {
        const ImuSample s = test::box_imu(p, Timestamp{t}, &rng);
        const Vec3 a_w = test::box_truth(p, s.t).T_w_b.R * (s.acc_b - x0.bias.ba) + st.value().g_w;
        pos += v * dt + 0.5 * dt * dt * a_w;
        v += a_w * dt;
      }
      out.push_back(pos - test::box_truth(p, Timestamp{t - period_ns}).T_w_b.p);
    }
    return out;
  }
  imu::Preintegrator pre(cfg.imu);
  pre.reset(x0.bias);
  (void)pre.integrate(window.back());
  std::vector<Vec3> out;
  for (const double ts : times_s) {
    const std::int64_t target = x0.t.ns + test::box_ns(ts);
    for (; t <= target; t += period_ns) {
      (void)pre.integrate(test::box_imu(p, Timestamp{t}, &rng));
    }
    const common::NavState x = pre.predict(x0, st.value().g_w);
    out.push_back(x.T_w_b.p - test::box_truth(p, x.t).T_w_b.p);
  }
  return out;
}

// Per scan: the matcher's correction along world x and the Hessian information along world
// x. Per LiDAR factor: the error of its measured translation along world x against the
// truth and the variance its covariance claims there. Per keyframe: the backend state
// after optimize().
class ImuStudyObserver final : public detail::CoreObserver {
 public:
  struct Keyframe {
    Timestamp t;
    common::NavState x;
    common::PoseCovariance cov;
  };
  explicit ImuStudyObserver(const BoxWorldParams& p) : p_(p) {}

  void on_scan_matched(Timestamp /*t*/,
                       const common::Pose& T_w_b_pred,
                       const common::Pose& T_w_b_match,
                       const common::MatchResult& match) override {
    const double dx = T_w_b_match.p.x() - T_w_b_pred.p.x();
    corr_x2_ += dx * dx;
    const Vec3 d_l = match.T_w_l.R.transpose() * Vec3::UnitX();
    info_x_.push_back(d_l.dot(match.H.topLeftCorner<3, 3>() * d_l));  // 1/m^2 (H: sigma 1 m)
    // Angle between the weakest translation eigenvector of H and world x (rad).
    const Eigen::SelfAdjointEigenSolver<common::Mat3> es(match.H.topLeftCorner<3, 3>());
    tilt_.push_back(std::acos(std::min(1.0, std::abs(es.eigenvectors().col(0).dot(d_l)))));
  }

  void on_lidar_factor(Timestamp t_i,
                       Timestamp t_j,
                       const common::Pose& T_bi_bj,
                       const common::PoseCovariance& cov_b) override {
    const common::NavState xi = test::box_truth(p_, t_i);
    const common::NavState xj = test::box_truth(p_, t_j);
    // Translation error of the measurement in w, and its covariance (rho of b_j, rotated
    // into w with the true attitude: first order).
    const Vec3 t_true = xi.T_w_b.R.transpose() * (xj.T_w_b.p - xi.T_w_b.p);
    const double ex = (xi.T_w_b.R * (T_bi_bj.p - t_true)).x();
    const common::Mat3 C_w = xj.T_w_b.R * cov_b.topLeftCorner<3, 3>() * xj.T_w_b.R.transpose();
    const double var_x = C_w(0, 0);
    nees_x_ += ex * ex / var_x;
    const common::Vec6 e6 = common::boxminus(T_bi_bj, xi.T_w_b.inverse() * xj.T_w_b);
    nees6_ += e6.dot(cov_b.ldlt().solve(e6));
    err_x2_ += ex * ex;
    sigma_x_.push_back(std::sqrt(var_x));
  }

  void on_keyframe_optimized(Timestamp t,
                             const common::NavState& x,
                             const common::PoseCovariance& cov) override {
    keyframes_.push_back({t, x, cov});
  }

  [[nodiscard]] double corr_x_rms_m() const {
    return info_x_.empty() ? 0.0 : std::sqrt(corr_x2_ / static_cast<double>(info_x_.size()));
  }
  // Median information along world x per scan, 1/m^2 (H as built, unit residual sigma).
  [[nodiscard]] double median_info_x() const { return median(info_x_); }
  // LiDAR factors: "NEES_x (exp. 1) | err RMS (m) | median claimed sigma (m)".
  [[nodiscard]] std::string factor_x() const {
    const double n = sigma_x_.empty() ? 1.0 : static_cast<double>(sigma_x_.size());
    char buf[160];
    std::snprintf(buf,
                  sizeof(buf),
                  "%.3g | %.4f | %.3g | %.3g | %.3g",
                  nees_x_ / n,
                  std::sqrt(err_x2_ / n),
                  median(sigma_x_),
                  nees6_ / n,
                  median(tilt_));
    return buf;
  }
  [[nodiscard]] const std::vector<Keyframe>& keyframes() const { return keyframes_; }

 private:
  static double median(std::vector<double> v) {
    if (v.empty()) {
      return 0.0;
    }
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
    return v[v.size() / 2];
  }

  BoxWorldParams p_;
  double corr_x2_ = 0.0;
  std::vector<double> info_x_;
  double nees_x_ = 0.0;
  double err_x2_ = 0.0;
  std::vector<double> sigma_x_;
  double nees6_ = 0.0;
  std::vector<double> tilt_;
  std::vector<Keyframe> keyframes_;
};

struct ImuStudyCase {
  std::string name;
  std::string yaml;              // more sections (not gnss, wheel_odom)
  int keyframe_every_n = 0;      // CoreTestHooks::keyframe_every_n_scans
  bool own_state = false;        // CoreTestHooks::frontend_keeps_own_state
  bool bias_trajectory = false;  // also print the backend bias / velocity per keyframe
  bool oracle = false;           // CoreTestHooks::lidar_oracle = true relative pose
  bool x_removed = false;        // CoreTestHooks::lidar_cov: no information along world x
};

// One corridor run; one table row (columns in ImuOnlyObservabilityStudy's header). With
// `bias_rows`, appends the backend keyframe states about every 3 s.
std::string imu_study_run(const BoxWorldParams& p,
                          const fs::path& data,
                          const fs::path& dir,
                          const ImuStudyCase& c,
                          std::string* bias_rows) {
  const Config cfg = box_config(p, false, false, dir, c.yaml);
  ImuStudyObserver obs(p);
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  hooks.keyframe_every_n_scans = c.keyframe_every_n;
  hooks.frontend_keeps_own_state = c.own_state;
  if (c.x_removed) {
    hooks.lidar_cov = [&p](Timestamp t_j, const common::PoseCovariance& cov) {
      // + (1 km)^2 along world x in b_j: the information along x vanishes, the rest stays.
      const Vec3 d_b = test::box_truth(p, t_j).T_w_b.R.transpose() * Vec3::UnitX();
      common::PoseCovariance out = cov;
      out.topLeftCorner<3, 3>() += 1.0e6 * d_b * d_b.transpose();
      return out;
    };
  }
  if (c.oracle) {
    hooks.lidar_oracle = [&p](Timestamp t_i, Timestamp t_j) {
      return test::box_truth(p, t_i).T_w_b.inverse() * test::box_truth(p, t_j).T_w_b;
    };
  }
  const HooksScope scope(&hooks);
  const ReplayResult r = replay(data, cfg);
  double sx = 0.0;
  double sy = 0.0;
  for (const Estimate& x : r.estimates) {
    const Vec3 d = x.state.T_w_b.p - test::box_truth(p, x.state.t).T_w_b.p;
    sx += d.x() * d.x();
    sy += d.y() * d.y();
  }
  const double n = std::max(1.0, static_cast<double>(r.estimates.size()));
  const double t_init_s = cfg.imu.init_window_s;
  double end_x = 0.0;
  double T_s = 0.0;
  if (!r.estimates.empty()) {
    const Estimate& e = r.estimates.back();
    end_x = e.state.T_w_b.p.x() - test::box_truth(p, e.state.t).T_w_b.p.x();
    T_s = static_cast<double>(e.state.t.ns - p.t0_ns) * 1e-9 - t_init_s;
  }
  const double sigma_dr = dr_sigma_m(p.acc_noise_density, t_init_s, T_s);
  double kf_x = 0.0;
  double kf_sx = 0.0;
  double kf_vx = 0.0;
  Vec3 kf_ba = Vec3::Zero();
  if (!obs.keyframes().empty()) {
    const ImuStudyObserver::Keyframe& k = obs.keyframes().back();
    const common::NavState truth = test::box_truth(p, k.t);
    kf_x = k.x.T_w_b.p.x() - truth.T_w_b.p.x();
    const common::Mat3 cov_p_w =
        k.x.T_w_b.R * k.cov.topLeftCorner<3, 3>() * k.x.T_w_b.R.transpose();
    kf_sx = std::sqrt(std::max(0.0, cov_p_w(0, 0)));
    kf_vx = k.x.v_w_b.x() - truth.v_w_b.x();
    kf_ba = k.x.bias.ba;
  }
  if (bias_rows != nullptr) {
    double next_s = 4.0;
    for (std::size_t i = 0; i < obs.keyframes().size(); ++i) {
      const ImuStudyObserver::Keyframe& k = obs.keyframes()[i];
      const double ts = static_cast<double>(k.t.ns - p.t0_ns) * 1e-9;
      if (ts < next_s && i + 1 != obs.keyframes().size()) {
        continue;
      }
      next_s = ts + 3.0;
      const common::NavState truth = test::box_truth(p, k.t);
      const Vec3 ba_w = k.x.T_w_b.R * k.x.bias.ba;
      // Attitude error in w (rad): x / y components are the tilt (roll / pitch in w).
      const Vec3 dth_w = common::so3_log(k.x.T_w_b.R * truth.T_w_b.R.transpose());
      char buf[256];
      std::snprintf(buf,
                    sizeof(buf),
                    "| %s | %.1f | %+.4f | %+.4f | %+.5f / %+.5f / %+.5f | %+.5f / %+.5f | "
                    "%+.2f / %+.2f |\n",
                    c.name.c_str(),
                    ts,
                    k.x.T_w_b.p.x() - truth.T_w_b.p.x(),
                    k.x.v_w_b.x() - truth.v_w_b.x(),
                    k.x.bias.ba.x(),
                    k.x.bias.ba.y(),
                    k.x.bias.ba.z(),
                    ba_w.x(),
                    ba_w.y(),
                    1e3 * dth_w.x(),
                    1e3 * dth_w.y());
      *bias_rows += buf;
    }
  }
  char buf[512];
  std::snprintf(
      buf,
      sizeof(buf),
      "| %s | %.3f | %+.3f | %.1f | %.4f | %llu | %llu | %.4f | %.3g | %s | %+.3f (%.3f) | %+.4f | "
      "%+.4f / %+.4f |",
      c.name.c_str(),
      std::sqrt(sx / n),
      end_x,
      std::abs(end_x) / sigma_dr,
      std::sqrt(sy / n),
      static_cast<unsigned long long>(r.stats.n_keyframes),
      static_cast<unsigned long long>(r.stats.n_match_failures),
      obs.corr_x_rms_m(),
      obs.median_info_x(),
      obs.factor_x().c_str(),
      kf_x,
      kf_sx,
      kf_vx,
      kf_ba.x(),
      kf_ba.y());
  return buf;
}

TEST(PipelineComponent, ImuOnlyObservabilityStudy) {
  const char* on = std::getenv("MSS_IMU_STUDY");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_IMU_STUDY=1 (GTSAM build)";
  }
  StudyReport rep("imu_study.md", "MSS_IMU_STUDY_RUN_ID", "imu-study");
  rep.line(
      "# IMU-only observability study (generated by "
      "PipelineComponent.ImuOnlyObservabilityStudy)");

  // 1. Dead reckoning reference: free inertial (attitude from the gyro) and attitude-aided
  // (true attitude, the LiDAR-aided case), Monte Carlo over 200 seeds.
  {
    const BoxWorldParams p = imu_corridor_params(false);
    const Config cfg = box_config(p, false, false, rep.dir() / "dr");
    const std::vector<double> times = {5.0, 10.0, 15.0, 17.0};
    const int n_seeds = 200;
    std::vector<double> free2(times.size(), 0.0);
    std::vector<double> aided2(times.size(), 0.0);
    std::vector<Vec3> free42;
    std::vector<Vec3> aided42;
    for (int s = 0; s < n_seeds; ++s) {
      const auto seed = static_cast<std::uint32_t>(42 + s);
      const std::vector<Vec3> e = dead_reckoning_errors(p, cfg, seed, times);
      const std::vector<Vec3> a = dead_reckoning_errors(p, cfg, seed, times, true);
      if (s == 0) {
        free42 = e;
        aided42 = a;
      }
      for (std::size_t k = 0; k < times.size(); ++k) {
        free2[k] += 0.5 * e[k].head<2>().squaredNorm();  // per horizontal axis
        aided2[k] += 0.5 * a[k].head<2>().squaredNorm();
      }
    }
    rep.line("");
    rep.line("## 1. Dead reckoning (no LiDAR), circle scene, q_a = " +
             std::to_string(p.acc_noise_density) +
             " m/s^2/sqrt(Hz), q_g = " + std::to_string(p.gyro_noise_density) + " rad/s/sqrt(Hz)");
    rep.line("");
    rep.line(
        "| T after init (s) | free: seed 42 err x (m) | free: MC RMS per axis (m) | aided: seed "
        "42 err x (m) | aided: MC RMS per axis (m) | analytic VRW (m) | analytic acc tilt (m) "
        "| analytic VRW + acc tilt (m) | analytic gyro (m) |");
    rep.line("| --- | --- | --- | --- | --- | --- | --- | --- | --- |");
    for (std::size_t k = 0; k < times.size(); ++k) {
      char buf[256];
      std::snprintf(buf,
                    sizeof(buf),
                    "| %.0f | %+.3f | %.3f | %+.3f | %.3f | %.3f | %.3f | %.3f | %.3f |",
                    times[k],
                    free42[k].x(),
                    std::sqrt(free2[k] / n_seeds),
                    aided42[k].x(),
                    std::sqrt(aided2[k] / n_seeds),
                    dr_sigma_vrw_m(p.acc_noise_density, times[k]),
                    dr_sigma_tilt_m(p.acc_noise_density, cfg.imu.init_window_s, times[k]),
                    dr_sigma_m(p.acc_noise_density, cfg.imu.init_window_s, times[k]),
                    dr_sigma_gyro_m(p.gyro_noise_density, cfg.imu.init_window_s, times[k]));
      rep.line(buf);
    }
  }

  // 2 to 7. Pipeline runs. Settings: the matcher's degeneracy handling and the LiDAR factor's
  // information along world x (shipped, relaxed by 1e8 along flagged directions, or exactly
  // removed along the true world x by the study hook).
  struct Setting {
    std::string name;
    std::string yaml;
    bool x_removed;
  };
  const std::string deg_off = "    degeneracy:\n      enabled: false\n";
  const std::string deg_on = "    degeneracy:\n      enabled: true\n";
  const std::string free8 = "      relax_factor: 1.0e8\n";
  const std::vector<Setting> settings = {
      {"deg off", deg_off, false},
      {"deg on", deg_on, false},
      {"deg on, x free", deg_on + free8, false},
      {"deg on, x removed", deg_on, true},
  };
  // The keyframe-0 priors tightened to what the static initialisation measured
  // (section 5): roll / pitch 0.02 deg (0.35 mrad, init tilt sigma 0.42 mrad), acc bias
  // 0.004 m/s^2 (q_a / sqrt(T_init)), velocity 0.01 m/s (static window).
  const std::string tilt = "    backend:\n      prior_rot_sigma_deg: 0.02\n";
  const std::string tight =
      "    backend:\n      prior_rot_sigma_deg: 0.02\n      prior_bias_acc_sigma_mps2: 0.004\n"
      "      prior_vel_sigma_mps: 0.01\n";
  const std::string frozen_bias =
      "    backend:\n      prior_bias_acc_sigma_mps2: 1.0e-4\n"
      "    imu:\n      acc_random_walk: 1.0e-7\n";
  const auto relin = [](const char* v) {
    return std::string("    backend:\n      relinearize_threshold: ") + v + "\n";
  };
  std::string bias_rows;
  for (const bool straight : {false, true}) {
    const BoxWorldParams p = imu_corridor_params(straight);
    const std::string scene = straight ? "straight" : "circle";
    const fs::path dir = rep.dir() / scene;
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    rep.line("");
    rep.line("## Corridor, " + scene + " (x walls out of range, LiDAR + IMU only)");
    rep.line("");
    rep.line(
        "| case | along-x RMS (m) | end x err (m) | end / sigma_DR | lateral-y RMS (m) | "
        "keyframes | match failures | matcher x corr RMS (m) | median H info x (1/m^2) | "
        "LiDAR factor x: NEES (exp. 1) | err RMS (m) | median sigma (m) | LiDAR factor NEES "
        "6-DoF (exp. 6) | median tilt of weakest H dir from x (rad) | last KF x err (sigma) "
        "(m) | last KF v_x err (m/s) | last KF ba_b x / y (m/s^2) |");
    rep.line(
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
        "--- | --- | --- |");
    int idx = 0;
    for (const Setting& s : settings) {
      std::vector<ImuStudyCase> cases = {
          {"baseline", s.yaml, 0, false, true},
          {"no backend feedback", s.yaml, 0, true, false},
          {"bias frozen", s.yaml + frozen_bias, 0, false, true},
          {"relin 0.01", s.yaml + relin("0.01"), 0, false, false},
          {"relin 1.0", s.yaml + relin("1.0"), 0, false, false},
          {"KF every 20 scans", s.yaml, 20, false, false},
          {"KF every 5 scans", s.yaml, 5, false, false},
          {"oracle measurement", s.yaml, 0, false, false, true},
          {"tilt prior 0.02 deg", s.yaml + tilt, 0, false, false},
          {"tight kf0 priors", s.yaml + tight, 0, false, true},
      };
      for (ImuStudyCase& c : cases) {
        c.name = s.name + ", " + c.name;
        c.x_removed = s.x_removed;
        std::string* rows = (c.bias_trajectory && !straight) ? &bias_rows : nullptr;
        rep.line(imu_study_run(p, dir / "data", dir / ("c" + std::to_string(idx++)), c, rows));
      }
    }
    // Section 6: spurious x information of the plane fits (collinearity gate, flag threshold).
    const std::vector<Setting> spurious = {
        {"deg off, collinearity 0.1",
         deg_off + "    scan_match:\n      min_mid_eig_ratio: 0.1\n",
         false},
        {"deg on, x free, thr 1e4", deg_on + free8 + "      eig_threshold: 1.0e4\n", false},
    };
    for (const Setting& s : spurious) {
      for (ImuStudyCase c : {ImuStudyCase{"baseline", s.yaml, 0, false, false},
                             ImuStudyCase{"no backend feedback", s.yaml, 0, true, false}}) {
        c.name = s.name + ", " + c.name;
        rep.line(imu_study_run(p, dir / "data", dir / ("c" + std::to_string(idx++)), c, nullptr));
      }
    }
  }
  rep.line("");
  rep.line("## Backend keyframe states, circle (bias in b and rotated to w)");
  rep.line("");
  rep.line(
      "| case | t (s) | x err (m) | v_x err (m/s) | ba_b x / y / z (m/s^2) | ba_w x / y "
      "(m/s^2) | tilt err about w x / y (mrad) |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- |");
  std::size_t start = 0;
  while (start < bias_rows.size()) {
    const std::size_t end = bias_rows.find('\n', start);
    rep.line(bias_rows.substr(start, end - start));
    start = end + 1;
  }
}

}  // namespace
}  // namespace mss::pipeline
