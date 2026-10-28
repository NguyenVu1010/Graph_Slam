// Component tests of the pipeline estimator on the synthetic box world
// (tests/component/box_world.hpp, docs/derivations/pipeline.md section 7): trajectory
// accuracy, allocation-free non-keyframe scans and the robustness cases of the plan's
// Review Focus list.
#include <algorithm>
#include <array>
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
#include <map>
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
#include "pipeline_test_support.hpp"

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline {
namespace {

using namespace ct;  // NOLINT(google-build-using-namespace): shared test helpers

// Seeds of PrealignStaticFixesAnchorFrameAlongAxis.
constexpr std::array<std::uint32_t, 4> kPrealignSeeds = {42U, 46U, 48U, 50U};

// GNSS must help where the LiDAR drifts (owner decision on review I3 of PR 4b): a 120 m
// corridor with the end walls out of range, 60 s, GNSS 0.5 m at 5 Hz, no wheel, degeneracy on
// with the absolute detector and an "unobserved" variance of 1000 m^2 along the corridor axis
// (kCorridorRelaxYaml). Why 1000 m^2: the LiDAR factor must not resist an along-axis GNSS
// correction as large as the along-axis drift over the run, which reaches tens of metres in this
// corridor (LiDAR-only ATE 8 to 67 m over seeds 42..44), so its std must be of the order of the
// 30 m drive or more: 1000 m^2 is a 32 m std per keyframe step, far above the 0.5 m GNSS sigma.
// It is not taken from the sweep, which is chaotic (degeneracy.md section 7: worst fused ATE over
// three seeds 1.70 m at 1000 m^2, 2.94 m at 100, 3.38 m at 1e4, 23.9 m at 1). The response is
// chaotic in the seed, so the test asserts robust properties over seeds 42, 43, 44: on every
// seed no chi-square lock-out (> 200 GNSS factors) and fused ATE <= 0.5 x LiDAR-only ATE, and
// the median fused ATE <= 2.18 m (owner decision). Open item (gnss_fusion_study.md sections 12
// and 13): the fused ATE is not consistently better than raw GNSS alone (about 0.87 m): 0.51 /
// 0.59 / 1.70 m over seeds 42..44 (0.6x to 2x), and a median of 1.99 m over seeds 42..56.
TEST(PipelineComponent, CorridorDriftCorrectedByGnss) {
  std::vector<double> fused_m;
  for (const std::uint32_t seed : {42U, 43U, 44U}) {
    BoxWorldParams p = corridor_params();
    p.seed = seed;
    const fs::path dir = test_dir() / ("seed" + std::to_string(seed));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const CorridorRun lidar_only = run_corridor(p, dir / "data", dir / "lidar_only", false);
    const CorridorRun with_gnss = run_corridor(p, dir / "data", dir / "gnss", true);
    EXPECT_GE(lidar_only.ate_m, 0.3) << "seed " << seed;
    if (kWithGtsam) {  // the stub backend adds no factor: nothing to correct with
      EXPECT_LE(with_gnss.ate_m, 0.5 * lidar_only.ate_m) << "seed " << seed;
      // About 300 fixes after the first estimate; a chi-square lock-out shows up here.
      EXPECT_GT(with_gnss.stats.n_gnss_factors, 200U) << "seed " << seed;
    }
    fused_m.push_back(with_gnss.ate_m);
  }
  if (kWithGtsam) {
    std::sort(fused_m.begin(), fused_m.end());
    constexpr double kMedianFusedAteBoundM = 2.18;  // m, owner decision (b) on PR 4b review I3
    std::printf("[ corridor ] fused ATE median %.3f m over seeds 42..44\n", fused_m[1]);
    EXPECT_LE(fused_m[1], kMedianFusedAteBoundM);
  }
}

// Degeneracy with its shipped settings (absolute detector, unobserved_var_m2 0.001 m^2,
// PR 4b). With the shipped degeneracy.enabled false this geometry gives a singular Hessian at
// every scan (no keyframe, nothing to fuse), so the run turns degeneracy on.
const char* const kCorridorDefaultDegeneracyYaml = "    degeneracy:\n      enabled: true\n";

// Review I-1: the corridor with the shipped degeneracy covariance. The LiDAR factor claims
// centimetres (0.001 m^2) along the unobservable axis, so GNSS barely corrects it, but
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

// Records the GNSS frame variable E after the first optimisation that has it, the fixes of
// the GNSS factors added before that optimisation (the first batch) and the keyframe time of
// every GNSS factor.
class FrameObserver : public detail::CoreObserver {
 public:
  void on_frame_estimate(Timestamp /*t*/, const common::Pose& T_e_w) override {
    if (!have_e_) {
      E_first_ = T_e_w;
      have_e_ = true;
    }
  }
  void on_gnss_factor(Timestamp t_k,
                      const common::GnssMeasurement& m,
                      const common::Pose& /*T_e_w*/) override {
    t_k_.push_back(t_k);
    if (!have_e_) {
      first_batch_t_.push_back(m.t);
    }
  }
  bool have_e_ = false;
  common::Pose E_first_;
  std::vector<Timestamp> t_k_;
  std::vector<Timestamp> first_batch_t_;
};

// Along-axis error (in w, rotated with the true T_e_w) of every dataset fix, by time (ns).
std::map<std::int64_t, double> fix_errors_x(const BoxWorldParams& p, const fs::path& data) {
  std::map<std::int64_t, double> out;
  auto opened = io::DatasetReader::open(data, 65536);
  EXPECT_TRUE(opened.is_ok()) << opened.reason();
  if (!opened.is_ok()) {
    return out;
  }
  io::DatasetReader reader = std::move(opened).value();
  ScanFrame scan(65536);
  io::Record rec;
  const common::Pose T_true = test::box_T_e_w(p);
  for (;;) {
    const auto more = reader.next(rec, scan);
    if (!more.is_ok() || !more.value()) {
      break;
    }
    if (rec.kind == io::RecordKind::kGnss) {
      const Vec3 r_e = rec.gnss.p_e_g - T_true * (test::box_truth(p, rec.gnss.t).T_w_b * p.p_b_g);
      out[rec.gnss.t.ns] = (T_true.R.transpose() * r_e).x();
    }
  }
  return out;
}

// ADR-022 known answer. In the corridor (LiDAR blind along the axis, no wheel) the along-axis
// translation of E has no absolute anchor but keyframe 0; on main its value right after the
// alignment comes from the aligner, fitted to dead-reckoned positions (0.1 to 1 m off here).
// With gnss.attach_prealign_fixes the fixes received before the alignment are factors from
// the first optimisation with E on: the static ones on keyframe 0 (its pose held by its prior),
// the others on the first keyframes, which the IMU places from rest to centimetres. So E's
// along-axis offset from the truth after that optimisation (x of T_true^-1 E, in w) is the
// mean along-axis error (from the dataset truth) of the fixes of that first batch. The
// difference is not a sampling error of that mean: it is the mean along-axis error of the
// batch's keyframes (IMU from rest, keyframe 0's prior) plus the pull of E's loose prior. The
// tolerance 2 sigma / sqrt(n) (about 0.13 m) is only a conservative scale for that.
// Measured |E - mean| 0.081 / 0.001 / 0.005 / 0.030 m on seeds 42 / 46 / 48 / 50.
// Checks: (1) that bound; (2) negative control: off, E is farther from that mean than on (off,
// E - truth is +0.49 / +0.97 / +0.12 / +0.84 m); (3) the brief's static-only form: within
// 2 sigma / sqrt(n_static) (about 0.30 m) of the mean error of the static-window fixes
// (measured 1.0 to 1.3 sigma / sqrt(n_static)); (4) every static-window fix is a factor on
// keyframe 0 (Stats::n_gnss_prealign_static) unless the chi-square gate rejected it at release
// (2 of 11 on seed 46).
TEST(PipelineComponent, PrealignFixesAnchorFrameAlongAxis) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "the stub backend has no frame variable";
  }
  for (const std::uint32_t seed : kPrealignSeeds) {
    BoxWorldParams p = corridor_params();
    p.seed = seed;
    const fs::path dir = test_dir() / ("seed" + std::to_string(seed));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const std::map<std::int64_t, double> err_x = fix_errors_x(p, dir / "data");
    const common::Pose T_true = test::box_T_e_w(p);
    std::array<double, 2> dx = {0.0, 0.0};
    double batch_mean_x = 0.0;
    std::size_t batch_n = 0;
    Timestamp t_kf0{};
    std::array<std::uint64_t, 2> attached = {0, 0};
    std::array<std::size_t, 2> at_kf0 = {0, 0};
    std::uint64_t static_attached = 0;
    std::uint64_t rejected = 0;
    for (int on = 0; on < 2; ++on) {
      const Config cfg =
          box_config(p,
                     true,
                     false,
                     dir / (on != 0 ? "on" : "off"),
                     std::string("    preprocess:\n      max_range_m: 10.0\n") + kCorridorRelaxYaml,
                     on != 0 ? "      attach_prealign_fixes: true\n" : "");
      FrameObserver obs;
      detail::CoreTestHooks hooks;
      hooks.observer = &obs;
      ReplayResult rr;
      {
        const HooksScope scope(&hooks);
        rr = replay(dir / "data", cfg);
      }
      ASSERT_FALSE(rr.estimates.empty());
      ASSERT_TRUE(obs.have_e_);
      t_kf0 = rr.estimates.front().state.t;  // keyframe 0
      dx[static_cast<std::size_t>(on)] = (T_true.inverse() * obs.E_first_).p.x();
      attached[static_cast<std::size_t>(on)] = rr.stats.n_gnss_prealign_attached;
      at_kf0[static_cast<std::size_t>(on)] =
          static_cast<std::size_t>(std::count(obs.t_k_.begin(), obs.t_k_.end(), t_kf0));
      if (on != 0) {
        double sum = 0.0;
        for (const Timestamp t : obs.first_batch_t_) {
          sum += err_x.at(t.ns);
        }
        batch_n = obs.first_batch_t_.size();
        static_attached = rr.stats.n_gnss_prealign_static;
        rejected = rr.stats.n_gnss_prealign_rejected;
        batch_mean_x = batch_n > 0 ? sum / static_cast<double>(batch_n) : 0.0;
      }
    }
    // Fixes of the static initialisation window [t_kf0 - imu.init_window_s, t_kf0].
    std::size_t n_static = 0;
    double static_sum_x = 0.0;
    for (const auto& [t_ns, e] : err_x) {
      if (t_ns >= t_kf0.ns - test::box_ns(2.0) && t_ns <= t_kf0.ns) {
        ++n_static;
        static_sum_x += e;
      }
    }
    ASSERT_GT(n_static, 0U);
    const double static_mean_x = static_sum_x / static_cast<double>(n_static);
    const double sigma_static = p.gnss_sigma_m / std::sqrt(static_cast<double>(n_static));
    const double sigma_mean = p.gnss_sigma_m / std::sqrt(static_cast<double>(batch_n));
    std::printf(
        "[ prealign ] seed %u: first batch %zu fixes, mean along-x error %+.3f m (sigma %.3f); "
        "E - truth along x after alignment: off %+.3f m, on %+.3f m; on: %llu attached, %zu on "
        "keyframe 0 (%zu static, mean along-x error %+.3f m, sigma %.3f)\n",
        seed,
        batch_n,
        batch_mean_x,
        sigma_mean,
        dx[0],
        dx[1],
        static_cast<unsigned long long>(attached[1]),
        at_kf0[1],
        n_static,
        static_mean_x,
        sigma_static);
    EXPECT_EQ(attached[0], 0U);
    EXPECT_EQ(at_kf0[0], 0U);
    // The static-window fixes, plus the still-static ones after keyframe 0 nearer to it.
    EXPECT_GE(at_kf0[1], n_static);
    EXPECT_GT(batch_n, n_static);
    EXPECT_LE(static_attached, static_cast<std::uint64_t>(n_static));
    EXPECT_GE(static_attached + rejected, static_cast<std::uint64_t>(n_static));
    EXPECT_LT(std::abs(dx[1] - batch_mean_x), 2.0 * sigma_mean) << "seed " << seed;
    EXPECT_LT(std::abs(dx[1] - batch_mean_x), std::abs(dx[0] - batch_mean_x)) << "seed " << seed;
    EXPECT_LT(std::abs(dx[1] - static_mean_x), 2.0 * sigma_static) << "seed " << seed;
  }
}

// Records the aligner T_e_w given with the first GNSS factor (the centre of E's prior) and the
// first alignment's predicted yaw sigma.
class AlignObserver : public detail::CoreObserver {
 public:
  void on_alignment(Timestamp /*t*/,
                    const common::Pose& /*T_e_w*/,
                    double yaw_sigma_rad,
                    const gnss::GnssTrackYawAligner& track) override {
    if (n_alignments_++ == 0) {
      sigma_rad_ = yaw_sigma_rad;
      no_wheel_segments_ = track.n_no_wheel_segments();
      skipped_turns_ = track.n_skipped_turns();
    }
  }
  void on_gnss_factor(Timestamp /*t_k*/,
                      const common::GnssMeasurement& /*m*/,
                      const common::Pose& T_e_w) override {
    if (!have_first_) {
      first_ = T_e_w;
      have_first_ = true;
    }
  }
  int n_alignments_ = 0;
  int no_wheel_segments_ = -1;
  int skipped_turns_ = -1;
  double sigma_rad_ = std::numeric_limits<double>::quiet_NaN();
  bool have_first_ = false;
  common::Pose first_;
};

// ADR-023 (gnss.align_yaw_from_track): on corridor seeds 51 and 52 GnssAligner gives E's prior a
// yaw 179 / 132 deg off at the first GNSS factor (gnss_fusion_study.md section 13: the front
// end's along-axis velocity changes sign before the alignment). With the flag the yaw is taken
// from the GNSS track against the heading: within 10 deg (brief), and within 3 times the
// predicted sigma at the first alignment. Negative control: off, both are above 90 deg (the
// flag-off path is the unchanged previous code).
TEST(PipelineComponent, TrackYawAlignsCorridorFlipSeeds) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "the front end follows the backend; measured with GTSAM";
  }
  constexpr double kTargetDeg = 10.0;  // deg, brief of design 2
  for (const std::uint32_t seed : {51U, 52U}) {
    BoxWorldParams p = corridor_params();
    p.seed = seed;
    const fs::path dir = test_dir() / ("seed" + std::to_string(seed));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const common::Pose T_true = test::box_T_e_w(p);
    for (const bool on : {false, true}) {
      const Config cfg =
          box_config(p,
                     true,
                     false,
                     dir / (on ? "on" : "off"),
                     std::string("    preprocess:\n      max_range_m: 10.0\n") + kCorridorRelaxYaml,
                     on ? "      align_yaw_from_track: true\n" : "");
      AlignObserver obs;
      detail::CoreTestHooks hooks;
      hooks.observer = &obs;
      {
        const HooksScope scope(&hooks);
        (void)replay(dir / "data", cfg);
      }
      ASSERT_TRUE(obs.have_first_) << "seed " << seed << " on " << on;
      const double err_deg = std::abs(wrap_deg(yaw_of(T_true.R.transpose() * obs.first_.R)));
      std::printf(
          "[ track yaw ] seed %u on %d: aligner yaw error at the first factor %.2f deg, "
          "predicted sigma %.2f deg\n",
          seed,
          on ? 1 : 0,
          err_deg,
          obs.sigma_rad_ * kRadToDeg);
      if (on) {
        EXPECT_LT(err_deg, kTargetDeg) << "seed " << seed;
        EXPECT_LT(err_deg, 3.0 * obs.sigma_rad_ * kRadToDeg) << "seed " << seed;
      } else {
        EXPECT_GT(err_deg, 90.0) << "seed " << seed;
        EXPECT_TRUE(std::isnan(obs.sigma_rad_));
      }
    }
  }
}

// ADR-023 addendum: the direction of travel comes from the wheel speed when wheel odometry is
// on. Box world (2 m circle, wheel samples in the dataset): with wheel_odom.enabled every
// segment up to the first alignment has a wheel sign; without it every one used the
// forward-driving fallback. The circle (29 deg per chunk) passes the 45 deg turn gate.
TEST(PipelineComponent, TrackYawUsesWheelSignWhenAvailable) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "the front end follows the backend; measured with GTSAM";
  }
  BoxWorldParams p;
  p.seed = 42;
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const common::Pose T_true = test::box_T_e_w(p);
  for (const bool wheel : {false, true}) {
    const Config cfg = box_config(p,
                                  true,
                                  wheel,
                                  dir / (wheel ? "wheel" : "no_wheel"),
                                  "",
                                  "      align_yaw_from_track: true\n");
    AlignObserver obs;
    detail::CoreTestHooks hooks;
    hooks.observer = &obs;
    {
      const HooksScope scope(&hooks);
      (void)replay(dir / "data", cfg);
    }
    ASSERT_GE(obs.n_alignments_, 1) << "wheel " << wheel;
    ASSERT_TRUE(obs.have_first_);
    const double err_deg = std::abs(wrap_deg(yaw_of(T_true.R.transpose() * obs.first_.R)));
    std::printf(
        "[ track yaw ] box world wheel %d: yaw error %.2f deg, sigma %.2f deg, "
        "no-wheel segments %d, turn-gated %d\n",
        wheel ? 1 : 0,
        err_deg,
        obs.sigma_rad_ * kRadToDeg,
        obs.no_wheel_segments_,
        obs.skipped_turns_);
    EXPECT_LT(err_deg, 3.0 * obs.sigma_rad_ * kRadToDeg);
    EXPECT_EQ(obs.skipped_turns_, 0);
    if (wheel) {
      EXPECT_EQ(obs.no_wheel_segments_, 0);
    } else {
      EXPECT_GT(obs.no_wheel_segments_, 0);
    }
  }
}

TEST(PipelineComponent, BoxWorldAteWithinTolerance) {
  run_box_world(false);
}

TEST(PipelineComponent, BoxWorldWithGnssAndWheelWithinTolerance) {
  run_box_world(true);
}

// Corridor replay (PR 4b, docs/derivations/scan_match.md section 6): the box world with the
// x walls moved beyond preprocess.max_range_m, so walls y = +-5 m, floor and ceiling remain
// and the world x axis is unobserved by the LiDAR. Neighbourhoods across the wall-floor and
// wall-ceiling edges leak about 1.15 % of the translational information onto the axis
// (scan_match.md section 3); the absolute detector (eig_threshold 100) still flags the axis
// in every keyframe. With the axis flagged the scan-match covariance along x is "unobserved", so
// the wheel odometry and IMU factors decide x. Errors are RMS over all estimates, no alignment.
struct AxisErrors {
  double along_m = 0.0;    // RMS of the x error
  double lateral_m = 0.0;  // RMS of the y error
  std::size_t n = 0;
  std::uint64_t match_failures = 0;
  std::uint64_t keyframes = 0;
  std::uint64_t degenerate_keyframes = 0;
};

AxisErrors corridor_errors(const std::string& extra_overlay, const std::string& tag) {
  BoxWorldParams p;
  p.box_half_m.x() = 1000.0;  // m, beyond preprocess.max_range_m (100 m): x walls never seen
  const fs::path dir = test_dir() / tag;
  fs::create_directories(dir);
  {
    std::ofstream f(dir / "overlay.yaml");
    f << test::box_overlay_yaml(p, false) << extra_overlay;
  }
  const auto cfg = load_config(kDefault, dir / "overlay.yaml");
  EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
  EXPECT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const ReplayResult r = replay(dir / "data", cfg.value());
  AxisErrors e;
  double sx = 0.0;
  double sy = 0.0;
  for (const Estimate& x : r.estimates) {
    const Vec3 d = x.state.T_w_b.p - test::box_truth(p, x.state.t).T_w_b.p;
    sx += d.x() * d.x();
    sy += d.y() * d.y();
  }
  e.n = r.estimates.size();
  e.along_m = e.n == 0 ? 1e9 : std::sqrt(sx / static_cast<double>(e.n));
  e.lateral_m = e.n == 0 ? 1e9 : std::sqrt(sy / static_cast<double>(e.n));
  e.match_failures = r.stats.n_match_failures;
  e.keyframes = r.stats.n_keyframes;
  e.degenerate_keyframes = r.stats.n_degenerate_keyframes;
  std::printf(
      "[ corridor ] %s: along-axis RMS %.4f m, lateral RMS %.4f m, %zu estimates, "
      "%llu keyframes, %llu match failures\n",
      tag.c_str(),
      e.along_m,
      e.lateral_m,
      e.n,
      static_cast<unsigned long long>(e.keyframes),
      static_cast<unsigned long long>(e.match_failures));
  return e;
}

TEST(PipelineComponent, CorridorFollowsWheelAlongAxis) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend (wheel factor in the graph)";
  }
  const std::string wheel = "    wheel_odom:\n      enabled: true\n";
  const std::string deg_abs = "    degeneracy:\n      enabled: true\n";
  const AxisErrors on = corridor_errors(wheel + deg_abs, "absolute_wheel");
  const AxisErrors off =
      corridor_errors(wheel + "    degeneracy:\n      enabled: false\n", "deg_off_wheel");
  EXPECT_GT(on.keyframes, 5U);
  EXPECT_EQ(on.degenerate_keyframes, on.keyframes - 1);  // every LiDAR factor (not keyframe 0)
  EXPECT_EQ(on.match_failures, 0U);
  // Measured on 3f90e34: absolute 3.8 mm along / 0.8 mm lateral; off (H nearly singular
  // along x, the Gauss-Newton step wanders) 0.30 m / 3.3 mm.
  EXPECT_LT(on.along_m, 0.03);
  EXPECT_LT(on.lateral_m, 0.01);
  EXPECT_LT(on.along_m, off.along_m);
}

// Corridor without wheel odometry, absolute detector (it flags the axis, see above) and the
// config/default.yaml unobserved variances: only the IMU observes the axis. Measured 0.021 m
// along-axis RMS on 3f90e34 with the ADR-020 gravity-direction factor and the ADR-021
// decoupled relative factor (0.160 m on 94b9f16 without them); degeneracy.md section 7:
// 0.073 m at 1e-4, 0.14 m at 0.01, 2.2 m at 0.1, 0.82 m at 1.0, 6.1 m with degeneracy off.
// Deterministic (seed 42, deterministic: true); measured with the WSL GTSAM in ~/opt.
TEST(PipelineComponent, CorridorImuOnlyAlongAxisBounded) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend";
  }
  const AxisErrors e =
      corridor_errors("    degeneracy:\n      enabled: true\n", "imu_only_absolute");
  EXPECT_GT(e.keyframes, 5U);
  EXPECT_EQ(e.match_failures, 0U);
  EXPECT_LT(e.along_m, 0.1);
  EXPECT_LT(e.lateral_m, 0.01);
}

// Degeneracy study behind docs/derivations/degeneracy.md section 7 (tables). Runs only with
// MSS_DEGENERACY_STUDY=1 and GTSAM (about 2 min); prints one [study] line per setting and
// scene. Scenes: corridor (x walls out of range) with wheel odometry, corridor IMU only, box
// LiDAR only, box with GNSS and wheel, and the box with a 3x footprint (48 x 30 m) in both
// modes. Not a pass/fail check.
TEST(DegeneracyStudy, Tables) {
  const char* flag = std::getenv("MSS_DEGENERACY_STUDY");
  if (flag == nullptr || std::string(flag) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_DEGENERACY_STUDY=1 (GTSAM build)";
  }
  const auto vars = [](const char* m2, const char* rad2) {
    return std::string("    scan_match:\n      unobserved_var_m2: ") + m2 +
           "\n      unobserved_var_rad2: " + rad2 + "\n";
  };
  std::vector<std::pair<std::string, std::string>> settings = {
      {"off", "    degeneracy:\n      enabled: false\n"},
      {"absolute 100", "    degeneracy:\n      enabled: true\n"},
  };
  for (const char* v : {"1.0e-4", "0.01", "0.1", "1.0"}) {
    const std::string rad2 = std::to_string(std::stod(v) / 10.0);
    settings.emplace_back(std::string("absolute 100 var ") + v,
                          "    degeneracy:\n      enabled: true\n" + vars(v, rad2.c_str()));
  }
  // MSS_DEGENERACY_STUDY_FILTER: run only the settings whose name contains this text.
  const char* filter = std::getenv("MSS_DEGENERACY_STUDY_FILTER");

  const char* scene_names[] = {"corridor+wheel",
                               "corridor imu-only",
                               "box lidar-only",
                               "box gnss+wheel",
                               "box3x lidar-only",
                               "box3x gnss+wheel"};
  for (std::size_t s = 0; s < settings.size(); ++s) {
    if (filter != nullptr && settings[s].first.find(filter) == std::string::npos) {
      continue;
    }
    for (int scene = 0; scene < 6; ++scene) {
      BoxWorldParams p;
      if (scene < 2) {
        p.box_half_m.x() = 1000.0;  // m, beyond preprocess.max_range_m
      }
      if (scene >= 4) {  // 48 x 30 m footprint, same height: ranges 3x larger
        p.box_half_m.x() *= 3.0;
        p.box_half_m.y() *= 3.0;
      }
      std::string overlay = test::box_overlay_yaml(p, scene == 3 || scene == 5);
      if (scene == 0) {
        overlay += "    wheel_odom:\n      enabled: true\n";
      }
      overlay += settings[s].second;
      const fs::path dir = test_dir() / ("s" + std::to_string(s) + "_" + std::to_string(scene));
      fs::create_directories(dir);
      {
        std::ofstream f(dir / "overlay.yaml");
        f << overlay;
      }
      const auto cfg = load_config(kDefault, dir / "overlay.yaml");
      ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
      ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
      const ReplayResult r = replay(dir / "data", cfg.value());
      double sx = 0.0;
      double sy = 0.0;
      double s3 = 0.0;
      for (const Estimate& x : r.estimates) {
        const Vec3 d = x.state.T_w_b.p - test::box_truth(p, x.state.t).T_w_b.p;
        sx += d.x() * d.x();
        sy += d.y() * d.y();
        s3 += d.squaredNorm();
      }
      const double n = std::max(1.0, static_cast<double>(r.estimates.size()));
      std::printf(
          "[study] %-32s %-18s ATE %.4f m, along-x %.4f m, lateral-y %.4f m, "
          "%llu keyframes (%llu degenerate, %llu rotation), %llu match failures\n",
          settings[s].first.c_str(),
          scene_names[scene],
          std::sqrt(s3 / n),
          std::sqrt(sx / n),
          std::sqrt(sy / n),
          static_cast<unsigned long long>(r.stats.n_keyframes),
          static_cast<unsigned long long>(r.stats.n_degenerate_keyframes),
          static_cast<unsigned long long>(r.stats.n_degenerate_rot_keyframes),
          static_cast<unsigned long long>(r.stats.n_match_failures));
    }
  }
}

// Unobserved-variance sweep of the #19 GNSS corridor scenario (CorridorDriftCorrectedByGnss,
// kCorridorRelaxYaml), degeneracy.md section 7. Runs only with MSS_DEGENERACY_STUDY=1 and
// GTSAM; prints one [gnss-study] line per seed and variance. Not a pass/fail check.
TEST(DegeneracyStudy, GnssCorridorVariance) {
  const char* flag = std::getenv("MSS_DEGENERACY_STUDY");
  if (flag == nullptr || std::string(flag) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_DEGENERACY_STUDY=1 (GTSAM build)";
  }
  for (const std::uint32_t seed : {42U, 43U, 44U}) {
    BoxWorldParams p = corridor_params();
    p.seed = seed;
    const fs::path dir = test_dir() / ("seed" + std::to_string(seed));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    for (const char* v : {"1.0", "10.0", "100.0", "1000.0", "1.0e4"}) {
      const std::string rad2 = std::to_string(std::stod(v) / 100.0);
      const std::string yaml = std::string("    degeneracy:\n      enabled: true\n") +
                               "    scan_match:\n      unobserved_var_m2: " + v +
                               "\n      unobserved_var_rad2: " + rad2 + "\n";
      const CorridorRun lidar =
          run_corridor(p, dir / "data", dir / (std::string("lidar_") + v), false, "", yaml);
      const CorridorRun fused =
          run_corridor(p, dir / "data", dir / (std::string("gnss_") + v), true, "", yaml);
      std::printf(
          "[gnss-study] seed %u var %s m^2: LiDAR-only ATE %.3f m (%llu kf), fused ATE "
          "%.3f m (%llu kf, %llu gnss factors, %llu realign)\n",
          seed,
          v,
          lidar.ate_m,
          static_cast<unsigned long long>(lidar.stats.n_keyframes),
          fused.ate_m,
          static_cast<unsigned long long>(fused.stats.n_keyframes),
          static_cast<unsigned long long>(fused.stats.n_gnss_factors),
          static_cast<unsigned long long>(fused.stats.n_gnss_realign));
    }
  }
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
  // `prealign_max_fixes` > 0: also gnss.attach_prealign_fixes with that buffer bound (ADR-022).
  explicit StaticCore(bool aux = false, int prealign_max_fixes = 0)
      : aux_(aux),
        cfg_(make_cfg(aux, prealign_max_fixes)),
        core_(std::make_unique<detail::EstimatorCore>(cfg_)) {
    scan_.reserve(static_cast<std::size_t>(p_.rings * p_.columns));
    test::box_scan(p_, Timestamp{p_.t0_ns}, scan_);  // static: identical at every time
    empty_.reserve(1);
  }

  static Config make_cfg(bool aux = false, int prealign_max_fixes = 0) {
    const auto loaded = load_config(kDefault);
    EXPECT_TRUE(loaded.is_ok()) << loaded.reason();
    Config c = loaded.value();
    c.frames.T_b_l = test::box_default_T_b_l();
    c.frames.p_b_g = BoxWorldParams{}.p_b_g;
    c.gnss.enabled = aux;
    c.wheel_odom.enabled = aux;
    if (prealign_max_fixes > 0) {
      c.gnss.attach_prealign_fixes = true;
      c.gnss.prealign_max_fixes = prealign_max_fixes;
    }
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

// ADR-022: with gnss.attach_prealign_fixes the static body never aligns (no spread), so every
// fix goes to the pre-alignment buffer, which fills (4 fixes) and then drops and counts; the
// non-keyframe scans still do not allocate.
// Re-review N1: one NaN or Inf wheel speed must not poison the ADR-023 wheel odometer (it used
// to stay NaN for the rest of the run, silently switching the track-yaw aligner to its
// forward-driving fallback). Non-finite samples are skipped and counted; the odometer integrates
// the finite ones (trapezoid over the gap).
TEST(PipelineComponent, WheelOdometerSkipsNonFiniteSpeed) {
  detail::EstimatorCore core(StaticCore::make_cfg(true));
  for (int k = 0; k <= 50; ++k) {  // 1 s at 50 Hz, 1 m/s forward
    common::WheelSample w;
    w.t = Timestamp{1'000'000'000 + k * 20'000'000LL};
    w.v_mps = 1.0;
    if (k == 10) {
      w.v_mps = std::numeric_limits<double>::quiet_NaN();
    } else if (k == 30) {
      w.v_mps = std::numeric_limits<double>::infinity();
    }
    core.on_wheel(w);
  }
  EXPECT_EQ(core.counters().n_wheel_nonfinite, 2U);
  EXPECT_TRUE(std::isfinite(core.wheel_odometer_m()));
  EXPECT_NEAR(core.wheel_odometer_m(), 1.0, 1e-9);
}

TEST(PipelineComponent, PrealignBufferBoundedNoAllocation) {
  StaticCore s(true, 4);
  s.scan_at(s.t_scan(0));  // initialisation + keyframe 0
  ASSERT_TRUE(s.core().has_estimate());
  for (int k = 1; k <= 5; ++k) {
    s.scan_at(s.t_scan(k));  // warm-up
  }
  const std::uint64_t dropped = s.core().counters().n_gnss_prealign_dropped;
  AllocCounter::reset();
  {
    const EigenMallocGuard guard;
    for (int k = 6; k < 16; ++k) {
      s.scan_at(s.t_scan(k));
    }
  }
  EXPECT_EQ(AllocCounter::count_this_thread(), 0U);
  // 5 Hz: one second of scans brings 5 more fixes, all dropped (buffer full).
  EXPECT_EQ(s.core().counters().n_gnss_prealign_dropped, dropped + 5U);
  EXPECT_EQ(s.core().counters().n_gnss_prealign_attached, 0U);
  EXPECT_EQ(s.core().counters().n_gnss_factors, 0U);
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

// GNSS study table header and rows.
void gnss_study_header(StudyReport& rep, const std::string& title) {
  rep.line("");
  rep.line("## " + title);
  rep.line("");
  rep.line(
      "| case | GNSS factors | ATE (m) | ATE 4-DoF (m) | rebuilds | LiDAR NEES (exp. 6) | "
      "LiDAR err / sqrt(tr cov_p) (m) | GNSS NEES used / true (exp. 3) | aligner |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- | --- | --- |");
}
void gnss_study_row(StudyReport& rep, const std::string& name, const StudyResult& r) {
  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                "| %s | %llu | %.4f | %.4f | %llu | ",
                name.c_str(),
                static_cast<unsigned long long>(r.stats.n_gnss_factors),
                r.ate_m,
                r.ate4_m,
                static_cast<unsigned long long>(r.stats.n_map_rebuilds));
  rep.line(std::string(buf) + r.factors + " |");
}

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

  gnss_study_header(rep, "Box world (sections 1 to 4, 9), fixed T_e_w unless 'frame variable'");
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
      gnss_study_row(rep, label + " (" + tag + ")", study_run(p, dir / "data", dir, c));
    }
  }

  gnss_study_header(rep, "Degenerate corridor (sections 8, 9)");
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
        gnss_study_row(rep, label, study_run(p, dir / "data", dir, c));
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

// Seeds 42, 43, 44 (IMU noise), shipped configuration. sigma_DR is the dead-reckoning sigma
// per axis after T: sigma_DR = hypot(q_a T^1.5 / sqrt(3), 0.5 q_a / sqrt(T_init) T^2)
// (velocity random walk, static-init tilt; q_a = BoxWorldParams::acc_noise_density):
// 0.70 m at T = 17.9 s. Medians over the three seeds are asserted (the 15-seed study,
// imu_only_observability.md section 10e, has tails no 20 s scene can bound per seed):
// - circle: |end x err| <= 3 sigma_DR (the brief's target; 15-seed median 1.45 m, without the
//   factor 21.7 m); along-x RMS <= 3.0 m, the worst RMS of the 15 seeds (2.84 m, median
//   1.36 m; without the factor median 11.6 m): mid-run excursions of up to 24 sigma_DR(t)
//   remain, with transient tilt errors of 8 to 11 mrad (ADR-020 known gap);
// - straight: |end x err| <= 1.86 m and along-x RMS <= 1.88 m, 2x the measured medians of
//   seeds 42..44 with the shipped factor (0.93 m and 0.94 m; per seed 0.93 / 0.47 / 2.72 m
//   and 0.61 / 1.34 / 0.94 m). Without the factor the same seeds give 6.90 m and 3.52 m, so
//   both halves fail with the factor off. Over 15 seeds the straight medians are 2.72 m
//   = 3.9 sigma_DR (misses the brief's 3 sigma_DR, ADR-020 known gap) and 1.34 m; without
//   the factor 3.56 m and 1.85 m. The circle half is the stronger on / off distinction
//   (without the factor its medians over these seeds are 84.8 m and 34.7 m).
// Lateral error at the LiDAR level (< 1 cm) on every seed.
// The numbers above were measured with relative_factor between (PR 4d). With the default
// decoupled (ADR-021) the same seeds give: circle end 2.30 / 0.78 / 0.08 m (median 0.78 m),
// RMS median 0.74 m; straight end 0.82 / 0.60 / 8.18 m (median 0.82 m), RMS median 1.43 m.
// The limits are unchanged.
TEST(PipelineComponent, CorridorImuOnlyAlongAxisWithinDeadReckoning) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend (IMU factors in the graph)";
  }
  for (const bool straight : {false, true}) {
    std::vector<double> end_err_m;
    std::vector<double> rms_m;
    double sigma_dr_m = 0.0;
    for (const std::uint32_t seed : {42U, 43U, 44U}) {
      BoxWorldParams p = imu_corridor_params(straight);
      p.seed = seed;
      const fs::path dir =
          test_dir() / ((straight ? "straight_" : "circle_") + std::to_string(seed));
      ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
      const AxisRun r = corridor_axis_run(p, dir / "data", dir / "imu_only", false, "");
      sigma_dr_m = dr_sigma_m(p.acc_noise_density, r.init_window_s, r.T_s);
      std::printf("[ corridor axis ] %s seed %u: end / sigma_DR %.2f (sigma_DR %.3f m, T %.1f s)\n",
                  straight ? "straight" : "circle",
                  seed,
                  std::abs(r.end_x_m) / sigma_dr_m,
                  sigma_dr_m,
                  r.T_s);
      EXPECT_GT(r.T_s, 17.0) << seed;
      EXPECT_LT(r.lateral_rms_m, 0.01) << seed;
      EXPECT_GT(r.keyframes, 5U) << seed;
      EXPECT_LT(r.keyframes, 40U) << seed;  // the 1 m rule does not fire on a drift along x
      // At rest: the rest gate passes, the factor is added and kept (check off, shipped).
      EXPECT_EQ(r.gravity_gated, 0U) << seed;
      EXPECT_EQ(r.gravity_removed, 0U) << seed;
      end_err_m.push_back(std::abs(r.end_x_m));
      rms_m.push_back(r.along_rms_m);
    }
    std::sort(end_err_m.begin(), end_err_m.end());
    std::sort(rms_m.begin(), rms_m.end());
    std::printf(
        "[ corridor axis ] %s: median end error %.3f m (3 sigma_DR %.3f m), median RMS %.3f m\n",
        straight ? "straight" : "circle",
        end_err_m[1],
        3.0 * sigma_dr_m,
        rms_m[1]);
    if (straight) {
      EXPECT_LE(end_err_m[1], 1.86);  // m, 2 x 0.93 (seeds 42..44, shipped)
      EXPECT_LE(rms_m[1], 1.88);      // m, 2 x 0.94
    } else {
      EXPECT_LE(end_err_m[1], 3.0 * sigma_dr_m);
      EXPECT_LE(rms_m[1], 3.0);
    }
  }
}

// The same corridor with wheel odometry: the wheel speed observes the axis.
TEST(PipelineComponent, CorridorWithWheelAlongAxisMillimetres) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend (wheel factor in the graph)";
  }
  const BoxWorldParams p = imu_corridor_params(false);
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const AxisRun r = corridor_axis_run(p, dir / "data", dir / "wheel", true, "");
  EXPECT_LE(r.along_rms_m, 0.005);
  EXPECT_LT(r.lateral_rms_m, 0.01);
}

// ---------------------------------------------------------------------------------------
// ADR-020 rest precondition (imu_only_observability.md section 12): the keyframe-0 gravity
// factor reads the init window as "body at rest". A start in motion that the initializer
// still accepts (constant per-axis readings) must not make the estimate much worse: the rest
// gate does not add the factor (turn), or the factor costs little because the initializer's
// R_w_b reads the same mean (straight acceleration). The consistency check is off (shipped).

struct StartRun {
  double ate_m = 0.0;
  std::uint64_t gated = 0;
  std::uint64_t removed = 0;
  std::uint64_t unchecked = 0;
  std::size_t estimates = 0;
  double chi2 = -1.0;  // gravity-factor consistency check (-1: not run)
  bool check_ran = false;
  detail::GravityCheckOutcome outcome = detail::GravityCheckOutcome::kKept;
};

// Records the innovation statistic and outcome of the gravity factor's consistency check.
class GravityCheckObserver final : public detail::CoreObserver {
 public:
  void on_gravity_check(double chi2, detail::GravityCheckOutcome outcome) override {
    chi2_ = chi2;
    outcome_ = outcome;
    ran_ = true;
  }
  [[nodiscard]] double chi2() const { return chi2_; }
  [[nodiscard]] detail::GravityCheckOutcome outcome() const { return outcome_; }
  [[nodiscard]] bool ran() const { return ran_; }

 private:
  double chi2_ = -1.0;
  detail::GravityCheckOutcome outcome_ = detail::GravityCheckOutcome::kKept;
  bool ran_ = false;
};

StartRun start_run(const BoxWorldParams& p,
                   const fs::path& data,
                   const fs::path& dir,
                   const std::string& backend_yaml) {
  GravityCheckObserver obs;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  const HooksScope scope(&hooks);
  const ReplayResult r = replay(data, box_config(p, false, false, dir, backend_yaml));
  StartRun out;
  out.ate_m = trajectory_errors(p, r.estimates).ate_m;
  out.gated = r.stats.n_gravity_factor_gated;
  out.removed = r.stats.n_gravity_factor_removed;
  out.unchecked = r.stats.n_gravity_factor_unchecked;
  out.estimates = r.estimates.size();
  out.chi2 = obs.chi2();
  out.check_ran = obs.ran();
  out.outcome = obs.outcome();
  std::printf(
      "[ moving start ] %s: ATE %.4f m, %zu estimates, gated %llu, removed %llu, unchecked "
      "%llu, check innovation chi2 %.2f\n",
      dir.filename().string().c_str(),
      out.ate_m,
      out.estimates,
      static_cast<unsigned long long>(out.gated),
      static_cast<unsigned long long>(out.removed),
      static_cast<unsigned long long>(out.unchecked),
      out.chi2);
  return out;
}

const char* const kGravityOffYaml = "    backend:\n      static_gravity_factor: false\n";

const char* const kGravityCheckOnYaml =
    "    backend:\n      static_gravity_check_keyframes: 5\n";  // evaluated knob, off shipped

BoxWorldParams turning_start_params() {
  BoxWorldParams p;
  p.static_s = 0.0;
  p.ramp_s = 0.0;
  return p;
}

// Straight run accelerating from t = 0 over `ramp_s` (no still phase).
BoxWorldParams ramp_start_params(double ramp_s) {
  BoxWorldParams p;
  p.static_s = 0.0;
  p.ramp_s = ramp_s;
  p.motion_s = 20.0 - ramp_s;
  p.yaw_rate_radps = 1e-4;  // rad/s: an almost straight run
  p.box_half_m.x() = 30.0;  // m: the run stays inside the box
  return p;
}

// Shipped config. Box circle from t = 0 (no still phase): a 0.25 rad/s turn, constant in b,
// so the initializer accepts the window. The rest gate rejects it (|mean gyro| 0.25 > 0.15
// rad/s): no factor, the same estimate as with the factor off.
TEST(PipelineComponent, GravityFactorTurningStartGated) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend";
  }
  const BoxWorldParams p = turning_start_params();
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const StartRun off = start_run(p, dir / "data", dir / "off", kGravityOffYaml);
  const StartRun on = start_run(p, dir / "data", dir / "on", "");
  ASSERT_GT(off.estimates, 100U);
  EXPECT_EQ(on.gated, 1U);
  EXPECT_LE(on.ate_m, off.ate_m * 1.0001);
}

// Shipped config (consistency check off). Straight runs accelerating from t = 0, the gyro
// reading ~0, so the rest gate passes and the factor is added and kept although the window
// holds the forward acceleration a, read as a tilt a / g (6 s ramp: mean a about 0.06 m/s^2
// in the window; 2 s ramp: 0.25 m/s^2). The cost is small because the initializer's R_w_b
// comes from the same window mean: the run starts with that tilt with or without the factor.
// Measured ATE, factor off -> kept: 6 s 0.1043 -> 0.1028 m (-1.4 %), 2 s 0.5465 -> 0.5584 m
// (+2.2 %). Bound: +5 %.
TEST(PipelineComponent, GravityFactorAcceleratingStartKeptCostsLittle) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend";
  }
  for (const double ramp_s : {6.0, 2.0}) {
    const BoxWorldParams p = ramp_start_params(ramp_s);
    const fs::path dir = test_dir() / ("ramp_" + std::to_string(static_cast<int>(ramp_s)));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const StartRun off = start_run(p, dir / "data", dir / "off", kGravityOffYaml);
    const StartRun on = start_run(p, dir / "data", dir / "on", "");
    ASSERT_GT(off.estimates, 100U);
    EXPECT_EQ(on.gated, 0U) << ramp_s;
    EXPECT_EQ(on.removed, 0U) << ramp_s;
    EXPECT_FALSE(on.check_ran) << ramp_s;
    EXPECT_LE(on.ate_m, off.ate_m * 1.05) << ramp_s;
  }
}

// The consistency check as an evaluated knob (backend.static_gravity_check_keyframes: 5,
// off in the shipped config, ADR-020): it removes the factor at the moving starts above.
// Innovation statistic at the check: turning start with the gyro bound opened 106.7, 6 s ramp
// 38.1, 2 s ramp 233 (removed; ATE 1.0006, 0.1025, 0.5575 m against 1.0017, 0.1043, 0.5465 m
// with the factor off). At rest it also removes the factor where the rest of the graph is
// inconsistent at keyframe 5 (imu_only_observability.md section 10g), so it ships off.
TEST(PipelineComponent, GravityFactorCheckRemovesMovingStarts) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "needs the GTSAM backend";
  }
  {
    const BoxWorldParams p = turning_start_params();
    const fs::path dir = test_dir() / "turn";
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const StartRun off = start_run(p, dir / "data", dir / "off", kGravityOffYaml);
    const StartRun open = start_run(
        p,
        dir / "data",
        dir / "gate_open",
        std::string("    imu:\n      init_max_mean_gyro_radps: 1.0\n") + kGravityCheckOnYaml);
    EXPECT_EQ(open.gated, 0U);
    EXPECT_EQ(open.removed, 1U);
    EXPECT_TRUE(open.check_ran);
    EXPECT_EQ(open.outcome, detail::GravityCheckOutcome::kRemoved);
    EXPECT_LE(open.ate_m, off.ate_m * 1.1);
  }
  for (const double ramp_s : {6.0, 2.0}) {
    const BoxWorldParams p = ramp_start_params(ramp_s);
    const fs::path dir = test_dir() / ("ramp_" + std::to_string(static_cast<int>(ramp_s)));
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    const StartRun off = start_run(p, dir / "data", dir / "off", kGravityOffYaml);
    const StartRun on = start_run(p, dir / "data", dir / "on", kGravityCheckOnYaml);
    EXPECT_EQ(on.gated, 0U) << ramp_s;
    EXPECT_EQ(on.removed, 1U) << ramp_s;
    EXPECT_EQ(on.unchecked, 0U) << ramp_s;
    EXPECT_EQ(on.outcome, detail::GravityCheckOutcome::kRemoved) << ramp_s;
    EXPECT_LE(on.ate_m, off.ate_m * 1.1) << ramp_s;
  }
}

}  // namespace
}  // namespace mss::pipeline
