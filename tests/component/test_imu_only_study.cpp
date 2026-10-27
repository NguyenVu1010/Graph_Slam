// IMU-only observability study (docs/derivations/imu_only_observability.md), skipped unless
// MSS_IMU_STUDY=1 (GTSAM build): the diagnosis tables (sections 1 to 8) and the evaluation of
// the static-window gravity factor (section 9). Results under results/<run-id>/.
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
#include <iterator>
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

#include "box_world.hpp"
#include "estimator_core.hpp"
#include "pipeline_test_support.hpp"
#include "study_seams.hpp"

#ifndef MSS_TEST_WITH_GTSAM
#error "MSS_TEST_WITH_GTSAM must be defined (tests/component/CMakeLists.txt)"
#endif

namespace mss::pipeline {
namespace {

using namespace ct;  // NOLINT(google-build-using-namespace): shared test helpers

// ---------------------------------------------------------------------------------------
// IMU-only observability study (docs/derivations/imu_only_observability.md), skipped unless
// MSS_IMU_STUDY=1 (GTSAM build). Writes results/<run-id>/imu_study.md under the repository
// (run id from MSS_IMU_STUDY_RUN_ID, default "imu-study"). Corridor: the box world with its
// x walls beyond preprocess.max_range_m, LiDAR + IMU only (no wheel, no GNSS), so only the
// IMU carries the world x axis. Uses the test-only CoreTestHooks seam (keyframe period,
// front end without backend feedback, per-scan and per-keyframe observer).

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
    // Tilt part of the rotation error, in w (rad): the LiDAR chain's tilt increment error.
    const Vec3 er_w = xj.T_w_b.R * e6.tail<3>();
    tilt_err2_ += er_w.head<2>().squaredNorm() / 2.0;
    err_x2_ += ex * ex;
    sigma_x_.push_back(std::sqrt(var_x));
  }

  void on_keyframe_optimized(Timestamp t,
                             const common::NavState& x,
                             const common::PoseCovariance& cov) override {
    keyframes_.push_back({t, x, cov});
  }

  void on_gravity_check(double chi2, detail::GravityCheckOutcome outcome) override {
    gravity_chi2_ = chi2;
    gravity_outcome_ = outcome;
    gravity_checked_ = true;
  }
  // The gravity check: "-" (not run), "kept 1.2", "REMOVED 43.0", "unchecked", "failed".
  [[nodiscard]] std::string gravity_check() const {
    if (!gravity_checked_) {
      return "-";
    }
    char buf[48];
    switch (gravity_outcome_) {
      case detail::GravityCheckOutcome::kKept:
        std::snprintf(buf, sizeof(buf), "kept %.1f", gravity_chi2_);
        break;
      case detail::GravityCheckOutcome::kRemoved:
        std::snprintf(buf, sizeof(buf), "REMOVED %.1f", gravity_chi2_);
        break;
      case detail::GravityCheckOutcome::kRemovalFailed:
        std::snprintf(buf, sizeof(buf), "failed %.1f", gravity_chi2_);
        break;
      case detail::GravityCheckOutcome::kUnchecked:
        std::snprintf(buf, sizeof(buf), "unchecked");
        break;
    }
    return buf;
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
  void on_frame_estimate(Timestamp t, const common::Pose& T_e_w) override {
    frames_.emplace_back(t, T_e_w);
  }
  [[nodiscard]] const std::vector<std::pair<Timestamp, common::Pose>>& frames() const {
    return frames_;
  }
  [[nodiscard]] std::size_t n_factors() const { return sigma_x_.size(); }
  // Mean 6-DoF NEES of the LiDAR factors (expected 6).
  [[nodiscard]] double nees6() const {
    return sigma_x_.empty() ? 0.0 : nees6_ / static_cast<double>(sigma_x_.size());
  }
  // RMS per axis of the tilt part of the LiDAR factors' rotation error, rad.
  [[nodiscard]] double tilt_err_rms() const {
    return sigma_x_.empty() ? 0.0 : std::sqrt(tilt_err2_ / static_cast<double>(sigma_x_.size()));
  }

 private:
  double gravity_chi2_ = 0.0;
  detail::GravityCheckOutcome gravity_outcome_ = detail::GravityCheckOutcome::kKept;
  bool gravity_checked_ = false;

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
  std::vector<std::pair<Timestamp, common::Pose>> frames_;
  double tilt_err2_ = 0.0;
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
  // Diagnosis rows run without the fix (static_gravity_factor false, the main behaviour).
  const std::string pin = "    backend:\n      static_gravity_factor: false\n";
  const std::string tilt = pin + "      prior_rot_sigma_deg: 0.02\n";
  const std::string tight = pin +
                            "      prior_rot_sigma_deg: 0.02\n      prior_bias_acc_sigma_mps2: "
                            "0.004\n      prior_vel_sigma_mps: 0.01\n";
  // Section 9: the fix, the static-window specific-force factor on keyframe 0.
  const std::string gravity = "    backend:\n      static_gravity_factor: true\n";
  const std::string frozen_bias = pin +
                                  "      prior_bias_acc_sigma_mps2: 1.0e-4\n"
                                  "    imu:\n      acc_random_walk: 1.0e-7\n";
  const auto relin = [&pin](const char* v) {
    return pin + "      relinearize_threshold: " + v + "\n";
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
          {"baseline", s.yaml + pin, 0, false, true},
          {"no backend feedback", s.yaml + pin, 0, true, false},
          {"bias frozen", s.yaml + frozen_bias, 0, false, true},
          {"relin 0.01", s.yaml + relin("0.01"), 0, false, false},
          {"relin 1.0", s.yaml + relin("1.0"), 0, false, false},
          {"KF every 20 scans", s.yaml + pin, 20, false, false},
          {"KF every 5 scans", s.yaml + pin, 5, false, false},
          {"oracle measurement", s.yaml + pin, 0, false, false, true},
          {"tilt prior 0.02 deg", s.yaml + tilt, 0, false, false},
          {"tight kf0 priors", s.yaml + tight, 0, false, true},
          {"static gravity factor", s.yaml + gravity, 0, false, true},
          {"static gravity factor, KF every 5 scans", s.yaml + gravity, 5, false, false},
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
      for (ImuStudyCase c : {ImuStudyCase{"baseline", s.yaml + pin, 0, false, false},
                             ImuStudyCase{"no backend feedback", s.yaml + pin, 0, true, false}}) {
        c.name = s.name + ", " + c.name;
        rep.line(imu_study_run(p, dir / "data", dir / ("c" + std::to_string(idx++)), c, nullptr));
      }
    }
  }
  // Section 9: a constant accelerometer bias (0.05, 0.03, 0) m/s^2 in b, which the static
  // window cannot tell from tilt: a tight tilt prior pins the wrong tilt, the factor does not.
  {
    BoxWorldParams p = imu_corridor_params(false);
    p.acc_bias_b = Vec3(0.05, 0.03, 0.0);
    const fs::path dir = rep.dir() / "circle_biased";
    ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
    rep.line("");
    rep.line("## Corridor, circle, accelerometer bias (0.05, 0.03, 0) m/s^2 in b");
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
    for (const Setting& s : {settings[1], settings[2]}) {
      for (ImuStudyCase c :
           {ImuStudyCase{"baseline", s.yaml + pin, 0, false, false},
            ImuStudyCase{"tilt prior 0.02 deg", s.yaml + tilt, 0, false, false},
            ImuStudyCase{"tight kf0 priors", s.yaml + tight, 0, false, false},
            ImuStudyCase{"static gravity factor", s.yaml + gravity, 0, false, false}}) {
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

// Section 9 evaluation (MSS_IMU_STUDY=1): the fix and the prior variants over 5 seeds
// (42..46, IMU and GNSS noise) on five scenes; one row per scene and variant with the
// median and the maximum of |metric| over the seeds. Writes imu_fix_eval.md.
TEST(PipelineComponent, ImuOnlyObservabilityFixEvaluation) {
  const char* on = std::getenv("MSS_IMU_STUDY");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_IMU_STUDY=1 (GTSAM build)";
  }
  StudyReport rep("imu_fix_eval.md", "MSS_IMU_STUDY_RUN_ID", "imu-study");
  rep.line("# IMU-only observability: fix evaluation over seeds 42..46");
  rep.line("");
  rep.line("| scene | metric | variant | median (m) | max (m) | per seed (m) |");
  rep.line("| --- | --- | --- | --- | --- | --- |");
  const std::string relax = "    degeneracy:\n      enabled: true\n      relax_factor: 1.0e8\n";
  const std::string pin = "    backend:\n      static_gravity_factor: false\n";
  // Scale 1: hook-scaled noise is relative to the window std of the mean (round-1 rows).
  const std::string fac =
      "    backend:\n      static_gravity_factor: true\n      static_gravity_sigma_scale: 1.0\n";
  const std::string tight =
      "      prior_rot_sigma_deg: 0.02\n      prior_bias_acc_sigma_mps2: 0.004\n"
      "      prior_vel_sigma_mps: 0.01\n";
  const std::vector<std::pair<std::string, std::string>> variants = {
      {"main (no factor)", pin},
      {"static gravity factor", fac},
      {"tight kf0 priors", pin + tight},
      {"factor + tight priors", fac + tight},
      {"factor + velocity prior 0.01 m/s", fac + "      prior_vel_sigma_mps: 0.01\n"},
  };
  struct Scene {
    std::string name;
    std::string metric;
  };
  const std::vector<Scene> scenes = {
      {"20 s circle corridor, LiDAR + IMU, x free", "end x err"},
      {"20 s straight corridor, LiDAR + IMU, x free", "end x err"},
      {"60 s corridor (CorridorDriftCorrectedByGnss), LiDAR + IMU", "ATE"},
      {"60 s corridor (CorridorDriftCorrectedByGnss), + GNSS 0.5 m", "ATE"},
      {"box world, LiDAR + IMU (shipped degeneracy off)", "ATE"},
      {"20 s circle corridor, x free, acc bias (0.05, 0.03, 0) m/s^2", "end x err"},
  };
  for (std::size_t si = 0; si < scenes.size(); ++si) {
    std::vector<std::vector<double>> vals(variants.size());
    for (std::uint32_t seed = 42; seed < 47; ++seed) {
      BoxWorldParams p;
      if (si < 2 || si == 5) {
        p = imu_corridor_params(si == 1);
        if (si == 5) {
          p.acc_bias_b = Vec3(0.05, 0.03, 0.0);
        }
      } else if (si < 4) {
        p = corridor_params();
      }
      p.seed = seed;
      const fs::path dir =
          rep.dir() / "fix_eval" / ("s" + std::to_string(si) + "_" + std::to_string(seed));
      ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
      for (std::size_t vi = 0; vi < variants.size(); ++vi) {
        const fs::path vdir = dir / ("v" + std::to_string(vi));
        const std::string& backend = variants[vi].second;
        double v = 0.0;
        if (si < 2 || si == 5) {
          v = corridor_axis_run(p, dir / "data", vdir, false, backend).end_x_m;
        } else if (si < 4) {
          v = run_corridor(p, dir / "data", vdir, si == 3, "", kCorridorRelaxYaml + backend).ate_m;
        } else {
          v = trajectory_errors(
                  p, replay(dir / "data", box_config(p, false, false, vdir, backend)).estimates)
                  .ate_m;
        }
        vals[vi].push_back(v);
      }
    }
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
      std::vector<double> a;
      std::string per;
      for (const double v : vals[vi]) {
        a.push_back(std::abs(v));
        char b[32];
        std::snprintf(b, sizeof(b), "%s%+.3f", per.empty() ? "" : " ", v);
        per += b;
      }
      std::sort(a.begin(), a.end());
      char buf[512];
      std::snprintf(buf,
                    sizeof(buf),
                    "| %s | %s | %s | %.3f | %.3f | %s |",
                    scenes[si].name.c_str(),
                    scenes[si].metric.c_str(),
                    variants[vi].first.c_str(),
                    a[a.size() / 2],
                    a.back(),
                    per.c_str());
      rep.line(buf);
    }
  }
}

// ---------------------------------------------------------------------------------------
// Round 2 (imu_only_observability.md sections 10, 11): why the static-gravity factor
// degraded the 60 s GNSS corridor. Variants of the factor's noise and form against the
// corridor with GNSS, the box-world LiDAR NEES, and the tilt wander of the LiDAR chain.

using CovFn = std::function<common::Mat3(const Vec3&, const common::Mat3&)>;

struct Round2Run {
  double ate_m = 0.0;
  double nees6 = 0.0;
  double tilt_factor_rms_rad = 0.0;  // per axis, tilt part of the LiDAR factors' rotation error
  std::size_t n_factors = 0;
  std::uint64_t gravity_removed = 0;    // Stats::n_gravity_factor_removed
  std::uint64_t gravity_unchecked = 0;  // Stats::n_gravity_factor_unchecked
  std::string gravity_check;            // ImuStudyObserver::gravity_check()
  std::vector<ImuStudyObserver::Keyframe> keyframes;
  std::vector<std::pair<Timestamp, common::Pose>> frames;
};

// Study seam (backend study_seams.hpp): the round-1 full 3x1 residual for the scope when
// `full`; a scope with `full` false changes nothing (an enclosing scope stays in force).
class FullResidualScope {
 public:
  explicit FullResidualScope(bool full) : full_(full) {
    if (full_) {
      backend::detail::set_static_gravity_full_residual(true);
    }
  }
  ~FullResidualScope() {
    if (full_) {
      backend::detail::set_static_gravity_full_residual(false);
    }
  }
  FullResidualScope(const FullResidualScope&) = delete;
  FullResidualScope& operator=(const FullResidualScope&) = delete;

 private:
  bool full_;
};

// Study seam (backend study_seams.hpp): the fix-round-1 posterior statistic of the check for
// the scope when `on`; a scope with `on` false changes nothing.
class PosteriorStatisticScope {
 public:
  explicit PosteriorStatisticScope(bool on) : on_(on) {
    if (on_) {
      backend::detail::set_static_gravity_posterior_statistic(true);
    }
  }
  ~PosteriorStatisticScope() {
    if (on_) {
      backend::detail::set_static_gravity_posterior_statistic(false);
    }
  }
  PosteriorStatisticScope(const PosteriorStatisticScope&) = delete;
  PosteriorStatisticScope& operator=(const PosteriorStatisticScope&) = delete;

 private:
  bool on_;
};

// `corridor`: the 60 s corridor overlay (max_range 10 m, degeneracy relax 1e8), with GNSS
// when `gnss`; else the box world with the plain overlay.
Round2Run round2_run(const BoxWorldParams& p,
                     const fs::path& data,
                     const fs::path& dir,
                     bool corridor,
                     bool gnss,
                     const std::string& backend_yaml,
                     const CovFn& gcov,
                     bool full_residual = false) {
  const std::string extra = corridor ? std::string("    preprocess:\n      max_range_m: 10.0\n") +
                                           kCorridorRelaxYaml + backend_yaml
                                     : backend_yaml;
  const Config cfg = box_config(p, gnss, false, dir, extra);
  ImuStudyObserver obs(p);
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  hooks.static_gravity_cov = gcov;
  const FullResidualScope full(full_residual);
  const HooksScope scope(&hooks);
  const ReplayResult r = replay(data, cfg);
  Round2Run out;
  out.ate_m = trajectory_errors(p, r.estimates).ate_m;
  out.nees6 = obs.nees6();
  out.tilt_factor_rms_rad = obs.tilt_err_rms();
  out.n_factors = obs.n_factors();
  out.gravity_removed = r.stats.n_gravity_factor_removed;
  out.gravity_unchecked = r.stats.n_gravity_factor_unchecked;
  out.gravity_check = obs.gravity_check();
  out.keyframes = obs.keyframes();
  out.frames = obs.frames();
  return out;
}

// Attitude error of a keyframe in w (rad): x / y tilt, z yaw.
Vec3 attitude_error_w(const BoxWorldParams& p, const ImuStudyObserver::Keyframe& k) {
  return common::so3_log(k.x.T_w_b.R * test::box_truth(p, k.t).T_w_b.R.transpose());
}

double median_of(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v[v.size() / 2];
}

std::string joined(const std::vector<double>& v, const char* fmt) {
  std::string s;
  for (const double x : v) {
    char b[32];
    std::snprintf(b, sizeof(b), fmt, x);
    s += (s.empty() ? "" : " ") + std::string(b);
  }
  return s;
}

TEST(PipelineComponent, ImuStaticGravityRound2Study) {
  const char* on = std::getenv("MSS_IMU_STUDY");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_IMU_STUDY=1 (GTSAM build)";
  }
  StudyReport rep("imu_round2.md", "MSS_IMU_STUDY_RUN_ID", "imu-study");
  rep.line("# Static-gravity factor, round 2 (generated by ImuStaticGravityRound2Study)");
  const auto scaled = [](double s2) -> CovFn {
    return [s2](const Vec3&, const common::Mat3& c) -> common::Mat3 { return s2 * c; };
  };
  // Direction only: (1 km/s^2)^2 added along the measured specific force, so the factor
  // keeps the two tilt components and drops the magnitude (|g| against ba_z).
  const CovFn direction_only = [](const Vec3& f, const common::Mat3& c) -> common::Mat3 {
    const Vec3 u = f.normalized();
    const common::Mat3 out = c + 1.0e6 * u * u.transpose();
    return out;
  };
  const std::string off = "    backend:\n      static_gravity_factor: false\n";
  // Scale 1: hook-scaled noise is relative to the window std of the mean (round-1 rows).
  const std::string fac =
      "    backend:\n      static_gravity_factor: true\n      static_gravity_sigma_scale: 1.0\n";
  // The full residual has 3 DoF: its check uses the 3-DoF 0.95 quantile.
  const std::string fac_full = fac + "      static_gravity_chi2_gate: 7.814727903251178\n";
  // 10d: the static window's tilt sigma (0.42 mrad = 0.024 deg) as the keyframe-0 rotation
  // prior, no factor (it applies to yaw too: the yaw of w is a gauge).
  const std::string tilt_prior = off + "      prior_rot_sigma_deg: 0.024\n";
  // `full`: the round-1 full 3x1 residual through the study seam (FullResidualScope).
  struct Variant {
    std::string name;
    std::string yaml;
    CovFn cov;
    bool full = false;
  };
  const std::vector<Variant> variants = {
      {"main (factor off)", off, {}},
      {"full residual, window sigma", fac_full, {}, true},
      {"full residual, sigma / 10", fac_full, scaled(0.01), true},
      {"full residual, sigma x 3", fac_full, scaled(9.0), true},
      {"full residual, sigma x 10", fac_full, scaled(100.0), true},
      {"direction only, window sigma", fac, direction_only},
      {"tilt prior 0.42 mrad, no factor", tilt_prior, {}},
      {"shipped default (direction only, scale 2)", "", {}},
  };
  const std::vector<std::uint32_t> seeds = {42, 43, 44, 45, 46};

  // 10b: magnitude of the window mean against the pipeline's gravity constant.
  rep.line("");
  rep.line("## 10b. |mean specific force of the init window| - imu.gravity_mps2, m/s^2");
  rep.line("");
  {
    std::vector<double> dg;
    for (const std::uint32_t seed : seeds) {
      BoxWorldParams p = corridor_params();
      p.seed = seed;
      std::mt19937 rng(seed);
      Vec3 mean = Vec3::Zero();
      int n = 0;
      for (std::int64_t t = p.t0_ns; t <= p.t0_ns + test::box_ns(2.0);
           t += test::box_ns(1.0 / p.imu_rate_hz)) {
        mean += test::box_imu(p, Timestamp{t}, &rng).acc_b;
        ++n;
      }
      mean /= static_cast<double>(n);
      dg.push_back(mean.norm() - test::kBoxGravity);
    }
    rep.line("seeds 42..46: " + joined(dg, "%+.5f") +
             " (box world and sim/mss_sim/synth.py: 9.80665 m/s^2, as imu.gravity_mps2)");
  }

  // 10a, 10b, 10d: GNSS corridor ATE and box-world NEES per variant over the seeds.
  rep.line("");
  rep.line("## 10a/b/d. Variants over seeds 42..46");
  rep.line("");
  rep.line(
      "| variant | GNSS corridor ATE median (max) (m) | per seed (m) | box NEES median (exp. 6) | "
      "per seed | box ATE median (mm) | end E yaw err median (mrad) | end E pos err median "
      "(m) | end kf tilt err median (mrad) |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- | --- | --- |");
  std::vector<Round2Run> series;  // seed 42 GNSS corridor runs, per variant
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    const Variant& v = variants[vi];
    std::vector<double> ate, nees, box_ate, e_yaw, e_pos, tilt;
    for (const std::uint32_t seed : seeds) {
      BoxWorldParams pc = corridor_params();
      pc.seed = seed;
      const fs::path dc = rep.dir() / "round2" / ("corridor_" + std::to_string(seed));
      if (vi == 0) {
        ASSERT_TRUE(test::write_box_dataset(dc / "data", pc).is_ok());
      }
      const Round2Run rc = round2_run(
          pc, dc / "data", dc / ("v" + std::to_string(vi)), true, true, v.yaml, v.cov, v.full);
      BoxWorldParams pb;
      pb.seed = seed;
      const fs::path db = rep.dir() / "round2" / ("box_" + std::to_string(seed));
      if (vi == 0) {
        ASSERT_TRUE(test::write_box_dataset(db / "data", pb).is_ok());
      }
      const Round2Run rb = round2_run(
          pb, db / "data", db / ("v" + std::to_string(vi)), false, false, v.yaml, v.cov, v.full);
      ate.push_back(rc.ate_m);
      nees.push_back(rb.nees6);
      box_ate.push_back(rb.ate_m * 1e3);
      if (!rc.frames.empty()) {
        const common::Pose T_true = test::box_T_e_w(pc);
        const common::Pose& E = rc.frames.back().second;
        e_yaw.push_back(std::abs(common::so3_log(T_true.R.transpose() * E.R).z()) * 1e3);
        e_pos.push_back((E.p - T_true.p).norm());
      }
      if (!rc.keyframes.empty()) {
        tilt.push_back(attitude_error_w(pc, rc.keyframes.back()).head<2>().norm() * 1e3);
      }
      if (seed == 42) {
        series.push_back(rc);
      }
    }
    char buf[512];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %.2f (%.2f) | %s | %.2f | %s | %.2f | %.2f | %.2f | %.2f |",
                  v.name.c_str(),
                  median_of(ate),
                  *std::max_element(ate.begin(), ate.end()),
                  joined(ate, "%.2f").c_str(),
                  median_of(nees),
                  joined(nees, "%.1f").c_str(),
                  median_of(box_ate),
                  median_of(e_yaw),
                  median_of(e_pos),
                  median_of(tilt));
    rep.line(buf);
  }

  // 10c: seed 42 GNSS corridor over time.
  rep.line("");
  rep.line("## 10c. GNSS corridor, seed 42, backend keyframe and E errors over time");
  rep.line("");
  rep.line(
      "| variant | t (s) | x err (m) | z err (m) | kf tilt err about w x / y (mrad) | kf yaw err "
      "(mrad) | ba_b x / z (m/s^2) | E tilt x / y (mrad) | E yaw err (mrad) | E pos err (m) |");
  rep.line("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |");
  const BoxWorldParams p42 = corridor_params();
  const common::Pose T_true = test::box_T_e_w(p42);
  for (const std::size_t vi :
       {std::size_t{0}, std::size_t{1}, std::size_t{5}, std::size_t{6}, std::size_t{7}}) {
    const Round2Run& rc = series[vi];
    double next_s = 10.0;
    for (std::size_t k = 0; k < rc.keyframes.size(); ++k) {
      const ImuStudyObserver::Keyframe& kf = rc.keyframes[k];
      const double ts = static_cast<double>(kf.t.ns - p42.t0_ns) * 1e-9;
      if (ts < next_s && k + 1 != rc.keyframes.size()) {
        continue;
      }
      next_s = ts + 10.0;
      const common::NavState truth = test::box_truth(p42, kf.t);
      const Vec3 a = attitude_error_w(p42, kf);
      Vec3 e_rot = Vec3::Constant(std::nan(""));
      double e_pos = std::nan("");
      for (const auto& fe : rc.frames) {
        if (fe.first == kf.t) {
          e_rot = common::so3_log(T_true.R.transpose() * fe.second.R);
          e_pos = (fe.second.p - T_true.p).norm();
        }
      }
      char buf[384];
      std::snprintf(buf,
                    sizeof(buf),
                    "| %s | %.1f | %+.3f | %+.3f | %+.2f / %+.2f | %+.2f | %+.4f / %+.4f | "
                    "%+.2f / %+.2f | %+.2f | %.3f |",
                    variants[vi].name.c_str(),
                    ts,
                    kf.x.T_w_b.p.x() - truth.T_w_b.p.x(),
                    kf.x.T_w_b.p.z() - truth.T_w_b.p.z(),
                    a.x() * 1e3,
                    a.y() * 1e3,
                    a.z() * 1e3,
                    kf.x.bias.ba.x(),
                    kf.x.bias.ba.z(),
                    e_rot.x() * 1e3,
                    e_rot.y() * 1e3,
                    e_rot.z() * 1e3,
                    e_pos);
      rep.line(buf);
    }
  }

  // Section 11: tilt wander of the LiDAR chain, 60 s corridor without GNSS.
  rep.line("");
  rep.line("## 11. Tilt wander, 60 s corridor, LiDAR + IMU (relax 1e8), seeds 42..46");
  rep.line("");
  rep.line(
      "| variant | kf tilt err median at 10 / 20 / 40 / 60 s (mrad) | LiDAR factor tilt err RMS "
      "per axis (mrad) | factors | rms sqrt(N) (mrad) | ATE median (m) |");
  rep.line("| --- | --- | --- | --- | --- | --- |");
  for (const std::size_t vi : {std::size_t{0}, std::size_t{1}, std::size_t{5}, std::size_t{7}}) {
    std::vector<std::vector<double>> at(4);
    std::vector<double> ate, rms, nf;
    for (const std::uint32_t seed : seeds) {
      BoxWorldParams pc = corridor_params();
      pc.seed = seed;
      const fs::path dc = rep.dir() / "round2" / ("corridor_" + std::to_string(seed));
      const Round2Run r = round2_run(pc,
                                     dc / "data",
                                     dc / ("lidar_v" + std::to_string(vi)),
                                     true,
                                     false,
                                     variants[vi].yaml,
                                     variants[vi].cov,
                                     variants[vi].full);
      const double marks[4] = {10.0, 20.0, 40.0, 60.0};
      for (std::size_t m = 0; m < 4; ++m) {
        double best = 1e9;
        double val = 0.0;
        for (const ImuStudyObserver::Keyframe& kf : r.keyframes) {
          const double ts = static_cast<double>(kf.t.ns - pc.t0_ns) * 1e-9;
          if (std::abs(ts - marks[m]) < best) {
            best = std::abs(ts - marks[m]);
            val = attitude_error_w(pc, kf).head<2>().norm() * 1e3;
          }
        }
        at[m].push_back(val);
      }
      ate.push_back(r.ate_m);
      rms.push_back(r.tilt_factor_rms_rad * 1e3);
      nf.push_back(static_cast<double>(r.n_factors));
    }
    char buf[384];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %.2f / %.2f / %.2f / %.2f | %.3f | %.0f | %.2f | %.2f |",
                  variants[vi].name.c_str(),
                  median_of(at[0]),
                  median_of(at[1]),
                  median_of(at[2]),
                  median_of(at[3]),
                  median_of(rms),
                  median_of(nf),
                  median_of(rms) * std::sqrt(median_of(nf)),
                  median_of(ate));
    rep.line(buf);
  }
}

// Section 10e / 12: the candidate forms over 15 seeds (42..56) on every scene that decides:
// the 60 s corridor with GNSS (ATE) and LiDAR only (ATE), the box-world LiDAR NEES, and the
// 20 s IMU-only corridors with x unobserved (end x error, along-x RMS, max |x|/sigma_DR(t)).
// Per variant: medians and maxima; against main, paired per-seed wins / losses and a 95 %
// bootstrap CI (2000 resamples of the 15 seed pairs, fixed seed) of the median difference.
// MSS_IMU_R2_FIRST / MSS_IMU_R2_LAST select a subset of the variants (default all).

struct SeedScores {
  std::vector<double> gnss_ate, lidar_ate, nees, circle_end, circle_rms, circle_ratio, straight_end,
      straight_rms, straight_ratio;
  // Per scene (kScenes order): factors removed / not checked by the consistency check, and
  // the check of every seed ("kept 1.2", "REMOVED 43.0", ...).
  std::array<std::uint64_t, 5> removed{};
  std::array<std::uint64_t, 5> unchecked{};
  std::array<std::vector<std::string>, 5> checks;
  std::string circle_max_detail;  // per seed: t of the max ratio and the tilt then
};

constexpr std::array<const char*, 5> kScenes = {"GNSS 60 s corridor",
                                                "LiDAR-only 60 s corridor",
                                                "box NEES",
                                                "IMU-only circle",
                                                "IMU-only straight"};

// 95 % bootstrap CI of median(a_i - b_i) over paired seeds (2000 resamples, seed 7).
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

// "median a / median b | wins/losses | CI": wins = seeds where a < b (lower is better).
std::string paired(const std::vector<double>& a, const std::vector<double>& b, bool abs_value) {
  std::vector<double> x = a;
  std::vector<double> y = b;
  if (abs_value) {
    for (double& v : x) {
      v = std::abs(v);
    }
    for (double& v : y) {
      v = std::abs(v);
    }
  }
  int wins = 0;
  int losses = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    wins += x[i] < y[i] ? 1 : 0;
    losses += x[i] > y[i] ? 1 : 0;
  }
  const std::pair<double, double> ci = bootstrap_median_diff(x, y);
  char buf[160];
  std::snprintf(buf,
                sizeof(buf),
                "%.2f (max %.2f) | %d/%d | [%+.2f, %+.2f]",
                median_of(x),
                *std::max_element(x.begin(), x.end()),
                wins,
                losses,
                ci.first,
                ci.second);
  return buf;
}

TEST(PipelineComponent, ImuStaticGravityRound2Seeds) {
  const char* on = std::getenv("MSS_IMU_STUDY");
  if (on == nullptr || std::string(on) != "1" || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_IMU_STUDY=1 (GTSAM build)";
  }
  StudyReport rep("imu_round2_seeds.md", "MSS_IMU_STUDY_RUN_ID", "imu-study");
  rep.line("# Static-gravity factor forms over seeds 42..56 (ImuStaticGravityRound2Seeds)");
  const std::string off = "    backend:\n      static_gravity_factor: false\n";
  const auto scale = [](const char* s) {
    return std::string("    backend:\n      static_gravity_factor: true\n") +
           "      static_gravity_sigma_scale: " + s + "\n";
  };
  // The full residual has 3 DoF: its check uses the 3-DoF 0.95 quantile (N-8).
  const std::string gate3 = "      static_gravity_chi2_gate: 7.814727903251178\n";
  // No check (the round-2 code path): the check keyframe is never reached.
  const std::string no_check = "      static_gravity_check_keyframes: 1000000\n";
  struct Variant {
    std::string name;
    std::string yaml;
    bool full = false;
    bool posterior = false;  // fix-round-1 posterior statistic (study seam)
  };
  const std::vector<Variant> variants = {
      {"main (factor off)", off},
      {"full residual, window sigma (study seam)", scale("1.0") + gate3, true},
      {"shipped form, scale 1", scale("1.0")},
      {"shipped form, scale 1.5", scale("1.5")},
      {"shipped form, scale 2 (default)", scale("2.0")},
      {"shipped form, scale 3", scale("3.0")},
      {"scale 1, no check (round 2)", scale("1.0") + no_check},
      {"scale 1, posterior statistic (fix round 1)", scale("1.0"), false, true},
      {"scale 2, no check (round 2)", scale("2.0") + no_check},
      {"scale 2, posterior statistic (fix round 1)", scale("2.0"), false, true},
  };
  const char* first_env = std::getenv("MSS_IMU_R2_FIRST");
  const char* last_env = std::getenv("MSS_IMU_R2_LAST");
  const std::size_t v_first = first_env != nullptr ? std::stoul(first_env) : 0;
  const std::size_t v_last = last_env != nullptr ? std::stoul(last_env) : variants.size() - 1;
  std::vector<std::uint32_t> seeds;
  for (std::uint32_t s = 42; s < 57; ++s) {
    seeds.push_back(s);
  }
  std::vector<SeedScores> scores(variants.size());
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    if (vi != 0 && (vi < v_first || vi > v_last)) {
      continue;  // main always runs: it is the paired reference
    }
    const Variant& v = variants[vi];
    SeedScores& sc = scores[vi];
    const FullResidualScope full(v.full);
    const PosteriorStatisticScope posterior(v.posterior);
    for (const std::uint32_t seed : seeds) {
      const std::string tag = std::to_string(seed);
      const std::string vd = "v" + std::to_string(vi);
      BoxWorldParams pc = corridor_params();
      pc.seed = seed;
      const fs::path dc = rep.dir() / "round2s" / ("corridor_" + tag);
      BoxWorldParams pb;
      pb.seed = seed;
      const fs::path db = rep.dir() / "round2s" / ("box_" + tag);
      BoxWorldParams p1 = imu_corridor_params(false);
      p1.seed = seed;
      const fs::path d1 = rep.dir() / "round2s" / ("circle_" + tag);
      BoxWorldParams p2 = imu_corridor_params(true);
      p2.seed = seed;
      const fs::path d2 = rep.dir() / "round2s" / ("straight_" + tag);
      if (vi == 0) {
        ASSERT_TRUE(test::write_box_dataset(dc / "data", pc).is_ok());
        ASSERT_TRUE(test::write_box_dataset(db / "data", pb).is_ok());
        ASSERT_TRUE(test::write_box_dataset(d1 / "data", p1).is_ok());
        ASSERT_TRUE(test::write_box_dataset(d2 / "data", p2).is_ok());
      }
      const Round2Run rg = round2_run(pc, dc / "data", dc / vd, true, true, v.yaml, {});
      const Round2Run rl = round2_run(pc, dc / "data", dc / ("l" + vd), true, false, v.yaml, {});
      const Round2Run rb = round2_run(pb, db / "data", db / vd, false, false, v.yaml, {});
      sc.gnss_ate.push_back(rg.ate_m);
      sc.lidar_ate.push_back(rl.ate_m);
      sc.nees.push_back(rb.nees6);
      ImuStudyObserver oc(p1);
      ImuStudyObserver os(p2);
      AxisRun c;
      AxisRun s;
      {
        detail::CoreTestHooks hooks;
        hooks.observer = &oc;
        const HooksScope scope(&hooks);
        c = corridor_axis_run(p1, d1 / "data", d1 / vd, false, v.yaml);
      }
      {
        detail::CoreTestHooks hooks;
        hooks.observer = &os;
        const HooksScope scope(&hooks);
        s = corridor_axis_run(p2, d2 / "data", d2 / vd, false, v.yaml);
      }
      const std::array<std::uint64_t, 5> rem = {rg.gravity_removed,
                                                rl.gravity_removed,
                                                rb.gravity_removed,
                                                c.gravity_removed,
                                                s.gravity_removed};
      const std::array<std::uint64_t, 5> unc = {rg.gravity_unchecked,
                                                rl.gravity_unchecked,
                                                rb.gravity_unchecked,
                                                c.gravity_unchecked,
                                                s.gravity_unchecked};
      const std::array<std::string, 5> chk = {rg.gravity_check,
                                              rl.gravity_check,
                                              rb.gravity_check,
                                              oc.gravity_check(),
                                              os.gravity_check()};
      for (std::size_t k = 0; k < kScenes.size(); ++k) {
        sc.removed[k] += rem[k];
        sc.unchecked[k] += unc[k];
        sc.checks[k].push_back(std::to_string(seed) + ": " + chk[k]);
      }
      sc.circle_end.push_back(c.end_x_m);
      sc.circle_rms.push_back(c.along_rms_m);
      sc.circle_ratio.push_back(c.max_ratio);
      sc.straight_end.push_back(s.end_x_m);
      sc.straight_rms.push_back(s.along_rms_m);
      sc.straight_ratio.push_back(s.max_ratio);
      char b[96];
      std::snprintf(b,
                    sizeof(b),
                    "%s%u: %.1f at %.1f s, tilt %.2f mrad",
                    sc.circle_max_detail.empty() ? "" : "; ",
                    seed,
                    c.max_ratio,
                    c.t_at_max_s,
                    c.tilt_at_max_rad * 1e3);
      sc.circle_max_detail += b;
    }
  }
  const SeedScores& m = scores[0];
  rep.line("");
  rep.line(
      "Cells: median (max) | wins / losses against main over the 15 paired seeds | 95 % "
      "bootstrap CI of the median paired difference (variant - main). Lower is better.");
  const auto table = [&](const char* title, auto get, bool abs_value) {
    rep.line("");
    rep.line(std::string("## ") + title);
    rep.line("");
    rep.line("| variant | median (max) | wins / losses | CI of median diff |");
    rep.line("| --- | --- | --- | --- |");
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
      if (get(scores[vi]).empty()) {
        continue;
      }
      rep.line("| " + variants[vi].name + " | " + paired(get(scores[vi]), get(m), abs_value) +
               " |");
    }
  };
  table("60 s corridor + GNSS, ATE (m)", [](const SeedScores& s) { return s.gnss_ate; }, false);
  table(
      "60 s corridor, LiDAR + IMU, ATE (m)",
      [](const SeedScores& s) { return s.lidar_ate; },
      false);
  table("Box world LiDAR factor NEES (exp. 6)", [](const SeedScores& s) { return s.nees; }, false);
  table(
      "20 s circle x free, |end x err| (m)",
      [](const SeedScores& s) { return s.circle_end; },
      true);
  table(
      "20 s circle x free, along-x RMS (m)",
      [](const SeedScores& s) { return s.circle_rms; },
      false);
  table(
      "20 s circle x free, max |x err(t)| / sigma_DR(t)",
      [](const SeedScores& s) { return s.circle_ratio; },
      false);
  table(
      "20 s straight x free, |end x err| (m)",
      [](const SeedScores& s) { return s.straight_end; },
      true);
  table(
      "20 s straight x free, along-x RMS (m)",
      [](const SeedScores& s) { return s.straight_rms; },
      false);
  table(
      "20 s straight x free, max |x err(t)| / sigma_DR(t)",
      [](const SeedScores& s) { return s.straight_ratio; },
      false);
  rep.line("");
  rep.line("## Consistency check per scene: removed / not evaluated (of 15 runs)");
  rep.line("");
  rep.line(std::string("| variant | ") + kScenes[0] + " | " + kScenes[1] + " | " + kScenes[2] +
           " | " + kScenes[3] + " | " + kScenes[4] + " |");
  rep.line("| --- | --- | --- | --- | --- | --- |");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    const SeedScores& s = scores[vi];
    if (s.gnss_ate.empty()) {
      continue;
    }
    std::string row = "| " + variants[vi].name + " |";
    for (std::size_t k = 0; k < kScenes.size(); ++k) {
      row += " " + std::to_string(s.removed[k]) + " / " + std::to_string(s.unchecked[k]) + " |";
    }
    rep.line(row);
  }
  rep.line("");
  rep.line("## Consistency check per seed (statistic at the check keyframe)");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    const SeedScores& s = scores[vi];
    if (s.gnss_ate.empty() || vi == 0) {
      continue;
    }
    rep.line("");
    rep.line("- " + variants[vi].name + ":");
    for (std::size_t k = 0; k < kScenes.size(); ++k) {
      std::string line = std::string("  - ") + kScenes[k] + ":";
      for (const std::string& c : s.checks[k]) {
        line += " " + c + ";";
      }
      rep.line(line);
    }
  }
  rep.line("");
  rep.line("## Per seed, and where the circle's largest excursion is");
  rep.line("");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    const SeedScores& s = scores[vi];
    if (s.gnss_ate.empty()) {
      continue;
    }
    rep.line("- " + variants[vi].name + ":");
    rep.line("  - GNSS ATE: " + joined(s.gnss_ate, "%.2f"));
    rep.line("  - LiDAR-only ATE: " + joined(s.lidar_ate, "%.2f"));
    rep.line("  - box NEES: " + joined(s.nees, "%.1f"));
    rep.line("  - circle end / RMS: " + joined(s.circle_end, "%+.2f") + " / " +
             joined(s.circle_rms, "%.2f"));
    rep.line("  - straight end / RMS: " + joined(s.straight_end, "%+.2f") + " / " +
             joined(s.straight_rms, "%.2f"));
    rep.line("  - circle max |x|/sigma_DR: " + s.circle_max_detail);
  }
}

// M-4 of the PR 4d review: where the E-01 per-scan p95 changes with the gravity factor.
// MSS_IMU_LATENCY_DATA = an mssdata directory (the E-01 dataset of run_integration.py),
// MSS_IMU_LATENCY_OVERLAY = the scenario overlay (config/sim/warehouse.yaml). Replays it 5
// times per variant, interleaved, timing run_once() per processed scan, split into keyframe
// and non-keyframe scans (Stats::n_keyframes grew); medians over the repetitions.
TEST(PipelineComponent, ImuStaticGravityLatencySplit) {
  const char* data_env = std::getenv("MSS_IMU_LATENCY_DATA");
  const char* overlay_env = std::getenv("MSS_IMU_LATENCY_OVERLAY");
  if (data_env == nullptr || overlay_env == nullptr || !kWithGtsam) {
    GTEST_SKIP() << "set MSS_IMU_LATENCY_DATA and MSS_IMU_LATENCY_OVERLAY (GTSAM build)";
  }
  StudyReport rep("imu_latency_split.md", "MSS_IMU_STUDY_RUN_ID", "imu-study");
  std::string overlay_text;
  {
    std::ifstream in(overlay_env);
    overlay_text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  struct Variant {
    std::string name;
    bool factor;
  };
  const std::vector<Variant> variants = {{"factor off (main behaviour)", false},
                                         {"factor on (shipped)", true}};
  // [variant][rep] -> {kf p50, kf p95, non-kf p50, non-kf p95, all p95, keyframes, iSAM2 ms}
  std::vector<std::vector<std::vector<double>>> res(variants.size());
  for (int repn = 0; repn < 5; ++repn) {
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
      const fs::path ov = rep.dir() / ("latency_overlay_" + std::to_string(vi) + ".yaml");
      {
        std::ofstream f(ov);
        f << overlay_text << "    backend:\n      static_gravity_factor: "
          << (variants[vi].factor ? "true" : "false") << "\n";
      }
      const auto cfg = load_config(kDefault, ov);
      ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
      auto opened = io::DatasetReader::open(data_env, cfg.value().realtime.max_points_per_scan);
      ASSERT_TRUE(opened.is_ok()) << opened.reason();
      io::DatasetReader reader = std::move(opened).value();
      Estimator est(cfg.value());
      ScanFrame scan(cfg.value().realtime.max_points_per_scan);
      io::Record rec;
      std::vector<double> kf_ms;
      std::vector<double> other_ms;
      std::uint64_t n_scans = 0;
      std::uint64_t n_kf = 0;
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
        const Estimator::Stats st = est.stats();
        if (st.n_scans != n_scans) {
          n_scans = st.n_scans;
          const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
          (st.n_keyframes != n_kf ? kf_ms : other_ms).push_back(ms);
          n_kf = st.n_keyframes;
        }
      }
      est.flush();
      const auto pct = [](std::vector<double> v, double q) {
        std::sort(v.begin(), v.end());
        return v.empty() ? 0.0
                         : v[std::min(v.size() - 1,
                                      static_cast<std::size_t>(q * static_cast<double>(v.size())))];
      };
      std::vector<double> all = kf_ms;
      all.insert(all.end(), other_ms.begin(), other_ms.end());
      res[vi].push_back({pct(kf_ms, 0.5),
                         pct(kf_ms, 0.95),
                         pct(other_ms, 0.5),
                         pct(other_ms, 0.95),
                         pct(all, 0.95),
                         static_cast<double>(kf_ms.size())});
    }
  }
  rep.line("# E-01 per-scan latency split (ImuStaticGravityLatencySplit), median of 5");
  rep.line("");
  rep.line(
      "| variant | keyframe scans | keyframe p50 / p95 (ms) | non-keyframe p50 / p95 (ms) | all "
      "p95 (ms) |");
  rep.line("| --- | --- | --- | --- | --- |");
  for (std::size_t vi = 0; vi < variants.size(); ++vi) {
    std::vector<double> col[6];
    for (const auto& r : res[vi]) {
      for (int k = 0; k < 6; ++k) {
        col[k].push_back(r[static_cast<std::size_t>(k)]);
      }
    }
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "| %s | %.0f | %.3f / %.3f | %.3f / %.3f | %.3f |",
                  variants[vi].name.c_str(),
                  median_of(col[5]),
                  median_of(col[0]),
                  median_of(col[1]),
                  median_of(col[2]),
                  median_of(col[3]),
                  median_of(col[4]));
    rep.line(buf);
  }
}

}  // namespace
}  // namespace mss::pipeline
