#include "estimator_core.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "mss/common/lie.hpp"
#include "mss/common/logger.hpp"
#include "mss/common/span.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "frames.hpp"
#include "gravity_rest.hpp"
#include "imu_trajectory_builder.hpp"

namespace mss::pipeline::detail {

namespace {

const Config& validated(const Config& cfg) {
  const double window_samples = std::ceil(cfg.imu.init_window_s * cfg.imu.rate_hz);
  // +3: both ends of the window, the sample at or before its start and the one at or
  // after the scan start.
  if (!(window_samples + 3.0 <= static_cast<double>(kMaxImuQueue))) {
    throw std::invalid_argument("imu.init_window_s: window does not fit the IMU buffer (" +
                                std::to_string(window_samples) + " samples at imu.rate_hz, max " +
                                std::to_string(kMaxImuQueue - 3) + ")");
  }
  if (cfg.map.window_keyframes <= 0 ||
      static_cast<std::size_t>(cfg.map.window_keyframes) > cfg.map.max_keyframes) {
    throw std::invalid_argument("map.window_keyframes: must be in [1, map.max_keyframes] (got " +
                                std::to_string(cfg.map.window_keyframes) + ")");
  }
  if (cfg.realtime.max_points_per_scan == 0) {
    throw std::invalid_argument("realtime.max_points_per_scan: must be > 0");
  }
  return cfg;
}

preprocess::PreprocessConfig keyframe_preprocess(const Config& cfg) {
  preprocess::PreprocessConfig c;
  c.min_range_m = cfg.preprocess.min_range_m;
  c.max_range_m = cfg.preprocess.max_range_m;
  c.downsample_voxel_m = cfg.map.keyframe_voxel_m;
  c.deskew_enabled = false;  // the input is already deskewed
  return c;
}

// Pose covariance of the first-keyframe prior, used when optimize() fails (stub build).
common::PoseCovariance prior_cov(const backend::BackendConfig& b) {
  common::PoseCovariance c = common::PoseCovariance::Zero();
  const double var_p = b.prior_pos_sigma_m * b.prior_pos_sigma_m;
  const double var_r = b.prior_rot_sigma_rad * b.prior_rot_sigma_rad;
  for (int k = 0; k < 3; ++k) {
    c(k, k) = var_p;
    c(k + 3, k + 3) = var_r;
  }
  return c;
}

const CoreTestHooks* g_test_hooks = nullptr;  // test-only, see set_core_test_hooks

}  // namespace

void set_core_test_hooks(const CoreTestHooks* hooks) {
  g_test_hooks = hooks;
}

EstimatorCore::EstimatorCore(const Config& cfg)
    : cfg_(validated(cfg)),
      T_b_l_(cfg.frames.T_b_l),
      T_l_b_(cfg.frames.T_b_l.inverse()),
      g_w_(0.0, 0.0, -cfg.imu.gravity_mps2),
      init_window_ns_(static_cast<std::int64_t>(std::llround(
          cfg.imu.init_window_s * static_cast<double>(common::kNanosecondsPerSecond)))),
      initializer_(cfg.imu),
      pred_(cfg.imu),
      kf_(cfg.imu),
      scratch_(cfg.imu),
      preprocessor_(cfg.preprocess, cfg.frames.T_b_l, cfg.realtime.max_points_per_scan),
      kf_preprocessor_(
          keyframe_preprocess(cfg), cfg.frames.T_b_l, cfg.realtime.max_points_per_scan),
      map_(cfg.map),
      matcher_(cfg.scan_match, cfg.degeneracy, cfg.realtime.max_points_per_scan),
      graph_(cfg.backend, cfg.imu, cfg.frames.p_b_g),
      wheel_(cfg.wheel_odom, cfg.frames.T_b_o),
      fusion_(cfg.gnss, cfg.frames.p_b_g),
      store_(cfg.map.max_keyframes),
      window_(static_cast<std::size_t>(cfg.map.window_keyframes)),
      init_window_(std::make_unique<std::array<common::ImuSample, kMaxImuQueue>>()),
      traj_(std::make_unique<imu::ImuTrajectory>()),
      gnss_out_(std::make_unique<std::array<common::GnssMeasurement, kMaxAuxQueue>>()),
      early_out_(cfg.gnss.enabled && cfg.gnss.attach_prealign_fixes
                     ? std::make_unique<std::array<EarlyFactor, kMaxAuxQueue>>()
                     : nullptr),
      pre_out_(cfg.realtime.max_points_per_scan),
      kf_cloud_(cfg.realtime.max_points_per_scan) {
  hooks_ = g_test_hooks;
  if (g_test_hooks != nullptr) {
    if (g_test_hooks->force_T_e_w) {
      fusion_.force_T_e_w(g_test_hooks->T_e_w);
    }
    if (g_test_hooks->gnss_shift_oracle) {
      fusion_.set_shift_oracle(&g_test_hooks->gnss_shift_oracle);
    }
  }
}

EstimatorCore::~EstimatorCore() = default;

void EstimatorCore::feed_preintegrators(const common::ImuSample& s) {
  if (s.t <= pred_last_t_) {
    return;
  }
  // A rejected (non-finite) sample is skipped by both: the next one bridges the gap.
  (void)pred_.integrate(s);
  (void)kf_.integrate(s);
  pred_last_t_ = s.t;
}

void EstimatorCore::on_imu(const common::ImuSample& s) {
  if (!s.acc_b.allFinite() || !s.gyro_b.allFinite()) {
    // Bad sensor data: drop it; the neighbours bridge the gap (longer dt).
    ++counters_.n_nonfinite_imu;
    if (!imu_nonfinite_streak_) {
      common::log_warn("non-finite imu sample dropped");
    }
    imu_nonfinite_streak_ = true;
    return;
  }
  imu_nonfinite_streak_ = false;
  if (!imu_.empty() && s.t <= imu_.back().t) {
    return;
  }
  if (imu_.full() && initialised_) {
    // The evicted sample precedes every future scan start: integrate it first.
    feed_preintegrators(imu_.front());
  }
  (void)imu_.push(s);
  process_deferred_gnss();
}

void EstimatorCore::on_wheel(const common::WheelSample& s) {
  if (have_wheel_) {
    if (s.t <= last_wheel_t_) {
      return;
    }
    wheel_period_ns_ = s.t.ns - last_wheel_t_.ns;
    // ADR-023: signed odometer for the track-yaw direction of travel (trapezoid).
    wheel_odometer_m_ +=
        0.5 * (last_wheel_v_mps_ + s.v_mps) * (static_cast<double>(wheel_period_ns_) * 1e-9);
  }
  last_wheel_v_mps_ = s.v_mps;
  have_wheel_ = true;
  last_wheel_t_ = s.t;
  if (wheel_queue_.size() == wheel_queue_.capacity()) {
    common::WheelSample oldest;
    wheel_queue_.pop(oldest);
    consume_wheel(oldest);
  }
  wheel_queue_.push(s);
}

void EstimatorCore::consume_wheel(const common::WheelSample& s) {
  // Gravity rest gate (ADR-020): consumed in time order, so at the init scan (after
  // integrate_wheel(init_t_)) this is the last moving sample at or before init_t_.
  if (std::abs(s.v_mps) > cfg_.imu.init_max_wheel_speed_mps) {
    wheel_moved_ = true;
    last_wheel_motion_t_ = s.t;
  }
  (void)wheel_.integrate(s);
}

void EstimatorCore::integrate_wheel(common::Timestamp t) {
  while (!wheel_queue_.empty() && wheel_queue_.front().t <= t) {
    common::WheelSample s;
    wheel_queue_.pop(s);
    consume_wheel(s);
  }
}

void EstimatorCore::prune_imu(common::Timestamp t) {
  while (imu_.size() > 1 && imu_.at(1).t <= t) {
    imu_.pop_front();
  }
}

void EstimatorCore::reset_prediction() {
  pred_.reset(front_.bias);
  (void)pred_.integrate(boundary_);
}

void EstimatorCore::log_backend_failure(const std::string& reason) {
  ++counters_.n_backend_failures;
  if (!backend_failure_streak_) {
    common::log_warn("backend optimize failed, using the front-end state: " + reason);
  }
  backend_failure_streak_ = true;
}

void EstimatorCore::on_gnss(const common::GnssMeasurement& m) {
  if (!cfg_.gnss.enabled) {
    return;
  }
  if (!has_estimate_) {
    if (early_out_ != nullptr) {  // ADR-022: kept for keyframe 0 (static window)
      if (!fusion_.on_preinit_fix(m).is_ok()) {
        ++counters_.n_gnss_rejected;  // gate_fix, as on the live path (literal, no log)
      }
      sync_prealign_counters();
    }
    return;
  }
  if (m.t < front_.t) {
    // Older than the last processed scan start: the front-end no longer has that pose.
    ++counters_.n_gnss_late;
    if (!gnss_late_streak_) {
      common::log_warn("gnss fix older than the last processed scan, rejected");
    }
    gnss_late_streak_ = true;
    return;
  }
  gnss_late_streak_ = false;
  if (!gnss_deferred_.push(m)) {
    ++counters_.n_gnss_ring_overflow;  // the oldest deferred fix was overwritten
  }
  process_deferred_gnss();
}

void EstimatorCore::on_end_of_data() {
  counters_.n_gnss_unfused_eof += gnss_deferred_.size();
  common::GnssMeasurement m;
  while (gnss_deferred_.pop(m)) {
  }
}

void EstimatorCore::process_deferred_gnss() {
  while (!gnss_deferred_.empty() && !imu_.empty() && imu_.back().t >= gnss_deferred_.front().t) {
    common::GnssMeasurement m;
    gnss_deferred_.pop(m);
    if (m.t < front_.t) {
      ++counters_.n_gnss_late;  // overtaken by a scan while waiting (cannot happen in drain order)
      continue;
    }
    fuse_gnss(m);
  }
}

void EstimatorCore::fuse_gnss(const common::GnssMeasurement& m) {
  // Body pose at m.t (pipeline.md section 5).
  const common::NavState x =
      predict_state_at(imu_, pred_, pred_last_t_, front_, m.t, g_w_, scratch_);
  FixOutcome outcome = FixOutcome::kAccepted;
  // ADR-023: wheel travel since the previous fix, when wheel samples arrived in between.
  double wheel_travel_m = std::numeric_limits<double>::quiet_NaN();
  if (cfg_.wheel_odom.enabled && have_wheel_ && have_gnss_odometer_ &&
      last_wheel_t_.ns > last_fix_odometer_t_.ns) {
    wheel_travel_m = wheel_odometer_m_ - fix_odometer_m_;
  }
  have_gnss_odometer_ = true;
  fix_odometer_m_ = wheel_odometer_m_;
  last_fix_odometer_t_ = m.t;
  (void)fusion_.on_fix(m, x.T_w_b, latest_.cov, &outcome, wheel_travel_m);
  if (early_out_ != nullptr) {
    sync_prealign_counters();
  }
  if (fusion_.n_realign() != counters_.n_gnss_realign) {
    counters_.n_gnss_realign = fusion_.n_realign();
    common::log_warn(
        "gnss: chi-square rejected gnss.max_consecutive_rejects fixes in a row, "
        "aligner restarted");
  }
  if (observer() != nullptr && fusion_.n_alignments() != n_alignments_seen_) {
    n_alignments_seen_ = fusion_.n_alignments();
    observer()->on_alignment(
        m.t, fusion_.T_e_w(), fusion_.align_yaw_sigma_rad(), fusion_.track_aligner());
  }
  if (observer() != nullptr && outcome == FixOutcome::kAccepted) {
    observer()->on_fix_accepted(m.t, fusion_.T_e_w());
  }
  if (outcome == FixOutcome::kAccepted || outcome == FixOutcome::kNotAligned) {
    gnss_reject_streak_ = false;
    return;
  }
  ++counters_.n_gnss_rejected;
  if (!gnss_reject_streak_) {
    // Literal per outcome: no allocation (once per streak of rejections).
    switch (outcome) {
      case FixOutcome::kGateFix:
        common::log_info("gnss fix rejected by gate_fix (status or covariance)");
        break;
      case FixOutcome::kChiSquare:
        common::log_info("gnss fix rejected: chi-square gate exceeded");
        break;
      default:
        common::log_info("gnss fix rejected: non-finite pose");
        break;
    }
  }
  gnss_reject_streak_ = true;
}

void EstimatorCore::sync_prealign_counters() {
  counters_.n_gnss_prealign_attached = fusion_.n_prealign_attached();
  counters_.n_gnss_prealign_static = fusion_.n_prealign_static();
  counters_.n_gnss_prealign_dropped = fusion_.n_prealign_dropped();
  counters_.n_gnss_prealign_not_static = fusion_.n_prealign_not_static();
  counters_.n_gnss_prealign_rejected = fusion_.n_prealign_rejected();
}

void EstimatorCore::attach_prealign_fixes(common::KeyId j, common::Timestamp t) {
  fusion_.on_keyframe(j, t, front_.T_w_b);
  for (;;) {
    const std::size_t n = fusion_.take_prealign(*early_out_);
    if (n == 0) {
      break;
    }
    for (std::size_t k = 0; k < n; ++k) {
      const EarlyFactor& e = (*early_out_)[k];
      graph_.add_gnss_factor(e.kf, e.m, fusion_.T_e_w());
      if (observer() != nullptr) {
        observer()->on_gnss_factor(e.t_k, e.m, fusion_.T_e_w());
      }
    }
    counters_.n_gnss_factors += n;
  }
  sync_prealign_counters();
}

void EstimatorCore::on_scan(const common::ScanFrame& raw_l) {
  const common::Timestamp t = raw_l.t_start;
  const common::Timestamp t_end = common::scan_end_time(raw_l);

  // 1. Out-of-order or duplicate scan (Review Focus 3).
  if (has_last_scan_ && t <= last_scan_t_) {
    ++counters_.n_skipped_scans;
    common::log_warn("scan not newer than the previous one, skipped");
    return;
  }
  // The buffered IMU does not span [t, t_end]: no IMU up to the scan end (flush() at the
  // end of data, Review Focus 4) or none before the scan start (IMU started after the
  // LiDAR). Nothing is integrated, so the state stays at the previous scan.
  const std::optional<common::ImuSample> s_t = imu_.interpolate(t);
  if (!s_t || imu_.back().t < t_end) {
    ++counters_.n_skipped_scans;
    ++counters_.n_uncovered_scans;
    if (!uncovered_streak_) {
      common::log_warn("scan not covered by imu samples, skipped");
    }
    uncovered_streak_ = true;
    return;
  }
  uncovered_streak_ = false;
  // Fixes up to the IMU now buffered are fused against the previous scan's state first.
  process_deferred_gnss();
  has_last_scan_ = true;
  last_scan_t_ = t;

  // 2. Initialisation or prediction to t (pipeline.md section 1).
  common::NavState predicted;
  if (!initialised_) {
    // Window from the last sample at or before t - init_window_s to the first sample at or
    // after t (it exists: the scan is covered), so it spans at least init_window_s.
    const std::size_t i_start = imu_.first_after(common::Timestamp{t.ns - init_window_ns_});
    const common::Timestamp w0 = i_start > 0 ? imu_.at(i_start - 1).t : imu_.front().t;
    const common::Timestamp w1 = imu_.at(imu_.first_after(common::Timestamp{t.ns - 1})).t;
    const std::size_t n = imu_.copy_range(w0, w1, *init_window_);
    const common::Result<imu::InitState> init =
        initializer_.try_init(common::Span<const common::ImuSample>(init_window_->data(), n));
    if (!init.is_ok()) {
      ++counters_.n_skipped_scans;
      if (!init_failure_logged_) {
        common::log_info("imu initialisation waiting for a static window: " + init.reason());
        init_failure_logged_ = true;
      }
      return;
    }
    boundary_ = *s_t;
    initialised_ = true;
    front_ = common::NavState{};
    front_.t = t;
    front_.T_w_b.R = init.value().R_w_b;
    front_.bias = init.value().bias;
    init_ = init.value();
    init_t_ = t;
    g_w_ = init.value().g_w;
    pred_.reset(front_.bias);
    kf_.reset(front_.bias);
    (void)pred_.integrate(boundary_);
    (void)kf_.integrate(boundary_);
    pred_last_t_ = t;
    integrate_wheel(t);
    // Samples up to t are consumed now; later ones, already queued, do not count.
    init_wheel_moved_ = wheel_moved_ && last_wheel_motion_t_.ns >= t.ns - init_window_ns_ &&
                        last_wheel_motion_t_.ns <= t.ns;
    (void)wheel_.take_delta();
    predicted = front_;
    common::log_info("imu initialised");
  } else {
    for (std::size_t i = imu_.first_after(pred_last_t_); i < imu_.size() && imu_.at(i).t < t; ++i) {
      feed_preintegrators(imu_.at(i));
    }
    boundary_ = *s_t;
    feed_preintegrators(boundary_);
    pred_last_t_ = t;
    predicted = pred_.predict(front_, g_w_);
    predicted.t = t;
    integrate_wheel(t);
  }
  prune_imu(t);

  // 3. Deskew trajectory; 4. preprocessing.
  const bool traj_ok =
      build_imu_trajectory(imu_, predicted, t_end, g_w_, scratch_, *traj_).is_ok() &&
      preprocessor_.process(raw_l, *traj_, pre_out_).is_ok();
  if (!traj_ok) {
    ++counters_.n_skipped_scans;
    front_ = predicted;
    reset_prediction();
    return;
  }

  // 5. Keyframe 0.
  if (!has_estimate_) {
    first_keyframe(t, predicted);
    return;
  }

  // 6. Scan-to-map matching from the IMU prediction.
  const common::Result<common::MatchResult> match =
      matcher_.align(pre_out_, map_, predicted.T_w_b * T_b_l_);
  if (!match.is_ok()) {
    ++counters_.n_match_failures;
    if (!match_failure_streak_) {
      common::log_warn("scan match failed, following the imu prediction: " + match.reason());
    }
    match_failure_streak_ = true;
    front_ = predicted;
    reset_prediction();
    latest_.state = front_;  // cov and keyframe id unchanged
    return;
  }
  match_failure_streak_ = false;

  // 7. Matched body state (pipeline.md section 3).
  const common::MatchResult& m = match.value();
  const common::PoseCovariance cov_b = lidar_cov_in_body(m.cov, T_b_l_);
  front_.t = t;
  front_.T_w_b = m.T_w_l * T_l_b_;
  front_.v_w_b = predicted.v_w_b;
  front_.bias = predicted.bias;
  common::PoseCovariance cov_out = cov_b;
  if (observer() != nullptr) {
    observer()->on_scan_matched(t, predicted.T_w_b, front_.T_w_b, m);
  }

  // 8. Keyframe.
  bool keyframe = false;
  if (hooks_ != nullptr && hooks_->keyframe_every_n_scans > 0) {  // test-only rule
    keyframe = ++scans_since_keyframe_ >= hooks_->keyframe_every_n_scans;
  } else {
    keyframe = is_new_keyframe(ref_, front_.T_w_b, cfg_.backend);
  }
  if (keyframe) {
    scans_since_keyframe_ = 0;
    add_keyframe(t, m, cov_b, cov_out);
  }

  // 9. Map pruning on sensor time.
  if (common::seconds_between(last_prune_t_, t) >= cfg_.map.prune_period_s) {
    map_.prune(front_.T_w_b.p);
    last_prune_t_ = t;
  }

  // 10. Publish.
  reset_prediction();
  latest_.state = front_;
  latest_.cov = cov_out;
  latest_.keyframe_id = last_kf_id_;
}

void EstimatorCore::first_keyframe(common::Timestamp t, const common::NavState& predicted) {
  const common::KeyId id = graph_.add_keyframe(t, predicted);
  if (cfg_.backend.static_gravity_factor) {
    // The window statistics describe the body at rest at init_t_ (ADR-020): only valid when
    // keyframe 0 is that scan and the rest gate passes (imu_only_observability.md 12).
    const bool wheel_moved = cfg_.wheel_odom.enabled && init_wheel_moved_;
    const char* rest_failure =
        t == init_t_ ? gravity_rest_failure(
                           init_, cfg_.imu, cfg_.backend.prior_bias_acc_sigma_mps2, wheel_moved)
                     : "keyframe 0 is not the init scan";
    if (rest_failure == nullptr) {
      const common::Mat3 cov =
          (hooks_ != nullptr && hooks_->static_gravity_cov)  // test-only override
              ? hooks_->static_gravity_cov(init_.acc_mean_b, init_.acc_mean_cov)
              : init_.acc_mean_cov;
      graph_.add_static_gravity_factor(id, init_.acc_mean_b, cov);
      gravity_factor_added_ = true;
    } else {
      ++counters_.n_gravity_factor_gated;
      common::log_info(std::string("static gravity factor not added: ") + rest_failure);
    }
  }
  front_ = predicted;
  common::PoseCovariance cov = prior_cov(cfg_.backend);
  const common::Result<common::Estimate> opt = graph_.optimize();
  if (opt.is_ok()) {
    front_ = opt.value().state;
    front_.t = t;
    cov = opt.value().cov;
    backend_failure_streak_ = false;
  } else {
    log_backend_failure(opt.reason());
  }
  kf_.reset(front_.bias);
  (void)kf_.integrate(boundary_);
  (void)wheel_.take_delta();  // the odometry chain starts at keyframe 0
  ref_ = front_.T_w_b;
  if (early_out_ != nullptr) {
    // ADR-022: fixes of the IMU initialisation window are at rest at keyframe 0's pose; only
    // when keyframe 0 is that scan and the wheels did not move in it (as ADR-020's gate).
    const bool static_ok = t == init_t_ && !(cfg_.wheel_odom.enabled && init_wheel_moved_);
    fusion_.on_first_keyframe(
        id, t, ref_, cov, common::Timestamp{init_t_.ns - init_window_ns_}, static_ok);
    sync_prealign_counters();
  }
  last_kf_id_ = id;
  last_kf_t_ = t;
  ++counters_.n_keyframes;
  insert_keyframe_cloud(id, t);
  last_prune_t_ = t;
  reset_prediction();
  has_estimate_ = true;
  latest_.state = front_;
  latest_.cov = cov;
  latest_.keyframe_id = id;
}

void EstimatorCore::add_keyframe(common::Timestamp t,
                                 const common::MatchResult& match,
                                 const common::PoseCovariance& cov_b,
                                 common::PoseCovariance& cov_out) {
  const common::KeyId i = last_kf_id_;
  const common::KeyId j = graph_.add_keyframe(t, front_);
  graph_.add_imu_factor(i, j, kf_.delta());
  // LiDAR factor measurement (pipeline.md section 4).
  const common::Pose T_bi_bj = (hooks_ != nullptr && hooks_->lidar_oracle)
                                   ? hooks_->lidar_oracle(last_kf_t_, t)
                                   : ref_.inverse() * front_.T_w_b;
  const common::PoseCovariance cov_f =
      (hooks_ != nullptr && hooks_->lidar_cov) ? hooks_->lidar_cov(t, cov_b) : cov_b;  // test-only
  if (observer() != nullptr) {
    observer()->on_lidar_factor(last_kf_t_, t, T_bi_bj, cov_f);
  }
  if (cfg_.degeneracy.enabled) {
    const common::DegeneracyReport report_l = degeneracy::analyze(match.H, cfg_.degeneracy);
    const common::DegeneracyReport report_b = degeneracy_in_body(report_l, T_b_l_);
    bool any_flagged = false;
    bool rot_flagged = false;
    for (int k = 0; k < 6; ++k) {
      if (report_l.degenerate[static_cast<std::size_t>(k)]) {
        any_flagged = true;
        const common::Vec6 v = report_l.eigvecs.col(k);
        rot_flagged = rot_flagged || v.tail<3>().squaredNorm() > v.head<3>().squaredNorm();
      }
    }
    counters_.n_degenerate_keyframes += any_flagged ? 1U : 0U;
    counters_.n_degenerate_rot_keyframes += rot_flagged ? 1U : 0U;
    graph_.add_lidar_factor(i, j, T_bi_bj, cov_f, &report_b);
  } else {
    graph_.add_lidar_factor(i, j, T_bi_bj, cov_f, nullptr);
  }
  // Wheel odometry between the keyframes (graph.hpp time check).
  const common::OdomDelta d = wheel_.take_delta();
  if (cfg_.wheel_odom.enabled) {
    if (d.t1 > d.t0 && odom_delta_in_time(d, last_kf_t_, t, wheel_period_ns_)) {
      graph_.add_odom_factor(i, j, d);
      ++counters_.n_odom_factors;
    } else {
      ++counters_.n_odom_rejected;
      common::log_warn("wheel delta does not match the keyframe times, factor dropped");
    }
  }
  // GNSS fixes waiting for this keyframe (pipeline.md section 5).
  if (cfg_.gnss.enabled) {
    if (early_out_ != nullptr) {
      attach_prealign_fixes(j, t);  // ADR-022, before the queued ones (same T_e_w)
    }
    const std::size_t n = fusion_.collect(t, front_.T_w_b, *gnss_out_);
    for (std::size_t k = 0; k < n; ++k) {
      graph_.add_gnss_factor(j, (*gnss_out_)[k], fusion_.T_e_w());
      if (observer() != nullptr) {
        observer()->on_gnss_factor(t, (*gnss_out_)[k], fusion_.T_e_w());
      }
    }
    counters_.n_gnss_factors += n;
  }
  const common::Result<common::Estimate> opt = graph_.optimize();
  const bool optimised = opt.is_ok();
  if (optimised) {
    if (observer() != nullptr) {
      observer()->on_keyframe_optimized(t, opt.value().state, opt.value().cov);
      const common::Result<common::Pose> E = graph_.gnss_frame_estimate();
      if (E.is_ok()) {
        observer()->on_frame_estimate(t, E.value());
      }
    }
    if (hooks_ == nullptr || !hooks_->frontend_keeps_own_state) {  // test-only seam
      front_ = opt.value().state;
      front_.t = t;
    }
    cov_out = opt.value().cov;
    backend_failure_streak_ = false;
    check_gravity_factor(j);
  } else {
    log_backend_failure(opt.reason());
  }
  kf_.reset(front_.bias);
  (void)kf_.integrate(boundary_);
  ref_ = front_.T_w_b;
  if (early_out_ != nullptr) {
    fusion_.set_keyframe_reference(j, t, ref_);  // ADR-022: target of backward shifts
  }
  last_kf_id_ = j;
  last_kf_t_ = t;
  ++counters_.n_keyframes;
  insert_keyframe_cloud(j, t);
  if (optimised) {
    maybe_rebuild_map();
  }
}

void EstimatorCore::check_gravity_factor(common::KeyId j) {
  if (!gravity_factor_added_ || gravity_factor_checked_ ||
      cfg_.backend.static_gravity_check_keyframes == 0 ||  // check off (shipped, ADR-020)
      j < static_cast<common::KeyId>(cfg_.backend.static_gravity_check_keyframes)) {
    return;
  }
  gravity_factor_checked_ = true;  // once: a failed removal is not retried (ADR-020)
  const common::Result<double> chi2 = graph_.static_gravity_chi2();
  GravityCheckOutcome outcome = GravityCheckOutcome::kKept;
  if (!chi2.is_ok()) {
    outcome = GravityCheckOutcome::kUnchecked;
    ++counters_.n_gravity_factor_unchecked;
    common::log_info("static gravity factor kept unchecked: " + chi2.reason());
  } else if (chi2.value() > cfg_.backend.static_gravity_chi2_gate) {
    if (graph_.remove_static_gravity_factor()) {
      outcome = GravityCheckOutcome::kRemoved;
      ++counters_.n_gravity_factor_removed;
      common::log_warn(
          "static gravity factor removed: its innovation failed "
          "backend.static_gravity_chi2_gate (the init window was not at rest)");
    } else {
      outcome = GravityCheckOutcome::kRemovalFailed;
      common::log_warn(
          "static gravity factor failed backend.static_gravity_chi2_gate but its removal "
          "failed: the factor stays (not retried)");
    }
  }
  if (observer() != nullptr) {
    observer()->on_gravity_check(
        chi2.is_ok() ? chi2.value() : std::numeric_limits<double>::quiet_NaN(), outcome);
  }
}

void EstimatorCore::insert_keyframe_cloud(common::KeyId id, common::Timestamp t) {
  if (!kf_preprocessor_.process(pre_out_, *traj_, kf_cloud_).is_ok()) {
    kf_cloud_.clear();
  }
  kf_cloud_.t_start = t;
  (void)store_.add(id, t, kf_cloud_);
  map_.insert(kf_cloud_, ref_ * T_b_l_);
  window_.push(id, ref_);
  if (map_.num_dropped() > map_dropped_) {
    map_dropped_ = map_.num_dropped();
    common::log_warn("voxel map full (map.max_voxels): points dropped");
  }
}

void EstimatorCore::maybe_rebuild_map() {
  if (window_.size() == 0) {
    return;
  }
  const WindowEntry& oldest = window_.at(0);
  const common::Result<common::NavState> st = graph_.keyframe_state(oldest.id);
  if (!st.is_ok() || !pose_moved(oldest.T_w_b_inserted,
                                 st.value().T_w_b,
                                 cfg_.map.rebuild_trans_m,
                                 cfg_.map.rebuild_rot_rad)) {
    return;
  }
  counters_.n_map_spacing_replaced_cleared += map_.num_spacing_replaced();
  map_.clear();
  for (std::size_t i = 0; i < window_.size(); ++i) {
    const WindowEntry& e = window_.at(i);
    const KeyframeEntry* kf = store_.find(e.id);
    if (kf == nullptr) {
      continue;
    }
    const common::Result<common::NavState> s = graph_.keyframe_state(e.id);
    const common::Pose T_w_b = s.is_ok() ? s.value().T_w_b : e.T_w_b_inserted;
    map_.insert(kf->cloud_l, T_w_b * T_b_l_);
    window_.set_pose(i, T_w_b);
  }
  map_dropped_ = map_.num_dropped();
  ++counters_.n_map_rebuilds;
  common::log_info("voxel map rebuilt from the keyframe window after a pose correction");
}

}  // namespace mss::pipeline::detail
