#include "estimator_core.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/SVD>

#include "mss/common/logger.hpp"
#include "mss/common/span.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "frames.hpp"
#include "gravity_rest.hpp"
#include "imu_trajectory_builder.hpp"

namespace mss::pipeline::detail {

namespace {

constexpr double kPiRad = 3.14159265358979323846;  // pi, for wrapping heading differences

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
      matcher_(cfg.scan_match, cfg.degeneracy, cfg.realtime.max_points_per_scan),
      backend_(cfg_, g_test_hooks),
      wheel_(cfg.wheel_odom, cfg.frames.T_b_o),
      init_window_(std::make_unique<std::array<common::ImuSample, kMaxImuQueue>>()),
      traj_(std::make_unique<imu::ImuTrajectory>()),
      pre_out_(cfg.realtime.max_points_per_scan),
      kf_cloud_(cfg.realtime.max_points_per_scan) {
  hooks_ = g_test_hooks;
  if (!cfg_.deterministic) {  // ADR-033: the backend thread may be started
    chan_ = std::make_unique<BackendChannel>(cfg.realtime.max_points_per_scan);
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
  }
  // ADR-023: signed odometer for the track-yaw direction of travel (trapezoid between finite
  // samples). A non-finite speed is skipped and counted, so it cannot poison the odometer.
  if (!std::isfinite(s.v_mps)) {
    ++counters_.n_wheel_nonfinite;
  } else {
    // Backlog G9: across a gap of more than gnss.align_yaw_wheel_gap_periods nominal periods
    // the trapezoid is an interpolation (a driver that goes silent while the robot stops); its
    // excess over one period is bounded and counted.
    if (odometer_.push(s.t, s.v_mps)) {
      ++counters_.n_wheel_gap_bounded;
      if (!wheel_gap_logged_) {
        common::log_warn(
            "wheel odometry: a sample gap longer than gnss.align_yaw_wheel_gap_periods nominal "
            "periods; the odometer interpolates across it (see Stats::n_wheel_gap_bounded)");
        wheel_gap_logged_ = true;
      }
    }
  }
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

void EstimatorCore::on_gnss(const common::GnssMeasurement& m) {
  if (!cfg_.gnss.enabled) {
    return;
  }
  if (!has_estimate_) {
    // ADR-022: no front-end pose yet; the backend keeps it for keyframe 0 (static window).
    GnssFixMsg msg;
    msg.m = m;
    msg.preinit = true;
    send_fix(msg);
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
  GnssFixMsg msg;
  msg.m = m;
  // Body pose at m.t (pipeline.md section 5) and the covariance of the latest estimate.
  msg.T_w_b = predict_state_at(imu_, pred_, pred_last_t_, front_, m.t, g_w_, scratch_).T_w_b;
  msg.P_w_b = latest_.cov;
  // ADR-023: the absolute wheel odometer reading, when a finite wheel sample arrived since the
  // previous fix (NaN otherwise). The track-yaw aligner differences the readings of the fixes
  // it consumes, so the travel of a fix rejected before it is carried to the next one.
  gnss::TrackYawReadings& readings = msg.readings;
  if (cfg_.wheel_odom.enabled && odometer_.has_reading() && have_fix_odometer_t_ &&
      odometer_.last_t().ns > last_fix_odometer_t_.ns) {
    readings.wheel_odometer_m = odometer_.reading_m();
  }
  have_fix_odometer_t_ = true;
  last_fix_odometer_t_ = m.t;
  // Backlog G9: the bounds of the gaps the odometer interpolated across (with wheel data).
  if (cfg_.wheel_odom.enabled && odometer_.has_reading()) {
    readings.wheel_gap_bound_m = odometer_.gap_bound_m();
  }
  // Design 2b: the heading variation up to the last scan (NaN before the first one).
  if (have_scan_heading_) {
    readings.heading_variation_rad = heading_variation_rad_;
  }
  // Root-cause study: the count of scans that did not observe the travel axis (NaN when off).
  if (have_scan_heading_ && cfg_.gnss.align_yaw_frontend_travel) {
    readings.travel_unobserved_scans = travel_unobserved_scans_;
  }
  readings.t = m.t;  // the fix time (sensor clock)
  readings.has_time = true;
  send_fix(msg);
}

void EstimatorCore::on_scan(const common::ScanFrame& raw_l) {
  const common::Timestamp t = raw_l.t_start;
  const common::Timestamp t_end = common::scan_end_time(raw_l);
  // ADR-033: the map snapshot the backend published last, held (unchanged) for the whole scan.
  // The front-end only reads it; the backend writes and publishes the next one.
  const MapRef map = backend_.acquire_map();
  if (async_) {
    // ADR-033 decision 4: backend corrections enter at a scan boundary, only those whose keyframe
    // this snapshot holds, so the pose and the map carry the same corrections.
    apply_backend_updates(map->n_keyframes);
  }

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
    if (init_.level_start_fallback) {  // initialisation runs once: logged once
      common::log_warn(
          "imu level start: horizontal mean specific force above "
          "imu.init_level_max_bias_mps2, body taken as tilted (flag-off initialisation)");
    }
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
    if (async_ && !backend_can_take_keyframe()) {
      ++counters_.n_backend_keyframes_dropped;  // backend full: retried at the next scan
      ++counters_.n_skipped_scans;
      front_ = predicted;
      reset_prediction();
      return;
    }
    first_keyframe(t, predicted);
    return;
  }

  // 6. Scan-to-map matching from the IMU prediction.
  const common::Result<common::MatchResult> match =
      matcher_.align(pre_out_, map->map, predicted.T_w_b * T_b_l_);
  if (!match.is_ok()) {
    ++counters_.n_match_failures;
    if (!match_failure_streak_) {
      common::log_warn("scan match failed, following the imu prediction: " + match.reason());
    }
    match_failure_streak_ = true;
    front_ = predicted;
    reset_prediction();
    latest_.state = front_;  // cov and keyframe id unchanged
    track_heading(nullptr);
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
  if (keyframe && async_ && !backend_can_take_keyframe()) {
    // ADR-033 full-queue policy: the keyframe is refused and counted, the front-end never waits;
    // the scan continues as a non-keyframe and the next scan retries (the rule still holds).
    ++counters_.n_backend_keyframes_dropped;
    keyframe = false;
  }
  if (keyframe) {
    scans_since_keyframe_ = 0;
    add_keyframe(t, m, cov_b, cov_out);
  }

  // 9. Map pruning on sensor time.
  if (common::seconds_between(last_prune_t_, t) >= cfg_.map.prune_period_s &&
      send_prune(front_.T_w_b.p)) {
    last_prune_t_ = t;  // a refused request (backend queue full) is retried at the next scan
  }

  // 10. Publish.
  reset_prediction();
  latest_.state = front_;
  latest_.cov = cov_out;
  latest_.keyframe_id = last_kf_id_;
  track_heading(&m);
}

void EstimatorCore::track_heading(const common::MatchResult* match) {
  // Design 2b (gnss.md section 5.1): the total absolute heading variation of the front end,
  // read by the track-yaw turn gate at each fix. Only with gnss.align_yaw_from_track (flag off:
  // nothing runs here).
  if (!(cfg_.gnss.enabled && cfg_.gnss.align_yaw_from_track)) {
    return;
  }
  const double heading_rad = travel_heading_rad(front_.T_w_b.R, cfg_.frames.T_b_o.R);
  if (have_scan_heading_) {
    heading_variation_rad_ +=
        std::abs(std::remainder(heading_rad - last_scan_heading_rad_, 2.0 * kPiRad));
  }
  have_scan_heading_ = true;
  last_scan_heading_rad_ = heading_rad;
  // Backlog G3 (gnss.md section 5.3): a scan observes the travel axis when the marginal
  // information of its matching along the axis reaches the degeneracy module's threshold.
  double unobserved = std::numeric_limits<double>::quiet_NaN();
  if (cfg_.gnss.align_yaw_frontend_travel) {
    // The odometry frame's origin in l: the aligner uses the displacement of o (m-2 of the
    // H4 review), so a rotation error moves it through the lever arm.
    const common::Vec3 p_l_o = T_l_b_ * cfg_.frames.T_b_o.p;
    if (match == nullptr || !travel_axis_observed(front_.T_w_b.R,
                                                  cfg_.frames.T_b_o.R,
                                                  match->T_w_l.R,
                                                  match->H,
                                                  p_l_o,
                                                  cfg_.degeneracy.eig_threshold)) {
      travel_unobserved_scans_ += 1.0;
    }
    unobserved = travel_unobserved_scans_;
  }
  if (observer() != nullptr) {
    observer()->on_track_heading(front_.t, front_.T_w_b, heading_variation_rad_, unobserved);
  }
}

void EstimatorCore::first_keyframe(common::Timestamp t, const common::NavState& predicted) {
  KeyframeMsg msg;
  msg.t = t;
  msg.x0 = predicted;
  msg.first = true;
  if (cfg_.backend.static_gravity_factor) {
    // The window statistics describe the body at rest at init_t_ (ADR-020): only valid when
    // keyframe 0 is that scan and the rest gate passes (imu_only_observability.md 12).
    const bool wheel_moved = cfg_.wheel_odom.enabled && init_wheel_moved_;
    const char* rest_failure =
        t == init_t_ ? gravity_rest_failure(
                           init_, cfg_.imu, cfg_.backend.prior_bias_acc_sigma_mps2, wheel_moved)
                     : "keyframe 0 is not the init scan";
    if (rest_failure == nullptr) {
      msg.gravity_cov = (hooks_ != nullptr && hooks_->static_gravity_cov)  // test-only override
                            ? hooks_->static_gravity_cov(init_.acc_mean_b, init_.acc_mean_cov)
                            : init_.acc_mean_cov;
      msg.gravity_f_b = init_.acc_mean_b;
      msg.add_static_gravity = true;
    } else {
      ++counters_.n_gravity_factor_gated;
      common::log_info(std::string("static gravity factor not added: ") + rest_failure);
    }
  }
  // ADR-022: fixes of the IMU initialisation window are at rest at keyframe 0's pose; only
  // when keyframe 0 is that scan and the wheels did not move in it (as ADR-020's gate).
  msg.prealign_static_from = common::Timestamp{init_t_.ns - init_window_ns_};
  msg.prealign_static_ok = t == init_t_ && !(cfg_.wheel_odom.enabled && init_wheel_moved_);
  common::ScanFrame& cloud = keyframe_cloud_out();
  prepare_keyframe_cloud(t, cloud);
  msg.cloud_l = &cloud;
  // Threaded (ADR-033): the front-end starts from `predicted`; the backend's estimate comes later
  // as a correction (apply_update).
  BackendUpdate u;
  if (async_) {
    u.id = send_keyframe(msg);
  } else {
    u = backend_.add_keyframe(msg);
    next_kf_id_ = u.id + 1;
  }
  const common::KeyId id = u.id;
  front_ = predicted;
  common::PoseCovariance cov = prior_pose_cov(cfg_.backend);
  if (u.optimised) {
    front_ = u.opt.state;
    front_.t = t;
    cov = u.opt.cov;
  }
  kf_.reset(front_.bias);
  (void)kf_.integrate(boundary_);
  (void)wheel_.take_delta();  // the odometry chain starts at keyframe 0
  ref_ = front_.T_w_b;
  last_kf_id_ = id;
  last_kf_t_ = t;
  ++counters_.n_keyframes;
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
  KeyframeMsg msg;
  msg.t = t;
  msg.x0 = front_;
  msg.i = last_kf_id_;
  msg.imu_delta = kf_.delta();
  // LiDAR factor measurement (pipeline.md section 4).
  msg.T_bi_bj = (hooks_ != nullptr && hooks_->lidar_oracle) ? hooks_->lidar_oracle(last_kf_t_, t)
                                                            : ref_.inverse() * front_.T_w_b;
  msg.lidar_cov =
      (hooks_ != nullptr && hooks_->lidar_cov) ? hooks_->lidar_cov(t, cov_b) : cov_b;  // test-only
  if (observer() != nullptr) {
    observer()->on_lidar_factor(last_kf_t_, t, msg.T_bi_bj, msg.lidar_cov);
  }
  if (cfg_.degeneracy.enabled) {
    const common::DegeneracyReport report_l = degeneracy::analyze(match.H, cfg_.degeneracy);
    msg.lidar_degeneracy_b = degeneracy_in_body(report_l, T_b_l_);
    msg.has_degeneracy = true;
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
  }
  // Wheel odometry between the keyframes (graph.hpp time check).
  const common::OdomDelta d = wheel_.take_delta();
  if (cfg_.wheel_odom.enabled) {
    if (d.t1 > d.t0 && odom_delta_in_time(d, last_kf_t_, t, wheel_period_ns_)) {
      msg.odom = d;
      msg.has_odom = true;
      ++counters_.n_odom_factors;
    } else {
      ++counters_.n_odom_rejected;
      common::log_warn("wheel delta does not match the keyframe times, factor dropped");
    }
  }
  common::ScanFrame& cloud = keyframe_cloud_out();
  prepare_keyframe_cloud(t, cloud);
  msg.cloud_l = &cloud;
  // Threaded (ADR-033): the front-end keeps its matched state; the optimised one (and a loop
  // update) comes later as a correction at a scan boundary (apply_update).
  BackendUpdate u;
  if (async_) {
    u.id = send_keyframe(msg);
  } else {
    u = backend_.add_keyframe(msg);
    next_kf_id_ = u.id + 1;
  }
  const common::KeyId j = u.id;
  if (u.optimised) {
    if (hooks_ == nullptr || !hooks_->frontend_keeps_own_state) {  // test-only seam
      front_ = u.opt.state;
      front_.t = t;
    }
    cov_out = u.opt.cov;
  }
  kf_.reset(front_.bias);
  (void)kf_.integrate(boundary_);
  // KeyframeBackend gives GnssFusion the same reference pose (ADR-022) and inserts the keyframe
  // cloud into the map at it.
  ref_ = front_.T_w_b;
  last_kf_id_ = j;
  last_kf_t_ = t;
  ++counters_.n_keyframes;
  if (u.optimised && u.loop_added) {
    // ADR-026: an accepted loop changed the graph; re-sync to the estimate after its updates
    // (the backend has rebuilt the map at the new poses).
    if (hooks_ == nullptr || !hooks_->frontend_keeps_own_state) {  // test-only seam
      front_ = u.loop_opt.state;
      front_.t = t;
    }
    cov_out = u.loop_opt.cov;
    kf_.reset(front_.bias);
    (void)kf_.integrate(boundary_);
    ref_ = front_.T_w_b;
  }
}

void EstimatorCore::prepare_keyframe_cloud(common::Timestamp t, common::ScanFrame& out) {
  if (!kf_preprocessor_.process(pre_out_, *traj_, out).is_ok()) {
    out.clear();
  }
  out.t_start = t;
}

common::ScanFrame& EstimatorCore::keyframe_cloud_out() {
  return async_ ? chan_->clouds[cloud_slot_] : kf_cloud_;
}

MapExportInfo EstimatorCore::visit_map(const MapKeyframeVisitor& visit) const {
  if (async_) {
    const std::lock_guard<std::mutex> lock(chan_->backend_mutex);
    return backend_.visit_map(visit);
  }
  return backend_.visit_map(visit);
}

std::size_t EstimatorCore::graph_snapshot(GraphSnapshot& out) const {
  if (async_) {
    const std::lock_guard<std::mutex> lock(chan_->backend_mutex);
    return backend_.graph_snapshot(out);
  }
  return backend_.graph_snapshot(out);
}

// ---------------------------------------------------------------------------------------------
// ADR-033 backend thread.

namespace {
// Upper bound of one idle wait of the backend thread: catches a notify that arrived between its
// empty-queue check and its wait (the front-end notifies without the mutex, so it never waits).
// Wall time only; never part of an estimate.
constexpr std::chrono::milliseconds kBackendIdleWait{5};

// The rotation nearest to `M` (Frobenius norm, SVD). Composing rotations with corrections
// (T_corr * T, T_corr = T_opt * T_x0^-1 with R^T as the inverse) amplifies a non-orthonormal
// residue of the rotations each time: about 3x per keyframe, until scan matching breaks after a
// few dozen keyframes. Projecting after each correction keeps it at rounding level.
common::Mat3 nearest_rotation(const common::Mat3& M) {
  const Eigen::JacobiSVD<common::Mat3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  common::Mat3 V = svd.matrixV();
  if ((svd.matrixU() * V.transpose()).determinant() < 0.0) {
    V.col(2) = -V.col(2);
  }
  return svd.matrixU() * V.transpose();
}
}  // namespace

void EstimatorCore::start_backend() {
  if (chan_ == nullptr || async_) {
    return;
  }
  chan_->stop.store(false);
  chan_->counters = backend_.counters();  // no thread yet
  backend_.set_defer_publish(true);
  pending_first_kf_ = next_kf_id_;
  has_held_update_ = false;
  async_ = true;
}

void EstimatorCore::notify_backend() {
  chan_->wake.notify_one();
}

void EstimatorCore::request_backend_stop() {
  if (chan_ == nullptr) {
    return;
  }
  chan_->stop.store(true);
  notify_backend();
}

void EstimatorCore::run_backend() {
  BackendChannel& c = *chan_;
  for (;;) {
    if (c.to_backend.pop(c.msg)) {
      handle_backend_msg(c.msg);
      continue;
    }
    if (c.stop.load()) {
      if (c.to_backend.empty()) {
        return;  // every message sent before the stop request is handled
      }
      continue;
    }
    std::unique_lock<std::mutex> lock(c.wake_mutex);
    c.wake.wait_for(
        lock, kBackendIdleWait, [&c] { return !c.to_backend.empty() || c.stop.load(); });
  }
}

void EstimatorCore::handle_backend_msg(BackendMsg& m) {
  BackendChannel& c = *chan_;
  if (m.kind == BackendMsg::Kind::kKeyframe && hooks_ != nullptr &&
      hooks_->before_backend_keyframe) {
    hooks_->before_backend_keyframe(c.n_keyframes_handled);  // test-only slow backend
  }
  bool update_dropped = false;
  {
    const std::lock_guard<std::mutex> lock(c.backend_mutex);
    switch (m.kind) {
      case BackendMsg::Kind::kKeyframe: {
        const BackendUpdate u = backend_.add_keyframe(m.kf);
        // The store copied the cloud: the slot goes back to the front-end.
        c.cloud_busy[m.cloud_slot].store(false);
        ++c.n_keyframes_handled;
        // The update before the snapshot that holds the keyframe (MapBuffer::n_keyframes).
        update_dropped = !c.to_frontend.push(u);
        backend_.publish_deferred_map();
        break;
      }
      case BackendMsg::Kind::kGnssFix:
        backend_.on_gnss_fix(m.fix);
        break;
      case BackendMsg::Kind::kPrune:
        backend_.prune_map(m.p_w_b);
        break;
    }
  }
  const std::lock_guard<std::mutex> lock(c.counters_mutex);
  c.counters = backend_.counters();
  c.n_updates_dropped += update_dropped ? 1U : 0U;
}

bool EstimatorCore::backend_can_take_keyframe() {
  if (chan_->to_backend.size() >= kBackendQueueDepth) {
    return false;  // only the backend pops: the space stays until our push
  }
  for (std::size_t i = 0; i < kKeyframeCloudSlots; ++i) {
    if (!chan_->cloud_busy[i].load()) {
      cloud_slot_ = i;
      return true;
    }
  }
  return false;
}

common::KeyId EstimatorCore::send_keyframe(const KeyframeMsg& msg) {
  BackendChannel& c = *chan_;
  const common::KeyId id = next_kf_id_++;
  if (next_kf_id_ - pending_first_kf_ > kMaxPendingKeyframes) {
    ++pending_first_kf_;  // the oldest pending update was lost; forget it
  }
  pending_x0_[id % kMaxPendingKeyframes] = msg.x0;
  c.cloud_busy[cloud_slot_].store(true);  // published to the backend by the push
  BackendMsg& bm = c.send;
  bm.kind = BackendMsg::Kind::kKeyframe;
  bm.kf = msg;  // msg.cloud_l points into clouds[cloud_slot_] (keyframe_cloud_out)
  bm.cloud_slot = cloud_slot_;
  // Cannot fail: backend_can_take_keyframe() saw room and only this thread pushes.
  (void)c.to_backend.push(bm);
  notify_backend();
  return id;
}

void EstimatorCore::send_fix(const GnssFixMsg& msg) {
  if (!async_) {
    backend_.on_gnss_fix(msg);
    return;
  }
  BackendMsg& bm = chan_->send;
  bm.kind = BackendMsg::Kind::kGnssFix;
  bm.fix = msg;
  if (!chan_->to_backend.push(bm)) {
    ++counters_.n_backend_fixes_dropped;  // ADR-033: refused and counted, never waits
    return;
  }
  notify_backend();
}

bool EstimatorCore::send_prune(const common::Vec3& p_w_b) {
  if (!async_) {
    backend_.prune_map(p_w_b);
    return true;
  }
  BackendMsg& bm = chan_->send;
  bm.kind = BackendMsg::Kind::kPrune;
  bm.p_w_b = p_w_b;
  if (!chan_->to_backend.push(bm)) {
    ++counters_.n_backend_prunes_dropped;
    return false;
  }
  notify_backend();
  return true;
}

void EstimatorCore::apply_backend_updates(std::uint64_t epoch) {
  for (;;) {
    if (!has_held_update_) {
      if (!chan_->to_frontend.pop(held_update_)) {
        return;
      }
      has_held_update_ = true;
    }
    if (held_update_.id >= epoch) {
      return;  // its snapshot is not published yet: applied at a later scan
    }
    has_held_update_ = false;
    apply_update(held_update_);
  }
}

void EstimatorCore::apply_update(const BackendUpdate& u) {
  if (u.id < pending_first_kf_ || u.id >= next_kf_id_) {
    return;  // not pending (forgotten, or before start_backend())
  }
  // Updates arrive in keyframe order; pending keyframes before u.id lost theirs (queue full).
  pending_first_kf_ = u.id + 1;
  if (!u.optimised || (hooks_ != nullptr && hooks_->frontend_keeps_own_state)) {
    return;  // no estimate (stub build, failed optimize): the front-end keeps its state
  }
  const common::NavState& x = u.loop_added ? u.loop_opt.state : u.opt.state;
  const common::NavState& x0 = pending_x0_[u.id % kMaxPendingKeyframes];
  // T_corr maps the front-end frame at the time of the keyframe to the backend's (ADR-033 4):
  // T_w_b(opt) = T_corr * T_w_b(x0); every front-end pose since then moves with it. The front-end
  // only integrates the velocity (scan matching gives a pose), so the backend's velocity error at
  // the keyframe, dv, is still in the front-end velocity now: it is removed too (to first order).
  const common::Pose T_corr = x.T_w_b * x0.T_w_b.inverse();
  const common::Vec3 dv = x.v_w_b - T_corr.R * x0.v_w_b;
  front_.T_w_b = T_corr * front_.T_w_b;
  front_.v_w_b = T_corr.R * front_.v_w_b + dv;
  front_.T_w_b.R = nearest_rotation(front_.T_w_b.R);
  front_.bias = x.bias;
  ref_ = T_corr * ref_;
  ref_.R = nearest_rotation(ref_.R);
  for (common::KeyId k = pending_first_kf_; k < next_kf_id_; ++k) {
    common::NavState& p = pending_x0_[k % kMaxPendingKeyframes];
    p.T_w_b = T_corr * p.T_w_b;
    p.T_w_b.R = nearest_rotation(p.T_w_b.R);
    p.v_w_b = T_corr.R * p.v_w_b + dv;
  }
  if (pred_last_t_ == front_.t) {
    reset_prediction();  // nothing integrated since the boundary sample: restart at the new bias
  }
  if (have_scan_heading_) {  // a correction is not a turn of the robot (design 2b)
    last_scan_heading_rad_ = travel_heading_rad(front_.T_w_b.R, cfg_.frames.T_b_o.R);
  }
  latest_.state = front_;
  ++counters_.n_backend_corrections;
}

void EstimatorCore::finish_backend() {
  if (!async_) {
    return;
  }
  // The thread is joined: every update it sent is queued, and its last snapshot is published.
  apply_backend_updates(std::numeric_limits<std::uint64_t>::max());
  backend_.set_defer_publish(false);
  async_ = false;
}

CoreCounters EstimatorCore::counters() const {
  CoreCounters c = counters_;
  // Threaded: the copy the backend thread published after its last message (ADR-033).
  BackendCounters b;
  if (async_) {
    const std::lock_guard<std::mutex> lock(chan_->counters_mutex);
    b = chan_->counters;
    c.n_backend_updates_dropped = chan_->n_updates_dropped;
  } else {
    b = backend_.counters();
    c.n_backend_updates_dropped = chan_ != nullptr ? chan_->n_updates_dropped : 0U;
  }
  c.n_backend_failures = b.n_backend_failures;
  c.n_gravity_factor_removed = b.n_gravity_factor_removed;
  c.n_gravity_factor_unchecked = b.n_gravity_factor_unchecked;
  c.n_loop_searches = b.n_loop_searches;
  c.n_loops_accepted = b.n_loops_accepted;
  c.n_loops_rejected = b.n_loops_rejected;
  c.n_loops_pcm_rejected = b.n_loops_pcm_rejected;
  c.n_loops_removed = b.n_loops_removed;
  c.n_loops_no_marginal = b.n_loops_no_marginal;
  c.n_loops_released = b.n_loops_released;
  c.n_gnss_factors = b.n_gnss_factors;
  c.n_gnss_rejected = b.n_gnss_rejected;
  c.n_gnss_realign = b.n_gnss_realign;
  c.n_gnss_wheel_mismatch = b.n_gnss_wheel_mismatch;
  c.n_gnss_frontend_wheel_disagree = b.n_gnss_frontend_wheel_disagree;
  c.n_gnss_prealign_attached = b.n_gnss_prealign_attached;
  c.n_gnss_prealign_static = b.n_gnss_prealign_static;
  c.n_gnss_prealign_dropped = b.n_gnss_prealign_dropped;
  c.n_gnss_prealign_not_static = b.n_gnss_prealign_not_static;
  c.n_gnss_prealign_rejected = b.n_gnss_prealign_rejected;
  c.n_map_rebuilds = b.n_map_rebuilds;
  c.n_map_spacing_replaced_cleared = b.n_map_spacing_replaced_cleared;
  c.n_map_publish_skipped = b.n_map_publish_skipped;
  c.n_map_full_copies = b.n_map_full_copies;
  return c;
}

}  // namespace mss::pipeline::detail
