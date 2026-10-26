#include "estimator_core.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "mss/common/lie.hpp"
#include "mss/common/logger.hpp"
#include "mss/common/span.hpp"
#include "mss/degeneracy/degeneracy.hpp"

#include "frames.hpp"
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

}  // namespace

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
      pre_out_(cfg.realtime.max_points_per_scan),
      kf_cloud_(cfg.realtime.max_points_per_scan) {}

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
  if (!imu_.empty() && s.t <= imu_.back().t) {
    return;
  }
  if (imu_.full() && initialised_) {
    // The evicted sample precedes every future scan start: integrate it first.
    feed_preintegrators(imu_.front());
  }
  (void)imu_.push(s);
}

void EstimatorCore::on_wheel(const common::WheelSample& s) {
  if (have_wheel_) {
    if (s.t <= last_wheel_t_) {
      return;
    }
    wheel_period_ns_ = s.t.ns - last_wheel_t_.ns;
  }
  have_wheel_ = true;
  last_wheel_t_ = s.t;
  if (wheel_queue_.size() == wheel_queue_.capacity()) {
    common::WheelSample oldest;
    wheel_queue_.pop(oldest);
    (void)wheel_.integrate(oldest);
  }
  wheel_queue_.push(s);
}

void EstimatorCore::integrate_wheel(common::Timestamp t) {
  while (!wheel_queue_.empty() && wheel_queue_.front().t <= t) {
    common::WheelSample s;
    wheel_queue_.pop(s);
    (void)wheel_.integrate(s);
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
  if (!cfg_.gnss.enabled || !has_estimate_) {
    return;
  }
  // Body pose at m.t: the prediction advanced through the buffered samples up to m.t.
  scratch_ = pred_;
  common::Timestamp last = pred_last_t_;
  for (std::size_t i = imu_.first_after(last); i < imu_.size() && imu_.at(i).t <= m.t; ++i) {
    (void)scratch_.integrate(imu_.at(i));
    last = imu_.at(i).t;
  }
  if (last < m.t) {
    const std::optional<common::ImuSample> s = imu_.interpolate(m.t);
    if (s) {
      (void)scratch_.integrate(*s);
    }
  }
  const common::NavState x = scratch_.predict(front_, g_w_);
  const common::Status st = fusion_.on_fix(m, x.T_w_b, latest_.cov);
  if (!st.is_ok()) {
    const std::string reason = st.reason();
    if (reason != "gnss not aligned yet") {
      ++counters_.n_gnss_rejected;
      common::log_info("gnss fix rejected: " + reason);
    }
  }
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
  // No IMU up to the scan end: only possible at flush() (Review Focus 4). Nothing is
  // integrated, so the state stays at the previous scan.
  const std::optional<common::ImuSample> s_t = imu_.interpolate(t);
  if (!s_t || imu_.back().t < t_end) {
    ++counters_.n_skipped_scans;
    ++counters_.n_uncovered_scans;
    common::log_warn("scan not covered by imu samples, skipped");
    return;
  }
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
    g_w_ = init.value().g_w;
    pred_.reset(front_.bias);
    kf_.reset(front_.bias);
    (void)pred_.integrate(boundary_);
    (void)kf_.integrate(boundary_);
    pred_last_t_ = t;
    integrate_wheel(t);
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

  // 8. Keyframe.
  if (is_new_keyframe(ref_, front_.T_w_b, cfg_.backend)) {
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
  const common::Pose T_bi_bj = ref_.inverse() * front_.T_w_b;
  if (cfg_.degeneracy.enabled) {
    const common::DegeneracyReport report_b =
        degeneracy_in_body(degeneracy::analyze(match.H, cfg_.degeneracy), T_b_l_);
    graph_.add_lidar_factor(i, j, T_bi_bj, cov_b, &report_b);
  } else {
    graph_.add_lidar_factor(i, j, T_bi_bj, cov_b, nullptr);
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
    const std::size_t n = fusion_.collect(t, front_.T_w_b, *gnss_out_);
    for (std::size_t k = 0; k < n; ++k) {
      graph_.add_gnss_factor(j, (*gnss_out_)[k], fusion_.T_e_w());
    }
    counters_.n_gnss_factors += n;
  }
  const common::Result<common::Estimate> opt = graph_.optimize();
  const bool optimised = opt.is_ok();
  if (optimised) {
    front_ = opt.value().state;
    front_.t = t;
    cov_out = opt.value().cov;
    backend_failure_streak_ = false;
  } else {
    log_backend_failure(opt.reason());
  }
  kf_.reset(front_.bias);
  (void)kf_.integrate(boundary_);
  ref_ = front_.T_w_b;
  last_kf_id_ = j;
  last_kf_t_ = t;
  ++counters_.n_keyframes;
  insert_keyframe_cloud(j, t);
  if (optimised) {
    maybe_rebuild_map();
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
