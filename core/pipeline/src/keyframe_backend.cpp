#include "keyframe_backend.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>

#include "mss/common/lie.hpp"
#include "mss/common/logger.hpp"

#include "estimator_core.hpp"

namespace mss::pipeline::detail {

namespace {

constexpr double kPiRad = 3.14159265358979323846;  // pi, for degrees in the loop logs

const char* outcome_name(loop::LoopOutcome o) {
  switch (o) {
    case loop::LoopOutcome::kAccepted:
      return "accepted";
    case loop::LoopOutcome::kNoCandidate:
      return "no_candidate";
    case loop::LoopOutcome::kDescriptor:
      return "descriptor";
    case loop::LoopOutcome::kIcpFailed:
      return "icp_failed";
    case loop::LoopOutcome::kCorrection:
      return "correction";
    case loop::LoopOutcome::kInliers:
      return "inliers";
    case loop::LoopOutcome::kRms:
      return "rms";
    case loop::LoopOutcome::kDegenerate:
      return "degenerate";
    case loop::LoopOutcome::kAmbiguous:
      return "ambiguous";
    case loop::LoopOutcome::kNoMarginal:
      return "no_marginal";
  }
  return "unknown";
}

// One debug line per loop search (H6, C1-31: which stage stops each candidate). For a verified
// loop it adds the measurement T_bi_bj (translation, rotation vector) and the square roots of
// the diagonal of its floored covariance, for the NEES against ground truth (C1-3).
void log_loop_search(common::KeyId j, common::Timestamp t, const loop::LoopResult& r) {
  if (r.outcome == loop::LoopOutcome::kNoCandidate) {
    return;  // review M-13: nothing to diagnose, no formatting cost on most keyframes
  }
  char buf[512];
  int n = std::snprintf(buf,
                        sizeof(buf),
                        "loop search: j %llu t_ns %lld outcome %s n_cand %d cand %llu cand_t_ns "
                        "%lld sc %.3f ratio %.3f rms %.4f eig %.1f amb %.3f corr_m %.3f "
                        "corr_yaw_deg %.2f",
                        static_cast<unsigned long long>(j),
                        static_cast<long long>(t.ns),
                        outcome_name(r.outcome),
                        r.n_candidates,
                        static_cast<unsigned long long>(r.candidate),
                        static_cast<long long>(r.candidate_t.ns),
                        r.sc_distance,
                        r.inlier_ratio,
                        r.rms_m,
                        r.min_eig,
                        r.ambiguity_ratio,
                        r.correction_m,
                        r.correction_yaw_rad * 180.0 / kPiRad);
  if (r.outcome == loop::LoopOutcome::kAccepted && n > 0 &&
      static_cast<std::size_t>(n) < sizeof(buf)) {
    const common::Pose& Z = r.constraint.T_bi_bj;
    const common::Vec3 phi = common::so3_log(Z.R);
    const common::Vec6 sd = r.constraint.cov.diagonal().cwiseSqrt();
    (void)std::snprintf(buf + n,
                        sizeof(buf) - static_cast<std::size_t>(n),
                        " Z %.5f %.5f %.5f %.6f %.6f %.6f sd %.5f %.5f %.5f %.6f %.6f %.6f",
                        Z.p.x(),
                        Z.p.y(),
                        Z.p.z(),
                        phi.x(),
                        phi.y(),
                        phi.z(),
                        sd(0),
                        sd(1),
                        sd(2),
                        sd(3),
                        sd(4),
                        sd(5));
  }
  common::log_debug(buf);
}

// ADR-026 amendment / ADR-028: the graph's relative marginal of (i, j) for the optional loop
// correction gate (loop_closure.graph_covariance_gate). Lives for one try_loop_closure() call,
// on the estimator thread; keeps the reason of the last failure for the log (review M-5).
class GraphRelativeCovariance final : public loop::RelativeCovarianceSource {
 public:
  explicit GraphRelativeCovariance(const backend::Graph& graph) : graph_(graph) {}
  common::Result<common::PoseCovariance> relative_cov(common::KeyId i,
                                                      common::KeyId j) const override {
    common::Result<common::PoseCovariance> r = graph_.relative_pose_cov(i, j);
    if (!r.is_ok()) {
      last_error_ = r.reason();
    }
    return r;
  }
  const std::string& last_error() const { return last_error_; }

 private:
  const backend::Graph& graph_;
  mutable std::string last_error_;
};

}  // namespace

KeyframeBackend::KeyframeBackend(const Config& cfg, const CoreTestHooks* hooks)
    : cfg_(cfg),
      T_b_l_(cfg.frames.T_b_l),
      graph_(cfg.backend, cfg.imu, cfg.frames.p_b_g),
      store_(cfg.map.max_keyframes),
      loop_(cfg.loop_closure,
            cfg.scan_match,
            cfg.map,
            cfg.frames.T_b_l,
            cfg.realtime.max_points_per_scan),
      selector_(cfg.loop_closure, cfg.map.max_keyframes),
      fusion_(cfg.gnss, cfg.frames.p_b_g, cfg.frames.T_b_o),
      gnss_out_(std::make_unique<std::array<common::GnssMeasurement, kMaxAuxQueue>>()),
      early_out_(cfg.gnss.enabled && cfg.gnss.attach_prealign_fixes
                     ? std::make_unique<std::array<EarlyFactor, kMaxAuxQueue>>()
                     : nullptr),
      hooks_(hooks) {
  if (cfg.loop_closure.enabled) {
    loop_views_.reserve(cfg.map.max_keyframes);
    graph_loops_.reserve(2 * static_cast<std::size_t>(cfg.loop_closure.pcm_max_loops));
  }
  if (early_out_ != nullptr) {
    prealign_factors_.reserve(static_cast<std::size_t>(cfg.gnss.prealign_max_fixes));
  }
  if (hooks != nullptr) {  // test-only seams
    if (hooks->force_T_e_w) {
      fusion_.force_T_e_w(hooks->T_e_w);
    }
    if (hooks->gnss_shift_oracle) {
      fusion_.set_shift_oracle(&hooks->gnss_shift_oracle);
    }
    fusion_.set_first_alignment_yaw_offset(hooks->gnss_first_align_yaw_offset_rad);
  }
}

KeyframeBackend::~KeyframeBackend() = default;

CoreObserver* KeyframeBackend::observer() const {
  return hooks_ != nullptr ? hooks_->observer : nullptr;
}

void KeyframeBackend::log_backend_failure(const std::string& reason) {
  ++counters_.n_backend_failures;
  if (!backend_failure_streak_) {
    common::log_warn("backend optimize failed, using the front-end state: " + reason);
  }
  backend_failure_streak_ = true;
}

BackendUpdate KeyframeBackend::add_keyframe(const KeyframeMsg& msg) {
  BackendUpdate u;
  const common::KeyId j = graph_.add_keyframe(msg.t, msg.x0);
  u.id = j;
  n_known_keyframes_ = static_cast<std::size_t>(j) + 1;
  if (msg.first) {
    if (msg.add_static_gravity) {
      graph_.add_static_gravity_factor(j, msg.gravity_f_b, msg.gravity_cov);
      gravity_factor_added_ = true;
    }
  } else {
    graph_.add_imu_factor(msg.i, j, msg.imu_delta);
    if (cfg_.loop_closure.enabled && cfg_.loop_closure.pcm_enabled &&
        !selector_.add_odometry(j, msg.T_bi_bj, msg.lidar_cov)) {
      common::log_warn("loop selector: odometry chain not extended (keyframe out of order)");
    }
    graph_.add_lidar_factor(msg.i,
                            j,
                            msg.T_bi_bj,
                            msg.lidar_cov,
                            msg.has_degeneracy ? &msg.lidar_degeneracy_b : nullptr);
    if (msg.has_odom) {
      graph_.add_odom_factor(msg.i, j, msg.odom);
    }
  }
  // GNSS fixes waiting for this keyframe (pipeline.md section 5): the ADR-022 pre-alignment ones
  // first, then the queued ones, all with the same T_e_w.
  if (!msg.first && cfg_.gnss.enabled) {
    const std::size_t n = collect_gnss(j, msg.t, msg.x0.T_w_b);
    const common::Pose T_e_w = fusion_.T_e_w();
    for (const EarlyFactor& e : prealign_factors_) {
      graph_.add_gnss_factor(e.kf, e.m, T_e_w);
    }
    for (std::size_t k = 0; k < n; ++k) {
      graph_.add_gnss_factor(j, (*gnss_out_)[k], T_e_w);
    }
  }
  const common::Result<common::Estimate> opt = graph_.optimize();
  u.optimised = opt.is_ok();
  if (u.optimised) {
    if (!msg.first && observer() != nullptr) {
      observer()->on_keyframe_optimized(msg.t, opt.value().state, opt.value().cov);
      const common::Result<common::Pose> E = graph_.gnss_frame_estimate();
      if (E.is_ok()) {
        observer()->on_frame_estimate(msg.t, E.value());
      }
    }
    u.opt = opt.value();
    backend_failure_streak_ = false;
    if (!msg.first) {
      check_gravity_factor(j);
    }
  } else {
    log_backend_failure(opt.reason());
  }
  if (msg.cloud_l != nullptr) {
    (void)store_.add(j, msg.t, *msg.cloud_l);
  }
  if (msg.first) {
    if (early_out_ != nullptr) {
      // ADR-022: fixes of the IMU initialisation window are at rest at keyframe 0's pose, the
      // state the front-end starts from.
      fusion_.on_first_keyframe(j,
                                msg.t,
                                u.optimised ? u.opt.state.T_w_b : msg.x0.T_w_b,
                                u.optimised ? u.opt.cov : prior_pose_cov(cfg_.backend),
                                msg.prealign_static_from,
                                msg.prealign_static_ok);
      sync_prealign_counters();
    }
    return u;
  }
  // The reference pose of keyframe j (ADR-022: target of backward shifts) is the state the
  // front-end continues from (EstimatorCore::add_keyframe).
  const bool take_backend = hooks_ == nullptr || !hooks_->frontend_keeps_own_state;  // test seam
  if (early_out_ != nullptr) {
    fusion_.set_keyframe_reference(
        j, msg.t, u.optimised && take_backend ? u.opt.state.T_w_b : msg.x0.T_w_b);
  }
  if (u.optimised && cfg_.loop_closure.enabled) {
    u.loop_added = try_loop_closure(j, msg.t, u.loop_opt);
    if (u.loop_added && early_out_ != nullptr && take_backend) {
      fusion_.set_keyframe_reference(j, msg.t, u.loop_opt.state.T_w_b);
    }
  }
  return u;
}

std::size_t KeyframeBackend::collect_gnss(common::KeyId j,
                                          common::Timestamp t,
                                          const common::Pose& T_w_bj) {
  prealign_factors_.clear();
  if (early_out_ != nullptr) {
    // ADR-022: assigns the pre-alignment fixes up to keyframe j and, once aligned, takes them
    // for their keyframes, before the queued ones.
    fusion_.on_keyframe(j, t, T_w_bj);
    for (;;) {
      const std::size_t n = fusion_.take_prealign(*early_out_);
      if (n == 0) {
        break;
      }
      for (std::size_t k = 0; k < n; ++k) {
        const EarlyFactor& e = (*early_out_)[k];
        prealign_factors_.push_back(e);
        if (observer() != nullptr) {
          observer()->on_gnss_factor(e.t_k, e.m, fusion_.T_e_w());
        }
      }
      counters_.n_gnss_factors += n;
    }
    sync_prealign_counters();
  }
  const std::size_t n = fusion_.collect(t, T_w_bj, *gnss_out_);
  for (std::size_t k = 0; k < n; ++k) {
    if (observer() != nullptr) {
      observer()->on_gnss_factor(t, (*gnss_out_)[k], fusion_.T_e_w());
    }
  }
  counters_.n_gnss_factors += n;
  return n;
}

void KeyframeBackend::sync_prealign_counters() {
  counters_.n_gnss_prealign_attached = fusion_.n_prealign_attached();
  counters_.n_gnss_prealign_static = fusion_.n_prealign_static();
  counters_.n_gnss_prealign_dropped = fusion_.n_prealign_dropped();
  counters_.n_gnss_prealign_not_static = fusion_.n_prealign_not_static();
  counters_.n_gnss_prealign_rejected = fusion_.n_prealign_rejected();
}

void KeyframeBackend::on_gnss_fix(const GnssFixMsg& msg) {
  const common::GnssMeasurement& m = msg.m;
  if (msg.preinit) {
    if (early_out_ != nullptr) {  // ADR-022: kept for keyframe 0 (static window)
      if (!fusion_.on_preinit_fix(m).is_ok()) {
        ++counters_.n_gnss_rejected;  // gate_fix, as on the live path (literal, no log)
      }
      sync_prealign_counters();
    }
    return;
  }
  if (observer() != nullptr) {
    observer()->on_track_readings(m.t, msg.readings);
  }
  FixOutcome outcome = FixOutcome::kAccepted;
  (void)fusion_.on_fix(m, msg.T_w_b, msg.P_w_b, &outcome, msg.readings);
  if (fusion_.n_track_wheel_mismatches() != counters_.n_gnss_wheel_mismatch) {
    counters_.n_gnss_wheel_mismatch = fusion_.n_track_wheel_mismatches();
    if (!track_restart_logged_) {
      common::log_warn(
          "gnss: the GNSS track disagrees with the wheel travel (wheels turning on a still "
          "robot, slipping wheels, or a wrong odometer scale); see Stats::n_gnss_wheel_mismatch");
      track_restart_logged_ = true;
    }
  }
  if (fusion_.n_track_frontend_wheel_disagreements() != counters_.n_gnss_frontend_wheel_disagree) {
    counters_.n_gnss_frontend_wheel_disagree = fusion_.n_track_frontend_wheel_disagreements();
    if (!track_fe_disagree_logged_) {
      common::log_warn(
          "gnss: the front end holds the robot still while the wheels turn (wheels spinning on a "
          "still robot, or a LiDAR that sees surroundings moving with the robot); see "
          "Stats::n_gnss_frontend_wheel_disagree");
      track_fe_disagree_logged_ = true;
    }
  }
  if (early_out_ != nullptr) {
    sync_prealign_counters();
  }
  if (fusion_.n_realign() != counters_.n_gnss_realign) {
    counters_.n_gnss_realign = fusion_.n_realign();
    common::log_warn(
        "gnss: chi-square rejected gnss.max_consecutive_rejects fixes in a row, "
        "aligner restarted");
  }
  if (fusion_.aligned()) {
    track_restart_logged_ = false;
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

common::PoseCovariance prior_pose_cov(const backend::BackendConfig& b) {
  common::PoseCovariance c = common::PoseCovariance::Zero();
  const double var_p = b.prior_pos_sigma_m * b.prior_pos_sigma_m;
  const double var_r = b.prior_rot_sigma_rad * b.prior_rot_sigma_rad;
  for (int k = 0; k < 3; ++k) {
    c(k, k) = var_p;
    c(k + 3, k + 3) = var_r;
  }
  return c;
}

bool KeyframeBackend::try_loop_closure(common::KeyId j,
                                       common::Timestamp t,
                                       common::Estimate& loop_opt) {
  if (loop_skip_ > 0) {
    --loop_skip_;
    return false;
  }
  // Views of the stored keyframes at the optimised poses of the backend (no cloud copied).
  loop_views_.clear();
  for (std::size_t i = 0; i < store_.size(); ++i) {
    const KeyframeEntry& e = store_.at(i);
    const common::Result<common::NavState> s = graph_.keyframe_state(e.id);
    if (!s.is_ok()) {
      continue;
    }
    loop::KeyframeView v;
    v.id = e.id;
    v.t = e.t;
    v.T_w_b = s.value().T_w_b;
    v.cloud_l = &e.cloud_l;
    loop_views_.push_back(v);
  }
  if (loop_views_.empty() || loop_views_.back().id != j) {
    return false;  // stub backend, or keyframe j has no pose: nothing to search with
  }
  ++counters_.n_loop_searches;
  const GraphRelativeCovariance rel(graph_);
  loop::LoopResult r;
  if (hooks_ != nullptr && hooks_->loop_inject) {  // test-only seam
    r = hooks_->loop_inject(common::Span<const loop::KeyframeView>(loop_views_));
  } else {
    r = loop_.detect(loop_views_.back(), common::Span<const loop::KeyframeView>(loop_views_), &rel);
  }
  if (observer() != nullptr) {
    observer()->on_loop_search(t, r);
  }
  log_loop_search(j, t, r);
  if (r.outcome == loop::LoopOutcome::kNoMarginal) {
    ++counters_.n_loops_no_marginal;
    common::log_warn("loop closure: relative marginal failed, candidate rejected: " +
                     rel.last_error());
  }
  if (r.outcome == loop::LoopOutcome::kAmbiguous) {
    // Review H3 I-2: in repeated structure every search is ambiguous; pause the search.
    loop_skip_ = cfg_.loop_closure.ambiguity_backoff_keyframes;
  }
  if (r.outcome != loop::LoopOutcome::kAccepted) {
    counters_.n_loops_rejected += r.n_verified > 0 ? 1U : 0U;
    return false;
  }
  // ADR-028: PCM decides which verified loops are in the graph; it may also take out loops
  // committed earlier that the new one contradicts, or bring back loops it had left out.
  const auto keyframe_t = [this](common::KeyId id) {
    for (const loop::KeyframeView& v : loop_views_) {
      if (v.id == id) {
        return v.t;
      }
    }
    return common::Timestamp{};
  };
  bool selected = true;
  bool pcm_changed = false;
  if (cfg_.loop_closure.pcm_enabled) {
    const loop::LoopSelector::Decision d = selector_.propose(r.constraint);
    selected = d.selected;
    for (const auto& ij : selector_.to_remove()) {
      for (std::size_t a = graph_loops_.size(); a > 0; --a) {
        if (graph_loops_[a - 1].lc.i == ij.first && graph_loops_[a - 1].lc.j == ij.second) {
          graph_loops_.erase(graph_loops_.begin() + static_cast<std::ptrdiff_t>(a - 1));
          break;
        }
      }
      char buf[160];
      if (graph_.remove_loop_factor(ij.first, ij.second)) {
        ++counters_.n_loops_removed;
        removed_loops_.push_back(ij);  // graph_snapshot() only (ADR-031)
        std::snprintf(buf,
                      sizeof(buf),
                      "loop closure removed: j %llu i %llu (inconsistent with the selected loops)",
                      static_cast<unsigned long long>(ij.second),
                      static_cast<unsigned long long>(ij.first));
        common::log_info(buf);
      } else {
        std::snprintf(buf,
                      sizeof(buf),
                      "loop closure: unselected loop j %llu i %llu was not in the graph (dropped "
                      "with a failed update)",
                      static_cast<unsigned long long>(ij.second),
                      static_cast<unsigned long long>(ij.first));
        common::log_info(buf);
      }
    }
    for (const common::LoopConstraint& lc : selector_.to_add()) {
      PcmLoop g;
      g.lc = lc;
      g.ev = release_evidence(lc);
      graph_loops_.push_back(g);
    }
    const bool kernels_changed = apply_loop_kernels();
    if (!selected) {
      ++counters_.n_loops_pcm_rejected;
    }
    pcm_changed = !selector_.to_add().empty() || !selector_.to_remove().empty() || kernels_changed;
    if (!pcm_changed) {
      return false;  // nothing changed in the graph
    }
  } else {
    graph_.add_loop_factor(r.constraint);
    plain_loops_.emplace_back(r.constraint.i, r.constraint.j);  // graph_snapshot() only
  }
  const bool drop =  // test-only seam
      hooks_ != nullptr && hooks_->drop_loop_updates > 0 && hooks_->pass_loop_updates-- <= 0;
  common::Result<common::Estimate> opt = drop ? drop_pending_loops_for_test() : graph_.optimize();
  for (PcmLoop& g : graph_loops_) {
    counters_.n_loops_released += opt.is_ok() && g.pending && g.released ? 1U : 0U;
    g.pending = false;  // committed, or dropped with the failed update
  }
  if (!opt.is_ok()) {
    log_backend_failure(opt.reason());  // the pending loop factors were dropped with the update
    if (cfg_.loop_closure.pcm_enabled) {
      // Review H3 m-1: the selector forgets the loops that did not make it into the graph
      // (new ones, and with ADR-030 also re-added ones whose old factor was removed).
      for (std::size_t a = graph_loops_.size(); a > 0; --a) {
        const common::LoopConstraint& lc = graph_loops_[a - 1].lc;
        if (!graph_.has_loop_factor(lc.i, lc.j)) {
          (void)selector_.forget(lc.i, lc.j);
          graph_loops_.erase(graph_loops_.begin() + static_cast<std::ptrdiff_t>(a - 1));
        }
      }
    }
    return false;
  }
  // The loop factors are committed from here on: whatever the extra updates do, the loops
  // count, the front end re-syncs and the next keyframes skip the search (no duplicate loop).
  // A failed extra update leaves the graph at the estimate of the last good update
  // (graph.hpp), which `opt` holds.
  for (int k = 0; k < cfg_.loop_closure.isam_extra_updates; ++k) {
    const common::Result<common::Estimate> more =
        (hooks_ != nullptr && hooks_->fail_loop_extra_updates)  // test-only seam
            ? common::Result<common::Estimate>::error("test: injected extra-update failure")
            : graph_.optimize();
    if (!more.is_ok()) {
      log_backend_failure(more.reason());
      break;
    }
    opt = more;
  }
  // One line and one count per loop factor that entered the graph (review H3 m-4: with PCM a
  // loop may enter later than its detection), parsed by the E-07 evaluation: the two
  // keyframes with their times and the measured T_bi_bj (translation in b_i, yaw).
  const auto log_entered = [&](const common::LoopConstraint& lc) {
    ++counters_.n_loops_accepted;
    const common::Pose& Z = lc.T_bi_bj;
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "loop closure accepted: j %llu t_ns %lld i %llu t_ns %lld dp_m %.4f %.4f %.4f "
                  "dyaw_rad %.5f",
                  static_cast<unsigned long long>(lc.j),
                  static_cast<long long>(keyframe_t(lc.j).ns),
                  static_cast<unsigned long long>(lc.i),
                  static_cast<long long>(keyframe_t(lc.i).ns),
                  Z.p.x(),
                  Z.p.y(),
                  Z.p.z(),
                  std::atan2(Z.R(1, 0), Z.R(0, 0)));
    common::log_info(buf);
    // How much of the correction the graph took (H6, C1-13): the loop residual against the
    // graph poses before the loop (the search's views) and after the updates.
    const auto view_pose = [this](common::KeyId id) -> std::optional<common::Pose> {
      for (const loop::KeyframeView& v : loop_views_) {
        if (v.id == id) {
          return v.T_w_b;
        }
      }
      return std::nullopt;
    };
    const std::optional<common::Pose> bi = view_pose(lc.i);
    const std::optional<common::Pose> bj = view_pose(lc.j);
    const common::Result<common::NavState> ai = graph_.keyframe_state(lc.i);
    const common::Result<common::NavState> aj = graph_.keyframe_state(lc.j);
    if (bi && bj && ai.is_ok() && aj.is_ok()) {
      const common::Vec6 before = common::boxminus(Z, bi->inverse() * *bj);
      const common::Vec6 after = common::boxminus(Z, ai.value().T_w_b.inverse() * aj.value().T_w_b);
      std::snprintf(buf,
                    sizeof(buf),
                    "loop applied: j %llu i %llu residual_m %.4f -> %.4f residual_deg %.3f -> %.3f",
                    static_cast<unsigned long long>(lc.j),
                    static_cast<unsigned long long>(lc.i),
                    before.head<3>().norm(),
                    after.head<3>().norm(),
                    before.tail<3>().norm() * 180.0 / kPiRad,
                    after.tail<3>().norm() * 180.0 / kPiRad);
      common::log_debug(buf);
    }
  };
  if (cfg_.loop_closure.pcm_enabled) {
    for (const common::LoopConstraint& lc : selector_.to_add()) {
      log_entered(lc);
    }
  } else {
    log_entered(r.constraint);
  }
  loop_opt = opt.value();  // the front end re-syncs to it (EstimatorCore::add_keyframe)
  loop_skip_ = selected ? cfg_.loop_closure.min_interval_keyframes : 0;
  if (hooks_ != nullptr && hooks_->inspect_graph) {  // test-only seam
    hooks_->inspect_graph(graph_);
  }
  return true;
}

bool release_plausible(const ReleaseEvidence& a, const loop::LoopConfig& c) {
  return a.asked_m <= c.kernel_release_drift_ratio * a.path_m;
}

bool release_partners(const ReleaseEvidence& a,
                      const ReleaseEvidence& b,
                      const loop::LoopConfig& c) {
  const auto w = static_cast<common::KeyId>(c.kernel_release_window_keyframes);
  const auto near = [w](common::KeyId x, common::KeyId y) { return (x > y ? x - y : y - x) <= w; };
  return release_plausible(a, c) && release_plausible(b, c) && near(a.i, b.i) && near(a.j, b.j) &&
         (a.p_i - b.p_i).norm() >= c.kernel_release_min_separation_m &&
         (a.p_j - b.p_j).norm() >= c.kernel_release_min_separation_m;
}

ReleaseEvidence KeyframeBackend::release_evidence(const common::LoopConstraint& lc) const {
  ReleaseEvidence ev;
  ev.i = lc.i;
  ev.j = lc.j;
  const loop::KeyframeView* vi = nullptr;
  const loop::KeyframeView* vj = nullptr;
  const loop::KeyframeView* prev = nullptr;
  for (const loop::KeyframeView& v : loop_views_) {
    if (prev != nullptr && v.id > lc.i && v.id <= lc.j) {
      ev.path_m += (v.T_w_b.p - prev->T_w_b.p).norm();
    }
    vi = v.id == lc.i ? &v : vi;
    vj = v.id == lc.j ? &v : vj;
    prev = &v;
  }
  if (vi == nullptr || vj == nullptr) {
    ev.asked_m = std::numeric_limits<double>::infinity();  // never plausible
    return ev;
  }
  ev.p_i = vi->T_w_b.p;
  ev.p_j = vj->T_w_b.p;
  ev.asked_m = (lc.T_bi_bj.p - (vi->T_w_b.inverse() * vj->T_w_b).p).norm();
  return ev;
}

bool KeyframeBackend::apply_loop_kernels() {
  bool changed = false;
  for (std::size_t a = 0; a < graph_loops_.size(); ++a) {
    PcmLoop& g = graph_loops_[a];
    bool partner = false;
    for (std::size_t b = 0; b < graph_loops_.size() && cfg_.loop_closure.kernel_release; ++b) {
      partner =
          partner || (b != a && release_partners(g.ev, graph_loops_[b].ev, cfg_.loop_closure));
    }
    const backend::Graph::LoopKernel kernel =
        partner ? backend::Graph::LoopKernel::kNone : backend::Graph::LoopKernel::kCauchy;
    if (g.added && partner == g.released) {
      continue;
    }
    if (g.added) {
      // The old factor leaves in the same update that adds the new one (graph.hpp).
      (void)graph_.remove_loop_factor(g.lc.i, g.lc.j);
      changed = true;
      char buf[128];
      std::snprintf(buf,
                    sizeof(buf),
                    "loop closure kernel %s: j %llu i %llu",
                    partner ? "released" : "restored",
                    static_cast<unsigned long long>(g.lc.j),
                    static_cast<unsigned long long>(g.lc.i));
      common::log_info(buf);
    }
    graph_.add_loop_factor(g.lc, kernel);
    g.added = true;
    g.pending = true;
    g.released = partner;
  }
  return changed;
}

// Test-only (CoreTestHooks::drop_loop_updates): an update that fails and drops the pending loop
// factors, as a failed optimize() does (graph.hpp).
common::Result<common::Estimate> KeyframeBackend::drop_pending_loops_for_test() {
  --hooks_->drop_loop_updates;
  // Every loop factor pending in this update, new or re-added with another kernel (review M-3);
  // the removal of a re-added loop's old factor stands, as in a real failed update.
  for (const PcmLoop& g : graph_loops_) {
    if (g.pending) {
      (void)graph_.remove_loop_factor(g.lc.i, g.lc.j);
    }
  }
  if (!cfg_.loop_closure.pcm_enabled) {
    for (const common::LoopConstraint& lc : selector_.to_add()) {
      (void)graph_.remove_loop_factor(lc.i, lc.j);
    }
  }
  return common::Result<common::Estimate>::error("test: injected loop update failure");
}

void KeyframeBackend::check_gravity_factor(common::KeyId j) {
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

MapExportInfo KeyframeBackend::visit_map(const MapKeyframeVisitor& visit) const {
  MapExportInfo info;
  info.n_evicted = static_cast<std::size_t>(store_.n_evicted());
  for (std::size_t i = 0; i < store_.size(); ++i) {
    const KeyframeEntry& e = store_.at(i);
    const common::Result<common::NavState> s = graph_.keyframe_state(e.id);
    if (!s.is_ok()) {
      ++info.n_no_pose;
      continue;
    }
    MapKeyframe kf;
    kf.id = e.id;
    kf.t = e.t;
    kf.T_w_l = s.value().T_w_b * T_b_l_;
    kf.xyz_l = e.cloud_l.xyz();
    ++info.n_keyframes;
    info.n_points_in += e.cloud_l.size();
    visit(kf);
  }
  info.n_points_out = info.n_points_in;
  return info;
}

bool add_graph_loop(common::KeyId i,
                    common::KeyId j,
                    LoopState state,
                    std::vector<GraphLoop>& out) {
  for (const GraphLoop& l : out) {
    if (l.i == i && l.j == j) {
      return false;
    }
  }
  GraphLoop l;
  l.i = i;
  l.j = j;
  l.state = state;
  out.push_back(l);
  return true;
}

std::size_t KeyframeBackend::graph_snapshot(GraphSnapshot& out) const {
  out.keyframes.clear();
  out.loops.clear();
  const std::size_t n_known = n_known_keyframes_;
  for (std::size_t k = 0; k < n_known; ++k) {
    const common::Result<common::NavState> s = graph_.keyframe_state(static_cast<common::KeyId>(k));
    if (s.is_ok()) {
      GraphKeyframe g;
      g.id = static_cast<common::KeyId>(k);
      g.t = s.value().t;
      g.T_w_b = s.value().T_w_b;
      out.keyframes.push_back(g);
    }
  }
  // In the graph first: with PCM every loop in the graph is in graph_loops_ (a failed update erases
  // the ones it dropped); without PCM a failed update drops the pending factor, so ask the graph.
  // Then the loops PCM removed; one PCM brought back is already listed with its current state.
  for (const PcmLoop& g : graph_loops_) {
    (void)add_graph_loop(
        g.lc.i, g.lc.j, g.released ? LoopState::kReleased : LoopState::kActive, out.loops);
  }
  for (const auto& ij : plain_loops_) {
    if (graph_.has_loop_factor(ij.first, ij.second)) {
      (void)add_graph_loop(ij.first, ij.second, LoopState::kActive, out.loops);
    }
  }
  for (const auto& ij : removed_loops_) {
    (void)add_graph_loop(ij.first, ij.second, LoopState::kRemoved, out.loops);
  }
  for (GraphLoop& l : out.loops) {
    const common::Result<common::NavState> si = graph_.keyframe_state(l.i);
    const common::Result<common::NavState> sj = graph_.keyframe_state(l.j);
    l.t_i = si.is_ok() ? si.value().t : common::Timestamp{};
    l.t_j = sj.is_ok() ? sj.value().t : common::Timestamp{};
  }
  return n_known;
}

}  // namespace mss::pipeline::detail
