#include "mss/backend/graph.hpp"

#include <gtsam/config.h>

#ifndef GTSAM_POSE3_EXPMAP
#error "backend assumes Pose3::retract = T * Expmap (docs/derivations/backend_factors.md section 1)"
#endif

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

#include "mss/common/logger.hpp"

#include "constants.hpp"
#include "conversions.hpp"
#include "gnss_factor.hpp"
#include "imu_factor.hpp"
#include "odom_factor.hpp"
#include "static_gravity_factor.hpp"
#include "validate.hpp"

namespace mss::backend {

namespace {

using common::KeyId;
using common::Mat6;
using common::NavState;
using common::Vec3;
using Vec6 = Eigen::Matrix<double, 6, 1>;

using detail::kLoopCauchyK;  // constants.hpp
constexpr double kNsPerS = 1e9;
constexpr double kMsPerNs = 1e-6;
// IMU duration tolerance in sample periods (graph.hpp, add_imu_factor).
constexpr double kImuTolSamples = 1.5;

// Symmetric positive definite and finite (Cholesky succeeds on the symmetrised matrix).
template <int N>
bool is_spd(const Eigen::Matrix<double, N, N>& cov) {
  if (!cov.allFinite()) {
    return false;
  }
  const Eigen::Matrix<double, N, N> sym = 0.5 * (cov + cov.transpose());
  const Eigen::LLT<Eigen::Matrix<double, N, N>> llt(sym);
  return llt.info() == Eigen::Success;
}

template <int N>
Eigen::Matrix<double, N, N> symmetrised(const Eigen::Matrix<double, N, N>& cov) {
  const Eigen::Matrix<double, N, N> sym = 0.5 * (cov + cov.transpose());
  return sym;
}

bool pose_finite(const common::Pose& T) {
  return T.R.allFinite() && T.p.allFinite();
}

bool delta_finite(const common::PreintegratedDelta& d) {
  return d.dR.allFinite() && d.dv.allFinite() && d.dp.allFinite() && std::isfinite(d.dt_s) &&
         d.cov.allFinite() && d.J_dR_bg.allFinite() && d.J_dv_ba.allFinite() &&
         d.J_dv_bg.allFinite() && d.J_dp_ba.allFinite() && d.J_dp_bg.allFinite() &&
         d.bias_lin.ba.allFinite() && d.bias_lin.bg.allFinite();
}

std::string call_name(const char* call, KeyId i, KeyId j) {
  return std::string(call) + "(" + std::to_string(i) + ", " + std::to_string(j) + ")";
}

void reject(const std::string& call, const std::string& why) {
  common::log_warn("backend: " + call + " rejected: " + why);
}

bool values_finite(const gtsam::Values& values) {
  for (const gtsam::Key key : values.keys()) {
    const gtsam::Symbol s(key);
    if (s.chr() == 'x' || key == detail::frame_key()) {
      if (!values.at<gtsam::Pose3>(key).matrix().allFinite()) {
        return false;
      }
    } else if (s.chr() == 'v') {
      if (!values.at<gtsam::Vector3>(key).allFinite()) {
        return false;
      }
    } else if (s.chr() == 'b') {
      if (!values.at<gtsam::imuBias::ConstantBias>(key).vector().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

// ADR-018: process-lifetime TBB limit. The first single-thread Graph creates a
// tbb::global_control(max_allowed_parallelism, 1) that is never released, so TBB never
// spawns workers in a deterministic process (a released and re-acquired limit leaves woken
// workers spinning). A later Graph with single_thread == false stays limited (logged once).
std::atomic<bool> g_tbb_limited{false};
std::atomic<bool> g_tbb_limited_warned{false};

void apply_tbb_limit(bool single_thread) {
  if (!single_thread) {
    if (g_tbb_limited.load() && !g_tbb_limited_warned.exchange(true)) {
      common::log_warn(
          "backend: TBB already limited to 1 thread by an earlier deterministic Graph");
    }
    return;
  }
#ifdef GTSAM_USE_TBB
  // Function-local static: created once (thread-safe), intentionally leaked so it is never
  // released, not even during static destruction at exit.
  static const auto* const limit =
      new tbb::global_control(tbb::global_control::max_allowed_parallelism, 1);
  static_cast<void>(limit);
  if (!g_tbb_limited.exchange(true)) {
    common::log_info("backend: TBB limited to 1 thread (deterministic)");
  }
#else
  common::log_info("backend: single_thread has no effect, GTSAM is built without TBB");
#endif
}

}  // namespace

/// Owner of the iSAM2 instance and of every factor not yet given to it.
/// pending_priors_: keyframe-0 priors, kept when an optimize() fails; pending_factors_:
/// all other factors since the last successful optimize(), dropped when it fails;
/// committed_factors_: everything iSAM2 holds (to rebuild it after a failure).
struct Graph::Impl {
  BackendConfig cfg;
  imu::ImuConfig imu_cfg;
  Vec3 p_b_g = Vec3::Zero();
  Vec3 g_w = Vec3::Zero();
  gtsam::ISAM2Params params;
  std::unique_ptr<gtsam::ISAM2> isam;
  std::vector<NavState> guesses;          // index = KeyId; t = keyframe time
  std::vector<bool> imu_end_committed;    // index = KeyId: a committed IMU factor ends here
  std::vector<KeyId> imu_end_pending;     // keyframes where a pending IMU factor ends
  std::vector<bool> imu_start_committed;  // index = KeyId: a committed IMU factor starts here
  std::vector<KeyId> imu_start_pending;   // keyframes where a pending IMU factor starts
  // index = KeyId: the keyframe pose was not in iSAM2 when an optimize() failed, so the
  // factors linking it to the anchored graph may have been dropped (section 9).
  std::vector<bool> orphaned;
  gtsam::NonlinearFactorGraph pending_priors;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::NonlinearFactorGraph committed_factors;
  gtsam::Values last_estimate;  // as of the last successful optimize()
  // ADR-019 frame variable E = T_e_w (cfg.gnss_frame_variable): created with its prior by
  // the first add_gnss_factor, initial value T_e_w_guess.
  bool have_frame = false;
  common::Pose T_e_w_guess;
  std::size_t degenerate_keyframes = 0;

  [[nodiscard]] bool known(KeyId k) const { return k < guesses.size(); }

  [[nodiscard]] bool imu_starts_at(KeyId k) const {
    if (imu_start_committed[k]) {
      return true;
    }
    for (const KeyId e : imu_start_pending) {
      if (e == k) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool imu_ends_at(KeyId k) const {
    if (imu_end_committed[k]) {
      return true;
    }
    for (const KeyId e : imu_end_pending) {
      if (e == k) {
        return true;
      }
    }
    return false;
  }

  // Priors with the first-keyframe sigmas (section 2), centred on the guess of `k`.
  void push_velocity_bias_priors(KeyId k, gtsam::NonlinearFactorGraph& graph) const {
    const NavState& x = guesses[k];
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        detail::vel_key(k),
        gtsam::Vector3(x.v_w_b),
        gtsam::noiseModel::Isotropic::Sigma(3, cfg.prior_vel_sigma_mps));
    Vec6 bias_sigmas;
    bias_sigmas << Vec3::Constant(cfg.prior_bias_acc_sigma_mps2),
        Vec3::Constant(cfg.prior_bias_gyro_sigma_radps);  // [acc, gyro]
    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        detail::bias_key(k),
        detail::to_gtsam(x.bias),
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  }

  void push_keyframe0_priors() {
    const NavState& x = guesses[0];
    Vec6 pose_sigmas;
    pose_sigmas << Vec3::Constant(cfg.prior_rot_sigma_rad),
        Vec3::Constant(cfg.prior_pos_sigma_m);  // GTSAM order [phi, rho]
    pending_priors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        detail::pose_key(0),
        detail::to_gtsam(x.T_w_b),
        gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
    push_velocity_bias_priors(0, pending_priors);
  }

  // Initial value of `key` from the keyframe guesses (the frame variable: T_e_w_guess).
  void insert_guess(gtsam::Key key, gtsam::Values& values) const {
    if (key == detail::frame_key()) {
      values.insert(key, detail::to_gtsam(T_e_w_guess));
      return;
    }
    const gtsam::Symbol s(key);
    const NavState& x = guesses[static_cast<std::size_t>(s.index())];
    if (s.chr() == 'x') {
      values.insert(key, detail::to_gtsam(x.T_w_b));
    } else if (s.chr() == 'v') {
      values.insert(key, gtsam::Vector3(x.v_w_b));
    } else {
      values.insert(key, detail::to_gtsam(x.bias));
    }
  }

  // State of keyframe k from `values` where present, else from its guess.
  [[nodiscard]] NavState state_of(KeyId k, const gtsam::Values& values) const {
    NavState x = guesses[k];
    if (values.exists(detail::pose_key(k))) {
      x.T_w_b = detail::from_gtsam(values.at<gtsam::Pose3>(detail::pose_key(k)));
    }
    if (values.exists(detail::vel_key(k))) {
      x.v_w_b = values.at<gtsam::Vector3>(detail::vel_key(k));
    }
    if (values.exists(detail::bias_key(k))) {
      x.bias = detail::from_gtsam(values.at<gtsam::imuBias::ConstantBias>(detail::bias_key(k)));
    }
    return x;
  }

  // After a failed update the iSAM2 instance may be half-updated: rebuild it from the
  // committed factors at the last good estimate (section 9). If the rebuild itself throws,
  // restart from a fresh iSAM2: the last estimate becomes the keyframe guesses and every
  // committed factor is queued again (as pending priors) for the next optimize().
  void rebuild() {
    isam = std::make_unique<gtsam::ISAM2>(params);
    if (committed_factors.empty()) {
      return;
    }
    try {
      isam->update(committed_factors, last_estimate);
    } catch (const std::exception& e) {
      common::log_error(
          std::string("backend: rebuilding iSAM2 failed, all factors queued again: ") + e.what());
      isam = std::make_unique<gtsam::ISAM2>(params);
      for (KeyId k = 0; k < guesses.size(); ++k) {
        guesses[k] = state_of(k, last_estimate);
      }
      if (last_estimate.exists(detail::frame_key())) {
        T_e_w_guess = detail::from_gtsam(last_estimate.at<gtsam::Pose3>(detail::frame_key()));
      }
      // The IMU start/end flags stay set: the re-queued factors are still accepted IMU
      // factors (they never pass through add_imu_factor again).
      pending_priors.push_back(committed_factors);
      committed_factors = gtsam::NonlinearFactorGraph();
      last_estimate = gtsam::Values();
    }
  }

  // Pose re-anchor (section 9): a connected component of `graph` (by factor keys) that
  // holds no key already in iSAM2 and not X(0), and contains an orphaned keyframe, gets a
  // pose prior on its lowest pose key, centred on the guess, with the first-keyframe sigmas.
  void anchor_orphan_components(gtsam::NonlinearFactorGraph& graph,
                                const gtsam::Values& held) const {
    std::map<gtsam::Key, gtsam::Key> parent;
    const auto find = [&parent](gtsam::Key k) {
      while (parent[k] != k) {
        k = parent[k];
      }
      return k;
    };
    for (const gtsam::NonlinearFactor::shared_ptr& f : graph) {
      if (!f || f->keys().empty()) {
        continue;
      }
      for (const gtsam::Key key : f->keys()) {
        parent.emplace(key, key);
      }
      const gtsam::Key root = find(f->keys().front());
      for (const gtsam::Key key : f->keys()) {
        const gtsam::Key r = find(key);
        if (r != root) {
          parent[r] = root;
        }
      }
    }
    struct Component {
      bool anchored = false;
      bool has_orphan = false;
      bool has_pose = false;
      KeyId lowest_pose = 0;
    };
    std::map<gtsam::Key, Component> components;
    for (const auto& entry : parent) {
      const gtsam::Key key = entry.first;
      if (key == detail::frame_key()) {
        continue;  // ADR-019: E links components but neither anchors nor is a keyframe
      }
      Component& c = components[find(key)];
      const gtsam::Symbol sym(key);
      const auto k = static_cast<KeyId>(sym.index());
      c.anchored = c.anchored || held.exists(key) || key == detail::pose_key(0);
      c.has_orphan = c.has_orphan || orphaned[k];
      if (sym.chr() == 'x' && (!c.has_pose || k < c.lowest_pose)) {
        c.has_pose = true;
        c.lowest_pose = k;
      }
    }
    for (const auto& entry : components) {
      const Component& c = entry.second;
      if (c.anchored || !c.has_orphan || !c.has_pose) {
        continue;
      }
      Vec6 pose_sigmas;
      pose_sigmas << Vec3::Constant(cfg.prior_rot_sigma_rad),
          Vec3::Constant(cfg.prior_pos_sigma_m);  // GTSAM order [phi, rho]
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          detail::pose_key(c.lowest_pose),
          detail::to_gtsam(guesses[c.lowest_pose].T_w_b),
          gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
      common::log_warn("backend: optimize: keyframe " + std::to_string(c.lowest_pose) +
                       " re-anchored with a pose prior (disconnected after a failed optimize)");
    }
  }
};

Graph::Graph(const BackendConfig& cfg, const imu::ImuConfig& imu_cfg, const common::Vec3& p_b_g) {
  detail::validate(cfg, imu_cfg, p_b_g);
  impl_ = std::make_unique<Impl>();
  apply_tbb_limit(cfg.single_thread);  // before the ISAM2 is created (ADR-018)
  impl_->cfg = cfg;
  impl_->imu_cfg = imu_cfg;
  impl_->p_b_g = p_b_g;
  impl_->g_w = Vec3(0.0, 0.0, -imu_cfg.gravity_mps2);
  gtsam::ISAM2Params& params = impl_->params;
  params.relinearizeThreshold = cfg.relinearize_threshold;
  params.relinearizeSkip = cfg.relinearize_skip;
  params.factorization = gtsam::ISAM2Params::CHOLESKY;
  // Wildfire threshold 0: full back-substitution every update (deterministic, exact).
  params.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0);
  params.cacheLinearizedFactors = true;
  params.findUnusedFactorSlots = false;
  impl_->isam = std::make_unique<gtsam::ISAM2>(params);
}

Graph::~Graph() = default;
Graph::Graph(Graph&&) noexcept = default;
Graph& Graph::operator=(Graph&&) noexcept = default;

common::KeyId Graph::add_keyframe(common::Timestamp t, const common::NavState& x0) {
  Impl& m = *impl_;
  const KeyId k = m.guesses.size();
  if (x0.t != t) {
    common::log_warn("backend: add_keyframe " + std::to_string(k) + ": x0.t != t");
  }
  if (k > 0 && t <= m.guesses.back().t) {
    common::log_warn("backend: add_keyframe " + std::to_string(k) + ": time not increasing");
  }
  NavState guess = x0;
  guess.t = t;
  const NavState fallback = k > 0 ? m.guesses.back() : NavState{};
  const bool pose_ok = pose_finite(x0.T_w_b);
  const bool vel_ok = x0.v_w_b.allFinite();
  const bool bias_ok = x0.bias.ba.allFinite() && x0.bias.bg.allFinite();
  if (!pose_ok || !vel_ok || !bias_ok) {
    common::log_warn("backend: add_keyframe " + std::to_string(k) +
                     ": non-finite x0 replaced by the previous keyframe guess");
    if (!pose_ok) {
      guess.T_w_b = fallback.T_w_b;
    }
    if (!vel_ok) {
      guess.v_w_b = fallback.v_w_b;
    }
    if (!bias_ok) {
      guess.bias = fallback.bias;
    }
  }
  m.guesses.push_back(guess);
  m.imu_end_committed.push_back(false);
  m.imu_start_committed.push_back(false);
  m.orphaned.push_back(false);
  if (k == 0) {
    m.push_keyframe0_priors();
  }
  return k;
}

void Graph::add_imu_factor(common::KeyId i,
                           common::KeyId j,
                           const common::PreintegratedDelta& delta) {
  Impl& m = *impl_;
  const std::string call = call_name("add_imu_factor", i, j);
  if (!m.known(i) || !m.known(j)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (j != i + 1) {
    reject(call, "keyframes not consecutive");
    return;
  }
  if (m.imu_starts_at(i)) {
    reject(call, "duplicate IMU factor");
    return;
  }
  if (!delta_finite(delta)) {
    reject(call, "non-finite delta");
    return;
  }
  if (!(delta.dt_s > 0.0)) {
    reject(call, "dt_s <= 0");
    return;
  }
  if (!is_spd<9>(delta.cov)) {
    reject(call, "covariance not positive definite");
    return;
  }
  const std::int64_t span_ns = m.guesses[j].t.ns - m.guesses[i].t.ns;
  const std::int64_t delta_ns = std::llround(delta.dt_s * kNsPerS);
  const double mismatch_ns = std::fabs(static_cast<double>(delta_ns - span_ns));
  const double tol_ns = kImuTolSamples * kNsPerS / m.imu_cfg.rate_hz;
  if (mismatch_ns > tol_ns) {
    std::array<char, 96> why{};
    std::snprintf(why.data(),
                  why.size(),
                  "duration mismatch %.3f ms > %.3f ms",
                  mismatch_ns * kMsPerNs,
                  tol_ns * kMsPerNs);
    reject(call, why.data());
    return;
  }

  common::PreintegratedDelta d = delta;
  d.cov = symmetrised<9>(delta.cov);
  m.pending_factors.emplace_shared<detail::ImuPreintFactor>(detail::pose_key(i),
                                                            detail::vel_key(i),
                                                            detail::pose_key(j),
                                                            detail::vel_key(j),
                                                            detail::bias_key(i),
                                                            d,
                                                            m.g_w);
  const double sqrt_dt = std::sqrt(delta.dt_s);
  Vec6 rw_sigmas;
  rw_sigmas << Vec3::Constant(m.imu_cfg.acc_random_walk * sqrt_dt),
      Vec3::Constant(m.imu_cfg.gyro_random_walk * sqrt_dt);  // [acc, gyro]
  m.pending_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
      detail::bias_key(i),
      detail::bias_key(j),
      gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(rw_sigmas));
  // IMU-gap re-anchor (section 9).
  if (i > 0 && !m.imu_ends_at(i)) {
    m.push_velocity_bias_priors(i, m.pending_factors);
  }
  m.imu_end_pending.push_back(j);
  m.imu_start_pending.push_back(i);
}

void Graph::add_lidar_factor(common::KeyId i,
                             common::KeyId j,
                             const common::Pose& T_bi_bj,
                             const common::PoseCovariance& cov,
                             const common::DegeneracyReport* degeneracy) {
  Impl& m = *impl_;
  const std::string call = call_name("add_lidar_factor", i, j);
  if (!m.known(i) || !m.known(j)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (i >= j) {
    reject(call, "i >= j");
    return;
  }
  if (!pose_finite(T_bi_bj)) {
    reject(call, "non-finite pose");
    return;
  }
  if (!is_spd<6>(cov)) {
    reject(call, "covariance not positive definite");
    return;
  }
  m.pending_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      detail::pose_key(i),
      detail::pose_key(j),
      detail::to_gtsam(T_bi_bj),
      gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(symmetrised<6>(cov))));
  if (degeneracy != nullptr) {
    bool any = false;
    for (const bool flag : degeneracy->degenerate) {
      any = any || flag;
    }
    if (any) {
      ++m.degenerate_keyframes;
      common::log_debug("backend: " + call + " degenerate (" +
                        std::to_string(m.degenerate_keyframes) + " degenerate keyframes)");
    }
  }
}

void Graph::add_gnss_factor(common::KeyId k,
                            const common::GnssMeasurement& gm,
                            const common::Pose& T_e_w) {
  Impl& m = *impl_;
  const std::string call = "add_gnss_factor(" + std::to_string(k) + ")";
  if (!m.known(k)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (!gm.p_e_g.allFinite() || !pose_finite(T_e_w)) {
    reject(call, "non-finite input");
    return;
  }
  if (!is_spd<3>(gm.cov)) {
    reject(call, "covariance not positive definite");
    return;
  }
  if (m.cfg.gnss_frame_variable) {
    if (!m.have_frame) {
      // One prior on E (ADR-019, backend_factors.md section 6b), GTSAM order [phi, rho]:
      // roll/pitch tight (E is 4-DoF), yaw and translation loose around the first guess.
      Vec6 sigmas;
      sigmas << m.cfg.gnss_frame_tilt_sigma_rad, m.cfg.gnss_frame_tilt_sigma_rad,
          m.cfg.gnss_frame_yaw_sigma_rad, Vec3::Constant(m.cfg.gnss_frame_pos_sigma_m);
      m.pending_priors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          detail::frame_key(),
          detail::to_gtsam(T_e_w),
          gtsam::noiseModel::Diagonal::Sigmas(sigmas));
      m.T_e_w_guess = T_e_w;
      m.have_frame = true;
    }
    m.pending_factors.emplace_shared<detail::GnssFrameFactor>(
        detail::pose_key(k), detail::frame_key(), gm.p_e_g, symmetrised<3>(gm.cov), m.p_b_g);
    return;
  }
  m.pending_factors.emplace_shared<detail::GnssPositionFactor>(
      detail::pose_key(k), gm.p_e_g, symmetrised<3>(gm.cov), T_e_w, m.p_b_g);
}

void Graph::add_odom_factor(common::KeyId i, common::KeyId j, const common::OdomDelta& odom) {
  Impl& m = *impl_;
  const std::string call = call_name("add_odom_factor", i, j);
  if (!m.known(i) || !m.known(j)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (i >= j) {
    reject(call, "i >= j");
    return;
  }
  std::optional<detail::OdomBetweenFactor> f =
      detail::OdomBetweenFactor::create(detail::pose_key(i), detail::pose_key(j), odom);
  if (!f) {
    reject(call, "non-finite input or zero / indefinite covariance");
    return;
  }
  m.pending_factors.push_back(f->clone());
}

void Graph::add_static_gravity_factor(common::KeyId k,
                                      const common::Vec3& f_b,
                                      const common::Mat3& cov) {
  Impl& m = *impl_;
  const std::string call = "add_static_gravity_factor(" + std::to_string(k) + ")";
  if (!m.known(k)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (!f_b.allFinite() || !(f_b.norm() > 0.0)) {
    reject(call, "non-finite or zero specific force");
    return;
  }
  if (!is_spd<3>(cov)) {
    reject(call, "covariance not positive definite");
    return;
  }
  // With the keyframe-0 priors: kept when an optimize() fails (section 6c).
  m.pending_priors.emplace_shared<detail::StaticGravityFactor>(
      detail::pose_key(k),
      detail::bias_key(k),
      f_b,
      symmetrised<3>(cov) * (m.cfg.static_gravity_sigma_scale * m.cfg.static_gravity_sigma_scale),
      m.g_w);
}

void Graph::add_loop_factor(const common::LoopConstraint& lc) {
  Impl& m = *impl_;
  const std::string call = call_name("add_loop_factor", lc.i, lc.j);
  if (!m.known(lc.i) || !m.known(lc.j)) {
    reject(call, "unknown keyframe");
    return;
  }
  if (lc.i >= lc.j) {
    reject(call, "i >= j");
    return;
  }
  if (!pose_finite(lc.T_bi_bj)) {
    reject(call, "non-finite pose");
    return;
  }
  if (!is_spd<6>(lc.cov)) {
    reject(call, "covariance not positive definite");
    return;
  }
  const gtsam::SharedNoiseModel noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(kLoopCauchyK),
      gtsam::noiseModel::Gaussian::Covariance(detail::permute_cov(symmetrised<6>(lc.cov))));
  m.pending_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      detail::pose_key(lc.i), detail::pose_key(lc.j), detail::to_gtsam(lc.T_bi_bj), noise);
}

common::Result<common::Estimate> Graph::optimize() {
  Impl& m = *impl_;
  if (m.guesses.empty()) {
    return common::Result<common::Estimate>::error("empty graph");
  }
  gtsam::NonlinearFactorGraph graph = m.pending_priors;
  graph.push_back(m.pending_factors);
  gtsam::Values new_values;
  const gtsam::Values& held = m.isam->getLinearizationPoint();
  m.anchor_orphan_components(graph, held);
  for (const gtsam::Key key : graph.keys()) {
    if (!held.exists(key)) {
      m.insert_guess(key, new_values);
    }
  }

  std::string failure;
  gtsam::Values estimate;
  if (!graph.empty() || !held.empty()) {
    try {
      m.isam->update(graph, new_values);
      estimate = m.isam->calculateEstimate();
      if (!values_finite(estimate)) {
        failure = "non-finite estimate";
      }
    } catch (const std::exception& e) {
      failure = e.what();
    }
  }
  if (!failure.empty()) {
    m.pending_factors = gtsam::NonlinearFactorGraph();
    m.imu_end_pending.clear();
    m.imu_start_pending.clear();
    m.rebuild();
    const gtsam::Values& kept = m.isam->getLinearizationPoint();
    for (KeyId k = 0; k < m.guesses.size(); ++k) {
      if (!kept.exists(detail::pose_key(k))) {
        m.orphaned[k] = true;
      }
    }
    return common::Result<common::Estimate>::error("optimization failed: " + failure);
  }

  m.committed_factors.push_back(graph);
  m.pending_priors = gtsam::NonlinearFactorGraph();
  m.pending_factors = gtsam::NonlinearFactorGraph();
  for (const KeyId e : m.imu_end_pending) {
    m.imu_end_committed[e] = true;
  }
  m.imu_end_pending.clear();
  for (const KeyId st : m.imu_start_pending) {
    m.imu_start_committed[st] = true;
  }
  m.imu_start_pending.clear();
  for (KeyId k = 0; k < m.guesses.size(); ++k) {
    if (estimate.exists(detail::pose_key(k))) {
      m.orphaned[k] = false;
    }
  }
  m.last_estimate = estimate;

  const KeyId latest = m.guesses.size() - 1;
  common::Estimate est;
  est.state = m.state_of(latest, m.last_estimate);
  est.cov = marginal_cov(latest);
  est.keyframe_id = latest;
  return common::Result<common::Estimate>::ok(est);
}

common::PoseCovariance Graph::marginal_cov(common::KeyId k) const {
  const Impl& m = *impl_;
  common::PoseCovariance nan =
      common::PoseCovariance::Constant(std::numeric_limits<double>::quiet_NaN());
  if (!m.known(k) || !m.last_estimate.exists(detail::pose_key(k))) {
    return nan;
  }
  try {
    const gtsam::Matrix cov_g = m.isam->marginalCovariance(detail::pose_key(k));
    const Mat6 cov_g6 = cov_g;
    return detail::permute_cov(cov_g6);
  } catch (const std::exception& e) {
    common::log_warn(std::string("backend: marginal_cov failed: ") + e.what());
    return nan;
  }
}

common::Result<common::NavState> Graph::keyframe_state(common::KeyId k) const {
  const Impl& m = *impl_;
  if (!m.known(k)) {
    return common::Result<common::NavState>::error("unknown keyframe");
  }
  if (!m.last_estimate.exists(detail::pose_key(k))) {
    return common::Result<common::NavState>::error("keyframe not optimised");
  }
  return common::Result<common::NavState>::ok(m.state_of(k, m.last_estimate));
}

common::Result<common::Pose> Graph::gnss_frame_estimate() const {
  const Impl& m = *impl_;
  if (!m.last_estimate.exists(detail::frame_key())) {
    return common::Result<common::Pose>::error("no frame variable");
  }
  return common::Result<common::Pose>::ok(
      detail::from_gtsam(m.last_estimate.at<gtsam::Pose3>(detail::frame_key())));
}

}  // namespace mss::backend
