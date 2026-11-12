#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "mss/backend/graph.hpp"
#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/loop/loop_detector.hpp"
#include "mss/loop/loop_selector.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/graph_snapshot.hpp"
#include "mss/pipeline/map_export.hpp"

#include "gnss_fusion.hpp"
#include "keyframe_store.hpp"

namespace mss::pipeline::detail {

class CoreObserver;
struct CoreTestHooks;

/// ADR-030 kernel-release rule (review B-1), on what the pipeline knows of a loop when it enters
/// the graph: its keyframes, their positions on the graph at that time (m, w), the correction it
/// asks of the graph (|p(Z) - p(T_w_bi^-1 T_w_bj)|, m) and the distance travelled from i to j
/// (sum of the keyframe steps, m).
struct ReleaseEvidence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::KeyId i = 0;
  common::KeyId j = 0;
  common::Vec3 p_i = common::Vec3::Zero();
  common::Vec3 p_j = common::Vec3::Zero();
  double asked_m = 0.0;
  double path_m = 0.0;
};
/// The correction is plausible as drift: asked_m <= kernel_release_drift_ratio * path_m.
bool release_plausible(const ReleaseEvidence& a, const loop::LoopConfig& c);
/// a and b are partners: both plausible, i and j each within kernel_release_window_keyframes,
/// and candidates and queries each at least kernel_release_min_separation_m apart (two places:
/// one look-alike seen from nearby queries gives loops whose candidates are close together).
/// Tests: LoopKernelRelease.PartnerRule*.
bool release_partners(const ReleaseEvidence& a,
                      const ReleaseEvidence& b,
                      const loop::LoopConfig& c);

/// ADR-031 loop listing of KeyframeBackend::graph_snapshot(): appends loop (i, j) with `state` to
/// `out` unless (i, j) is already listed; returns whether it was appended. The snapshot lists the
/// loops in the graph first, then the PCM-removed ones, so a loop that was removed and later
/// re-added appears once, with its current state. Times are not set here.
/// Tests: PipelineInternals.GraphLoopListedOnceWithItsCurrentState.
bool add_graph_loop(common::KeyId i, common::KeyId j, LoopState state, std::vector<GraphLoop>& out);

/// One keyframe from the front-end to the backend (ADR-033): the new keyframe's initial guess and
/// every factor that ends at it, in the order the backend adds them to the graph. The spans and
/// `cloud_l` point into front-end buffers that stay valid for the KeyframeBackend::add_keyframe
/// call.
struct KeyframeMsg {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Timestamp t{};
  common::NavState x0;  ///< initial guess of the new keyframe (x0.t == t)
  /// Keyframe 0: priors only (no IMU, LiDAR, wheel or GNSS factor), optional gravity factor.
  bool first = false;
  /// Keyframe 0, ADR-020: the static-gravity factor (mean specific force in b, covariance).
  bool add_static_gravity = false;
  common::Vec3 gravity_f_b = common::Vec3::Zero();
  common::Mat3 gravity_cov = common::Mat3::Zero();
  /// Keyframe j > 0: the previous keyframe i and the IMU, LiDAR (pipeline.md section 4) and wheel
  /// factors i -> j.
  common::KeyId i = 0;
  common::PreintegratedDelta imu_delta;
  common::Pose T_bi_bj;
  common::PoseCovariance lidar_cov = common::PoseCovariance::Zero();
  bool has_degeneracy = false;  ///< degeneracy.enabled: lidar_degeneracy_b is set
  common::DegeneracyReport lidar_degeneracy_b;
  bool has_odom = false;  ///< wheel_odom.enabled and the delta passed the time check
  common::OdomDelta odom;
  /// GNSS factors with `T_e_w`: the ADR-022 pre-alignment fixes on their keyframes first, then
  /// the fixes queued for this keyframe (pipeline.md section 5).
  common::Span<const EarlyFactor> prealign_gnss;
  common::Span<const common::GnssMeasurement> gnss;
  common::Pose T_e_w;
  /// The keyframe cloud in l for the store (ADR-016) and the loop search.
  const common::ScanFrame* cloud_l = nullptr;
};

/// What the backend returns for one keyframe (ADR-033): the id the graph gave it, the estimate
/// after its optimize() and, when an accepted loop changed the graph (ADR-026), the estimate
/// after the loop updates. The front-end applies them in that order.
struct BackendUpdate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::KeyId id = 0;
  bool optimised = false;  ///< optimize() after the keyframe succeeded: `opt` is set
  common::Estimate opt;
  bool loop_added = false;  ///< an accepted loop changed the graph: `loop_opt` is set
  common::Estimate loop_opt;
};

/// Counters of KeyframeBackend, all since construction (merged into CoreCounters).
struct BackendCounters {
  std::uint64_t n_backend_failures = 0;
  std::uint64_t n_gravity_factor_removed = 0;
  std::uint64_t n_gravity_factor_unchecked = 0;
  std::uint64_t n_loop_searches = 0;
  std::uint64_t n_loops_accepted = 0;
  std::uint64_t n_loops_rejected = 0;
  std::uint64_t n_loops_pcm_rejected = 0;
  std::uint64_t n_loops_removed = 0;
  std::uint64_t n_loops_no_marginal = 0;
  std::uint64_t n_loops_released = 0;
};

/// The keyframe backend of the estimator (ADR-033 step M1a): factor graph, keyframe store, loop
/// detection and PCM. The front-end (EstimatorCore) talks to it only through KeyframeMsg in,
/// BackendUpdate out and the read queries below. Internal to the pipeline. Single-threaded:
/// called on the estimator thread.
class KeyframeBackend {
 public:
  /// Builds the graph, store and loop modules from the validated `cfg`. `hooks` (nullptr in
  /// production) must outlive the backend. Throws as the module constructors.
  KeyframeBackend(const Config& cfg, const CoreTestHooks* hooks);
  ~KeyframeBackend();
  KeyframeBackend(const KeyframeBackend&) = delete;
  KeyframeBackend& operator=(const KeyframeBackend&) = delete;

  /// Adds the keyframe and its factors, optimises, runs the ADR-020 gravity check, stores the
  /// cloud and (loop_closure.enabled, optimised, keyframe j > 0) runs the loop search.
  BackendUpdate add_keyframe(const KeyframeMsg& msg);

  /// Optimised state of keyframe `k` (backend::Graph::keyframe_state).
  [[nodiscard]] common::Result<common::NavState> keyframe_state(common::KeyId k) const {
    return graph_.keyframe_state(k);
  }
  /// Stored cloud of keyframe `k` in l, or nullptr when evicted or never added.
  [[nodiscard]] const common::ScanFrame* keyframe_cloud(common::KeyId k) const {
    const KeyframeEntry* e = store_.find(k);
    return e != nullptr ? &e->cloud_l : nullptr;
  }
  [[nodiscard]] const BackendCounters& counters() const { return counters_; }
  /// Global map export (Estimator::visit_map): calls `visit` for every keyframe in the store,
  /// oldest first, at T_w_b(k) * T_b_l with T_w_b(k) = graph_.keyframe_state(k) now; entries
  /// without a backend estimate are skipped and counted (n_no_pose). Reads only; allocates
  /// nothing itself.
  MapExportInfo visit_map(const MapKeyframeVisitor& visit) const;
  /// Graph snapshot (Estimator::graph_snapshot, ADR-031): clears `out`, then lists every
  /// keyframe graph_.keyframe_state() has an estimate for and every loop that entered the graph
  /// (graph_loops_ with PCM, plain_loops_ without; then removed_loops_). Returns the number of
  /// keyframes the graph knows (estimated or not). Reads only; allocates only when `out` grows.
  std::size_t graph_snapshot(GraphSnapshot& out) const;

 private:
  // ADR-026: loop search for keyframe j (at t, already optimised and in the store). On an
  // accepted loop: loop factor, optimize() (+ loop_closure.isam_extra_updates), the estimate in
  // `loop_opt`; returns true.
  bool try_loop_closure(common::KeyId j, common::Timestamp t, common::Estimate& loop_opt);
  common::Result<common::Estimate> drop_pending_loops_for_test();
  // ADR-030 (with PCM): adds the loops of graph_loops_ not yet in the graph and re-adds those
  // whose kernel state changed (loop_closure.kernel_release: no kernel when a partner loop is
  // within the window). Returns true when a loop already in the graph changed its kernel.
  bool apply_loop_kernels();
  // ADR-030: the release evidence of a loop entering the graph, from loop_views_ (the graph
  // poses of this search).
  ReleaseEvidence release_evidence(const common::LoopConstraint& lc) const;
  // ADR-020 consistency check of the gravity factor after keyframe `j` was optimised.
  void check_gravity_factor(common::KeyId j);
  void log_backend_failure(const std::string& reason);

  Config cfg_;
  common::Pose T_b_l_;
  backend::Graph graph_;
  KeyframeStore store_;
  loop::LoopDetector loop_;
  loop::LoopSelector selector_;  // ADR-028: PCM over the verified loops (pcm_enabled)
  std::vector<loop::KeyframeView> loop_views_;  // reserved map.max_keyframes when enabled
  // ADR-030: the loop factors PCM put in the graph (selected, or frozen when they left PCM) and
  // whether each one is in the graph without the kernel. Owner: this class, estimator thread.
  struct PcmLoop {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::LoopConstraint lc;
    ReleaseEvidence ev;
    bool added = false;     // add_loop_factor called (it may still be pending)
    bool released = false;  // added with LoopKernel::kNone
    bool pending = false;   // (re-)added in the update being prepared
  };
  // Reserved 2 * pcm_max_loops when enabled; it can grow past that (frozen loops stay in the
  // graph), one allocation per growth, per accepted loop, off the hot path (review M-17). The
  // partner scan is O(n^2) per graph change.
  std::vector<PcmLoop> graph_loops_;
  // ADR-031 bookkeeping for graph_snapshot() only (never read by the estimation): the loops added
  // without PCM, in order (shown while Graph::has_loop_factor holds them), and the loops PCM
  // removed from the graph, in removal order. One allocation per growth, per accepted or removed
  // loop, off the hot path. Owner: this class, estimator thread.
  std::vector<std::pair<common::KeyId, common::KeyId>> plain_loops_;
  std::vector<std::pair<common::KeyId, common::KeyId>> removed_loops_;
  int loop_skip_ = 0;  // keyframes left to skip after an accepted loop (min_interval_keyframes)
  // KeyIds are dense from 0 (backend::Graph::add_keyframe): the newest id + 1, 0 before keyframe 0.
  std::size_t n_known_keyframes_ = 0;
  // Gravity-factor state (ADR-020): added at keyframe 0; checked once.
  bool gravity_factor_added_ = false;
  bool gravity_factor_checked_ = false;
  bool backend_failure_streak_ = false;
  BackendCounters counters_;
  // Test-only seam (CoreTestHooks); nullptr in production.
  const CoreTestHooks* hooks_ = nullptr;

  [[nodiscard]] CoreObserver* observer() const;
};

}  // namespace mss::pipeline::detail
