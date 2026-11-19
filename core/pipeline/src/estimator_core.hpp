#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mss/backend/graph.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/initializer.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/loop/loop_detector.hpp"
#include "mss/loop/loop_selector.hpp"
#include "mss/odom/wheel_integrator.hpp"
#include "mss/pipeline/bounded_queue.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/graph_snapshot.hpp"
#include "mss/pipeline/map_export.hpp"
#include "mss/pipeline/spsc_queue.hpp"
#include "mss/preprocess/preprocessor.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "gnss_fusion.hpp"
#include "imu_buffer.hpp"
#include "keyframe_backend.hpp"
#include "keyframe_policy.hpp"
#include "wheel_odometer.hpp"

namespace mss::pipeline::detail {

/// GNSS fixes waiting for the IMU sample at or after their time (pipeline.md section 5).
/// At 5-10 Hz GNSS and >= 100 Hz IMU at most one or two wait; when full, the oldest is
/// dropped and counted in n_gnss_ring_overflow.
constexpr std::size_t kMaxDeferredGnss = 32;

/// Outcome of the keyframe-0 gravity factor's consistency check (ADR-020).
enum class GravityCheckOutcome {
  kKept,           ///< statistic <= backend.static_gravity_chi2_gate
  kRemoved,        ///< above the gate, removed (n_gravity_factor_removed)
  kRemovalFailed,  ///< above the gate, removal failed: the factor stays (warning, no retry)
  kUnchecked,      ///< statistic not evaluated, factor kept (n_gravity_factor_unchecked)
};

/// Test-only observer of the factors EstimatorCore and its KeyframeBackend add (GNSS fusion study,
/// tests/component/test_pipeline_component.cpp `GnssFusionStudy`). Called on the estimator
/// thread, only at keyframes and accepted fixes; never set in production.
class CoreObserver {
 public:
  virtual ~CoreObserver() = default;
  /// LiDAR between factor i -> j: measurement and covariance as given to the graph.
  virtual void on_lidar_factor(common::Timestamp /*t_i*/,
                               common::Timestamp /*t_j*/,
                               const common::Pose& /*T_bi_bj*/,
                               const common::PoseCovariance& /*cov_b*/) {}
  /// GNSS factor at keyframe time `t_k` (fix shifted to t_k) with the T_e_w given to the graph.
  virtual void on_gnss_factor(common::Timestamp /*t_k*/,
                              const common::GnssMeasurement& /*m*/,
                              const common::Pose& /*T_e_w*/) {}
  /// The GNSS fusion became aligned (first time or after a realign) with the fix at `t`:
  /// `T_e_w` the aligner estimate, `yaw_sigma_rad` its predicted yaw 1-sigma (ADR-023 study,
  /// backlog G1) and `track` the track-yaw aligner (counters: segments without
  /// wheel data, reversals, turn-gated segments; meaningful with gnss.align_yaw_from_track).
  virtual void on_alignment(common::Timestamp /*t*/,
                            const common::Pose& /*T_e_w*/,
                            double /*yaw_sigma_rad*/,
                            const gnss::GnssTrackYawAligner& /*track*/) {}
  /// A fix was accepted; `T_e_w` is the aligner estimate after it.
  virtual void on_fix_accepted(common::Timestamp /*t*/, const common::Pose& /*T_e_w*/) {}
  /// Scan at `t` matched: the IMU-predicted body pose, the matched one and the match result
  /// (Hessian, covariance in l) (IMU study).
  virtual void on_scan_matched(common::Timestamp /*t*/,
                               const common::Pose& /*T_w_b_pred*/,
                               const common::Pose& /*T_w_b_match*/,
                               const common::MatchResult& /*match*/) {}
  /// Backend state of the newest keyframe after a successful optimize() (IMU study).
  virtual void on_keyframe_optimized(common::Timestamp /*t*/,
                                     const common::NavState& /*x*/,
                                     const common::PoseCovariance& /*cov*/) {}
  /// The backend's GNSS frame variable E after a successful optimize() (IMU study).
  virtual void on_frame_estimate(common::Timestamp /*t*/, const common::Pose& /*T_e_w*/) {}
  /// The gravity factor consistency check ran: its innovation statistic (NaN when unchecked)
  /// and the outcome.
  virtual void on_gravity_check(double /*chi2*/, GravityCheckOutcome /*outcome*/) {}
  /// A loop-closure search ran for the keyframe at `t` (ADR-026; loop_closure.enabled).
  virtual void on_loop_search(common::Timestamp /*t*/, const loop::LoopResult& /*result*/) {}
  /// Design 2b (only with gnss.align_yaw_from_track): the front-end heading of the scan at `t`
  /// was read from the body pose `T_w_b`; `heading_variation_rad` is the total absolute heading
  /// variation passed to the track-yaw turn gate so far, `travel_unobserved_scans` the count of
  /// scans that did not observe the travel axis (NaN without gnss.align_yaw_frontend_travel).
  virtual void on_track_heading(common::Timestamp /*t*/,
                                const common::Pose& /*T_w_b*/,
                                double /*heading_variation_rad*/,
                                double /*travel_unobserved_scans*/) {}
  /// The readings passed to the track-yaw aligner with the fix at `t` (backlog G9, G10: wheel
  /// odometer, wheel-sample gap bounds, fix time), whatever the aligner in use.
  virtual void on_track_readings(common::Timestamp /*t*/,
                                 const gnss::TrackYawReadings& /*readings*/) {}
};

/// Test-only seam (studies): `observer` receives the factor events; with `force_T_e_w` the
/// GNSS fusion uses `T_e_w` from the first fix and never runs the aligner. IMU study
/// (imu_only_observability.md): `keyframe_every_n_scans` > 0 replaces the distance /
/// rotation keyframe rule by one keyframe every n matched scans; `frontend_keeps_own_state`
/// keeps the front-end state (matched pose, IMU velocity and bias) at a keyframe instead of
/// taking the backend estimate, so the LiDAR factors carry no backend feedback.
struct CoreTestHooks {
  CoreObserver* observer = nullptr;
  bool force_T_e_w = false;
  common::Pose T_e_w;
  int keyframe_every_n_scans = 0;
  bool frontend_keeps_own_state = false;
  /// Study oracle: when set, the LiDAR factor measurement T_bi_bj is oracle(t_i, t_j)
  /// (the covariance stays the scan-matching one).
  std::function<common::Pose(common::Timestamp, common::Timestamp)> lidar_oracle;
  /// Study: when set, the LiDAR factor covariance is lidar_cov(t_j, cov_b).
  std::function<common::PoseCovariance(common::Timestamp, const common::PoseCovariance&)> lidar_cov;
  /// Study: when set, the static-gravity factor covariance is static_gravity_cov(f_b, cov).
  std::function<common::Mat3(const common::Vec3&, const common::Mat3&)> static_gravity_cov;
  /// Study (gnss_fusion_study.md section 13): when set, a GNSS fix is shifted to its keyframe
  /// by gnss_shift_oracle(t_fix, t_k), the antenna displacement in w, instead of the front-end
  /// poses (GnssFusion::collect).
  std::function<common::Vec3(common::Timestamp, common::Timestamp)> gnss_shift_oracle;
  /// Study (gnss.md section 5.1, design 2b): the T_e_w published for the first chunk of fixes from
  /// the first GNSS alignment on is turned by this yaw (rad) about each fix
  /// (GnssFusion::set_first_alignment_yaw_offset).
  double gnss_first_align_yaw_offset_rad = 0.0;
  /// ADR-026 failure path: every loop_closure.isam_extra_updates update after an accepted loop
  /// reports a failure (the loop factor itself is already committed by the first update).
  bool fail_loop_extra_updates = false;
  /// ADR-028 tests: when set, replaces LoopDetector::detect at every loop search: the views
  /// (store order, the query last) in, the detection result out.
  std::function<loop::LoopResult(common::Span<const loop::KeyframeView>)> loop_inject;
  /// ADR-028 failure path: the next `drop_loop_updates` updates after a PCM change fail and
  /// drop the pending loop factors (as a failed optimize() does); the selector must forget them.
  /// Decremented by the core (the hooks object is mutable test state).
  mutable int drop_loop_updates = 0;
  /// Loop updates that succeed before drop_loop_updates applies (ADR-030 failure-path tests).
  mutable int pass_loop_updates = 0;
  /// ADR-028 tests: called with the backend graph after every loop search that changed it (after
  /// the extra updates), to check which loop factors it holds.
  std::function<void(const backend::Graph&)> inspect_graph;
  /// ADR-033 stress tests (backend thread only): called on the backend thread before it handles
  /// each keyframe message, with the number of keyframes it handled so far; it may block or
  /// sleep to model a slow backend. Never called on the sequential path.
  std::function<void(std::uint64_t)> before_backend_keyframe;
};

/// Process-wide hooks read by every EstimatorCore constructor (nullptr: none, the default).
/// Owner: the test that sets them, on the test thread, before building an Estimator; not
/// thread-safe, never used by production code.
void set_core_test_hooks(const CoreTestHooks* hooks);

/// Diagnostic counters of EstimatorCore, all since construction.
struct CoreCounters {
  std::uint64_t n_keyframes = 0;                 ///< keyframes added to the graph
  std::uint64_t n_match_failures = 0;            ///< scans whose scan matching failed
  std::uint64_t n_skipped_scans = 0;             ///< scans that produced no estimate
  std::uint64_t n_uncovered_scans = 0;           ///< of those: IMU does not span the scan
  std::uint64_t n_map_rebuilds = 0;              ///< VoxelMap rebuilds (ADR-016)
  std::uint64_t n_degenerate_keyframes = 0;      ///< keyframes with a flagged LiDAR direction
  std::uint64_t n_degenerate_rot_keyframes = 0;  ///< of those: a rotation-dominant one flagged
  std::uint64_t n_gnss_factors = 0;              ///< GNSS factors added
  std::uint64_t n_gnss_rejected = 0;             ///< fixes rejected (gate_fix, chi-square, pose)
  std::uint64_t n_gnss_late = 0;                 ///< fixes older than the last processed scan start
  std::uint64_t n_gnss_ring_overflow = 0;        ///< deferred fixes overwritten (ring full)
  std::uint64_t n_gnss_unfused_eof = 0;  ///< deferred fixes left at end of data (no IMU after)
  std::uint64_t n_gnss_realign = 0;      ///< aligner restarts (gnss.max_consecutive_rejects)
  /// Design 2b: chunks with a GNSS track / wheel travel mismatch in the track-yaw aligner.
  std::uint64_t n_gnss_wheel_mismatch = 0;
  std::uint64_t n_gnss_frontend_wheel_disagree = 0;  ///< backlog G3, see Estimator::Stats
  std::uint64_t n_wheel_gap_bounded = 0;             ///< backlog G9, see Estimator::Stats
  std::uint64_t n_wheel_nonfinite =
      0;  ///< wheel samples with a non-finite speed (ADR-023 odometer)
  // ADR-022, see Estimator::Stats.
  std::uint64_t n_gnss_prealign_attached = 0;
  std::uint64_t n_gnss_prealign_static = 0;
  std::uint64_t n_gnss_prealign_dropped = 0;
  std::uint64_t n_gnss_prealign_not_static = 0;
  std::uint64_t n_gnss_prealign_rejected = 0;
  std::uint64_t n_nonfinite_imu = 0;             ///< IMU samples dropped (non-finite acc or gyro)
  std::uint64_t n_odom_factors = 0;              ///< wheel factors added
  std::uint64_t n_odom_rejected = 0;             ///< wheel deltas dropped by the time check
  std::uint64_t n_backend_failures = 0;          ///< optimize() errors (stub build: every keyframe)
  std::uint64_t n_gravity_factor_gated = 0;      ///< kf-0 gravity factor not added (rest gate)
  std::uint64_t n_gravity_factor_removed = 0;    ///< removed by the chi-square check (ADR-020)
  std::uint64_t n_gravity_factor_unchecked = 0;  ///< check not evaluated, factor kept (ADR-020)
  // ADR-026 (loop_closure.enabled), see Estimator::Stats.
  std::uint64_t n_loop_searches = 0;
  std::uint64_t n_loops_accepted = 0;
  std::uint64_t n_loops_rejected = 0;
  std::uint64_t n_loops_pcm_rejected = 0;  // ADR-028
  std::uint64_t n_loops_removed = 0;
  std::uint64_t n_loops_no_marginal = 0;
  std::uint64_t n_loops_released = 0;  // ADR-030: loop factors (re-)added without the kernel
  /// VoxelMap spacing replacements counted before the map's last clear() (rebuilds); the
  /// total is this plus VoxelMap::num_spacing_replaced().
  std::uint64_t n_map_spacing_replaced_cleared = 0;
  /// ADR-033 map snapshots: updates dropped for want of a free buffer (then rebuilt), and
  /// buffers copied whole instead of replaying the logged updates (MapPublisher).
  std::uint64_t n_map_publish_skipped = 0;
  std::uint64_t n_map_full_copies = 0;
  /// ADR-033 backend thread (see Estimator::Stats): messages the front-end could not queue
  /// (keyframes also for want of a free cloud buffer), updates the backend could not queue, and
  /// corrections the front-end applied.
  std::uint64_t n_backend_keyframes_dropped = 0;
  std::uint64_t n_backend_fixes_dropped = 0;
  std::uint64_t n_backend_prunes_dropped = 0;
  std::uint64_t n_backend_updates_dropped = 0;
  std::uint64_t n_backend_corrections = 0;
};

/// ADR-033 backend thread: compile-time capacities (no allocation after start-up).
/// Messages front-end -> backend (keyframes, GNSS fixes, prune requests, in send order).
constexpr std::size_t kBackendQueueDepth = 64;
/// BackendUpdates backend -> front-end.
constexpr std::size_t kUpdateQueueDepth = 64;
/// Keyframe clouds in flight to the backend (ScanFrames of realtime.max_points_per_scan each).
constexpr std::size_t kKeyframeCloudSlots = 4;
/// Keyframes sent whose BackendUpdate the front-end has not applied yet (at most the cloud slots,
/// the update queue and the update held back, see EstimatorCore::apply_backend_updates).
constexpr std::size_t kMaxPendingKeyframes = kUpdateQueueDepth + kKeyframeCloudSlots + 2;

/// One message front-end -> backend thread (ADR-033). One queue for every kind keeps the send
/// order, which the backend relies on: the fixes sent before a keyframe are fused into it.
struct BackendMsg {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum class Kind : std::uint8_t { kKeyframe, kGnssFix, kPrune };
  Kind kind = Kind::kPrune;
  KeyframeMsg kf;  ///< kKeyframe; kf.cloud_l points into the cloud slot `cloud_slot`
  std::size_t cloud_slot = 0;
  GnssFixMsg fix;                             ///< kGnssFix
  common::Vec3 p_w_b = common::Vec3::Zero();  ///< kPrune: front-end body position
};

/// The state shared between the front-end (the thread that calls Estimator::run_once / flush) and
/// the backend thread (ADR-033). Ownership: the queues have one producer and one consumer each
/// (SpscQueue); a cloud slot belongs to the front-end while `cloud_busy` is false and to the
/// backend from the push of its keyframe until the backend clears the flag. `backend_mutex` is
/// held by the backend thread while it handles one message and by the read queries
/// (visit_map, graph_snapshot); the front-end pipeline never takes it. `counters_mutex` guards the
/// counters the backend publishes after each message (held for one copy).
struct BackendChannel {
  explicit BackendChannel(std::size_t max_points) {
    for (common::ScanFrame& c : clouds) {
      c.reserve(max_points);
    }
  }
  SpscQueue<BackendMsg, kBackendQueueDepth> to_backend;
  SpscQueue<BackendUpdate, kUpdateQueueDepth> to_frontend;
  std::array<common::ScanFrame, kKeyframeCloudSlots> clouds;
  std::array<std::atomic<bool>, kKeyframeCloudSlots> cloud_busy{};
  // Wake-up: the front-end notifies after each push; the backend waits with a timeout
  // (kBackendIdleWait) that also catches a notify sent between its empty check and its wait.
  std::mutex wake_mutex;
  std::condition_variable wake;
  std::atomic<bool> stop{false};
  std::mutex backend_mutex;
  std::mutex counters_mutex;
  BackendCounters counters;               // under counters_mutex
  std::uint64_t n_updates_dropped = 0;    // under counters_mutex
  std::uint64_t n_keyframes_handled = 0;  // backend thread only
  BackendMsg msg;                         // backend thread only: the message being handled
  BackendMsg send;                        // front-end thread only: the message being queued
};

/// The per-message algorithm of the estimator (docs/derivations/pipeline.md, ADR-033). Owns every
/// module: the front-end ones directly, the graph, keyframe store, loop closure, GNSS fusion and
/// the map writer through KeyframeBackend; the front-end matches each scan against the map
/// snapshot the backend published last and never changes it (it asks the backend to prune).
/// Messages must arrive in the Estimator's k-way merge order (IMU, wheel, GNSS by t; scans by
/// their end time). Internal to the pipeline. Threading: every method on the front-end thread
/// (the Estimator's caller), except run_backend() (the backend thread, between start_backend()
/// and finish_backend()), which reaches only backend_ and the BackendChannel. Large (~0.5 MB):
/// hold it on the heap.
class EstimatorCore {
 public:
  /// Builds every module from the validated `cfg` and reserves every buffer.
  /// Throws std::invalid_argument (from module constructors, or naming the key when
  /// `imu.init_window_s` does not fit the IMU buffer or `map.window_keyframes >
  /// map.max_keyframes`), std::bad_alloc.
  explicit EstimatorCore(const Config& cfg);
  ~EstimatorCore();
  EstimatorCore(const EstimatorCore&) = delete;
  EstimatorCore& operator=(const EstimatorCore&) = delete;

  /// Buffers one IMU sample (non-increasing t ignored). No allocation.
  void on_imu(const common::ImuSample& s);
  /// Buffers one wheel sample and records the observed wheel period. No allocation.
  void on_wheel(const common::WheelSample& s);
  /// Defers one GNSS fix until an IMU sample at or after m.t is buffered, then sends it to the
  /// backend (GnssFixMsg: gate, alignment, queue) at the body pose predicted to m.t
  /// (gnss.enabled). Before keyframe 0 it is sent at once, without a pose (ADR-022). Fixes older
  /// than the last processed scan start are rejected (n_gnss_late). No allocation.
  void on_gnss(const common::GnssMeasurement& m);
  /// End of data (Estimator::flush): deferred fixes that no IMU sample reached are
  /// dropped and counted in n_gnss_unfused_eof.
  void on_end_of_data();
  /// Processes one raw scan (frame l). Non-keyframe scans do not allocate.
  void on_scan(const common::ScanFrame& raw_l);

  [[nodiscard]] bool has_estimate() const { return has_estimate_; }
  /// Latest estimate (Estimator::latest_state() semantics). Default until has_estimate().
  [[nodiscard]] const common::Estimate& latest() const { return latest_; }
  /// The front-end counters merged with the backend's (BackendCounters, GNSS fusion included).
  [[nodiscard]] CoreCounters counters() const;
  /// ADR-023: signed wheel odometer (m) over the finite wheel samples received so far.
  [[nodiscard]] double wheel_odometer_m() const { return odometer_.reading_m(); }
  /// Backlog G9: sum (m) of the travel bounds of the wheel-sample gaps longer than
  /// gnss.align_yaw_wheel_gap_periods nominal periods that the odometer interpolated across.
  [[nodiscard]] double wheel_gap_bound_m() const { return odometer_.gap_bound_m(); }
  /// Backlog G9: the nominal wheel-sample period (s) the gaps are measured against.
  [[nodiscard]] double wheel_nominal_period_s() const { return odometer_.nominal_period_s(); }
  /// The latest published map snapshot (read-only, for diagnostics such as Estimator::stats());
  /// hold the Ref while reading it.
  [[nodiscard]] MapRef voxel_map() const { return backend_.acquire_map(); }
  /// Global map export (Estimator::visit_map): KeyframeBackend::visit_map. With the backend
  /// thread it takes BackendChannel::backend_mutex (waits for the message being handled).
  MapExportInfo visit_map(const MapKeyframeVisitor& visit) const;
  /// Graph snapshot (Estimator::graph_snapshot, ADR-031): KeyframeBackend::graph_snapshot. Locks
  /// as visit_map().
  std::size_t graph_snapshot(GraphSnapshot& out) const;

  /// ADR-033 backend thread, driven by Estimator (the only owner of threads):
  /// start_backend() on the front-end thread before the thread starts (precondition:
  /// deterministic: false, so the channel exists); from then on keyframes, GNSS fixes and prune
  /// requests go through BackendChannel::to_backend and the front-end applies the BackendUpdates
  /// at scan boundaries. run_backend() is the thread body: it handles the messages in order and
  /// returns once request_backend_stop() was called and its queue is empty. finish_backend() on
  /// the front-end thread after the join applies the updates left and returns to the sequential
  /// path. Throws nothing.
  [[nodiscard]] bool can_thread() const { return chan_ != nullptr; }
  [[nodiscard]] bool backend_threaded() const { return async_; }
  void start_backend();
  void run_backend();
  void request_backend_stop();
  void finish_backend();

 private:
  // ADR-033 front-end side. A keyframe needs a free slot in the queue and a free cloud slot
  // (chosen into cloud_slot_); checked before any keyframe state changes.
  [[nodiscard]] bool backend_can_take_keyframe();
  // Queues the keyframe (cloud already in the chosen slot) and records its x0; returns its id.
  common::KeyId send_keyframe(const KeyframeMsg& msg);
  // Sequential: to the backend now. Threaded: queued (false and counted when full).
  void send_fix(const GnssFixMsg& msg);
  bool send_prune(const common::Vec3& p_w_b);
  // Applies, in order, the queued BackendUpdates whose keyframe the snapshot (epoch
  // MapBuffer::n_keyframes) holds; the first one it does not stays held for the next scan.
  void apply_backend_updates(std::uint64_t epoch);
  // T_corr = T_w_b(opt) * T_w_b(x0)^-1 applied to front_, ref_ and the pending x0 (ADR-033 4).
  void apply_update(const BackendUpdate& u);
  // Backend thread: handles one message and publishes the counters.
  void handle_backend_msg(BackendMsg& m);
  void notify_backend();
  // The buffer the next keyframe cloud goes to: kf_cloud_, or the chosen cloud slot (threaded).
  common::ScanFrame& keyframe_cloud_out();
  // Processes deferred GNSS fixes whose time the buffered IMU now covers.
  void process_deferred_gnss();
  // Sends one fix to the backend (GnssFixMsg) at the body pose predicted from front_ to m.t
  // (m.t >= front_.t, IMU covers it), with the track-yaw readings of the front-end.
  void fuse_gnss(const common::GnssMeasurement& m);
  void track_heading(const common::MatchResult* match);
  // Feeds `s` to both preintegrators when it is newer than the last fed sample.
  void feed_preintegrators(const common::ImuSample& s);
  // Drops buffered samples older than the last one at or before `t`.
  void prune_imu(common::Timestamp t);
  // Integrates buffered wheel samples with t <= `t`.
  void integrate_wheel(common::Timestamp t);
  void consume_wheel(const common::WheelSample& s);  // integrate one sample, track motion
  // First scan after initialisation: keyframe 0.
  void first_keyframe(common::Timestamp t, const common::NavState& predicted);
  // Adds keyframe j for the matched scan at `t` (front_ holds the matched state).
  void add_keyframe(common::Timestamp t,
                    const common::MatchResult& match,
                    const common::PoseCovariance& cov_b,
                    common::PoseCovariance& cov_out);
  // The keyframe cloud at `t` (from pre_out_) into `out`, for the backend's store.
  void prepare_keyframe_cloud(common::Timestamp t, common::ScanFrame& out);
  // Prediction preintegrator restarted at front_ (bias) with the boundary sample.
  void reset_prediction();

  Config cfg_;
  common::Pose T_b_l_;
  common::Pose T_l_b_;
  common::Vec3 g_w_;
  std::int64_t init_window_ns_ = 0;

  imu::ImuInitializer initializer_;
  imu::Preintegrator pred_;     // scan-to-scan prediction
  imu::Preintegrator kf_;       // keyframe-to-keyframe factor delta
  imu::Preintegrator scratch_;  // deskew trajectory / GNSS pose prediction
  preprocess::Preprocessor preprocessor_;
  preprocess::Preprocessor kf_preprocessor_;
  scan_match::ScanMatcher matcher_;
  // ADR-033: graph, keyframe store, loop closure, GNSS fusion, the map; reached only through
  // KeyframeMsg, GnssFixMsg and prune_map in, BackendUpdate and the map snapshot out and its read
  // queries.
  KeyframeBackend backend_;
  odom::WheelIntegrator wheel_;

  ImuBuffer imu_;
  std::unique_ptr<std::array<common::ImuSample, kMaxImuQueue>> init_window_;
  std::unique_ptr<imu::ImuTrajectory> traj_;
  BoundedQueue<common::WheelSample, kMaxAuxQueue> wheel_queue_;
  BoundedQueue<common::GnssMeasurement, kMaxDeferredGnss> gnss_deferred_;
  common::ScanFrame pre_out_;
  common::ScanFrame kf_cloud_;

  bool initialised_ = false;
  bool has_estimate_ = false;
  bool has_last_scan_ = false;
  common::Timestamp last_scan_t_{};
  common::Timestamp pred_last_t_{};  // time of the last sample fed to pred_ / kf_
  common::ImuSample boundary_;       // interpolated sample at the last scan start
  common::NavState front_;           // front-end state at last_scan_t_
  common::Pose ref_;                 // reference pose of the last keyframe (pipeline.md s4)
  imu::InitState init_;              // static-window statistics (static_gravity_factor)
  common::Timestamp init_t_{};       // scan start at which the initialisation passed
  common::KeyId last_kf_id_ = 0;
  common::Timestamp last_kf_t_{};
  common::Timestamp last_prune_t_{};
  bool have_wheel_ = false;
  common::Timestamp last_wheel_t_{};
  // ADR-023: signed wheel odometer (trapezoid of the finite WheelSample::v_mps) for the
  // track-yaw direction of travel, with the bound of the travel it interpolates across sample
  // gaps longer than gnss.align_yaw_wheel_gap_periods nominal periods (backlog G9), and the
  // time of the last GNSS fix it was read at.
  WheelOdometer odometer_{cfg_.gnss.align_yaw_wheel_gap_periods};
  bool wheel_gap_logged_ = false;  // one log line per run (backlog G9)
  bool have_fix_odometer_t_ = false;
  common::Timestamp last_fix_odometer_t_{};
  // Design 2b: total absolute heading variation of the front end (rad), summed scan to scan when
  // gnss.align_yaw_from_track, and the heading of the last scan it was updated at.
  bool have_scan_heading_ = false;
  double last_scan_heading_rad_ = 0.0;
  double heading_variation_rad_ = 0.0;
  // Scans whose matching did not observe the travel axis (gnss.align_yaw_frontend_travel).
  double travel_unobserved_scans_ = 0.0;
  std::int64_t wheel_period_ns_ = 0;
  bool init_failure_logged_ = false;
  bool match_failure_streak_ = false;
  bool imu_nonfinite_streak_ = false;
  bool gnss_late_streak_ = false;
  bool uncovered_streak_ = false;

  common::Estimate latest_;
  CoreCounters counters_;
  // Test-only seam (CoreTestHooks), read at construction; nullptr in production. The hooks
  // object must outlive the core (HooksScope in the tests).
  const CoreTestHooks* hooks_ = nullptr;
  int scans_since_keyframe_ = 0;  // test-only: matched scans since the last keyframe
  // Time of the last consumed wheel sample (taken from wheel_queue_, so in time order) with
  // a speed above imu.init_max_wheel_speed_mps; set when wheel_moved_.
  bool wheel_moved_ = false;
  common::Timestamp last_wheel_motion_t_{};
  bool init_wheel_moved_ = false;  // such a sample in [init_t_ - window, init_t_]

  // ADR-033 backend thread. chan_ exists only with deterministic: false; async_ between
  // start_backend() and finish_backend(). Front-end thread only below.
  std::unique_ptr<BackendChannel> chan_;
  bool async_ = false;
  std::size_t cloud_slot_ = 0;    // chosen by backend_can_take_keyframe()
  common::KeyId next_kf_id_ = 0;  // KeyIds are dense from 0 (KeyframeBackend)
  // x0 (pose, velocity) of keyframes pending_first_kf_ .. next_kf_id_-1 (ring by id), kept in the
  // current front-end frame (each applied correction moves them too).
  common::KeyId pending_first_kf_ = 0;
  std::array<common::NavState, kMaxPendingKeyframes> pending_x0_;
  BackendUpdate held_update_;  // popped, but its snapshot was not published yet
  bool has_held_update_ = false;
  [[nodiscard]] CoreObserver* observer() const {
    return hooks_ != nullptr ? hooks_->observer : nullptr;
  }
};

}  // namespace mss::pipeline::detail
