#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "mss/backend/graph.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/initializer.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/imu/trajectory.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/odom/wheel_integrator.hpp"
#include "mss/pipeline/bounded_queue.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/map_export.hpp"
#include "mss/preprocess/preprocessor.hpp"
#include "mss/scan_match/scan_matcher.hpp"

#include "gnss_fusion.hpp"
#include "imu_buffer.hpp"
#include "keyframe_policy.hpp"
#include "keyframe_store.hpp"

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

/// Test-only observer of the factors EstimatorCore adds (the GNSS fusion study,
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
  /// `T_e_w` the aligner estimate, `yaw_sigma_rad` its predicted yaw 1-sigma (NaN for
  /// GnssAligner; ADR-023 study) and `track` the track-yaw aligner (counters: segments without
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
  /// Design 2b (only with gnss.align_yaw_from_track): the front-end heading of the scan at `t`
  /// was read from the body pose `T_w_b`; `heading_variation_rad` is the total absolute heading
  /// variation passed to the track-yaw turn gate so far, `travel_unobserved_scans` the count of
  /// scans that did not observe the travel axis (NaN without gnss.align_yaw_frontend_travel).
  virtual void on_track_heading(common::Timestamp /*t*/,
                                const common::Pose& /*T_w_b*/,
                                double /*heading_variation_rad*/,
                                double /*travel_unobserved_scans*/) {}
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
  /// VoxelMap spacing replacements counted before the map's last clear() (rebuilds); the
  /// total is this plus VoxelMap::num_spacing_replaced().
  std::uint64_t n_map_spacing_replaced_cleared = 0;
};

/// The per-message algorithm of the single-threaded estimator (docs/derivations/pipeline.md,
/// ADR-005 threads deferred). Owns every module. Messages must arrive in the Estimator's
/// k-way merge order (IMU, wheel, GNSS by t; scans by their end time).
/// Internal to the pipeline. Single-threaded. Large (~0.5 MB): hold it on the heap.
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
  /// Defers one GNSS fix until an IMU sample at or after m.t is buffered, then gates,
  /// aligns and queues it at the body pose interpolated to m.t (gnss.enabled and an
  /// estimate needed). Fixes older than the last processed scan start are rejected
  /// (n_gnss_late). No allocation.
  void on_gnss(const common::GnssMeasurement& m);
  /// End of data (Estimator::flush): deferred fixes that no IMU sample reached are
  /// dropped and counted in n_gnss_unfused_eof.
  void on_end_of_data();
  /// Processes one raw scan (frame l). Non-keyframe scans do not allocate.
  void on_scan(const common::ScanFrame& raw_l);

  [[nodiscard]] bool has_estimate() const { return has_estimate_; }
  /// Latest estimate (Estimator::latest_state() semantics). Default until has_estimate().
  [[nodiscard]] const common::Estimate& latest() const { return latest_; }
  [[nodiscard]] const CoreCounters& counters() const { return counters_; }
  /// ADR-023: signed wheel odometer (m) over the finite wheel samples received so far.
  [[nodiscard]] double wheel_odometer_m() const { return wheel_odometer_m_; }
  /// The VoxelMap cache (read-only, for diagnostics such as Estimator::stats()).
  [[nodiscard]] const map::VoxelMap& voxel_map() const { return map_; }
  /// Global map export (Estimator::visit_map): calls `visit` for every keyframe in the store,
  /// oldest first, at T_w_b(k) * T_b_l with T_w_b(k) = graph_.keyframe_state(k) now; entries
  /// without a backend estimate are skipped and counted (n_no_pose). Reads only; allocates
  /// nothing itself.
  MapExportInfo visit_map(const MapKeyframeVisitor& visit) const;

 private:
  // Processes deferred GNSS fixes whose time the buffered IMU now covers.
  void process_deferred_gnss();
  // One fix at the body pose predicted from front_ to m.t (m.t >= front_.t, IMU covers it).
  void fuse_gnss(const common::GnssMeasurement& m);
  // ADR-022: assigns the pre-alignment fixes up to keyframe j (at t, front_ holds its
  // front-end pose) and, once aligned, adds them as factors on their keyframes.
  void attach_prealign_fixes(common::KeyId j, common::Timestamp t);
  void sync_prealign_counters();
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
  // Inserts the keyframe cloud (from pre_out_) at front_ into store, map and window.
  void insert_keyframe_cloud(common::KeyId id, common::Timestamp t);
  // ADR-016 rebuild check after a successful optimize().
  void maybe_rebuild_map();
  // ADR-020 consistency check of the gravity factor after keyframe `j` was optimised.
  void check_gravity_factor(common::KeyId j);
  // Prediction preintegrator restarted at front_ (bias) with the boundary sample.
  void reset_prediction();
  void log_backend_failure(const std::string& reason);

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
  map::VoxelMap map_;
  scan_match::ScanMatcher matcher_;
  backend::Graph graph_;
  odom::WheelIntegrator wheel_;
  GnssFusion fusion_;
  KeyframeStore store_;
  MapWindow window_;

  ImuBuffer imu_;
  std::unique_ptr<std::array<common::ImuSample, kMaxImuQueue>> init_window_;
  std::unique_ptr<imu::ImuTrajectory> traj_;
  std::unique_ptr<std::array<common::GnssMeasurement, kMaxAuxQueue>> gnss_out_;
  // ADR-022 output of GnssFusion::take_prealign; nullptr unless gnss.attach_prealign_fixes.
  std::unique_ptr<std::array<EarlyFactor, kMaxAuxQueue>> early_out_;
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
  // ADR-023: signed wheel odometer (m, trapezoid of the finite WheelSample::v_mps) for the
  // track-yaw direction of travel, and the time of the last GNSS fix it was read at.
  bool have_odometer_ = false;
  double last_odometer_v_mps_ = 0.0;
  common::Timestamp last_odometer_t_{};
  double wheel_odometer_m_ = 0.0;
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
  std::size_t map_dropped_ = 0;
  bool init_failure_logged_ = false;
  bool match_failure_streak_ = false;
  bool backend_failure_streak_ = false;
  bool imu_nonfinite_streak_ = false;
  bool gnss_reject_streak_ = false;
  bool track_restart_logged_ = false;    // one log line until the next alignment
  std::uint64_t n_alignments_seen_ = 0;  // observer only: alignments reported so far
  bool gnss_late_streak_ = false;
  bool uncovered_streak_ = false;

  common::Estimate latest_;
  CoreCounters counters_;
  // Test-only seam (CoreTestHooks), read at construction; nullptr in production. The hooks
  // object must outlive the core (HooksScope in the tests).
  const CoreTestHooks* hooks_ = nullptr;
  int scans_since_keyframe_ = 0;  // test-only: matched scans since the last keyframe
  // Gravity-factor state (ADR-020): added at keyframe 0; checked once.
  bool gravity_factor_added_ = false;
  bool gravity_factor_checked_ = false;
  // Time of the last consumed wheel sample (taken from wheel_queue_, so in time order) with
  // a speed above imu.init_max_wheel_speed_mps; set when wheel_moved_.
  bool wheel_moved_ = false;
  common::Timestamp last_wheel_motion_t_{};
  bool init_wheel_moved_ = false;  // such a sample in [init_t_ - window, init_t_]

  [[nodiscard]] CoreObserver* observer() const {
    return hooks_ != nullptr ? hooks_->observer : nullptr;
  }
};

}  // namespace mss::pipeline::detail
