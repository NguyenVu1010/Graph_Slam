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
};

/// Process-wide hooks read by every EstimatorCore constructor (nullptr: none, the default).
/// Owner: the test that sets them, on the test thread, before building an Estimator; not
/// thread-safe, never used by production code.
void set_core_test_hooks(const CoreTestHooks* hooks);

/// Diagnostic counters of EstimatorCore, all since construction.
struct CoreCounters {
  std::uint64_t n_keyframes = 0;           ///< keyframes added to the graph
  std::uint64_t n_match_failures = 0;      ///< scans whose scan matching failed
  std::uint64_t n_skipped_scans = 0;       ///< scans that produced no estimate
  std::uint64_t n_uncovered_scans = 0;     ///< of those: IMU does not span the scan
  std::uint64_t n_map_rebuilds = 0;        ///< VoxelMap rebuilds (ADR-016)
  std::uint64_t n_gnss_factors = 0;        ///< GNSS factors added
  std::uint64_t n_gnss_rejected = 0;       ///< fixes rejected (gate_fix, chi-square, pose)
  std::uint64_t n_gnss_late = 0;           ///< fixes older than the last processed scan start
  std::uint64_t n_gnss_ring_overflow = 0;  ///< deferred fixes overwritten (ring full)
  std::uint64_t n_gnss_unfused_eof = 0;    ///< deferred fixes left at end of data (no IMU after)
  std::uint64_t n_gnss_realign = 0;        ///< aligner restarts (gnss.max_consecutive_rejects)
  std::uint64_t n_nonfinite_imu = 0;       ///< IMU samples dropped (non-finite acc or gyro)
  std::uint64_t n_odom_factors = 0;        ///< wheel factors added
  std::uint64_t n_odom_rejected = 0;       ///< wheel deltas dropped by the time check
  std::uint64_t n_backend_failures = 0;    ///< optimize() errors (stub build: every keyframe)
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
  /// The VoxelMap cache (read-only, for diagnostics such as Estimator::stats()).
  [[nodiscard]] const map::VoxelMap& voxel_map() const { return map_; }

 private:
  // Processes deferred GNSS fixes whose time the buffered IMU now covers.
  void process_deferred_gnss();
  // One fix at the body pose predicted from front_ to m.t (m.t >= front_.t, IMU covers it).
  void fuse_gnss(const common::GnssMeasurement& m);
  // Feeds `s` to both preintegrators when it is newer than the last fed sample.
  void feed_preintegrators(const common::ImuSample& s);
  // Drops buffered samples older than the last one at or before `t`.
  void prune_imu(common::Timestamp t);
  // Integrates buffered wheel samples with t <= `t`.
  void integrate_wheel(common::Timestamp t);
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
  std::int64_t wheel_period_ns_ = 0;
  std::size_t map_dropped_ = 0;
  bool init_failure_logged_ = false;
  bool match_failure_streak_ = false;
  bool backend_failure_streak_ = false;
  bool imu_nonfinite_streak_ = false;
  bool gnss_reject_streak_ = false;
  bool gnss_late_streak_ = false;
  bool uncovered_streak_ = false;

  common::Estimate latest_;
  CoreCounters counters_;
  CoreObserver* observer_ = nullptr;       // test-only (CoreTestHooks)
  int keyframe_every_n_scans_ = 0;         // test-only (CoreTestHooks)
  int scans_since_keyframe_ = 0;           // test-only: matched scans since the last keyframe
  bool frontend_keeps_own_state_ = false;  // test-only (CoreTestHooks)
  std::function<common::Pose(common::Timestamp, common::Timestamp)> lidar_oracle_;  // test-only
  std::function<common::PoseCovariance(common::Timestamp, const common::PoseCovariance&)>
      lidar_cov_;  // test-only
};

}  // namespace mss::pipeline::detail
