#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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

/// Diagnostic counters of EstimatorCore, all since construction.
struct CoreCounters {
  std::uint64_t n_keyframes = 0;         ///< keyframes added to the graph
  std::uint64_t n_match_failures = 0;    ///< scans whose scan matching failed
  std::uint64_t n_skipped_scans = 0;     ///< scans that produced no estimate
  std::uint64_t n_uncovered_scans = 0;   ///< of those: no IMU up to the scan end
  std::uint64_t n_map_rebuilds = 0;      ///< VoxelMap rebuilds (ADR-016)
  std::uint64_t n_gnss_factors = 0;      ///< GNSS factors added
  std::uint64_t n_gnss_rejected = 0;     ///< fixes rejected (gate_fix, chi-square, pose)
  std::uint64_t n_odom_factors = 0;      ///< wheel factors added
  std::uint64_t n_odom_rejected = 0;     ///< wheel deltas dropped by the time check
  std::uint64_t n_backend_failures = 0;  ///< optimize() errors (stub build: every keyframe)
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
  /// Gates, aligns and queues one GNSS fix (gnss.enabled and an estimate needed).
  void on_gnss(const common::GnssMeasurement& m);
  /// Processes one raw scan (frame l). Non-keyframe scans do not allocate.
  void on_scan(const common::ScanFrame& raw_l);

  bool has_estimate() const { return has_estimate_; }
  /// Latest estimate (Estimator::latest_state() semantics). Default until has_estimate().
  const common::Estimate& latest() const { return latest_; }
  const CoreCounters& counters() const { return counters_; }

 private:
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

  common::Estimate latest_;
  CoreCounters counters_;
};

}  // namespace mss::pipeline::detail
