#pragma once

#include <cstdint>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/config.hpp"

namespace mss::pipeline {

/// The estimator: owns every module instance, the bounded queues and (ADR-005) the only
/// threads in core. Apps (bench, ros2_node) talk only to this class.
///
/// Message flow: push_*() copy a message into a bounded queue and return at once (never
/// block). Processing happens in run_once() (deterministic, single-threaded driver) or,
/// once ADR-005 threads are implemented, on the estimator's own threads between start()
/// and stop().
///
/// Current phase (Task 8): single-threaded only; the module algorithms are not plugged
/// in yet, so latest_state() reports an identity pose (see latest_state()).
///
/// Keyframe-boundary duties of the estimator (contract for imu, odom and backend):
///   - keyframe time = scan `t_start`;
///   - after `imu::Preintegrator::reset()` the estimator re-feeds the last IMU sample of
///     the previous interval as the first sample of the new one;
///   - before `backend::Graph::add_odom_factor` the estimator checks
///     `|OdomDelta.t0 - t_i|` and `|OdomDelta.t1 - t_j|` `<= 1.5 x` the observed wheel
///     sample period (interval between consecutive accepted WheelSamples) and drops the
///     factor with one log line otherwise.
///
/// Ownership/threading: one instance per process, owned by the app. Shared state with a
/// documented owner (ADR-005): the latest Estimate (written by the backend stage, read by
/// latest_state()) and the map (front-end). In the current single-threaded phase every
/// method must be called from one thread; no method is thread-safe yet.
class Estimator {
 public:
  /// Per-run counters, all since construction.
  struct Stats {
    std::uint64_t n_scans = 0;          ///< scans processed by run_once()
    std::uint64_t n_imu = 0;            ///< IMU samples processed by run_once()
    std::uint64_t n_gnss = 0;           ///< GNSS fixes processed by run_once()
    std::uint64_t n_odom = 0;           ///< wheel samples processed by run_once()
    std::uint64_t n_dropped_scans = 0;  ///< scans discarded by push_scan (queue full or too big)
    std::uint64_t n_rejected_imu = 0;   ///< samples refused by push_imu (non-increasing t)
    double push_scan_us_p50 = 0.0;      ///< us, median wall time of push_scan()
    double push_scan_us_p95 = 0.0;      ///< us, 95th percentile wall time of push_scan()
  };

  /// Builds every module and reserves every buffer: a pool of
  /// `realtime.scan_queue_depth + 2` ScanFrames of `realtime.max_points_per_scan` points,
  /// the IMU queue (`imu_queue_depth`) and the GNSS/wheel queues (`aux_queue_depth`).
  /// No allocation happens after the constructor on any push_*/run_once path.
  /// Preconditions: `cfg` comes from load_config (validated).
  /// Errors: throws (initialisation may throw) std::invalid_argument from module
  ///   constructors, std::bad_alloc.
  /// Hot path: no.
  /// Tests: Task 8 test_estimator (PushScanCopiesWithoutAlloc), E-01..E-03.
  explicit Estimator(const Config& cfg);

  /// Copies `scan` (memcpy of the SoA arrays, no allocation) into a free pool frame and
  /// queues it. The caller keeps ownership of `scan` and may reuse it immediately.
  /// When the scan queue is full, the OLDEST queued scan is dropped, the new one is
  /// queued, n_dropped_scans is incremented, and the call returns
  /// Status::error("scan queue full, dropped oldest") (latency over completeness).
  /// Frames: scan in l, per-point times relative to scan.t_start. Units: m, ns.
  /// Errors: also Status::error("scan larger than max_points_per_scan") when
  ///   scan.size() > realtime.max_points_per_scan; that scan is not queued and counts in
  ///   n_dropped_scans.
  /// Ordering is not checked here; run_once() processes in key order (scan end time),
  /// see run_once().
  /// Hot path: yes (per scan, caller's thread): no allocation, no exception; logs at
  ///   most one warning per call.
  /// Tests: Task 8 test_estimator (PushScanCopiesWithoutAlloc, ScanQueueFullDropsOldest),
  ///   E-03.
  common::Status push_scan(const common::ScanFrame& scan);

  /// Queues one IMU sample.
  /// Errors: Status::error("imu timestamp not increasing") when `s.t <=` the previous
  ///   ACCEPTED sample's t; the sample is dropped and n_rejected_imu incremented.
  ///   When the IMU queue is full (a configuration error, ADR-005) the oldest sample is
  ///   dropped, an error is logged, and the call returns
  ///   Status::error("imu queue full, dropped oldest").
  /// Frames: b. Units: m/s^2, rad/s, ns.
  /// Hot path: yes (per IMU sample): no allocation, no exception.
  /// Tests: Task 8 test_estimator (ImuNonIncreasingRejected), E-03.
  common::Status push_imu(const common::ImuSample& s);

  /// Queues one GNSS fix (aux queue, depth aux_queue_depth). Queued regardless of
  /// gnss.enabled; the flag only decides whether run_once() uses the fix (it is counted
  /// in n_gnss either way).
  /// Queue full: oldest dropped, warning logged, Status::error("gnss queue full, dropped
  /// oldest").
  /// Frames: m.p_e_g in e. Units: m, m^2.
  /// Hot path: no (5 Hz) but allocation-free. Tests: E-01, E-03.
  common::Status push_gnss(const common::GnssMeasurement& m);

  /// Queues one wheel-odometry sample (aux queue, depth aux_queue_depth). Queued
  /// regardless of wheel_odom.enabled; the flag only decides whether run_once() uses it
  /// (counted in n_odom either way).
  /// Queue full: oldest dropped, warning logged, Status::error("odom queue full, dropped
  /// oldest").
  /// Units: m/s, rad/s. Hot path: yes (50 Hz), allocation-free. Tests: E-01, E-03.
  common::Status push_odom(const common::WheelSample& s);

  /// Copy of the most recent estimate. Before the first scan has been processed it is a
  /// default-constructed Estimate (state.t == Timestamp{}, identity pose, keyframe_id 0);
  /// callers detect "no estimate yet" by `state.t == Timestamp{}`.
  /// Current phase (Task 8): state.t = t_start of the last processed scan, pose identity,
  /// zero velocity and bias, cov zero (no algorithm plugged in yet).
  /// Frames: state.T_w_b, v_w_b in w; cov [rho, phi] of T_w_b. Hot path: no allocation.
  /// Tests: Task 8 test_estimator (LatestStateTimestampFollowsScans), E-01.
  common::Estimate latest_state() const;

  /// Deterministic driver: on the caller's thread, drains ALL queues and processes the
  /// messages in key order, then returns. A scan's ordering key is its end time
  /// `t_start + max(t_offset_ns)`; other messages use `t`. run_once() merges the queue
  /// heads by that key (k-way merge; ties: IMU, wheel, GNSS, scan); each queue is FIFO
  /// and assumed sorted by its producer (IMU is checked by push_imu, others are not). With
  /// `deterministic: true` this is the only way processing happens, and two runs over
  /// the same input produce bit-identical results (E-02).
  /// Errors: none returned; per-message failures are logged and counted.
  /// Hot path: yes (per scan); no allocation.
  /// Tests: Task 8 test_estimator (RunOnceDrainsQueues), E-02.
  void run_once();

  /// Starts the ADR-005 threads (front-end, backend, loop). Not implemented in this
  /// phase: with `deterministic: true` it is a no-op that logs a warning (use
  /// run_once()); with `deterministic: false` it is also a no-op that logs a warning
  /// that threaded mode is not implemented yet. Never throws. Tests: Task 8.
  void start();

  /// Stops and joins the threads started by start(). Same no-op-with-warning behaviour
  /// as start() in this phase. Never throws. Tests: Task 8.
  void stop();

  /// Snapshot of the counters. push_scan percentiles come from a fixed ring holding the
  /// most recent 4096 push_scan() durations measured with std::chrono::steady_clock (wall time,
  /// diagnostics only; never used by the estimation, so E-02 determinism is unaffected).
  /// Percentiles are nearest-rank on the `n = min(count, 4096)` stored samples:
  /// `p_q = sorted[ceil(q * n) - 1]` (q = 0.50, 0.95).
  /// 0.0 before the first push_scan(). Tests: Task 8 test_estimator.
  Stats stats() const;
};

}  // namespace mss::pipeline
