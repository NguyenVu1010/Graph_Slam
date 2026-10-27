#pragma once

#include <cstdint>
#include <memory>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/config.hpp"

namespace mss::pipeline {

/// The estimator: owns every module instance, the bounded queues and (ADR-005) the only
/// threads in core. Apps (bench, ros2_node) talk only to this class.
///
/// Message flow: push_*() copy a message into a bounded queue and return at once (never
/// block). Processing happens in run_once() / flush() (deterministic, single-threaded
/// driver) or, once ADR-005 threads are implemented, on the estimator's own threads
/// between start() and stop().
///
/// IMU coverage gating: a queued scan is processed only once an IMU sample with
/// `t >= scan_end_time(scan)` (its ordering key) has been accepted by push_imu, so the
/// IMU samples of the whole scan span are in hand before the scan (deskew, propagation).
/// Until then the scan, and everything after it in key order, stays queued. Callers may
/// therefore call run_once() as often as they like: processing early never reorders or
/// starves a scan. flush() ignores the gate (end of data).
///
/// Current phase: the modules are plugged in (IMU static initialisation, deskew, scan
/// matching, keyframes, backend::Graph, ADR-016 keyframe store and map cache, GNSS and
/// wheel factors; docs/derivations/pipeline.md) and run single-threaded inside
/// run_once() / flush(). start() / stop() remain no-ops (threads are a later PR, ADR-005).
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
    std::uint64_t n_scans = 0;            ///< scans processed by run_once() / flush()
    std::uint64_t n_imu = 0;              ///< IMU samples processed by run_once() / flush()
    std::uint64_t n_gnss = 0;             ///< GNSS fixes processed by run_once() / flush()
    std::uint64_t n_odom = 0;             ///< wheel samples processed by run_once() / flush()
    std::uint64_t n_dropped_scans = 0;    ///< scans discarded by push_scan (queue full or too big)
    std::uint64_t n_rejected_imu = 0;     ///< samples refused by push_imu (non-increasing t)
    std::uint64_t n_dropped_imu = 0;      ///< queued IMU samples overwritten (IMU queue full)
    std::uint64_t n_dropped_aux = 0;      ///< queued GNSS + wheel messages overwritten (full)
    std::uint64_t n_keyframes = 0;        ///< keyframes added to the backend graph
    std::uint64_t n_match_failures = 0;   ///< scans whose scan matching failed (IMU fallback)
    std::uint64_t n_skipped_scans = 0;    ///< processed scans that produced no estimate
    std::uint64_t n_uncovered_scans = 0;  ///< of those: IMU does not span the scan (EOF,
                                          ///< or IMU starting after the LiDAR)
    std::uint64_t n_map_rebuilds = 0;     ///< VoxelMap rebuilds after a keyframe correction
    std::uint64_t n_gnss_factors = 0;     ///< GNSS factors added to the graph
    std::uint64_t n_odom_factors = 0;     ///< wheel-odometry factors added to the graph
    std::uint64_t n_gnss_late = 0;        ///< GNSS fixes older than the last processed scan
    std::uint64_t n_gnss_ring_overflow = 0;  ///< GNSS fixes dropped: waiting-for-IMU ring full
    std::uint64_t n_gnss_unfused_eof = 0;    ///< GNSS fixes after the last IMU sample at flush()
    std::uint64_t n_gnss_realign = 0;  ///< GNSS aligner restarts after gnss.max_consecutive_rejects
    std::uint64_t n_nonfinite_imu = 0;  ///< IMU samples dropped (non-finite acc or gyro)
    double push_scan_us_p50 = 0.0;      ///< us, median wall time of push_scan()
    double push_scan_us_p95 = 0.0;      ///< us, 95th percentile wall time of push_scan()
  };

  /// Builds every module and reserves every buffer: a pool of
  /// `realtime.scan_queue_depth + 2` ScanFrames of `realtime.max_points_per_scan` points,
  /// the IMU queue (`imu_queue_depth`) and the GNSS/wheel queues (`aux_queue_depth`).
  /// No allocation happens after the constructor on the push_* paths and on run_once /
  /// flush for non-keyframe scans (Status errors returned there use string literals, see
  /// result.hpp); keyframe scans allocate inside backend::Graph (see run_once()).
  /// Preconditions: `cfg` comes from load_config (validated).
  /// Errors: throws (initialisation may throw) std::invalid_argument from module
  ///   constructors or when a realtime.*_queue_depth is <= 0 or above its compile-time
  ///   maximum in bounded_queue.hpp (the message names the key), std::bad_alloc.
  /// Hot path: no.
  /// Tests: Task 8 test_estimator (PushScanCopiesWithoutAlloc), E-01..E-03.
  explicit Estimator(const Config& cfg);
  ~Estimator();
  Estimator(const Estimator&) = delete;
  Estimator& operator=(const Estimator&) = delete;
  /// Movable (the state lives behind one pointer allocated by the constructor); a
  /// moved-from Estimator may only be destroyed or assigned to.
  Estimator(Estimator&&) noexcept;
  Estimator& operator=(Estimator&&) noexcept;

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
  /// Hot path: yes (per scan, caller's thread): no allocation on the success path and on
  ///   error paths that use string literals (all of them here), no exception; logs at
  ///   most one warning per call.
  /// Tests: Task 8 test_estimator (PushScanCopiesWithoutAlloc, ScanQueueFullDropsOldest),
  ///   E-03.
  common::Status push_scan(const common::ScanFrame& scan);

  /// Queues one IMU sample.
  /// Errors: Status::error("imu timestamp not increasing") when `s.t <=` the previous
  ///   ACCEPTED sample's t; the sample is dropped and n_rejected_imu incremented (no
  ///   log: hot path; callers watch the counter).
  ///   When the IMU queue is full (a configuration error, ADR-005) the oldest sample is
  ///   dropped, n_dropped_imu is incremented (no log: hot path; callers watch the
  ///   counter), and the call returns Status::error("imu queue full, dropped oldest").
  /// Frames: b. Units: m/s^2, rad/s, ns.
  /// Hot path: yes (per IMU sample): no allocation on the success path and on error paths
  ///   that use string literals (all of them here), no logging, no exception.
  /// Tests: Task 8 test_estimator (ImuNonIncreasingRejected), E-03.
  common::Status push_imu(const common::ImuSample& s);

  /// Queues one GNSS fix (aux queue, depth aux_queue_depth). Queued regardless of
  /// gnss.enabled; the flag only decides whether run_once() uses the fix (it is counted
  /// in n_gnss either way).
  /// Queue full: oldest dropped, n_dropped_aux incremented (no log),
  /// Status::error("gnss queue full, dropped oldest").
  /// Frames: m.p_e_g in e. Units: m, m^2.
  /// Hot path: no (5 Hz) but allocation-free (literal error reasons). Tests: E-01, E-03.
  common::Status push_gnss(const common::GnssMeasurement& m);

  /// Queues one wheel-odometry sample (aux queue, depth aux_queue_depth). Queued
  /// regardless of wheel_odom.enabled; the flag only decides whether run_once() uses it
  /// (counted in n_odom either way).
  /// Queue full: oldest dropped, n_dropped_aux incremented (no log),
  /// Status::error("odom queue full, dropped oldest").
  /// Units: m/s, rad/s. Hot path: yes (50 Hz), allocation-free (literal error reasons).
  /// Tests: E-01, E-03.
  common::Status push_odom(const common::WheelSample& s);

  /// Copy of the most recent estimate. Until the IMU is initialised (imu.init_window_s of
  /// still data before a scan) and the first keyframe exists it is a default-constructed
  /// Estimate (state.t == Timestamp{}, identity pose, keyframe_id 0); callers detect
  /// "no estimate yet" by `state.t == Timestamp{}`.
  /// Afterwards: state.t = `t_start` of the last scan that produced an estimate; T_w_b,
  /// v_w_b and bias from the front-end state (optimised keyframe state on keyframe scans,
  /// scan-matched pose plus IMU-predicted velocity otherwise, IMU prediction when scan
  /// matching fails); cov = marginal of the keyframe or the scan-match covariance in b
  /// (unchanged from the previous estimate when scan matching fails); keyframe_id = last
  /// keyframe.
  /// Frames: state.T_w_b, v_w_b in w; cov [rho, phi] of T_w_b. Hot path: no allocation.
  /// Tests: Task 8 test_estimator (LatestStateTimestampFollowsScans), E-01.
  common::Estimate latest_state() const;

  /// Deterministic driver: on the caller's thread, drains the queues in key order until
  /// the first uncovered scan, then returns. A scan's ordering key is its end time
  /// `t_start + max(t_offset_ns)`; other messages use `t`. run_once() merges the queue
  /// heads by that key (k-way merge; ties: IMU, wheel, GNSS, scan); each queue is FIFO
  /// and assumed sorted by its producer (IMU is checked by push_imu, others are not).
  /// IMU coverage gate: a scan is processed only when an IMU sample with `t >= key` has
  /// been accepted by push_imu; otherwise run_once() returns after processing everything
  /// that precedes that scan, leaving the scan and every later message queued (so an IMU
  /// sample at exactly `key` is processed before the scan it covers). With
  /// `deterministic: true` this (plus flush()) is the only way processing happens, and
  /// two runs over the same input produce bit-identical results (E-02).
  /// Errors: none returned; per-message failures are logged and counted.
  /// Hot path: yes (per scan); allocation-free on non-keyframe scans. Keyframe scans
  ///   allocate inside backend::Graph (GTSAM iSAM2) and when a KeyframeStore slot grows.
  /// Tests: Task 8 test_estimator (RunOnceDrainsQueues, ScanWaitsForImuCoverage), E-02.
  void run_once();

  /// End of data: like run_once() but without the IMU coverage gate, so every queued
  /// message is processed in key order and all queues are empty on return. Call once
  /// when the input ends (bench at EOF, node shutdown).
  /// A scan whose end no IMU sample reaches (data ends mid-scan) is consumed (n_scans,
  /// n_skipped_scans, n_uncovered_scans) without an estimate update.
  /// Errors: none returned. Hot path: no (once); allocates only as run_once() does.
  /// Tests: Task 8 test_estimator (ScanWaitsForImuCoverage), bench component tests.
  void flush();

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

 private:
  // Queues (~0.3 MB at the compile-time maxima of bounded_queue.hpp), the ScanFrame pool
  // and the counters live in one block allocated by the constructor, so an Estimator is
  // cheap to hold on the stack of any thread.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mss::pipeline
