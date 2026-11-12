#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/isr_ring.hpp"
#include "mss2d/messages.hpp"
#include "mss2d/status.hpp"

namespace mss2d {

/// Receives messages in merge order. Arguments are valid only during the call.
class Sink {
 public:
  virtual void on_imu(const ImuSample2d& s) = 0;
  virtual void on_wheel(const WheelSample& s) = 0;
  virtual void on_scan(const Scan2d& s) = 0;

 protected:
  ~Sink() = default;
};

/// Counters; drop and out-of-order counts mirror pipeline::Estimator::Stats where meaningful.
struct MergerStats {
  std::uint32_t n_imu = 0;  ///< delivered to the sink
  std::uint32_t n_wheel = 0;
  std::uint32_t n_scans = 0;
  std::uint32_t n_dropped_imu = 0;  ///< ring full, new message rejected
  std::uint32_t n_dropped_wheel = 0;
  std::uint32_t n_dropped_scan = 0;
  std::uint32_t n_out_of_order_imu = 0;  ///< non-increasing t (scan: key), rejected
  std::uint32_t n_out_of_order_wheel = 0;
  std::uint32_t n_out_of_order_scan = 0;
};

/// Owns the three rings and merges them by timestamp with the rules of
/// pipeline::Estimator::run_once()/flush() (ADR-005, ADR-033):
///  - k-way merge of the ring heads by key: `t` for IMU and wheel, the scan end time
///    `t_start + max(t_offset_ns)` for a scan; ties go IMU, then wheel, then scan.
///  - IMU coverage gate: run_once() delivers a scan only once an IMU sample with
///    t >= key has been accepted; otherwise it stops there, leaving that scan and every
///    later message queued, so calling it early never reorders anything.
///  - flush() ignores the gate (end of data).
///  - before a scan, IMU samples are delivered up to the first one at or after the scan
///    end (the deskew needs it); wheel samples never move ahead of a scan.
///
/// Threading: push_*() belong to the producer (ISR), run_once()/flush()/stats() to the main
/// loop. A rejected message returns a Status (string literal reason) and bumps a counter.
/// No allocation after construction. The object is large (about 26 KB): give it static
/// storage.
class Merger {
 public:
  /// `cfg` must have passed Config::validate().
  explicit Merger(const Config& cfg);

  // Producer side. Non-increasing time (scan: key) is rejected as out of order.
  Status push_imu(const ImuSample2d& s);
  Status push_wheel(const WheelSample& s);
  Status push_scan(const Scan2d& s);  ///< count > max_scan_points is rejected

  // Consumer side. Return the number of messages delivered.
  std::size_t run_once(Sink& sink);
  std::size_t flush(Sink& sink);

  MergerStats stats() const;

 private:
  std::size_t drain(Sink& sink, bool require_imu_coverage);
  void deliver_imu(Sink& sink);
  bool imu_covers(Timestamp key) const;

  std::size_t max_scan_points_;
  IsrRing<ImuSample2d, kImuRingCapacity> imu_;
  IsrRing<WheelSample, kWheelRingCapacity> wheel_;
  IsrRing<Scan2d, kScanRingCapacity> scan_;

  // Producer-owned ordering state.
  bool have_push_imu_ = false;
  bool have_push_wheel_ = false;
  bool have_push_scan_ = false;
  Timestamp last_push_imu_{};
  Timestamp last_push_wheel_{};
  Timestamp last_push_scan_{};
  // Producer-written, consumer-read (single writer, relaxed).
  std::atomic<std::uint32_t> ooo_imu_{0};
  std::atomic<std::uint32_t> ooo_wheel_{0};
  std::atomic<std::uint32_t> ooo_scan_{0};

  // Consumer-owned.
  bool have_delivered_imu_ = false;
  Timestamp last_delivered_imu_{};
  MergerStats delivered_{};
};

}  // namespace mss2d
