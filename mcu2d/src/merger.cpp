#include "mss2d/merger.hpp"

namespace mss2d {
namespace {

// Tie-break order: on equal keys the lower value goes first (as pipeline::Source).
enum Source : int { kImu = 0, kWheel = 1, kScan = 2, kNone = 3 };

// Single writer (the producer), so a plain load + store is enough.
void bump(std::atomic<std::uint32_t>& c) {
  c.store(c.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
}

}  // namespace

Merger::Merger(const Config& cfg)
    : max_scan_points_(cfg.max_scan_points),
      imu_(cfg.imu_ring_depth),
      wheel_(cfg.wheel_ring_depth),
      scan_(cfg.scan_ring_depth) {}

Status Merger::push_imu(const ImuSample2d& s) {
  if (have_push_imu_ && s.t <= last_push_imu_) {
    bump(ooo_imu_);
    return Status::error("imu: non-increasing timestamp, dropped");
  }
  if (!imu_.push(s)) {
    return Status::error("imu ring full, dropped");
  }
  have_push_imu_ = true;
  last_push_imu_ = s.t;
  return Status::ok();
}

Status Merger::push_wheel(const WheelSample& s) {
  if (have_push_wheel_ && s.t <= last_push_wheel_) {
    bump(ooo_wheel_);
    return Status::error("wheel: non-increasing timestamp, dropped");
  }
  if (!wheel_.push(s)) {
    return Status::error("wheel ring full, dropped");
  }
  have_push_wheel_ = true;
  last_push_wheel_ = s.t;
  return Status::ok();
}

Status Merger::push_scan(const Scan2d& s) {
  if (s.count > max_scan_points_) {
    return Status::error("scan: more points than max_scan_points, dropped");
  }
  const Timestamp key = s.key();
  if (have_push_scan_ && key <= last_push_scan_) {
    bump(ooo_scan_);
    return Status::error("scan: non-increasing key, dropped");
  }
  if (!scan_.push(s)) {
    return Status::error("scan ring full, dropped");
  }
  have_push_scan_ = true;
  last_push_scan_ = key;
  return Status::ok();
}

// A scan with key `key` is covered once an accepted IMU sample has t >= key. IMU t is
// strictly increasing, so the newest one (ring tail, else the last delivered) decides.
bool Merger::imu_covers(Timestamp key) const {
  const ImuSample2d* newest = imu_.back();
  if (newest != nullptr) {
    return key <= newest->t;
  }
  return have_delivered_imu_ && key <= last_delivered_imu_;
}

void Merger::deliver_imu(Sink& sink) {
  const ImuSample2d* s = imu_.front();
  sink.on_imu(*s);
  have_delivered_imu_ = true;
  last_delivered_imu_ = s->t;
  ++delivered_.n_imu;
  imu_.pop();
}

std::size_t Merger::drain(Sink& sink, bool require_imu_coverage) {
  std::size_t n = 0;
  for (;;) {
    // k-way merge of the ring heads by key; strictly smaller wins, so a tie keeps the
    // source earlier in tie-break order.
    const ImuSample2d* hi = imu_.front();
    const WheelSample* hw = wheel_.front();
    const Scan2d* hs = scan_.front();
    int best = kNone;
    Timestamp best_key{};
    Timestamp scan_key{};
    if (hi != nullptr) {
      best = kImu;
      best_key = hi->t;
    }
    if (hw != nullptr && (best == kNone || hw->t < best_key)) {
      best = kWheel;
      best_key = hw->t;
    }
    if (hs != nullptr) {
      scan_key = hs->key();  // one pass over <= kMaxScanPoints points
      if (best == kNone || scan_key < best_key) {
        best = kScan;
        best_key = scan_key;
      }
    }
    switch (best) {
      case kImu:
        deliver_imu(sink);
        ++n;
        break;
      case kWheel:
        sink.on_wheel(*hw);
        ++delivered_.n_wheel;
        wheel_.pop();
        ++n;
        break;
      case kScan:
        if (require_imu_coverage && !imu_covers(scan_key)) {
          return n;  // wait for IMU up to the scan end; later messages stay queued too
        }
        // Hand over IMU up to the first sample at or after the scan end (it sorts after
        // the scan); only IMU moves ahead of the scan, never wheel.
        while (imu_.front() != nullptr &&
               !(have_delivered_imu_ && scan_key <= last_delivered_imu_)) {
          deliver_imu(sink);
          ++n;
        }
        sink.on_scan(*hs);
        ++delivered_.n_scans;
        scan_.pop();
        ++n;
        break;
      default:
        return n;
    }
  }
}

std::size_t Merger::run_once(Sink& sink) {
  return drain(sink, /*require_imu_coverage=*/true);
}

std::size_t Merger::flush(Sink& sink) {
  return drain(sink, /*require_imu_coverage=*/false);
}

MergerStats Merger::stats() const {
  MergerStats s = delivered_;
  s.n_dropped_imu = imu_.dropped();
  s.n_dropped_wheel = wheel_.dropped();
  s.n_dropped_scan = scan_.dropped();
  s.n_out_of_order_imu = ooo_imu_.load(std::memory_order_relaxed);
  s.n_out_of_order_wheel = ooo_wheel_.load(std::memory_order_relaxed);
  s.n_out_of_order_scan = ooo_scan_.load(std::memory_order_relaxed);
  return s;
}

}  // namespace mss2d
