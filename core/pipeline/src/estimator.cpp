#include "mss/pipeline/estimator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mss/common/logger.hpp"
#include "mss/pipeline/bounded_queue.hpp"
#include "mss/pipeline/merge_order.hpp"

namespace mss::pipeline {
namespace {

// Number of push_scan() durations kept for the percentiles (see Estimator::stats()).
constexpr std::size_t kLatencySamples = 4096;

// One queued scan: the pool slot holding the points and its ordering key (end time).
struct ScanEntry {
  std::size_t slot = 0;
  common::Timestamp key{};
};

// Reads a realtime.* depth and checks it against the compile-time queue capacity.
std::size_t checked_depth(int depth, std::size_t max, const char* key) {
  if (depth <= 0 || static_cast<std::size_t>(depth) > max) {
    throw std::invalid_argument(std::string(key) + ": " + std::to_string(depth) +
                                " is outside [1, " + std::to_string(max) +
                                "] (compile-time maximum in mss/pipeline/bounded_queue.hpp)");
  }
  return static_cast<std::size_t>(depth);
}

// Nearest-rank percentile on sorted[0, n): sorted[ceil(q * n) - 1] with q = num / den,
// computed in integers. Precondition: n > 0.
double nearest_rank(const std::array<double, kLatencySamples>& sorted,
                    std::size_t n,
                    std::size_t num,
                    std::size_t den) {
  const std::size_t rank = (num * n + den - 1) / den;  // ceil(q * n), >= 1 for n >= 1
  return sorted[rank - 1];
}

}  // namespace

// Owner: the Estimator; single-threaded in this phase (no mutex, see ADR-005 TODO in
// bounded_queue.hpp).
struct Estimator::Impl {
  explicit Impl(const Config& c)
      : cfg(c),
        scan_depth(
            checked_depth(c.realtime.scan_queue_depth, kMaxScanQueue, "realtime.scan_queue_depth")),
        imu_depth(
            checked_depth(c.realtime.imu_queue_depth, kMaxImuQueue, "realtime.imu_queue_depth")),
        aux_depth(
            checked_depth(c.realtime.aux_queue_depth, kMaxAuxQueue, "realtime.aux_queue_depth")) {
    if (c.realtime.max_points_per_scan == 0) {
      throw std::invalid_argument("realtime.max_points_per_scan: must be > 0");
    }
    // Pool: every queued scan plus one being processed plus one spare.
    pool.resize(scan_depth + 2);
    for (common::ScanFrame& f : pool) {
      f.reserve(c.realtime.max_points_per_scan);
    }
    slot_busy.assign(pool.size(), false);
  }

  // Returns a free pool slot. There is always one: at most scan_depth slots are queued
  // and the pool has scan_depth + 2.
  std::size_t acquire_slot() {
    for (std::size_t i = 0; i < slot_busy.size(); ++i) {
      if (!slot_busy[i]) {
        slot_busy[i] = true;
        return i;
      }
    }
    return slot_busy.size();  // unreachable by construction; checked by the caller
  }
  void release_slot(std::size_t slot) { slot_busy[slot] = false; }

  void record_push_latency(double us) {
    latency_us[latency_next] = us;
    latency_next = (latency_next + 1) % kLatencySamples;
    latency_count = std::min(latency_count + 1, kLatencySamples);
  }

  Config cfg;
  std::size_t scan_depth;
  std::size_t imu_depth;
  std::size_t aux_depth;

  std::vector<common::ScanFrame> pool;
  std::vector<bool> slot_busy;

  BoundedQueue<ScanEntry, kMaxScanQueue> scans;
  BoundedQueue<common::ImuSample, kMaxImuQueue> imu;
  BoundedQueue<common::GnssMeasurement, kMaxAuxQueue> gnss;
  BoundedQueue<common::WheelSample, kMaxAuxQueue> odom;

  bool have_imu = false;
  common::Timestamp last_imu_t{};

  common::Estimate latest;
  Stats stats;

  // Ring of the most recent push_scan() wall times in microseconds.
  std::array<double, kLatencySamples> latency_us{};
  std::size_t latency_next = 0;
  std::size_t latency_count = 0;
};

Estimator::Estimator(const Config& cfg) : impl_(std::make_unique<Impl>(cfg)) {}
Estimator::~Estimator() = default;
Estimator::Estimator(Estimator&&) noexcept = default;
Estimator& Estimator::operator=(Estimator&&) noexcept = default;

common::Status Estimator::push_scan(const common::ScanFrame& scan) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point t0 = Clock::now();
  Impl& s = *impl_;
  common::Status status = common::Status::ok();
  if (scan.size() > s.cfg.realtime.max_points_per_scan) {
    ++s.stats.n_dropped_scans;
    common::log_warn("push_scan: scan larger than max_points_per_scan, dropped");
    status = common::Status::error("scan larger than max_points_per_scan");
  } else {
    if (s.scans.size() >= s.scan_depth) {
      ScanEntry oldest;
      s.scans.pop(oldest);
      s.release_slot(oldest.slot);
      ++s.stats.n_dropped_scans;
      common::log_warn("push_scan: scan queue full, dropped oldest");
      status = common::Status::error("scan queue full, dropped oldest");
    }
    const std::size_t slot = s.acquire_slot();
    if (slot < s.pool.size() && s.pool[slot].copy_from(scan)) {
      s.scans.push(ScanEntry{slot, common::scan_end_time(scan)});
    } else {
      // Not reachable: the pool always has a free slot and capacity was checked above.
      if (slot < s.pool.size()) {
        s.release_slot(slot);
      }
      ++s.stats.n_dropped_scans;
      common::log_error("push_scan: no free scan slot, dropped");
      status = common::Status::error("no free scan slot");
    }
  }
  const std::chrono::duration<double, std::micro> dt = Clock::now() - t0;
  s.record_push_latency(dt.count());
  return status;
}

common::Status Estimator::push_imu(const common::ImuSample& sample) {
  Impl& s = *impl_;
  if (s.have_imu && sample.t <= s.last_imu_t) {
    ++s.stats.n_rejected_imu;
    common::log_warn("push_imu: imu timestamp not increasing, sample dropped");
    return common::Status::error("imu timestamp not increasing");
  }
  s.have_imu = true;
  s.last_imu_t = sample.t;
  if (s.imu.size() >= s.imu_depth) {
    common::ImuSample dropped;
    s.imu.pop(dropped);
    s.imu.push(sample);
    ++s.stats.n_dropped_imu;  // counter only: no log on the per-sample hot path
    return common::Status::error("imu queue full, dropped oldest");
  }
  s.imu.push(sample);
  return common::Status::ok();
}

common::Status Estimator::push_gnss(const common::GnssMeasurement& m) {
  Impl& s = *impl_;
  if (s.gnss.size() >= s.aux_depth) {
    common::GnssMeasurement dropped;
    s.gnss.pop(dropped);
    s.gnss.push(m);
    ++s.stats.n_dropped_aux;  // counter only: no log on the hot path
    return common::Status::error("gnss queue full, dropped oldest");
  }
  s.gnss.push(m);
  return common::Status::ok();
}

common::Status Estimator::push_odom(const common::WheelSample& sample) {
  Impl& s = *impl_;
  if (s.odom.size() >= s.aux_depth) {
    common::WheelSample dropped;
    s.odom.pop(dropped);
    s.odom.push(sample);
    ++s.stats.n_dropped_aux;  // counter only: no log on the hot path
    return common::Status::error("odom queue full, dropped oldest");
  }
  s.odom.push(sample);
  return common::Status::ok();
}

common::Estimate Estimator::latest_state() const {
  return impl_->latest;
}

void Estimator::run_once() {
  Impl& s = *impl_;
  for (;;) {
    std::array<QueueHead, kNumSources> heads{};
    heads[static_cast<std::size_t>(Source::kImu)] = {
        !s.imu.empty(), s.imu.empty() ? common::Timestamp{} : s.imu.front().t};
    heads[static_cast<std::size_t>(Source::kWheel)] = {
        !s.odom.empty(), s.odom.empty() ? common::Timestamp{} : s.odom.front().t};
    heads[static_cast<std::size_t>(Source::kGnss)] = {
        !s.gnss.empty(), s.gnss.empty() ? common::Timestamp{} : s.gnss.front().t};
    heads[static_cast<std::size_t>(Source::kScan)] = {
        !s.scans.empty(), s.scans.empty() ? common::Timestamp{} : s.scans.front().key};
    switch (select_next(heads)) {
      case Source::kImu: {
        common::ImuSample sample;
        s.imu.pop(sample);
        ++s.stats.n_imu;  // no IMU module yet: processing = counting
        break;
      }
      case Source::kWheel: {
        common::WheelSample sample;
        s.odom.pop(sample);
        ++s.stats.n_odom;
        break;
      }
      case Source::kGnss: {
        common::GnssMeasurement m;
        s.gnss.pop(m);
        ++s.stats.n_gnss;
        break;
      }
      case Source::kScan: {
        ScanEntry entry;
        s.scans.pop(entry);
        ++s.stats.n_scans;
        // No algorithm yet (Task 8): identity pose, zero velocity and bias, zero cov,
        // stamped with the scan start time (the keyframe time).
        s.latest = common::Estimate{};
        s.latest.state.t = s.pool[entry.slot].t_start;
        s.release_slot(entry.slot);
        break;
      }
      case Source::kNone:
        return;
    }
  }
}

void Estimator::start() {
  if (impl_->cfg.deterministic) {
    common::log_warn("Estimator::start: deterministic mode has no threads; call run_once()");
  } else {
    common::log_warn(
        "Estimator::start: threaded mode is not implemented yet (ADR-005); call run_once()");
  }
}

void Estimator::stop() {
  if (impl_->cfg.deterministic) {
    common::log_warn("Estimator::stop: deterministic mode has no threads; nothing to stop");
  } else {
    common::log_warn("Estimator::stop: threaded mode is not implemented yet (ADR-005)");
  }
}

Estimator::Stats Estimator::stats() const {
  const Impl& s = *impl_;
  Stats out = s.stats;
  const std::size_t n = s.latency_count;
  if (n > 0) {
    // Off the hot path: sort a copy of the stored samples.
    std::array<double, kLatencySamples> sorted = s.latency_us;
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(n));
    out.push_scan_us_p50 = nearest_rank(sorted, n, 50, 100);
    out.push_scan_us_p95 = nearest_rank(sorted, n, 95, 100);
  }
  return out;
}

}  // namespace mss::pipeline
