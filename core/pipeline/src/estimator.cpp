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

#include "estimator_core.hpp"

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
    core = std::make_unique<detail::EstimatorCore>(c);
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

  // A scan with ordering key `key` is covered once an accepted IMU sample has t >= key
  // (push_imu keeps last_imu_t strictly increasing, so the latest accepted t decides).
  // Only finite samples count: the core drops non-finite ones, so they cannot cover a scan.
  bool imu_covers(common::Timestamp key) const {
    return have_finite_imu && key <= last_finite_imu_t;
  }

  // k-way merge of the queue heads by key (see Estimator::run_once()). With
  // `require_imu_coverage`, stops at the first scan in key order that no IMU sample covers
  // yet, leaving it and everything after it queued.
  void drain(bool require_imu_coverage);

  void feed_imu(const common::ImuSample& s) {
    core->on_imu(s);
    if (s.acc_b.allFinite() && s.gyro_b.allFinite()) {  // the core drops the others
      core_has_imu = true;
      core_last_imu_t = s.t;  // push_imu keeps the queue strictly increasing
    }
  }

  void record_push_latency(double us) {
    latency_us[latency_next] = us;
    latency_next = (latency_next + 1) % kLatencySamples;
    latency_count = std::min(latency_count + 1, kLatencySamples);
  }

  Config cfg;
  std::size_t scan_depth;
  std::size_t imu_depth;
  std::size_t aux_depth;

  // Every module and the per-message algorithm (docs/derivations/pipeline.md).
  std::unique_ptr<detail::EstimatorCore> core;

  std::vector<common::ScanFrame> pool;
  std::vector<bool> slot_busy;

  BoundedQueue<ScanEntry, kMaxScanQueue> scans;
  BoundedQueue<common::ImuSample, kMaxImuQueue> imu;
  BoundedQueue<common::GnssMeasurement, kMaxAuxQueue> gnss;
  BoundedQueue<common::WheelSample, kMaxAuxQueue> odom;

  bool have_imu = false;
  common::Timestamp last_imu_t{};
  // Latest accepted IMU sample with finite acc and gyro (coverage gate).
  bool have_finite_imu = false;
  common::Timestamp last_finite_imu_t{};
  // Latest IMU sample handed to the core (drain).
  bool core_has_imu = false;
  common::Timestamp core_last_imu_t{};

  common::Estimate latest;
  Stats stats;

  // Ring of the most recent push_scan() wall times in microseconds.
  std::array<double, kLatencySamples> latency_us{};
  std::size_t latency_next = 0;
  std::size_t latency_count = 0;
};

void Estimator::Impl::drain(bool require_imu_coverage) {
  for (;;) {
    std::array<QueueHead, kNumSources> heads{};
    heads[static_cast<std::size_t>(Source::kImu)] = {
        !imu.empty(), imu.empty() ? common::Timestamp{} : imu.front().t};
    heads[static_cast<std::size_t>(Source::kWheel)] = {
        !odom.empty(), odom.empty() ? common::Timestamp{} : odom.front().t};
    heads[static_cast<std::size_t>(Source::kGnss)] = {
        !gnss.empty(), gnss.empty() ? common::Timestamp{} : gnss.front().t};
    heads[static_cast<std::size_t>(Source::kScan)] = {
        !scans.empty(), scans.empty() ? common::Timestamp{} : scans.front().key};
    switch (select_next(heads)) {
      case Source::kImu: {
        common::ImuSample sample;
        imu.pop(sample);
        ++stats.n_imu;
        feed_imu(sample);
        break;
      }
      case Source::kWheel: {
        common::WheelSample sample;
        odom.pop(sample);
        ++stats.n_odom;
        core->on_wheel(sample);
        break;
      }
      case Source::kGnss: {
        common::GnssMeasurement m;
        gnss.pop(m);
        ++stats.n_gnss;
        core->on_gnss(m);
        break;
      }
      case Source::kScan: {
        if (require_imu_coverage && !imu_covers(scans.front().key)) {
          return;  // wait for IMU up to the scan end; later messages stay queued too
        }
        // The core needs the IMU up to the first sample at or after the scan end (deskew,
        // docs/derivations/pipeline.md section 2). That sample sorts after the scan, so it
        // is handed over now; only IMU samples move ahead of the scan, never wheel or GNSS.
        while (!imu.empty() && !(core_has_imu && scans.front().key <= core_last_imu_t)) {
          common::ImuSample sample;
          imu.pop(sample);
          ++stats.n_imu;
          feed_imu(sample);
        }
        ScanEntry entry;
        scans.pop(entry);
        ++stats.n_scans;
        core->on_scan(pool[entry.slot]);
        if (core->has_estimate()) {
          latest = core->latest();
        }
        release_slot(entry.slot);
        break;
      }
      case Source::kNone:
        return;
    }
  }
}

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
    ++s.stats.n_rejected_imu;  // counter only: no log on the per-sample hot path
    return common::Status::error("imu timestamp not increasing");
  }
  s.have_imu = true;
  s.last_imu_t = sample.t;
  if (sample.acc_b.allFinite() && sample.gyro_b.allFinite()) {
    s.have_finite_imu = true;
    s.last_finite_imu_t = sample.t;
  }
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
  impl_->drain(/*require_imu_coverage=*/true);
}

void Estimator::flush() {
  impl_->drain(/*require_imu_coverage=*/false);
  impl_->core->on_end_of_data();
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
  const detail::CoreCounters& c = s.core->counters();
  out.n_keyframes = c.n_keyframes;
  out.n_match_failures = c.n_match_failures;
  out.n_skipped_scans = c.n_skipped_scans;
  out.n_uncovered_scans = c.n_uncovered_scans;
  out.n_map_rebuilds = c.n_map_rebuilds;
  out.n_gnss_factors = c.n_gnss_factors;
  out.n_odom_factors = c.n_odom_factors;
  out.n_gnss_late = c.n_gnss_late;
  out.n_gnss_ring_overflow = c.n_gnss_ring_overflow;
  out.n_gnss_unfused_eof = c.n_gnss_unfused_eof;
  out.n_nonfinite_imu = c.n_nonfinite_imu;
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
