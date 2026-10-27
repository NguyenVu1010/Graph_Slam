// Latency benchmark and dataset replay. Runs on Pi5 without ROS.
//
// Usage: mss_bench <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR
//
// Replays an mssdata dataset (ADR-014) in time order into pipeline::Estimator: IMU, GNSS
// and wheel records go to push_imu/push_gnss/push_odom, scans to push_scan; ground-truth
// records are counted and ignored. run_once() is called by the Replayer below (before and
// after each scan push and on queue pressure); flush() at the end of the dataset. Writes
// only under DIR:
//   DIR/est.tum     one line per scan that produced an estimate, from latest_state(); none
//                   before the IMU static initialisation (imu.init_window_s of still data)
//   DIR/bench.json  Estimator::Stats, replay wall time, record counts per stream and the
//                   per-scan processing latency (scan_process_ms_p50/p95, PF-01, from
//                   n_latency_samples samples; n_latency_dropped did not fit the buffer)
// and prints a summary table on stdout.
// Exit codes: 0 success; 2 usage, config, dataset, estimator-init or output error (reason
// on stderr); 3 the replay finished but some records were not processed (dropped or
// rejected: records.* != n_*): one warning line on stderr, outputs still written.
// --out is required so the tool never writes into the source tree by default.
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_format.hpp"
#include "mss/io/dataset_reader.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"

namespace {

namespace fs = std::filesystem;

constexpr int kExitError = 2;       // usage, config, dataset or output error
constexpr int kExitIncomplete = 3;  // replay done, some records not processed

struct Args {
  fs::path config;
  fs::path dataset;
  std::optional<fs::path> overlay;
  fs::path out;
};

void usage(const char* argv0) {
  std::fprintf(
      stderr, "usage: %s <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR\n", argv0);
}

std::optional<Args> parse_args(int argc, char** argv) {
  Args a;
  int positional = 0;
  bool have_out = false;
  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];
    if (std::strcmp(arg, "--overlay") == 0 || std::strcmp(arg, "--out") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "mss_bench: %s needs a value\n", arg);
        return std::nullopt;
      }
      if (std::strcmp(arg, "--overlay") == 0) {
        a.overlay = fs::path(argv[++i]);
      } else {
        a.out = fs::path(argv[++i]);
        have_out = true;
      }
    } else if (std::strncmp(arg, "--", 2) == 0) {
      std::fprintf(stderr, "mss_bench: unknown option %s\n", arg);
      return std::nullopt;
    } else if (positional == 0) {
      a.config = fs::path(arg);
      ++positional;
    } else if (positional == 1) {
      a.dataset = fs::path(arg);
      ++positional;
    } else {
      std::fprintf(stderr, "mss_bench: unexpected argument %s\n", arg);
      return std::nullopt;
    }
  }
  if (positional != 2 || !have_out) {
    if (!have_out) {
      std::fprintf(stderr, "mss_bench: --out DIR is required\n");
    }
    return std::nullopt;
  }
  return a;
}

// Records read per stream.
struct Counts {
  std::uint64_t imu = 0;
  std::uint64_t gnss = 0;
  std::uint64_t odom = 0;
  std::uint64_t lidar = 0;
  std::uint64_t gt = 0;
};

// Per-scan processing latency percentiles, ms (PF-01).
struct ScanLatency {
  double p50_ms = 0.0;
  double p95_ms = 0.0;
  std::uint64_t n_samples = 0;  // samples behind the percentiles
  std::uint64_t n_dropped = 0;  // samples discarded past the reserved capacity
};

// Nearest-rank percentile on a sorted, non-empty vector: sorted[ceil(q * n) - 1] with
// q = num / den, computed in integers as in Estimator::stats().
double nearest_rank(const std::vector<double>& sorted, std::size_t num, std::size_t den) {
  const std::size_t rank = (num * sorted.size() + den - 1) / den;  // >= 1 for n >= 1
  return sorted[rank - 1];
}

// One TUM line: "t x y z qx qy qz qw" with t in seconds printed from integer ns (9 decimals)
// and the pose with 17 significant digits, so equal files mean bit-identical doubles (E-02).
void write_tum_line(std::FILE* f, const mss::common::NavState& s) {
  const std::int64_t ns = s.t.ns;
  const bool neg = ns < 0;
  const std::uint64_t mag =
      neg ? static_cast<std::uint64_t>(-(ns + 1)) + 1U : static_cast<std::uint64_t>(ns);
  const std::uint64_t sec = mag / static_cast<std::uint64_t>(mss::common::kNanosecondsPerSecond);
  const std::uint64_t nsec = mag % static_cast<std::uint64_t>(mss::common::kNanosecondsPerSecond);
  // Hamilton quaternion of R_w_b; Eigen stores (x, y, z, w), TUM writes (qx qy qz qw).
  const Eigen::Quaterniond q(s.T_w_b.R);
  std::fprintf(f,
               "%s%" PRIu64 ".%09" PRIu64 " %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
               neg ? "-" : "",
               sec,
               nsec,
               s.T_w_b.p.x(),
               s.T_w_b.p.y(),
               s.T_w_b.p.z(),
               q.x(),
               q.y(),
               q.z(),
               q.w());
}

bool write_json(const fs::path& path,
                const mss::pipeline::Estimator::Stats& st,
                double replay_wall_s,
                const Counts& c,
                const ScanLatency& lat) {
  std::FILE* f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    return false;
  }
  std::fprintf(f,
               "{\n"
               "  \"n_scans\": %" PRIu64
               ",\n"
               "  \"n_imu\": %" PRIu64
               ",\n"
               "  \"n_gnss\": %" PRIu64
               ",\n"
               "  \"n_odom\": %" PRIu64
               ",\n"
               "  \"n_dropped_scans\": %" PRIu64
               ",\n"
               "  \"n_rejected_imu\": %" PRIu64
               ",\n"
               "  \"n_dropped_imu\": %" PRIu64
               ",\n"
               "  \"n_dropped_aux\": %" PRIu64
               ",\n"
               "  \"n_keyframes\": %" PRIu64
               ",\n"
               "  \"n_match_failures\": %" PRIu64
               ",\n"
               "  \"n_skipped_scans\": %" PRIu64
               ",\n"
               "  \"n_uncovered_scans\": %" PRIu64
               ",\n"
               "  \"n_map_rebuilds\": %" PRIu64
               ",\n"
               "  \"n_gnss_factors\": %" PRIu64
               ",\n"
               "  \"n_odom_factors\": %" PRIu64
               ",\n"
               "  \"n_gnss_late\": %" PRIu64
               ",\n"
               "  \"n_gnss_ring_overflow\": %" PRIu64
               ",\n"
               "  \"n_gnss_unfused_eof\": %" PRIu64
               ",\n"
               "  \"n_gnss_realign\": %" PRIu64
               ",\n"
               "  \"n_nonfinite_imu\": %" PRIu64
               ",\n"
               "  \"push_scan_us_p50\": %.3f,\n"
               "  \"push_scan_us_p95\": %.3f,\n"
               "  \"scan_process_ms_p50\": %.6f,\n"
               "  \"scan_process_ms_p95\": %.6f,\n"
               "  \"n_latency_samples\": %" PRIu64
               ",\n"
               "  \"n_latency_dropped\": %" PRIu64
               ",\n"
               "  \"replay_wall_s\": %.6f,\n"
               "  \"records\": {\n"
               "    \"imu\": %" PRIu64
               ",\n"
               "    \"gnss\": %" PRIu64
               ",\n"
               "    \"odom\": %" PRIu64
               ",\n"
               "    \"lidar\": %" PRIu64
               ",\n"
               "    \"gt\": %" PRIu64
               "\n"
               "  }\n"
               "}\n",
               st.n_scans,
               st.n_imu,
               st.n_gnss,
               st.n_odom,
               st.n_dropped_scans,
               st.n_rejected_imu,
               st.n_dropped_imu,
               st.n_dropped_aux,
               st.n_keyframes,
               st.n_match_failures,
               st.n_skipped_scans,
               st.n_uncovered_scans,
               st.n_map_rebuilds,
               st.n_gnss_factors,
               st.n_odom_factors,
               st.n_gnss_late,
               st.n_gnss_ring_overflow,
               st.n_gnss_unfused_eof,
               st.n_gnss_realign,
               st.n_nonfinite_imu,
               st.push_scan_us_p50,
               st.push_scan_us_p95,
               lat.p50_ms,
               lat.p95_ms,
               lat.n_samples,
               lat.n_dropped,
               replay_wall_s,
               c.imu,
               c.gnss,
               c.odom,
               c.lidar,
               c.gt);
  return std::fclose(f) == 0;
}

void print_table(const fs::path& dataset,
                 const mss::pipeline::Estimator::Stats& st,
                 double replay_wall_s,
                 const Counts& c,
                 const ScanLatency& lat) {
  std::printf("mss_bench: %s\n", dataset.c_str());
  std::printf("%-8s %10s %10s\n", "stream", "records", "processed");
  std::printf("%-8s %10" PRIu64 " %10" PRIu64 "\n", "imu", c.imu, st.n_imu);
  std::printf("%-8s %10" PRIu64 " %10" PRIu64 "\n", "gnss", c.gnss, st.n_gnss);
  std::printf("%-8s %10" PRIu64 " %10" PRIu64 "\n", "odom", c.odom, st.n_odom);
  std::printf("%-8s %10" PRIu64 " %10" PRIu64 "\n", "lidar", c.lidar, st.n_scans);
  std::printf("%-8s %10" PRIu64 " %10s\n", "gt", c.gt, "ignored");
  std::printf("dropped scans %" PRIu64 ", dropped imu %" PRIu64 ", dropped aux %" PRIu64
              ", rejected imu %" PRIu64 "\n",
              st.n_dropped_scans,
              st.n_dropped_imu,
              st.n_dropped_aux,
              st.n_rejected_imu);
  std::printf("push_scan p50 %.3f us, p95 %.3f us\n", st.push_scan_us_p50, st.push_scan_us_p95);
  std::printf("scan processing p50 %.3f ms, p95 %.3f ms\n", lat.p50_ms, lat.p95_ms);
  std::printf("replay wall time %.3f s\n", replay_wall_s);
}

// Feeds the merged record stream into the Estimator and decides when to call run_once().
// Deterministic: the decisions depend only on the data and the realtime.* depths.
//
// Draining early is always safe: the Estimator gates every scan on IMU coverage (it
// processes a scan only once an IMU sample with t >= the scan end time has been queued,
// and leaves the scan and everything after it queued until then; see
// Estimator::run_once()). So the replayer needs no watermark and no knowledge of scan
// end times; it only has to call run_once() often enough that no queue overflows:
//   - before every scan push: the previous scan is processed as soon as the IMU sample
//     covering its end has been pushed, which may come after the next scan's record (an
//     IMU stamp offset); so the scan queue never holds more than the scan still waiting
//     for that sample plus the new one, and scan_queue_depth >= 2 never drops a scan;
//   - after every scan push (processes the new scan when it is already covered);
//   - before a push that would fill a queue to its configured depth: the replayer counts
//     pushes per queue since the last run_once() and drains when the count reaches
//     `depth - 1`, so sensors that lead the LiDAR (or a long scan gap) are not dropped.
//     An uncovered scan only holds back messages with a key after its end time, i.e.
//     at most about one IMU period of GNSS / wheel data while the IMU stream is live.
// flush() at the end processes whatever is left, covered or not.
//
// PF-01 latency: every run_once()/flush() call is timed (steady_clock); a call that raised
// Stats::n_scans adds one sample per processed scan, its wall time divided by the number of
// scans. Estimator::stats() (which sorts its own latency buffer) is read only while a
// successfully pushed scan is still unprocessed, so it adds no work to the other calls. The
// sample buffer is reserved once for the dataset's scan count, so the replay loop does not
// allocate; samples past it are counted, not stored. Diagnostics only: the estimate does
// not depend on it.
class Replayer {
 public:
  Replayer(mss::pipeline::Estimator& est,
           const mss::pipeline::RealtimeConfig& rt,
           std::FILE* tum,
           std::size_t n_scans_expected)
      : est_(est),
        tum_(tum),
        imu_limit_(static_cast<std::size_t>(rt.imu_queue_depth) - 1U),
        aux_limit_(static_cast<std::size_t>(rt.aux_queue_depth) - 1U) {
    scan_ms_.reserve(n_scans_expected);
  }

  void feed(const mss::io::Record& rec, const mss::common::ScanFrame& scan) {
    using mss::io::RecordKind;
    // Push failures (drops, rejections) are counted by the estimator.
    switch (rec.kind) {
      case RecordKind::kImu:
        ++counts_.imu;
        drain_if(n_imu_ >= imu_limit_);
        (void)est_.push_imu(rec.imu);
        ++n_imu_;
        break;
      case RecordKind::kGnss:
        ++counts_.gnss;
        drain_if(n_gnss_ >= aux_limit_);
        (void)est_.push_gnss(rec.gnss);
        ++n_gnss_;
        break;
      case RecordKind::kOdom:
        ++counts_.odom;
        drain_if(n_odom_ >= aux_limit_);
        (void)est_.push_odom(rec.odom);
        ++n_odom_;
        break;
      case RecordKind::kLidar:
        ++counts_.lidar;
        drain_if(true);
        if (est_.push_scan(scan).is_ok()) {
          ++n_pushed_scans_;  // a dropped scan is never processed, so it is not pending
        }
        drain_if(true);
        break;
      case RecordKind::kGt:
        ++counts_.gt;
        break;
    }
  }

  // Processes what is left. Call once at the end of the dataset.
  void finish() {
    timed(true);
    write_latest();
  }

  const Counts& counts() const { return counts_; }

  // Nearest-rank p50 / p95 of the per-scan samples; zeros when no scan was processed.
  [[nodiscard]] ScanLatency latency() const {
    ScanLatency out;
    out.n_samples = scan_ms_.size();
    out.n_dropped = n_latency_dropped_;
    if (scan_ms_.empty()) {
      return out;
    }
    std::vector<double> sorted = scan_ms_;  // after the replay: allocating here is fine
    std::sort(sorted.begin(), sorted.end());
    out.p50_ms = nearest_rank(sorted, 50, 100);
    out.p95_ms = nearest_rank(sorted, 95, 100);
    return out;
  }

 private:
  void drain_if(bool pressure) {
    if (!pressure) {
      return;
    }
    timed(false);
    n_imu_ = 0;
    n_gnss_ = 0;
    n_odom_ = 0;
    write_latest();
  }

  // run_once() (or flush() at the end of the dataset) with the PF-01 per-scan timing.
  // Stats::n_scans changes only inside run_once()/flush(), so n_scans_seen_ (its value after
  // the last read) is the count before this call; stats() is read only when a pushed scan
  // is still pending.
  void timed(bool at_end) {
    const auto t0 = std::chrono::steady_clock::now();
    if (at_end) {
      est_.flush();
    } else {
      est_.run_once();
    }
    const std::chrono::duration<double, std::milli> dt = std::chrono::steady_clock::now() - t0;
    if (n_pushed_scans_ <= n_scans_seen_) {
      return;  // nothing pending: this call cannot have processed a scan
    }
    const std::uint64_t after = est_.stats().n_scans;
    if (after <= n_scans_seen_) {
      return;
    }
    const std::uint64_t n = after - n_scans_seen_;
    n_scans_seen_ = after;
    const double per_scan_ms = dt.count() / static_cast<double>(n);
    for (std::uint64_t k = 0; k < n; ++k) {
      if (scan_ms_.size() < scan_ms_.capacity()) {
        scan_ms_.push_back(per_scan_ms);  // never grows past the reserved capacity
      } else {
        ++n_latency_dropped_;
      }
    }
  }

  // One TUM line per newly processed scan (latest_state() changes only when one is).
  void write_latest() {
    const mss::common::Estimate e = est_.latest_state();
    if (e.state.t != mss::common::Timestamp{} && e.state.t != last_written_) {
      write_tum_line(tum_, e.state);
      last_written_ = e.state.t;
    }
  }

  mss::pipeline::Estimator& est_;
  std::FILE* tum_;
  std::size_t imu_limit_;
  std::size_t aux_limit_;
  std::size_t n_imu_ = 0;  // pushes since the last run_once(), per queue
  std::size_t n_gnss_ = 0;
  std::size_t n_odom_ = 0;
  mss::common::Timestamp last_written_{};
  Counts counts_;
  std::vector<double> scan_ms_;  // ms per processed scan (PF-01), reserved in the constructor
  std::uint64_t n_latency_dropped_ = 0;  // samples that did not fit scan_ms_
  std::uint64_t n_pushed_scans_ = 0;     // scans accepted by push_scan()
  std::uint64_t n_scans_seen_ = 0;       // Stats::n_scans at the last read
};

bool all_processed(const Counts& c, const mss::pipeline::Estimator::Stats& st) {
  return c.imu == st.n_imu && c.gnss == st.n_gnss && c.odom == st.n_odom && c.lidar == st.n_scans;
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<Args> args = parse_args(argc, argv);
  if (!args) {
    usage(argv[0]);
    return kExitError;
  }

  const auto cfg = mss::pipeline::load_config(args->config, args->overlay);
  if (!cfg) {
    std::fprintf(stderr, "mss_bench: config error:\n%s\n", cfg.reason().c_str());
    return kExitError;
  }
  const std::size_t max_points = cfg.value().realtime.max_points_per_scan;
  auto opened = mss::io::DatasetReader::open(args->dataset, max_points);
  if (!opened) {
    std::fprintf(stderr, "mss_bench: dataset error: %s\n", opened.reason().c_str());
    return kExitError;
  }
  mss::io::DatasetReader reader = std::move(opened).value();

  // The constructor may throw on an invalid realtime config (initialisation errors).
  std::optional<mss::pipeline::Estimator> est;
  try {
    est.emplace(cfg.value());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "mss_bench: estimator init error: %s\n", e.what());
    return kExitError;
  }

  std::error_code ec;
  fs::create_directories(args->out, ec);
  if (ec) {
    std::fprintf(
        stderr, "mss_bench: cannot create %s: %s\n", args->out.c_str(), ec.message().c_str());
    return kExitError;
  }
  const fs::path json_path = args->out / "bench.json";
  fs::remove(json_path, ec);  // no stale result from an earlier run if this one fails
  const fs::path tum_path = args->out / "est.tum";
  std::FILE* tum = std::fopen(tum_path.c_str(), "w");
  if (tum == nullptr) {
    std::fprintf(stderr, "mss_bench: cannot write %s\n", tum_path.c_str());
    return kExitError;
  }

  mss::common::ScanFrame scan(max_points);
  mss::io::Record rec;
  Replayer replayer(*est, cfg.value().realtime, tum, reader.index().n_lidar);
  int status = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (;;) {
    const auto more = reader.next(rec, scan);
    if (!more) {
      std::fprintf(stderr, "mss_bench: dataset error: %s\n", more.reason().c_str());
      status = kExitError;
      break;
    }
    if (!more.value()) {
      break;
    }
    replayer.feed(rec, scan);
  }
  replayer.finish();
  const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - t0;

  if (std::fclose(tum) != 0) {
    std::fprintf(stderr, "mss_bench: cannot write %s\n", tum_path.c_str());
    return kExitError;
  }
  if (status != 0) {
    return status;
  }
  const mss::pipeline::Estimator::Stats st = est->stats();
  const Counts& counts = replayer.counts();
  const ScanLatency lat = replayer.latency();
  if (!write_json(json_path, st, wall.count(), counts, lat)) {
    std::fprintf(stderr, "mss_bench: cannot write %s\n", json_path.c_str());
    return kExitError;
  }
  print_table(args->dataset, st, wall.count(), counts, lat);
  if (!all_processed(counts, st)) {
    std::fprintf(stderr,
                 "mss_bench: WARNING not every record was processed (records != n_*; see the "
                 "dropped/rejected counters in bench.json)\n");
    return kExitIncomplete;
  }
  return 0;
}
