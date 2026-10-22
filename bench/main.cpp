// Latency benchmark and dataset replay. Runs on Pi5 without ROS.
//
// Usage: mss_bench <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR
//
// Replays an mssdata dataset (ADR-014) in time order into pipeline::Estimator: IMU, GNSS
// and wheel records go to push_imu/push_gnss/push_odom, scans to push_scan; ground-truth
// records are counted and ignored. run_once() is called by the Replayer below (watermark
// and queue-pressure rules), not blindly after each scan. Writes only under DIR:
//   DIR/est.tum     one line per drain that processed a scan, from latest_state()
//   DIR/bench.json  Estimator::Stats, replay wall time and record counts per stream
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

// One TUM line: "t x y z qx qy qz qw" with t in seconds printed from integer ns (9 decimals).
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
               "%s%" PRIu64 ".%09" PRIu64 " %.12g %.12g %.12g %.12g %.12g %.12g %.12g\n",
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
                const Counts& c) {
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
               "  \"push_scan_us_p50\": %.3f,\n"
               "  \"push_scan_us_p95\": %.3f,\n"
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
               st.push_scan_us_p50,
               st.push_scan_us_p95,
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
                 const Counts& c) {
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
  std::printf("replay wall time %.3f s\n", replay_wall_s);
}

// Feeds the merged record stream into the Estimator and decides when to call run_once().
// Deterministic: the decisions depend only on the data and the realtime.* depths.
//
// Watermark rule. DatasetReader emits a scan at its t_start, but the Estimator orders it
// by its end key t_start + max(t_offset_ns) (computed here from the ScanFrame). After a
// scan is pushed, that key is the watermark, and the replayer drains (run_once) only
// before pushing the first record that sorts after the scan: IMU, wheel or GNSS with
// t > watermark (on t == key they sort before the scan, tie order IMU, wheel, GNSS, scan),
// or another scan with t_start >= watermark. So the IMU/wheel/GNSS records inside
// (t_start, t_end] are queued before the scan is processed.
//
// Pressure rule. The replayer counts pushes per queue since the last drain (run_once
// empties every queue) and drains before a push that would fill a queue to its configured
// depth (count >= depth - 1), so sensors that lead the LiDAR are not dropped:
//   - GNSS / wheel (aux, depth 32 = 0.64 s of 50 Hz wheel): drained on pressure even while
//     a scan is pending (that scan is then processed before its later IMU; rare);
//   - IMU: drained on pressure only when no scan is pending. While a scan is pending the
//     IMU samples of its span must reach the queue first; imu_queue_depth (400 = 2 s) is
//     far above the samples per scan (20 at 200 Hz / 10 Hz). An overflow is counted in
//     n_dropped_imu and turns into exit code 3;
//   - scans: drained before every scan push when no scan is pending (so a pending scan
//     always starts with an empty IMU queue and its span samples fit), and before a push
//     that would fill the scan queue.
// The watermark is the largest end key of the pending scans.
class Replayer {
 public:
  Replayer(mss::pipeline::Estimator& est, const mss::pipeline::RealtimeConfig& rt, std::FILE* tum)
      : est_(est),
        tum_(tum),
        imu_limit_(static_cast<std::size_t>(rt.imu_queue_depth) - 1U),
        aux_limit_(static_cast<std::size_t>(rt.aux_queue_depth) - 1U),
        scan_limit_(static_cast<std::size_t>(rt.scan_queue_depth) - 1U) {}

  void feed(const mss::io::Record& rec, const mss::common::ScanFrame& scan) {
    using mss::io::RecordKind;
    if (rec.kind == RecordKind::kGt) {
      ++counts_.gt;
      return;
    }
    if (scan_pending_ &&
        (rec.t > watermark_ || (rec.kind == RecordKind::kLidar && rec.t >= watermark_))) {
      drain();
    }
    // Push failures (drops, rejections) are counted by the estimator.
    switch (rec.kind) {
      case RecordKind::kImu:
        ++counts_.imu;
        if (n_imu_ >= imu_limit_ && !scan_pending_) {
          drain();
        }
        (void)est_.push_imu(rec.imu);
        ++n_imu_;
        break;
      case RecordKind::kGnss:
        ++counts_.gnss;
        if (n_gnss_ >= aux_limit_) {
          drain();
        }
        (void)est_.push_gnss(rec.gnss);
        ++n_gnss_;
        break;
      case RecordKind::kOdom:
        ++counts_.odom;
        if (n_odom_ >= aux_limit_) {
          drain();
        }
        (void)est_.push_odom(rec.odom);
        ++n_odom_;
        break;
      case RecordKind::kLidar:
        ++counts_.lidar;
        // With no scan pending, drain first so the scan starts with empty queues and its
        // span samples always fit (safe: the reader is time-ordered, so everything queued
        // has t <= t_start <= the scan key).
        if (!scan_pending_ || n_scans_ >= scan_limit_) {
          drain();
        }
        (void)est_.push_scan(scan);
        ++n_scans_;
        // max(): an overlapping, shorter scan must not move the watermark backwards.
        watermark_ = scan_pending_ ? std::max(watermark_, mss::common::scan_end_time(scan))
                                   : mss::common::scan_end_time(scan);
        scan_pending_ = true;
        break;
      case RecordKind::kGt:
        break;
    }
  }

  // Drains what is left. Call once at the end of the dataset.
  void finish() { drain(); }

  const Counts& counts() const { return counts_; }

 private:
  void drain() {
    est_.run_once();
    n_imu_ = 0;
    n_gnss_ = 0;
    n_odom_ = 0;
    n_scans_ = 0;
    scan_pending_ = false;
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
  std::size_t scan_limit_;
  std::size_t n_imu_ = 0;  // pushes since the last drain, per queue
  std::size_t n_gnss_ = 0;
  std::size_t n_odom_ = 0;
  std::size_t n_scans_ = 0;
  bool scan_pending_ = false;  // a scan was pushed and not drained yet
  mss::common::Timestamp watermark_{};
  mss::common::Timestamp last_written_{};
  Counts counts_;
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
  Replayer replayer(*est, cfg.value().realtime, tum);
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
  if (!write_json(json_path, st, wall.count(), counts)) {
    std::fprintf(stderr, "mss_bench: cannot write %s\n", json_path.c_str());
    return kExitError;
  }
  print_table(args->dataset, st, wall.count(), counts);
  if (!all_processed(counts, st)) {
    std::fprintf(stderr,
                 "mss_bench: WARNING not every record was processed (records != n_*; see the "
                 "dropped/rejected counters in bench.json)\n");
    return kExitIncomplete;
  }
  return 0;
}
