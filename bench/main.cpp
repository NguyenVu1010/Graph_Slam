// Latency benchmark and dataset replay. Runs on Pi5 without ROS.
//
// Usage: mss_bench <default.yaml> <mssdata-dir> [--overlay x.yaml] --out DIR
//
// Replays an mssdata dataset (ADR-014) in time order into pipeline::Estimator: IMU, GNSS
// and wheel records go to push_imu/push_gnss/push_odom, each scan to push_scan followed by
// run_once(); ground-truth records are counted and ignored. Writes only under DIR:
//   DIR/est.tum     one line per processed scan, from Estimator::latest_state()
//   DIR/bench.json  Estimator::Stats, replay wall time and record counts per stream
// and prints a summary table on stdout. Exit 0 on success; 2 on a usage, config, dataset
// or output error (reason on stderr). --out is required so the tool never writes into
// the source tree by default.
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

constexpr int kExitError = 2;

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
  std::printf("dropped scans %" PRIu64 ", rejected imu %" PRIu64 "\n",
              st.n_dropped_scans,
              st.n_rejected_imu);
  std::printf("push_scan p50 %.3f us, p95 %.3f us\n", st.push_scan_us_p50, st.push_scan_us_p95);
  std::printf("replay wall time %.3f s\n", replay_wall_s);
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

  std::error_code ec;
  fs::create_directories(args->out, ec);
  if (ec) {
    std::fprintf(
        stderr, "mss_bench: cannot create %s: %s\n", args->out.c_str(), ec.message().c_str());
    return kExitError;
  }
  const fs::path tum_path = args->out / "est.tum";
  std::FILE* tum = std::fopen(tum_path.c_str(), "w");
  if (tum == nullptr) {
    std::fprintf(stderr, "mss_bench: cannot write %s\n", tum_path.c_str());
    return kExitError;
  }

  int status = 0;
  {
    // The constructor may throw on an invalid realtime config (initialisation errors).
    std::optional<mss::pipeline::Estimator> est;
    try {
      est.emplace(cfg.value());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "mss_bench: estimator init error: %s\n", e.what());
      std::fclose(tum);
      return kExitError;
    }

    mss::common::ScanFrame scan(max_points);
    mss::io::Record rec;
    Counts counts;
    mss::common::Timestamp last_written{};

    // After run_once(), write the latest state once per newly processed scan.
    auto drain = [&]() {
      est->run_once();
      const mss::common::Estimate e = est->latest_state();
      if (e.state.t != mss::common::Timestamp{} && e.state.t != last_written) {
        write_tum_line(tum, e.state);
        last_written = e.state.t;
      }
    };

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
      // Push failures (drops, rejections) are logged and counted by the estimator.
      switch (rec.kind) {
        case mss::io::RecordKind::kImu:
          ++counts.imu;
          (void)est->push_imu(rec.imu);
          break;
        case mss::io::RecordKind::kGnss:
          ++counts.gnss;
          (void)est->push_gnss(rec.gnss);
          break;
        case mss::io::RecordKind::kOdom:
          ++counts.odom;
          (void)est->push_odom(rec.odom);
          break;
        case mss::io::RecordKind::kLidar:
          ++counts.lidar;
          (void)est->push_scan(scan);
          drain();
          break;
        case mss::io::RecordKind::kGt:
          ++counts.gt;
          break;
      }
    }
    drain();  // messages after the last scan
    const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - t0;

    if (std::fclose(tum) != 0) {
      std::fprintf(stderr, "mss_bench: cannot write %s\n", tum_path.c_str());
      return kExitError;
    }
    if (status != 0) {
      return status;
    }
    const mss::pipeline::Estimator::Stats st = est->stats();
    const fs::path json_path = args->out / "bench.json";
    if (!write_json(json_path, st, wall.count(), counts)) {
      std::fprintf(stderr, "mss_bench: cannot write %s\n", json_path.c_str());
      return kExitError;
    }
    print_table(args->dataset, st, wall.count(), counts);
  }
  return 0;
}
