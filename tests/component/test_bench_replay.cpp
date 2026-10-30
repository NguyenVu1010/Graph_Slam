// Component test: mss_bench replays a small synthetic mssdata dataset with a known record
// count and writes est.tum and bench.json under --out.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_format.hpp"
#include "mss/io/dataset_writer.hpp"

#include "box_world.hpp"

namespace {

namespace fs = std::filesystem;
using mss::common::GnssMeasurement;
using mss::common::ImuSample;
using mss::common::NavState;
using mss::common::ScanFrame;
using mss::common::Timestamp;
using mss::common::Vec3;
using mss::common::WheelSample;

const fs::path kRepo{MSS_REPO_DIR};
constexpr std::int64_t kT0 = 1000000000;  // ns, dataset start
constexpr std::int64_t kMs = 1000000;     // ns per millisecond
constexpr std::size_t kPointsPerScan = 1000;

// Unique temporary directory per test, removed on destruction.
class TempDir {
 public:
  TempDir() {
    const std::string test = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    path_ = fs::temp_directory_path() /
            ("mss_bench_" + test + "_" + std::to_string(static_cast<long>(::getpid())));
    std::error_code ec;
    fs::remove_all(path_, ec);
    fs::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

// 1 s of data: 200 IMU (200 Hz), 5 GNSS (5 Hz), 50 wheel (50 Hz), 10 scans (10 Hz) of
// 1000 points spread over 99 ms, 10 ground-truth poses (ignored by the bench).
// `lead_ms` milliseconds of IMU (and, with `with_aux`, GNSS and wheel) data come before
// the first scan. Without `with_aux` the dataset has no GNSS or wheel records. With
// `dup_imu` the IMU sample at t = kT0 + 500 ms is written twice (same timestamp), which the
// writer accepts and Estimator::push_imu rejects. `imu_offset_ms` shifts every IMU stamp
// (kT0 + offset + k * 5 ms); `extra_imu` appends that many IMU samples at the end (one
// reaches past the last scan's end, so flush() leaves no scan uncovered).
void write_dataset(const fs::path& dir,
                   int lead_ms = 0,
                   bool with_aux = true,
                   bool dup_imu = false,
                   int imu_offset_ms = 0,
                   int extra_imu = 0) {
  mss::io::DatasetIndex meta;
  auto created = mss::io::DatasetWriter::create(dir, meta);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  mss::io::DatasetWriter w = std::move(created).value();
  const int n_imu = 200 * (1000 + lead_ms) / 1000 + extra_imu;
  const int n_gnss = with_aux ? 5 * (1000 + lead_ms) / 1000 : 0;
  const int n_odom = with_aux ? 50 * (1000 + lead_ms) / 1000 : 0;
  const std::int64_t t_scan0 = kT0 + lead_ms * kMs;
  for (int k = 0; k < n_imu; ++k) {
    ImuSample s;
    s.t = Timestamp{kT0 + imu_offset_ms * kMs + k * 5 * kMs};
    s.acc_b = Vec3(0.0, 0.0, 9.80665);
    ASSERT_TRUE(w.write_imu(s).is_ok());
    if (dup_imu && k == 100) {
      ASSERT_TRUE(w.write_imu(s).is_ok());
    }
  }
  for (int k = 0; k < n_gnss; ++k) {
    GnssMeasurement m;
    m.t = Timestamp{kT0 + k * 200 * kMs};
    ASSERT_TRUE(w.write_gnss(m).is_ok());
  }
  for (int k = 0; k < n_odom; ++k) {
    WheelSample s;
    s.t = Timestamp{kT0 + k * 20 * kMs};
    ASSERT_TRUE(w.write_odom(s).is_ok());
  }
  ScanFrame scan(kPointsPerScan);
  for (int k = 0; k < 10; ++k) {
    scan.clear();
    scan.t_start = Timestamp{t_scan0 + k * 100 * kMs};
    for (std::size_t i = 0; i < kPointsPerScan; ++i) {
      const float f = static_cast<float>(i);
      ASSERT_TRUE(
          scan.push(0.01F * f, 1.0F, 0.5F, 1.0F, static_cast<std::uint32_t>(i * 99000U), 0));
    }
    ASSERT_TRUE(w.write_lidar(scan).is_ok());
    NavState gt;
    gt.t = scan.t_start;
    ASSERT_TRUE(w.write_gt(gt).is_ok());
  }
  ASSERT_TRUE(w.finish().is_ok());
}

std::string read_file(const fs::path& p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Value of a numeric top-level key in bench.json; -1 (and a test failure) when it is missing.
double json_number(const std::string& json, const std::string& key) {
  const std::string tag = "\"" + key + "\": ";
  const std::size_t pos = json.find(tag);
  if (pos == std::string::npos) {
    ADD_FAILURE() << key << " missing in " << json;
    return -1.0;
  }
  return std::stod(json.substr(pos + tag.size()));
}

int run(const std::string& cmd) {
  const int status = std::system(cmd.c_str());
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

// Shell command running mss_bench on config/default.yaml (+ optional overlay).
std::string bench_cmd(const fs::path& data, const fs::path& out, const fs::path& overlay = {}) {
  std::string cmd = std::string("'") + MSS_BENCH_PATH + "' '" +
                    (kRepo / "config/default.yaml").string() + "' '" + data.string() + "' --out '" +
                    out.string() + "'";
  if (!overlay.empty()) {
    cmd += " --overlay '" + overlay.string() + "'";
  }
  return cmd;
}

// C-09. 2 s of still IMU before the first scan, so the IMU initialises at scan 0 and every
// scan produces an estimate; one extra IMU sample (t = 3000 ms) reaches past the last scan's
// end (2998.9 ms), so flush() leaves no scan uncovered: 10 trajectory lines.
TEST(BenchReplay, ReplaysSyntheticDataset) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 2000, true, false, 0, 1));
  const std::string cmd = bench_cmd(data, out) + " 2>/dev/null";
  ASSERT_EQ(run(cmd), 0) << cmd;

  std::ifstream tum(out / "est.tum");
  ASSERT_TRUE(tum.is_open());
  std::string line;
  int n_lines = 0;
  while (std::getline(tum, line)) {
    if (n_lines == 0) {
      // First estimate at scan 0 (t = 3 s), still at the initial position.
      EXPECT_EQ(line.rfind("3.000000000 ", 0), 0U) << line;
      std::istringstream ss(line);
      double t_s = 0.0;
      double x = 1.0;
      double y = 1.0;
      double z = 1.0;
      ss >> t_s >> x >> y >> z;
      EXPECT_LT(std::sqrt(x * x + y * y + z * z), 1e-6) << line;
    }
    ++n_lines;
  }
  EXPECT_EQ(n_lines, 10);

  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_imu\": 601,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_gnss\": 15,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_odom\": 150,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_uncovered_scans\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gt\": 10"), std::string::npos) << json;
  EXPECT_NE(json.find("\"replay_wall_s\""), std::string::npos) << json;
  // records == processed for every stream
  EXPECT_NE(json.find("\"imu\": 601,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gnss\": 15,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"odom\": 150,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"lidar\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_aux\": 0,"), std::string::npos) << json;
  // PF-01: per-scan processing latency, ms.
  const double p50 = json_number(json, "scan_process_ms_p50");
  const double p95 = json_number(json, "scan_process_ms_p95");
  EXPECT_GE(p50, 0.0) << json;
  EXPECT_GE(p95, p50) << json;
  const double p99 = json_number(json, "scan_process_ms_p99");
  EXPECT_GE(p99, p95) << json;
  EXPECT_GE(json_number(json, "scan_process_ms_max"), p99) << json;
  // One sample per processed scan, all inside the buffer reserved from index.json.
  EXPECT_NE(json.find("\"n_latency_samples\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_latency_dropped\": 0,"), std::string::npos) << json;
  // Map counters (additive fields): the map holds points; spacing counts are >= 0.
  EXPECT_GE(json_number(json, "n_map_points"), 0.0) << json;
  EXPECT_GE(json_number(json, "n_map_spacing_replaced"), 0.0) << json;
}

// 3 s of IMU / GNSS / wheel before the first scan: more than the IMU (400) and aux (32)
// queue depths, so the bench must drain on queue pressure instead of dropping.
TEST(BenchReplay, SensorsLeadingLidarAreNotDropped) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 3000));
  const std::string cmd = bench_cmd(data, out);
  ASSERT_EQ(run(cmd), 0) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_imu\": 800,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_gnss\": 20,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_odom\": 200,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_aux\": 0,"), std::string::npos) << json;
}

// IMU only (no GNSS / wheel to force aux drains) leading the LiDAR by 3.95 s: 790 samples,
// plus the one at scan 0's t_start. Pressure drains after 399 pushes leave 392 queued when
// scan 0 arrives; the run_once() after the scan push processes them (they precede the
// scan), so its span samples fit in the 400-deep IMU queue.
TEST(BenchReplay, ImuOnlyLeadIsNotDropped) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 3950, false));
  const std::string cmd = bench_cmd(data, out);
  ASSERT_EQ(run(cmd), 0) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_imu\": 990,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"imu\": 990,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_gnss\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_odom\": 0,"), std::string::npos) << json;
}

// A duplicated IMU timestamp is rejected by push_imu, so one record is not processed: the
// bench still writes its outputs but exits 3. (Queue overflow can no longer be forced
// through the bench: it drains on pressure, and the Estimator's IMU coverage gate makes
// every early drain safe.)
TEST(BenchReplay, UnprocessedRecordsExit3) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 0, true, true));
  const std::string cmd = bench_cmd(data, out) + " 2>/dev/null";
  EXPECT_EQ(run(cmd), 3) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"imu\": 201,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_imu\": 200,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_rejected_imu\": 1,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
}

// A tiny IMU queue (depth 4) no longer drops samples: the bench drains on pressure and the
// Estimator holds each scan until an IMU sample covers its end time.
TEST(BenchReplay, SmallImuQueueStillComplete) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  const fs::path overlay = tmp.path() / "small_imu_queue.yaml";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data));
  {
    std::ofstream f(overlay);
    f << "mss_node:\n  ros__parameters:\n    realtime:\n      imu_queue_depth: 4\n";
  }
  const std::string cmd = bench_cmd(data, out, overlay);
  ASSERT_EQ(run(cmd), 0) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_imu\": 200,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
}

// Regression: with the IMU stamped 3 ms after the LiDAR grid, the sample covering scan
// k-1's end arrives after scan k's record. Draining only after a scan push left two
// uncovered scans queued and the third push dropped one (scan_queue_depth 2).
TEST(BenchReplay, ImuStampOffsetDoesNotDropScans) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 0, true, false, 3));
  const std::string cmd = bench_cmd(data, out) + " 2>/dev/null";
  ASSERT_EQ(run(cmd), 0) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_dropped_scans\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10,"), std::string::npos) << json;
}

// E-02: two runs over the same input give byte-identical trajectories.
TEST(BenchReplay, E02_TwoRunsAreBitIdentical) {
  TempDir tmp;
  mss::test::BoxWorldParams params;
  params.motion_s = 5.0;
  const fs::path data = tmp.path() / "mssdata";
  ASSERT_TRUE(mss::test::write_box_dataset(data, params).is_ok());
  const fs::path overlay = tmp.path() / "box.yaml";
  {
    std::ofstream f(overlay);
    f << mss::test::box_overlay_yaml(params, true);
  }
  const fs::path out1 = tmp.path() / "out1";
  const fs::path out2 = tmp.path() / "out2";
  const std::string cmd1 = bench_cmd(data, out1, overlay) + " >/dev/null 2>&1";
  const std::string cmd2 = bench_cmd(data, out2, overlay) + " >/dev/null 2>&1";
  ASSERT_EQ(run(cmd1), 0) << cmd1;
  ASSERT_EQ(run(cmd2), 0) << cmd2;
  const std::string tum1 = read_file(out1 / "est.tum");
  const std::string tum2 = read_file(out2 / "est.tum");
  EXPECT_FALSE(tum1.empty());
  EXPECT_TRUE(tum1 == tum2) << "est.tum differs between two runs";
  const std::string json1 = read_file(out1 / "bench.json");
  const std::string json2 = read_file(out2 / "bench.json");
  const auto n_scans = [](const std::string& json) {
    const std::size_t pos = json.find("\"n_scans\": ");
    return pos == std::string::npos ? std::string() : json.substr(pos, json.find(',', pos) - pos);
  };
  EXPECT_FALSE(n_scans(json1).empty()) << json1;
  EXPECT_EQ(n_scans(json1), n_scans(json2));
}

TEST(BenchReplay, MissingDatasetExits2AndWritesNothing) {
  TempDir tmp;
  const fs::path out = tmp.path() / "out";
  const std::string cmd =
      std::string("'") + MSS_BENCH_PATH + "' '" + (kRepo / "config/default.yaml").string() + "' '" +
      (tmp.path() / "missing").string() + "' --out '" + out.string() + "' 2>/dev/null";
  EXPECT_EQ(run(cmd), 2);
  EXPECT_FALSE(fs::exists(out));
}

// M2: an unwritable --map-out directory is found at start-up: exit 2 before the replay, so
// nothing is written under --out (it used to be found after the whole run, with est.tum and
// bench.json already written).
TEST(BenchReplay, BadMapOutDirectoryExits2BeforeTheReplay) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 2000, true, false, 0, 1));
  const fs::path bad = tmp.path() / "no_such_dir" / "map.pcd";
  const std::string cmd = bench_cmd(data, out) + " --map-out '" + bad.string() + "' 2>/dev/null";
  EXPECT_EQ(run(cmd), 2) << cmd;
  EXPECT_FALSE(fs::exists(out)) << "the replay ran before the map-out check";
  // A good path passes the check and leaves no probe file behind. The export itself needs the
  // GTSAM backend (M5), so the exit code and the file are only checked when it succeeded.
  const fs::path good = tmp.path() / "map.pcd";
  {
    std::ofstream user(good.string() + ".probe");  // the old probe name: must be left alone
    user << "mine";
  }
  const std::string ok_cmd =
      bench_cmd(data, out) + " --map-out '" + good.string() + "' 2>/dev/null";
  const int ok_exit = run(ok_cmd);
  EXPECT_TRUE(ok_exit == 0 || ok_exit == 2) << ok_cmd;
  if (ok_exit == 0) {
    EXPECT_TRUE(fs::exists(good));
  }
  for (const auto& e : fs::directory_iterator(tmp.path())) {
    EXPECT_EQ(e.path().filename().string().rfind(".mss_bench_probe", 0), std::string::npos)
        << "probe file left behind: " << e.path();
  }
  EXPECT_FALSE(fs::exists(good.string() + ".tmp"));
  EXPECT_EQ(read_file(good.string() + ".probe"), "mine");
}

TEST(BenchReplay, MissingOutIsUsageError) {
  const std::string cmd = std::string("'") + MSS_BENCH_PATH + "' '" +
                          (kRepo / "config/default.yaml").string() + "' /nonexistent 2>/dev/null";
  EXPECT_EQ(run(cmd), 2);
}

}  // namespace
