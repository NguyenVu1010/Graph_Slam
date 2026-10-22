// Component test: mss_bench replays a small synthetic mssdata dataset with a known record
// count and writes est.tum and bench.json under --out.
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
// `lead_s` seconds of IMU, GNSS and wheel data come before the first scan.
void write_dataset(const fs::path& dir, int lead_s = 0) {
  mss::io::DatasetIndex meta;
  auto created = mss::io::DatasetWriter::create(dir, meta);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  mss::io::DatasetWriter w = std::move(created).value();
  const int n_imu = 200 * (1 + lead_s);
  const int n_gnss = 5 * (1 + lead_s);
  const int n_odom = 50 * (1 + lead_s);
  const std::int64_t t_scan0 = kT0 + lead_s * 1000 * kMs;
  for (int k = 0; k < n_imu; ++k) {
    ImuSample s;
    s.t = Timestamp{kT0 + k * 5 * kMs};
    s.acc_b = Vec3(0.0, 0.0, 9.80665);
    ASSERT_TRUE(w.write_imu(s).is_ok());
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

TEST(BenchReplay, ReplaysSyntheticDataset) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data));
  const std::string cmd = bench_cmd(data, out);
  ASSERT_EQ(run(cmd), 0) << cmd;

  std::ifstream tum(out / "est.tum");
  ASSERT_TRUE(tum.is_open());
  std::string line;
  int n_lines = 0;
  while (std::getline(tum, line)) {
    if (n_lines == 0) {
      EXPECT_EQ(line, "1.000000000 0 0 0 0 0 0 1");  // identity pose, TUM (x, y, z, w)
    }
    ++n_lines;
  }
  EXPECT_EQ(n_lines, 10);

  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"n_imu\": 200"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_gnss\": 5"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_odom\": 50"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_scans\": 10"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gt\": 10"), std::string::npos) << json;
  EXPECT_NE(json.find("\"replay_wall_s\""), std::string::npos) << json;
  // records == processed for every stream
  EXPECT_NE(json.find("\"imu\": 200,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gnss\": 5,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"odom\": 50,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"lidar\": 10,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"n_dropped_aux\": 0,"), std::string::npos) << json;
}

// 3 s of IMU / GNSS / wheel before the first scan: more than the IMU (400) and aux (32)
// queue depths, so the bench must drain on queue pressure instead of dropping.
TEST(BenchReplay, SensorsLeadingLidarAreNotDropped) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data, 3));
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

// An IMU queue shorter than the IMU samples inside one scan span must overflow while the
// scan is pending: the bench still writes its outputs but exits 3.
TEST(BenchReplay, UnprocessedRecordsExit3) {
  TempDir tmp;
  const fs::path data = tmp.path() / "mssdata";
  const fs::path out = tmp.path() / "out";
  const fs::path overlay = tmp.path() / "small_imu_queue.yaml";
  ASSERT_NO_FATAL_FAILURE(write_dataset(data));
  {
    std::ofstream f(overlay);
    f << "mss_node:\n  ros__parameters:\n    realtime:\n      imu_queue_depth: 4\n";
  }
  const std::string cmd = bench_cmd(data, out, overlay) + " 2>/dev/null";
  EXPECT_EQ(run(cmd), 3) << cmd;
  const std::string json = read_file(out / "bench.json");
  EXPECT_NE(json.find("\"imu\": 200,"), std::string::npos) << json;
  EXPECT_EQ(json.find("\"n_dropped_imu\": 0,"), std::string::npos) << json;
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

TEST(BenchReplay, MissingOutIsUsageError) {
  const std::string cmd = std::string("'") + MSS_BENCH_PATH + "' '" +
                          (kRepo / "config/default.yaml").string() + "' /nonexistent 2>/dev/null";
  EXPECT_EQ(run(cmd), 2);
}

}  // namespace
