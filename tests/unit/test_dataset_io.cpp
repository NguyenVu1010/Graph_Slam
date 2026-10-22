#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_format.hpp"
#include "mss/io/dataset_reader.hpp"
#include "mss/io/dataset_writer.hpp"

#include "alloc_counter.hpp"

namespace mss::io {
namespace {

namespace fs = std::filesystem;
using common::GnssMeasurement;
using common::ImuSample;
using common::NavState;
using common::ScanFrame;
using common::Timestamp;
using common::Vec3;
using common::WheelSample;

constexpr std::size_t kPointsPerScan = 100;

// Unique temporary directory per test, removed on destruction.
class TempDir {
 public:
  TempDir() {
    const std::string test = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    path_ = fs::temp_directory_path() /
            ("mss_io_" + test + "_" + std::to_string(static_cast<long>(::getpid())));
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

ImuSample make_imu(std::int64_t t, int k) {
  ImuSample s;
  s.t = Timestamp{t};
  s.acc_b = Vec3(0.1 + k, -0.3 * k, 9.80665 + 1e-3 * k);
  s.gyro_b = Vec3(1e-4 * k, -2e-3, 0.0123456789 * k);
  return s;
}

GnssMeasurement make_gnss(std::int64_t t, int k) {
  GnssMeasurement m;
  m.t = Timestamp{t};
  m.p_e_g = Vec3(10.5 + k, -3.25 * k, 1.0 / 3.0);
  // Asymmetric on purpose, so a row/column-major mix-up is caught.
  m.cov << 1.0 + k, 2.0, 3.0, 4.0, 5.0 + k, 6.0, 7.0, 8.0, 9.0 + k;
  m.status = static_cast<std::int8_t>(k - 1);
  return m;
}

WheelSample make_odom(std::int64_t t, int k) {
  WheelSample s;
  s.t = Timestamp{t};
  s.v_mps = 1.5 + 0.25 * k;
  s.omega_radps = -0.1 * k;
  return s;
}

NavState make_gt(std::int64_t t, int k) {
  NavState s;
  s.t = Timestamp{t};
  s.T_w_b.p = Vec3(1.0 * k, 2.0 + k, -0.5);
  s.T_w_b.R = Eigen::AngleAxisd(0.3 * k + 0.1, Vec3(0.2, -0.5, 1.0).normalized()).matrix();
  return s;
}

void fill_scan(ScanFrame& scan, std::int64_t t_start, int k, std::size_t n) {
  scan.reserve(n);
  scan.clear();
  scan.t_start = Timestamp{t_start};
  for (std::size_t i = 0; i < n; ++i) {
    const float f = static_cast<float>(i);
    ASSERT_TRUE(scan.push(0.01F * f + static_cast<float>(k),
                          -0.02F * f,
                          1.5F + 0.001F * f,
                          10.0F * f,
                          static_cast<std::uint32_t>(i * 1000U + static_cast<std::size_t>(k)),
                          static_cast<std::uint16_t>(i % 16U)));
  }
}

DatasetIndex make_meta() {
  DatasetIndex meta;
  meta.origin_lla = gnss::Lla{21.0278, 105.8342, 12.0};
  meta.T_b_l.p = Vec3(0.1, -0.2, 0.3);
  meta.T_b_l.R = Eigen::AngleAxisd(0.25, Vec3::UnitZ()).matrix();
  meta.p_b_g = Vec3(0.0, 0.5, 1.25);
  meta.body_frame = "base_link";
  meta.lidar_frame = "lidar_link";
  meta.gnss_cov_filled_from_sigma = true;
  return meta;
}

struct Expected {
  RecordKind kind;
  std::int64_t t;
  int k;  // index within its stream
};

// Times (ns) of the round-trip dataset, all distinct, interleaved across streams.
const std::vector<std::int64_t> kImuT = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000};
const std::vector<std::int64_t> kGnssT = {2500, 6500};
const std::vector<std::int64_t> kOdomT = {1500, 4500, 7500};
const std::vector<std::int64_t> kGtT = {500, 3100, 5500, 8500};
const std::vector<std::int64_t> kLidarT = {2200, 7200};

// Writes the round-trip dataset to `dir`; returns the expected merged order.
std::vector<Expected> write_round_trip(const fs::path& dir) {
  std::vector<Expected> expected;
  auto created = DatasetWriter::create(dir, make_meta());
  EXPECT_TRUE(created.is_ok()) << created.reason();
  if (!created.is_ok()) {
    return expected;
  }
  DatasetWriter w = std::move(created).value();
  for (std::size_t k = 0; k < kImuT.size(); ++k) {
    EXPECT_TRUE(w.write_imu(make_imu(kImuT[k], static_cast<int>(k))).is_ok());
    expected.push_back({RecordKind::kImu, kImuT[k], static_cast<int>(k)});
  }
  for (std::size_t k = 0; k < kGnssT.size(); ++k) {
    EXPECT_TRUE(w.write_gnss(make_gnss(kGnssT[k], static_cast<int>(k))).is_ok());
    expected.push_back({RecordKind::kGnss, kGnssT[k], static_cast<int>(k)});
  }
  for (std::size_t k = 0; k < kOdomT.size(); ++k) {
    EXPECT_TRUE(w.write_odom(make_odom(kOdomT[k], static_cast<int>(k))).is_ok());
    expected.push_back({RecordKind::kOdom, kOdomT[k], static_cast<int>(k)});
  }
  for (std::size_t k = 0; k < kGtT.size(); ++k) {
    EXPECT_TRUE(w.write_gt(make_gt(kGtT[k], static_cast<int>(k))).is_ok());
    expected.push_back({RecordKind::kGt, kGtT[k], static_cast<int>(k)});
  }
  ScanFrame scan(kPointsPerScan);
  for (std::size_t k = 0; k < kLidarT.size(); ++k) {
    fill_scan(scan, kLidarT[k], static_cast<int>(k), kPointsPerScan);
    EXPECT_TRUE(w.write_lidar(scan).is_ok());
    expected.push_back({RecordKind::kLidar, kLidarT[k], static_cast<int>(k)});
  }
  const common::Status st = w.finish();
  EXPECT_TRUE(st.is_ok()) << st.reason();
  std::stable_sort(expected.begin(), expected.end(), [](const Expected& a, const Expected& b) {
    return a.t < b.t;
  });
  return expected;
}

TEST(DatasetIo, RecordSizesMatchFormat) {
  static_assert(sizeof(ImuRecord) == 56);
  static_assert(sizeof(GnssRecord) == 105);
  static_assert(sizeof(OdomRecord) == 24);
  static_assert(sizeof(GtRecord) == 64);
  static_assert(sizeof(LidarHeader) == 12);
  static_assert(sizeof(LidarPoint) == 22);
  // Field offsets from docs/mssdata_format.md.
  EXPECT_EQ(offsetof(ImuRecord, gyro), 32U);
  EXPECT_EQ(offsetof(GnssRecord, cov), 32U);
  EXPECT_EQ(offsetof(GnssRecord, status), 104U);
  EXPECT_EQ(offsetof(OdomRecord, omega), 16U);
  EXPECT_EQ(offsetof(GtRecord, q_wxyz), 32U);
  EXPECT_EQ(offsetof(LidarHeader, n), 8U);
  EXPECT_EQ(offsetof(LidarPoint, intensity), 12U);
  EXPECT_EQ(offsetof(LidarPoint, t_offset_ns), 16U);
  EXPECT_EQ(offsetof(LidarPoint, ring), 20U);
  EXPECT_EQ(kFormatVersion, 1);
  EXPECT_TRUE(host_is_little_endian());
}

TEST(DatasetIo, WriteThenReadRoundTrip) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  const std::vector<Expected> expected = write_round_trip(dir);
  ASSERT_EQ(expected.size(), 21U);
  EXPECT_EQ(fs::file_size(dir / "imu.bin"), 10U * 56U);
  EXPECT_EQ(fs::file_size(dir / "lidar" / "000001.bin"), 12U + kPointsPerScan * 22U);

  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  DatasetReader reader = std::move(opened).value();

  const DatasetIndex& idx = reader.index();
  const DatasetIndex meta = make_meta();
  EXPECT_EQ(idx.version, 1);
  EXPECT_EQ(idx.n_imu, 10U);
  EXPECT_EQ(idx.n_gnss, 2U);
  EXPECT_EQ(idx.n_odom, 3U);
  EXPECT_EQ(idx.n_gt, 4U);
  EXPECT_EQ(idx.n_lidar, 2U);
  EXPECT_EQ(idx.origin_lla.lat_deg, meta.origin_lla.lat_deg);
  EXPECT_EQ(idx.origin_lla.lon_deg, meta.origin_lla.lon_deg);
  EXPECT_EQ(idx.origin_lla.alt_m, meta.origin_lla.alt_m);
  EXPECT_EQ(idx.T_b_l.p, meta.T_b_l.p);
  EXPECT_TRUE(idx.T_b_l.R.isApprox(meta.T_b_l.R, 1e-12));
  EXPECT_EQ(idx.p_b_g, meta.p_b_g);
  EXPECT_EQ(idx.body_frame, "base_link");
  EXPECT_EQ(idx.lidar_frame, "lidar_link");
  EXPECT_TRUE(idx.gnss_cov_filled_from_sigma);

  Record rec;
  ScanFrame scan(kPointsPerScan);
  ScanFrame want_scan(kPointsPerScan);
  std::size_t i = 0;
  while (true) {
    auto r = reader.next(rec, scan);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    if (!r.value()) {
      break;
    }
    ASSERT_LT(i, expected.size());
    const Expected& e = expected[i];
    ASSERT_EQ(rec.kind, e.kind) << "record " << i;
    EXPECT_EQ(rec.t.ns, e.t) << "record " << i;
    switch (e.kind) {
      case RecordKind::kImu: {
        const ImuSample want = make_imu(e.t, e.k);
        EXPECT_EQ(rec.imu.t, want.t);
        EXPECT_EQ(rec.imu.acc_b, want.acc_b);
        EXPECT_EQ(rec.imu.gyro_b, want.gyro_b);
        break;
      }
      case RecordKind::kGnss: {
        const GnssMeasurement want = make_gnss(e.t, e.k);
        EXPECT_EQ(rec.gnss.t, want.t);
        EXPECT_EQ(rec.gnss.p_e_g, want.p_e_g);
        EXPECT_EQ(rec.gnss.cov, want.cov);
        EXPECT_EQ(rec.gnss.status, want.status);
        break;
      }
      case RecordKind::kOdom: {
        const WheelSample want = make_odom(e.t, e.k);
        EXPECT_EQ(rec.odom.t, want.t);
        EXPECT_EQ(rec.odom.v_mps, want.v_mps);
        EXPECT_EQ(rec.odom.omega_radps, want.omega_radps);
        break;
      }
      case RecordKind::kGt: {
        const NavState want = make_gt(e.t, e.k);
        EXPECT_EQ(rec.gt.t, want.t);
        EXPECT_EQ(rec.gt.T_w_b.p, want.T_w_b.p);
        EXPECT_TRUE(rec.gt.T_w_b.R.isApprox(want.T_w_b.R, 1e-12));
        EXPECT_EQ(rec.gt.v_w_b, Vec3::Zero());
        break;
      }
      case RecordKind::kLidar: {
        fill_scan(want_scan, e.t, e.k, kPointsPerScan);
        EXPECT_EQ(scan.t_start, want_scan.t_start);
        ASSERT_EQ(scan.size(), kPointsPerScan);
        EXPECT_EQ(std::memcmp(scan.xyz().data(), want_scan.xyz().data(), 3 * 4 * kPointsPerScan),
                  0);
        EXPECT_EQ(
            std::memcmp(scan.intensity().data(), want_scan.intensity().data(), 4 * kPointsPerScan),
            0);
        for (std::size_t p = 0; p < kPointsPerScan; ++p) {
          EXPECT_EQ(scan.t_offset_ns()[p], want_scan.t_offset_ns()[p]);
          EXPECT_EQ(scan.ring()[p], want_scan.ring()[p]);
        }
        break;
      }
    }
    ++i;
  }
  EXPECT_EQ(i, 21U);
  // Exhausted stays exhausted.
  auto again = reader.next(rec, scan);
  ASSERT_TRUE(again.is_ok());
  EXPECT_FALSE(again.value());
}

TEST(DatasetIo, TieOrderIsStable) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  {
    auto created = DatasetWriter::create(dir, make_meta());
    ASSERT_TRUE(created.is_ok()) << created.reason();
    DatasetWriter w = std::move(created).value();
    ScanFrame scan(kPointsPerScan);
    fill_scan(scan, 100, 0, 3);
    // Written in an order different from the expected tie order.
    ASSERT_TRUE(w.write_lidar(scan).is_ok());
    ASSERT_TRUE(w.write_gt(make_gt(100, 0)).is_ok());
    ASSERT_TRUE(w.write_gnss(make_gnss(100, 0)).is_ok());
    ASSERT_TRUE(w.write_odom(make_odom(100, 0)).is_ok());
    ASSERT_TRUE(w.write_imu(make_imu(100, 0)).is_ok());
  }  // destructor finishes
  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  DatasetReader reader = std::move(opened).value();
  Record rec;
  ScanFrame scan(kPointsPerScan);
  std::vector<RecordKind> kinds;
  while (true) {
    auto r = reader.next(rec, scan);
    ASSERT_TRUE(r.is_ok()) << r.reason();
    if (!r.value()) {
      break;
    }
    EXPECT_EQ(rec.t.ns, 100);
    kinds.push_back(rec.kind);
  }
  const std::vector<RecordKind> want = {
      RecordKind::kImu, RecordKind::kOdom, RecordKind::kGnss, RecordKind::kGt, RecordKind::kLidar};
  EXPECT_EQ(kinds, want);
}

TEST(DatasetIo, NextDoesNotAllocate) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  ASSERT_EQ(write_round_trip(dir).size(), 21U);
  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  DatasetReader reader = std::move(opened).value();
  Record rec;
  ScanFrame scan(kPointsPerScan);

  std::size_t n = 0;
  bool failed = false;
  AllocCounter::reset();
  while (true) {
    const common::Result<bool> r = reader.next(rec, scan);
    if (!r.is_ok()) {
      failed = true;
      break;
    }
    if (!r.value()) {
      break;
    }
    ++n;
  }
  const std::size_t allocs = AllocCounter::count();
  EXPECT_FALSE(failed);
  EXPECT_EQ(n, 21U);
  EXPECT_EQ(allocs, 0U);
}

TEST(DatasetIo, TruncatedLidarFileIsStatusError) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  ASSERT_EQ(write_round_trip(dir).size(), 21U);
  fs::resize_file(dir / "lidar" / "000001.bin", 50);

  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  DatasetReader reader = std::move(opened).value();
  Record rec;
  ScanFrame scan(kPointsPerScan);
  std::size_t n_ok = 0;
  std::string reason;
  for (int guard = 0; guard < 100; ++guard) {
    auto r = reader.next(rec, scan);
    if (!r.is_ok()) {
      reason = r.reason();
      break;
    }
    ASSERT_TRUE(r.value()) << "reached the end without an error";
    ++n_ok;
  }
  EXPECT_NE(reason.find("000001.bin"), std::string::npos) << reason;
  // Everything before the second scan (t = 7200) came out first.
  EXPECT_GE(n_ok, 1U);
  // The error is sticky.
  auto again = reader.next(rec, scan);
  ASSERT_FALSE(again.is_ok());
  EXPECT_EQ(again.reason(), reason);
}

TEST(DatasetIo, TooManyPointsRejected) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  ASSERT_EQ(write_round_trip(dir).size(), 21U);
  auto opened = DatasetReader::open(dir, 50);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  DatasetReader reader = std::move(opened).value();
  Record rec;
  ScanFrame scan(50);
  std::string reason;
  for (int guard = 0; guard < 100; ++guard) {
    auto r = reader.next(rec, scan);
    if (!r.is_ok()) {
      reason = r.reason();
      break;
    }
    ASSERT_TRUE(r.value()) << "reached the end without an error";
    EXPECT_NE(rec.kind, RecordKind::kLidar);
  }
  EXPECT_NE(reason.find("000000.bin"), std::string::npos) << reason;
  EXPECT_NE(reason.find("max_points_per_scan"), std::string::npos) << reason;
}

TEST(DatasetIo, MissingIndexIsError) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  ASSERT_EQ(write_round_trip(dir).size(), 21U);
  fs::remove(dir / "index.json");
  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_FALSE(opened.is_ok());
  EXPECT_NE(opened.reason().find("index.json"), std::string::npos) << opened.reason();

  auto missing_dir = DatasetReader::open(tmp.path() / "does_not_exist", kPointsPerScan);
  EXPECT_FALSE(missing_dir.is_ok());
}

TEST(DatasetIo, CountMismatchIsError) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  ASSERT_EQ(write_round_trip(dir).size(), 21U);
  fs::resize_file(dir / "imu.bin", 9U * 56U);
  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_FALSE(opened.is_ok());
  EXPECT_NE(opened.reason().find("imu.bin"), std::string::npos) << opened.reason();

  fs::resize_file(dir / "imu.bin", 9U * 56U + 3U);
  opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_FALSE(opened.is_ok());
  EXPECT_NE(opened.reason().find("imu.bin"), std::string::npos) << opened.reason();
}

TEST(DatasetIo, WriterRejectsBackwardTimeAndNonEmptyDir) {
  TempDir tmp;
  const fs::path dir = tmp.path() / "ds";
  auto created = DatasetWriter::create(dir, make_meta());
  ASSERT_TRUE(created.is_ok()) << created.reason();
  DatasetWriter w = std::move(created).value();
  ASSERT_TRUE(w.write_imu(make_imu(200, 0)).is_ok());
  EXPECT_TRUE(w.write_imu(make_imu(200, 1)).is_ok());  // equal t is allowed
  EXPECT_FALSE(w.write_imu(make_imu(100, 2)).is_ok());
  ASSERT_TRUE(w.finish().is_ok());
  EXPECT_TRUE(w.finish().is_ok());  // idempotent
  EXPECT_FALSE(w.write_imu(make_imu(300, 3)).is_ok());

  auto opened = DatasetReader::open(dir, kPointsPerScan);
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  EXPECT_EQ(opened.value().index().n_imu, 2U);

  auto again = DatasetWriter::create(dir, make_meta());
  EXPECT_FALSE(again.is_ok());
}

}  // namespace
}  // namespace mss::io
