#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "mss/common/types.hpp"
#include "mss/gnss/gnss.hpp"

// On-disk layout of the `mssdata` dataset, version 1 (ADR-014, docs/mssdata_format.md).
// Must match the numpy dtypes in tools/bag_to_mssdata.py byte for byte.
//
// Little-endian hosts only: records are read and written with memcpy, without byte
// swapping. DatasetReader::open and DatasetWriter::create check the host byte order at
// run time and return an error on a big-endian host.
namespace mss::io {

constexpr int kFormatVersion = 1;
// Lidar scans are stored as lidar/%06d.bin (zero-padded to this many digits).
constexpr std::size_t kLidarNameWidth = 6;

#pragma pack(push, 1)
// imu.bin, 56 B: t ns; acc m/s^2 in b; gyro rad/s in b.
struct ImuRecord {
  std::int64_t t;
  double acc[3];
  double gyro[3];
};
// gnss.bin, 105 B: t ns; p_e_g m in e; cov m^2 row-major 3x3; NavSatStatus.status.
struct GnssRecord {
  std::int64_t t;
  double p_e_g[3];
  double cov[9];
  std::int8_t status;
};
// odom.bin, 24 B: t ns; v m/s; omega rad/s.
struct OdomRecord {
  std::int64_t t;
  double v;
  double omega;
};
// gt.bin, 64 B: t ns; p = p_w_b m; R_w_b as a Hamilton quaternion in order (w, x, y, z).
struct GtRecord {
  std::int64_t t;
  double p[3];
  double q_wxyz[4];
};
// lidar/NNNNNN.bin header, 12 B: t_start ns; n points that follow.
struct LidarHeader {
  std::int64_t t_start;
  std::uint32_t n;
};
// lidar/NNNNNN.bin point, 22 B: xyz m in l; intensity; t_offset ns since t_start; ring.
struct LidarPoint {
  float xyz[3];
  float intensity;
  std::uint32_t t_offset_ns;
  std::uint16_t ring;
};
#pragma pack(pop)

static_assert(sizeof(ImuRecord) == 56, "imu.bin record is 56 B");
static_assert(sizeof(GnssRecord) == 105, "gnss.bin record is 105 B");
static_assert(sizeof(OdomRecord) == 24, "odom.bin record is 24 B");
static_assert(sizeof(GtRecord) == 64, "gt.bin record is 64 B");
static_assert(sizeof(LidarHeader) == 12, "lidar header is 12 B");
static_assert(sizeof(LidarPoint) == 22, "lidar point is 22 B");

// True when the host stores integers little-endian (the only supported byte order).
bool host_is_little_endian();

// Contents of index.json. Counts are numbers of records (scans for lidar).
struct DatasetIndex {
  int version = kFormatVersion;
  std::size_t n_imu = 0;
  std::size_t n_gnss = 0;
  std::size_t n_odom = 0;
  std::size_t n_gt = 0;
  std::size_t n_lidar = 0;
  gnss::Lla origin_lla;                       // origin of the ENU frame e
  common::Pose T_b_l;                         // LiDAR in body: p_b = R * p_l + p
  common::Vec3 p_b_g = common::Vec3::Zero();  // m, GNSS antenna lever arm in b
  std::string body_frame = "base_link";
  std::string lidar_frame = "lidar_link";
  bool gnss_cov_filled_from_sigma = false;
};

enum class RecordKind { kImu, kGnss, kOdom, kLidar, kGt };

// One merged record. Only the member matching `kind` is meaningful; for kLidar the scan
// is written into the caller's ScanFrame and `t` is its t_start.
struct Record {
  RecordKind kind = RecordKind::kImu;
  common::Timestamp t{};
  common::ImuSample imu;
  common::GnssMeasurement gnss;
  common::WheelSample odom;
  common::NavState gt;  // T_w_b from gt.bin; v_w_b and bias are zero (not stored)
};

}  // namespace mss::io
