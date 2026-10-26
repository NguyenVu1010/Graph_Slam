#pragma once

#include <filesystem>
#include <memory>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_format.hpp"

namespace mss::io {

/// Writes an `mssdata` directory (docs/mssdata_format.md) readable by DatasetReader and by
/// the consumers of tools/bag_to_mssdata.py output. Used by tests and tools, not on the
/// hot path.
///
/// Within each stream, records must be written with non-decreasing `t` (lidar: t_start);
/// a backwards `t` returns Status::error and writes nothing. finish() closes the files and
/// writes index.json with the counts actually written and the metadata given to create()
/// (the counts and version in `meta` are ignored). finish() is idempotent: later calls
/// return the first result. The destructor calls finish() if it was not called. Writes
/// after finish() fail. Nothing throws. Movable, not copyable.
class DatasetWriter {
 public:
  /// `dir` must not exist or must be an empty directory. Fails on a big-endian host, on
  /// non-finite metadata, or when the files cannot be created.
  static common::Result<DatasetWriter> create(const std::filesystem::path& dir,
                                              const DatasetIndex& meta);

  DatasetWriter(DatasetWriter&& other) noexcept;
  DatasetWriter& operator=(DatasetWriter&& other) noexcept;
  DatasetWriter(const DatasetWriter&) = delete;
  DatasetWriter& operator=(const DatasetWriter&) = delete;
  ~DatasetWriter();

  common::Status write_imu(const common::ImuSample& s);
  common::Status write_gnss(const common::GnssMeasurement& m);
  common::Status write_odom(const common::WheelSample& s);
  /// Stores t, T_w_b.p and T_w_b.R (as a quaternion); v_w_b and bias are not stored.
  common::Status write_gt(const common::NavState& s);
  common::Status write_lidar(const common::ScanFrame& scan);
  common::Status finish();

 private:
  struct Impl;
  explicit DatasetWriter(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace mss::io
