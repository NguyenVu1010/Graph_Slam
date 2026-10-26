#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>

#include "mss/common/result.hpp"
#include "mss/common/scan_frame.hpp"
#include "mss/io/dataset_format.hpp"

namespace mss::io {

/// Reads an `mssdata` directory (docs/mssdata_format.md) and replays its five streams
/// merged by time.
///
/// Merge order: records come out by `t` ascending (lidar by t_start). Records with equal
/// `t` in different streams come out in the fixed order imu, odom, gnss, gt, lidar.
///
/// Memory: all buffers (one read chunk per record stream, one AoS staging buffer of
/// `max_points_per_scan` LidarPoints, the lidar path buffer) are allocated in open();
/// next() does not allocate on its success path. Files are read with POSIX read(2), so no
/// stdio buffers are allocated per scan.
///
/// Errors: open() returns an error for a missing or malformed index.json, an unsupported
/// version, a missing file, a file whose size is not a multiple of its record size or does
/// not match the count in index.json, or a big-endian host. next() returns an error naming
/// the file for a malformed lidar file (shorter than 12 B, n > max_points_per_scan,
/// size != 12 + n * 22), a short read, a time that goes backwards within a stream, or a
/// scan larger than the caller's ScanFrame capacity. After an error the reader is stuck:
/// every later next() returns the same error. Nothing throws.
///
/// Movable, not copyable. Not thread-safe.
class DatasetReader {
 public:
  static common::Result<DatasetReader> open(const std::filesystem::path& dir,
                                            std::size_t max_points_per_scan);

  DatasetReader(DatasetReader&& other) noexcept;
  DatasetReader& operator=(DatasetReader&& other) noexcept;
  DatasetReader(const DatasetReader&) = delete;
  DatasetReader& operator=(const DatasetReader&) = delete;
  ~DatasetReader();

  /// Precondition: the reader was not moved from.
  const DatasetIndex& index() const;

  /// ok(true): `out` holds the next record (for kLidar, `scan` holds the points).
  /// ok(false): all streams are exhausted. error(reason): see the class comment.
  common::Result<bool> next(Record& out, common::ScanFrame& scan);

 private:
  struct Impl;
  explicit DatasetReader(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace mss::io
