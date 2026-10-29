#pragma once

#include <filesystem>

#include <Eigen/Core>

#include "mss/common/result.hpp"
#include "mss/common/span.hpp"

namespace mss::io {

/// Writes `points` as a PCD v0.7 file (Point Cloud Library format, readable by CloudCompare
/// and pcl_viewer): header `FIELDS x y z`, `SIZE 4 4 4`, `TYPE F F F`, `COUNT 1 1 1`,
/// `WIDTH n`, `HEIGHT 1`, `VIEWPOINT 0 0 0 1 0 0 0`, `POINTS n`, `DATA binary`, then n
/// records of three little-endian float32 (x, y, z). The coordinates are written as given;
/// the caller states their frame (the pipeline's map export is in w) and unit (m).
/// Durability guarantee: the data is written to `<path>.tmp`, flushed to the device (fsync on
/// POSIX; Windows has no flush here) and then renamed over `path`. So `path` is either the
/// previous file or the complete new one, also after a process crash or, on POSIX, a power
/// loss; on any error the temporary file is removed and an existing `path` is untouched. An
/// empty cloud writes a valid header with 0 points.
/// Errors (nothing thrown): Status error naming the path when `path` is empty, names an
///   existing directory, its parent directory does not exist, the host is big-endian, or the
///   temporary file cannot be opened, written, flushed or closed, or the rename fails.
/// Allocation: none beyond the stream buffer and the path strings. Hot path: no.
/// Tests: test_dataset_io (Pcd*).
common::Status write_pcd(const std::filesystem::path& path,
                         common::Span<const Eigen::Vector3f> points);

namespace detail {
/// Test seam: makes write_pcd() fail at one step so the cleanup of the temporary file can be
/// tested. kNone (the default) is the normal behaviour.
enum class PcdFault { kNone = 0, kWriteFail, kSyncFail, kRenameFail };
void set_pcd_fault(PcdFault f);
}  // namespace detail

}  // namespace mss::io
