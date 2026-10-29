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
/// The data is written to `<path>.tmp` and renamed over `path`, so a failed write never leaves a
/// truncated `path` (an existing file stays). An empty cloud writes a valid header with 0 points.
/// Errors (nothing thrown): Status error naming the path when `path` is empty, names an
///   existing directory, its parent directory does not exist, the host is big-endian, or the
///   temporary file cannot be opened, written or closed, or the rename fails (the temporary file is
///   removed).
/// Allocation: none beyond the stream buffer and the path strings. Hot path: no.
/// Tests: test_dataset_io (Pcd*).
common::Status write_pcd(const std::filesystem::path& path,
                         common::Span<const Eigen::Vector3f> points);

}  // namespace mss::io
