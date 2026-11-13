#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "mss2d/messages.hpp"
#include "mss2d/small_matrix.hpp"

namespace mss2d {

/// Cartesian scan points in the LiDAR frame l, m. Fixed storage; `count` are valid.
struct PointCloud2d {
  std::uint32_t count = 0;
  std::array<Vec2f, kMaxScanPoints> pts{};
};

/// Polar -> Cartesian in frame l, keeping beams with min_range_m <= range <= max_range_m.
/// A zero or NaN range means "no return" and is dropped. No deskew here.
inline void scan_to_points(const Scan2d& scan,
                           float min_range_m,
                           float max_range_m,
                           PointCloud2d& out) {
  out.count = 0;
  for (std::uint32_t i = 0; i < scan.count && i < kMaxScanPoints; ++i) {
    const ScanPoint2d& sp = scan.points[i];
    if (!(sp.range_m >= min_range_m && sp.range_m <= max_range_m && sp.range_m > 0.0F)) {
      continue;
    }
    out.pts[out.count++] =
        Vec2f{sp.range_m * std::cos(sp.angle_rad), sp.range_m * std::sin(sp.angle_rad)};
  }
}

}  // namespace mss2d
