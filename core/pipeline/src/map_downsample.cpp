#include "map_downsample.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace mss::pipeline::detail {
namespace {

struct VoxelKey {
  std::int64_t i = 0;
  std::int64_t j = 0;
  std::int64_t k = 0;
  bool operator==(const VoxelKey& o) const { return i == o.i && j == o.j && k == o.k; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& v) const {
    // Teschner et al. spatial hash primes.
    constexpr std::uint64_t kP1 = 73856093U;
    constexpr std::uint64_t kP2 = 19349663U;
    constexpr std::uint64_t kP3 = 83492791U;
    return static_cast<std::size_t>((static_cast<std::uint64_t>(v.i) * kP1) ^
                                    (static_cast<std::uint64_t>(v.j) * kP2) ^
                                    (static_cast<std::uint64_t>(v.k) * kP3));
  }
};

// Smallest edge accepted (matches the load_config minimum, 1 mm) and the largest voxel index
// magnitude: floor(x * inv) is cast to int64 only below this (2^62 is well inside int64).
constexpr double kMinVoxelM = 0.001;
constexpr double kMaxIndex = 4.0e18;

bool voxel_index(double x, double inv, std::int64_t& out) {
  const double v = std::floor(x * inv);
  if (!(std::abs(v) < kMaxIndex)) {  // NaN, infinity or too large
    return false;
  }
  out = static_cast<std::int64_t>(v);
  return true;
}

struct Accumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t n = 0;
};

}  // namespace

void voxel_centroids(std::vector<Eigen::Vector3f>& points, double voxel_m) {
  if (!(voxel_m >= kMinVoxelM) || points.empty()) {
    return;
  }
  const double inv = 1.0 / voxel_m;
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> index;
  index.reserve(points.size());
  std::vector<Accumulator> acc;
  acc.reserve(points.size());
  for (const Eigen::Vector3f& p : points) {
    const Eigen::Vector3d pd = p.cast<double>();
    VoxelKey key;
    if (!voxel_index(pd.x(), inv, key.i) || !voxel_index(pd.y(), inv, key.j) ||
        !voxel_index(pd.z(), inv, key.k)) {
      continue;  // non-finite or beyond the int64 index range: not exported
    }
    const auto [it, inserted] = index.try_emplace(key, acc.size());
    if (inserted) {
      acc.emplace_back();
    }
    Accumulator& a = acc[it->second];
    a.sum += pd;
    ++a.n;
  }
  points.resize(acc.size());
  for (std::size_t m = 0; m < acc.size(); ++m) {
    const Eigen::Vector3d c = acc[m].sum / static_cast<double>(acc[m].n);
    points[m] = c.cast<float>();
  }
}

}  // namespace mss::pipeline::detail
