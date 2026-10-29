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

struct Accumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t n = 0;
};

}  // namespace

void voxel_centroids(std::vector<Eigen::Vector3f>& points, double voxel_m) {
  if (!(voxel_m > 0.0) || points.empty()) {
    return;
  }
  const double inv = 1.0 / voxel_m;
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> index;
  index.reserve(points.size());
  std::vector<Accumulator> acc;
  acc.reserve(points.size());
  for (const Eigen::Vector3f& p : points) {
    const Eigen::Vector3d pd = p.cast<double>();
    const VoxelKey key{static_cast<std::int64_t>(std::floor(pd.x() * inv)),
                       static_cast<std::int64_t>(std::floor(pd.y() * inv)),
                       static_cast<std::int64_t>(std::floor(pd.z() * inv))};
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
