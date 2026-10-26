#include "mss/map/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"

// Design and invariants: docs/derivations/map.md.

namespace mss::map {

namespace {

// Largest pool size: pool indices are 32-bit and kEmptySlot must stay unused.
constexpr std::size_t kMaxVoxelsLimit = std::size_t{1} << 30;

// Multipliers of the spatial hash (Teschner et al. 2003, as in iVox).
constexpr std::uint32_t kHashX = 73856093u;
constexpr std::uint32_t kHashY = 19349669u;
constexpr std::uint32_t kHashZ = 83492791u;

template <typename T>
[[noreturn]] void reject(const char* key, const char* rule, const T& got) {
  std::ostringstream os;
  os << "map." << key << " must be " << rule << ", got " << got;
  throw std::invalid_argument(os.str());
}

}  // namespace

VoxelMap::VoxelMap(const MapConfig& cfg) : cfg_(cfg) {
  if (!(std::isfinite(cfg.voxel_m) && cfg.voxel_m > 0.0)) {
    reject("voxel_m", "finite and > 0", cfg.voxel_m);
  }
  if (cfg.max_points_per_voxel <= 0) {
    reject("max_points_per_voxel", "> 0", cfg.max_points_per_voxel);
  }
  if (cfg.knn_k <= 0 || cfg.knn_k > NeighborBuffer::kMaxNeighbors) {
    reject("knn_k", "in [1, 32]", cfg.knn_k);
  }
  if (cfg.max_voxels == 0 || cfg.max_voxels > kMaxVoxelsLimit) {
    reject("max_voxels", "in [1, 2^30]", cfg.max_voxels);
  }
  if (!(std::isfinite(cfg.prune_radius_m) && cfg.prune_radius_m > 0.0)) {
    reject("prune_radius_m", "finite and > 0", cfg.prune_radius_m);
  }
  const std::size_t n = static_cast<std::size_t>(cfg.max_points_per_voxel);
  if (n > std::numeric_limits<std::size_t>::max() / 3 / cfg.max_voxels) {
    reject("max_points_per_voxel",
           "small enough that max_voxels * N * 3 floats fit in size_t",
           cfg.max_points_per_voxel);
  }

  std::size_t capacity = 1;
  while (capacity < 2 * cfg.max_voxels) {
    capacity <<= 1;
  }
  slots_.assign(capacity, HashSlot{});
  mask_ = capacity - 1;
  voxels_.assign(cfg.max_voxels, VoxelMeta{});
  points_.assign(cfg.max_voxels * n * 3, 0.0F);
  free_.resize(cfg.max_voxels);
  for (std::size_t i = 0; i < cfg.max_voxels; ++i) {
    free_[i] = static_cast<std::uint32_t>(cfg.max_voxels - 1 - i);  // index 0 popped first
  }
  free_top_ = cfg.max_voxels;
}

std::optional<VoxelMap::Key> VoxelMap::key_of(const common::Vec3& p_w) const {
  constexpr double kLo = static_cast<double>(std::numeric_limits<std::int32_t>::min()) + 1.0;
  constexpr double kHi = static_cast<double>(std::numeric_limits<std::int32_t>::max()) - 1.0;
  Key key{};
  for (int a = 0; a < 3; ++a) {
    const double f = std::floor(p_w[a] / cfg_.voxel_m);
    if (!(f >= kLo && f <= kHi)) {  // also rejects NaN
      return std::nullopt;
    }
    key[static_cast<std::size_t>(a)] = static_cast<std::int32_t>(f);
  }
  return key;
}

std::size_t VoxelMap::home_slot(std::int32_t kx, std::int32_t ky, std::int32_t kz) const {
  const std::uint32_t h = (static_cast<std::uint32_t>(kx) * kHashX) ^
                          (static_cast<std::uint32_t>(ky) * kHashY) ^
                          (static_cast<std::uint32_t>(kz) * kHashZ);
  return static_cast<std::size_t>(h) & mask_;
}

std::size_t VoxelMap::find_slot(std::int32_t kx, std::int32_t ky, std::int32_t kz) const {
  // Load factor <= 0.5, so an empty slot always ends the probe sequence.
  std::size_t s = home_slot(kx, ky, kz);
  while (true) {
    const HashSlot& slot = slots_[s];
    if (slot.voxel == kEmptySlot || (slot.kx == kx && slot.ky == ky && slot.kz == kz)) {
      return s;
    }
    s = (s + 1) & mask_;
  }
}

std::uint32_t VoxelMap::find_voxel(std::int32_t kx, std::int32_t ky, std::int32_t kz) const {
  return slots_[find_slot(kx, ky, kz)].voxel;
}

void VoxelMap::insert(const common::ScanFrame& scan, const common::Pose& T_w_l) {
  const common::Span<const float> xyz = scan.xyz();
  const std::size_t n_per_voxel = static_cast<std::size_t>(cfg_.max_points_per_voxel);
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const common::Vec3 p_l(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    const common::Vec3 p_w = T_w_l * p_l;
    // Stored as float; the key comes from the rounded value so the point lies in its voxel.
    const float fx = static_cast<float>(p_w.x());
    const float fy = static_cast<float>(p_w.y());
    const float fz = static_cast<float>(p_w.z());
    const std::optional<Key> key = key_of(common::Vec3(fx, fy, fz));
    if (!key) {
      continue;
    }
    const std::size_t s = find_slot((*key)[0], (*key)[1], (*key)[2]);
    std::uint32_t v = slots_[s].voxel;
    if (v == kEmptySlot) {
      if (num_voxels_ == cfg_.max_voxels) {
        ++dropped_;
        continue;
      }
      v = free_[--free_top_];
      slots_[s] = HashSlot{(*key)[0], (*key)[1], (*key)[2], v};
      voxels_[v] = VoxelMeta{(*key)[0], (*key)[1], (*key)[2], 0, 0, true};
      ++num_voxels_;
    }
    VoxelMeta& meta = voxels_[v];
    float* dst = &points_[((static_cast<std::size_t>(v) * n_per_voxel) +
                           static_cast<std::size_t>(meta.head)) *
                          3];
    dst[0] = fx;
    dst[1] = fy;
    dst[2] = fz;
    meta.head = (meta.head + 1) % cfg_.max_points_per_voxel;
    if (meta.count < cfg_.max_points_per_voxel) {
      ++meta.count;
      ++num_points_;
    }
  }
}

int VoxelMap::knn(const common::Vec3& /*p_w*/, int /*k*/, NeighborBuffer& out) const {
  out.size = 0;
  return 0;
}

void VoxelMap::prune(const common::Vec3& /*p_w_b*/) {}

std::size_t VoxelMap::num_voxels() const {
  return num_voxels_;
}

std::size_t VoxelMap::num_dropped() const {
  return dropped_;
}

std::size_t VoxelMap::num_points() const {
  return num_points_;
}

}  // namespace mss::map
