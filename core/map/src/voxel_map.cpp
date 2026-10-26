#include "mss/map/voxel_map.hpp"

#include <algorithm>
#include <array>
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
  const auto n = static_cast<std::size_t>(cfg.max_points_per_voxel);
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
  const auto n_per_voxel = static_cast<std::size_t>(cfg_.max_points_per_voxel);
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const common::Vec3 p_l(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    const common::Vec3 p_w = T_w_l * p_l;
    // Stored as float; the key comes from the rounded value so the point lies in its voxel.
    const auto fx = static_cast<float>(p_w.x());
    const auto fy = static_cast<float>(p_w.y());
    const auto fz = static_cast<float>(p_w.z());
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

int VoxelMap::knn(const common::Vec3& p_w, int k, NeighborBuffer& out) const {
  out.size = 0;
  const int k_eff = std::min(k <= 0 ? cfg_.knn_k : k, NeighborBuffer::kMaxNeighbors);
  const std::optional<Key> q = key_of(p_w);
  if (!q) {
    return 0;
  }
  const auto n_per_voxel = static_cast<std::size_t>(cfg_.max_points_per_voxel);
  // Squared distances of out.p_w[0, n) in double: ranking is done in double, out.dist2 is float.
  std::array<double, NeighborBuffer::kMaxNeighbors> d2s{};
  int n = 0;
  for (std::int32_t dz = -1; dz <= 1; ++dz) {
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
      for (std::int32_t dx = -1; dx <= 1; ++dx) {
        const std::uint32_t v = find_voxel((*q)[0] + dx, (*q)[1] + dy, (*q)[2] + dz);
        if (v == kEmptySlot) {
          continue;
        }
        const VoxelMeta& meta = voxels_[v];
        const float* pts = &points_[static_cast<std::size_t>(v) * n_per_voxel * 3];
        for (std::int32_t s = 0; s < meta.count; ++s) {
          const float* p = pts + static_cast<std::size_t>(s) * 3;
          const double ex = static_cast<double>(p[0]) - p_w.x();
          const double ey = static_cast<double>(p[1]) - p_w.y();
          const double ez = static_cast<double>(p[2]) - p_w.z();
          const double d2 = ex * ex + ey * ey + ez * ez;
          // Strict <: on equal d2 the earlier candidate (visit order, then slot) stays first.
          if (n == k_eff && !(d2 < d2s[static_cast<std::size_t>(n - 1)])) {
            continue;
          }
          int j = (n < k_eff) ? n++ : n - 1;
          while (j > 0 && d2 < d2s[static_cast<std::size_t>(j - 1)]) {
            const auto u = static_cast<std::size_t>(j);
            d2s[u] = d2s[u - 1];
            out.p_w[u] = out.p_w[u - 1];
            --j;
          }
          const auto u = static_cast<std::size_t>(j);
          d2s[u] = d2;
          out.p_w[u] = common::Vec3(p[0], p[1], p[2]);
        }
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    const auto u = static_cast<std::size_t>(i);
    out.dist2[u] = static_cast<float>(d2s[u]);
  }
  out.size = n;
  return n;
}

void VoxelMap::erase_slot(std::size_t slot) {
  // Linear probing without tombstones: move later entries of the chain back into the hole
  // unless their home lies cyclically in (hole, j].
  std::size_t i = slot;
  std::size_t j = slot;
  while (true) {
    j = (j + 1) & mask_;
    const HashSlot& next = slots_[j];
    if (next.voxel == kEmptySlot) {
      break;
    }
    const std::size_t h = home_slot(next.kx, next.ky, next.kz);
    if (((j - h) & mask_) >= ((j - i) & mask_)) {
      slots_[i] = next;
      i = j;
    }
  }
  slots_[i].voxel = kEmptySlot;
}

void VoxelMap::clear() {
  std::fill(slots_.begin(), slots_.end(), HashSlot{});
  std::fill(voxels_.begin(), voxels_.end(), VoxelMeta{});
  // points_ is not zeroed: only the `count` entries of a used voxel are ever read.
  const std::size_t n_voxels = voxels_.size();
  for (std::size_t i = 0; i < n_voxels; ++i) {
    free_[i] = static_cast<std::uint32_t>(n_voxels - 1 - i);  // index 0 popped first
  }
  free_top_ = n_voxels;
  num_voxels_ = 0;
  num_points_ = 0;
  dropped_ = 0;
}

void VoxelMap::prune(const common::Vec3& p_w_b) {
  const double r2 = cfg_.prune_radius_m * cfg_.prune_radius_m;
  for (std::size_t v = 0; v < voxels_.size(); ++v) {
    VoxelMeta& meta = voxels_[v];
    if (!meta.used) {
      continue;
    }
    const common::Vec3 centre_w((meta.kx + 0.5) * cfg_.voxel_m,
                                (meta.ky + 0.5) * cfg_.voxel_m,
                                (meta.kz + 0.5) * cfg_.voxel_m);
    const common::Vec3 d = centre_w - p_w_b;
    if (!(d.squaredNorm() > r2)) {
      continue;
    }
    erase_slot(find_slot(meta.kx, meta.ky, meta.kz));
    --num_voxels_;
    num_points_ -= static_cast<std::size_t>(meta.count);
    meta.count = 0;
    meta.head = 0;
    meta.used = false;
    free_[free_top_++] = static_cast<std::uint32_t>(v);
  }
}

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
