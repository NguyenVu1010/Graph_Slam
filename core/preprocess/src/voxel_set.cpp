#include "mss/preprocess/detail/voxel_set.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mss::preprocess::detail {

namespace {
constexpr std::uint32_t kEmptySlot = std::numeric_limits<std::uint32_t>::max();
// Slots per key: keeps the load factor <= 0.5 so linear probes stay short.
constexpr std::size_t kSlotsPerKey = 2;
// Spatial hash primes of Teschner et al. 2003 (the iVox / Faster-LIO hash, ADR-007).
constexpr std::uint64_t kHashPrimeX = 73856093;
constexpr std::uint64_t kHashPrimeY = 19349669;
constexpr std::uint64_t kHashPrimeZ = 83492791;

std::uint64_t hash(const VoxelKey& k) noexcept {
  // Unsigned arithmetic: wrap-around is well defined.
  return (static_cast<std::uint64_t>(k.x) * kHashPrimeX) ^
         (static_cast<std::uint64_t>(k.y) * kHashPrimeY) ^
         (static_cast<std::uint64_t>(k.z) * kHashPrimeZ);
}
}  // namespace

VoxelKey voxel_key(float x, float y, float z, double voxel_m) noexcept {
  return VoxelKey{static_cast<std::int64_t>(std::floor(static_cast<double>(x) / voxel_m)),
                  static_cast<std::int64_t>(std::floor(static_cast<double>(y) / voxel_m)),
                  static_cast<std::int64_t>(std::floor(static_cast<double>(z) / voxel_m))};
}

VoxelSet::VoxelSet(std::size_t max_keys) {
  if (max_keys == 0 || max_keys > static_cast<std::size_t>(kEmptySlot) - 1) {
    throw std::invalid_argument("VoxelSet max_keys must be in [1, UINT32_MAX - 1]");
  }
  std::size_t slot_count = 1;
  while (slot_count < kSlotsPerKey * max_keys) {
    slot_count *= 2;
  }
  slots_.assign(slot_count, kEmptySlot);
  keys_.resize(max_keys);
  mask_ = static_cast<std::uint64_t>(slot_count - 1);
}

void VoxelSet::clear() noexcept {
  std::fill(slots_.begin(), slots_.end(), kEmptySlot);
  size_ = 0;
}

bool VoxelSet::insert(const VoxelKey& key) noexcept {
  std::uint64_t i = hash(key) & mask_;
  while (true) {
    std::uint32_t& slot = slots_[static_cast<std::size_t>(i)];
    if (slot == kEmptySlot) {
      keys_[size_] = key;
      slot = static_cast<std::uint32_t>(size_);
      ++size_;
      return true;
    }
    if (keys_[slot] == key) {
      return false;
    }
    i = (i + 1) & mask_;
  }
}

}  // namespace mss::preprocess::detail
