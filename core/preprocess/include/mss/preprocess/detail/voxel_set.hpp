#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mss::preprocess::detail {

/// Integer voxel index (docs/derivations/preprocess.md section 5).
struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

/// floor(coord / voxel_m) per axis, coordinates widened to double. Units: metres.
/// Precondition: voxel_m > 0 and finite coordinates.
VoxelKey voxel_key(float x, float y, float z, double voxel_m) noexcept;

/// Set of voxel keys, open addressing with linear probing, sized once.
/// Ownership/threading: owns two vectors allocated once in the constructor; one per
/// Preprocessor, used on the front-end thread only.
class VoxelSet {
 public:
  /// Reserves room for `max_keys` keys (load factor <= 0.5). Allocates.
  /// Errors: throws std::invalid_argument if max_keys is 0 or > UINT32_MAX - 1.
  explicit VoxelSet(std::size_t max_keys);

  /// Forgets every key. O(slot count), no allocation.
  void clear() noexcept;
  /// Returns true if `key` was not in the set (and adds it). Precondition: size() < max_keys.
  /// No allocation.
  bool insert(const VoxelKey& key) noexcept;
  std::size_t size() const noexcept { return size_; }

 private:
  std::vector<std::uint32_t> slots_;  // index into keys_, kEmptySlot when free
  std::vector<VoxelKey> keys_;        // sized max_keys once
  std::size_t size_ = 0;
  std::uint64_t mask_ = 0;  // slots_.size() - 1 (power of two)
};

}  // namespace mss::preprocess::detail
