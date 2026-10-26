#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/config.hpp"

namespace mss::map {

/// Fixed-size result buffer for VoxelMap::knn. Plain value type, no heap memory.
/// Entries `[0, size)` are valid and sorted by ascending `dist2`. Ties are broken by
/// voxel visit order (dz, then dy, then dx, each from -1 to 1, relative to the query's
/// voxel), then by slot order within the voxel. M-01 compares the `dist2` sequence and
/// the point multiset.
struct NeighborBuffer {
  static constexpr int kMaxNeighbors = 32;
  std::array<common::Vec3, kMaxNeighbors> p_w{};  ///< m, neighbour positions in w
  std::array<float, kMaxNeighbors> dist2{};       ///< m^2, squared distance to the query
  int size = 0;                                   ///< valid entries, 0 <= size <= k
};

/// Voxel hash map of LiDAR points in the world frame w (ADR-007, iVox style).
///
/// Key: integer voxel coordinate `floor(p_w / voxel_m)` per axis. Each voxel stores at
/// most `max_points_per_voxel` points in a ring: when full, a new point overwrites the
/// oldest one in that voxel. Open-addressing table with capacity reserved once for
/// `max_voxels` voxels; no allocation after construction.
/// Ownership/threading: single-threaded; one instance owned by pipeline::Estimator.
/// Written and read by the front-end thread; the backend never touches it directly
/// (ADR-005: pose corrections go through a queue and are applied between scans).
class VoxelMap {
 public:
  /// Validates `cfg` and reserves storage for `cfg.max_voxels` voxels of
  /// `cfg.max_points_per_voxel` points each.
  /// Frames: none. Units: see MapConfig.
  /// Errors: throws std::invalid_argument when `voxel_m <= 0`, `max_points_per_voxel <= 0`,
  ///   `knn_k <= 0`, `knn_k > NeighborBuffer::kMaxNeighbors`, `max_voxels == 0` or
  ///   `prune_radius_m <= 0`; also std::invalid_argument when `voxel_m` or
  ///   `prune_radius_m` is not finite, `max_voxels > 2^30`, or the point storage size
  ///   overflows std::size_t; may throw std::bad_alloc.
  /// Hot path: no (allocates).
  /// Tests: M-01, M-02.
  explicit VoxelMap(const MapConfig& cfg);

  /// Inserts every point of `scan` (frame l) into the map after transforming it with
  /// `p_w = T_w_l * p_l` (float coordinates promoted to double).
  /// Points with non-finite coordinates are skipped. A point whose voxel does not exist
  /// yet while the table already holds `max_voxels` voxels is dropped silently (no
  /// per-point log) and counted in num_dropped(), which the pipeline logs off the hot
  /// path.
  /// Frames: `scan` in l (already preprocessed: deskewed to its t_start), `T_w_l` = pose
  ///   of l in w at `scan.t_start`.
  /// Units: metres.
  /// Preconditions: none.
  /// Errors: none (void); bad points are skipped as above.
  /// Hot path: yes, per scan. No heap allocation, no logging, no exceptions.
  /// Tests: M-01 (inserted points are found), M-02 (<= max_points_per_voxel per voxel).
  void insert(const common::ScanFrame& scan, const common::Pose& T_w_l);

  /// Finds up to `k` nearest stored points to `p_w` among the 27 voxels around (and
  /// including) the voxel that contains `p_w`, writes them into `out` sorted by ascending
  /// squared distance and returns how many were found (also stored in `out.size`).
  /// Search scope is exactly those 27 voxels: points farther away are never returned,
  /// even when fewer than `k` were found (no radius limit beyond that).
  /// `k` handling: `k <= 0` means "use MapConfig::knn_k"; `k > kMaxNeighbors` is clamped
  ///   to kMaxNeighbors. Returned count is in [0, min(k_effective, kMaxNeighbors)].
  /// Frames: `p_w` and the results in w. Units: metres, `dist2` in m^2.
  /// Preconditions: `p_w` finite (a non-finite query returns 0).
  /// Errors: none; 0 neighbours is a valid result.
  /// Hot path: yes, per point of every scan-matching iteration. No heap allocation, no
  ///   logging, no exceptions; does not modify the map.
  /// Tests: M-01 (equals brute force restricted to the 27-voxel neighbourhood), M-03.
  int knn(const common::Vec3& p_w, int k, NeighborBuffer& out) const;

  /// Removes every voxel whose centre `(key + 0.5) * voxel_m` (per axis, key = integer
  /// voxel coordinate) is farther than `prune_radius_m` (3-D Euclidean distance) from
  /// `p_w_b`. Freed slots are reused by later inserts.
  /// The caller (pipeline) decides when to call it, every `prune_period_s` of sensor
  /// time; prune() itself keeps no clock.
  /// Frames: `p_w_b` = body position in w. Units: metres.
  /// Errors: none. Does not allocate.
  /// Hot path: no. Runs between scans (iterates the whole table).
  /// Tests: M-02 (voxels outside the radius are removed).
  void prune(const common::Vec3& p_w_b);

  /// Number of occupied voxels, in [0, max_voxels]. O(1). Hot path safe.
  /// Tests: M-02.
  std::size_t num_voxels() const;

  /// Number of points dropped by insert() since construction because their voxel did not
  /// exist and the table was full (`max_voxels`). insert() never logs; the pipeline reads
  /// this counter and logs increases off the hot path. O(1). Hot path safe. Tests: M-02.
  std::size_t num_dropped() const;

  /// Number of stored points over all voxels, <= num_voxels() * max_points_per_voxel.
  /// O(1). Hot path safe. Tests: M-02.
  std::size_t num_points() const;

 private:
  using Key = std::array<std::int32_t, 3>;

  static constexpr std::uint32_t kEmptySlot = 0xFFFFFFFFu;  // HashSlot::voxel of a free slot

  // One open-addressing slot: voxel key and its pool index (kEmptySlot when free).
  struct HashSlot {
    std::int32_t kx = 0;
    std::int32_t ky = 0;
    std::int32_t kz = 0;
    std::uint32_t voxel = kEmptySlot;
  };

  // Pool entry. Its points live in points_ from index ((pool index * N) + slot) * 3.
  struct VoxelMeta {
    std::int32_t kx = 0;
    std::int32_t ky = 0;
    std::int32_t kz = 0;
    std::int32_t count = 0;  // stored points, <= max_points_per_voxel
    std::int32_t head = 0;   // ring slot written next
    bool used = false;
  };

  // Voxel key of `p_w`; nullopt when a coordinate is non-finite or floor(p / voxel_m) lies
  // outside [INT32_MIN + 1, INT32_MAX - 1] (so key +- 1 stays representable).
  std::optional<Key> key_of(const common::Vec3& p_w) const;
  // Pool index of the voxel with this key, or kEmptySlot if it is absent.
  std::uint32_t find_voxel(std::int32_t kx, std::int32_t ky, std::int32_t kz) const;
  // Hash slot where the key's probe sequence starts.
  std::size_t home_slot(std::int32_t kx, std::int32_t ky, std::int32_t kz) const;
  // Slot that holds the key, or the empty slot that ends its probe sequence.
  std::size_t find_slot(std::int32_t kx, std::int32_t ky, std::int32_t kz) const;
  // Empties slots_[slot] by backward-shift deletion, so probe chains stay intact.
  void erase_slot(std::size_t slot);

  MapConfig cfg_;
  std::vector<HashSlot> slots_;      // capacity: power of two >= 2 * max_voxels
  std::size_t mask_ = 0;             // slots_.size() - 1
  std::vector<VoxelMeta> voxels_;    // max_voxels entries
  std::vector<float> points_;        // max_voxels * N * 3 floats, m, frame w
  std::vector<std::uint32_t> free_;  // stack of free pool indices, top at free_[free_top_ - 1]
  std::size_t free_top_ = 0;
  std::size_t num_voxels_ = 0;
  std::size_t num_points_ = 0;
  std::size_t dropped_ = 0;
};

}  // namespace mss::map
