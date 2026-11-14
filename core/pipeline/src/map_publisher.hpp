#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mss/common/scan_frame.hpp"
#include "mss/common/types.hpp"
#include "mss/map/config.hpp"
#include "mss/map/voxel_map.hpp"
#include "mss/pipeline/snapshot_slots.hpp"

#include "keyframe_store.hpp"

namespace mss::pipeline::detail {

/// Buffers of the front-end map snapshot (ADR-033 decision 3): three, the count the ADR sets
/// when only the front-end holds a snapshot (one published, one held, one being written).
constexpr std::size_t kMapSnapshotBuffers = 3;
/// Map operations kept for bringing a lagging buffer up to date, beyond the window_keyframes
/// inserts of one rebuild.
constexpr std::size_t kMapLogExtraOps = 64;

/// One snapshot buffer: the map and the number of logged operations it holds.
struct MapBuffer {
  explicit MapBuffer(const map::MapConfig& cfg) : map(cfg) {}
  map::VoxelMap map;
  /// Writer only: `map` holds the operations 1..applied of MapPublisher's log.
  std::uint64_t applied = 0;
};
using MapSlots = SnapshotSlots<MapBuffer, kMapSnapshotBuffers>;
/// A held map snapshot (MapSlots::Ref): the front-end holds one for a whole scan.
using MapRef = MapSlots::Ref;

/// Writer of the front-end map snapshots (ADR-033 decision 3, ADR-016): the backend applies
/// every VoxelMap change through it and publishes the result; readers acquire() the latest
/// published map and never change it.
///
/// Bringing a free buffer up to date: every operation (insert a keyframe cloud, prune, clear)
/// is numbered and logged; begin() replays onto the free buffer the logged operations it
/// lacks, then the update's own operations are applied to it and logged, and publish() makes
/// it the latest snapshot. A VoxelMap is deterministic, so a buffer that replays the same
/// operations from the same state is identical, bit for bit, to the one map of a single-buffer
/// pipeline. Replay is cheaper than copying: the map arrays are reserved for map.max_voxels
/// (~150 MB with the default config), a keyframe insert touches a few thousand points. A clear
/// (map rebuild) empties the log first: it resets any state, so a buffer lagging behind it
/// replays from it. A buffer that lacks an operation the log no longer holds (log full) or
/// whose keyframe cloud the store has evicted is copied from the published buffer instead
/// (n_full_copies; no allocation: the arrays have the same sizes).
///
/// When no buffer is free (only if a reader holds more than one snapshot), begin() returns
/// false and the update is dropped (n_publish_skipped); lost_update() then asks the caller to
/// rebuild the map, which resets it.
///
/// Memory: kMapSnapshotBuffers maps, allocated at construction. No allocation afterwards.
/// Threading: begin(), the operations and publish() from the writer thread only (the
/// backend); acquire() from any thread. Internal to the pipeline.
class MapPublisher {
 public:
  /// Allocates the buffers (throws as VoxelMap's constructor) and publishes the empty map.
  /// `store` (keyframe clouds for replay) and `window_keyframes` (inserts of one rebuild) come
  /// from the owner; `store` must outlive the publisher.
  MapPublisher(const map::MapConfig& cfg, const KeyframeStore& store, std::size_t window_keyframes);
  MapPublisher(const MapPublisher&) = delete;
  MapPublisher& operator=(const MapPublisher&) = delete;

  /// Any thread. The latest published map (never empty: the empty map is published first).
  [[nodiscard]] MapRef acquire() const { return slots_.acquire(); }

  /// Starts one update: takes a free buffer and brings it up to date. False when no buffer is
  /// free: the operations until the next begin() do nothing and publish() publishes nothing.
  bool begin();
  /// Inserts `cloud_l` (keyframe `id`, the cloud the store keeps for it) at T_w_l.
  void insert(common::KeyId id, const common::ScanFrame& cloud_l, const common::Pose& T_w_l);
  /// VoxelMap::prune around the body position p_w_b.
  void prune(const common::Vec3& p_w_b);
  /// VoxelMap::clear (start of a rebuild, ADR-016). Clears lost_update().
  void clear();
  /// Makes the updated buffer the latest snapshot.
  void publish();

  /// Writer only. The map being updated (between a successful begin() and publish()), else
  /// the published one: the map the operations so far produced.
  [[nodiscard]] const map::VoxelMap& map() const;
  /// An update was dropped since the last clear(): the map lacks it.
  [[nodiscard]] bool lost_update() const { return lost_update_; }
  [[nodiscard]] std::uint64_t n_publish_skipped() const { return n_publish_skipped_; }
  [[nodiscard]] std::uint64_t n_full_copies() const { return n_full_copies_; }

 private:
  enum class OpKind : std::uint8_t { kInsert, kPrune, kClear };
  struct Op {
    OpKind kind = OpKind::kClear;
    common::KeyId id = 0;                       // kInsert
    common::Pose T_w_l;                         // kInsert
    common::Vec3 p_w_b = common::Vec3::Zero();  // kPrune
  };

  // Brings `b` to the end of the log. Precondition: the published buffer is there already.
  void catch_up(MapBuffer& b);
  // Applies a logged op to `m`; false when it needs a cloud the store no longer holds.
  bool replay(const Op& op, map::VoxelMap& m) const;
  void log(const Op& op);

  MapSlots slots_;
  const KeyframeStore& store_;
  // Logged operations; log_[k] has number first_op_ + k. Reserved once, never grows.
  std::vector<Op> log_;
  std::size_t log_capacity_ = 0;
  std::uint64_t first_op_ = 1;
  std::uint64_t next_op_ = 1;
  MapBuffer* writing_ = nullptr;          // the buffer of the update in progress
  const MapBuffer* published_ = nullptr;  // the buffer last published
  bool lost_update_ = false;
  std::uint64_t n_publish_skipped_ = 0;
  std::uint64_t n_full_copies_ = 0;
};

}  // namespace mss::pipeline::detail
