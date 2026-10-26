#include "keyframe_store.hpp"

#include <stdexcept>

namespace mss::pipeline::detail {

KeyframeStore::KeyframeStore(std::size_t max_keyframes) {
  if (max_keyframes == 0) {
    throw std::invalid_argument("map.max_keyframes: must be > 0");
  }
  slots_.resize(max_keyframes);
}

common::Status KeyframeStore::add(common::KeyId id,
                                  common::Timestamp t,
                                  const common::ScanFrame& cloud_l) {
  if (size_ > 0 && id <= at(size_ - 1).id) {
    return common::Status::error("keyframe id not increasing");
  }
  if (size_ == slots_.size()) {
    head_ = (head_ + 1) % slots_.size();  // evict the oldest
    --size_;
    ++n_evicted_;
  }
  KeyframeEntry& slot = slots_[(head_ + size_) % slots_.size()];
  if (slot.cloud_l.capacity() < cloud_l.size()) {
    slot.cloud_l.clear();
    slot.cloud_l.reserve(cloud_l.size());  // allocates: only when a slot grows
  }
  (void)slot.cloud_l.copy_from(cloud_l);  // capacity checked above, cannot fail
  slot.id = id;
  slot.t = t;
  ++size_;
  return common::Status::ok();
}

const KeyframeEntry* KeyframeStore::find(common::KeyId id) const {
  std::size_t lo = 0;
  std::size_t hi = size_;
  while (lo < hi) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (at(mid).id < id) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo < size_ && at(lo).id == id ? &at(lo) : nullptr;
}

}  // namespace mss::pipeline::detail
