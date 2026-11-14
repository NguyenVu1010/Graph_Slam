#include "map_publisher.hpp"

#include <utility>

namespace mss::pipeline::detail {

MapPublisher::MapPublisher(const map::MapConfig& cfg,
                           const KeyframeStore& store,
                           std::size_t window_keyframes)
    : slots_(std::in_place, cfg), store_(store), log_capacity_(window_keyframes + kMapLogExtraOps) {
  log_.reserve(log_capacity_);
  // Version 1: the empty map, so a reader always finds one.
  MapBuffer* b = slots_.begin_write();
  slots_.publish();
  published_ = b;
}

bool MapPublisher::begin() {
  writing_ = slots_.begin_write();
  if (writing_ == nullptr) {
    ++n_publish_skipped_;
    lost_update_ = true;
    return false;
  }
  catch_up(*writing_);
  return true;
}

void MapPublisher::catch_up(MapBuffer& b) {
  const std::uint64_t last = next_op_ - 1;
  if (b.applied == last) {
    return;
  }
  bool ok = true;
  std::size_t from = 0;
  if (b.applied + 1 >= first_op_) {
    from = static_cast<std::size_t>(b.applied + 1 - first_op_);
  } else {
    // The log lost operations `b` lacks; a clear at its start resets any state.
    ok = !log_.empty() && log_.front().kind == OpKind::kClear;
  }
  for (std::size_t k = from; ok && k < log_.size(); ++k) {
    ok = replay(log_[k], b.map);
  }
  if (!ok) {
    // Same array sizes: copy assignment allocates nothing.
    b.map = published_->map;
    ++n_full_copies_;
  }
  b.applied = last;
}

bool MapPublisher::replay(const Op& op, map::VoxelMap& m) const {
  switch (op.kind) {
    case OpKind::kInsert: {
      const KeyframeEntry* e = store_.find(op.id);
      if (e == nullptr) {
        return false;
      }
      m.insert(e->cloud_l, op.T_w_l);
      return true;
    }
    case OpKind::kPrune:
      m.prune(op.p_w_b);
      return true;
    case OpKind::kClear:
      m.clear();
      return true;
  }
  return false;
}

void MapPublisher::log(const Op& op) {
  if (op.kind == OpKind::kClear) {
    log_.clear();  // every earlier operation is superseded
    first_op_ = next_op_;
  } else if (log_.size() == log_capacity_) {
    log_.erase(log_.begin());  // no allocation; the capacity is small
    ++first_op_;
  }
  log_.push_back(op);
  ++next_op_;
  writing_->applied = next_op_ - 1;
}

void MapPublisher::insert(common::KeyId id,
                          const common::ScanFrame& cloud_l,
                          const common::Pose& T_w_l) {
  if (writing_ == nullptr) {
    return;
  }
  writing_->map.insert(cloud_l, T_w_l);
  Op op;
  op.kind = OpKind::kInsert;
  op.id = id;
  op.T_w_l = T_w_l;
  log(op);
}

void MapPublisher::prune(const common::Vec3& p_w_b) {
  if (writing_ == nullptr) {
    return;
  }
  writing_->map.prune(p_w_b);
  Op op;
  op.kind = OpKind::kPrune;
  op.p_w_b = p_w_b;
  log(op);
}

void MapPublisher::clear() {
  if (writing_ == nullptr) {
    return;
  }
  writing_->map.clear();
  Op op;
  op.kind = OpKind::kClear;
  log(op);
  lost_update_ = false;
}

void MapPublisher::publish() {
  if (writing_ != nullptr) {
    slots_.publish();
    published_ = writing_;
  }
  writing_ = nullptr;
}

const map::VoxelMap& MapPublisher::map() const {
  return writing_ != nullptr ? writing_->map : published_->map;
}

}  // namespace mss::pipeline::detail
