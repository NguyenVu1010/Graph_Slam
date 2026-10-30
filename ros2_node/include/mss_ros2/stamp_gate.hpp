#pragma once

#include "mss/common/timestamp.hpp"

namespace mss_ros2 {

/// Strictly increasing stamp filter of one topic. Checking and committing are separate so a
/// message that fails a later step (the cloud conversion) does not advance the stamp: a
/// malformed message from a second publisher must not block the good ones (backlog R4).
class StampGate {
 public:
  /// True when `t` is later than the last committed stamp (or none was committed yet).
  bool is_new(mss::common::Timestamp t) const { return !has_last_ || t > last_; }
  /// Records `t` as the last accepted stamp. Call only for a message that is being used.
  void commit(mss::common::Timestamp t) {
    has_last_ = true;
    last_ = t;
  }
  /// Last committed stamp; meaningful only when has_last().
  mss::common::Timestamp last() const { return last_; }
  bool has_last() const { return has_last_; }

 private:
  bool has_last_ = false;
  mss::common::Timestamp last_{};
};

}  // namespace mss_ros2
