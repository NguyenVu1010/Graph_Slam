#pragma once

#include "mss2d/messages.hpp"

namespace mss2d {

/// One complete LiDAR turn at a time (ADR-034 decision 2). A driver (RPLIDAR A1/A2 packet
/// parser on the MCU, a recording or a simulator on the host) assembles the points of one
/// revolution, stamps t_start on the sensor clock and the per-point t_offset_ns, and hands the
/// turn over; the caller pushes it into Merger::push_scan(). Angles are counter-clockwise from
/// +x of frame l, whatever the sensor's own convention.
class LidarSource {
 public:
  /// Fills `out` with the next complete turn and returns true, or returns false when none
  /// is ready yet (or the source has ended). Must not allocate.
  virtual bool next_scan(Scan2d& out) = 0;

 protected:
  ~LidarSource() = default;
};

}  // namespace mss2d
