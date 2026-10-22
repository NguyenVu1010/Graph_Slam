#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"

namespace mss::common {

// One LiDAR scan as a structure of arrays. Memory is allocated only in reserve()
// (or the capacity constructor); push(), clear(), copy_from(), set_size() and the
// mutable accessors never allocate, so the object is safe to use on the hot path.
// Not copyable (copies must be explicit via copy_from); movable.
//
// Units: x, y, z in metres in the LiDAR frame l; intensity as reported by the sensor;
// t_offset_ns is nanoseconds since t_start (sensor clock); ring is the laser index.
class ScanFrame {
 public:
  ScanFrame() = default;
  explicit ScanFrame(std::size_t capacity) { reserve(capacity); }
  ScanFrame(const ScanFrame&) = delete;
  ScanFrame& operator=(const ScanFrame&) = delete;
  ScanFrame(ScanFrame&&) noexcept = default;
  ScanFrame& operator=(ScanFrame&&) noexcept = default;

  // Allocates storage for `capacity` points. Intended to be called once. Calling it
  // again with a different capacity is allowed only when size() == 0 and reallocates;
  // when the frame is not empty the call is ignored.
  void reserve(std::size_t capacity);

  std::size_t capacity() const { return capacity_; }
  std::size_t size() const { return size_; }
  void clear() { size_ = 0; }

  // Appends one point. Returns false (and changes nothing) when the frame is full.
  bool push(
      float x, float y, float z, float intensity, std::uint32_t t_offset_ns, std::uint16_t ring);

  // Copies t_start and all points with memcpy. Returns false (and changes nothing)
  // when capacity() < other.size().
  bool copy_from(const ScanFrame& other);

  Timestamp t_start{};  // sensor-clock time of the first point

  Span<const float> xyz() const { return {xyz_.data(), 3 * size_}; }  // x0 y0 z0 x1 ...
  Span<const float> intensity() const { return {intensity_.data(), size_}; }
  Span<const std::uint32_t> t_offset_ns() const { return {t_offset_ns_.data(), size_}; }
  Span<const std::uint16_t> ring() const { return {ring_.data(), size_}; }

  // Bulk-write access over the full capacity (xyz_mut has 3 * capacity floats).
  // After writing n points call set_size(n).
  float* xyz_mut() { return xyz_.data(); }
  float* intensity_mut() { return intensity_.data(); }
  std::uint32_t* t_offset_ns_mut() { return t_offset_ns_.data(); }
  std::uint16_t* ring_mut() { return ring_.data(); }

  // Sets the number of valid points. Returns false (and changes nothing) if n > capacity().
  bool set_size(std::size_t n);

 private:
  std::vector<float> xyz_;
  std::vector<float> intensity_;
  std::vector<std::uint32_t> t_offset_ns_;
  std::vector<std::uint16_t> ring_;
  std::size_t capacity_ = 0;
  std::size_t size_ = 0;
};

}  // namespace mss::common
