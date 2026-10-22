#include "mss/common/scan_frame.hpp"

#include <cstring>

namespace mss::common {

void ScanFrame::reserve(std::size_t capacity) {
  if (capacity == capacity_ || size_ != 0) {
    return;
  }
  xyz_.assign(3 * capacity, 0.0F);
  intensity_.assign(capacity, 0.0F);
  t_offset_ns_.assign(capacity, 0U);
  ring_.assign(capacity, 0U);
  capacity_ = capacity;
}

bool ScanFrame::push(
    float x, float y, float z, float intensity, std::uint32_t t_offset_ns, std::uint16_t ring) {
  if (size_ >= capacity_) {
    return false;
  }
  float* p = xyz_.data() + 3 * size_;
  p[0] = x;
  p[1] = y;
  p[2] = z;
  intensity_[size_] = intensity;
  t_offset_ns_[size_] = t_offset_ns;
  ring_[size_] = ring;
  ++size_;
  return true;
}

bool ScanFrame::copy_from(const ScanFrame& other) {
  if (capacity_ < other.size_) {
    return false;
  }
  const std::size_t n = other.size_;
  if (n > 0) {
    std::memcpy(xyz_.data(), other.xyz_.data(), 3 * n * sizeof(float));
    std::memcpy(intensity_.data(), other.intensity_.data(), n * sizeof(float));
    std::memcpy(t_offset_ns_.data(), other.t_offset_ns_.data(), n * sizeof(std::uint32_t));
    std::memcpy(ring_.data(), other.ring_.data(), n * sizeof(std::uint16_t));
  }
  size_ = n;
  t_start = other.t_start;
  return true;
}

bool ScanFrame::set_size(std::size_t n) {
  if (n > capacity_) {
    return false;
  }
  size_ = n;
  return true;
}

}  // namespace mss::common
