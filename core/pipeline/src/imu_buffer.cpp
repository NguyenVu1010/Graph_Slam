#include "imu_buffer.hpp"

namespace mss::pipeline::detail {

ImuBuffer::ImuBuffer() : data_(kMaxImuQueue) {}

bool ImuBuffer::push(const common::ImuSample& s) {
  if (size_ > 0 && s.t <= back().t) {
    return false;
  }
  if (full()) {
    pop_front();
  }
  data_[(head_ + size_) % data_.size()] = s;
  ++size_;
  return true;
}

void ImuBuffer::pop_front() {
  head_ = (head_ + 1) % data_.size();
  --size_;
}

std::size_t ImuBuffer::first_after(common::Timestamp t) const {
  // Binary search on the ring (t strictly increasing).
  std::size_t lo = 0;
  std::size_t hi = size_;
  while (lo < hi) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (at(mid).t <= t) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

std::optional<common::ImuSample> ImuBuffer::interpolate(common::Timestamp t) const {
  if (size_ == 0 || t < front().t || t > back().t) {
    return std::nullopt;
  }
  const std::size_t ib = first_after(t);  // >= 1 because front().t <= t
  const common::ImuSample& a = at(ib - 1);
  if (a.t == t || ib == size_) {
    return a;  // exact sample (ib == size_ only when t == back().t)
  }
  const common::ImuSample& b = at(ib);
  // Weight from integer ns differences; double only for the ratio (pipeline.md section 1).
  const double w = static_cast<double>(t.ns - a.t.ns) / static_cast<double>(b.t.ns - a.t.ns);
  common::ImuSample s;
  s.t = t;
  s.acc_b = a.acc_b + w * (b.acc_b - a.acc_b);
  s.gyro_b = a.gyro_b + w * (b.gyro_b - a.gyro_b);
  return s;
}

std::size_t ImuBuffer::copy_range(common::Timestamp t0,
                                  common::Timestamp t1,
                                  std::array<common::ImuSample, kMaxImuQueue>& out) const {
  std::size_t n = 0;
  for (std::size_t i = first_after(common::Timestamp{t0.ns - 1}); i < size_; ++i) {
    const common::ImuSample& s = at(i);
    if (s.t > t1) {
      break;
    }
    out[n++] = s;
  }
  return n;
}

}  // namespace mss::pipeline::detail
