#pragma once

#include <array>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace mss::common {

// Minimal non-owning view over contiguous memory (C++17 has no std::span).
// The viewed storage must outlive the Span.
template <class T>
class Span {
 public:
  Span() = default;
  Span(T* data, std::size_t size) : data_(data), size_(size) {}

  // Span<const U> from std::vector<U> / std::array<U, N>.
  template <
      class U,
      class A,
      class V = T,
      class = std::enable_if_t<std::is_const_v<V> && std::is_same_v<std::remove_const_t<V>, U>>>
  Span(const std::vector<U, A>& v) : data_(v.data()), size_(v.size()) {}

  template <
      class U,
      std::size_t N,
      class V = T,
      class = std::enable_if_t<std::is_const_v<V> && std::is_same_v<std::remove_const_t<V>, U>>>
  Span(const std::array<U, N>& a) : data_(a.data()), size_(N) {}

  // Span<const U> from Span<U> (a mutable view converts to a read-only one).
  template <
      class U,
      class V = T,
      class = std::enable_if_t<std::is_const_v<V> && std::is_same_v<std::remove_const_t<V>, U>>>
  Span(const Span<U>& s) : data_(s.data()), size_(s.size()) {}

  // No view of a temporary container: it would dangle when the full-expression ends.
  template <class U, class A>
  Span(std::vector<U, A>&&) = delete;
  template <class U, std::size_t N>
  Span(std::array<U, N>&&) = delete;

  T* begin() const { return data_; }
  T* end() const { return data_ + size_; }
  T* data() const { return data_; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  T& operator[](std::size_t i) const { return data_[i]; }

 private:
  T* data_ = nullptr;
  std::size_t size_ = 0;
};

}  // namespace mss::common
