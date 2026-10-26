#pragma once

// Counts global operator new calls in a test binary, and forbids Eigen heap allocation in
// a scope.
//
// Exactly ONE translation unit per test binary must define MSS_ALLOC_COUNTER_IMPL before
// including this header; it provides the replacement global operator new/delete (plain,
// array, aligned (std::align_val_t) and nothrow variants, all counted the same way).
// Other TUs include the header without the macro and only get the AllocCounter API.
// Usage: set up, call AllocCounter::reset(), run the measured region, assert count() == 0.
//
// Eigen allocates dynamic matrices with std::malloc, which AllocCounter cannot see. For
// that, test targets are compiled with EIGEN_RUNTIME_NO_MALLOC (and without NDEBUG, so
// eigen_assert is live; see mss_add_test in tests/CMakeLists.txt): inside an
// EigenMallocGuard scope any Eigen heap allocation from test code fails an eigen_assert
// and aborts the test. Outside guards Eigen may allocate (Eigen's default). Library code
// (core/) is compiled without the macro, so the guard checks Eigen allocations made in
// header-inlined code instantiated in the test TU only.

#include <cstddef>

#include <Eigen/Core>

#ifndef EIGEN_RUNTIME_NO_MALLOC
#error "alloc_counter.hpp needs EIGEN_RUNTIME_NO_MALLOC (set by mss_add_test)"
#endif

struct AllocCounter {
  static std::size_t count();
  static void reset();
};

// RAII: Eigen heap allocation is forbidden (eigen_assert -> abort) while it lives.
struct EigenMallocGuard {
  EigenMallocGuard() { Eigen::internal::set_is_malloc_allowed(false); }
  ~EigenMallocGuard() { Eigen::internal::set_is_malloc_allowed(true); }
  EigenMallocGuard(const EigenMallocGuard&) = delete;
  EigenMallocGuard& operator=(const EigenMallocGuard&) = delete;
};

#ifdef MSS_ALLOC_COUNTER_IMPL

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<std::size_t> g_alloc_count{0};

void* counted_alloc(std::size_t n) noexcept {
  g_alloc_count.fetch_add(1);
  return std::malloc(n == 0 ? 1 : n);
}

void* counted_aligned_alloc(std::size_t n, std::align_val_t al) noexcept {
  g_alloc_count.fetch_add(1);
  const std::size_t a = static_cast<std::size_t>(al);
  // std::aligned_alloc needs a size that is a non-zero multiple of the alignment.
  const std::size_t size = ((n == 0 ? 1 : n) + a - 1) / a * a;
  return std::aligned_alloc(a, size);
}
}  // namespace

std::size_t AllocCounter::count() {
  return g_alloc_count.load();
}
void AllocCounter::reset() {
  g_alloc_count.store(0);
}

void* operator new(std::size_t n) {
  if (void* p = counted_alloc(n)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t n) {
  return operator new(n);
}
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  return counted_alloc(n);
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
  return counted_alloc(n);
}
void* operator new(std::size_t n, std::align_val_t al) {
  if (void* p = counted_aligned_alloc(n, al)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t al) {
  return operator new(n, al);
}
void* operator new(std::size_t n, std::align_val_t al, const std::nothrow_t&) noexcept {
  return counted_aligned_alloc(n, al);
}
void* operator new[](std::size_t n, std::align_val_t al, const std::nothrow_t&) noexcept {
  return counted_aligned_alloc(n, al);
}

void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}
void operator delete(void* p, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete(void* p, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}

#endif  // MSS_ALLOC_COUNTER_IMPL
