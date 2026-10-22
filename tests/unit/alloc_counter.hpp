#pragma once

// Counts global operator new calls in a test binary.
//
// Exactly ONE translation unit per test binary must define MSS_ALLOC_COUNTER_IMPL before
// including this header; it provides the replacement global operator new/delete.
// Other TUs include the header without the macro and only get the AllocCounter API.
// Usage: set up, call AllocCounter::reset(), run the measured region, assert count() == 0.

#include <cstddef>

struct AllocCounter {
  static std::size_t count();
  static void reset();
};

#ifdef MSS_ALLOC_COUNTER_IMPL

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<std::size_t> g_alloc_count{0};
}

std::size_t AllocCounter::count() {
  return g_alloc_count.load();
}
void AllocCounter::reset() {
  g_alloc_count.store(0);
}

void* operator new(std::size_t n) {
  g_alloc_count.fetch_add(1);
  if (void* p = std::malloc(n == 0 ? 1 : n)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t n) {
  return operator new(n);
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

#endif  // MSS_ALLOC_COUNTER_IMPL
