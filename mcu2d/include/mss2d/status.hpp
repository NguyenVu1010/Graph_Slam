#pragma once

#include <utility>

namespace mss2d {

// Success or a reason. No exceptions, no std::string: the reason is a pointer to a string
// literal (static storage), safe to keep and to log by pointer. Shape mirrors
// mss::common::Status.
class Status {
 public:
  static constexpr Status ok() { return Status(nullptr); }
  /// `reason` must be a string literal.
  static constexpr Status error(const char* reason) { return Status(reason); }

  constexpr bool is_ok() const { return reason_ == nullptr; }
  constexpr explicit operator bool() const { return is_ok(); }
  /// Empty string when ok.
  constexpr const char* reason() const { return reason_ != nullptr ? reason_ : ""; }

 private:
  constexpr explicit Status(const char* r) : reason_(r) {}
  const char* reason_;
};

// A value or a Status error. T must be default constructible (no heap, no optional).
template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)) {}  // NOLINT: implicit on purpose
  Result(Status error) : status_(error) {}       // NOLINT: implicit on purpose

  bool is_ok() const { return status_.is_ok(); }
  const Status& status() const { return status_; }
  /// Valid only when is_ok().
  const T& value() const { return value_; }
  T& value() { return value_; }

 private:
  Status status_ = Status::ok();
  T value_{};
};

}  // namespace mss2d
