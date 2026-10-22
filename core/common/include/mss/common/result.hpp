#pragma once

#include <optional>
#include <string>
#include <utility>

namespace mss::common {

// Per-message processing never throws: it returns a Status or Result<T>.
// `reason` is a short human-readable string for the log, not for control flow.
class Status {
 public:
  static Status ok() { return Status{}; }
  static Status error(std::string reason) { return Status{std::move(reason)}; }

  bool is_ok() const { return reason_.empty(); }
  explicit operator bool() const { return is_ok(); }
  const std::string& reason() const { return reason_; }

 private:
  Status() = default;
  explicit Status(std::string reason) : reason_(std::move(reason)) {}
  std::string reason_;
};

template <typename T>
class Result {
 public:
  static Result ok(T value) { return Result{std::move(value)}; }
  static Result error(std::string reason) { return Result{Status::error(std::move(reason))}; }

  bool is_ok() const { return value_.has_value(); }
  explicit operator bool() const { return is_ok(); }
  const std::string& reason() const { return status_.reason(); }

  // Precondition: is_ok(). Callers check first; there is no exception path.
  const T& value() const& { return *value_; }
  T& value() & { return *value_; }
  T&& value() && { return std::move(*value_); }

 private:
  explicit Result(T value) : value_(std::move(value)), status_(Status::ok()) {}
  explicit Result(Status status) : status_(std::move(status)) {}
  std::optional<T> value_;
  Status status_;
};

}  // namespace mss::common
