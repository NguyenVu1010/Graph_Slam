#pragma once

#include <rclcpp/rclcpp.hpp>

#include "mss/common/logger.hpp"

namespace mss_ros2 {

// Adapts the core logger interface to rclcpp logging.
class RclcppLogger final : public mss::common::Logger {
 public:
  explicit RclcppLogger(rclcpp::Logger logger);
  void log(mss::common::LogLevel level, std::string_view msg) override;

 private:
  rclcpp::Logger logger_;
};

}  // namespace mss_ros2
