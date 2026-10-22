#include "mss_ros2/rclcpp_logger.hpp"

#include <string>

namespace mss_ros2 {

RclcppLogger::RclcppLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {}

void RclcppLogger::log(mss::common::LogLevel level, std::string_view msg) {
  const std::string s(msg);
  switch (level) {
    case mss::common::LogLevel::kDebug:
      RCLCPP_DEBUG(logger_, "%s", s.c_str());
      break;
    case mss::common::LogLevel::kInfo:
      RCLCPP_INFO(logger_, "%s", s.c_str());
      break;
    case mss::common::LogLevel::kWarn:
      RCLCPP_WARN(logger_, "%s", s.c_str());
      break;
    case mss::common::LogLevel::kError:
      RCLCPP_ERROR(logger_, "%s", s.c_str());
      break;
  }
}

}  // namespace mss_ros2
