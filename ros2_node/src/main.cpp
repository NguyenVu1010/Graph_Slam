// mss_node: subscribes to LiDAR / IMU / GNSS / wheel odometry, converts messages to
// core structs, feeds pipeline::Estimator, publishes pose + TF. Subscriptions land
// with the pipeline; today the node only installs the logger and spins.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mss/common/logger.hpp"

#include "mss_ros2/rclcpp_logger.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mss_node");
  mss_ros2::RclcppLogger logger(node->get_logger());
  mss::common::set_logger(&logger);
  mss::common::log_info("mss_node up; pipeline not wired yet");
  rclcpp::spin(node);
  mss::common::set_logger(nullptr);
  rclcpp::shutdown();
  return 0;
}
