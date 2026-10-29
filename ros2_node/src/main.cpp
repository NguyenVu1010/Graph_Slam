// mss_node: subscribes to LiDAR / IMU / GNSS / wheel odometry, converts messages to core
// structs, feeds pipeline::Estimator, publishes odometry, path, TF and the map (ADR-024).
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mss_ros2/mss_node.hpp"

namespace {
// Process exit code for a configuration or initialisation error (same meaning as mss_bench's 2).
constexpr int kExitConfigError = 2;
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<mss_ros2::MssNode> node;
  try {
    node = std::make_shared<mss_ros2::MssNode>();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("mss_node"), "initialisation failed: %s", e.what());
    rclcpp::shutdown();
    return kExitConfigError;
  }
  // ADR-024 section 4: one thread for every callback and timer (the Estimator contract).
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  node->finish();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
