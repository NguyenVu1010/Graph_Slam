// mss_node: subscribes to LiDAR / IMU / GNSS / wheel odometry, converts messages to
// core structs, feeds pipeline::Estimator, publishes pose + TF. Subscriptions land
// with the pipeline; today the node reads `config_path`, installs the logger and spins.
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mss/common/logger.hpp"

#include "mss_ros2/rclcpp_logger.hpp"

namespace {
// Process exit code for a missing configuration (same meaning as mss_bench's 2).
constexpr int kExitConfigError = 2;
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mss_node");
  mss_ros2::RclcppLogger logger(node->get_logger());
  mss::common::set_logger(&logger);
  // config/default.yaml (ADR-008) is read by pipeline::load_config, not as ROS parameters;
  // the node only receives its path.
  std::string default_config;
  try {
    default_config =
        ament_index_cpp::get_package_share_directory("mss_ros2") + "/config/default.yaml";
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    // Not installed through the ament index: config_path must then be set explicitly.
  }
  const std::string config_path =
      node->declare_parameter<std::string>("config_path", default_config);
  if (config_path.empty()) {
    mss::common::log_error("config_path not set and mss_ros2 share directory not found");
    mss::common::set_logger(nullptr);
    rclcpp::shutdown();
    return kExitConfigError;
  }
  mss::common::log_info("mss_node up; config_path=" + config_path + "; pipeline not wired yet");
  rclcpp::spin(node);
  mss::common::set_logger(nullptr);
  rclcpp::shutdown();
  return 0;
}
