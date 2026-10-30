// Node-level tests of the input ordering rules of MssNode (backlog R4, R21, R22): messages are
// fed to the subscription callbacks directly (no DDS), the counters tell what was accepted.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "mss_ros2/mss_node.hpp"

namespace {

using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr std::int32_t kSec = 1700000000;  // the sim bags start here
constexpr std::uint32_t kPointStep = 22;   // bridge layout: x y z intensity t ring
constexpr std::uint32_t kNumPoints = 4;

PointCloud2::SharedPtr cloud(std::int32_t sec, bool valid) {
  auto m = std::make_shared<PointCloud2>();
  m->header.stamp.sec = sec;
  m->height = 1;
  m->width = kNumPoints;
  const auto field = [](const char* name, std::uint32_t off, std::uint8_t type) {
    PointField f;
    f.name = name;
    f.offset = off;
    f.datatype = type;
    f.count = 1;
    return f;
  };
  m->fields = {field("x", 0, PointField::FLOAT32),
               field("y", 4, PointField::FLOAT32),
               field("z", 8, PointField::FLOAT32),
               field("intensity", 12, PointField::FLOAT32),
               field("t", 16, PointField::UINT32),
               field("ring", 20, PointField::UINT16)};
  m->point_step = kPointStep;
  m->row_step = kPointStep * kNumPoints;
  m->data.assign(m->row_step, 0);
  if (!valid) {
    m->data.pop_back();  // truncated: the conversion fails
  }
  return m;
}

NavSatFix::SharedPtr fix(std::int32_t sec, double lat, double lon, int status = 0) {
  auto m = std::make_shared<NavSatFix>();
  m->header.stamp.sec = sec;
  m->status.status = static_cast<std::int8_t>(status);
  m->latitude = lat;
  m->longitude = lon;
  m->altitude = 10.0;
  m->position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  return m;
}

class MssNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static std::shared_ptr<mss_ros2::MssNode> make_node() {
    rclcpp::NodeOptions opt;
    opt.append_parameter_override("config_path",
                                  std::string(MSS_REPO_DIR) + "/config/default.yaml");
    return std::make_shared<mss_ros2::MssNode>(opt);
  }
};

// R4/R22: a malformed cloud with a later stamp must not make the good, earlier-stamped scan
// "not increasing".
TEST_F(MssNodeTest, MalformedCloudDoesNotAdvanceTheLidarStamp) {
  const auto node = make_node();
  node->on_lidar(cloud(kSec + 100, false));
  EXPECT_EQ(node->lidar_stats().convert_errors, 1U);
  node->on_lidar(cloud(kSec + 1, true));
  const auto s = node->lidar_stats();
  EXPECT_EQ(s.received, 2U);
  EXPECT_EQ(s.convert_errors, 1U);
  EXPECT_EQ(s.time_skipped, 0U) << "the good scan was dropped as not increasing";
  // The accepted scan did advance the gate: an equal stamp is now skipped.
  node->on_lidar(cloud(kSec + 1, true));
  EXPECT_EQ(node->lidar_stats().time_skipped, 1U);
}

// R1/R22: an impossible first fix with a late stamp is skipped before the origin and must not
// advance the GNSS stamp gate.
TEST_F(MssNodeTest, ImpossibleFirstFixDoesNotAdvanceTheGnssStamp) {
  const auto node = make_node();
  node->on_gnss(fix(kSec + 100, 1000.0, 105.0));
  EXPECT_FALSE(node->has_origin());
  EXPECT_EQ(node->n_gnss_before_origin(), 1U);
  node->on_gnss(fix(kSec + 1, 21.0278, 105.8342));
  EXPECT_TRUE(node->has_origin());
  EXPECT_EQ(node->gnss_stats().time_skipped, 0U) << "the good fix was dropped as not increasing";
}

// R21: after the origin, an impossible fix is skipped and counted, and does not advance the gate.
TEST_F(MssNodeTest, LaterImpossibleFixIsSkippedAndCounted) {
  const auto node = make_node();
  node->on_gnss(fix(kSec + 1, 21.0278, 105.8342));
  ASSERT_TRUE(node->has_origin());
  node->on_gnss(fix(kSec + 100, 21.0, 500.0));
  node->on_gnss(fix(kSec + 101, std::nan(""), 105.0));
  EXPECT_EQ(node->gnss_stats().convert_errors, 2U);
  node->on_gnss(fix(kSec + 2, 21.0279, 105.8343));
  const auto s = node->gnss_stats();
  EXPECT_EQ(s.convert_errors, 2U);
  EXPECT_EQ(s.time_skipped, 0U);
}

}  // namespace
