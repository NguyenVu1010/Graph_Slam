// Node-level tests of MssNode: the input ordering rules (backlog R4, R21, R22; messages fed to the
// subscription callbacks directly, no DDS) and viz.enabled (ADR-031; services called by clients).
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "mss_ros2/mss_node.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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

// --- viz.enabled (ADR-031) --------------------------------------------------------------------

bool has_service(const rclcpp::Node& n, const std::string& name) {
  return n.get_service_names_and_types().count(name) > 0;
}

// Flag off (the default): no graph topics, no services, no viz.* parameter beyond the flag.
TEST_F(MssNodeTest, VizOffAddsNothing) {
  const auto node = make_node();
  EXPECT_EQ(node->count_publishers("/mss/graph/keyframes"), 0U);
  EXPECT_EQ(node->count_publishers("/mss/graph/loops"), 0U);
  EXPECT_EQ(node->count_publishers("/mss/graph/path"), 0U);
  EXPECT_EQ(node->count_publishers("/mss/graph/counts"), 0U);
  EXPECT_FALSE(has_service(*node, "/mss/save"));
  EXPECT_FALSE(has_service(*node, "/mss/pause"));
  EXPECT_TRUE(node->has_parameter("viz.enabled"));
  EXPECT_FALSE(node->has_parameter("viz.rate_hz"));
  EXPECT_FALSE(node->has_parameter("viz.gz_world"));
  EXPECT_EQ(node->count_publishers("/mss/paused"), 0U);
  EXPECT_FALSE(node->has_parameter("viz.paused_after_s"));
}

// gz stand-in for the world control: records the request of each call; a call stays in flight
// while `hold` is set, then replies true (or an error when `ok` is false).
struct FakeWorld {
  std::vector<std::string> calls;  // the --req of each gz call
  bool ok = true;
  bool hold = false;
};

class FakeCall final : public mss_ros2::PendingCommand {
 public:
  explicit FakeCall(const FakeWorld* w) : w_(w) {}
  std::optional<mss_ros2::CommandResult> poll() override {
    if (w_->hold) {
      return std::nullopt;
    }
    mss_ros2::CommandResult r;
    r.started = true;
    r.exit_code = w_->ok ? 0 : 1;
    r.output = w_->ok ? "data: true\n" : "error\n";
    return r;
  }

 private:
  const FakeWorld* w_;
};

// The node's services are called through real clients on an executor that also spins the node,
// so the deferred responses (review M-2) and the clock-based world state (review M-1) are
// exercised as in the running node.
class MssNodeVizTest : public MssNodeTest {
 protected:
  static constexpr double kPausedAfterS = 0.2;  // viz.paused_after_s of the test node, s

  void SetUp() override {
    // Unique per process (review N-1).
    base_ = std::filesystem::temp_directory_path() /
            ("mss_node_viz_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(base_);
    client_ = std::make_shared<rclcpp::Node>("mss_viz_test_client");
    clock_pub_ = client_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    exec_.add_node(client_);
  }
  void TearDown() override {
    if (node_) {
      exec_.remove_node(node_);
    }
    exec_.remove_node(client_);
    std::filesystem::remove_all(base_);
  }

  void make_viz_node(const std::string& world, bool sim_time) {
    rclcpp::NodeOptions opt;
    opt.append_parameter_override("config_path",
                                  std::string(MSS_REPO_DIR) + "/config/default.yaml");
    opt.append_parameter_override("use_sim_time", sim_time);
    opt.append_parameter_override("viz.enabled", true);
    opt.append_parameter_override("viz.save_dir", base_.string());
    opt.append_parameter_override("viz.run_id", "run-a");
    opt.append_parameter_override("viz.gz_world", world);
    opt.append_parameter_override("viz.paused_after_s", kPausedAfterS);
    node_ = std::make_shared<mss_ros2::MssNode>(opt);
    node_->set_world_command_starter([this](const std::vector<std::string>& argv, double) {
      gz_.calls.push_back(argv.back());
      return std::make_unique<FakeCall>(&gz_);
    });
    exec_.add_node(node_);
  }

  // Spins for `seconds` of wall time; with `clock`, publishes /clock advancing in real time.
  void spin_for(double seconds, bool clock) {
    const auto end = std::chrono::steady_clock::now() + to_dur(seconds);
    while (std::chrono::steady_clock::now() < end) {
      step(clock);
    }
  }
  void step(bool clock) {
    constexpr double kStepS = 0.005;
    if (clock) {
      sim_ns_ += static_cast<std::int64_t>(kStepS * 1e9);
      rosgraph_msgs::msg::Clock c;
      c.clock.sec = static_cast<std::int32_t>(sim_ns_ / 1000000000);
      c.clock.nanosec = static_cast<std::uint32_t>(sim_ns_ % 1000000000);
      clock_pub_->publish(c);
    }
    exec_.spin_some(to_dur(kStepS));
    std::this_thread::sleep_for(to_dur(kStepS));
  }
  static std::chrono::nanoseconds to_dur(double s) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(s));
  }

  // Sends a request and spins (publishing /clock when `clock`) until the answer, at most 5 s.
  template <typename Srv>
  typename Srv::Response::SharedPtr call(const std::string& name,
                                         typename Srv::Request::SharedPtr req,
                                         bool clock = false) {
    auto future = send<Srv>(name, req);
    return wait_for<Srv>(future, clock);
  }
  template <typename Srv>
  typename rclcpp::Client<Srv>::SharedFuture send(const std::string& name,
                                                  typename Srv::Request::SharedPtr req) {
    auto cli = client_->create_client<Srv>(name);
    clients_.push_back(cli);
    EXPECT_TRUE(cli->wait_for_service(std::chrono::seconds(5)));
    return cli->async_send_request(req).future.share();
  }
  template <typename Srv>
  typename Srv::Response::SharedPtr wait_for(typename rclcpp::Client<Srv>::SharedFuture f,
                                             bool clock,
                                             double max_s = 5.0) {
    const auto end = std::chrono::steady_clock::now() + to_dur(max_s);
    while (std::chrono::steady_clock::now() < end) {
      if (f.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        return f.get();
      }
      step(clock);
    }
    return nullptr;
  }

  std_srvs::srv::Trigger::Response::SharedPtr save(bool clock = false) {
    return call<std_srvs::srv::Trigger>(
        "/mss/save", std::make_shared<std_srvs::srv::Trigger::Request>(), clock);
  }
  static std_srvs::srv::SetBool::Request::SharedPtr pause_req(bool paused) {
    auto r = std::make_shared<std_srvs::srv::SetBool::Request>();
    r->data = paused;
    return r;
  }

  std::filesystem::path base_;
  FakeWorld gz_;
  std::int64_t sim_ns_ = 1000000000;  // 1 s
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::vector<rclcpp::ClientBase::SharedPtr> clients_;
  std::shared_ptr<mss_ros2::MssNode> node_;
};

TEST_F(MssNodeVizTest, VizOnCreatesTopicsAndServices) {
  make_viz_node("warehouse", true);
  EXPECT_EQ(node_->count_publishers("/mss/graph/keyframes"), 1U);
  EXPECT_EQ(node_->count_publishers("/mss/graph/loops"), 1U);
  EXPECT_EQ(node_->count_publishers("/mss/graph/path"), 1U);
  EXPECT_EQ(node_->count_publishers("/mss/graph/counts"), 1U);
  EXPECT_EQ(node_->count_publishers("/mss/paused"), 1U);
  EXPECT_TRUE(has_service(*node_, "/mss/save"));
  EXPECT_TRUE(has_service(*node_, "/mss/pause"));
  EXPECT_EQ(node_->save_dir(), (base_ / "run-a").string());
}

// Review M-2: the gz call runs while the executor keeps serving; the pause is answered once gz
// is done, and a second request meanwhile gets an immediate "busy".
TEST_F(MssNodeVizTest, PauseIsAnsweredLaterAndTheExecutorStaysFree) {
  make_viz_node("warehouse", true);
  gz_.hold = true;
  auto pending = send<std_srvs::srv::SetBool>("/mss/pause", pause_req(true));
  spin_for(0.3, false);
  EXPECT_NE(pending.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  const auto busy = save();  // served while the pause is in flight
  ASSERT_NE(busy, nullptr) << "the executor was blocked";
  EXPECT_FALSE(busy->success);
  EXPECT_NE(busy->message.find("busy"), std::string::npos) << busy->message;
  gz_.hold = false;
  const auto done = wait_for<std_srvs::srv::SetBool>(pending, false);
  ASSERT_NE(done, nullptr);
  EXPECT_TRUE(done->success) << done->message;
  EXPECT_EQ(done->message, "world paused");
  EXPECT_EQ(gz_.calls, (std::vector<std::string>{"pause: true"}));

  gz_.ok = false;
  const auto failed = call<std_srvs::srv::SetBool>("/mss/pause", pause_req(false));
  ASSERT_NE(failed, nullptr);
  EXPECT_FALSE(failed->success);
  EXPECT_NE(failed->message.find("failed"), std::string::npos) << failed->message;
}

// Review M-1, running world (sim clock advancing in real time): save pauses it, writes the three
// files into save-001, resumes it; the next save goes to save-002 (review N-8).
TEST_F(MssNodeVizTest, SaveOfARunningWorldPausesWritesAndResumes) {
  make_viz_node("warehouse", true);
  spin_for(3 * kPausedAfterS, true);
  const auto res = save(true);
  ASSERT_NE(res, nullptr);
  ASSERT_TRUE(res->success) << res->message;
  const std::filesystem::path dir = base_ / "run-a" / "save-001";
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "kf_opt.tum"));
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "loops.csv"));
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "map.pcd"));
  EXPECT_NE(res->message.find(dir.string()), std::string::npos) << res->message;
  EXPECT_EQ(gz_.calls, (std::vector<std::string>{"pause: true", "pause: false"}));
  const auto again = save(true);
  ASSERT_NE(again, nullptr);
  EXPECT_TRUE(again->success) << again->message;
  EXPECT_TRUE(std::filesystem::is_directory(base_ / "run-a" / "save-002"));
}

// Review M-1, world paused by someone else (GUI, gz CLI: the sim clock stopped): save writes and
// makes no gz call at all, so it never resumes that world; /mss/paused reports true.
TEST_F(MssNodeVizTest, SaveOfAnExternallyPausedWorldNeverResumesIt) {
  make_viz_node("warehouse", true);
  bool paused_seen = false;
  const auto sub = client_->create_subscription<std_msgs::msg::Bool>(
      "/mss/paused",
      rclcpp::QoS(1).reliable().transient_local(),
      [&paused_seen](const std_msgs::msg::Bool& m) { paused_seen = m.data; });
  spin_for(3 * kPausedAfterS, true);  // running
  EXPECT_FALSE(paused_seen);
  spin_for(3 * kPausedAfterS, false);  // the clock stops: paused from outside
  EXPECT_TRUE(paused_seen);
  const auto res = save(false);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_NE(res->message.find("world left as it is"), std::string::npos) << res->message;
  EXPECT_TRUE(gz_.calls.empty()) << "a save must not touch a world it did not pause";
}

// Review M-3: pause through /mss/pause with the world running, then save at once (well within
// viz.paused_after_s): the node's own successful pause counts at once, so /mss/paused is true
// before the save is answered and the save sends no gz call (exactly one call: pause: true).
TEST_F(MssNodeVizTest, PauseThenSaveAtOnceKeepsTheWorldPaused) {
  make_viz_node("warehouse", true);
  bool paused_seen = false;
  const auto sub = client_->create_subscription<std_msgs::msg::Bool>(
      "/mss/paused",
      rclcpp::QoS(1).reliable().transient_local(),
      [&paused_seen](const std_msgs::msg::Bool& m) { paused_seen = m.data; });
  spin_for(3 * kPausedAfterS, true);  // running
  ASSERT_FALSE(paused_seen);
  const auto p = call<std_srvs::srv::SetBool>("/mss/pause", pause_req(true), true);
  ASSERT_NE(p, nullptr);
  ASSERT_TRUE(p->success) << p->message;
  // The world stops (the clock is no longer published); spin briefly, far less than
  // paused_after_s, so only the node's own pause can tell it is paused.
  const auto t0 = std::chrono::steady_clock::now();
  spin_for(0.02, false);
  EXPECT_TRUE(paused_seen) << "/mss/paused did not follow the node's own pause at once";
  const auto res = save(false);
  const double dt_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_LT(dt_s, kPausedAfterS) << "the test did not stay inside the clock-only window";
  EXPECT_EQ(gz_.calls, (std::vector<std::string>{"pause: true"}));
}

// Review M-3, the reverse: resume through /mss/pause, then save at once: the world counts as
// running, so the save pauses it and resumes it.
TEST_F(MssNodeVizTest, ResumeThenSaveAtOncePausesAndResumes) {
  make_viz_node("warehouse", true);
  spin_for(3 * kPausedAfterS, false);  // the clock stands: paused
  const auto p = call<std_srvs::srv::SetBool>("/mss/pause", pause_req(false), false);
  ASSERT_NE(p, nullptr);
  ASSERT_TRUE(p->success) << p->message;
  const auto res = save(true);  // the world runs again: the clock advances
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_EQ(gz_.calls, (std::vector<std::string>{"pause: false", "pause: true", "pause: false"}));
}

// Review N2-1: with a Gazebo world the state comes from /clock, so use_sim_time must be on.
TEST_F(MssNodeVizTest, WorldWithoutSimTimeIsRejected) {
  EXPECT_THROW(make_viz_node("warehouse", false), std::invalid_argument);
}

// No Gazebo world (e.g. a bag): pause fails with a reason, save still writes.
TEST_F(MssNodeVizTest, WithoutAWorldSaveStillWrites) {
  make_viz_node("", false);
  const auto res = save();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_NE(res->message.find("no Gazebo world"), std::string::npos) << res->message;
  const auto p = call<std_srvs::srv::SetBool>("/mss/pause", pause_req(true));
  ASSERT_NE(p, nullptr);
  EXPECT_FALSE(p->success);
  EXPECT_TRUE(gz_.calls.empty());
}

}  // namespace
