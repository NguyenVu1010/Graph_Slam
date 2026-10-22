#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "mss/backend/config.hpp"
#include "mss/common/result.hpp"
#include "mss/common/types.hpp"
#include "mss/degeneracy/config.hpp"
#include "mss/gnss/config.hpp"
#include "mss/imu/config.hpp"
#include "mss/loop/config.hpp"
#include "mss/map/config.hpp"
#include "mss/odom/config.hpp"
#include "mss/preprocess/config.hpp"
#include "mss/scan_match/config.hpp"

namespace mss::pipeline {

// YAML: frames.*  Extrinsics are given as translation_m + quaternion_wxyz (Hamilton order).
struct FramesConfig {
  std::string world;                          // frame id of w
  std::string body;                           // frame id of b (= IMU)
  std::string lidar;                          // frame id of l
  std::string gnss;                           // frame id of g (antenna)
  common::Pose T_b_l;                         // LiDAR in body
  common::Pose T_b_o;                         // wheel-odometry frame o in body
  common::Vec3 p_b_g = common::Vec3::Zero();  // m, GNSS lever arm in body (YAML: p_b_g_m)
};

// YAML: realtime.*  Bounded queues and scheduling (ADR-005).
struct RealtimeConfig {
  int scan_queue_depth = 0;             // scans (> 0)
  int imu_queue_depth = 0;              // samples (> 0)
  int aux_queue_depth = 0;              // GNSS / wheel messages (> 0)
  std::size_t max_points_per_scan = 0;  // ScanFrame capacity reserved once (> 0)
  bool pin_cores = false;               // pin estimator threads to cores
  int priority = 0;                     // SCHED_FIFO priority, 0 = default scheduler
};

// YAML: lidar.*
struct LidarConfig {
  double range_sigma_m = 0.0;  // m, range noise sigma
};

// Validated contents of mss_node.ros__parameters (config/default.yaml + optional overlay).
struct Config {
  bool deterministic = false;
  std::uint64_t random_seed = 0;
  FramesConfig frames;
  imu::ImuConfig imu;
  LidarConfig lidar;
  gnss::GnssConfig gnss;
  odom::WheelOdomConfig wheel_odom;
  preprocess::PreprocessConfig preprocess;
  map::MapConfig map;
  scan_match::ScanMatchConfig scan_match;
  degeneracy::DegeneracyConfig degeneracy;
  loop::LoopConfig loop_closure;
  backend::BackendConfig backend;
  RealtimeConfig realtime;
};

// Loads `mss_node.ros__parameters` from `default_yaml`, deep-merges `overlay` on top (overlay
// wins; a key absent from the default is an error; a mapping only replaces a mapping and a
// scalar only a scalar, except that a null default takes anything), then validates every field.
// Keys ending in `_deg` are converted to the matching `_rad` field. Never throws: on failure the
// reason lists every problem, one line per problem, as `<dotted.key>: <reason>`.
common::Result<Config> load_config(
    const std::filesystem::path& default_yaml,
    const std::optional<std::filesystem::path>& overlay = std::nullopt);

// Same as load_config, from YAML text (for tests and callers that already hold the text).
common::Result<Config> parse_config(const std::string& default_text,
                                    const std::optional<std::string>& overlay_text = std::nullopt);

}  // namespace mss::pipeline
