#include "mss/pipeline/config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace mss::pipeline {
namespace {

using common::Pose;
using common::Result;
using common::Vec3;

constexpr double kRadPerDeg = EIGEN_PI / 180.0;
// Quaternions in YAML must be unit length within this (ADR-008); not a tunable.
constexpr double kQuaternionNormTol = 1e-6;

// Accumulated `<dotted.key>: <reason>` problems. Duplicates are dropped.
class Errors {
 public:
  void add(const std::string& key, const std::string& reason) {
    for (const auto& e : entries_) {
      if (e.first == key && e.second == reason) {
        return;
      }
    }
    entries_.emplace_back(key, reason);
  }
  bool empty() const { return entries_.empty(); }
  const std::vector<std::pair<std::string, std::string>>& entries() const { return entries_; }

 private:
  std::vector<std::pair<std::string, std::string>> entries_;
};

std::string kind_name(const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Null:
      return "null";
    case YAML::NodeType::Scalar:
      return "scalar";
    case YAML::NodeType::Sequence:
      return "sequence";
    case YAML::NodeType::Map:
      return "mapping";
    case YAML::NodeType::Undefined:
      break;
  }
  return "undefined";
}

std::string describe(const YAML::Node& node) {
  if (node.IsScalar()) {
    return "'" + node.Scalar() + "'";
  }
  return "a " + kind_name(node);
}

std::string join_key(const std::string& prefix, const std::string& key) {
  return prefix.empty() ? key : prefix + "." + key;
}

// Lookup that never inserts (non-const operator[] would).
YAML::Node child(const YAML::Node& map, const std::string& key) {
  const YAML::Node& const_map = map;
  return const_map[key];
}

// Deep-merges `over` into `base` (mutated in place), with the semantics of
// sim/mss_sim/config_overlay.py: overlay wins; a key absent from base is an error; a mapping
// only replaces a mapping and a scalar only a scalar, except that a null base takes anything.
void merge_overlay(YAML::Node base,
                   const YAML::Node& over,
                   const std::string& prefix,
                   Errors& errors) {
  for (const auto& kv : over) {
    const std::string key = kv.first.Scalar();
    const std::string path = join_key(prefix, key);
    const YAML::Node current = child(base, key);
    const YAML::Node& value = kv.second;
    if (!current.IsDefined()) {
      errors.add(path, "unknown key (not in base config)");
    } else if (current.IsMap() && value.IsMap()) {
      merge_overlay(base[key], value, path, errors);
    } else if (current.IsNull() || current.Type() == value.Type()) {
      base[key] = YAML::Clone(value);
    } else {
      errors.add(path,
                 "overlay type mismatch: base is a " + kind_name(current) + ", overlay is a " +
                     kind_name(value));
    }
  }
}

enum class Range { kAny, kPositive, kNonNegative, kAtLeastOne };

template <typename T>
bool in_range(T value, Range range) {
  switch (range) {
    case Range::kAny:
      return true;
    case Range::kPositive:
      return value > T{0};
    case Range::kNonNegative:
      return value >= T{0};
    case Range::kAtLeastOne:
      return value >= T{1};
  }
  return false;
}

const char* range_text(Range range) {
  switch (range) {
    case Range::kAny:
      break;
    case Range::kPositive:
      return "must be > 0";
    case Range::kNonNegative:
      return "must be >= 0";
    case Range::kAtLeastOne:
      return "must be >= 1";
  }
  return "";
}

template <typename T>
const char* type_name() {
  if constexpr (std::is_same_v<T, bool>) {
    return "a boolean";
  } else if constexpr (std::is_same_v<T, std::string>) {
    return "a string";
  } else if constexpr (std::is_floating_point_v<T>) {
    return "a number";
  } else if constexpr (std::is_unsigned_v<T>) {
    return "a non-negative integer";
  } else {
    return "an integer";
  }
}

// Reads typed fields by dotted key from the merged parameter tree, recording every problem
// and every key it consumed, so that unconsumed keys can be reported as unknown.
class Reader {
 public:
  Reader(YAML::Node root, Errors& errors) : root_(std::move(root)), errors_(errors) {}

  // Reads a scalar; `out` is assigned only if the key exists, has the type and is in range.
  template <typename T>
  bool field(const std::string& key, T& out, Range range = Range::kAny) {
    const std::optional<YAML::Node> node = find(key);
    if (!node) {
      return false;
    }
    T value{};
    if (!decode(*node, value)) {
      errors_.add(key, std::string("expected ") + type_name<T>() + ", got " + describe(*node));
      return false;
    }
    if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
      if (!in_range(value, range)) {
        errors_.add(key, std::string(range_text(range)) + " (got " + node->Scalar() + ")");
        return false;
      }
    }
    out = value;
    return true;
  }

  // YAML key `<name>_deg` (degrees) -> `out` in radians (struct field `<name>_rad`).
  void angle_deg(const std::string& key, double& out, Range range = Range::kAny) {
    double deg = 0.0;
    if (field(key, deg, range)) {
      out = deg * kRadPerDeg;
    }
  }

  void vec3(const std::string& key, Vec3& out) {
    Vec3 v = Vec3::Zero();
    if (sequence(key, v.data(), 3)) {
      out = v;
    }
  }

  // `<key>.translation_m` [x, y, z] and `<key>.quaternion_wxyz` [w, x, y, z] -> Pose.
  void pose(const std::string& key, Pose& out) {
    Pose pose;
    const bool translation_ok = sequence(key + ".translation_m", pose.p.data(), 3);
    const std::string q_key = key + ".quaternion_wxyz";
    std::array<double, 4> wxyz{1.0, 0.0, 0.0, 0.0};
    bool rotation_ok = sequence(q_key, wxyz.data(), wxyz.size());
    if (rotation_ok) {
      // Hamilton order: Eigen::Quaterniond(w, x, y, z).
      const Eigen::Quaterniond q(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
      const double norm = q.norm();
      if (std::abs(norm - 1.0) > kQuaternionNormTol) {
        std::ostringstream os;
        os.precision(9);
        os << "quaternion norm " << norm << " differs from 1 by more than " << kQuaternionNormTol;
        errors_.add(q_key, os.str());
        rotation_ok = false;
      } else {
        pose.R = q.normalized().toRotationMatrix();
      }
    }
    if (translation_ok && rotation_ok) {
      out = pose;
    }
  }

  // Reports every key of the tree that no field consumed, in document order.
  void report_unknown() { walk(root_, ""); }

 private:
  template <typename T>
  static bool decode(const YAML::Node& node, T& value) {
    if (!node.IsScalar() || !YAML::convert<T>::decode(node, value)) {
      return false;
    }
    if constexpr (std::is_floating_point_v<T>) {
      return std::isfinite(value);
    }
    return true;
  }

  bool sequence(const std::string& key, double* out, std::size_t n) {
    const std::optional<YAML::Node> node = find(key);
    if (!node) {
      return false;
    }
    bool ok = node->IsSequence() && node->size() == n;
    for (std::size_t i = 0; ok && i < n; ++i) {
      ok = decode((*node)[i], out[i]);
    }
    if (!ok) {
      errors_.add(key,
                  "expected a sequence of " + std::to_string(n) + " finite numbers, got " +
                      describe(*node));
    }
    return ok;
  }

  // Returns the node at `key`, or nullopt after recording the problem.
  std::optional<YAML::Node> find(const std::string& key) {
    consumed_.insert(key);
    YAML::Node node = root_;
    std::string path;
    std::size_t start = 0;
    while (true) {
      const std::size_t dot = key.find('.', start);
      const std::string part = key.substr(start, dot == std::string::npos ? dot : dot - start);
      if (!node.IsMap()) {
        if (!path.empty()) {
          consumed_.insert(path);
        }
        errors_.add(path, "expected a mapping, got " + describe(node));
        return std::nullopt;
      }
      path = join_key(path, part);
      const YAML::Node next = child(node, part);
      if (!next.IsDefined()) {
        errors_.add(key, "missing key");
        return std::nullopt;
      }
      node.reset(next);  // rebind; `node = next` would overwrite the tree through the reference
      if (dot == std::string::npos) {
        return node;
      }
      start = dot + 1;
    }
  }

  bool has_consumed_below(const std::string& prefix) const {
    const std::string lower = prefix + ".";
    const auto it = consumed_.lower_bound(lower);
    return it != consumed_.end() && it->compare(0, lower.size(), lower) == 0;
  }

  void walk(const YAML::Node& node, const std::string& path) {
    if (!path.empty() && consumed_.count(path) != 0) {
      return;
    }
    if (node.IsMap() && (path.empty() || has_consumed_below(path))) {
      for (const auto& kv : node) {
        walk(kv.second, join_key(path, kv.first.Scalar()));
      }
      return;
    }
    errors_.add(path, "unknown key");
  }

  YAML::Node root_;
  Errors& errors_;
  std::set<std::string> consumed_;
};

// Pre-order position of every key path in the document, for ordering error lines.
void index_keys(const YAML::Node& node,
                const std::string& path,
                std::map<std::string, std::size_t>& order) {
  if (!node.IsMap()) {
    return;
  }
  for (const auto& kv : node) {
    const std::string key = join_key(path, kv.first.Scalar());
    order.emplace(key, order.size());
    index_keys(kv.second, key, order);
  }
}

// Joins the errors into one message, one line per problem, in document order. A key that is
// not in the document (missing, or unknown in the overlay) sorts with its nearest ancestor.
std::string format_errors(const Errors& errors, const YAML::Node& doc) {
  std::map<std::string, std::size_t> order;
  index_keys(doc, "", order);
  auto position = [&order](std::string key) {
    while (!key.empty()) {
      const auto it = order.find(key);
      if (it != order.end()) {
        return it->second;
      }
      const std::size_t dot = key.rfind('.');
      key = dot == std::string::npos ? std::string() : key.substr(0, dot);
    }
    return std::size_t{0};
  };
  std::vector<std::pair<std::string, std::string>> entries = errors.entries();
  std::stable_sort(entries.begin(), entries.end(), [&position](const auto& a, const auto& b) {
    return position(a.first) < position(b.first);
  });
  std::string message;
  for (const auto& e : entries) {
    if (!message.empty()) {
      message += "\n";
    }
    message += e.first + ": " + e.second;
  }
  return message;
}

// Parses `text` and returns its `mss_node.ros__parameters` mapping. Throws YAML::Exception on
// malformed YAML (caught by the caller).
Result<YAML::Node> parameters(const std::string& text, const std::string& source) {
  const YAML::Node doc = YAML::Load(text);
  const YAML::Node node = doc.IsMap() ? child(doc, "mss_node") : YAML::Node();
  const YAML::Node params =
      node.IsDefined() && node.IsMap() ? child(node, "ros__parameters") : YAML::Node();
  if (!params.IsDefined() || !params.IsMap()) {
    return Result<YAML::Node>::error(source +
                                     ": mss_node.ros__parameters is missing or not a mapping");
  }
  return Result<YAML::Node>::ok(params);
}

Config read_config(Reader& r) {
  Config c;
  r.field("deterministic", c.deterministic);
  r.field("random_seed", c.random_seed);

  r.field("frames.world", c.frames.world);
  r.field("frames.body", c.frames.body);
  r.field("frames.lidar", c.frames.lidar);
  r.field("frames.gnss", c.frames.gnss);
  r.pose("frames.T_b_l", c.frames.T_b_l);
  r.pose("frames.T_b_o", c.frames.T_b_o);
  r.vec3("frames.p_b_g_m", c.frames.p_b_g);

  r.field("imu.rate_hz", c.imu.rate_hz, Range::kPositive);
  // Zero IMU noise would make the preintegration covariance singular: strictly positive.
  r.field("imu.acc_noise_density", c.imu.acc_noise_density, Range::kPositive);
  r.field("imu.gyro_noise_density", c.imu.gyro_noise_density, Range::kPositive);
  r.field("imu.acc_random_walk", c.imu.acc_random_walk, Range::kPositive);
  r.field("imu.gyro_random_walk", c.imu.gyro_random_walk, Range::kPositive);
  r.field("imu.gravity_mps2", c.imu.gravity_mps2);
  r.field("imu.init_window_s", c.imu.init_window_s);
  r.field("imu.init_max_acc_std_mps2", c.imu.init_max_acc_std_mps2);
  r.field("imu.init_max_gyro_std_radps", c.imu.init_max_gyro_std_radps);

  r.field("gnss.enabled", c.gnss.enabled);
  r.field("gnss.sigma_h_m", c.gnss.sigma_h_m, Range::kNonNegative);
  r.field("gnss.sigma_v_m", c.gnss.sigma_v_m, Range::kNonNegative);
  r.field("gnss.max_cov_m2", c.gnss.max_cov_m2, Range::kPositive);
  r.field("gnss.align_min_pairs", c.gnss.align_min_pairs, Range::kAtLeastOne);
  r.field("gnss.align_min_spread_m", c.gnss.align_min_spread_m, Range::kPositive);
  r.field("gnss.chi2_gate", c.gnss.chi2_gate, Range::kPositive);
  r.field("gnss.max_consecutive_rejects", c.gnss.max_consecutive_rejects, Range::kAtLeastOne);

  r.field("wheel_odom.enabled", c.wheel_odom.enabled);
  r.field("wheel_odom.wheel_radius_m", c.wheel_odom.wheel_radius_m);
  r.field("wheel_odom.track_m", c.wheel_odom.track_m);
  r.field("wheel_odom.speed_sigma_mps", c.wheel_odom.speed_sigma_mps, Range::kNonNegative);
  r.field(
      "wheel_odom.yaw_rate_sigma_radps", c.wheel_odom.yaw_rate_sigma_radps, Range::kNonNegative);

  r.field("lidar.range_sigma_m", c.lidar.range_sigma_m, Range::kNonNegative);

  r.field("preprocess.min_range_m", c.preprocess.min_range_m);
  r.field("preprocess.max_range_m", c.preprocess.max_range_m);
  r.field("preprocess.downsample_voxel_m", c.preprocess.downsample_voxel_m);
  r.field("preprocess.deskew_enabled", c.preprocess.deskew_enabled);

  r.field("map.voxel_m", c.map.voxel_m, Range::kPositive);
  r.field("map.max_points_per_voxel", c.map.max_points_per_voxel, Range::kPositive);
  r.field("map.min_point_dist_m", c.map.min_point_dist_m, Range::kNonNegative);
  r.field("map.knn_k", c.map.knn_k, Range::kPositive);
  r.field("map.prune_radius_m", c.map.prune_radius_m, Range::kPositive);
  r.field("map.prune_period_s", c.map.prune_period_s, Range::kPositive);
  r.field("map.max_voxels", c.map.max_voxels, Range::kPositive);
  r.field("map.keyframe_voxel_m", c.map.keyframe_voxel_m, Range::kPositive);
  r.field("map.max_keyframes", c.map.max_keyframes, Range::kPositive);
  r.field("map.window_keyframes", c.map.window_keyframes, Range::kPositive);
  r.field("map.rebuild_trans_m", c.map.rebuild_trans_m, Range::kPositive);
  r.angle_deg("map.rebuild_rot_deg", c.map.rebuild_rot_rad, Range::kPositive);

  r.field("scan_match.max_iterations", c.scan_match.max_iterations, Range::kPositive);
  r.field("scan_match.rot_tol_rad", c.scan_match.rot_tol_rad);
  r.field("scan_match.trans_tol_m", c.scan_match.trans_tol_m);
  r.field("scan_match.max_corr_dist_m", c.scan_match.max_corr_dist_m);
  r.field("scan_match.plane_rms_max_m", c.scan_match.plane_rms_max_m);
  r.field("scan_match.robust_huber_delta_m", c.scan_match.robust_huber_delta_m);
  r.field("scan_match.min_inliers", c.scan_match.min_inliers);
  r.field("scan_match.cov_scale", c.scan_match.cov_scale);
  // Floor on the residual sigma: a zero floor would allow a zero (singular) covariance.
  r.field("scan_match.min_residual_sigma_m", c.scan_match.min_residual_sigma_m, Range::kPositive);

  r.field("degeneracy.enabled", c.degeneracy.enabled);
  r.field("degeneracy.eig_threshold", c.degeneracy.eig_threshold, Range::kNonNegative);
  r.field("degeneracy.relax_factor", c.degeneracy.relax_factor, Range::kAtLeastOne);

  r.field("loop_closure.enabled", c.loop_closure.enabled);
  r.field("loop_closure.search_radius_m", c.loop_closure.search_radius_m);
  r.field("loop_closure.min_time_gap_s", c.loop_closure.min_time_gap_s);

  r.field("backend.keyframe_dist_m", c.backend.keyframe_dist_m, Range::kPositive);
  r.angle_deg("backend.keyframe_rot_deg", c.backend.keyframe_rot_rad);
  r.field("backend.relinearize_threshold", c.backend.relinearize_threshold);
  r.field("backend.relinearize_skip", c.backend.relinearize_skip, Range::kPositive);
  r.field("backend.prior_pos_sigma_m", c.backend.prior_pos_sigma_m, Range::kPositive);
  r.angle_deg("backend.prior_rot_sigma_deg", c.backend.prior_rot_sigma_rad, Range::kPositive);
  r.field("backend.prior_vel_sigma_mps", c.backend.prior_vel_sigma_mps, Range::kPositive);
  r.field(
      "backend.prior_bias_acc_sigma_mps2", c.backend.prior_bias_acc_sigma_mps2, Range::kPositive);
  r.field("backend.prior_bias_gyro_sigma_radps",
          c.backend.prior_bias_gyro_sigma_radps,
          Range::kPositive);
  r.field("backend.gnss_frame_variable", c.backend.gnss_frame_variable);
  r.angle_deg(
      "backend.gnss_frame_tilt_sigma_deg", c.backend.gnss_frame_tilt_sigma_rad, Range::kPositive);
  r.angle_deg(
      "backend.gnss_frame_yaw_sigma_deg", c.backend.gnss_frame_yaw_sigma_rad, Range::kPositive);
  r.field("backend.gnss_frame_pos_sigma_m", c.backend.gnss_frame_pos_sigma_m, Range::kPositive);
  // ADR-018: one source of truth (ADR-008), deterministic mode also pins TBB to one thread.
  c.backend.single_thread = c.deterministic;

  r.field("realtime.scan_queue_depth", c.realtime.scan_queue_depth, Range::kPositive);
  r.field("realtime.imu_queue_depth", c.realtime.imu_queue_depth, Range::kPositive);
  r.field("realtime.aux_queue_depth", c.realtime.aux_queue_depth, Range::kPositive);
  r.field("realtime.max_points_per_scan", c.realtime.max_points_per_scan, Range::kPositive);
  r.field("realtime.pin_cores", c.realtime.pin_cores);
  r.field("realtime.priority", c.realtime.priority);
  return c;
}

Result<std::string> read_text(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    return Result<std::string>::error(path.string() + ": cannot open file");
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return Result<std::string>::ok(ss.str());
}

Result<Config> parse_impl(const std::string& default_text,
                          const std::optional<std::string>& overlay_text,
                          const std::string& default_source,
                          const std::string& overlay_source) {
  Result<YAML::Node> base = parameters(default_text, default_source);
  if (!base) {
    return Result<Config>::error(base.reason());
  }
  YAML::Node merged = YAML::Clone(base.value());
  Errors errors;
  if (overlay_text) {
    Result<YAML::Node> over = parameters(*overlay_text, overlay_source);
    if (!over) {
      return Result<Config>::error(over.reason());
    }
    merge_overlay(merged, over.value(), "", errors);
  }
  Reader reader(merged, errors);
  Config config = read_config(reader);
  reader.report_unknown();
  if (!errors.empty()) {
    return Result<Config>::error(format_errors(errors, merged));
  }
  return Result<Config>::ok(std::move(config));
}

// Parses with yaml-cpp exceptions (and any other) converted to an error Result.
Result<Config> parse_guarded(const std::string& default_text,
                             const std::optional<std::string>& overlay_text,
                             const std::string& default_source,
                             const std::string& overlay_source) {
  try {
    return parse_impl(default_text, overlay_text, default_source, overlay_source);
  } catch (const YAML::Exception& e) {
    return Result<Config>::error("YAML error: " + std::string(e.what()) + " (in " + default_source +
                                 (overlay_text ? " or " + overlay_source : std::string()) + ")");
  } catch (const std::exception& e) {
    return Result<Config>::error("config error: " + std::string(e.what()));
  } catch (...) {
    return Result<Config>::error("config error: unknown exception while parsing");
  }
}

}  // namespace

Result<Config> parse_config(const std::string& default_text,
                            const std::optional<std::string>& overlay_text) {
  return parse_guarded(default_text, overlay_text, "<default>", "<overlay>");
}

Result<Config> load_config(const std::filesystem::path& default_yaml,
                           const std::optional<std::filesystem::path>& overlay) {
  try {
    Result<std::string> base = read_text(default_yaml);
    if (!base) {
      return Result<Config>::error(base.reason());
    }
    std::optional<std::string> overlay_text;
    if (overlay) {
      Result<std::string> over = read_text(*overlay);
      if (!over) {
        return Result<Config>::error(over.reason());
      }
      overlay_text = std::move(over).value();
    }
    return parse_guarded(base.value(),
                         overlay_text,
                         default_yaml.string(),
                         overlay ? overlay->string() : std::string());
  } catch (const std::exception& e) {
    return Result<Config>::error(default_yaml.string() + ": " + e.what());
  } catch (...) {
    return Result<Config>::error(default_yaml.string() + ": unknown exception while loading");
  }
}

}  // namespace mss::pipeline
