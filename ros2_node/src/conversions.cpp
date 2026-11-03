#include "mss_ros2/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace mss_ros2 {

namespace {

using sensor_msgs::msg::PointField;

constexpr double kMaxAbsLatitudeDeg = 90.0;    // WGS-84 domain of gnss::lla_to_enu
constexpr double kMaxAbsLongitudeDeg = 180.0;  // WGS-84 domain of gnss::lla_to_enu

// Offset of a scalar (count 1) field named `name` with datatype `type`. kMissing: no such
// field; kWrongType: present with another datatype or count.
enum class FieldLookup { kOk, kMissing, kWrongType };

FieldLookup find_field(const sensor_msgs::msg::PointCloud2& msg,
                       const char* name,
                       std::uint8_t type,
                       std::uint32_t& offset) {
  for (const PointField& f : msg.fields) {
    if (f.name == name) {
      if (f.datatype != type || f.count != 1) {
        return FieldLookup::kWrongType;
      }
      offset = f.offset;
      return FieldLookup::kOk;
    }
  }
  return FieldLookup::kMissing;
}

bool host_is_big_endian() {
  const std::uint16_t probe = 1;
  std::uint8_t first = 0;
  std::memcpy(&first, &probe, 1);
  return first == 0;
}

template <typename T>
T read_at(const std::uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

}  // namespace

mss::common::Timestamp to_timestamp(const builtin_interfaces::msg::Time& stamp) {
  return mss::common::timestamp_from_sec_nsec(static_cast<std::int64_t>(stamp.sec),
                                              static_cast<std::int64_t>(stamp.nanosec));
}

builtin_interfaces::msg::Time to_msg_time(mss::common::Timestamp t) {
  std::int64_t sec = t.ns / mss::common::kNanosecondsPerSecond;
  std::int64_t nsec = t.ns % mss::common::kNanosecondsPerSecond;
  if (nsec < 0) {  // floor division for times before the epoch
    sec -= 1;
    nsec += mss::common::kNanosecondsPerSecond;
  }
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(sec);
  out.nanosec = static_cast<std::uint32_t>(nsec);
  return out;
}

mss::common::Status scan_from_cloud(const sensor_msgs::msg::PointCloud2& msg,
                                    mss::common::ScanFrame& out) {
  out.clear();
  if (msg.is_bigendian != host_is_big_endian()) {
    return mss::common::Status::error("PointCloud2 byte order differs from the host");
  }
  std::uint32_t off_x = 0;
  std::uint32_t off_y = 0;
  std::uint32_t off_z = 0;
  if (find_field(msg, "x", PointField::FLOAT32, off_x) != FieldLookup::kOk ||
      find_field(msg, "y", PointField::FLOAT32, off_y) != FieldLookup::kOk ||
      find_field(msg, "z", PointField::FLOAT32, off_z) != FieldLookup::kOk) {
    return mss::common::Status::error("PointCloud2 needs FLOAT32 fields x, y, z");
  }
  std::uint32_t off_t = 0;
  std::uint32_t off_ring = 0;
  const FieldLookup t_ok = find_field(msg, "t", PointField::UINT32, off_t);
  const FieldLookup ring_ok = find_field(msg, "ring", PointField::UINT16, off_ring);
  if (t_ok == FieldLookup::kMissing || ring_ok == FieldLookup::kMissing) {
    return mss::common::Status::error(
        "PointCloud2 has no per-point t / ring field; run the noise bridge first");
  }
  if (t_ok != FieldLookup::kOk || ring_ok != FieldLookup::kOk) {
    return mss::common::Status::error("PointCloud2 field t must be UINT32 and ring UINT16");
  }
  std::uint32_t off_i = 0;
  const FieldLookup i_ok = find_field(msg, "intensity", PointField::FLOAT32, off_i);
  if (i_ok == FieldLookup::kWrongType) {
    return mss::common::Status::error("PointCloud2 field intensity must be FLOAT32");
  }
  const bool has_intensity = i_ok == FieldLookup::kOk;

  // Bounds in 64-bit: offsets are untrusted uint32 values, and offset + width in 32-bit
  // arithmetic wraps for offsets near UINT32_MAX (an out-of-bounds read).
  const std::uint64_t step = msg.point_step;
  const auto fits = [step](std::uint32_t offset, std::uint64_t bytes) {
    return static_cast<std::uint64_t>(offset) + bytes <= step;
  };
  if (!fits(off_x, sizeof(float)) || !fits(off_y, sizeof(float)) || !fits(off_z, sizeof(float)) ||
      !fits(off_t, sizeof(std::uint32_t)) || !fits(off_ring, sizeof(std::uint16_t)) ||
      (has_intensity && !fits(off_i, sizeof(float)))) {
    return mss::common::Status::error("PointCloud2 field lies past point_step");
  }
  // Capacity first: then n * step <= capacity * UINT32_MAX cannot overflow 64 bits.
  const std::uint64_t n = static_cast<std::uint64_t>(msg.width) * msg.height;
  if (n > out.capacity()) {
    return mss::common::Status::error("PointCloud2 has more points than max_points_per_scan");
  }
  if (msg.data.size() < n * step) {
    return mss::common::Status::error("PointCloud2 data shorter than width * height * point_step");
  }
  float* xyz = out.xyz_mut();
  float* inten = out.intensity_mut();
  std::uint32_t* toff = out.t_offset_ns_mut();
  std::uint16_t* ring = out.ring_mut();
  const std::uint8_t* base = msg.data.data();
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t* p = base + i * step;
    xyz[3 * i + 0] = read_at<float>(p + off_x);
    xyz[3 * i + 1] = read_at<float>(p + off_y);
    xyz[3 * i + 2] = read_at<float>(p + off_z);
    inten[i] = has_intensity ? read_at<float>(p + off_i) : 0.0F;
    toff[i] = read_at<std::uint32_t>(p + off_t);
    ring[i] = read_at<std::uint16_t>(p + off_ring);
  }
  out.t_start = to_timestamp(msg.header.stamp);
  out.set_size(n);
  return mss::common::Status::ok();
}

mss::common::ImuSample imu_from_msg(const sensor_msgs::msg::Imu& msg) {
  mss::common::ImuSample s;
  s.t = to_timestamp(msg.header.stamp);
  s.acc_b = mss::common::Vec3(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  s.gyro_b =
      mss::common::Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  return s;
}

mss::common::WheelSample wheel_from_odometry(const nav_msgs::msg::Odometry& msg) {
  mss::common::WheelSample s;
  s.t = to_timestamp(msg.header.stamp);
  s.v_mps = msg.twist.twist.linear.x;
  s.omega_radps = msg.twist.twist.angular.z;
  return s;
}

bool fix_in_range(const sensor_msgs::msg::NavSatFix& msg) {
  return std::isfinite(msg.latitude) && std::isfinite(msg.longitude) &&
         std::isfinite(msg.altitude) && std::abs(msg.latitude) <= kMaxAbsLatitudeDeg &&
         std::abs(msg.longitude) <= kMaxAbsLongitudeDeg;
}

bool origin_from_fix(const sensor_msgs::msg::NavSatFix& msg, mss::gnss::Lla& out) {
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || !fix_in_range(msg)) {
    return false;
  }
  out.lat_deg = msg.latitude;
  out.lon_deg = msg.longitude;
  out.alt_m = msg.altitude;
  return true;
}

mss::common::GnssMeasurement gnss_from_navsat(const sensor_msgs::msg::NavSatFix& msg,
                                              const mss::gnss::Lla& origin,
                                              const GnssFallbackSigma& fallback) {
  mss::common::GnssMeasurement m;
  m.t = to_timestamp(msg.header.stamp);
  mss::gnss::Lla lla;
  lla.lat_deg = msg.latitude;
  lla.lon_deg = msg.longitude;
  lla.alt_m = msg.altitude;
  m.p_e_g = mss::gnss::lla_to_enu(lla, origin);
  if (msg.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    const double var_h = fallback.sigma_h_m * fallback.sigma_h_m;
    const double var_v = fallback.sigma_v_m * fallback.sigma_v_m;
    m.cov = mss::common::Vec3(var_h, var_h, var_v).asDiagonal();
  } else {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        m.cov(r, c) = msg.position_covariance[static_cast<std::size_t>(3 * r + c)];
      }
    }
  }
  m.status = msg.status.status;
  return m;
}

geometry_msgs::msg::Pose pose_to_msg(const mss::common::Pose& T_w_b) {
  geometry_msgs::msg::Pose out;
  out.position.x = T_w_b.p.x();
  out.position.y = T_w_b.p.y();
  out.position.z = T_w_b.p.z();
  // Hamilton quaternion (w, x, y, z) of R_w_b; ROS stores the fields as x, y, z, w.
  Eigen::Quaterniond q(T_w_b.R);
  q.normalize();
  out.orientation.w = q.w();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  return out;
}

mss::common::Mat6 cov_to_ros(const mss::common::Pose& T_w_b, const mss::common::Mat6& cov) {
  mss::common::Mat6 J = mss::common::Mat6::Zero();
  J.block<3, 3>(0, 0) = T_w_b.R;
  J.block<3, 3>(3, 3) = T_w_b.R;
  const mss::common::Mat6 out = J * cov * J.transpose();
  return out;
}

nav_msgs::msg::Odometry odometry_from_estimate(const mss::common::Estimate& e,
                                               const OutputFrames& frames) {
  nav_msgs::msg::Odometry out;
  out.header.stamp = to_msg_time(e.state.t);
  out.header.frame_id = frames.world;
  out.child_frame_id = frames.body;
  out.pose.pose = pose_to_msg(e.state.T_w_b);
  const mss::common::Mat6 cov = cov_to_ros(e.state.T_w_b, e.cov);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      out.pose.covariance[static_cast<std::size_t>(6 * r + c)] = cov(r, c);
    }
  }
  const mss::common::Vec3 v_b = e.state.T_w_b.R.transpose() * e.state.v_w_b;
  out.twist.twist.linear.x = v_b.x();
  out.twist.twist.linear.y = v_b.y();
  out.twist.twist.linear.z = v_b.z();
  return out;
}

geometry_msgs::msg::PoseStamped pose_stamped(const mss::common::Pose& T_w_b,
                                             mss::common::Timestamp t,
                                             const std::string& world_frame) {
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = to_msg_time(t);
  out.header.frame_id = world_frame;
  out.pose = pose_to_msg(T_w_b);
  return out;
}

geometry_msgs::msg::TransformStamped transform_from_pose(const mss::common::Pose& T_w_b,
                                                         mss::common::Timestamp t,
                                                         const OutputFrames& frames) {
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = to_msg_time(t);
  out.header.frame_id = frames.world;
  out.child_frame_id = frames.body;
  const geometry_msgs::msg::Pose pose = pose_to_msg(T_w_b);
  out.transform.translation.x = pose.position.x;
  out.transform.translation.y = pose.position.y;
  out.transform.translation.z = pose.position.z;
  out.transform.rotation = pose.orientation;
  return out;
}

void cloud_from_points(const std::vector<Eigen::Vector3f>& points_w,
                       mss::common::Timestamp t,
                       const std::string& world_frame,
                       sensor_msgs::msg::PointCloud2& out) {
  constexpr std::uint32_t kFloatBytes = 4;
  constexpr std::uint32_t kPointStep = 3 * kFloatBytes;
  out.header.stamp = to_msg_time(t);
  out.header.frame_id = world_frame;
  if (out.fields.size() != 3) {
    out.fields.resize(3);
    const char* names[3] = {"x", "y", "z"};
    for (std::uint32_t k = 0; k < 3; ++k) {
      out.fields[k].name = names[k];
      out.fields[k].offset = k * kFloatBytes;
      out.fields[k].datatype = PointField::FLOAT32;
      out.fields[k].count = 1;
    }
  }
  const std::size_t n = points_w.size();
  out.height = 1;
  out.width = static_cast<std::uint32_t>(n);
  out.is_bigendian = host_is_big_endian();
  out.point_step = kPointStep;
  out.row_step = static_cast<std::uint32_t>(n * kPointStep);
  out.is_dense = true;
  out.data.resize(n * kPointStep);
  std::uint8_t* dst = out.data.data();
  for (std::size_t i = 0; i < n; ++i) {
    std::memcpy(dst + i * kPointStep, points_w[i].data(), kPointStep);
  }
}

namespace {

using mss::pipeline::GraphKeyframe;
using mss::pipeline::GraphSnapshot;
using mss::pipeline::LoopState;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA rgba(float r, float g, float b) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = 1.0F;
  return c;
}

Marker marker_header(mss::common::Timestamp t, const std::string& world_frame) {
  Marker m;
  m.header.stamp = to_msg_time(t);
  m.header.frame_id = world_frame;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;  // identity (Hamilton w, x, y, z = 1, 0, 0, 0)
  return m;
}

void start_array(mss::common::Timestamp t, const std::string& world_frame, MarkerArray& out) {
  out.markers.clear();
  Marker del = marker_header(t, world_frame);
  del.action = Marker::DELETEALL;
  out.markers.push_back(del);
}

// Keyframe `id` of the snapshot (keyframes are in id order, possibly with gaps), or nullptr.
const GraphKeyframe* find_keyframe(const GraphSnapshot& snap, mss::common::KeyId id) {
  const auto it =
      std::lower_bound(snap.keyframes.begin(),
                       snap.keyframes.end(),
                       id,
                       [](const GraphKeyframe& k, mss::common::KeyId v) { return k.id < v; });
  return it != snap.keyframes.end() && it->id == id ? &*it : nullptr;
}

geometry_msgs::msg::Point point_of(const mss::common::Vec3& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

const char* state_name(LoopState s) {
  switch (s) {
    case LoopState::kActive:
      return "active";
    case LoopState::kReleased:
      return "released";
    case LoopState::kRemoved:
      return "removed";
  }
  return "unknown";
}

}  // namespace

std_msgs::msg::ColorRGBA loop_color(LoopState state) {
  switch (state) {
    case LoopState::kActive:
      return rgba(0.1F, 0.9F, 0.1F);
    case LoopState::kReleased:
      return rgba(0.1F, 0.8F, 0.9F);
    case LoopState::kRemoved:
      return rgba(0.95F, 0.15F, 0.15F);
  }
  return rgba(1.0F, 1.0F, 1.0F);
}

void keyframe_markers(const GraphSnapshot& snap,
                      mss::common::Timestamp t,
                      const std::string& world_frame,
                      const VizStyle& style,
                      MarkerArray& out) {
  start_array(t, world_frame, out);
  for (const GraphKeyframe& k : snap.keyframes) {
    Marker m = marker_header(t, world_frame);
    m.ns = "keyframes";
    m.id = static_cast<std::int32_t>(k.id);
    m.type = Marker::ARROW;
    m.pose = pose_to_msg(k.T_w_b);
    m.scale.x = style.arrow_length_m;
    m.scale.y = style.arrow_shaft_m;
    m.scale.z = style.arrow_head_m;
    m.color = rgba(1.0F, 0.75F, 0.1F);
    out.markers.push_back(m);
  }
}

void loop_markers(const GraphSnapshot& snap,
                  mss::common::Timestamp t,
                  const std::string& world_frame,
                  const VizStyle& style,
                  MarkerArray& out) {
  start_array(t, world_frame, out);
  for (std::size_t n = 0; n < snap.loops.size(); ++n) {
    const mss::pipeline::GraphLoop& l = snap.loops[n];
    const GraphKeyframe* ki = find_keyframe(snap, l.i);
    const GraphKeyframe* kj = find_keyframe(snap, l.j);
    if (ki == nullptr || kj == nullptr) {
      continue;
    }
    Marker m = marker_header(t, world_frame);
    m.ns = "loops";
    m.id = static_cast<std::int32_t>(n);
    m.type = Marker::LINE_LIST;
    m.scale.x = style.loop_width_m;
    m.color = loop_color(l.state);
    m.points.push_back(point_of(ki->T_w_b.p));
    m.points.push_back(point_of(kj->T_w_b.p));
    out.markers.push_back(m);
  }
}

void path_from_snapshot(const GraphSnapshot& snap,
                        mss::common::Timestamp t,
                        const std::string& world_frame,
                        nav_msgs::msg::Path& out) {
  out.header.stamp = to_msg_time(t);
  out.header.frame_id = world_frame;
  out.poses.clear();
  for (const GraphKeyframe& k : snap.keyframes) {
    out.poses.push_back(pose_stamped(k.T_w_b, k.t, world_frame));
  }
}

Marker counts_marker(const GraphSnapshot& snap,
                     const GraphCounts& counts,
                     mss::common::Timestamp t,
                     const std::string& world_frame,
                     const VizStyle& style) {
  Marker m = marker_header(t, world_frame);
  m.ns = "counts";
  m.id = 0;
  m.type = Marker::TEXT_VIEW_FACING;
  m.scale.z = style.text_height_m;
  m.color = rgba(1.0F, 1.0F, 1.0F);
  if (!snap.keyframes.empty()) {
    m.pose.position = point_of(snap.keyframes.back().T_w_b.p);
  }
  m.pose.position.z += style.text_above_m;
  std::size_t by_state[3] = {0, 0, 0};
  for (const mss::pipeline::GraphLoop& l : snap.loops) {
    ++by_state[static_cast<std::size_t>(l.state)];
  }
  m.text = "keyframes " + std::to_string(counts.keyframes) + " | loops: accepted " +
           std::to_string(counts.loops_accepted) + ", rejected " +
           std::to_string(counts.loops_rejected) + ", PCM rejected " +
           std::to_string(counts.loops_pcm_rejected) + ", removed " +
           std::to_string(counts.loops_removed) + "\nin graph: " + std::to_string(by_state[0]) +
           " " + state_name(LoopState::kActive) + ", " + std::to_string(by_state[1]) + " " +
           state_name(LoopState::kReleased) + "; " + std::to_string(by_state[2]) + " " +
           state_name(LoopState::kRemoved);
  return m;
}

}  // namespace mss_ros2
