#include "keyframe_policy.hpp"

#include <stdexcept>

#include "mss/common/lie.hpp"

namespace mss::pipeline::detail {

namespace {

double rotation_angle_rad(const common::Mat3& R_a, const common::Mat3& R_b) {
  const common::Mat3 dR = R_a.transpose() * R_b;
  return common::so3_log(dR).norm();
}

std::int64_t abs_ns(std::int64_t v) {
  return v < 0 ? -v : v;
}

}  // namespace

bool is_new_keyframe(const common::Pose& T_w_b_ref,
                     const common::Pose& T_w_b,
                     const backend::BackendConfig& cfg) {
  const double dist_m = (T_w_b.p - T_w_b_ref.p).norm();
  return dist_m >= cfg.keyframe_dist_m ||
         rotation_angle_rad(T_w_b_ref.R, T_w_b.R) >= cfg.keyframe_rot_rad;
}

bool pose_moved(const common::Pose& a, const common::Pose& b, double trans_m, double rot_rad) {
  const double dist_m = (b.p - a.p).norm();
  return dist_m > trans_m || rotation_angle_rad(a.R, b.R) > rot_rad;
}

bool odom_delta_in_time(const common::OdomDelta& d,
                        common::Timestamp t_i,
                        common::Timestamp t_j,
                        std::int64_t period_ns) {
  if (period_ns <= 0) {
    return false;
  }
  const std::int64_t limit = 3 * period_ns;
  return 2 * abs_ns(d.t0.ns - t_i.ns) <= limit && 2 * abs_ns(d.t1.ns - t_j.ns) <= limit;
}

MapWindow::MapWindow(std::size_t window_keyframes) {
  if (window_keyframes == 0) {
    throw std::invalid_argument("map.window_keyframes: must be > 0");
  }
  slots_.resize(window_keyframes);
}

void MapWindow::push(common::KeyId id, const common::Pose& T_w_b) {
  if (size_ == slots_.size()) {
    head_ = (head_ + 1) % slots_.size();
    --size_;
  }
  WindowEntry& e = slots_[(head_ + size_) % slots_.size()];
  e.id = id;
  e.T_w_b_inserted = T_w_b;
  ++size_;
}

void MapWindow::set_pose(std::size_t i, const common::Pose& T_w_b) {
  slots_[(head_ + i) % slots_.size()].T_w_b_inserted = T_w_b;
}

}  // namespace mss::pipeline::detail
