#include "mss2d/frontend.hpp"

#include <cmath>
#include <utility>

namespace mss2d {

namespace {

template <std::size_t... I>
std::array<OccupancyGrid, sizeof...(I)> make_grids(const GridConfig& cfg,
                                                   std::index_sequence<I...> /*unused*/) {
  return {{(static_cast<void>(I), OccupancyGrid(cfg))...}};
}

float seconds_between(Timestamp a, Timestamp b) {
  return static_cast<float>(static_cast<double>(b.ns - a.ns) /
                            static_cast<double>(kNanosecondsPerSecond));
}

}  // namespace

FrontEnd::FrontEnd(const Config& cfg, PoseSink* out)
    : cfg_(cfg),
      out_(out),
      T_b_l_(cfg.lidar.x_m, cfg.lidar.y_m, cfg.lidar.yaw_rad),
      T_l_b_(T_b_l_.inverse()),
      motion_(cfg.motion),
      matcher_(cfg.matcher),
      df_(cfg.grid),
      grids_(make_grids(cfg.grid, std::make_index_sequence<kSubmapPoolSize>{})) {}

void FrontEnd::on_imu(const ImuSample2d& s) {
  motion_.add_imu(s);
}

void FrontEnd::on_wheel(const WheelSample& s) {
  motion_.add_wheel(s);
}

const OccupancyGrid& FrontEnd::finished_submap(int i) const {
  return grids_[static_cast<std::size_t>(finished_[static_cast<std::size_t>(i)])];
}

const OccupancyGrid& FrontEnd::matching_submap() const {
  return grids_[static_cast<std::size_t>(active_[0])];
}

void FrontEnd::on_scan(const Scan2d& scan) {
  const Timestamp t_end = scan.key();
  if (have_pose_ && t_end <= t_prev_) {
    return;  // the merger rejects these already; keep the state consistent anyway
  }
  ++stats_.n_scans;

  // Prediction: body motion from the previous scan end to this one.
  const Pose2 T_prev_now = have_pose_ ? motion_.integrate(t_prev_, t_end) : Pose2();
  const Pose2 T_w_b_pred = T_w_b_ * T_prev_now;

  // Points in the LiDAR frame at the scan end.
  if (cfg_.motion.deskew) {
    deskew_.build(motion_, scan.t_start, t_end, cfg_.motion.deskew_knots, T_b_l_);
    deskew_scan(scan, deskew_, cfg_.matcher.min_range_m, cfg_.matcher.max_range_m, pts_);
  } else {
    scan_to_points(scan, cfg_.matcher.min_range_m, cfg_.matcher.max_range_m, pts_);
  }

  ScanPose out;
  out.t = t_end;
  Pose2 T_w_b = T_w_b_pred;
  if (!have_pose_) {
    start_submap(T_w_b);
    out.status = FrontEndStatus::kInitialised;
  } else {
    const OccupancyGrid& g = grids_[static_cast<std::size_t>(active_[0])];
    const Pose2 guess = g.T_w_s().inverse() * T_w_b_pred * T_b_l_;
    const MatchResult r = matcher_.match(g, df_, pts_, guess);
    out.n_points = r.n_points;
    out.cost_m2 = r.cost_m2;
    out.n_degenerate = r.degeneracy.n_degenerate;
    out.min_eigenvalue = r.degeneracy.eig.values[0];
    if (r.status == MatchStatus::kTooFewPoints) {
      out.status = FrontEndStatus::kPredicted;
    } else {
      stats_.n_not_converged += r.status == MatchStatus::kNotConverged ? 1U : 0U;
      Pose2 T_s_l = r.T_s_l;
      out.status = FrontEndStatus::kOk;
      if (r.degeneracy.n_degenerate > 0) {
        // Keep the prediction along the flagged eigen-directions: remove their component
        // from the correction delta = Log(guess^-1 T_s_l) (the correlative stage may have
        // moved there; Gauss-Newton did not).
        Vec3f delta = (guess.inverse() * T_s_l).log();
        const Mat3f& v = r.degeneracy.eig.vectors;
        for (int k = 0; k < 3; ++k) {
          if (!r.degeneracy.degenerate[k]) {
            continue;
          }
          const float c = v(0, k) * delta[0] + v(1, k) * delta[1] + v(2, k) * delta[2];
          for (int a = 0; a < 3; ++a) {
            delta[a] -= c * v(a, k);
          }
        }
        T_s_l = guess * Pose2::exp(delta);
        out.status = FrontEndStatus::kDegenerate;
      }
      T_w_b = g.T_w_s() * T_s_l * T_l_b_;
    }
    out.submap_id = slots_[static_cast<std::size_t>(active_[0])].id;

    // Constant-velocity twist for the next prediction, and the gyro bias from the yaw the
    // match observed (only when every direction was constrained).
    const float dt = seconds_between(t_prev_, t_end);
    const Pose2 T_prev_new = T_w_b_.inverse() * T_w_b;
    const Vec3f tw = T_prev_new.log();
    motion_.set_constant_velocity(tw[0] / dt, tw[1] / dt, tw[2] / dt);
    if (out.status == FrontEndStatus::kOk && cfg_.motion.gyro_bias_gain > 0.0F) {
      const float err = normalize_angle(T_prev_now.yaw - T_prev_new.yaw);
      motion_.set_gyro_bias(motion_.gyro_bias() + cfg_.motion.gyro_bias_gain * err / dt);
    }
  }

  switch (out.status) {
    case FrontEndStatus::kOk:
      ++stats_.n_ok;
      break;
    case FrontEndStatus::kDegenerate:
      ++stats_.n_degenerate;
      break;
    case FrontEndStatus::kPredicted:
      ++stats_.n_predicted;
      break;
    case FrontEndStatus::kInitialised:
      break;
  }
  if (out.status != FrontEndStatus::kPredicted) {
    insert_and_rotate(T_w_b);
  }
  if (out.status == FrontEndStatus::kInitialised) {
    out.submap_id = slots_[static_cast<std::size_t>(active_[0])].id;
  }

  have_pose_ = true;
  t_prev_ = t_end;
  T_w_b_ = T_w_b;
  out.T_w_b = T_w_b;
  last_ = out;
  if (out_ != nullptr) {
    out_->on_pose(out);
  }
}

void FrontEnd::start_submap(const Pose2& T_w_b) {
  int slot = -1;
  for (int i = 0; i < kSubmapPoolSize; ++i) {
    if (!in_use_[static_cast<std::size_t>(i)]) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    // Pool full: recycle the oldest finished submap (kSubmapPoolSize > 2, so one exists).
    slot = finished_[0];
    for (int i = 1; i < n_finished_; ++i) {
      finished_[static_cast<std::size_t>(i - 1)] = finished_[static_cast<std::size_t>(i)];
    }
    --n_finished_;
    ++stats_.n_submaps_recycled;
  }
  in_use_[static_cast<std::size_t>(slot)] = true;
  // Origin at the body position, axes along the map axes.
  grids_[static_cast<std::size_t>(slot)].reset(Pose2(T_w_b.x, T_w_b.y, 0.0F));
  slots_[static_cast<std::size_t>(slot)] = Slot{next_id_++, 0};
  active_[n_active_++] = slot;
  ++stats_.n_submaps_started;
}

bool FrontEnd::reached(int slot, const Pose2& T_w_b, float fraction) const {
  const Slot& s = slots_[static_cast<std::size_t>(slot)];
  const Pose2& o = grids_[static_cast<std::size_t>(slot)].T_w_s();
  const float dx = T_w_b.x - o.x;
  const float dy = T_w_b.y - o.y;
  const float d = fraction * cfg_.submap.max_distance_m;
  return static_cast<float>(s.n_scans) >=
             fraction * static_cast<float>(cfg_.submap.scans_per_submap) ||
         dx * dx + dy * dy >= d * d;
}

void FrontEnd::insert_and_rotate(const Pose2& T_w_b) {
  const Pose2 T_w_l = T_w_b * T_b_l_;
  for (int i = 0; i < n_active_; ++i) {
    OccupancyGrid& g = grids_[static_cast<std::size_t>(active_[i])];
    g.insert(pts_, g.T_w_s().inverse() * T_w_l);
    ++slots_[static_cast<std::size_t>(active_[i])].n_scans;
  }
  // Finish the older submap at the full limit; the newer one (about half full) takes over.
  if (n_active_ == 2 && reached(active_[0], T_w_b, 1.0F)) {
    finished_[static_cast<std::size_t>(n_finished_++)] = active_[0];
    active_[0] = active_[1];
    n_active_ = 1;
    ++stats_.n_submaps_finished;
  }
  // Start the next one when the newest is half full, so consecutive submaps overlap.
  if (n_active_ == 1 && reached(active_[0], T_w_b, 0.5F)) {
    start_submap(T_w_b);
  }
  df_.compute(grids_[static_cast<std::size_t>(active_[0])]);
}

}  // namespace mss2d
