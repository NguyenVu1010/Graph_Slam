#include "mss2d/backend.hpp"

#include <cmath>

namespace mss2d {

namespace {

Pose2d to_double(const Pose2& p) {
  return Pose2d(p.x, p.y, p.yaw);
}

Pose2 to_float(const Pose2d& p) {
  return Pose2(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.yaw));
}

float clamp_window(float lo, float per_m, double path_m, float hi) {
  const float w = lo + per_m * static_cast<float>(path_m);
  return w < hi ? w : hi;
}

}  // namespace

Backend::Backend(const Config& cfg)
    : cfg_(cfg), graph_(cfg.graph, cfg.loop.cauchy_c), matcher_(cfg) {}

int Backend::node_of_submap(std::uint32_t id) const {
  for (int k = 0; k < graph_.n_nodes(); ++k) {
    if (node_submap_[static_cast<std::size_t>(k)] == id) {
      return k;
    }
  }
  return -1;
}

void Backend::on_scan_done(FrontEnd& fe) {
  if (!fe.has_pose()) {
    return;
  }
  // Travelled path and travel along flagged directions, from consecutive front-end poses.
  const ScanPose& p = fe.last_pose();
  if (have_prev_) {
    const double d = std::hypot(static_cast<double>(p.T_w_b.x - prev_T_w_b_.x),
                                static_cast<double>(p.T_w_b.y - prev_T_w_b_.y));
    path_m_ += d;
    deg_[0] += d * static_cast<double>(p.degenerate_xy_w.xx);
    deg_[1] += d * static_cast<double>(p.degenerate_xy_w.xy);
    deg_[2] += d * static_cast<double>(p.degenerate_xy_w.yy);
  }
  prev_T_w_b_ = p.T_w_b;
  have_prev_ = true;

  // Submap ids are sequential: the k-th submap started has id k - 1.
  const FrontEndStats& st = fe.stats();
  while (seen_started_ < st.n_submaps_started) {
    Started& s = started_[static_cast<std::size_t>(n_started_ % kSubmapPoolSize)];
    s.id = seen_started_++;
    s.path_m = path_m_;
    for (int a = 0; a < 3; ++a) {
      s.deg[a] = deg_[a];
    }
    ++n_started_;
  }
  // The front end finishes at most one submap per scan.
  if (st.n_submaps_finished > seen_finished_) {
    seen_finished_ = st.n_submaps_finished;
    const int k = fe.n_finished_submaps() - 1;
    add_node(fe, fe.finished_submap(k), fe.finished_submap_id(k));
  }
}

Info3 Backend::odometry_info(
    int i, double path_m, const double deg[3], float dyaw, float yaw_j) const {
  const GraphConfig& g = cfg_.graph;
  const double d = path_m - node_path_[static_cast<std::size_t>(i)];
  const double sxy = static_cast<double>(g.odom_sigma_xy_m) + g.odom_sigma_xy_per_m * d;
  const double syaw = static_cast<double>(g.odom_sigma_yaw_rad) + g.odom_sigma_yaw_per_m * d +
                      g.odom_sigma_yaw_per_rad * std::abs(static_cast<double>(dyaw));
  // Sigma_xy = sxy^2 I + k^2 tr(D) D, D = sum over the scans of step * u u^T (m): for travel
  // L along one flagged direction u, k L along u, the wheel scale error (docs/backend.md).
  double dm[3];
  for (int a = 0; a < 3; ++a) {
    dm[a] = deg[a] - node_deg_[static_cast<std::size_t>(i)][static_cast<std::size_t>(a)];
  }
  const double k2 = static_cast<double>(g.odom_degenerate_fraction) * g.odom_degenerate_fraction;
  const double tr = dm[0] + dm[2];
  const double wxx = sxy * sxy + k2 * tr * dm[0];
  const double wxy = k2 * tr * dm[1];
  const double wyy = sxy * sxy + k2 * tr * dm[2];
  // Into frame j (the residual's frame): R^T W R.
  const double c = std::cos(static_cast<double>(yaw_j));
  const double s = std::sin(static_cast<double>(yaw_j));
  const double axx = c * c * wxx + 2.0 * c * s * wxy + s * s * wyy;
  const double axy = -c * s * wxx + (c * c - s * s) * wxy + c * s * wyy;
  const double ayy = s * s * wxx - 2.0 * c * s * wxy + c * c * wyy;
  const double det = axx * ayy - axy * axy;
  Info3 info;
  info.v[0] = static_cast<float>(ayy / det);
  info.v[1] = static_cast<float>(-axy / det);
  info.v[3] = static_cast<float>(axx / det);
  info.v[5] = static_cast<float>(1.0 / (syaw * syaw));
  return info;
}

void Backend::add_node(FrontEnd& fe, const OccupancyGrid& g, std::uint32_t id) {
  if (graph_.n_nodes() >= kMaxGraphNodes) {
    ++stats_.n_dropped_nodes;
    return;
  }
  const Pose2 origin = g.T_w_s();
  // Path and degenerate travel when this submap started (current values if not recorded).
  double path_s = path_m_;
  double deg_s[3] = {deg_[0], deg_[1], deg_[2]};
  for (int k = 0; k < kSubmapPoolSize && k < n_started_; ++k) {
    const Started& s = started_[static_cast<std::size_t>(k)];
    if (s.id == id) {
      path_s = s.path_m;
      for (int a = 0; a < 3; ++a) {
        deg_s[a] = s.deg[a];
      }
    }
  }

  int j = 0;
  if (graph_.n_nodes() == 0) {
    j = graph_.add_node(to_double(origin));
  } else {
    // The graph and front-end frames coincide at the newest node (corrections keep it so).
    const int i = graph_.n_nodes() - 1;
    const Pose2 Z = last_origin_fe_.inverse() * origin;
    j = graph_.add_node(graph_.node(i) * to_double(Z));
    const Info3 info = odometry_info(i, path_s, deg_s, Z.yaw, origin.yaw);
    (void)graph_.add_edge(i, j, Z, info, EdgeKind::kOdometry);  // room is kept for it
  }
  node_submap_[static_cast<std::size_t>(j)] = id;
  node_path_[static_cast<std::size_t>(j)] = path_s;
  for (int a = 0; a < 3; ++a) {
    node_deg_[static_cast<std::size_t>(j)][static_cast<std::size_t>(a)] = deg_s[a];
  }
  compact_submap(g, maps_[static_cast<std::size_t>(j)]);
  last_origin_fe_ = origin;
  ++stats_.n_nodes;
  if (!cfg_.loop.enabled || j == 0) {
    return;
  }

  std::array<int, kMaxLoopCandidates> added{};
  const int n_added = close_loops(fe, j, origin);
  for (int k = 0; k < n_added; ++k) {
    added[static_cast<std::size_t>(k)] = graph_.n_edges() - n_added + k;
  }
  if (n_added == 0) {
    return;
  }
  SolveReport r = graph_.solve();
  ++stats_.n_solves;
  if (r.status == SolveStatus::kTooLarge) {
    for (int k = 0; k < n_added; ++k) {
      graph_.set_edge_active(added[static_cast<std::size_t>(k)], false);
    }
    stats_.n_rejected_capacity += static_cast<std::uint32_t>(n_added);
    last_solve_ = r;
    return;
  }
  // Consistency with the rest of the graph: a new loop the kernel left with a large chi2
  // disagrees with the odometry and the older loops; drop it and solve again.
  bool dropped = false;
  for (int k = 0; k < n_added; ++k) {
    const int e = added[static_cast<std::size_t>(k)];
    if (graph_.chi2(e) > static_cast<double>(cfg_.loop.max_chi2)) {
      graph_.set_edge_active(e, false);
      ++stats_.n_rejected_chi2;
      dropped = true;
    } else {
      ++stats_.n_loops;
    }
  }
  if (dropped) {
    r = graph_.solve();
    ++stats_.n_solves;
  }
  last_solve_ = r;

  // Hand the correction of the newest node to the front end.
  const Pose2 C = to_float(graph_.node(j)) * origin.inverse();
  fe.apply_correction(C);
  last_origin_fe_ = C * origin;
  prev_T_w_b_ = C * prev_T_w_b_;
}

int Backend::close_loops(const FrontEnd& fe, int j, const Pose2& origin) {
  const LoopConfig& lc = cfg_.loop;
  // Query: the last scan, which the just finished submap j contains.
  const Pose2 T_sj_l = origin.inverse() * (fe.last_pose().T_w_b * fe.T_b_l());
  const Pose2d q = graph_.node(j) * to_double(T_sj_l);

  // Nearest candidates inside the drift-grown radius, at least min_path_m back.
  std::array<int, kMaxLoopCandidates> cand{};
  std::array<double, kMaxLoopCandidates> cand_d{};
  int n_cand = 0;
  for (int i = 0; i < j; ++i) {
    const double path = path_m_ - node_path_[static_cast<std::size_t>(i)];
    if (path < static_cast<double>(lc.min_path_m)) {
      continue;
    }
    const Pose2d& X = graph_.node(i);
    const double dist = std::hypot(q.x - X.x, q.y - X.y);
    if (dist > static_cast<double>(lc.search_radius_m) + lc.drift_per_m * path) {
      continue;
    }
    int k = 0;
    if (n_cand < lc.max_candidates) {
      k = n_cand++;
    } else if (dist < cand_d[static_cast<std::size_t>(n_cand - 1)]) {
      k = n_cand - 1;  // replaces the farthest
    } else {
      continue;
    }
    while (k > 0 && cand_d[static_cast<std::size_t>(k - 1)] > dist) {
      cand[static_cast<std::size_t>(k)] = cand[static_cast<std::size_t>(k - 1)];
      cand_d[static_cast<std::size_t>(k)] = cand_d[static_cast<std::size_t>(k - 1)];
      --k;
    }
    cand[static_cast<std::size_t>(k)] = i;
    cand_d[static_cast<std::size_t>(k)] = dist;
  }

  int n_added = 0;
  for (int c = 0; c < n_cand; ++c) {
    const int i = cand[static_cast<std::size_t>(c)];
    // Keep room for the odometry edges of the nodes still to come.
    if (graph_.n_edges() >= kMaxGraphEdges - (kMaxGraphNodes - graph_.n_nodes())) {
      ++stats_.n_rejected_capacity;
      continue;
    }
    const double path = path_m_ - node_path_[static_cast<std::size_t>(i)];
    const float wm = clamp_window(lc.min_window_m, lc.drift_per_m, path, lc.max_window_m);
    const float wr = clamp_window(lc.min_window_rad, lc.drift_rad_per_m, path, lc.max_window_rad);
    const Pose2 guess = to_float(graph_.node(i).inverse() * q);
    ++stats_.n_candidates;
    const LoopMatch m =
        matcher_.match(maps_[static_cast<std::size_t>(i)], fe.last_points(), guess, wm, wr);
    switch (m.status) {
      case LoopStatus::kAccepted:
        break;
      case LoopStatus::kDegenerate:
        ++stats_.n_rejected_degenerate;
        continue;
      case LoopStatus::kOutsideWindow:
        ++stats_.n_rejected_window;
        continue;
      case LoopStatus::kLowScore:
      case LoopStatus::kTooFewPoints:
        ++stats_.n_rejected_score;
        continue;
    }
    const Pose2 Z = m.T_s_l * T_sj_l.inverse();  // T_si_sj
    if (graph_.add_edge(
            i, j, Z, Info3::diagonal(lc.sigma_xy_m, lc.sigma_yaw_rad), EdgeKind::kLoop) >= 0) {
      ++n_added;
    }
  }
  return n_added;
}

}  // namespace mss2d
