#pragma once

#include <array>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/frontend.hpp"
#include "mss2d/loop_closer.hpp"
#include "mss2d/pose_graph.hpp"

namespace mss2d {

struct BackendStats {
  std::uint32_t n_nodes = 0;
  std::uint32_t n_dropped_nodes = 0;  ///< finished submaps not added: graph full
  std::uint32_t n_candidates = 0;     ///< loop candidates verified
  std::uint32_t n_loops = 0;          ///< loop edges kept after the post-solve check
  std::uint32_t n_rejected_score = 0;
  std::uint32_t n_rejected_degenerate = 0;
  std::uint32_t n_rejected_window = 0;
  std::uint32_t n_rejected_chi2 = 0;      ///< dropped after the solve
  std::uint32_t n_rejected_capacity = 0;  ///< edge buffer or factor envelope full
  std::uint32_t n_solves = 0;
};

/// Pose-graph backend (docs/backend.md): one node per finished submap, odometry edges
/// between consecutive submap origins with an information matrix from the travelled path
/// and the matcher's degeneracy flags, loop edges from LoopMatcher, a Levenberg-Marquardt
/// solve after new loops, and the correction handed back to the front end.
///
/// Call on_scan_done() after every FrontEnd::on_scan() from the main loop; it is
/// single-threaded and runs to completion (a node, its loop candidates and at most two
/// solves). No allocation; about 330 KB at the defaults (compact maps 160 KB, solver 150 KB),
/// give it static storage.
class Backend {
 public:
  /// `cfg` must have passed Config::validate().
  explicit Backend(const Config& cfg);

  void on_scan_done(FrontEnd& fe);

  const PoseGraph& graph() const { return graph_; }
  const BackendStats& stats() const { return stats_; }
  /// Node of submap `id`, or -1.
  int node_of_submap(std::uint32_t id) const;
  /// Report of the last solve.
  const SolveReport& last_solve() const { return last_solve_; }

 private:
  struct Started {
    std::uint32_t id = 0;
    double path_m = 0.0;  ///< cumulative path when the submap started
    double deg[3]{0, 0, 0};
  };

  void add_node(FrontEnd& fe, const OccupancyGrid& g, std::uint32_t id);
  Info3 odometry_info(int i, double path_m, const double deg[3], float dyaw, float yaw_j) const;
  int close_loops(const FrontEnd& fe, int j, const Pose2& origin_fe);

  Config cfg_;
  PoseGraph graph_;
  LoopMatcher matcher_;
  std::array<CompactSubmap, kMaxGraphNodes> maps_{};
  std::array<std::uint32_t, kMaxGraphNodes> node_submap_{};
  std::array<double, kMaxGraphNodes> node_path_{};
  std::array<std::array<double, 3>, kMaxGraphNodes> node_deg_{};
  std::array<Started, kSubmapPoolSize> started_{};  ///< ring of recent submap starts
  int n_started_ = 0;

  bool have_prev_ = false;
  Pose2 prev_T_w_b_{};
  double path_m_ = 0.0;     ///< cumulative body path (front-end poses)
  double deg_[3]{0, 0, 0};  ///< cumulative path-weighted degenerate projector (m)
  std::uint32_t seen_started_ = 0;
  std::uint32_t seen_finished_ = 0;
  Pose2 last_origin_fe_{};  ///< origin of the newest node in the front-end frame
  BackendStats stats_{};
  SolveReport last_solve_{};
};

}  // namespace mss2d
