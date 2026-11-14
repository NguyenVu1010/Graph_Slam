#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "mss2d/config.hpp"
#include "mss2d/pose2.hpp"
#include "mss2d/small_matrix.hpp"
#include "mss2d/status.hpp"

namespace mss2d {

using Pose2d = Pose2T<double>;
using Mat3d = Mat3T<double>;

/// Residual of a relative-pose edge (docs/backend.md section 2). Nodes X_i = T_w_i,
/// X_j = T_w_j, measurement Z = T_i_j:
///   e_xy  = R_z^T (R_i^T (t_j - t_i) - t_z)       (m, in the measured frame j)
///   e_yaw = wrap(yaw_j - yaw_i - yaw_z)            (rad)
/// zero when X_i^-1 X_j = Z. Variables X_i, X_j with right perturbation
/// X <- X * Exp(delta), delta = [x, y, yaw]. With S = [[0, -1], [1, 0]], d = R_i^T (t_j - t_i):
///   J_e_xi = [[-R_z^T, -R_z^T S d], [0, 0, -1]]
///   J_e_xj = [[R(yaw_j - yaw_i - yaw_z), 0], [0, 0, 1]]
/// exact at delta = 0 (Exp moves the position by R V(yaw) delta_xy, whose yaw derivative
/// vanishes at delta_xy = 0). Templated so the test checks it in double. Either Jacobian
/// pointer may be null.
template <typename S>
Vec3T<S> edge_residual(const Pose2T<S>& X_i,
                       const Pose2T<S>& X_j,
                       const Pose2T<S>& Z,
                       Mat3T<S>* J_e_xi,
                       Mat3T<S>* J_e_xj) {
  const S ci = std::cos(X_i.yaw);
  const S si = std::sin(X_i.yaw);
  const S cz = std::cos(Z.yaw);
  const S sz = std::sin(Z.yaw);
  const S tx = X_j.x - X_i.x;
  const S ty = X_j.y - X_i.y;
  const S dx = ci * tx + si * ty;  // d = R_i^T (t_j - t_i)
  const S dy = -si * tx + ci * ty;
  const S ux = dx - Z.x;
  const S uy = dy - Z.y;
  Vec3T<S> e;
  e[0] = cz * ux + sz * uy;
  e[1] = -sz * ux + cz * uy;
  e[2] = normalize_angle(X_j.yaw - X_i.yaw - Z.yaw);
  if (J_e_xi != nullptr) {
    Mat3T<S>& J = *J_e_xi;
    J = Mat3T<S>{};
    J(0, 0) = -cz;
    J(0, 1) = -sz;
    J(1, 0) = sz;
    J(1, 1) = -cz;
    // -R_z^T S d, S d = (-dy, dx).
    J(0, 2) = -(cz * -dy + sz * dx);
    J(1, 2) = -(-sz * -dy + cz * dx);
    J(2, 2) = -1;
  }
  if (J_e_xj != nullptr) {
    Mat3T<S>& J = *J_e_xj;
    J = Mat3T<S>{};
    const S phi = X_j.yaw - X_i.yaw - Z.yaw;
    const S cp = std::cos(phi);
    const S sp = std::sin(phi);
    J(0, 0) = cp;
    J(0, 1) = -sp;
    J(1, 0) = sp;
    J(1, 1) = cp;
    J(2, 2) = 1;
  }
  return e;
}

/// Symmetric 3x3 information matrix, upper triangle: xx, xy, x-yaw, yy, y-yaw, yaw-yaw.
struct Info3 {
  float v[6]{0, 0, 0, 0, 0, 0};

  static Info3 diagonal(float sigma_xy, float sigma_yaw);
  Mat3d to_mat() const;
};

enum class EdgeKind : std::uint8_t {
  kOdometry = 0,  ///< quadratic cost
  kLoop,          ///< Cauchy kernel
};

struct GraphEdge {
  std::uint16_t i = 0;  ///< from node
  std::uint16_t j = 0;  ///< to node
  EdgeKind kind = EdgeKind::kOdometry;
  bool active = true;  ///< inactive edges are kept but ignored
  Pose2 Z{};           ///< measured T_i_j
  Info3 info{};
};

enum class SolveStatus : std::uint8_t {
  kRunning = 0,  ///< step() again
  kConverged,
  kMaxIterations,
  kNothingToSolve,  ///< fewer than two nodes
  kTooLarge,        ///< the factor envelope exceeds kMaxFactorEntries
  kNotPositiveDefinite,
};

struct SolveReport {
  SolveStatus status = SolveStatus::kNothingToSolve;
  int iterations = 0;  ///< LM iterations, rejected ones included
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int factor_entries = 0;  ///< envelope size used
};

/// SE(2) pose graph with a Levenberg-Marquardt solver on a fixed buffer
/// (docs/backend.md). Node 0 is held fixed (gauge). The normal equations are assembled into
/// the envelope of a reverse Cuthill-McKee ordering (or of the insertion order when that is
/// smaller) and factored in place by a skyline Cholesky (fill stays inside the envelope).
/// Double precision: see docs/backend.md section 3. No allocation; about 130 KB, give it
/// static storage. Single-threaded.
///
/// Steppable: begin_solve() orders the graph, each step() runs one LM iteration (assemble,
/// factor, solve, accept or reject), so a caller can spread a solve over idle slices.
/// Nodes and edges must not change between begin_solve() and the end of the solve.
class PoseGraph {
 public:
  /// `cfg` must have passed GraphConfig::validate(); `cauchy_c` > 0 (LoopConfig::cauchy_c).
  PoseGraph(const GraphConfig& cfg, float cauchy_c);

  void clear();
  /// Index of the new node, or -1 when kMaxGraphNodes are in use.
  int add_node(const Pose2d& X_w_n);
  /// Index of the new edge, or -1 when full or the nodes do not exist.
  int add_edge(int i, int j, const Pose2& Z_i_j, const Info3& info, EdgeKind kind);
  void set_edge_active(int e, bool active) { edges_[static_cast<std::size_t>(e)].active = active; }
  void set_node(int i, const Pose2d& X) { nodes_[static_cast<std::size_t>(i)] = X; }

  int n_nodes() const { return n_nodes_; }
  int n_edges() const { return n_edges_; }
  const Pose2d& node(int i) const { return nodes_[static_cast<std::size_t>(i)]; }
  const GraphEdge& edge(int e) const { return edges_[static_cast<std::size_t>(e)]; }

  /// e^T Info e of edge `e` at the current estimate (no kernel).
  double chi2(int e) const;
  /// Sum over active edges of the (robust, for loop edges) cost at the current estimate.
  double cost() const;

  /// Orders the variables and sizes the envelope. kRunning on success.
  SolveStatus begin_solve();
  /// One LM iteration; kRunning while not finished.
  SolveStatus step();
  /// begin_solve() then step() until it finishes.
  SolveReport solve();
  /// Report of the current or last solve.
  const SolveReport& report() const { return report_; }

 private:
  static constexpr int kMaxVars = 3 * kMaxGraphNodes;

  double chi2_of(const GraphEdge& e) const;
  /// Cost of an edge with squared Mahalanobis norm s; weight = d cost / d s (IRLS).
  double kernel(EdgeKind kind, double s, double& weight) const;
  double edge_cost(const GraphEdge& e, double& weight) const;
  void order_nodes();
  /// Envelope size (doubles) of the insertion order (natural) or of the current pos_.
  long envelope_entries(bool natural) const;
  bool build_envelope();
  void assemble(double lambda);
  void add_block(int pi, int pj, const Mat3d& m);
  bool factor();
  void solve_in_place();
  int idx(int r, int c) const {
    return off_[static_cast<std::size_t>(r)] + c - first_[static_cast<std::size_t>(r)];
  }

  GraphConfig cfg_;
  double cauchy_c2_;
  std::array<Pose2d, kMaxGraphNodes> nodes_{};
  std::array<GraphEdge, kMaxGraphEdges> edges_{};
  int n_nodes_ = 0;
  int n_edges_ = 0;

  // Solver state.
  std::array<Pose2d, kMaxGraphNodes> backup_{};
  std::array<std::int16_t, kMaxGraphNodes> pos_{};    ///< node -> variable block, -1 fixed
  std::array<std::int16_t, kMaxGraphNodes> order_{};  ///< variable block -> node
  std::array<std::int16_t, kMaxGraphNodes> degree_{};
  std::array<std::int16_t, kMaxGraphNodes + 1> adj_start_{};
  std::array<std::int16_t, 2 * kMaxGraphEdges> adj_{};
  std::array<std::int32_t, kMaxVars> first_{};    ///< first column of each envelope row
  std::array<std::int32_t, kMaxVars + 1> off_{};  ///< row start in val_
  std::array<double, kMaxFactorEntries> val_{};
  std::array<double, kMaxVars> rhs_{};
  int n_vars_ = 0;
  double lambda_ = 0.0;
  double cost_ = 0.0;
  SolveReport report_{};
};

}  // namespace mss2d
