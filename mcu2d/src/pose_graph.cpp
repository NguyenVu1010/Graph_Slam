#include "mss2d/pose_graph.hpp"

#include <cmath>

namespace mss2d {

namespace {

constexpr double kLambdaMin = 1e-12;
constexpr double kLambdaMax = 1e12;
constexpr double kLambdaFactor = 10.0;

/// m = A^T B for 3x3 A, B.
Mat3d at_b(const Mat3d& a, const Mat3d& b) {
  Mat3d m;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a(k, r) * b(k, c);
      }
      m(r, c) = s;
    }
  }
  return m;
}

Mat3d mul(const Mat3d& a, const Mat3d& b) {
  Mat3d m;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a(r, k) * b(k, c);
      }
      m(r, c) = s;
    }
  }
  return m;
}

}  // namespace

Info3 Info3::diagonal(float sigma_xy, float sigma_yaw) {
  Info3 i;
  i.v[0] = 1.0F / (sigma_xy * sigma_xy);
  i.v[3] = i.v[0];
  i.v[5] = 1.0F / (sigma_yaw * sigma_yaw);
  return i;
}

Mat3d Info3::to_mat() const {
  Mat3d m;
  m(0, 0) = v[0];
  m(0, 1) = m(1, 0) = v[1];
  m(0, 2) = m(2, 0) = v[2];
  m(1, 1) = v[3];
  m(1, 2) = m(2, 1) = v[4];
  m(2, 2) = v[5];
  return m;
}

PoseGraph::PoseGraph(const GraphConfig& cfg, float cauchy_c)
    : cfg_(cfg), cauchy_c2_(static_cast<double>(cauchy_c) * static_cast<double>(cauchy_c)) {}

void PoseGraph::clear() {
  n_nodes_ = 0;
  n_edges_ = 0;
  report_ = SolveReport{};
}

int PoseGraph::add_node(const Pose2d& X_w_n) {
  if (n_nodes_ >= kMaxGraphNodes) {
    return -1;
  }
  nodes_[static_cast<std::size_t>(n_nodes_)] = X_w_n;
  return n_nodes_++;
}

int PoseGraph::add_edge(int i, int j, const Pose2& Z_i_j, const Info3& info, EdgeKind kind) {
  if (n_edges_ >= kMaxGraphEdges || i < 0 || j < 0 || i >= n_nodes_ || j >= n_nodes_ || i == j) {
    return -1;
  }
  GraphEdge& e = edges_[static_cast<std::size_t>(n_edges_)];
  e.i = static_cast<std::uint16_t>(i);
  e.j = static_cast<std::uint16_t>(j);
  e.kind = kind;
  e.active = true;
  e.Z = Z_i_j;
  e.info = info;
  return n_edges_++;
}

double PoseGraph::chi2(int ei) const {
  return chi2_of(edges_[static_cast<std::size_t>(ei)]);
}

double PoseGraph::chi2_of(const GraphEdge& e) const {
  const Pose2d Z(e.Z.x, e.Z.y, e.Z.yaw);
  const Vec3T<double> r = edge_residual<double>(nodes_[e.i], nodes_[e.j], Z, nullptr, nullptr);
  const Mat3d W = e.info.to_mat();
  double s = 0.0;
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      s += r[a] * W(a, b) * r[b];
    }
  }
  return s;
}

double PoseGraph::edge_cost(const GraphEdge& e, double& weight) const {
  return kernel(e.kind, chi2_of(e), weight);
}

double PoseGraph::kernel(EdgeKind kind, double s, double& weight) const {
  if (kind == EdgeKind::kLoop) {
    // Cauchy: rho(s) = c^2 ln(1 + s / c^2), IRLS weight rho'(s) = 1 / (1 + s / c^2).
    weight = 1.0 / (1.0 + s / cauchy_c2_);
    return cauchy_c2_ * std::log1p(s / cauchy_c2_);
  }
  weight = 1.0;
  return s;
}

double PoseGraph::cost() const {
  double c = 0.0;
  for (int k = 0; k < n_edges_; ++k) {
    const GraphEdge& e = edges_[static_cast<std::size_t>(k)];
    if (e.active) {
      double w = 0.0;
      c += edge_cost(e, w);
    }
  }
  return c;
}

void PoseGraph::order_nodes() {
  // Adjacency (CSR) of the free nodes 1..n-1; edges to the fixed node 0 add no structure.
  const int n = n_nodes_;
  degree_.fill(0);
  for (int k = 0; k < n_edges_; ++k) {
    const GraphEdge& e = edges_[static_cast<std::size_t>(k)];
    if (e.active && e.i != 0 && e.j != 0) {
      ++degree_[e.i];
      ++degree_[e.j];
    }
  }
  adj_start_[0] = 0;
  for (int v = 0; v < n; ++v) {
    adj_start_[static_cast<std::size_t>(v + 1)] = static_cast<std::int16_t>(
        adj_start_[static_cast<std::size_t>(v)] + degree_[static_cast<std::size_t>(v)]);
  }
  // Fill, using pos_ as the per-node write cursor.
  for (int v = 0; v < n; ++v) {
    pos_[static_cast<std::size_t>(v)] = adj_start_[static_cast<std::size_t>(v)];
  }
  for (int k = 0; k < n_edges_; ++k) {
    const GraphEdge& e = edges_[static_cast<std::size_t>(k)];
    if (e.active && e.i != 0 && e.j != 0) {
      adj_[static_cast<std::size_t>(pos_[e.i]++)] = static_cast<std::int16_t>(e.j);
      adj_[static_cast<std::size_t>(pos_[e.j]++)] = static_cast<std::int16_t>(e.i);
    }
  }

  // Cuthill-McKee: breadth-first from a lowest-degree node of each component, neighbours by
  // increasing degree; then reversed. pos_ = -2 marks unvisited.
  for (int v = 0; v < n; ++v) {
    pos_[static_cast<std::size_t>(v)] = -2;
  }
  pos_[0] = -1;
  int count = 0;
  const int n_free = n - 1;
  while (count < n_free) {
    int start = -1;
    for (int v = 1; v < n; ++v) {
      if (pos_[static_cast<std::size_t>(v)] == -2 &&
          (start < 0 ||
           degree_[static_cast<std::size_t>(v)] < degree_[static_cast<std::size_t>(start)])) {
        start = v;
      }
    }
    pos_[static_cast<std::size_t>(start)] = 0;  // visited
    order_[static_cast<std::size_t>(count++)] = static_cast<std::int16_t>(start);
    for (int head = count - 1; head < count; ++head) {
      const int v = order_[static_cast<std::size_t>(head)];
      const int seg = count;
      for (int a = adj_start_[static_cast<std::size_t>(v)];
           a < adj_start_[static_cast<std::size_t>(v + 1)];
           ++a) {
        const int u = adj_[static_cast<std::size_t>(a)];
        if (pos_[static_cast<std::size_t>(u)] != -2) {
          continue;
        }
        pos_[static_cast<std::size_t>(u)] = 0;
        // Insertion by degree (stable: ties keep adjacency order).
        int k = count++;
        while (k > seg &&
               degree_[static_cast<std::size_t>(order_[static_cast<std::size_t>(k - 1)])] >
                   degree_[static_cast<std::size_t>(u)]) {
          order_[static_cast<std::size_t>(k)] = order_[static_cast<std::size_t>(k - 1)];
          --k;
        }
        order_[static_cast<std::size_t>(k)] = static_cast<std::int16_t>(u);
      }
    }
  }
  // Reverse and record the positions.
  for (int a = 0, b = n_free - 1; a < b; ++a, --b) {
    const std::int16_t t = order_[static_cast<std::size_t>(a)];
    order_[static_cast<std::size_t>(a)] = order_[static_cast<std::size_t>(b)];
    order_[static_cast<std::size_t>(b)] = t;
  }
  for (int p = 0; p < n_free; ++p) {
    pos_[static_cast<std::size_t>(order_[static_cast<std::size_t>(p)])] =
        static_cast<std::int16_t>(p);
  }
  // Reverse Cuthill-McKee narrows graphs with many loops; with a few chords the insertion
  // order (a chain) can be smaller. Keep the smaller envelope.
  if (envelope_entries(true) <= envelope_entries(false)) {
    for (int v = 1; v < n; ++v) {
      pos_[static_cast<std::size_t>(v)] = static_cast<std::int16_t>(v - 1);
      order_[static_cast<std::size_t>(v - 1)] = static_cast<std::int16_t>(v);
    }
  }
}

long PoseGraph::envelope_entries(bool natural) const {
  // A block row reaching back b blocks holds 3 rows of 3 b + 1, 3 b + 2, 3 b + 3 entries.
  long total = 0;
  for (int v = 1; v < n_nodes_; ++v) {
    const int p = natural ? v - 1 : pos_[static_cast<std::size_t>(v)];
    int lo = p;
    for (int a = adj_start_[static_cast<std::size_t>(v)];
         a < adj_start_[static_cast<std::size_t>(v + 1)];
         ++a) {
      const int u = adj_[static_cast<std::size_t>(a)];
      const int q = natural ? u - 1 : pos_[static_cast<std::size_t>(u)];
      lo = q < lo ? q : lo;
    }
    total += 9L * (p - lo) + 6L;
  }
  return total;
}

bool PoseGraph::build_envelope() {
  const int n_free = n_nodes_ - 1;
  n_vars_ = 3 * n_free;
  off_[0] = 0;
  for (int p = 0; p < n_free; ++p) {
    const int v = order_[static_cast<std::size_t>(p)];
    int lo = p;
    for (int a = adj_start_[static_cast<std::size_t>(v)];
         a < adj_start_[static_cast<std::size_t>(v + 1)];
         ++a) {
      const int q = pos_[static_cast<std::size_t>(adj_[static_cast<std::size_t>(a)])];
      lo = q < lo ? q : lo;
    }
    for (int k = 0; k < 3; ++k) {
      const int r = 3 * p + k;
      first_[static_cast<std::size_t>(r)] = 3 * lo;
      const std::int32_t next = off_[static_cast<std::size_t>(r)] + (r - 3 * lo + 1);
      if (next > kMaxFactorEntries) {
        return false;
      }
      off_[static_cast<std::size_t>(r + 1)] = next;
    }
  }
  report_.factor_entries = off_[static_cast<std::size_t>(n_vars_)];
  return true;
}

void PoseGraph::add_block(int pi, int pj, const Mat3d& m) {
  // m is the (pi, pj) block of H; store the lower triangle (row >= column).
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      const int r = 3 * pi + a;
      const int c = 3 * pj + b;
      if (r >= c) {
        val_[static_cast<std::size_t>(idx(r, c))] += m(a, b);
      }
    }
  }
}

void PoseGraph::assemble(double lambda) {
  for (int k = 0; k < off_[static_cast<std::size_t>(n_vars_)]; ++k) {
    val_[static_cast<std::size_t>(k)] = 0.0;
  }
  for (int k = 0; k < n_vars_; ++k) {
    rhs_[static_cast<std::size_t>(k)] = 0.0;
  }
  for (int k = 0; k < n_edges_; ++k) {
    const GraphEdge& e = edges_[static_cast<std::size_t>(k)];
    if (!e.active) {
      continue;
    }
    Mat3d Ji;
    Mat3d Jj;
    const Pose2d Z(e.Z.x, e.Z.y, e.Z.yaw);
    const Vec3T<double> r = edge_residual<double>(nodes_[e.i], nodes_[e.j], Z, &Ji, &Jj);
    Mat3d W = e.info.to_mat();
    double s = 0.0;
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        s += r[a] * W(a, b) * r[b];
      }
    }
    double w = 1.0;
    (void)kernel(e.kind, s, w);
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        W(a, b) *= w;
      }
    }
    const int pi = pos_[e.i];
    const int pj = pos_[e.j];
    const Mat3d WJi = mul(W, Ji);
    const Mat3d WJj = mul(W, Jj);
    if (pi >= 0) {
      add_block(pi, pi, at_b(Ji, WJi));
      for (int a = 0; a < 3; ++a) {
        double g = 0.0;
        for (int b = 0; b < 3; ++b) {
          g += WJi(b, a) * r[b];
        }
        rhs_[static_cast<std::size_t>(3 * pi + a)] -= g;
      }
    }
    if (pj >= 0) {
      add_block(pj, pj, at_b(Jj, WJj));
      for (int a = 0; a < 3; ++a) {
        double g = 0.0;
        for (int b = 0; b < 3; ++b) {
          g += WJj(b, a) * r[b];
        }
        rhs_[static_cast<std::size_t>(3 * pj + a)] -= g;
      }
    }
    if (pi >= 0 && pj >= 0) {
      if (pi > pj) {
        add_block(pi, pj, at_b(Ji, WJj));
      } else {
        add_block(pj, pi, at_b(Jj, WJi));
      }
    }
  }
  // Marquardt damping: H + lambda diag(H).
  for (int r = 0; r < n_vars_; ++r) {
    val_[static_cast<std::size_t>(idx(r, r))] *= 1.0 + lambda;
  }
}

bool PoseGraph::factor() {
  // Row-wise skyline Cholesky H = L L^T in place; L(r, c) = 0 left of first_[r].
  for (int r = 0; r < n_vars_; ++r) {
    const int fr = first_[static_cast<std::size_t>(r)];
    for (int c = fr; c <= r; ++c) {
      const int fc = first_[static_cast<std::size_t>(c)];
      const int k0 = fr > fc ? fr : fc;
      double s = val_[static_cast<std::size_t>(idx(r, c))];
      const double* lr = &val_[static_cast<std::size_t>(idx(r, k0))];
      const double* lc = &val_[static_cast<std::size_t>(idx(c, k0))];
      for (int k = 0; k < c - k0; ++k) {
        s -= lr[k] * lc[k];
      }
      if (c < r) {
        val_[static_cast<std::size_t>(idx(r, c))] = s / val_[static_cast<std::size_t>(idx(c, c))];
      } else {
        if (!(s > 0.0)) {
          return false;
        }
        val_[static_cast<std::size_t>(idx(r, r))] = std::sqrt(s);
      }
    }
  }
  return true;
}

void PoseGraph::solve_in_place() {
  // L y = rhs, then L^T x = y (column sweep), both in rhs_.
  for (int r = 0; r < n_vars_; ++r) {
    const int fr = first_[static_cast<std::size_t>(r)];
    double s = rhs_[static_cast<std::size_t>(r)];
    for (int k = fr; k < r; ++k) {
      s -= val_[static_cast<std::size_t>(idx(r, k))] * rhs_[static_cast<std::size_t>(k)];
    }
    rhs_[static_cast<std::size_t>(r)] = s / val_[static_cast<std::size_t>(idx(r, r))];
  }
  for (int r = n_vars_ - 1; r >= 0; --r) {
    const double x = rhs_[static_cast<std::size_t>(r)] / val_[static_cast<std::size_t>(idx(r, r))];
    rhs_[static_cast<std::size_t>(r)] = x;
    for (int k = first_[static_cast<std::size_t>(r)]; k < r; ++k) {
      rhs_[static_cast<std::size_t>(k)] -= val_[static_cast<std::size_t>(idx(r, k))] * x;
    }
  }
}

SolveStatus PoseGraph::begin_solve() {
  report_ = SolveReport{};
  if (n_nodes_ < 2) {
    report_.status = SolveStatus::kNothingToSolve;
    return report_.status;
  }
  order_nodes();
  if (!build_envelope()) {
    report_.status = SolveStatus::kTooLarge;
    return report_.status;
  }
  lambda_ = static_cast<double>(cfg_.initial_lambda);
  cost_ = cost();
  report_.initial_cost = cost_;
  report_.final_cost = cost_;
  report_.status = SolveStatus::kRunning;
  return report_.status;
}

SolveStatus PoseGraph::step() {
  if (report_.status != SolveStatus::kRunning) {
    return report_.status;
  }
  ++report_.iterations;
  assemble(lambda_);
  bool accepted = false;
  double max_step = 0.0;
  double new_cost = cost_;
  if (factor()) {
    solve_in_place();
    for (int v = 1; v < n_nodes_; ++v) {
      const int p = pos_[static_cast<std::size_t>(v)];
      Vec3T<double> d;
      for (int a = 0; a < 3; ++a) {
        d[a] = rhs_[static_cast<std::size_t>(3 * p + a)];
        max_step = std::abs(d[a]) > max_step ? std::abs(d[a]) : max_step;
      }
      backup_[static_cast<std::size_t>(v)] = nodes_[static_cast<std::size_t>(v)];
      nodes_[static_cast<std::size_t>(v)] = nodes_[static_cast<std::size_t>(v)] * Pose2d::exp(d);
    }
    new_cost = cost();
    accepted = new_cost <= cost_;
    if (!accepted) {
      for (int v = 1; v < n_nodes_; ++v) {
        nodes_[static_cast<std::size_t>(v)] = backup_[static_cast<std::size_t>(v)];
      }
    }
  } else if (lambda_ >= kLambdaMax) {
    report_.status = SolveStatus::kNotPositiveDefinite;
    return report_.status;
  }

  if (accepted) {
    const double decrease = cost_ - new_cost;
    cost_ = new_cost;
    report_.final_cost = cost_;
    lambda_ = lambda_ / kLambdaFactor > kLambdaMin ? lambda_ / kLambdaFactor : kLambdaMin;
    if (max_step < static_cast<double>(cfg_.converged_step) ||
        decrease <= static_cast<double>(cfg_.converged_rel_cost) * cost_) {
      report_.status = SolveStatus::kConverged;
      return report_.status;
    }
  } else {
    lambda_ = lambda_ * kLambdaFactor < kLambdaMax ? lambda_ * kLambdaFactor : kLambdaMax;
    // A rejected step this small means the estimate cannot improve any more.
    if (max_step > 0.0 && max_step < static_cast<double>(cfg_.converged_step)) {
      report_.status = SolveStatus::kConverged;
      return report_.status;
    }
  }
  if (report_.iterations >= cfg_.max_iterations) {
    report_.status = SolveStatus::kMaxIterations;
  }
  return report_.status;
}

SolveReport PoseGraph::solve() {
  SolveStatus s = begin_solve();
  while (s == SolveStatus::kRunning) {
    s = step();
  }
  return report_;
}

}  // namespace mss2d
