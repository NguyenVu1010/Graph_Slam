#include "mss2d/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace mss2d {
namespace {

constexpr double kPi = 3.14159265358979323846;

Pose2 to_float(const Pose2d& p) {
  return Pose2(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.yaw));
}

double pose_error(const Pose2d& a, const Pose2d& b) {
  return std::hypot(a.x - b.x, a.y - b.y) + std::abs(normalize_angle(a.yaw - b.yaw));
}

// --- Jacobians ---

TEST(PoseGraphEdge, ResidualIsZeroForAConsistentMeasurement) {
  const Pose2d Xi(1.0, -2.0, 0.7);
  const Pose2d Xj(-0.5, 3.0, -2.9);
  const Vec3T<double> e = edge_residual<double>(Xi, Xj, Xi.inverse() * Xj, nullptr, nullptr);
  EXPECT_NEAR(e[0], 0.0, 1e-12);
  EXPECT_NEAR(e[1], 0.0, 1e-12);
  EXPECT_NEAR(e[2], 0.0, 1e-12);
}

TEST(PoseGraphEdge, AnalyticJacobiansMatchCentralDifferences) {
  // Residual e(X_i Exp(d_i), X_j Exp(d_j)); right perturbation, tangent [x, y, yaw].
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  std::uniform_real_distribution<double> a(-1.2, 1.2);
  const double h = 1e-6;
  for (int trial = 0; trial < 50; ++trial) {
    const Pose2d Xi(u(rng), u(rng), a(rng));
    const Pose2d Xj(u(rng), u(rng), a(rng));
    // Measurement off the estimate so the residual is not zero (yaw error kept off +-pi).
    const Pose2d Z = Xi.inverse() * Xj * Pose2d(0.3 * u(rng), 0.3 * u(rng), 0.3 * a(rng));
    Mat3d Ji;
    Mat3d Jj;
    (void)edge_residual<double>(Xi, Xj, Z, &Ji, &Jj);
    double num = 0.0;
    double den = 0.0;
    for (int k = 0; k < 3; ++k) {
      Vec3T<double> d;
      d[k] = h;
      Vec3T<double> m;
      m[k] = -h;
      const Vec3T<double> ip = edge_residual<double>(Xi * Pose2d::exp(d), Xj, Z, nullptr, nullptr);
      const Vec3T<double> im = edge_residual<double>(Xi * Pose2d::exp(m), Xj, Z, nullptr, nullptr);
      const Vec3T<double> jp = edge_residual<double>(Xi, Xj * Pose2d::exp(d), Z, nullptr, nullptr);
      const Vec3T<double> jm = edge_residual<double>(Xi, Xj * Pose2d::exp(m), Z, nullptr, nullptr);
      for (int r = 0; r < 3; ++r) {
        const double ni = (ip[r] - im[r]) / (2.0 * h);
        const double nj = (jp[r] - jm[r]) / (2.0 * h);
        num += (ni - Ji(r, k)) * (ni - Ji(r, k)) + (nj - Jj(r, k)) * (nj - Jj(r, k));
        den += Ji(r, k) * Ji(r, k) + Jj(r, k) * Jj(r, k);
      }
    }
    EXPECT_LT(std::sqrt(num / den), 1e-6) << "trial " << trial;
  }
}

// --- test graphs ---

/// Truth: n nodes on a circle of radius r, heading along the tangent.
std::vector<Pose2d> ring_truth(int n, double r) {
  std::vector<Pose2d> t;
  for (int k = 0; k < n; ++k) {
    const double a = 2.0 * kPi * k / n;
    t.emplace_back(r * std::cos(a), r * std::sin(a), a + kPi / 2);
  }
  return t;
}

struct Built {
  std::unique_ptr<PoseGraph> g;
  std::vector<Pose2d> truth;
};

/// Ring with odometry k -> k+1 and loops (n-1 -> 1) and (n/2 -> 1) between free nodes (so
/// they shape the factor; node 0 is fixed). Measurements get
/// gaussian noise (sigma_m, sigma_rad); the initial estimate chains the odometry with an
/// extra yaw drift per edge.
Built make_ring(int n, double sigma_m, double sigma_rad, double drift_rad, std::uint32_t seed) {
  Built b;
  GraphConfig cfg;
  cfg.max_iterations = 100;
  b.g = std::make_unique<PoseGraph>(cfg, 3.0F);
  b.truth = ring_truth(n, 5.0);
  std::mt19937 rng(seed);
  std::normal_distribution<double> nm(0.0, sigma_m > 0.0 ? sigma_m : 1.0);
  std::normal_distribution<double> nr(0.0, sigma_rad > 0.0 ? sigma_rad : 1.0);
  auto noisy = [&](const Pose2d& z) {
    if (sigma_m <= 0.0) {
      return z;
    }
    return z * Pose2d(nm(rng), nm(rng), nr(rng));
  };
  const Info3 info = Info3::diagonal(0.05F, 0.01F);
  b.g->add_node(b.truth[0]);
  Pose2d X = b.truth[0];
  for (int k = 1; k < n; ++k) {
    const Pose2d z = noisy(b.truth[static_cast<std::size_t>(k - 1)].inverse() *
                           b.truth[static_cast<std::size_t>(k)]);
    X = X * z * Pose2d(0.0, 0.0, drift_rad);
    b.g->add_node(X);
    b.g->add_edge(k - 1, k, to_float(z), info, EdgeKind::kOdometry);
  }
  b.g->add_edge(
      n - 1, 1, to_float(noisy(b.truth.back().inverse() * b.truth[1])), info, EdgeKind::kLoop);
  b.g->add_edge(n / 2,
                1,
                to_float(noisy(b.truth[static_cast<std::size_t>(n / 2)].inverse() * b.truth[1])),
                info,
                EdgeKind::kLoop);
  return b;
}

TEST(PoseGraphSolve, ExactMeasurementsRecoverTheTruthFromADriftedStart) {
  Built b = make_ring(40, 0.0, 0.0, 0.02, 1);  // 0.8 rad of accumulated yaw drift
  double before = 0.0;
  for (int k = 0; k < 40; ++k) {
    before = std::max(before, pose_error(b.g->node(k), b.truth[static_cast<std::size_t>(k)]));
  }
  const SolveReport r = b.g->solve();
  EXPECT_EQ(r.status, SolveStatus::kConverged);
  EXPECT_GT(before, 3.0);
  for (int k = 0; k < 40; ++k) {
    // Measurements are stored as float: about 1e-6 m and rad of rounding per edge.
    EXPECT_LT(pose_error(b.g->node(k), b.truth[static_cast<std::size_t>(k)]), 1e-4) << k;
  }
  EXPECT_LT(r.final_cost, 1e-3);
  std::printf("[ ring 40   ] %d iterations, cost %.3g -> %.3g, factor %d doubles\n",
              r.iterations,
              r.initial_cost,
              r.final_cost,
              r.factor_entries);
}

/// Independent reference: dense Gauss-Newton with IRLS weights on the same edges.
std::vector<Pose2d> dense_reference(const PoseGraph& g, float cauchy_c, int iterations) {
  const int n = g.n_nodes();
  const int m = 3 * (n - 1);
  std::vector<Pose2d> X;
  for (int k = 0; k < n; ++k) {
    X.push_back(g.node(k));
  }
  const double c2 = static_cast<double>(cauchy_c) * cauchy_c;
  for (int it = 0; it < iterations; ++it) {
    std::vector<double> H(static_cast<std::size_t>(m * m), 0.0);
    std::vector<double> rhs(static_cast<std::size_t>(m), 0.0);
    for (int ei = 0; ei < g.n_edges(); ++ei) {
      const GraphEdge& e = g.edge(ei);
      Mat3d J[2];
      const Vec3T<double> r =
          edge_residual<double>(X[e.i], X[e.j], Pose2d(e.Z.x, e.Z.y, e.Z.yaw), &J[0], &J[1]);
      const Mat3d W = e.info.to_mat();
      double s = 0.0;
      for (int a = 0; a < 3; ++a) {
        for (int c = 0; c < 3; ++c) {
          s += r[a] * W(a, c) * r[c];
        }
      }
      const double w = e.kind == EdgeKind::kLoop ? 1.0 / (1.0 + s / c2) : 1.0;
      const int node[2] = {e.i, e.j};
      for (int p = 0; p < 2; ++p) {
        if (node[p] == 0) {
          continue;
        }
        for (int a = 0; a < 3; ++a) {
          double gsum = 0.0;
          for (int x = 0; x < 3; ++x) {
            for (int y = 0; y < 3; ++y) {
              gsum += J[p](x, a) * W(x, y) * r[y];
            }
          }
          rhs[static_cast<std::size_t>(3 * (node[p] - 1) + a)] -= w * gsum;
          for (int q = 0; q < 2; ++q) {
            if (node[q] == 0) {
              continue;
            }
            for (int c = 0; c < 3; ++c) {
              double hsum = 0.0;
              for (int x = 0; x < 3; ++x) {
                for (int y = 0; y < 3; ++y) {
                  hsum += J[p](x, a) * W(x, y) * J[q](y, c);
                }
              }
              H[static_cast<std::size_t>((3 * (node[p] - 1) + a) * m + 3 * (node[q] - 1) + c)] +=
                  w * hsum;
            }
          }
        }
      }
    }
    // Dense Cholesky and solve.
    for (int j = 0; j < m; ++j) {
      for (int k = 0; k < j; ++k) {
        H[static_cast<std::size_t>(j * m + j)] -=
            H[static_cast<std::size_t>(j * m + k)] * H[static_cast<std::size_t>(j * m + k)];
      }
      H[static_cast<std::size_t>(j * m + j)] = std::sqrt(H[static_cast<std::size_t>(j * m + j)]);
      for (int i = j + 1; i < m; ++i) {
        double s = H[static_cast<std::size_t>(i * m + j)];
        for (int k = 0; k < j; ++k) {
          s -= H[static_cast<std::size_t>(i * m + k)] * H[static_cast<std::size_t>(j * m + k)];
        }
        H[static_cast<std::size_t>(i * m + j)] = s / H[static_cast<std::size_t>(j * m + j)];
      }
    }
    for (int i = 0; i < m; ++i) {
      for (int k = 0; k < i; ++k) {
        rhs[static_cast<std::size_t>(i)] -=
            H[static_cast<std::size_t>(i * m + k)] * rhs[static_cast<std::size_t>(k)];
      }
      rhs[static_cast<std::size_t>(i)] /= H[static_cast<std::size_t>(i * m + i)];
    }
    for (int i = m - 1; i >= 0; --i) {
      for (int k = i + 1; k < m; ++k) {
        rhs[static_cast<std::size_t>(i)] -=
            H[static_cast<std::size_t>(k * m + i)] * rhs[static_cast<std::size_t>(k)];
      }
      rhs[static_cast<std::size_t>(i)] /= H[static_cast<std::size_t>(i * m + i)];
    }
    for (int k = 1; k < n; ++k) {
      Vec3T<double> d;
      for (int a = 0; a < 3; ++a) {
        d[a] = rhs[static_cast<std::size_t>(3 * (k - 1) + a)];
      }
      X[static_cast<std::size_t>(k)] = X[static_cast<std::size_t>(k)] * Pose2d::exp(d);
    }
  }
  return X;
}

TEST(PoseGraphSolve, SkylineSolveMatchesADenseReference) {
  Built b = make_ring(30, 0.03, 0.005, 0.01, 3);
  const std::vector<Pose2d> ref = dense_reference(*b.g, 3.0F, 30);
  const SolveReport r = b.g->solve();
  EXPECT_EQ(r.status, SolveStatus::kConverged);
  double worst = 0.0;
  for (int k = 0; k < 30; ++k) {
    worst = std::max(worst, pose_error(b.g->node(k), ref[static_cast<std::size_t>(k)]));
  }
  EXPECT_LT(worst, 1e-6);
}

TEST(PoseGraphSolve, SteppedSolveIsBitIdenticalToTheBatchSolve) {
  Built a = make_ring(25, 0.03, 0.005, 0.01, 5);
  Built b = make_ring(25, 0.03, 0.005, 0.01, 5);
  (void)a.g->solve();
  ASSERT_EQ(b.g->begin_solve(), SolveStatus::kRunning);
  int steps = 0;
  while (b.g->step() == SolveStatus::kRunning) {
    ++steps;
  }
  EXPECT_GT(steps, 0);
  for (int k = 0; k < 25; ++k) {
    EXPECT_EQ(a.g->node(k).x, b.g->node(k).x);
    EXPECT_EQ(a.g->node(k).y, b.g->node(k).y);
    EXPECT_EQ(a.g->node(k).yaw, b.g->node(k).yaw);
  }
}

TEST(PoseGraphSolve, CauchyKernelBoundsAFalseLoopAndItsChi2StandsOut) {
  // Noisy ring with correct loops, plus one false loop that claims node 30 sits 1.5 m from
  // where it is. Worst node error against the same graph without the false loop.
  Built clean = make_ring(40, 0.01, 0.002, 0.0, 9);
  (void)clean.g->solve();
  auto run = [&clean](EdgeKind false_kind, double& worst, double& false_chi2) {
    Built b = make_ring(40, 0.01, 0.002, 0.0, 9);
    const Pose2d wrong = b.truth[30].inverse() * b.truth[10] * Pose2d(1.5, 0.0, 0.1);
    const int fe =
        b.g->add_edge(30, 10, to_float(wrong), Info3::diagonal(0.05F, 0.01F), false_kind);
    (void)b.g->solve();
    worst = 0.0;
    for (int k = 0; k < 40; ++k) {
      worst = std::max(worst, pose_error(b.g->node(k), clean.g->node(k)));
    }
    false_chi2 = b.g->chi2(fe);
  };
  double worst_robust = 0.0;
  double chi2_robust = 0.0;
  double worst_l2 = 0.0;
  double chi2_l2 = 0.0;
  run(EdgeKind::kLoop, worst_robust, chi2_robust);
  run(EdgeKind::kOdometry, worst_l2, chi2_l2);
  std::printf("[ false loop] worst shift %.3f (Cauchy) vs %.3f (quadratic); chi2 %.0f vs %.0f\n",
              worst_robust,
              worst_l2,
              chi2_robust,
              chi2_l2);
  // Measured: 0.12 m against 0.40 m. The quadratic edge drags the ring until its own chi2 is
  // small (2): the false loop then cannot be told apart. Under the kernel it keeps a large
  // chi2 (486), which the backend's post-solve check uses to drop it.
  EXPECT_LT(worst_robust, 0.2);
  EXPECT_GT(worst_l2, 2.5 * worst_robust);
  EXPECT_LT(chi2_l2, static_cast<double>(LoopConfig{}.max_chi2));
  EXPECT_GT(chi2_robust, static_cast<double>(LoopConfig{}.max_chi2));
}

// --- ordering and memory bound ---

TEST(PoseGraphSolve, ReverseCuthillMcKeeNarrowsARevisitedRing) {
  // Two laps of a 60-node ring, every third node of the second lap closing a loop to the
  // same place of the first (what a revisit produces): in insertion order those rows span a
  // whole lap; reverse Cuthill-McKee interleaves the laps.
  const int m = 60;
  GraphConfig cfg;
  cfg.max_iterations = 100;
  auto g = std::make_unique<PoseGraph>(cfg, 3.0F);
  const std::vector<Pose2d> lap = ring_truth(m, 5.0);
  const Info3 info = Info3::diagonal(0.05F, 0.01F);
  g->add_node(lap[0]);
  Pose2d X = lap[0];
  for (int k = 1; k < 2 * m; ++k) {
    const Pose2d z =
        lap[static_cast<std::size_t>((k - 1) % m)].inverse() * lap[static_cast<std::size_t>(k % m)];
    X = X * z * Pose2d(0.0, 0.0, 0.002);
    g->add_node(X);
    g->add_edge(k - 1, k, to_float(z), info, EdgeKind::kOdometry);
    if (k > m && k % 3 == 0) {
      g->add_edge(k - m, k, Pose2(), info, EdgeKind::kLoop);
    }
  }
  const SolveReport r = g->solve();
  EXPECT_EQ(r.status, SolveStatus::kConverged);
  for (int k = 0; k < 2 * m; ++k) {
    EXPECT_LT(pose_error(g->node(k), lap[static_cast<std::size_t>(k % m)]), 1e-4) << k;
  }
  // Insertion order: block row k > m reaches back to k - m, 9 m + 6 entries.
  const int natural = (2 * m - 1) * 6 + 9 * (2 * m - 2) + (m / 3 - 1) * 9 * (m - 1);
  std::printf("[ two laps  ] factor envelope %d doubles (insertion order about %d)\n",
              r.factor_entries,
              natural);
  // Measured 4 818 against 11 865: the breadth-first front spans about four nodes.
  EXPECT_LT(r.factor_entries, natural / 2);
}

TEST(PoseGraphSolve, KeepsTheInsertionOrderWhenItIsNarrower) {
  // One ring, loops from the last and the middle node to node 1: two long rows in insertion
  // order, while a breadth-first order has a wide front around the whole ring.
  Built b = make_ring(120, 0.0, 0.0, 0.001, 1);
  const SolveReport r = b.g->solve();
  EXPECT_EQ(r.status, SolveStatus::kConverged);
  // Chain rows (each node reaches back one block: 15 entries) plus the two loop rows.
  const int natural = 6 + 118 * 15 + 9 * (119 - 1 - 1) + 9 * (60 - 1 - 1);
  std::printf(
      "[ ring 120  ] factor envelope %d doubles (insertion order %d)\n", r.factor_entries, natural);
  EXPECT_LE(r.factor_entries, natural);
}

TEST(PoseGraphSolve, RefusesAGraphWhoseFactorDoesNotFit) {
  GraphConfig cfg;
  auto g = std::make_unique<PoseGraph>(cfg, 3.0F);
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> pick(0, kMaxGraphNodes - 1);
  for (int k = 0; k < kMaxGraphNodes; ++k) {
    ASSERT_EQ(g->add_node(Pose2d(k, 0.0, 0.0)), k);
  }
  EXPECT_EQ(g->add_node(Pose2d()), -1);  // full
  const Info3 info = Info3::diagonal(0.1F, 0.01F);
  for (int k = 1; k < kMaxGraphNodes; ++k) {
    ASSERT_GE(g->add_edge(k - 1, k, Pose2(1.0F, 0.0F, 0.0F), info, EdgeKind::kOdometry), 0);
  }
  while (g->n_edges() < kMaxGraphEdges) {
    const int i = pick(rng);
    const int j = pick(rng);
    if (i != j) {
      ASSERT_GE(
          g->add_edge(i, j, Pose2(static_cast<float>(j - i), 0.0F, 0.0F), info, EdgeKind::kLoop),
          0);
    }
  }
  EXPECT_EQ(g->add_edge(0, 1, Pose2(), info, EdgeKind::kLoop), -1);  // full
  EXPECT_EQ(g->begin_solve(), SolveStatus::kTooLarge);
  EXPECT_EQ(g->step(), SolveStatus::kTooLarge);
}

TEST(PoseGraphSolve, InactiveEdgesAreIgnored) {
  Built b = make_ring(20, 0.0, 0.0, 0.0, 1);
  const int fe = b.g->add_edge(
      15, 5, Pose2(3.0F, 0.0F, 0.0F), Info3::diagonal(0.05F, 0.01F), EdgeKind::kOdometry);
  b.g->set_edge_active(fe, false);
  EXPECT_LT(b.g->cost(), 1e-6);
  (void)b.g->solve();
  for (int k = 0; k < 20; ++k) {
    EXPECT_LT(pose_error(b.g->node(k), b.truth[static_cast<std::size_t>(k)]), 1e-4);
  }
}

}  // namespace
}  // namespace mss2d
