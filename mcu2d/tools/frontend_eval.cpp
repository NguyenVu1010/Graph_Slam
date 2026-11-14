// Runs the front end on the synthetic loop (sim/frontend_sim.hpp) and prints ATE, RPE, per-scan
// latency and the static footprint. With --out DIR it writes DIR/estimate.tum and DIR/truth.tum
// (TUM text, readable by eval/); nothing is written otherwise.
//
//   mss2d_frontend_eval [--out DIR] [--no-deskew] [--no-wheel] [--corridor | --revisit]
//                       [--graph] [--seed N] [--eig THRESHOLD]
//
// --corridor runs the straight drive in the featureless corridor instead of the loop,
// --revisit the lap with drift that comes back to its start; --eig overrides
// matcher.degeneracy_eig_threshold. --graph also runs the pose-graph backend (a second run):
// it prints the optimised trajectory's ATE and RPE, loop and solve statistics and costs, and
// with --out also writes DIR/optimized.tum.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "frontend_sim.hpp"
#include "mss2d/merger.hpp"

namespace {

constexpr int kRpeDeltaScans = 10;
constexpr int kSolveRepeats = 20;

double percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const std::size_t i = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
  return v[i];
}

void print_metrics(const char* name, const mss2d::sim::Metrics& m) {
  std::printf("%s ATE rmse %.4f m  max %.4f m   RPE(%d scans) %.4f m  %.3f deg\n",
              name,
              m.ate_rmse_m,
              m.ate_max_m,
              kRpeDeltaScans,
              m.rpe_trans_rmse_m,
              m.rpe_rot_rmse_deg);
}

/// Backend statistics, costs and a timed full re-solve of the final graph.
void report_graph(const mss2d::sim::RunResult& g) {
  using namespace mss2d;
  const BackendStats& bs = g.backend->stats();
  std::printf(
      "graph: nodes %u (dropped %u)  candidates %u  loops %u  rejected: score %u  degenerate %u"
      "  window %u  chi2 %u  capacity %u  solves %u\n",
      bs.n_nodes,
      bs.n_dropped_nodes,
      bs.n_candidates,
      bs.n_loops,
      bs.n_rejected_score,
      bs.n_rejected_degenerate,
      bs.n_rejected_window,
      bs.n_rejected_chi2,
      bs.n_rejected_capacity,
      bs.n_solves);
  print_metrics("optimized", sim::evaluate(g.optimized, g.truth, kRpeDeltaScans));
  print_metrics("online   ", sim::evaluate(sim::poses_of(g.est), g.truth, kRpeDeltaScans));
  std::printf(
      "backend per node: verify median %.3f ms  max %.3f ms; with a solve median %.3f ms  "
      "max %.3f ms (%zu solves)\n",
      percentile(g.node_ms, 0.5),
      percentile(g.node_ms, 1.0),
      percentile(g.solve_ms, 0.5),
      percentile(g.solve_ms, 1.0),
      g.solve_ms.size());
  const SolveReport& lr = g.backend->last_solve();
  std::printf("last online solve: %d iterations, cost %.3g -> %.3g, envelope %d doubles\n",
              lr.iterations,
              lr.initial_cost,
              lr.final_cost,
              lr.factor_entries);

  // Full solve of the final graph restarted from the odometry chain (worst-case start).
  auto pg = std::make_unique<PoseGraph>(g.backend->graph());
  std::vector<double> ms;
  SolveReport fr;
  for (int rep = 0; rep < kSolveRepeats; ++rep) {
    for (int e = 0; e < pg->n_edges(); ++e) {
      const GraphEdge& ed = pg->edge(e);
      if (ed.kind == EdgeKind::kOdometry) {
        pg->set_node(ed.j, pg->node(ed.i) * Pose2d(ed.Z.x, ed.Z.y, ed.Z.yaw));
      }
    }
    const auto t0 = std::chrono::steady_clock::now();
    fr = pg->solve();
    const auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::printf(
      "full solve from odometry: %d nodes, %d edges, %d iterations, median %.3f ms  max %.3f "
      "ms, envelope %d doubles\n",
      pg->n_nodes(),
      pg->n_edges(),
      fr.iterations,
      percentile(ms, 0.5),
      percentile(ms, 1.0),
      fr.factor_entries);
  std::printf("sizeof Backend %zu  PoseGraph %zu  LoopMatcher %zu  CompactSubmap %zu\n",
              sizeof(Backend),
              sizeof(PoseGraph),
              sizeof(LoopMatcher),
              sizeof(CompactSubmap));
}

}  // namespace

int main(int argc, char** argv) {
  using namespace mss2d;
  const char* out_dir = nullptr;
  Config cfg;
  bool corridor = false;
  bool revisit = false;
  bool graph = false;
  bool wheel = true;
  std::uint32_t seed = 1;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--no-deskew") == 0) {
      cfg.motion.deskew = false;
    } else if (std::strcmp(argv[i], "--no-wheel") == 0) {
      wheel = false;
    } else if (std::strcmp(argv[i], "--corridor") == 0) {
      corridor = true;
    } else if (std::strcmp(argv[i], "--revisit") == 0) {
      revisit = true;
    } else if (std::strcmp(argv[i], "--graph") == 0) {
      graph = true;
    } else if (std::strcmp(argv[i], "--eig") == 0 && i + 1 < argc) {
      cfg.matcher.degeneracy_eig_threshold = std::strtof(argv[++i], nullptr);
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else {
      std::fprintf(stderr,
                   "usage: %s [--out DIR] [--no-deskew] [--no-wheel] [--corridor | --revisit] "
                   "[--graph] [--seed N] [--eig THRESHOLD]\n",
                   argv[0]);
      return 2;
    }
  }
  sim::Scenario sc = corridor  ? sim::corridor_scenario()
                     : revisit ? sim::revisit_scenario()
                               : sim::loop_scenario();
  sc.wheel = wheel;
  sc.seed = seed;
  cfg.lidar.x_m = static_cast<float>(sc.lidar.T_b_l.x);
  cfg.lidar.y_m = static_cast<float>(sc.lidar.T_b_l.y);
  cfg.lidar.yaw_rad = static_cast<float>(sc.lidar.T_b_l.yaw);
  cfg.scan_rate_hz = static_cast<float>(sc.lidar.rate_hz);
  if (const Status s = cfg.validate(); !s.is_ok()) {
    std::fprintf(stderr, "config: %s\n", s.reason());
    return 1;
  }

  const sim::RunResult r = sim::run_frontend(sc, cfg);
  const std::vector<sim::Pose2d> est = sim::poses_of(r.est);
  const sim::Metrics m = sim::evaluate(est, r.truth, kRpeDeltaScans);
  std::printf("scans %zu  ok %u  degenerate %u  predicted %u  not converged %u\n",
              r.est.size(),
              r.stats.n_ok,
              r.stats.n_degenerate,
              r.stats.n_predicted,
              r.stats.n_not_converged);
  std::printf("submaps started %u  finished %u  recycled %u  gyro bias %.4f rad/s (true %.4f)\n",
              r.stats.n_submaps_started,
              r.stats.n_submaps_finished,
              r.stats.n_submaps_recycled,
              static_cast<double>(r.gyro_bias_rad_s),
              sc.gyro_bias_rad_s);
  print_metrics("front end", m);
  std::printf("on_scan latency median %.3f ms  p95 %.3f ms  max %.3f ms\n",
              percentile(r.latency_ms, 0.5),
              percentile(r.latency_ms, 0.95),
              percentile(r.latency_ms, 1.0));
  std::printf(
      "sizeof FrontEnd %zu  Merger %zu  OccupancyGrid %zu  DistanceField %zu  "
      "ScanMatcher %zu  PointCloud2d %zu  MotionModel %zu  DeskewTable %zu  Scan2d %zu\n",
      sizeof(FrontEnd),
      sizeof(Merger),
      sizeof(OccupancyGrid),
      sizeof(DistanceField),
      sizeof(ScanMatcher),
      sizeof(PointCloud2d),
      sizeof(MotionModel),
      sizeof(DeskewTable),
      sizeof(Scan2d));

  sim::RunResult g;
  if (graph) {
    g = sim::run_frontend(sc, cfg, true);
    report_graph(g);
  }

  if (out_dir != nullptr) {
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    const std::string base(out_dir);
    if (ec || !sim::write_tum((base + "/estimate.tum").c_str(), r.est, est) ||
        !sim::write_tum((base + "/truth.tum").c_str(), r.est, r.truth) ||
        (graph && !sim::write_tum((base + "/optimized.tum").c_str(), g.est, g.optimized))) {
      std::fprintf(stderr, "cannot write TUM files under %s\n", out_dir);
      return 1;
    }
    std::printf(
        "wrote estimate.tum, truth.tum%s under %s\n", graph ? ", optimized.tum" : "", out_dir);
  }
  return 0;
}
