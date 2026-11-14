// Runs the front end on the synthetic loop (sim/frontend_sim.hpp) and prints ATE, RPE, per-scan
// latency and the static footprint. With --out DIR it writes DIR/estimate.tum and DIR/truth.tum
// (TUM text, readable by eval/); nothing is written otherwise.
//
//   mss2d_frontend_eval [--out DIR] [--no-deskew] [--no-wheel] [--corridor] [--seed N]
//                       [--eig THRESHOLD]
//
// --corridor runs the straight drive in the featureless corridor instead of the loop; --eig
// overrides matcher.degeneracy_eig_threshold.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "frontend_sim.hpp"
#include "mss2d/merger.hpp"

namespace {

constexpr int kRpeDeltaScans = 10;

double percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const std::size_t i = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
  return v[i];
}

}  // namespace

int main(int argc, char** argv) {
  using namespace mss2d;
  const char* out_dir = nullptr;
  Config cfg;
  bool corridor = false;
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
    } else if (std::strcmp(argv[i], "--eig") == 0 && i + 1 < argc) {
      cfg.matcher.degeneracy_eig_threshold = std::strtof(argv[++i], nullptr);
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else {
      std::fprintf(stderr,
                   "usage: %s [--out DIR] [--no-deskew] [--no-wheel] [--corridor] [--seed N] "
                   "[--eig THRESHOLD]\n",
                   argv[0]);
      return 2;
    }
  }
  sim::Scenario sc = corridor ? sim::corridor_scenario() : sim::loop_scenario();
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
  std::printf("ATE rmse %.4f m  max %.4f m   RPE(%d scans) %.4f m  %.3f deg\n",
              m.ate_rmse_m,
              m.ate_max_m,
              kRpeDeltaScans,
              m.rpe_trans_rmse_m,
              m.rpe_rot_rmse_deg);
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

  if (out_dir != nullptr) {
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    const std::string base(out_dir);
    if (ec || !sim::write_tum((base + "/estimate.tum").c_str(), r.est, est) ||
        !sim::write_tum((base + "/truth.tum").c_str(), r.est, r.truth)) {
      std::fprintf(stderr, "cannot write TUM files under %s\n", out_dir);
      return 1;
    }
    std::printf("wrote %s/estimate.tum and %s/truth.tum\n", out_dir, out_dir);
  }
  return 0;
}
