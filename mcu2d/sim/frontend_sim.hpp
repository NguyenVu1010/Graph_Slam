#pragma once

// Closed-loop simulation of the front end on the host (tests and tools/frontend_eval): a known
// trajectory in a line-segment world, an RPLIDAR-like rotating scanner whose beams are cast
// from the pose at their own sample time, a biased noisy gyro and a noisy wheel encoder, all
// with fixed seeds. Heap and std::vector are fine here; this never runs on the MCU.

#include <cstdint>
#include <vector>

#include "mss2d/config.hpp"
#include "mss2d/frontend.hpp"
#include "mss2d/lidar_source.hpp"
#include "sim_world.hpp"

namespace mss2d {
namespace sim {

using Pose2d = Pose2T<double>;

/// Piecewise constant-twist trajectory of the body: on each segment the body moves with
/// forward speed v and yaw rate w, so the pose is start * Exp([v tau, 0, w tau]) exactly.
class Trajectory {
 public:
  struct State {
    Pose2d T_w_b;
    double v_m_s = 0.0;
    double w_rad_s = 0.0;
  };

  Trajectory(const Pose2d& start, double speed_m_s) : end_(start), speed_(speed_m_s) {}

  void wait(double duration_s) { add(duration_s, 0.0, 0.0); }
  void straight(double length_m) { add(length_m / speed_, speed_, 0.0); }
  /// Left turn for angle > 0, at the trajectory speed.
  void arc(double angle_rad, double radius_m);
  /// Turn in place, counter-clockwise for angle > 0.
  void spin(double angle_rad, double rate_rad_s);

  double duration_s() const { return t_end_; }
  State at(double t_s) const;

 private:
  struct Segment {
    double t0_s;
    double duration_s;
    double v_m_s;
    double w_rad_s;
    Pose2d start;
  };
  void add(double duration_s, double v, double w);
  static Pose2d advance(const Segment& s, double tau_s);

  std::vector<Segment> segs_;
  Pose2d end_;
  double speed_;
  double t_end_ = 0.0;
};

/// Rotating single-beam scanner: turn k starts at k / rate_hz, beam i is sampled at
/// t_start + i / (beams * rate_hz) at angle 2 pi i / beams in l, from the pose at that time.
struct LidarModel {
  int beams = 720;
  double rate_hz = 5.5;
  double max_range_m = 12.0;
  double sigma_m = 0.01;  ///< gaussian range noise
  Pose2d T_b_l{0.10, 0.0, 0.0};
};

/// LidarSource over a world and a trajectory: a turn is ready once the simulated clock has
/// passed its last beam.
class SyntheticLidar final : public LidarSource {
 public:
  SyntheticLidar(const World& world,
                 const Trajectory& traj,
                 const LidarModel& model,
                 std::uint32_t seed);

  void set_time(Timestamp now) { now_ = now; }
  bool next_scan(Scan2d& out) override;

 private:
  const World& world_;
  const Trajectory& traj_;
  LidarModel model_;
  std::int64_t period_ns_;
  std::int64_t next_start_ns_ = 0;
  Timestamp now_{};
  std::uint32_t seed_;
};

struct Scenario {
  World world;
  Trajectory traj{Pose2d(), 0.5};
  LidarModel lidar;
  double imu_rate_hz = 200.0;
  double gyro_bias_rad_s = 0.01;
  double gyro_sigma_rad_s = 0.005;  ///< white noise per sample
  bool wheel = true;
  double wheel_rate_hz = 50.0;
  double wheel_scale = 1.01;  ///< multiplicative speed error
  double wheel_sigma_m_s = 0.02;
  std::uint32_t seed = 1;
};

/// Ring corridor around an inner block (14 x 10 m outside, 2.5 m wide) with a few pillars and
/// boxes; one counter-clockwise loop on the corridor centre line at 0.5 m/s with 0.75 m
/// radius turns (0.67 rad/s, about 7 deg of yaw per 5.5 Hz turn) and one full turn in place at
/// 1 rad/s (10 deg per LiDAR turn), 1 s standing still first.
Scenario loop_scenario();

/// Straight drive along the featureless corridor of corridor_world() (degenerate along x).
Scenario corridor_scenario();

struct RunResult {
  std::vector<ScanPose> est;
  std::vector<Pose2d> truth;       ///< body pose at each est[i].t
  std::vector<double> latency_ms;  ///< FrontEnd::on_scan wall time per scan
  FrontEndStats stats;
  float gyro_bias_rad_s = 0.0F;
};

/// Feeds the sensors through a Merger into a FrontEnd in timestamp order.
RunResult run_frontend(const Scenario& sc, const Config& cfg);

struct Metrics {
  double ate_rmse_m = 0.0;  ///< after aligning the first poses (SE(2), no scale)
  double ate_max_m = 0.0;
  double rpe_trans_rmse_m = 0.0;  ///< over pairs `delta` scans apart
  double rpe_rot_rmse_deg = 0.0;
};

Metrics evaluate(const std::vector<Pose2d>& est, const std::vector<Pose2d>& truth, int delta);

std::vector<Pose2d> poses_of(const std::vector<ScanPose>& est);

/// TUM text, `t x y z qx qy qz qw`, t in s, z = 0, yaw as the quaternion about z. False on
/// I/O error.
bool write_tum(const char* path, const std::vector<ScanPose>& times, const std::vector<Pose2d>& T);

}  // namespace sim
}  // namespace mss2d
