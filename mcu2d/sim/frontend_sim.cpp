#include "frontend_sim.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

#include "mss2d/merger.hpp"

namespace mss2d {
namespace sim {

namespace {

constexpr double kNsPerS = 1e9;

std::int64_t to_ns(double s) {
  return static_cast<std::int64_t>(std::llround(s * kNsPerS));
}

double to_s(Timestamp t) {
  return static_cast<double>(t.ns) / kNsPerS;
}

Pose2d to_double(const Pose2& p) {
  return Pose2d(p.x, p.y, p.yaw);
}

}  // namespace

// --- Trajectory ---

void Trajectory::add(double duration_s, double v, double w) {
  Segment s{t_end_, duration_s, v, w, end_};
  segs_.push_back(s);
  end_ = advance(s, duration_s);
  t_end_ += duration_s;
}

void Trajectory::arc(double angle_rad, double radius_m) {
  const double w = (angle_rad >= 0.0 ? 1.0 : -1.0) * speed_ / radius_m;
  add(std::abs(angle_rad) * radius_m / speed_, speed_, w);
}

void Trajectory::spin(double angle_rad, double rate_rad_s) {
  add(std::abs(angle_rad) / rate_rad_s, 0.0, (angle_rad >= 0.0 ? 1.0 : -1.0) * rate_rad_s);
}

Pose2d Trajectory::advance(const Segment& s, double tau_s) {
  Vec3T<double> d;
  d[0] = s.v_m_s * tau_s;
  d[2] = s.w_rad_s * tau_s;
  return s.start * Pose2d::exp(d);
}

Trajectory::State Trajectory::at(double t_s) const {
  State st;
  if (segs_.empty() || t_s >= t_end_) {
    st.T_w_b = end_;
    return st;
  }
  for (const Segment& s : segs_) {
    if (t_s < s.t0_s + s.duration_s) {
      const double tau = t_s < s.t0_s ? 0.0 : t_s - s.t0_s;
      st.T_w_b = advance(s, tau);
      st.v_m_s = s.v_m_s;
      st.w_rad_s = s.w_rad_s;
      return st;
    }
  }
  st.T_w_b = end_;
  return st;
}

// --- SyntheticLidar ---

SyntheticLidar::SyntheticLidar(const World& world,
                               const Trajectory& traj,
                               const LidarModel& model,
                               std::uint32_t seed)
    : world_(world),
      traj_(traj),
      model_(model),
      period_ns_(to_ns(1.0 / model.rate_hz)),
      seed_(seed) {}

bool SyntheticLidar::next_scan(Scan2d& out) {
  const std::int64_t beams = model_.beams;
  const std::int64_t last_ns = next_start_ns_ + (beams - 1) * period_ns_ / beams;
  if (last_ns > now_.ns || static_cast<double>(last_ns) / kNsPerS > traj_.duration_s()) {
    return false;
  }
  std::mt19937 rng(seed_ * 7919U + static_cast<std::uint32_t>(next_start_ns_ / period_ns_));
  std::normal_distribution<double> noise(0.0, model_.sigma_m > 0.0 ? model_.sigma_m : 1.0);
  out.t_start = Timestamp{next_start_ns_};
  out.count = static_cast<std::uint32_t>(beams);
  for (std::int64_t i = 0; i < beams; ++i) {
    const std::int64_t off = i * period_ns_ / beams;
    const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(beams);
    const Pose2d T_w_l =
        traj_.at(static_cast<double>(next_start_ns_ + off) / kNsPerS).T_w_b * model_.T_b_l;
    double r = cast(world_, T_w_l.x, T_w_l.y, T_w_l.yaw + a, model_.max_range_m);
    const double n = noise(rng);
    if (r > 0.0 && model_.sigma_m > 0.0) {
      r += n;
    }
    ScanPoint2d& p = out.points[static_cast<std::size_t>(i)];
    p.range_m = static_cast<float>(r);
    p.angle_rad = static_cast<float>(a);
    p.t_offset_ns = static_cast<std::int32_t>(off);
  }
  next_start_ns_ += period_ns_;
  return true;
}

// --- scenarios ---

Scenario loop_scenario() {
  Scenario sc;
  // Walls off the 5 cm cell lattice on purpose (docs/scan_matching.md section 5).
  sc.world.add_box(0.013, -0.017, 14.03, 10.04, 0.0);  // outer walls
  sc.world.add_box(0.021, 0.008, 9.02, 4.97, 0.0);     // inner block
  sc.world.add_box(1.0, -4.7, 0.3, 0.3, 0.0);          // pillars and boxes along the walls
  sc.world.add_box(-2.5, 4.68, 0.3, 0.3, 0.0);
  sc.world.add_box(6.75, 1.0, 0.4, 0.6, 0.0);
  sc.world.add_box(-6.74, -1.0, 0.4, 0.7, 0.3);
  sc.world.add_box(3.0, 4.5, 0.5, 0.4, 0.4);
  sc.world.add_box(-4.0, -4.75, 0.6, 0.3, 0.0);
  const double r = 0.75;
  const double q = kPi / 2.0;
  sc.traj = Trajectory(Pose2d(-3.0, -3.75, 0.0), 0.5);
  sc.traj.wait(1.0);
  sc.traj.straight(8.0);
  sc.traj.arc(q, r);
  sc.traj.straight(6.0);
  sc.traj.arc(q, r);
  sc.traj.spin(2.0 * kPi, 1.0);
  sc.traj.straight(10.0);
  sc.traj.arc(q, r);
  sc.traj.straight(6.0);
  sc.traj.arc(q, r);
  sc.traj.straight(2.0);
  sc.traj.wait(0.5);
  return sc;
}

Scenario corridor_scenario() {
  Scenario sc;
  sc.world = corridor_world();
  sc.traj = Trajectory(Pose2d(0.0, 0.0, 0.0), 0.5);
  sc.traj.wait(0.5);
  sc.traj.straight(4.0);
  sc.wheel_scale = 1.0;
  return sc;
}

// --- run ---

namespace {

struct Collector final : PoseSink {
  const Trajectory* traj = nullptr;
  RunResult* res = nullptr;
  void on_pose(const ScanPose& p) override {
    res->est.push_back(p);
    res->truth.push_back(traj->at(to_s(p.t)).T_w_b);
  }
};

/// Forwards to the front end and times on_scan.
struct TimedSink final : Sink {
  FrontEnd* fe = nullptr;
  std::vector<double>* ms = nullptr;
  void on_imu(const ImuSample2d& s) override { fe->on_imu(s); }
  void on_wheel(const WheelSample& s) override { fe->on_wheel(s); }
  void on_scan(const Scan2d& s) override {
    const auto t0 = std::chrono::steady_clock::now();
    fe->on_scan(s);
    const auto t1 = std::chrono::steady_clock::now();
    ms->push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
};

}  // namespace

RunResult run_frontend(const Scenario& sc, const Config& cfg) {
  RunResult res;
  Collector col;
  col.traj = &sc.traj;
  col.res = &res;
  auto merger = std::make_unique<Merger>(cfg);
  auto fe = std::make_unique<FrontEnd>(cfg, &col);
  TimedSink timed;
  timed.fe = fe.get();
  timed.ms = &res.latency_ms;
  auto scan = std::make_unique<Scan2d>();
  SyntheticLidar lidar(sc.world, sc.traj, sc.lidar, sc.seed);

  std::mt19937 gyro_rng(sc.seed * 31U + 1U);
  std::mt19937 wheel_rng(sc.seed * 31U + 2U);
  std::normal_distribution<double> gyro_noise(0.0, sc.gyro_sigma_rad_s);
  std::normal_distribution<double> wheel_noise(0.0, sc.wheel_sigma_m_s);
  const std::int64_t imu_dt = to_ns(1.0 / sc.imu_rate_hz);
  const std::int64_t wheel_dt = to_ns(1.0 / sc.wheel_rate_hz);
  const std::int64_t t_end = to_ns(sc.traj.duration_s());
  std::int64_t next_wheel = 0;
  for (std::int64_t t = 0; t <= t_end + imu_dt; t += imu_dt) {
    while (sc.wheel && next_wheel <= t) {
      const Trajectory::State st = sc.traj.at(static_cast<double>(next_wheel) / kNsPerS);
      WheelSample w;
      w.t = Timestamp{next_wheel};
      w.v_m_s = static_cast<float>(st.v_m_s * sc.wheel_scale +
                                   (st.v_m_s != 0.0 ? wheel_noise(wheel_rng) : 0.0));
      (void)merger->push_wheel(w);
      next_wheel += wheel_dt;
    }
    lidar.set_time(Timestamp{t});
    while (lidar.next_scan(*scan)) {
      (void)merger->push_scan(*scan);
    }
    ImuSample2d s;
    s.t = Timestamp{t};
    s.gyro_z_rad_s = static_cast<float>(sc.traj.at(static_cast<double>(t) / kNsPerS).w_rad_s +
                                        sc.gyro_bias_rad_s + gyro_noise(gyro_rng));
    (void)merger->push_imu(s);
    merger->run_once(timed);
  }
  merger->flush(timed);
  res.stats = fe->stats();
  res.gyro_bias_rad_s = fe->gyro_bias();
  return res;
}

// --- metrics and output ---

std::vector<Pose2d> poses_of(const std::vector<ScanPose>& est) {
  std::vector<Pose2d> out;
  out.reserve(est.size());
  for (const ScanPose& p : est) {
    out.push_back(to_double(p.T_w_b));
  }
  return out;
}

Metrics evaluate(const std::vector<Pose2d>& est, const std::vector<Pose2d>& truth, int delta) {
  Metrics m;
  const std::size_t n = est.size() < truth.size() ? est.size() : truth.size();
  if (n == 0) {
    return m;
  }
  // T_true_est: maps the estimate's map frame onto the truth frame through the first pose.
  const Pose2d align = truth[0] * est[0].inverse();
  double se = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Pose2d e = align * est[i];
    const double dx = e.x - truth[i].x;
    const double dy = e.y - truth[i].y;
    const double d2 = dx * dx + dy * dy;
    se += d2;
    m.ate_max_m = std::sqrt(d2) > m.ate_max_m ? std::sqrt(d2) : m.ate_max_m;
  }
  m.ate_rmse_m = std::sqrt(se / static_cast<double>(n));
  double st = 0.0;
  double sr = 0.0;
  std::size_t k = 0;
  for (std::size_t i = 0; i + static_cast<std::size_t>(delta) < n; ++i) {
    const std::size_t j = i + static_cast<std::size_t>(delta);
    const Pose2d err = (truth[i].inverse() * truth[j]).inverse() * (est[i].inverse() * est[j]);
    st += err.x * err.x + err.y * err.y;
    sr += err.yaw * err.yaw;
    ++k;
  }
  if (k > 0) {
    m.rpe_trans_rmse_m = std::sqrt(st / static_cast<double>(k));
    m.rpe_rot_rmse_deg = std::sqrt(sr / static_cast<double>(k)) * 180.0 / kPi;
  }
  return m;
}

bool write_tum(const char* path, const std::vector<ScanPose>& times, const std::vector<Pose2d>& T) {
  std::FILE* f = std::fopen(path, "w");
  if (f == nullptr) {
    return false;
  }
  std::fprintf(f, "# t x y z qx qy qz qw\n");
  bool ok = true;
  for (std::size_t i = 0; i < times.size() && i < T.size(); ++i) {
    // Rotation about z by yaw: Hamilton (w, x, y, z) = (cos(yaw/2), 0, 0, sin(yaw/2)),
    // written in TUM order (x, y, z, w).
    const double h = 0.5 * T[i].yaw;
    ok = ok && std::fprintf(f,
                            "%.9f %.6f %.6f 0 0 0 %.9f %.9f\n",
                            to_s(times[i].t),
                            T[i].x,
                            T[i].y,
                            std::sin(h),
                            std::cos(h)) > 0;
  }
  return std::fclose(f) == 0 && ok;
}

}  // namespace sim
}  // namespace mss2d
