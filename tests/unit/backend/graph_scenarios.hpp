#pragma once

// Test data for the backend factor and graph tests: random helpers, a wheel-odometry delta,
// an IMU keyframe chain and the B-02 square loop. Fixed seeds only.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "mss/common/lie.hpp"
#include "mss/common/types.hpp"
#include "mss/imu/config.hpp"
#include "mss/imu/preintegrator.hpp"
#include "mss/odom/config.hpp"
#include "mss/odom/wheel_integrator.hpp"

#include "imu_test_utils.hpp"

namespace mss::backend::test {

using common::Mat6;
using common::NavState;
using common::Pose;
using common::Vec3;
using common::Vec6;

inline Vec3 random_vec3(std::mt19937& rng, double sigma) {
  std::normal_distribution<double> n(0.0, sigma);
  const double x = n(rng);
  const double y = n(rng);
  const double z = n(rng);
  return Vec3(x, y, z);
}

inline Vec6 random_vec6(std::mt19937& rng, double sigma) {
  Vec6 v;
  v << random_vec3(rng, sigma), random_vec3(rng, sigma);
  return v;
}

inline Pose random_pose(std::mt19937& rng) {
  Pose T;
  T.R = common::so3_exp(random_vec3(rng, 1.0));
  T.p = random_vec3(rng, 5.0);
  return T;
}

inline Pose pose_from(const Vec3& phi, const Vec3& p) {
  Pose T;
  T.R = common::so3_exp(phi);
  T.p = p;
  return T;
}

// Wheel-odometry delta: 50 samples at 50 Hz, straight for 25 then turning, through
// odom::WheelIntegrator with the given T_b_o. Its covariance has rank 3 (in-plane in o).
inline common::OdomDelta wheel_delta(const Pose& T_b_o, std::int64_t t0_ns = 1'000'000'000) {
  odom::WheelOdomConfig cfg;
  cfg.enabled = true;
  cfg.speed_sigma_mps = 0.05;
  cfg.yaw_rate_sigma_radps = 0.02;
  odom::WheelIntegrator wi(cfg, T_b_o);
  constexpr std::int64_t kDtNs = 20'000'000;
  for (int k = 0; k < 50; ++k) {
    common::WheelSample s;
    s.t = common::Timestamp{t0_ns + k * kDtNs};
    s.v_mps = 1.5;
    s.omega_radps = k < 25 ? 0.0 : 0.4;
    static_cast<void>(wi.integrate(s));
  }
  return wi.take_delta();
}

// IMU keyframe chain: `n_kf` keyframes, deltas from imu::Preintegrator over
// random_imu_sequence(seed, ...) split every `steps` samples with the boundary sample
// re-fed; truth by chaining predict() from x_0 with a constant bias.
struct ImuChain {
  std::vector<NavState> truth;
  std::vector<common::PreintegratedDelta> deltas;  // deltas[k]: keyframe k -> k + 1
};

inline ImuChain imu_chain(std::uint32_t seed, int n_kf, int steps) {
  const imu::ImuConfig cfg = imu::test::default_imu_config();
  const std::vector<common::ImuSample> seq =
      imu::test::random_imu_sequence(seed, (n_kf - 1) * steps + 1);
  ImuChain c;
  NavState x;
  x.t = seq[0].t;
  x.T_w_b = pose_from(Vec3(0.1, -0.2, 0.3), Vec3(1.0, 2.0, 0.5));
  x.v_w_b = Vec3(1.0, 0.5, -0.1);
  x.bias.ba = Vec3(0.02, -0.01, 0.03);
  x.bias.bg = Vec3(0.001, -0.002, 0.0015);
  c.truth.push_back(x);
  imu::Preintegrator pre(cfg);
  for (int k = 0; k + 1 < n_kf; ++k) {
    pre.reset(common::ImuBias{});  // linearised at zero bias, so the correction is exercised
    for (int s = k * steps; s <= (k + 1) * steps; ++s) {
      static_cast<void>(pre.integrate(seq[static_cast<std::size_t>(s)]));
    }
    c.deltas.push_back(pre.delta());
    x = pre.predict(x, Vec3(0.0, 0.0, -cfg.gravity_mps2));
    c.truth.push_back(x);
  }
  return c;
}

// B-02 square loop: 5 keyframes, t_k = 1 s + k s, corners of a 10 m square, yaw +90 deg
// per edge, keyframe 4 = keyframe 0 pose. Edges 0-1, 1-2, 2-3, 3-4 and the loop (0, 4).
struct Scenario {
  std::vector<NavState> truth;
  std::vector<NavState> guess;
  std::vector<std::pair<int, int>> edges;  // the last one is the loop (0, 4)
  std::vector<Pose> edge_meas;
  common::PoseCovariance edge_cov = common::PoseCovariance::Identity();
};

inline Scenario square_loop(bool noisy, std::uint32_t seed) {
  constexpr double kSideM = 10.0;
  constexpr double kHalfPi = 1.5707963267948966;
  constexpr double kPosSigmaM = 0.02;
  constexpr double kRotSigmaRad = 0.002;
  Scenario s;
  s.edge_cov.setZero();
  s.edge_cov.diagonal() << kPosSigmaM * kPosSigmaM, kPosSigmaM * kPosSigmaM,
      kPosSigmaM * kPosSigmaM, kRotSigmaRad * kRotSigmaRad, kRotSigmaRad * kRotSigmaRad,
      kRotSigmaRad * kRotSigmaRad;
  const Vec3 corners[4] = {Vec3(0.0, 0.0, 0.0),
                           Vec3(kSideM, 0.0, 0.0),
                           Vec3(kSideM, kSideM, 0.0),
                           Vec3(0.0, kSideM, 0.0)};
  for (int k = 0; k < 5; ++k) {
    NavState x;
    x.t = common::Timestamp{(1 + k) * 1'000'000'000LL};
    x.T_w_b = pose_from(Vec3(0.0, 0.0, kHalfPi * (k % 4)), corners[k % 4]);
    s.truth.push_back(x);
  }
  s.edges = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {0, 4}};
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);
  const Eigen::Matrix<double, 6, 1> sigmas = s.edge_cov.diagonal().cwiseSqrt();
  for (const auto& [i, j] : s.edges) {
    const Pose rel = s.truth[static_cast<std::size_t>(i)].T_w_b.inverse() *
                     s.truth[static_cast<std::size_t>(j)].T_w_b;
    Vec6 noise = Vec6::Zero();
    if (noisy) {
      for (int d = 0; d < 6; ++d) {
        noise(d) = sigmas(d) * n(rng);
      }
    }
    s.edge_meas.push_back(rel * common::se3_exp(noise));
  }
  s.guess = s.truth;
  if (noisy) {
    for (std::size_t k = 1; k < 5; ++k) {
      s.guess[k].T_w_b = s.guess[k - 1].T_w_b * s.edge_meas[k - 1];
    }
  } else {
    Vec6 d;
    d << 0.3, -0.2, 0.1, 0.05, -0.05, 0.08;
    for (std::size_t k = 1; k < 5; ++k) {
      s.guess[k].T_w_b = s.truth[k].T_w_b * common::se3_exp(d);
    }
  }
  return s;
}

}  // namespace mss::backend::test
