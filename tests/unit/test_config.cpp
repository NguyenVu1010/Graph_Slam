#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "mss/common/lie.hpp"
#include "mss/pipeline/config.hpp"

namespace {

using mss::common::Mat3;
using mss::common::Vec3;
using mss::pipeline::load_config;
using mss::pipeline::parse_config;

const std::filesystem::path kRepo{MSS_REPO_DIR};
const std::filesystem::path kDefault = kRepo / "config/default.yaml";
const std::filesystem::path kWarehouse = kRepo / "config/sim/warehouse.yaml";
constexpr double kRadPerDeg = EIGEN_PI / 180.0;

std::string read_file(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Replaces the first occurrence of `from`; fails the test if it is absent.
std::string replace_once(std::string text, const std::string& from, const std::string& to) {
  const std::size_t pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  if (pos != std::string::npos) {
    text.replace(pos, from.size(), to);
  }
  return text;
}

std::string overlay(const std::string& body) {
  return "mss_node:\n  ros__parameters:\n" + body;
}

bool mentions(const std::string& reason, const std::string& key) {
  return reason.find(key) != std::string::npos;
}

TEST(Config, LoadsDefaultFile) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto& c = cfg.value();
  EXPECT_NEAR(c.backend.keyframe_rot_rad, 10.0 * kRadPerDeg, 1e-12);
  EXPECT_NEAR(c.backend.prior_rot_sigma_rad, 1.0 * kRadPerDeg, 1e-12);
  EXPECT_DOUBLE_EQ(c.backend.prior_bias_gyro_sigma_radps, 0.01);
  EXPECT_DOUBLE_EQ(c.gnss.max_cov_m2, 25.0);
  EXPECT_DOUBLE_EQ(c.wheel_odom.yaw_rate_sigma_radps, 0.02);
  EXPECT_EQ(c.map.max_voxels, 500000U);
  EXPECT_DOUBLE_EQ(c.map.keyframe_voxel_m, 0.5);
  EXPECT_EQ(c.map.max_keyframes, 1000U);
  EXPECT_EQ(c.map.window_keyframes, 50);
  EXPECT_DOUBLE_EQ(c.map.rebuild_trans_m, 0.05);
  EXPECT_NEAR(c.map.rebuild_rot_rad, 0.5 * std::acos(-1.0) / 180.0, 1e-12);
  EXPECT_TRUE(c.frames.T_b_l.R.isApprox(Mat3::Identity(), 1e-12));
  EXPECT_TRUE(c.frames.T_b_l.p.isZero(0.0));
  EXPECT_TRUE(c.frames.T_b_o.R.isApprox(Mat3::Identity(), 1e-12));
  EXPECT_DOUBLE_EQ(c.imu.rate_hz, 200.0);
  EXPECT_DOUBLE_EQ(c.imu.gyro_noise_density, 3.5e-4);
  EXPECT_DOUBLE_EQ(c.imu.acc_noise_density, 5.8e-3);
  EXPECT_DOUBLE_EQ(c.imu.gyro_random_walk, 2.0e-5);
  EXPECT_DOUBLE_EQ(c.imu.acc_random_walk, 2.0e-4);
  EXPECT_EQ(c.realtime.max_points_per_scan, 65536U);
  EXPECT_EQ(c.random_seed, 42U);
  EXPECT_EQ(c.frames.body, "imu_link");
}

TEST(Config, OverlayWarehouse) {
  const auto cfg = load_config(kDefault, kWarehouse);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto& c = cfg.value();
  EXPECT_DOUBLE_EQ(c.wheel_odom.wheel_radius_m, 0.101);
  EXPECT_DOUBLE_EQ(c.frames.T_b_l.p.x(), 0.10);
  const Mat3 R_expected = mss::common::so3_exp(Vec3(0.0, 0.0349, 0.0));
  EXPECT_TRUE(c.frames.T_b_l.R.isApprox(R_expected, 1e-3)) << c.frames.T_b_l.R;
  EXPECT_DOUBLE_EQ(c.imu.rate_hz, 200.0);  // not in the overlay: default kept
  EXPECT_TRUE(c.gnss.enabled);
}

TEST(Config, MissingKeyNamed) {
  const std::string text = replace_once(read_file(kDefault), "      rate_hz: 200.0\n", "");
  const auto cfg = parse_config(text);
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "imu.rate_hz")) << cfg.reason();
}

TEST(Config, WrongTypeNamed) {
  const std::string base = read_file(kDefault);
  const auto bad_double = parse_config(replace_once(base, "rate_hz: 200.0", "rate_hz: \"fast\""));
  ASSERT_FALSE(bad_double.is_ok());
  EXPECT_TRUE(mentions(bad_double.reason(), "imu.rate_hz")) << bad_double.reason();

  const auto bad_bool =
      parse_config(replace_once(base, "deskew_enabled: true", "deskew_enabled: 3"));
  ASSERT_FALSE(bad_bool.is_ok());
  EXPECT_TRUE(mentions(bad_bool.reason(), "preprocess.deskew_enabled")) << bad_bool.reason();

  const auto bad_int = parse_config(replace_once(base, "knn_k: 5", "knn_k: 5.5"));
  ASSERT_FALSE(bad_int.is_ok());
  EXPECT_TRUE(mentions(bad_int.reason(), "map.knn_k")) << bad_int.reason();

  const auto bad_vec =
      parse_config(replace_once(base, "p_b_g_m: [0.0, 0.0, 0.0]", "p_b_g_m: [0.0]"));
  ASSERT_FALSE(bad_vec.is_ok());
  EXPECT_TRUE(mentions(bad_vec.reason(), "frames.p_b_g_m")) << bad_vec.reason();
}

TEST(Config, UnknownKeyNamed) {
  const std::string base = read_file(kDefault);
  const auto in_base = parse_config(
      replace_once(base, "      rate_hz: 200.0\n", "      rate_hz: 200.0\n      foo: 1\n"));
  ASSERT_FALSE(in_base.is_ok());
  EXPECT_TRUE(mentions(in_base.reason(), "imu.foo")) << in_base.reason();

  const auto in_overlay = parse_config(base, overlay("    imu:\n      foo: 1\n"));
  ASSERT_FALSE(in_overlay.is_ok());
  EXPECT_TRUE(mentions(in_overlay.reason(), "imu.foo")) << in_overlay.reason();
}

TEST(Config, OverlayTypeMismatch) {
  // Scalar over a mapping.
  const auto cfg = parse_config(read_file(kDefault), overlay("    imu: 5\n"));
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "imu")) << cfg.reason();
  EXPECT_TRUE(mentions(cfg.reason(), "overlay type mismatch")) << cfg.reason();
  // Mapping over a scalar.
  const auto map_over_scalar =
      parse_config(read_file(kDefault), overlay("    imu:\n      rate_hz:\n        x: 1\n"));
  ASSERT_FALSE(map_over_scalar.is_ok());
  EXPECT_TRUE(mentions(map_over_scalar.reason(), "imu.rate_hz")) << map_over_scalar.reason();
  EXPECT_TRUE(mentions(map_over_scalar.reason(), "overlay type mismatch"))
      << map_over_scalar.reason();
}

TEST(Config, BadDegKeyNamed) {
  const auto cfg =
      parse_config(read_file(kDefault), overlay("    backend:\n      keyframe_rot_deg: \"x\"\n"));
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "backend.keyframe_rot_deg")) << cfg.reason();
}

TEST(Config, ZeroImuNoiseRejected) {
  const std::string base = read_file(kDefault);
  for (const char* key :
       {"acc_noise_density", "gyro_noise_density", "acc_random_walk", "gyro_random_walk"}) {
    const auto cfg = parse_config(base, overlay(std::string("    imu:\n      ") + key + ": 0.0\n"));
    ASSERT_FALSE(cfg.is_ok()) << key;
    EXPECT_TRUE(mentions(cfg.reason(), std::string("imu.") + key)) << cfg.reason();
  }
}

TEST(Config, UnnormalisedQuaternionRejected) {
  const std::string text = replace_once(read_file(kDefault),
                                        "quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]",
                                        "quaternion_wxyz: [1, 0, 0.5, 0]");
  const auto cfg = parse_config(text);
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "frames.T_b_l.quaternion_wxyz")) << cfg.reason();
}

TEST(Config, NegativeSigmaRejected) {
  const std::string text = replace_once(read_file(kDefault), "sigma_h_m: 1.5", "sigma_h_m: -1.5");
  const auto cfg = parse_config(text);
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "gnss.sigma_h_m")) << cfg.reason();
}

TEST(Config, ScanMatchResidualSigmaFloor) {
  const auto ok = load_config(kDefault);
  ASSERT_TRUE(ok.is_ok()) << ok.reason();
  EXPECT_DOUBLE_EQ(ok.value().scan_match.min_residual_sigma_m, 0.005);
  const auto cfg = parse_config(read_file(kDefault),
                                overlay("    scan_match:\n      min_residual_sigma_m: 0.0\n"));
  ASSERT_FALSE(cfg.is_ok());
  EXPECT_TRUE(mentions(cfg.reason(), "scan_match.min_residual_sigma_m")) << cfg.reason();
}

TEST(Config, ScanMatchPlanarityGateRange) {
  const auto ok = load_config(kDefault);
  ASSERT_TRUE(ok.is_ok()) << ok.reason();
  // Shipped default is off (ADR-007 amendment: evaluated only on noise-free data).
  EXPECT_EQ(ok.value().scan_match.plane_min_eig_ratio, 0.0);
  for (const char* bad : {"-0.1", "1.0"}) {
    const auto cfg = parse_config(
        read_file(kDefault),
        overlay(std::string("    scan_match:\n      plane_min_eig_ratio: ") + bad + "\n"));
    ASSERT_FALSE(cfg.is_ok()) << bad;
    EXPECT_TRUE(mentions(cfg.reason(), "scan_match.plane_min_eig_ratio")) << cfg.reason();
  }
}

TEST(Config, ScanMatchCollinearityGateRange) {
  const auto ok = load_config(kDefault);
  ASSERT_TRUE(ok.is_ok()) << ok.reason();
  // Shipped default is off (ADR-007 amendment: breaks narrow corridors).
  EXPECT_EQ(ok.value().scan_match.min_mid_eig_ratio, 0.0);
  for (const char* bad : {"-0.1", "1.0"}) {
    const auto cfg = parse_config(
        read_file(kDefault),
        overlay(std::string("    scan_match:\n      min_mid_eig_ratio: ") + bad + "\n"));
    ASSERT_FALSE(cfg.is_ok()) << bad;
    EXPECT_TRUE(mentions(cfg.reason(), "scan_match.min_mid_eig_ratio")) << cfg.reason();
  }
}

TEST(Config, ScanMatchUnobservedVariances) {
  const auto ok = load_config(kDefault);
  ASSERT_TRUE(ok.is_ok()) << ok.reason();
  EXPECT_DOUBLE_EQ(ok.value().scan_match.unobserved_var_m2, 0.001);
  EXPECT_DOUBLE_EQ(ok.value().scan_match.unobserved_var_rad2, 1e-4);
  for (const char* key : {"unobserved_var_m2", "unobserved_var_rad2"}) {
    const auto cfg = parse_config(
        read_file(kDefault), overlay(std::string("    scan_match:\n      ") + key + ": 0.0\n"));
    ASSERT_FALSE(cfg.is_ok()) << key;
    EXPECT_TRUE(mentions(cfg.reason(), std::string("scan_match.") + key)) << cfg.reason();
  }
}

// ADR-018: backend.single_thread is not a YAML key; it follows `deterministic` (ADR-008).
TEST(Config, DeterministicSetsBackendSingleThread) {
  const std::string base = read_file(kDefault);
  const auto on = parse_config(base, overlay("    deterministic: true\n"));
  ASSERT_TRUE(on.is_ok()) << on.reason();
  EXPECT_TRUE(on.value().deterministic);
  EXPECT_TRUE(on.value().backend.single_thread);
  const auto off = parse_config(base, overlay("    deterministic: false\n"));
  ASSERT_TRUE(off.is_ok()) << off.reason();
  EXPECT_FALSE(off.value().deterministic);
  EXPECT_FALSE(off.value().backend.single_thread);
}

TEST(Config, MapMinPointDistValidated) {
  const auto ok = load_config(kDefault);
  ASSERT_TRUE(ok.is_ok()) << ok.reason();
  // Shipped default is off (ADR-007 amendment: E-01 cost at 0.1 m).
  EXPECT_EQ(ok.value().map.min_point_dist_m, 0.0);
  const std::string base = read_file(kDefault);
  const auto off = parse_config(base, overlay("    map:\n      min_point_dist_m: 0.0\n"));
  ASSERT_TRUE(off.is_ok()) << off.reason();
  EXPECT_EQ(off.value().map.min_point_dist_m, 0.0);
  const auto bad = parse_config(base, overlay("    map:\n      min_point_dist_m: -0.1\n"));
  ASSERT_FALSE(bad.is_ok());
  EXPECT_TRUE(mentions(bad.reason(), "map.min_point_dist_m: must be >= 0 (got -0.1)"))
      << bad.reason();
}

TEST(Config, GnssGateAndAlignmentKeysLoaded) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  EXPECT_EQ(cfg.value().gnss.align_min_pairs, 10);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_min_spread_m, 1.0);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.chi2_gate, 7.814727903251178);
  EXPECT_EQ(cfg.value().gnss.max_consecutive_rejects, 60);
  const auto bad =
      parse_config(read_file(kDefault), overlay("    gnss:\n      max_consecutive_rejects: 0\n"));
  ASSERT_FALSE(bad.is_ok());
  EXPECT_TRUE(mentions(bad.reason(), "gnss.max_consecutive_rejects: must be >= 1 (got 0)"))
      << bad.reason();
  // ADR-022: off by default (previous behaviour), buffer bound >= 1.
  EXPECT_FALSE(cfg.value().gnss.attach_prealign_fixes);
  EXPECT_EQ(cfg.value().gnss.prealign_max_fixes, 320);
  const auto bad_buf =
      parse_config(read_file(kDefault), overlay("    gnss:\n      prealign_max_fixes: 0\n"));
  ASSERT_FALSE(bad_buf.is_ok());
  EXPECT_TRUE(mentions(bad_buf.reason(), "gnss.prealign_max_fixes: must be >= 1 (got 0)"))
      << bad_buf.reason();
  // ADR-023: off by default (previous behaviour); chunk >= 2, sigma (deg -> rad) and n > 0.
  EXPECT_FALSE(cfg.value().gnss.align_yaw_from_track);
  EXPECT_EQ(cfg.value().gnss.align_yaw_chunk_fixes, 10);
  EXPECT_NEAR(cfg.value().gnss.align_yaw_max_sigma_rad, 3.0 * mss::gnss::kRadPerDegree, 1e-12);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_yaw_reverse_nsigma, 3.0);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_yaw_min_track_m, 5.0);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_yaw_min_coherence, 0.5);
  EXPECT_NEAR(cfg.value().gnss.align_yaw_max_turn_rad, 45.0 * mss::gnss::kRadPerDegree, 1e-12);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_yaw_min_var_m2, 0.25);
  EXPECT_DOUBLE_EQ(cfg.value().gnss.align_yaw_min_wheel_travel_m, 0.2);
  const auto bad_wheel = parse_config(
      read_file(kDefault), overlay("    gnss:\n      align_yaw_min_wheel_travel_m: 0\n"));
  ASSERT_FALSE(bad_wheel.is_ok());
  EXPECT_TRUE(
      mentions(bad_wheel.reason(), "gnss.align_yaw_min_wheel_travel_m: must be > 0 (got 0)"))
      << bad_wheel.reason();
  for (const char* v : {"0", "181"}) {
    const auto bad_turn =
        parse_config(read_file(kDefault),
                     overlay(std::string("    gnss:\n      align_yaw_max_turn_deg: ") + v + "\n"));
    ASSERT_FALSE(bad_turn.is_ok());
    EXPECT_TRUE(
        mentions(bad_turn.reason(),
                 std::string("gnss.align_yaw_max_turn_deg: must be in (0, 180] (got ") + v + ")"))
        << bad_turn.reason();
  }
  const auto bad_var =
      parse_config(read_file(kDefault), overlay("    gnss:\n      align_yaw_min_var_m2: 0\n"));
  ASSERT_FALSE(bad_var.is_ok());
  EXPECT_TRUE(mentions(bad_var.reason(), "gnss.align_yaw_min_var_m2: must be > 0 (got 0)"))
      << bad_var.reason();
  const auto bad_track =
      parse_config(read_file(kDefault), overlay("    gnss:\n      align_yaw_min_track_m: 0\n"));
  ASSERT_FALSE(bad_track.is_ok());
  EXPECT_TRUE(mentions(bad_track.reason(), "gnss.align_yaw_min_track_m: must be > 0 (got 0)"))
      << bad_track.reason();
  for (const char* v : {"0", "1.5"}) {
    const auto bad_coh =
        parse_config(read_file(kDefault),
                     overlay(std::string("    gnss:\n      align_yaw_min_coherence: ") + v + "\n"));
    ASSERT_FALSE(bad_coh.is_ok());
    EXPECT_TRUE(
        mentions(bad_coh.reason(),
                 std::string("gnss.align_yaw_min_coherence: must be in (0, 1] (got ") + v + ")"))
        << bad_coh.reason();
  }
  const auto bad_chunk =
      parse_config(read_file(kDefault), overlay("    gnss:\n      align_yaw_chunk_fixes: 1\n"));
  ASSERT_FALSE(bad_chunk.is_ok());
  EXPECT_TRUE(mentions(bad_chunk.reason(), "gnss.align_yaw_chunk_fixes: must be >= 2 (got 1)"))
      << bad_chunk.reason();
  const auto bad_sigma =
      parse_config(read_file(kDefault), overlay("    gnss:\n      align_yaw_max_sigma_deg: 0\n"));
  ASSERT_FALSE(bad_sigma.is_ok());
  EXPECT_TRUE(mentions(bad_sigma.reason(), "gnss.align_yaw_max_sigma_deg: must be > 0 (got 0)"))
      << bad_sigma.reason();
  const auto bad_n =
      parse_config(read_file(kDefault), overlay("    gnss:\n      align_yaw_reverse_nsigma: -1\n"));
  ASSERT_FALSE(bad_n.is_ok());
  EXPECT_TRUE(mentions(bad_n.reason(), "gnss.align_yaw_reverse_nsigma: must be > 0 (got -1)"))
      << bad_n.reason();
}

// Ruling on the static-init threshold: it must exceed the per-sample accelerometer noise
// acc_noise_density * sqrt(rate_hz) by about 3x, or a still sensor never initialises.
TEST(Config, ImuInitAccThresholdAboveSampleNoise) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto& imu = cfg.value().imu;
  const double sample_std_mps2 = imu.acc_noise_density * std::sqrt(imu.rate_hz);
  EXPECT_GE(imu.init_max_acc_std_mps2, 3.0 * sample_std_mps2);
  const double gyro_std_radps = imu.gyro_noise_density * std::sqrt(imu.rate_hz);
  EXPECT_GE(imu.init_max_gyro_std_radps, gyro_std_radps);
}

TEST(Config, GnssGateKeysValidated) {
  const std::string base = read_file(kDefault);
  const auto pairs = parse_config(base, overlay("    gnss:\n      align_min_pairs: 0\n"));
  ASSERT_FALSE(pairs.is_ok());
  EXPECT_TRUE(mentions(pairs.reason(), "gnss.align_min_pairs: must be >= 1 (got 0)"))
      << pairs.reason();
  const auto spread = parse_config(base, overlay("    gnss:\n      align_min_spread_m: 0\n"));
  ASSERT_FALSE(spread.is_ok());
  EXPECT_TRUE(mentions(spread.reason(), "gnss.align_min_spread_m: must be > 0 (got 0)"))
      << spread.reason();
  const auto gate = parse_config(base, overlay("    gnss:\n      chi2_gate: -1\n"));
  ASSERT_FALSE(gate.is_ok());
  EXPECT_TRUE(mentions(gate.reason(), "gnss.chi2_gate: must be > 0 (got -1)")) << gate.reason();
}

// ADR-021: backend.relative_factor defaults to decoupled; between and se3_exact parse (kept
// for comparison); any other value is a config error naming the key and the allowed values.
TEST(Config, RelativeFactorKey) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  EXPECT_EQ(cfg.value().backend.relative_factor, mss::backend::RelativeFactor::kDecoupled);
  EXPECT_EQ(mss::backend::BackendConfig{}.relative_factor,
            mss::backend::RelativeFactor::kDecoupled);
  const std::string base = read_file(kDefault);
  const auto btw = parse_config(base, overlay("    backend:\n      relative_factor: between\n"));
  ASSERT_TRUE(btw.is_ok()) << btw.reason();
  EXPECT_EQ(btw.value().backend.relative_factor, mss::backend::RelativeFactor::kBetween);
  const auto se3 = parse_config(base, overlay("    backend:\n      relative_factor: se3_exact\n"));
  ASSERT_TRUE(se3.is_ok()) << se3.reason();
  EXPECT_EQ(se3.value().backend.relative_factor, mss::backend::RelativeFactor::kSe3Exact);
  const auto dec = parse_config(base, overlay("    backend:\n      relative_factor: decoupled\n"));
  ASSERT_TRUE(dec.is_ok()) << dec.reason();
  EXPECT_EQ(dec.value().backend.relative_factor, mss::backend::RelativeFactor::kDecoupled);
  const auto bad = parse_config(base, overlay("    backend:\n      relative_factor: exact\n"));
  ASSERT_FALSE(bad.is_ok());
  EXPECT_TRUE(mentions(bad.reason(),
                       "backend.relative_factor: must be one of between, se3_exact, decoupled"))
      << bad.reason();
}

// PR 4d: the static-window gravity factor ships on with sigma scale 2 (evaluated, config
// comment); the flag parses as a bool and the scale must be > 0.
TEST(Config, StaticGravityFactorKey) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  EXPECT_TRUE(cfg.value().backend.static_gravity_factor);
  EXPECT_DOUBLE_EQ(cfg.value().backend.static_gravity_sigma_scale, 2.0);
  const std::string base = read_file(kDefault);
  const auto off =
      parse_config(base, overlay("    backend:\n      static_gravity_factor: false\n"));
  ASSERT_TRUE(off.is_ok()) << off.reason();
  const auto bad = parse_config(base, overlay("    backend:\n      static_gravity_factor: 3.5\n"));
  EXPECT_FALSE(bad.is_ok());
  const auto zero =
      parse_config(base, overlay("    backend:\n      static_gravity_sigma_scale: 0\n"));
  ASSERT_FALSE(zero.is_ok());
  EXPECT_TRUE(mentions(zero.reason(), "backend.static_gravity_sigma_scale")) << zero.reason();
  // ADR-020 rest gate (imu.*) and consistency check (backend.*): shipped values, > 0.
  EXPECT_DOUBLE_EQ(cfg.value().imu.init_max_mean_gyro_radps, 0.15);
  EXPECT_DOUBLE_EQ(cfg.value().imu.init_max_wheel_speed_mps, 0.1);
  EXPECT_DOUBLE_EQ(cfg.value().imu.init_gravity_check_sigma, 3.0);
  // Consistency check shipped off (0); 0 and positive values parse, negative ones do not.
  EXPECT_EQ(cfg.value().backend.static_gravity_check_keyframes, 0);
  EXPECT_EQ(mss::backend::BackendConfig{}.static_gravity_check_keyframes, 0);
  const auto on =
      parse_config(base, overlay("    backend:\n      static_gravity_check_keyframes: 5\n"));
  ASSERT_TRUE(on.is_ok()) << on.reason();
  EXPECT_EQ(on.value().backend.static_gravity_check_keyframes, 5);
  EXPECT_NEAR(cfg.value().backend.static_gravity_chi2_gate, 5.991, 1e-3);
  for (const char* bad_key : {"    imu:\n      init_max_mean_gyro_radps: 0\n",
                              "    imu:\n      init_max_wheel_speed_mps: -1\n",
                              "    imu:\n      init_gravity_check_sigma: 0\n",
                              "    backend:\n      static_gravity_check_keyframes: -1\n",
                              "    backend:\n      static_gravity_chi2_gate: 0\n"}) {
    EXPECT_FALSE(parse_config(base, overlay(bad_key)).is_ok()) << bad_key;
  }
}

// ADR-019: the GNSS frame variable and its prior sigmas (degrees in YAML, radians in code).
TEST(Config, GnssFrameVariableKeys) {
  const auto cfg = load_config(kDefault);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto& b = cfg.value().backend;
  EXPECT_TRUE(b.gnss_frame_variable);
  EXPECT_DOUBLE_EQ(b.gnss_frame_tilt_sigma_rad, 0.01 * kRadPerDeg);
  EXPECT_DOUBLE_EQ(b.gnss_frame_yaw_sigma_rad, 30.0 * kRadPerDeg);
  EXPECT_DOUBLE_EQ(b.gnss_frame_pos_sigma_m, 10.0);
  const std::string base = read_file(kDefault);
  const auto tilt =
      parse_config(base, overlay("    backend:\n      gnss_frame_tilt_sigma_deg: 0\n"));
  ASSERT_FALSE(tilt.is_ok());
  EXPECT_TRUE(mentions(tilt.reason(), "backend.gnss_frame_tilt_sigma_deg: must be > 0"))
      << tilt.reason();
  const auto pos = parse_config(base, overlay("    backend:\n      gnss_frame_pos_sigma_m: -1\n"));
  ASSERT_FALSE(pos.is_ok());
  EXPECT_TRUE(mentions(pos.reason(), "backend.gnss_frame_pos_sigma_m: must be > 0"))
      << pos.reason();
}

TEST(Config, AllErrorsReported) {
  std::string text = replace_once(read_file(kDefault), "      rate_hz: 200.0\n", "");
  text = replace_once(text, "sigma_h_m: 1.5", "sigma_h_m: -1.5");
  const auto cfg = parse_config(text);
  ASSERT_FALSE(cfg.is_ok());
  const std::string& reason = cfg.reason();
  EXPECT_TRUE(mentions(reason, "imu.rate_hz")) << reason;
  EXPECT_TRUE(mentions(reason, "gnss.sigma_h_m")) << reason;
  // One line per problem, in document order (imu comes before gnss).
  EXPECT_LT(reason.find("imu.rate_hz"), reason.find("gnss.sigma_h_m")) << reason;
  EXPECT_TRUE(mentions(reason, "\n")) << reason;
}

TEST(Config, MalformedInputReturnsErrorWithoutThrowing) {
  EXPECT_NO_THROW({
    const auto bad_yaml = parse_config("mss_node: [unclosed");
    EXPECT_FALSE(bad_yaml.is_ok());
    const auto no_params = parse_config("other: 1\n");
    ASSERT_FALSE(no_params.is_ok());
    EXPECT_TRUE(mentions(no_params.reason(), "mss_node.ros__parameters")) << no_params.reason();
    const auto bad_overlay = parse_config(read_file(kDefault), std::string("mss_node: {"));
    EXPECT_FALSE(bad_overlay.is_ok());
    const auto missing_file = load_config(kRepo / "config/does_not_exist.yaml");
    ASSERT_FALSE(missing_file.is_ok());
    EXPECT_TRUE(mentions(missing_file.reason(), "does_not_exist.yaml")) << missing_file.reason();
  });
}

}  // namespace
