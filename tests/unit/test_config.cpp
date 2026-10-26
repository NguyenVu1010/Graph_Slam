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
