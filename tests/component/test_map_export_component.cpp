// Component tests of the global map export (Estimator::visit_map / export_map, ADR-016
// decision 6 and its 2026-10-29 amendment) on the synthetic box world and corridor
// (box_world.hpp, exact ranges): the exported points lie on the known walls, and after a
// backend (GNSS) correction the exported keyframes follow the optimised poses.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/span.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"
#include "mss/pipeline/map_export.hpp"

#include "box_world.hpp"
#include "estimator_core.hpp"
#include "pipeline_test_support.hpp"

namespace mss::pipeline {
namespace {

using namespace ct;  // NOLINT(google-build-using-namespace): shared test helpers

// Distance (m) of a point in w to the nearest face |x_k| = half_k of the box centred at the
// origin of w (box_world.hpp: the box is centred on the start position, w = start frame).
double face_dist_m(const Vec3& half_m, const Eigen::Vector3f& p) {
  double d = 1e9;
  for (int k = 0; k < 3; ++k) {
    d = std::min(d, std::abs(std::abs(static_cast<double>(p(k))) - half_m(k)));
  }
  return d;
}

struct FaceStats {
  double rms_m = 0.0;
  double p99_m = 0.0;
  double max_m = 0.0;
};

FaceStats face_stats(const Vec3& half_m, const std::vector<Eigen::Vector3f>& pts) {
  FaceStats s;
  if (pts.empty()) {
    return s;
  }
  std::vector<double> d;
  d.reserve(pts.size());
  double sum_sq = 0.0;
  for (const Eigen::Vector3f& p : pts) {
    d.push_back(face_dist_m(half_m, p));
    sum_sq += d.back() * d.back();
  }
  std::sort(d.begin(), d.end());
  s.rms_m = std::sqrt(sum_sq / static_cast<double>(d.size()));
  s.p99_m = d[(d.size() * 99) / 100];
  s.max_m = d.back();
  return s;
}

struct Exported {
  bool ok = false;
  MapExportInfo info;
  std::vector<Eigen::Vector3f> points;
};

// Replays the box world and exports the map after the last scan record.
Exported run_and_export(const BoxWorldParams& p, const Config& cfg, const fs::path& data) {
  Exported out;
  const int last = test::box_num_scans(p);
  const ReplayResult r = replay(data, cfg, [&](Estimator& est, int k) {
    if (k == last) {
      const auto info = est.export_map(out.points);
      out.ok = info.is_ok();
      if (info.is_ok()) {
        out.info = info.value();
      } else {
        EXPECT_TRUE(out.points.empty());
      }
    }
  });
  EXPECT_GT(r.stats.n_keyframes, 5U);
  return out;
}

// Known answer: the box world has exact ranges, so every exported point lies on a wall of the
// box up to the keyframe pose error (box run ATE < 5 cm). map_export.voxel_m 0 (the default)
// keeps every stored point; 0.5 m keeps one centroid per voxel.
TEST(MapExportComponent, PointsLieOnBoxWalls) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  Config cfg = box_config(p, false, false, dir);
  ASSERT_EQ(cfg.map_export.voxel_m, 0.0);  // shipped default
  const Exported full = run_and_export(p, cfg, dir / "data");
  if (!kWithGtsam) {  // the stub backend has no keyframe estimate: nothing to export
    EXPECT_FALSE(full.ok);
    return;
  }
  ASSERT_TRUE(full.ok);
  EXPECT_GT(full.info.n_keyframes, 5U);
  EXPECT_EQ(full.info.n_no_pose, 0U);
  EXPECT_EQ(full.info.n_evicted, 0U);
  EXPECT_EQ(full.info.n_points_in, full.points.size());
  EXPECT_EQ(full.info.n_points_out, full.points.size());
  const FaceStats f = face_stats(p.box_half_m, full.points);
  std::printf(
      "[ map export ] box: %zu keyframes, %zu points, wall distance RMS %.4f m, p99 %.4f m, "
      "max %.4f m\n",
      full.info.n_keyframes,
      full.points.size(),
      f.rms_m,
      f.p99_m,
      f.max_m);
  EXPECT_LT(f.rms_m, 0.02);
  EXPECT_LT(f.p99_m, 0.05);

  cfg.map_export.voxel_m = 0.5;
  const Exported coarse = run_and_export(p, cfg, dir / "data");
  ASSERT_TRUE(coarse.ok);
  EXPECT_EQ(coarse.info.n_points_in, full.info.n_points_in);  // same run (deterministic)
  EXPECT_EQ(coarse.info.n_points_out, coarse.points.size());
  EXPECT_LT(coarse.points.size(), full.points.size() / 2);
  std::printf("[ map export ] box voxel 0.5 m: %zu points\n", coarse.points.size());
}

// Records the backend state of each keyframe right after its own optimize(): the pose the
// front end inserted its cloud at, before any later correction.
class InsertObserver final : public detail::CoreObserver {
 public:
  void on_keyframe_optimized(Timestamp t,
                             const common::NavState& x,
                             const common::PoseCovariance& /*cov*/) override {
    inserted_[t.ns] = x.T_w_b;
  }
  const std::map<std::int64_t, common::Pose>& inserted() const { return inserted_; }

 private:
  std::map<std::int64_t, common::Pose> inserted_;
};

struct KeyframeSnapshot {
  Timestamp t{};
  common::Pose T_w_l;
  std::vector<float> xyz_l;
};

std::map<common::KeyId, KeyframeSnapshot> snapshot(const Estimator& est) {
  std::map<common::KeyId, KeyframeSnapshot> out;
  const auto r = est.visit_map([&out](const MapKeyframe& kf) {
    KeyframeSnapshot& s = out[kf.id];
    s.t = kf.t;
    s.T_w_l = kf.T_w_l;
    s.xyz_l.assign(kf.xyz_l.data(), kf.xyz_l.data() + kf.xyz_l.size());
  });
  EXPECT_TRUE(r.is_ok()) << r.reason();
  if (r.is_ok()) {
    EXPECT_EQ(r.value().n_keyframes, out.size());
    EXPECT_EQ(r.value().n_no_pose, 0U);
  }
  return out;
}

// A backend correction: the corridor (end walls out of range, x unobserved by the LiDAR,
// kCorridorRelaxYaml) with GNSS, whose later fixes pull earlier keyframes along the axis. The
// map is exported at mid-run and after the last scan:
//  1. keyframes exported at mid-run moved by the end, with an unchanged stored cloud;
//  2. the end export reads the backend now, not the insertion poses (observer): early
//     keyframes differ from their insertion pose, the newest one (no later optimize) equals it;
//  3. export_map's points are exactly the visit_map clouds at those poses (so they moved with
//     their keyframes), and lie on the corridor's side walls, floor and ceiling.
TEST(MapExportComponent, FollowsBackendCorrection) {
  if (!kWithGtsam) {
    GTEST_SKIP() << "the stub backend has no keyframe estimates to export";
  }
  const BoxWorldParams p = corridor_params();
  const fs::path dir = test_dir();
  ASSERT_TRUE(test::write_box_dataset(dir / "data", p).is_ok());
  const Config cfg =
      box_config(p,
                 true,
                 false,
                 dir,
                 std::string("    preprocess:\n      max_range_m: 10.0\n") + kCorridorRelaxYaml);
  InsertObserver obs;
  detail::CoreTestHooks hooks;
  hooks.observer = &obs;
  const HooksScope scope(&hooks);

  const int n_scans = test::box_num_scans(p);
  std::map<common::KeyId, KeyframeSnapshot> mid;
  std::map<common::KeyId, KeyframeSnapshot> end;
  std::vector<Eigen::Vector3f> end_points;
  const ReplayResult r = replay(dir / "data", cfg, [&](Estimator& est, int k) {
    if (k == n_scans / 2) {
      mid = snapshot(est);
    } else if (k == n_scans) {
      end = snapshot(est);
      const auto e = est.export_map(end_points);
      EXPECT_TRUE(e.is_ok()) << e.reason();
    }
  });
  ASSERT_GT(r.stats.n_gnss_factors, 100U);
  ASSERT_FALSE(mid.empty());
  ASSERT_GT(end.size(), mid.size());

  // 1. Mid-run keyframes moved between the two exports.
  double max_move_m = 0.0;
  for (const auto& [id, m] : mid) {
    const auto it = end.find(id);
    ASSERT_NE(it, end.end()) << id;
    EXPECT_EQ(it->second.xyz_l, m.xyz_l) << id;  // the stored cloud never changes
    max_move_m = std::max(max_move_m, (it->second.T_w_l.p - m.T_w_l.p).norm());
  }

  // 2. End export against the insertion poses.
  const common::Pose T_l_b = cfg.frames.T_b_l.inverse();
  double max_vs_insert_m = 0.0;
  double newest_vs_insert_m = -1.0;
  std::size_t n_observed = 0;
  for (const auto& [id, e] : end) {
    const auto it = obs.inserted().find(e.t.ns);
    if (it == obs.inserted().end()) {
      EXPECT_EQ(id, 0U);  // keyframe 0 is added without an optimize() report
      continue;
    }
    ++n_observed;
    const double d = ((e.T_w_l * T_l_b).p - it->second.p).norm();
    max_vs_insert_m = std::max(max_vs_insert_m, d);
    newest_vs_insert_m = d;  // std::map: the last one is the newest keyframe
  }
  EXPECT_GE(n_observed + 1, end.size());

  // 3. export_map == visit_map clouds at the end poses, and on the corridor walls.
  // The corridor poses carry real yaw / lateral errors (fused ATE tens of cm), so the exported
  // points are off the walls by about the pose error. Known answer: the same stored clouds at
  // the TRUE keyframe poses lie on the walls (cloud and frame chain T_w_b * T_b_l correct), and
  // every exported point is off its wall by at most its displacement from that true point.
  std::size_t n = 0;
  std::size_t n_mismatch = 0;
  std::size_t n_beyond_pose_error = 0;
  std::vector<Eigen::Vector3f> true_points;
  for (const auto& [id, e] : end) {
    (void)id;
    const common::Pose T_w_l_true = test::box_truth(p, e.t).T_w_b * cfg.frames.T_b_l;
    for (std::size_t i = 0; i + 2 < e.xyz_l.size(); i += 3, ++n) {
      const Vec3 p_l(e.xyz_l[i], e.xyz_l[i + 1], e.xyz_l[i + 2]);
      const Vec3 p_w = e.T_w_l * p_l;
      const Vec3 p_true = T_w_l_true * p_l;
      true_points.emplace_back(p_true.cast<float>());
      const Eigen::Vector3f expected = p_w.cast<float>();
      if (n >= end_points.size() || end_points[n] != expected) {
        ++n_mismatch;
        continue;
      }
      constexpr double kSlackM = 0.02;  // m, deskew and true-cloud residual (box test: p99 13 mm)
      if (face_dist_m(p.box_half_m, end_points[n]) > (p_w - p_true).norm() + kSlackM) {
        ++n_beyond_pose_error;
      }
    }
  }
  EXPECT_EQ(n, end_points.size());
  EXPECT_EQ(n_mismatch, 0U);
  EXPECT_LE(n_beyond_pose_error, n / 100);
  const FaceStats f = face_stats(p.box_half_m, end_points);
  const FaceStats f_true = face_stats(p.box_half_m, true_points);
  std::printf(
      "[ map export ] corridor clouds at the true poses: wall distance RMS %.4f m, p99 "
      "%.4f m; %zu points beyond their pose error\n",
      f_true.rms_m,
      f_true.p99_m,
      n_beyond_pose_error);
  EXPECT_LT(f_true.rms_m, 0.02);
  EXPECT_LT(f_true.p99_m, 0.05);
  std::printf(
      "[ map export ] corridor: %zu -> %zu keyframes, max move mid->end %.3f m, max |end - "
      "insertion| %.3f m, newest %.2e m; %zu points, wall distance RMS %.4f m, p99 %.4f m\n",
      mid.size(),
      end.size(),
      max_move_m,
      max_vs_insert_m,
      newest_vs_insert_m,
      end_points.size(),
      f.rms_m,
      f.p99_m);
  EXPECT_GT(max_move_m, 0.05);
  EXPECT_GT(max_vs_insert_m, 0.05);
  EXPECT_LT(newest_vs_insert_m, 1e-6);
}

}  // namespace
}  // namespace mss::pipeline
