// Component tests of the estimator's backend thread (ADR-033, deterministic: false) on the
// synthetic box world (box_world.hpp, known trajectory): the threaded run tracks the truth like
// the sequential one, a stalled or slow backend never stalls the front-end (every scan still gets
// a pose, refused keyframes are counted, stop does not deadlock). Run under ThreadSanitizer in CI
// (the tsan job, no GTSAM: optimize() fails, so no correction is applied, but every queue, cloud
// slot and snapshot hand-over runs across the two threads).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "mss/common/scan_frame.hpp"
#include "mss/common/timestamp.hpp"
#include "mss/common/types.hpp"
#include "mss/io/dataset_reader.hpp"
#include "mss/pipeline/config.hpp"
#include "mss/pipeline/estimator.hpp"

#include "box_world.hpp"
#include "estimator_core.hpp"
#include "pipeline_test_support.hpp"

namespace mss::pipeline {
namespace {

using namespace ct;  // NOLINT(google-build-using-namespace): shared test helpers

struct ThreadedRun {
  ReplayResult r;
  double max_run_once_ms = 0.0;  // wall time of the slowest run_once() (front-end)
};

// As ct::replay(), with the backend thread started (cfg.deterministic must be false) and the
// wall time of every run_once() recorded. With `speedup` > 0 each scan is pushed no earlier than
// its sensor time divided by `speedup` after the first one (paced replay); 0: as fast as possible.
// flush() joins the thread.
ThreadedRun replay_threaded(const fs::path& data, const Config& cfg, double speedup = 0.0) {
  ThreadedRun out;
  auto opened = io::DatasetReader::open(data, cfg.realtime.max_points_per_scan);
  EXPECT_TRUE(opened.is_ok()) << opened.reason();
  if (!opened.is_ok()) {
    return out;
  }
  io::DatasetReader reader = std::move(opened).value();
  Estimator est(cfg);
  est.start();
  ScanFrame scan(cfg.realtime.max_points_per_scan);
  io::Record rec;
  Timestamp last{};
  bool have_t0 = false;
  Timestamp t0_scan{};
  const std::chrono::steady_clock::time_point wall0 = std::chrono::steady_clock::now();
  const auto run_once = [&]() {
    const auto t0 = std::chrono::steady_clock::now();
    est.run_once();
    const std::chrono::duration<double, std::milli> dt = std::chrono::steady_clock::now() - t0;
    out.max_run_once_ms = std::max(out.max_run_once_ms, dt.count());
    const Estimate e = est.latest_state();
    if (e.state.t != Timestamp{} && e.state.t != last) {
      out.r.estimates.push_back(e);
      last = e.state.t;
    }
  };
  for (;;) {
    const auto more = reader.next(rec, scan);
    EXPECT_TRUE(more.is_ok()) << more.reason();
    if (!more.is_ok() || !more.value()) {
      break;
    }
    switch (rec.kind) {
      case io::RecordKind::kImu:
        (void)est.push_imu(rec.imu);
        break;
      case io::RecordKind::kGnss:
        (void)est.push_gnss(rec.gnss);
        break;
      case io::RecordKind::kOdom:
        (void)est.push_odom(rec.odom);
        break;
      case io::RecordKind::kLidar:
        if (speedup > 0.0) {
          if (!have_t0) {
            t0_scan = scan.t_start;
            have_t0 = true;
          }
          const double dt_s = common::seconds_between(t0_scan, scan.t_start) / speedup;
          std::this_thread::sleep_until(wall0 + std::chrono::duration<double>(dt_s));
        }
        run_once();
        (void)est.push_scan(scan);
        break;
      case io::RecordKind::kGt:
        break;
    }
    run_once();
  }
  est.flush();
  out.r.stats = est.stats();
  return out;
}

Config threaded(Config cfg) {
  cfg.deterministic = false;
  cfg.backend.single_thread = false;  // follows `deterministic` (load_config, ADR-018)
  return cfg;
}

void print_run(const char* name, const BoxWorldParams& p, const ThreadedRun& t) {
  const Estimator::Stats& s = t.r.stats;
  std::printf(
      "[ threads ] %s: ATE %.4f m, %zu estimates, %llu keyframes, %llu corrections, dropped kf "
      "%llu fix %llu prune %llu update %llu, max run_once %.1f ms\n",
      name,
      trajectory_errors(p, t.r.estimates).ate_m,
      t.r.estimates.size(),
      static_cast<unsigned long long>(s.n_keyframes),
      static_cast<unsigned long long>(s.n_backend_corrections),
      static_cast<unsigned long long>(s.n_backend_keyframes_dropped),
      static_cast<unsigned long long>(s.n_backend_fixes_dropped),
      static_cast<unsigned long long>(s.n_backend_prunes_dropped),
      static_cast<unsigned long long>(s.n_backend_updates_dropped),
      t.max_run_once_ms);
}

// The threaded run on the box world with GNSS and wheel odometry: a pose for every scan, nothing
// dropped at this load, the same accuracy class as the sequential run (ADR-033: ATE within 5%
// of it plus a small absolute margin for this short, accurate run).
TEST(EstimatorThreads, BoxWorldThreadedTracksLikeSequential) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  const Config cfg = box_config(p, true, dir);
  const auto written = test::write_box_dataset(dir / "data", p);
  ASSERT_TRUE(written.is_ok()) << written.reason();
  const ReplayResult seq = replay(dir / "data", cfg);
  const ThreadedRun thr = replay_threaded(dir / "data", threaded(cfg));
  const Estimator::Stats& s = thr.r.stats;
  print_run("box world", p, thr);
  const double ate_seq = trajectory_errors(p, seq.estimates).ate_m;
  const double ate_thr = trajectory_errors(p, thr.r.estimates).ate_m;
  std::printf("[ threads ] sequential ATE %.4f m, threaded %.4f m\n", ate_seq, ate_thr);

  EXPECT_EQ(thr.r.estimates.size(), expected_estimates(p, cfg));
  EXPECT_EQ(s.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_GT(s.n_keyframes, 5U);
  EXPECT_GT(s.n_gnss_factors, 0U);
  EXPECT_GT(s.n_odom_factors, 0U);
  EXPECT_EQ(s.n_backend_keyframes_dropped, 0U);
  EXPECT_EQ(s.n_backend_fixes_dropped, 0U);
  EXPECT_EQ(s.n_backend_prunes_dropped, 0U);
  EXPECT_EQ(s.n_backend_updates_dropped, 0U);
  EXPECT_EQ(s.n_map_publish_skipped, 0U);
  EXPECT_GT(s.n_map_points, 0U);
  // Every correction composes rotations (T_corr * T); without re-projection onto SO(3) their
  // non-orthonormal residue grows about 3x per keyframe (1e-5 after these 23 keyframes) and
  // breaks scan matching after a few dozen. It must stay at rounding level.
  double max_ortho = 0.0;
  for (const Estimate& e : thr.r.estimates) {
    const common::Mat3& R = e.state.T_w_b.R;
    max_ortho = std::max(max_ortho, (R.transpose() * R - common::Mat3::Identity()).norm());
  }
  EXPECT_LT(max_ortho, 1e-12);
  if (kWithGtsam) {
    // Every keyframe's update reached the front-end (the last ones at flush()).
    EXPECT_EQ(s.n_backend_corrections, s.n_keyframes);
    EXPECT_LE(ate_thr, 1.05 * ate_seq + 0.01);
  } else {
    EXPECT_EQ(s.n_backend_corrections, 0U);  // stub optimize() fails: nothing to correct
    EXPECT_LT(ate_thr, 0.10);
  }
}

// A backend that stops after two keyframes until the replay is over: the front-end keeps
// producing a pose for every scan against the last published map, refuses keyframes once the
// cloud buffers are taken (counted), and flush() joins without a deadlock once the backend runs
// again, handing it every keyframe that was queued.
TEST(EstimatorThreads, StalledBackendDoesNotStallFrontEnd) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  const Config cfg = threaded(box_config(p, false, true, dir));
  const auto written = test::write_box_dataset(dir / "data", p);
  ASSERT_TRUE(written.is_ok()) << written.reason();

  std::mutex m;
  std::condition_variable cv;
  bool released = false;
  std::atomic<std::uint64_t> n_seen{0};
  detail::CoreTestHooks hooks;
  hooks.before_backend_keyframe = [&](std::uint64_t n_handled) {
    n_seen.store(n_handled + 1);
    if (n_handled < 2) {
      return;
    }
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [&released] { return released; });
  };
  const HooksScope scope(&hooks);
  ThreadedRun thr;
  {
    auto opened = io::DatasetReader::open(dir / "data", cfg.realtime.max_points_per_scan);
    ASSERT_TRUE(opened.is_ok()) << opened.reason();
    io::DatasetReader reader = std::move(opened).value();
    Estimator est(cfg);
    est.start();
    // Releases the backend on every exit (a failed ASSERT included), before ~Estimator joins it.
    struct Release {
      std::mutex& m;
      std::condition_variable& cv;
      bool& released;
      ~Release() {
        {
          const std::lock_guard<std::mutex> lock(m);
          released = true;
        }
        cv.notify_all();
      }
    } release{m, cv, released};
    ScanFrame scan(cfg.realtime.max_points_per_scan);
    io::Record rec;
    Timestamp last{};
    const auto collect = [&]() {
      const Estimate e = est.latest_state();
      if (e.state.t != Timestamp{} && e.state.t != last) {
        thr.r.estimates.push_back(e);
        last = e.state.t;
      }
    };
    for (;;) {
      const auto more = reader.next(rec, scan);
      ASSERT_TRUE(more.is_ok()) << more.reason();
      if (!more.value()) {
        break;
      }
      if (rec.kind == io::RecordKind::kImu) {
        (void)est.push_imu(rec.imu);
      } else if (rec.kind == io::RecordKind::kOdom) {
        (void)est.push_odom(rec.odom);
      } else if (rec.kind == io::RecordKind::kLidar) {
        est.run_once();
        collect();
        (void)est.push_scan(scan);
      }
      est.run_once();
      collect();
    }
    // Still stalled here (a front-end that waited for the backend would never get here).
    EXPECT_GT(est.stats().n_backend_keyframes_dropped, 0U);
    {
      const std::lock_guard<std::mutex> lock(m);
      released = true;
    }
    cv.notify_all();
    est.flush();  // joins the backend after it handled every queued keyframe
    collect();
    thr.r.stats = est.stats();
  }
  print_run("stalled backend", p, thr);
  const Estimator::Stats& s = thr.r.stats;
  EXPECT_EQ(thr.r.estimates.size(), expected_estimates(p, cfg));
  EXPECT_EQ(s.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_GT(s.n_backend_keyframes_dropped, 0U);
  // Keyframe 0, 1 and 2 (the one it stalled on), then one per free cloud slot: the backend
  // handled exactly the keyframes the front-end handed over.
  EXPECT_EQ(n_seen.load(), s.n_keyframes);
  EXPECT_LE(s.n_keyframes, 3U + detail::kKeyframeCloudSlots);
  EXPECT_EQ(s.n_backend_updates_dropped, 0U);
  // The front-end matched the scans against the last published map (keyframes 0 and 1).
  EXPECT_LT(trajectory_errors(p, thr.r.estimates).ate_m, 0.2);
}

// A slow backend on a paced replay (4x sensor rate, one scan per 25 ms): from keyframe 1 on the
// backend sleeps 400 ms (16 paced scan periods) per keyframe. The front-end keeps a pose for every
// scan and never waits, and the run ends cleanly. Whether keyframes are refused depends on how fast
// the front-end runs (under ThreadSanitizer it can be slower than the backend), so drop counting is
// checked deterministically by StalledBackendDoesNotStallFrontEnd instead.
TEST(EstimatorThreads, SlowBackendKeepsEveryScan) {
  const BoxWorldParams p;
  const fs::path dir = test_dir();
  const Config cfg = threaded(box_config(p, false, true, dir));
  const auto written = test::write_box_dataset(dir / "data", p);
  ASSERT_TRUE(written.is_ok()) << written.reason();
  detail::CoreTestHooks hooks;
  hooks.before_backend_keyframe = [](std::uint64_t n_handled) {
    if (n_handled == 0) {
      return;  // keyframe 0 at once: the map exists from the start
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  };
  const HooksScope scope(&hooks);
  const ThreadedRun thr = replay_threaded(dir / "data", cfg, 4.0);
  print_run("slow backend", p, thr);
  const Estimator::Stats& s = thr.r.stats;
  EXPECT_EQ(thr.r.estimates.size(), expected_estimates(p, cfg));
  EXPECT_EQ(s.n_scans, static_cast<std::uint64_t>(test::box_num_scans(p)));
  EXPECT_GT(s.n_keyframes, 5U);
  EXPECT_EQ(s.n_backend_updates_dropped, 0U);
  EXPECT_LT(trajectory_errors(p, thr.r.estimates).ate_m, 0.2);
  // The front-end never waits for the backend: no run_once() lasts one backend keyframe.
  EXPECT_LT(thr.max_run_once_ms, 400.0);
}

// start() / stop() without data, twice, and an Estimator destroyed while its thread runs.
TEST(EstimatorThreads, StartStopJoinCleanly) {
  const BoxWorldParams p;
  const Config cfg = threaded(box_config(p, false, false, test_dir()));
  {
    Estimator est(cfg);
    est.start();
    est.start();  // already running: no second thread
    est.stop();
    est.stop();
    est.start();
    est.stop();
    EXPECT_EQ(est.stats().n_keyframes, 0U);
  }
  {
    Estimator est(cfg);
    est.start();  // the destructor joins
  }
}

}  // namespace
}  // namespace mss::pipeline
