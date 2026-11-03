// Known-answer tests of the /mss/save writer and the Gazebo world control (ADR-031).
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "mss/pipeline/graph_snapshot.hpp"

#include "mss_ros2/run_saver.hpp"
#include "mss_ros2/world_control.hpp"

namespace {

namespace fs = std::filesystem;
using mss::common::Timestamp;
using mss::pipeline::LoopState;

constexpr std::int64_t kT0Ns = 1700000000LL * 1000000000LL + 5;  // 1700000000.000000005 s

std::string read_file(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

// Unique per process (review N-1): parallel builds on one machine must not share the directory.
fs::path fresh_dir(const std::string& name) {
  const fs::path d =
      fs::temp_directory_path() / ("mss_run_saver_" + std::to_string(::getpid()) + "_" + name);
  fs::remove_all(d);
  return d;
}

mss::pipeline::GraphSnapshot two_keyframes() {
  mss::pipeline::GraphSnapshot s;
  for (int k = 0; k < 2; ++k) {
    mss::pipeline::GraphKeyframe g;
    g.id = static_cast<mss::common::KeyId>(k);
    g.t = Timestamp{kT0Ns + k * 500000000LL};
    g.T_w_b.p = mss::common::Vec3(1.5 * k, -2.0, 0.25);
    s.keyframes.push_back(g);
  }
  s.loops.push_back({0, 1, s.keyframes[0].t, s.keyframes[1].t, LoopState::kReleased});
  s.loops.push_back({0, 1, s.keyframes[0].t, s.keyframes[1].t, LoopState::kRemoved});
  return s;
}

TEST(RunSaver, TumLineFromIntegerNanoseconds) {
  mss::common::Pose T;
  T.p = mss::common::Vec3(1.0, -2.5, 0.125);
  EXPECT_EQ(mss_ros2::tum_line(Timestamp{kT0Ns}, T), "1700000000.000000005 1 -2.5 0.125 0 0 0 1\n");
  EXPECT_EQ(mss_ros2::tum_line(Timestamp{-1}, T).substr(0, 13), "-0.000000001 ");
}

TEST(RunSaver, LoopsCsv) {
  EXPECT_EQ(mss_ros2::loops_csv(two_keyframes()),
            "i,j,t_i_s,t_j_s,state\n"
            "0,1,1700000000.000000005,1700000000.500000005,released\n"
            "0,1,1700000000.000000005,1700000000.500000005,removed\n");
  EXPECT_EQ(mss_ros2::loops_csv({}), "i,j,t_i_s,t_j_s,state\n");
}

TEST(RunSaver, WritesTheThreeFilesIntoANewDirectory) {
  const fs::path dir = fresh_dir("three") / "results" / "run-1";
  const std::vector<Eigen::Vector3f> map = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  const auto r = mss_ros2::save_run(dir, two_keyframes(), map);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().dir, dir);
  EXPECT_EQ(r.value().n_keyframes, 2U);
  EXPECT_EQ(r.value().n_loops, 2U);
  EXPECT_EQ(r.value().n_points, 2U);
  std::vector<std::string> names;
  for (const auto& e : fs::directory_iterator(dir)) {
    names.push_back(e.path().filename().string());
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, (std::vector<std::string>{"kf_opt.tum", "loops.csv", "map.pcd"}));  // no .tmp
  const std::string tum = read_file(dir / "kf_opt.tum");
  const std::string header =
      "# kf_opt.tum from /mss/save: every keyframe of the backend graph at its optimised T_w_b\n"
      "# (mss_bench's kf_opt.tum lists only the keyframes still in the map store)\n"
      "# t x y z qx qy qz qw\n";
  EXPECT_EQ(tum,
            header +
                "1700000000.000000005 0 -2 0.25 0 0 0 1\n"
                "1700000000.500000005 1.5 -2 0.25 0 0 0 1\n");
  EXPECT_EQ(read_file(dir / "loops.csv"), mss_ros2::loops_csv(two_keyframes()));
  const std::string pcd = read_file(dir / "map.pcd");
  EXPECT_NE(pcd.find("POINTS 2\n"), std::string::npos);
  EXPECT_NE(pcd.find("DATA binary\n"), std::string::npos);

  // A second save overwrites the same files (an empty graph: header-only files).
  const auto r2 = mss_ros2::save_run(dir, {}, {});
  ASSERT_TRUE(r2.is_ok()) << r2.reason();
  EXPECT_EQ(read_file(dir / "kf_opt.tum"), header);
  EXPECT_NE(read_file(dir / "map.pcd").find("POINTS 0\n"), std::string::npos);
  fs::remove_all(fresh_dir("three"));
}

TEST(RunSaver, ErrorWhenTheDirectoryCannotBeCreated) {
  const fs::path base = fresh_dir("blocked");
  fs::create_directories(base);
  std::ofstream(base / "file") << "x";
  const auto r = mss_ros2::save_run(base / "file" / "run", {}, {});
  EXPECT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("cannot create directory"), std::string::npos) << r.reason();
  fs::remove_all(base);
}

// --- WorldControl with a mock gz ------------------------------------------------------------

// A pending command that finishes after `polls_left` polls with `reply` (0: at the first poll).
class FakePending final : public mss_ros2::PendingCommand {
 public:
  FakePending(mss_ros2::CommandResult reply, int polls_left)
      : reply_(std::move(reply)), polls_left_(polls_left) {}
  std::optional<mss_ros2::CommandResult> poll() override {
    if (polls_left_-- > 0) {
      return std::nullopt;
    }
    return reply_;
  }

 private:
  mss_ros2::CommandResult reply_;
  int polls_left_;
};

struct FakeGz {
  std::vector<std::vector<std::string>> calls;
  mss_ros2::CommandResult reply;
  int polls = 0;  // polls before the reply
  mss_ros2::CommandStarter starter() {
    return [this](const std::vector<std::string>& argv, double /*limit_s*/) {
      calls.push_back(argv);
      return std::make_unique<FakePending>(reply, polls);
    };
  }
};

// Polls until the request in flight finishes (at most `max_polls`).
std::optional<mss::common::Status> finish(mss_ros2::WorldControl& w, int max_polls = 10) {
  for (int k = 0; k < max_polls; ++k) {
    if (std::optional<mss::common::Status> st = w.poll()) {
      return st;
    }
  }
  return std::nullopt;
}

TEST(WorldControl, PauseCallsTheWorldControlServiceWithoutBlocking) {
  FakeGz gz;
  gz.reply = {true, false, 0, "data: true\n"};
  gz.polls = 3;
  mss_ros2::WorldControl w("warehouse", 3000, gz.starter());
  ASSERT_TRUE(w.begin(true).is_ok());
  EXPECT_TRUE(w.busy());
  EXPECT_FALSE(w.begin(false).is_ok()) << "one request at a time";
  EXPECT_FALSE(w.poll().has_value());  // still running: poll() returns at once
  const auto st = finish(w);
  ASSERT_TRUE(st.has_value());
  EXPECT_TRUE(st->is_ok()) << st->reason();
  EXPECT_FALSE(w.busy());
  ASSERT_TRUE(w.begin(false).is_ok());
  ASSERT_TRUE(finish(w).has_value());
  ASSERT_EQ(gz.calls.size(), 2U);
  const std::vector<std::string> want = {"gz",
                                         "service",
                                         "-s",
                                         "/world/warehouse/control",
                                         "--reqtype",
                                         "gz.msgs.WorldControl",
                                         "--reptype",
                                         "gz.msgs.Boolean",
                                         "--timeout",
                                         "3000",
                                         "--req",
                                         "pause: true"};
  EXPECT_EQ(gz.calls[0], want);
  EXPECT_EQ(gz.calls[1].back(), "pause: false");
  EXPECT_DOUBLE_EQ(w.limit_s(), 3.0 + mss_ros2::WorldControl::kProcessMarginS);
}

TEST(WorldControl, FailuresGiveAReason) {
  FakeGz gz;
  mss_ros2::WorldControl w("outdoor", 1000, gz.starter());
  const auto reason = [&](mss_ros2::CommandResult r) {
    gz.reply = std::move(r);
    EXPECT_TRUE(w.begin(true).is_ok());
    const auto st = finish(w);
    return st ? st->reason() : std::string("(no result)");
  };
  EXPECT_NE(reason({false, false, -1, ""}).find("could not be started"), std::string::npos);
  EXPECT_NE(reason({true, true, -1, ""}).find("killed"), std::string::npos);
  EXPECT_NE(reason({true, false, 0, "data: false\n"}).find("failed"), std::string::npos);
  EXPECT_NE(reason({true, false, 1, "Service call timed out\n"}).find("timed out"),
            std::string::npos);
  // Review N-4: only the reply "data: true" counts, not "true" anywhere in the output.
  EXPECT_NE(reason({true, false, 0, "req: pause: true\ndata: false\n"}).find("failed"),
            std::string::npos);
  EXPECT_NE(reason({true, false, 0, "warning: use_true_time\n"}).find("failed"), std::string::npos);

  mss_ros2::WorldControl none("", 1000, gz.starter());
  EXPECT_FALSE(none.configured());
  const std::size_t n = gz.calls.size();
  EXPECT_FALSE(none.begin(true).is_ok());
  EXPECT_EQ(gz.calls.size(), n);  // no world: gz is not called
}

TEST(WorldControl, RejectsBadNamesAndTimeouts) {
  EXPECT_THROW(mss_ros2::WorldControl("a/b", 1000), std::invalid_argument);
  EXPECT_THROW(mss_ros2::WorldControl("w; rm -rf /", 1000), std::invalid_argument);
  EXPECT_THROW(mss_ros2::WorldControl("w", 0), std::invalid_argument);
}

// The real starter: exit status and output of a POSIX command, a missing binary, the time limit,
// and poll() never blocking while the child runs.
TEST(WorldControl, StartCommandRealProcess) {
  const auto ok = mss_ros2::run_command({"sh", "-c", "echo data: true"}, 5.0);
  EXPECT_TRUE(ok.started);
  EXPECT_FALSE(ok.timed_out);
  EXPECT_EQ(ok.exit_code, 0);
  EXPECT_EQ(ok.output, "data: true\n");
  const auto bad = mss_ros2::run_command({"sh", "-c", "echo oops >&2; exit 3"}, 5.0);
  EXPECT_EQ(bad.exit_code, 3);
  EXPECT_EQ(bad.output, "oops\n");
  const auto missing = mss_ros2::run_command({"mss-no-such-binary-xyz"}, 5.0);
  EXPECT_FALSE(missing.started && missing.exit_code == 0);
  const auto slow = mss_ros2::run_command({"sleep", "10"}, 0.3);
  EXPECT_TRUE(slow.timed_out);

  const auto t0 = std::chrono::steady_clock::now();
  const auto running = mss_ros2::start_command({"sleep", "5"}, 10.0);
  EXPECT_FALSE(running->poll().has_value());
  const double poll_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(poll_s, 1.0) << "poll() must not wait for the child";
  // Dropping a running command kills and reaps it (no zombie, no hang).
}

// The output pipe reaches the child only as its stdout / stderr: no other descriptor of the child
// refers to it (pipe2 O_CLOEXEC, review N-5).
TEST(WorldControl, ChildDoesNotInheritThePipe) {
  const auto r = mss_ros2::run_command(
      {"sh",
       "-c",
       "a=$(readlink /proc/$$/fd/1); for f in /proc/$$/fd/*; do n=${f##*/}; "
       "if [ \"$n\" -gt 2 ] && [ \"$(readlink $f)\" = \"$a\" ]; then echo LEAK $n; fi; done; "
       "echo checked"},
      5.0);
  ASSERT_EQ(r.exit_code, 0) << r.output;
  EXPECT_EQ(r.output, "checked\n");
}

// --- SimClockWatch (review M-1) ----------------------------------------------------------

TEST(SimClockWatch, PausedUntilTheClockRunsInRealTime) {
  mss_ros2::SimClockWatch w(0.5, 0.05);
  EXPECT_TRUE(w.paused(0.0));  // nothing seen yet
  constexpr std::int64_t kMs = 1000000;
  w.update(1000 * kMs, 10.0);
  EXPECT_TRUE(w.paused(10.0));  // one sample: rate unknown
  w.update(1100 * kMs, 10.1);   // 0.1 sim s in 0.1 wall s: running
  EXPECT_FALSE(w.paused(10.2));
  w.update(1100 * kMs, 10.3);  // unchanged
  EXPECT_FALSE(w.paused(10.6));
  EXPECT_TRUE(w.paused(10.7));  // 0.6 s without a change: paused (externally, e.g. the GUI)
  w.update(1101 * kMs, 10.8);   // a single 1 ms step 0.7 s later: 0.0014 x, still paused
  EXPECT_TRUE(w.paused(10.8));
  w.update(1151 * kMs, 10.85);  // resumed: 50 ms in 50 ms
  EXPECT_FALSE(w.paused(10.9));
  EXPECT_THROW(mss_ros2::SimClockWatch(0.0, 0.05), std::invalid_argument);
  EXPECT_THROW(mss_ros2::SimClockWatch(0.5, 0.0), std::invalid_argument);
}

// The child's stdout is a blocking descriptor (only the parent's read end is non-blocking,
// review N2-4), and a descriptor the parent opened without close-on-exec does not reach the child
// (closefrom, review N-5).
TEST(WorldControl, ChildStdoutBlockingAndNoInheritedDescriptors) {
  const auto flags = mss_ros2::run_command(
      {"sh", "-c", "grep ^flags /proc/$$/fdinfo/1 | tr -s ' \t' ' ' | cut -d' ' -f2"}, 5.0);
  ASSERT_EQ(flags.exit_code, 0) << flags.output;
  const long f = std::stol(flags.output, nullptr, 8);  // octal open flags
  EXPECT_EQ(f & O_NONBLOCK, 0) << flags.output;

  const int leaky = ::open("/dev/null", O_RDONLY);  // no O_CLOEXEC on purpose
  ASSERT_GE(leaky, 0);
  const auto r = mss_ros2::run_command(
      {"sh",
       "-c",
       "for f in /proc/$$/fd/*; do n=${f##*/}; "
       "if [ \"$n\" -gt 2 ] && [ \"$(readlink $f)\" = /dev/null ]; then echo LEAK $n; fi; done; "
       "echo checked"},
      5.0);
  ::close(leaky);
  ASSERT_EQ(r.exit_code, 0) << r.output;
  EXPECT_EQ(r.output, "checked\n");
}

// Review M-3: the node's own successful gz requests update the state at once.
TEST(SimClockWatch, OwnPauseAndResumeTakeEffectAtOnce) {
  constexpr std::int64_t kMs = 1000000;
  mss_ros2::SimClockWatch w(0.5, 0.05);
  w.update(1000 * kMs, 10.0);
  w.update(1100 * kMs, 10.1);
  ASSERT_FALSE(w.paused(10.1));  // running
  w.note_paused(10.15);          // /mss/pause true answered
  EXPECT_TRUE(w.paused(10.15));  // at once, not 0.5 s later
  w.update(1150 * kMs, 10.2);    // one trailing step of the world after the pause
  EXPECT_TRUE(w.paused(10.2));
  EXPECT_TRUE(w.paused(20.0));  // stays paused while the clock stands
  w.note_running(20.0);         // /mss/pause false answered
  EXPECT_FALSE(w.paused(20.0));
  w.update(1151 * kMs, 20.05);  // first change after the long stop: rate near 0, still running
  EXPECT_FALSE(w.paused(20.1));
  w.update(1201 * kMs, 20.1);  // advancing in real time: the plain rule takes over
  EXPECT_FALSE(w.paused(20.2));
  EXPECT_TRUE(w.paused(20.7));  // then paused from outside (GUI): seen after still_s

  // A note_paused() hint is cleared only by a sustained run of the clock over still_s (the
  // world resumed from outside), sampled every 50 ms at real time.
  mss_ros2::SimClockWatch v(0.5, 0.05);
  v.update(0, 1.0);
  v.note_paused(1.0);
  for (int k = 1; k <= 10; ++k) {  // 1.05 .. 1.5 s: the run spans 0.45 s < still_s
    v.update(50 * k * kMs, 1.0 + 0.05 * k);
    EXPECT_TRUE(v.paused(1.0 + 0.05 * k)) << k;
  }
  v.update(600 * kMs, 1.6);  // the run spans 0.55 s >= still_s: running again
  EXPECT_FALSE(v.paused(1.6));
  // A note_running() hint lapses after still_s without any change.
  v.note_running(5.0);
  EXPECT_FALSE(v.paused(5.4));
  EXPECT_TRUE(v.paused(5.6));
  EXPECT_THROW(mss_ros2::SimClockWatch(0.5, 1.5), std::invalid_argument);
}

// Review N3-1: after the node's own pause, /clock samples that arrive late (delayed by load in the
// bridge or DDS), each looking like an advance, do not clear "paused"; only a run over the full
// still_s does.
TEST(SimClockWatch, LateClockSamplesAfterOwnPauseKeepItPaused) {
  constexpr std::int64_t kMs = 1000000;
  mss_ros2::SimClockWatch w(0.5, 0.05);
  w.update(1000 * kMs, 10.0);
  w.update(1050 * kMs, 10.05);
  ASSERT_FALSE(w.paused(10.05));
  w.note_paused(10.1);
  // Two trailing samples from before the pause, delivered 0.1 and 0.15 s late, both advancing
  // at real time.
  w.update(1100 * kMs, 10.2);
  w.update(1150 * kMs, 10.25);
  EXPECT_TRUE(w.paused(10.25));
  EXPECT_TRUE(w.paused(12.0));  // and the clock stands afterwards: still paused
  // A crawl (single steps) after the note does not count either.
  w.update(1151 * kMs, 12.0);
  w.update(1152 * kMs, 12.5);
  EXPECT_TRUE(w.paused(12.5));
}

// Review N2-3: a save never reuses an existing directory, also after a restart with the same run
// id (the numbering starts from what is on disk).
TEST(RunSaver, NewSaveDirectoryNeverReusesOne) {
  const fs::path run = fresh_dir("numbering") / "run-x";
  fs::create_directories(run / "save-001");
  fs::create_directories(run / "save-003");
  const auto a = mss_ros2::new_save_dir(run, 1);
  ASSERT_TRUE(a.is_ok()) << a.reason();
  EXPECT_EQ(a.value().second, 2);
  EXPECT_EQ(a.value().first, run / "save-002");
  EXPECT_TRUE(fs::is_directory(run / "save-002"));
  const auto b = mss_ros2::new_save_dir(run, 3);
  ASSERT_TRUE(b.is_ok()) << b.reason();
  EXPECT_EQ(b.value().first, run / "save-004");
  // Review N3-2: a regular file of that name is skipped too, not an error.
  std::ofstream(run / "save-005") << "not a directory";
  const auto c = mss_ros2::new_save_dir(run, 5);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_EQ(c.value().first, run / "save-006");
  EXPECT_EQ(c.value().second, 6);
  EXPECT_TRUE(fs::is_regular_file(run / "save-005"));  // left untouched
  fs::remove_all(fresh_dir("numbering"));
}

}  // namespace
