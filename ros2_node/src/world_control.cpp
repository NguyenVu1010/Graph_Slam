#include "mss_ros2/world_control.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

extern char** environ;  // NOLINT(readability-redundant-declaration): POSIX, for posix_spawnp

namespace mss_ros2 {

namespace {

constexpr int kRunPollSliceMs = 20;  // wall ms between polls in the blocking run_command()

bool valid_world_name(const std::string& w) {
  for (const char c : w) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-';
    if (!ok) {
      return false;
    }
  }
  return true;
}

// The process a PosixCommand runs: output read from a non-blocking pipe, exit collected with
// waitpid(WNOHANG), SIGKILL once over the wall deadline.
class PosixCommand final : public PendingCommand {
 public:
  PosixCommand(const std::vector<std::string>& argv, double limit_s)
      : deadline_(std::chrono::steady_clock::now() +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(limit_s))) {
    if (argv.empty()) {
      done_ = true;
      return;
    }
    int fds[2];
    // Close-on-exec: neither end leaks into gz or into any other child (review N-5); dup2 in the
    // file actions clears the flag on the child's 1 and 2. Non-blocking on the parent's read end
    // only: the flag belongs to the open file description, so on the write end it would reach
    // the child's stdout and stderr (review N2-4).
    if (pipe2(fds, O_CLOEXEC) != 0) {
      done_ = true;
      return;
    }
    const int fl = fcntl(fds[0], F_GETFL);
    if (fl < 0 || fcntl(fds[0], F_SETFL, fl | O_NONBLOCK) != 0) {
      close(fds[0]);
      close(fds[1]);
      done_ = true;
      return;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);
    // Every other descriptor of the node (DDS sockets that rmw may not mark close-on-exec) is
    // closed in the child (glibc 2.34+).
    posix_spawn_file_actions_addclosefrom_np(&actions, STDERR_FILENO + 1);
    std::vector<char*> args;
    args.reserve(argv.size() + 1);
    for (const std::string& a : argv) {
      args.push_back(const_cast<char*>(a.c_str()));  // posix_spawnp does not modify them
    }
    args.push_back(nullptr);
    const int err = posix_spawnp(&pid_, args[0], &actions, nullptr, args.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(fds[1]);
    if (err != 0) {
      close(fds[0]);
      done_ = true;
      return;
    }
    fd_ = fds[0];
    result_.started = true;
  }

  ~PosixCommand() override {
    if (!done_ && pid_ > 0) {  // dropped while running: kill and reap, never leave a zombie
      kill(pid_, SIGKILL);
      int status = 0;
      while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {
      }
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }
  PosixCommand(const PosixCommand&) = delete;
  PosixCommand& operator=(const PosixCommand&) = delete;

  std::optional<CommandResult> poll() override {
    if (done_) {
      return result_;
    }
    drain();
    int status = 0;
    const pid_t w = waitpid(pid_, &status, WNOHANG);
    if (w == pid_) {
      drain();  // what it wrote just before exiting
      if (!result_.timed_out && WIFEXITED(status)) {
        result_.exit_code = WEXITSTATUS(status);
      }
      done_ = true;
      return result_;
    }
    if (w < 0 && errno != EINTR) {
      done_ = true;  // not our child any more (should not happen)
      return result_;
    }
    if (!result_.timed_out && std::chrono::steady_clock::now() >= deadline_) {
      result_.timed_out = true;
      kill(pid_, SIGKILL);  // reaped by a later poll()
    }
    return std::nullopt;
  }

 private:
  void drain() {
    if (fd_ < 0) {
      return;
    }
    char buf[256];
    for (;;) {
      const ssize_t got = read(fd_, buf, sizeof(buf));
      if (got > 0) {
        const std::size_t room = kMaxCommandOutput - result_.output.size();
        result_.output.append(buf, std::min(room, static_cast<std::size_t>(got)));
      } else if (got < 0 && errno == EINTR) {
        continue;
      } else {
        return;  // EAGAIN (nothing now) or EOF or error
      }
    }
  }

  std::chrono::steady_clock::time_point deadline_;
  pid_t pid_ = -1;
  int fd_ = -1;
  bool done_ = false;
  CommandResult result_;
};

}  // namespace

std::unique_ptr<PendingCommand> start_command(const std::vector<std::string>& argv,
                                              double limit_s) {
  return std::make_unique<PosixCommand>(argv, limit_s);
}

CommandResult run_command(const std::vector<std::string>& argv, double limit_s) {
  const std::unique_ptr<PendingCommand> c = start_command(argv, limit_s);
  for (;;) {
    if (std::optional<CommandResult> r = c->poll()) {
      return *r;
    }
    ::poll(nullptr, 0, kRunPollSliceMs);
  }
}

WorldControl::WorldControl(std::string world, int timeout_ms, CommandStarter starter)
    : world_(std::move(world)), timeout_ms_(timeout_ms), starter_(std::move(starter)) {
  if (!valid_world_name(world_)) {
    throw std::invalid_argument("viz.gz_world: '" + world_ +
                                "' is not a Gazebo world name (letters, digits, '_', '-')");
  }
  if (timeout_ms_ <= 0) {
    throw std::invalid_argument("viz.gz_timeout_ms: must be > 0");
  }
  if (!starter_) {
    throw std::invalid_argument("WorldControl: no command starter");
  }
}

double WorldControl::limit_s() const {
  return static_cast<double>(timeout_ms_) / 1000.0 + kProcessMarginS;
}

std::vector<std::string> WorldControl::command(bool paused) const {
  return {"gz",
          "service",
          "-s",
          "/world/" + world_ + "/control",
          "--reqtype",
          "gz.msgs.WorldControl",
          "--reptype",
          "gz.msgs.Boolean",
          "--timeout",
          std::to_string(timeout_ms_),
          "--req",
          paused ? "pause: true" : "pause: false"};
}

mss::common::Status WorldControl::begin(bool paused) {
  if (world_.empty()) {
    return mss::common::Status::error("no Gazebo world configured (viz.gz_world is empty)");
  }
  if (pending_ != nullptr) {
    return mss::common::Status::error("a world control request is still in flight");
  }
  pending_ = starter_(command(paused), limit_s());
  if (pending_ == nullptr) {
    return mss::common::Status::error("gz could not be started");
  }
  return mss::common::Status::ok();
}

std::optional<mss::common::Status> WorldControl::poll() {
  if (pending_ == nullptr) {
    return std::nullopt;
  }
  const std::optional<CommandResult> r = pending_->poll();
  if (!r) {
    return std::nullopt;
  }
  pending_.reset();
  return interpret(*r);
}

mss::common::Status WorldControl::interpret(const CommandResult& r) const {
  if (!r.started) {
    return mss::common::Status::error("gz could not be started (no gz on PATH?)");
  }
  if (r.timed_out) {
    return mss::common::Status::error("gz service /world/" + world_ + "/control killed after " +
                                      std::to_string(limit_s()) + " s");
  }
  // gz prints the Boolean reply as "data: true"; the request text ("pause: true") or a warning
  // that happens to contain "true" does not count (review N-4).
  if (r.exit_code != 0 || r.output.find("data: true") == std::string::npos) {
    return mss::common::Status::error("gz service /world/" + world_ + "/control failed (exit " +
                                      std::to_string(r.exit_code) + "): " + r.output);
  }
  return mss::common::Status::ok();
}

SimClockWatch::SimClockWatch(double still_s, double min_rtf)
    : still_s_(still_s), min_rtf_(min_rtf) {
  if (!(still_s_ > 0.0)) {
    throw std::invalid_argument("viz.paused_after_s: must be > 0");
  }
  if (!(min_rtf_ > 0.0 && min_rtf_ <= 1.0)) {
    throw std::invalid_argument("viz.running_min_rtf: must be in (0, 1]");
  }
}

void SimClockWatch::update(std::int64_t sim_ns, double wall_s) {
  if (!seen_) {
    seen_ = true;
    last_sim_ns_ = sim_ns;
    last_change_wall_s_ = wall_s;
    last_rtf_ = 0.0;  // unknown until the clock moves once more
    return;
  }
  if (sim_ns == last_sim_ns_) {
    return;
  }
  const double dwall = wall_s - last_change_wall_s_;
  const double dsim = 1e-9 * static_cast<double>(sim_ns - last_sim_ns_);  // dt only
  last_rtf_ = dwall > 0.0 ? dsim / dwall : 0.0;
  last_sim_ns_ = sim_ns;
  last_change_wall_s_ = wall_s;
  const bool advancing = last_rtf_ >= min_rtf_;
  if (hint_paused_ && wall_s > hint_wall_s_) {
    if (!advancing) {
      run_started_ = false;  // a stop or a crawl restarts the run
    } else if (!run_started_) {
      run_started_ = true;
      run_start_wall_s_ = wall_s;
    } else if (wall_s - run_start_wall_s_ >= still_s_) {
      hint_paused_ = false;  // a sustained run: the world runs again (resumed from outside)
      run_started_ = false;
    }
  }
  if (hint_running_ && advancing) {
    hint_running_ = false;  // confirmed by the clock: the plain rule says running now
  }
}

bool SimClockWatch::paused(double wall_s) const {
  if (hint_paused_) {
    return true;
  }
  if (hint_running_) {
    return wall_s - std::max(hint_wall_s_, last_change_wall_s_) > still_s_;
  }
  return !seen_ || wall_s - last_change_wall_s_ > still_s_ || last_rtf_ < min_rtf_;
}

void SimClockWatch::note_paused(double wall_s) {
  hint_paused_ = true;
  hint_running_ = false;
  run_started_ = false;
  hint_wall_s_ = wall_s;
}

void SimClockWatch::note_running(double wall_s) {
  hint_running_ = true;
  hint_paused_ = false;
  hint_wall_s_ = wall_s;
}

}  // namespace mss_ros2
