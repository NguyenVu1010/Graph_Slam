#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mss/common/result.hpp"

// Pause / resume of the Gazebo world for /mss/pause and /mss/save (viz.enabled, ADR-031), through
// the `gz service` CLI as sim/mss_sim/gz_cli.py does (no ros_gz_interfaces dependency). The gz
// process runs asynchronously: the caller starts it and polls it from a timer, so the node's
// single executor never waits on it (review M-2).
namespace mss_ros2 {

/// Outcome of one external command.
struct CommandResult {
  bool started = false;    ///< the process could be spawned (false: e.g. no `gz` on PATH)
  bool timed_out = false;  ///< killed after the wall limit
  int exit_code = -1;      ///< exit status when it exited normally, else -1
  std::string output;      ///< stdout and stderr, at most kMaxCommandOutput bytes
};

/// Bytes of command output kept (the gz reply is a few bytes).
constexpr std::size_t kMaxCommandOutput = 4096;

/// A command that was started and has not been collected yet.
class PendingCommand {
 public:
  virtual ~PendingCommand() = default;
  /// Never blocks: reads what output is available, checks for exit and for the wall limit
  /// (SIGKILL when over). Returns the result once the process has exited (or could not start),
  /// std::nullopt while it runs. After a result, further calls return the same result.
  virtual std::optional<CommandResult> poll() = 0;
};

/// Starts `argv` (argv[0] looked up on PATH, no shell) with a wall limit of `limit_s` seconds.
/// Never blocks and never throws. A spawn failure gives a PendingCommand whose first poll()
/// returns `started == false`.
using CommandStarter = std::function<std::unique_ptr<PendingCommand>(
    const std::vector<std::string>& argv, double limit_s)>;

/// The default CommandStarter: posix_spawnp, stdout and stderr on a close-on-exec pipe whose
/// read end (the parent's) alone is non-blocking (review N-5, N2-4); every other descriptor of the
/// parent is closed in the child (posix_spawn_file_actions_addclosefrom_np).
std::unique_ptr<PendingCommand> start_command(const std::vector<std::string>& argv, double limit_s);

/// start_command() then poll until done (sleeping between polls). For tests and tools only:
/// it blocks the calling thread.
CommandResult run_command(const std::vector<std::string>& argv, double limit_s);

/// Pauses and resumes one Gazebo world: `gz service -s /world/<world>/control --reqtype
/// gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout <ms> --req "pause: true|false"`.
/// The gz transport partition comes from the environment (GZ_PARTITION), as the launch sets it.
/// One request at a time: begin() then poll() until it returns a status.
class WorldControl {
 public:
  /// Seconds the gz process may run beyond its own --timeout (start-up, discovery), as
  /// gz_cli.PROCESS_MARGIN_S.
  static constexpr double kProcessMarginS = 2.0;

  /// `world`: Gazebo world name (letters, digits, '_' and '-' only; empty: no world, every
  /// request fails with a reason). `timeout_ms` > 0: the gz service request timeout.
  /// Errors: throws std::invalid_argument for a bad name, timeout or starter (initialisation).
  WorldControl(std::string world, int timeout_ms, CommandStarter starter = start_command);

  /// Starts `pause: <paused>`. Errors (nothing started): no world configured, or a request is
  /// still in flight. Never blocks.
  mss::common::Status begin(bool paused);
  /// The status of the request in flight once gz has finished: ok when gz exited 0 and replied
  /// `data: true`; else the reason (gz not found, killed after the limit, non-zero exit, a false
  /// reply, with the gz output). std::nullopt while it runs or when nothing is in flight.
  std::optional<mss::common::Status> poll();
  [[nodiscard]] bool busy() const { return pending_ != nullptr; }

  [[nodiscard]] bool configured() const { return !world_.empty(); }
  [[nodiscard]] const std::string& world() const { return world_; }
  /// Wall seconds a request may take at most (gz timeout + kProcessMarginS).
  [[nodiscard]] double limit_s() const;
  /// The argv sent for `paused` (for logs and tests).
  [[nodiscard]] std::vector<std::string> command(bool paused) const;
  /// Interprets a finished gz call (exposed for tests).
  [[nodiscard]] mss::common::Status interpret(const CommandResult& r) const;

 private:
  std::string world_;
  int timeout_ms_;
  CommandStarter starter_;
  std::unique_ptr<PendingCommand> pending_;
};

/// Whether the simulated world runs, from the sim clock alone (review M-1): the node samples its
/// ROS clock (sim time from /clock) on a wall timer. Running when the sim time changed within the
/// last `still_s` wall seconds and its last advance ran at a real-time factor of at least
/// `min_rtf` (so live_start's 1 ms start-up steps, about 0.01 x, read as paused). Paused before
/// any sample. Pure; tested in test_run_saver.
///
/// The clock alone lags by up to `still_s`. When the node's own gz request succeeds it knows the
/// new state at once (review M-3): note_paused() holds "paused" until the clock, sampled after the
/// note, has kept advancing at >= `min_rtf` on every change over a full `still_s` of wall time
/// (a sustained run, as an external resume gives). Late or trailing /clock samples after the
/// pause, however delayed, span far less than `still_s` and never clear it (review N3-1); a
/// change below `min_rtf` restarts the run. note_running() holds "running" until `still_s`
/// passes without a clock change or an advance at >= `min_rtf` confirms it. A pause or resume
/// from the Gazebo GUI or the gz CLI is still seen only through the clock, after up to `still_s`.
class SimClockWatch {
 public:
  /// Errors: throws std::invalid_argument unless 0 < still_s and 0 < min_rtf <= 1.
  SimClockWatch(double still_s, double min_rtf);
  /// One sample: sim time `sim_ns` read at wall time `wall_s` (steady clock, s).
  void update(std::int64_t sim_ns, double wall_s);
  [[nodiscard]] bool paused(double wall_s) const;
  /// The node's own `pause: true` succeeded at wall time `wall_s`.
  void note_paused(double wall_s);
  /// The node's own `pause: false` succeeded at wall time `wall_s`.
  void note_running(double wall_s);
  /// Whether any sim-clock sample was seen.
  [[nodiscard]] bool seen() const { return seen_; }

 private:
  double still_s_;
  double min_rtf_;
  bool seen_ = false;
  std::int64_t last_sim_ns_ = 0;
  double last_change_wall_s_ = 0.0;
  double last_rtf_ = 0.0;
  bool hint_paused_ = false;
  bool run_started_ = false;  ///< an advancing run after note_paused() is under way
  double run_start_wall_s_ = 0.0;
  bool hint_running_ = false;
  double hint_wall_s_ = 0.0;
};

}  // namespace mss_ros2
