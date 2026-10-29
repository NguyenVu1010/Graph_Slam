"""Clock checks of ``mss_sim.live_start``, without ROS so pytest covers them.

A live run must not report success on a clock that is not its own: a Gazebo server left over
from an earlier run, or another one on the same domain, already has a sim time far from 0, and
a frozen server never advances. ``live_start`` exits non-zero in both cases. ``run_live`` is the
whole start/stop sequence against a node protocol, so tests drive it with a fake node.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

POLL_S = 0.1  # wall s between checks


def start_clock_error(sim_s: float, max_start_s: float) -> str | None:
    """Reason to refuse the run when the sim clock at start is not near 0, else ``None``.

    ``sim_s`` is read after the paused world was stepped until the robot exists, so a fresh
    server is a few ms in; anything above ``max_start_s`` is someone else's clock.
    """
    if not 0.0 <= sim_s <= max_start_s:
        return (
            f"sim clock at start is {sim_s:.3f} s, not within [0, {max_start_s}] s: another or a "
            "left-over Gazebo publishes /clock"
        )
    return None


@dataclass
class StallWatch:
    """Fails when the sim clock does not advance for ``timeout_s`` of wall time, or goes back.

    ``update(wall_s, sim_s)`` returns ``None`` while the clock is healthy, else the reason.
    """

    timeout_s: float
    wall_s: float  # wall time of the last advance, s
    sim_s: float  # sim time at the last advance, s

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            msg = f"timeout_s must be > 0, got {self.timeout_s}"
            raise ValueError(msg)

    def update(self, wall_s: float, sim_s: float) -> str | None:
        if sim_s < self.sim_s:
            return f"sim clock went back from {self.sim_s:.3f} s to {sim_s:.3f} s (two clocks?)"
        if sim_s > self.sim_s:
            self.sim_s, self.wall_s = sim_s, wall_s
            return None
        if wall_s - self.wall_s > self.timeout_s:
            return (
                f"sim clock stuck at {sim_s:.3f} s for {wall_s - self.wall_s:.1f} wall s "
                f"(timeout {self.timeout_s} s)"
            )
        return None


class LiveNode(Protocol):
    """What ``run_live`` needs from the rclpy node (faked in tests)."""

    n_gt: int  # /ground_truth messages received

    def count_subscribers(self, topic: str) -> int: ...
    def spin_for(self, seconds: float) -> None: ...
    def sim_s(self) -> float: ...
    def info(self, msg: str) -> None: ...
    def error(self, msg: str) -> None: ...


WorldCall = Callable[[str, str, str], bool]  # (service, reqtype, request) -> ok


def run_live(
    args: argparse.Namespace, node: LiveNode, world_call: WorldCall, now: Callable[[], float]
) -> int:
    """The start/stop sequence of ``live_start``; 0 on a full run, 2 on any failure.

    ``args``: ``wait_subs``, ``wait_timeout_s``, ``step_s``, ``rtf``, ``max_start_clock_s``,
    ``stall_timeout_s``, ``stop_at_s``. ``now`` is the wall clock in seconds.
    """
    deadline = now() + args.wait_timeout_s
    while missing := [t for t in args.wait_subs if node.count_subscribers(t) == 0]:
        if now() > deadline:
            node.error(f"live_start: no subscriber on {missing} after {args.wait_timeout_s} s")
            return 2
        node.spin_for(POLL_S)
    physics = f"max_step_size: {args.step_s}, real_time_factor: {args.rtf}"
    if not world_call("set_physics", "gz.msgs.Physics", physics):
        node.error(f"live_start: set_physics ({physics}) failed")
        return 2
    n_steps = 0
    while node.n_gt == 0:
        if now() > deadline:
            node.error("live_start: no /ground_truth while stepping the paused world")
            return 2
        world_call("control", "gz.msgs.WorldControl", "multi_step: 1")
        n_steps += 1
        node.spin_for(POLL_S)
    if reason := start_clock_error(node.sim_s(), args.max_start_clock_s):
        node.error(f"live_start: {reason}")
        return 2
    if not world_call("control", "gz.msgs.WorldControl", "pause: false"):
        node.error("live_start: unpause failed")
        return 2
    sim0, wall0 = node.sim_s(), now()
    watch = StallWatch(args.stall_timeout_s, wall0, sim0)
    node.info(
        f"live_start: running at real-time factor {args.rtf} after {n_steps} start-up steps; "
        f"pause at sim t = {args.stop_at_s} s"
    )
    while (sim_s := node.sim_s()) < args.stop_at_s:
        if reason := watch.update(now(), sim_s):
            node.error(f"live_start: {reason}")
            world_call("control", "gz.msgs.WorldControl", "pause: true")
            return 2
        node.spin_for(POLL_S)
    world_call("control", "gz.msgs.WorldControl", "pause: true")
    sim_s, wall_s = node.sim_s() - sim0, now() - wall0
    node.info(
        f"live_start: done, {sim_s:.1f} sim s in {wall_s:.1f} wall s, "
        f"real-time factor {sim_s / wall_s:.3f} (requested {args.rtf})"
    )
    return 0
