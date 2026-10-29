"""live_start's clock checks: stale clock at start and a stalled run fail (no Gazebo needed)."""

import argparse

import pytest

from mss_sim.live_guard import StallWatch, run_live, start_clock_error


def test_fresh_clock_accepted() -> None:
    assert start_clock_error(0.0, 1.0) is None
    assert start_clock_error(0.012, 1.0) is None


@pytest.mark.parametrize("sim_s", [62.0, 1.001, -0.5])
def test_stale_clock_rejected(sim_s: float) -> None:
    reason = start_clock_error(sim_s, 1.0)
    assert reason is not None and "left-over Gazebo" in reason


def test_advancing_clock_is_healthy() -> None:
    w = StallWatch(timeout_s=10.0, wall_s=0.0, sim_s=0.0)
    for k in range(1, 100):
        assert w.update(wall_s=0.5 * k, sim_s=0.5 * k) is None


def test_stalled_clock_fails_after_timeout() -> None:
    w = StallWatch(timeout_s=10.0, wall_s=0.0, sim_s=5.0)
    assert w.update(wall_s=9.9, sim_s=5.0) is None
    reason = w.update(wall_s=10.1, sim_s=5.0)
    assert reason is not None and "stuck" in reason


def test_clock_going_back_fails() -> None:
    w = StallWatch(timeout_s=10.0, wall_s=0.0, sim_s=5.0)
    reason = w.update(wall_s=1.0, sim_s=4.0)
    assert reason is not None and "back" in reason


def test_timeout_must_be_positive() -> None:
    with pytest.raises(ValueError, match="timeout_s"):
        StallWatch(timeout_s=0.0, wall_s=0.0, sim_s=0.0)


class _FakeNode:
    """Wall clock advances by each spin; the sim clock follows ``sim_of_wall``."""

    def __init__(self, sim_of_wall, gt_after_steps: int = 3) -> None:  # noqa: ANN001
        self.wall = 0.0
        self.sim_of_wall = sim_of_wall
        self.n_gt = 0
        self._gt_after = gt_after_steps
        self.errors: list[str] = []
        self.calls: list[str] = []

    def count_subscribers(self, topic: str) -> int:
        return 1

    def spin_for(self, seconds: float) -> None:
        self.wall += seconds

    def sim_s(self) -> float:
        return self.sim_of_wall(self.wall)

    def info(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        self.errors.append(msg)

    def world_call(self, service: str, reqtype: str, req: str) -> bool:
        self.calls.append(req)
        if req == "multi_step: 1" and len(self.calls) > self._gt_after:
            self.n_gt += 1
        return True


def _args(**kw: float) -> argparse.Namespace:
    base = {
        "wait_subs": ["/lidar/points"],
        "wait_timeout_s": 120.0,
        "step_s": 0.001,
        "rtf": 1.0,
        "max_start_clock_s": 1.0,
        "stall_timeout_s": 10.0,
        "stop_at_s": 62.0,
    }
    return argparse.Namespace(**{**base, **kw})


def _run(node: _FakeNode, **kw: float) -> int:
    return run_live(_args(**kw), node, node.world_call, lambda: node.wall)


def test_run_live_full_run_exits_0() -> None:
    node = _FakeNode(lambda w: max(0.0, w - 1.0))  # starts at 0, then real time
    assert _run(node) == 0
    assert node.calls[-1] == "pause: true"
    assert not node.errors


def test_run_live_stale_clock_exits_2_without_unpausing() -> None:
    """A left-over server's clock (62 s) is refused before anything runs."""
    node = _FakeNode(lambda w: 62.0 + w)
    assert _run(node) == 2
    assert "pause: false" not in node.calls
    assert "left-over Gazebo" in node.errors[0]


def test_run_live_frozen_clock_exits_2() -> None:
    """The clock advances to 5 s and freezes: the run fails after the wall timeout."""
    node = _FakeNode(lambda w: min(max(0.0, w - 1.0), 5.0))
    assert _run(node, stall_timeout_s=3.0) == 2
    assert "stuck" in node.errors[0]
    assert node.wall < 1.0 + 5.0 + 3.0 + 1.0  # gave up soon after the timeout
