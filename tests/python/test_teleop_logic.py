"""ROS-free parts of mss_teleop (ADR-031): button edges, robot speed limits, world scenarios."""

from pathlib import Path

import pytest
import yaml

from mss_teleop.teleop_logic import (
    WORLD_SCENARIOS,
    ButtonEdges,
    PendingCalls,
    drive_limits,
    find_repo,
    pause_request,
    world_scenario,
)

REPO = Path(__file__).resolve().parents[2]
TELEOP = REPO / "mss_teleop"


def test_button_fires_once_per_press() -> None:
    e = ButtonEdges([3, 7])
    assert e.update([0] * 8) == []
    assert e.update([0, 0, 0, 1, 0, 0, 0, 0]) == [3]
    assert e.update([0, 0, 0, 1, 0, 0, 0, 0]) == []  # held: no repeat
    assert e.update([0, 0, 0, 1, 0, 0, 0, 1]) == [7]
    assert e.update([0] * 8) == []
    assert e.update([0, 0, 0, 1, 0, 0, 0, 1]) == [3, 7]


def test_short_button_array_counts_as_released() -> None:
    e = ButtonEdges([7])
    assert e.update([0, 0, 0, 0, 0, 0, 0, 1]) == [7]
    assert e.update([1, 1]) == []  # a pad with fewer buttons: 7 released
    assert e.update([0, 0, 0, 0, 0, 0, 0, 1]) == [7]


def test_drive_limits_are_the_robot_maxima() -> None:
    robot = yaml.safe_load((REPO / "sim" / "models" / "robot.yaml").read_text())
    lim = drive_limits(REPO / "sim" / "models" / "robot.yaml")
    assert lim.linear_mps == robot["drive"]["max_linear_velocity_mps"]
    assert lim.angular_radps == robot["drive"]["max_angular_velocity_radps"]


def test_drive_limits_reject_missing_or_bad(tmp_path: Path) -> None:
    p = tmp_path / "robot.yaml"
    p.write_text("drive: {max_linear_velocity_mps: 1.0}\n")
    with pytest.raises(ValueError, match="max_angular_velocity_radps"):
        drive_limits(p)
    p.write_text("drive: {max_linear_velocity_mps: 0, max_angular_velocity_radps: 1}\n")
    with pytest.raises(ValueError, match="max_linear_velocity_mps"):
        drive_limits(p)


@pytest.mark.parametrize("world", sorted(WORLD_SCENARIOS))
def test_world_reuses_the_scenario_and_its_loops_on_overlay(world: str) -> None:
    run = world_scenario(REPO, world)
    assert run.world == world
    assert run.scenario.is_file()
    assert run.overlay.is_file()
    assert run.overlay.name.endswith("_loops_on.yaml")
    over = yaml.safe_load(run.overlay.read_text())["mss_node"]["ros__parameters"]
    assert over["loop_closure"]["enabled"] is True


def test_unknown_world_and_repo(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="world:=corridor"):
        world_scenario(REPO, "corridor")
    assert find_repo(REPO / "sim" / "worlds") == REPO
    with pytest.raises(ValueError, match="repo:="):
        find_repo(tmp_path)


def test_joy_mapping_has_dead_man_and_no_speed_scale() -> None:
    """Speeds come from robot.yaml through the launch only (design: no new numbers)."""
    cfg = yaml.safe_load((TELEOP / "config" / "joy.yaml").read_text())
    tj = cfg["teleop_twist_joy_node"]["ros__parameters"]
    assert tj["require_enable_button"] is True
    assert tj["enable_turbo_button"] == -1
    assert not any(k.startswith("scale") for k in tj)
    cmd = cfg["mss_joy_commands"]["ros__parameters"]
    assert cmd["save_button"] != cmd["pause_button"]
    assert tj["enable_button"] not in (cmd["save_button"], cmd["pause_button"])


def test_rviz_config_shows_the_graph_topics() -> None:
    rviz = (TELEOP / "rviz" / "teleop.rviz").read_text()
    for topic in (
        "/mss/graph/keyframes",
        "/mss/graph/loops",
        "/mss/graph/path",
        "/mss/graph/counts",
    ):
        assert f"Value: {topic}" in rviz
    yaml.safe_load(rviz)  # still valid YAML


def test_pause_request_toggles_the_reported_state() -> None:
    assert pause_request(False) is True  # running: pause
    assert pause_request(True) is False  # paused (by anyone): resume
    assert pause_request(None) is None  # no /mss/paused yet: do nothing


def test_pending_call_blocks_until_done_or_stale() -> None:
    p = PendingCalls(15.0)
    t_save = p.start("save", 100.0)
    assert t_save is not None
    assert p.start("save", 101.0) is None  # still pending
    assert p.start("pause", 101.0) is not None  # other kinds are independent
    p.finish("save", t_save)
    t_old = p.start("save", 102.0)
    assert t_old is not None
    t_new = p.start("save", 120.0)  # 18 s without an answer: stale, a new call may go
    assert t_new is not None
    p.finish("save", t_old)  # the stale call completes late (review N2-5)
    assert p.start("save", 121.0) is None  # the newer call is still pending
    p.finish("save", t_new)
    assert p.start("save", 122.0) is not None
    with pytest.raises(ValueError, match="call_timeout_s"):
        PendingCalls(0.0)
