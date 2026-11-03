"""ROS-free parts of mss_teleop (ADR-031), unit-tested in tests/python/test_teleop_logic.py.

- ``ButtonEdges``: rising edges of gamepad buttons (a held button fires once).
- ``pause_request``: the Start toggle from mss_node's ``/mss/paused``.
- ``PendingCalls``: one call per kind in flight, stale after a timeout.
- ``drive_limits``: the speed limits of the simulated robot from ``sim/models/robot.yaml``, the
  only source of them (the DiffDrive plugin clamps to the same values).
- ``world_scenario``: the scenario and SLAM overlay a teleop world reuses (E-07W, E-07).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# Teleop world -> the scenario it reuses (noise, seed, GNSS, wheels, spawn pose) and that
# scenario's loops-on overlay (controller ruling, ADR-031). No new scenario files.
WORLD_SCENARIOS = {"warehouse": "sim/scenarios/E-07W.yaml", "outdoor": "sim/scenarios/E-07.yaml"}
LOOPS_ON_VARIANT = "loops_on"


@dataclass
class ButtonEdges:
    """Rising-edge detector for the buttons listed in ``watch`` (indices into Joy.buttons)."""

    watch: Sequence[int]
    _last: dict[int, bool] = field(default_factory=dict)

    def update(self, buttons: Sequence[int]) -> list[int]:
        """Buttons of ``watch`` pressed now and not pressed in the previous message.

        A button missing from ``buttons`` (a shorter array) counts as released.
        """
        pressed = []
        for b in self.watch:
            now = 0 <= b < len(buttons) and bool(buttons[b])
            if now and not self._last.get(b, False):
                pressed.append(b)
            self._last[b] = now
        return pressed


def pause_request(paused: bool | None) -> bool | None:
    """The ``/mss/pause`` request for a Start press: pause a running world, resume a paused one;
    ``None`` while the state (mss_node's ``/mss/paused``) is still unknown."""
    return None if paused is None else not paused


@dataclass
class PendingCalls:
    """Service calls in flight, by kind. A call older than ``timeout_s`` (wall) no longer blocks
    a new one: its future may never complete when the server died mid-call (review N-6)."""

    timeout_s: float  # wall s, > 0
    _started: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.timeout_s > 0:
            msg = f"call_timeout_s must be > 0, got {self.timeout_s}"
            raise ValueError(msg)

    def start(self, kind: str, now_s: float) -> float | None:
        """Register a call of ``kind`` at ``now_s`` and return its token (the start time); None
        when a fresh call of that kind is still pending."""
        t0 = self._started.get(kind)
        if t0 is not None and now_s - t0 <= self.timeout_s:
            return None
        self._started[kind] = now_s
        return now_s

    def finish(self, kind: str, token: float) -> None:
        """The call with ``token`` is done. A stale call that completes late does not clear a
        newer call of the same kind (review N2-5)."""
        if self._started.get(kind) == token:
            del self._started[kind]


@dataclass(frozen=True)
class DriveLimits:
    linear_mps: float  # max forward speed, m/s
    angular_radps: float  # max yaw rate, rad/s


def drive_limits(robot_yaml: Path) -> DriveLimits:
    """``drive.max_linear_velocity_mps`` and ``drive.max_angular_velocity_radps`` of robot.yaml.

    Raises ValueError naming the key when one is missing or not > 0.
    """
    doc = yaml.safe_load(Path(robot_yaml).read_text(encoding="utf-8"))
    drive = doc.get("drive", {}) if isinstance(doc, dict) else {}
    out = []
    for key in ("max_linear_velocity_mps", "max_angular_velocity_radps"):
        value = drive.get(key)
        if not isinstance(value, int | float) or not value > 0:
            msg = f"{robot_yaml}: drive.{key} must be a number > 0, got {value!r}"
            raise ValueError(msg)
        out.append(float(value))
    return DriveLimits(*out)


@dataclass(frozen=True)
class WorldRun:
    scenario: Path  # sim/scenarios/*.yaml (absolute)
    overlay: Path  # the scenario's loops-on pipeline overlay (absolute)
    world: str  # Gazebo world name of the scenario


def world_scenario(repo: Path, world: str) -> WorldRun:
    """The scenario and overlay that teleop ``world`` reuses.

    Raises ValueError for an unknown world or a scenario whose world or loops-on overlay does
    not match.
    """
    if world not in WORLD_SCENARIOS:
        msg = f"world:={world}: must be one of {sorted(WORLD_SCENARIOS)}"
        raise ValueError(msg)
    scenario = (Path(repo) / WORLD_SCENARIOS[world]).resolve()
    sc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    if sc.get("world") != world:
        msg = f"{scenario}: world is {sc.get('world')!r}, expected {world!r}"
        raise ValueError(msg)
    variant = sc.get("pipeline_config_variants", {}).get(LOOPS_ON_VARIANT)
    if not variant:
        msg = f"{scenario}: no pipeline_config_variants.{LOOPS_ON_VARIANT}"
        raise ValueError(msg)
    return WorldRun(scenario, (Path(repo) / variant).resolve(), world)


def find_repo(start: Path) -> Path:
    """``start`` or its nearest ancestor holding ``sim/mss_sim`` (the multi-sensor-slam repo).

    Raises ValueError when there is none.
    """
    start = Path(start).resolve()
    for d in (start, *start.parents):
        if (d / "sim" / "mss_sim" / "__init__.py").is_file():
            return d
    msg = f"no multi-sensor-slam repo (sim/mss_sim) at or above {start}: pass repo:=<path>"
    raise ValueError(msg)
