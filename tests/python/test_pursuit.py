"""mss_sim.pursuit on a kinematic unicycle: tracking, limits, determinism, still start, stop."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

import gen_sim_assets as gsa
from mss_sim.pursuit import (
    GOAL_TOLERANCE_M,
    SPEED_VARIATION,
    STILL_S,
    Limits,
    PurePursuit,
    driver_rng,
)
from mss_sim.scenario import Route, load_scenario

REPO = Path(__file__).resolve().parents[2]
LIMITS = Limits(v_max_mps=1.0, a_max_mps2=0.5, omega_max_radps=1.0)
SQUARE = Route(
    waypoints=((0.0, 0.0), (6.0, 0.0), (6.0, 6.0), (0.0, 6.0), (0.0, 0.0)),
    speed_mps=0.5,
    lookahead_m=1.0,
)
DT_S = 0.02  # 50 Hz, like driver.py


def simulate(route: Route, seed: int, t_max_s: float = 120.0) -> dict[str, np.ndarray]:
    """Drive a unicycle that executes the commands exactly; start on the first waypoint."""
    pp = PurePursuit(route, LIMITS, seed)
    x, y = route.waypoints[0]
    d = np.subtract(route.waypoints[1], route.waypoints[0])
    yaw = math.atan2(d[1], d[0])
    rows = []
    t = 0.0
    while t < t_max_s:
        v, w = pp.command(t, x, y, yaw)
        rows.append((t, x, y, yaw, v, w))
        if pp.done:
            break
        x += v * DT_S * math.cos(yaw + w * DT_S / 2)
        y += v * DT_S * math.sin(yaw + w * DT_S / 2)
        yaw += w * DT_S
        t += DT_S
    a = np.array(rows)
    return {"t": a[:, 0], "x": a[:, 1], "y": a[:, 2], "v": a[:, 4], "w": a[:, 5], "done": pp.done}


def _dist_to_route(xy: np.ndarray, route: Route) -> np.ndarray:
    p = np.asarray(route.waypoints)
    best = np.full(len(xy), np.inf)
    for a, b in zip(p[:-1], p[1:], strict=True):
        d = b - a
        u = np.clip(((xy - a) @ d) / (d @ d), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(a + u[:, None] * d - xy, axis=1))
    return best


def test_follows_a_square_and_stops_at_the_end() -> None:
    run = simulate(SQUARE, seed=42)
    assert run["done"]
    xy = np.column_stack([run["x"], run["y"]])
    err = _dist_to_route(xy, SQUARE)
    # Pure pursuit cuts the corners by a fraction of the lookahead; straights track closely.
    assert err.max() < 0.35
    mid_straight = (np.abs(run["y"]) < 1e-9) & (run["x"] > 2.0) & (run["x"] < 4.0)
    assert err[mid_straight].max() < 0.02
    assert np.hypot(run["x"][-1], run["y"][-1]) <= 2 * GOAL_TOLERANCE_M
    assert run["v"][-1] == 0.0 and run["w"][-1] == 0.0


def test_respects_limits() -> None:
    run = simulate(SQUARE, seed=7)
    assert run["v"].min() >= 0.0
    assert run["v"].max() <= min(LIMITS.v_max_mps, SQUARE.speed_mps * (1 + SPEED_VARIATION)) + 1e-12
    assert np.abs(run["w"]).max() <= LIMITS.omega_max_radps + 1e-12
    accel = np.diff(run["v"]) / np.diff(run["t"])
    assert np.abs(accel).max() <= LIMITS.a_max_mps2 + 1e-9


def test_first_five_seconds_command_zero() -> None:
    run = simulate(SQUARE, seed=1)
    still = run["t"] < STILL_S - 1e-9
    assert still.sum() >= int(STILL_S / DT_S) - 1
    assert np.all(run["v"][still] == 0.0) and np.all(run["w"][still] == 0.0)
    assert run["v"][~still].max() > 0.0


def test_deterministic_for_a_seed() -> None:
    a, b = simulate(SQUARE, seed=42), simulate(SQUARE, seed=42)
    for key in ("x", "y", "v", "w"):
        np.testing.assert_array_equal(a[key], b[key])
    c = simulate(SQUARE, seed=43)
    assert not np.array_equal(a["v"][: len(c["v"])], c["v"][: len(a["v"])])


def test_speed_variation_within_ten_percent() -> None:
    pp = PurePursuit(SQUARE, LIMITS, seed=42)
    speeds = np.array([pp.target_speed(s) for s in np.linspace(0, pp.length_m, 200)])
    assert np.all(np.abs(speeds / SQUARE.speed_mps - 1.0) <= SPEED_VARIATION + 1e-12)
    assert np.ptp(speeds) > 0.0


def test_driver_stream_is_the_sixth_child() -> None:
    expected = np.random.default_rng(np.random.SeedSequence(42).spawn(6)[5]).uniform(size=3)
    np.testing.assert_array_equal(driver_rng(42).uniform(size=3), expected)
    # The first five children (the noise bridge's streams) are unchanged by spawning six.
    five = np.random.SeedSequence(42).spawn(5)
    six = np.random.SeedSequence(42).spawn(6)
    for s5, s6 in zip(five, six[:5], strict=True):
        assert s5.generate_state(4).tolist() == s6.generate_state(4).tolist()


def test_limits_from_robot_yaml() -> None:
    assert Limits.from_robot(gsa.load_robot()) == LIMITS


@pytest.mark.parametrize("scenario", ["E-01", "E-04", "E-07"])
def test_scenario_routes_start_at_spawn_inside_world_and_finish(scenario: str) -> None:
    sc = load_scenario(REPO / f"sim/scenarios/{scenario}.yaml")
    assert sc.route is not None
    side = yaml.safe_load((REPO / f"sim/worlds/{sc.world}.planes.yaml").read_text())
    assert sc.route.waypoints[0] == (side["spawn"]["x_m"], side["spawn"]["y_m"])
    xs = sorted(p["d"] for p in side["planes"] if p["n"] == [1.0, 0.0, 0.0])
    ys = sorted(p["d"] for p in side["planes"] if p["n"] == [0.0, 1.0, 0.0])
    for x, y in sc.route.waypoints:
        assert xs[0] < x < xs[-1] and ys[0] < y < ys[-1]
    run = simulate(sc.route, seed=sc.seed, t_max_s=sc.duration_s)
    assert run["done"], f"{scenario}: route not finished within duration_s"
    assert run["t"][-1] > 0.8 * sc.duration_s  # uses most of the recording


FOOTPRINT_HALF_M = 0.30  # sphere wheels at y = +-0.20 m, r 0.10 m; body x +-0.30 m (robot.yaml)
MIN_CLEARANCE_M = 0.5
ROBOT_HEIGHT_M = 0.75  # LiDAR top above the floor; boxes above this cannot be hit


@pytest.mark.parametrize("scenario", ["E-01", "E-04", "E-07"])
def test_routes_keep_clearance(scenario: str) -> None:
    """Every point of the route polyline keeps >= 0.5 m between the robot's collision footprint
    (+-0.30 m) and every world box that reaches the robot's height."""
    sc = load_scenario(REPO / f"sim/scenarios/{scenario}.yaml")
    assert sc.route is not None
    boxes = [
        b
        for b in gsa.world_boxes(REPO / f"sim/worlds/{sc.world}.sdf")
        if b.hi[2] > 0.001 and b.lo[2] < ROBOT_HEIGHT_M  # not the floor, not the ceiling
    ]
    wp = np.asarray(sc.route.waypoints)
    pts = np.concatenate(
        [
            np.linspace(a, b, int(np.ceil(np.linalg.norm(b - a) / 0.05)) + 1)
            for a, b in zip(wp[:-1], wp[1:], strict=True)
        ]
    )
    worst = np.inf
    for b in boxes:
        dx = np.maximum(np.maximum(b.lo[0] - pts[:, 0], pts[:, 0] - b.hi[0]), 0.0)
        dy = np.maximum(np.maximum(b.lo[1] - pts[:, 1], pts[:, 1] - b.hi[1]), 0.0)
        worst = min(worst, float(np.min(np.hypot(dx, dy))))
    assert worst - FOOTPRINT_HALF_M >= MIN_CLEARANCE_M - 1e-9, worst
