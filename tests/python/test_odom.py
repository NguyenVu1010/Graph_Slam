import math

import numpy as np
import pytest

from mss_sim.odom import OdomTwist, WheelQuantizer, integrate_pose
from mss_sim.profile import Kinematics, OdomNoise

T0 = 1_700_000_000_000_000_000
DT_NS = 20_000_000  # 50 Hz
NAMES = ["left_wheel_joint", "right_wheel_joint"]
TRUE = Kinematics(wheel_radius_m=0.10, track_m=0.40)
BELIEVED = Kinematics(wheel_radius_m=0.101, track_m=0.394)


def _noise(sigma: float = 0.0) -> OdomNoise:
    return OdomNoise(
        ticks_per_rev=4096,
        speed_sigma_mps=sigma,
        kinematics_true=TRUE,
        kinematics_believed=BELIEVED,
    )


def _wheels(v: float, w: float) -> list[float]:
    half = w * TRUE.track_m / 2.0
    return [(v - half) / TRUE.wheel_radius_m, (v + half) / TRUE.wheel_radius_m]


def _run(q: WheelQuantizer, v: float, w: float, n: int) -> list[tuple[int, OdomTwist]]:
    out = []
    vel = _wheels(v, w)
    for i in range(n + 1):
        t = T0 + i * DT_NS
        tw = q.step(t, NAMES, vel)
        if tw is not None:
            out.append((t, tw))
    return out


def test_straight_line_distance_scaled_by_radius_error() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    twists = _run(q, 0.5, 0.0, 3000)
    pose = integrate_pose(twists)
    dist = float(np.hypot(pose[-1, 0], pose[-1, 1]))
    expected = 0.5 * 59.98 * (0.101 / 0.100)  # first step has no twist
    assert abs(dist - expected) / expected < 1e-3
    assert abs(pose[-1, 2]) < 1e-3


def test_spin_in_place_yaw_scaled_by_calibration_error() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    n = int(round(2 * math.pi / (DT_NS / 1e9)))
    twists = _run(q, 0.0, 1.0, n)
    pose = integrate_pose(twists)
    elapsed = (twists[-1][0] - T0) / 1e9
    expected = elapsed * (0.101 / 0.100) * (0.400 / 0.394)
    assert abs(pose[-1, 2] - expected) / expected < 5e-3
    assert abs(pose[-1, 0]) < 1e-2 and abs(pose[-1, 1]) < 1e-2


def test_missing_joint_returns_none_and_counts() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    assert q.step(T0, NAMES, [1.0, 1.0]) is None  # first call: time base only
    assert q.n_errors == 0
    assert q.step(T0 + DT_NS, ["left_wheel_joint", "other"], [1.0, 1.0]) is None
    assert q.n_errors == 1
    assert q.step(T0 + 2 * DT_NS, NAMES, [1.0, 1.0]) is not None


def test_backwards_or_repeated_stamp_returns_none() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    q.step(T0, NAMES, [1.0, 1.0])
    assert q.step(T0 + DT_NS, NAMES, [1.0, 1.0]) is not None
    assert q.step(T0, NAMES, [1.0, 1.0]) is None
    assert q.step(T0 + DT_NS, NAMES, [1.0, 1.0]) is None
    assert q.n_errors == 2
    assert q.step(T0 + 2 * DT_NS, NAMES, [1.0, 1.0]) is not None


def test_nan_rate_skipped_without_state_change() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    q.step(T0, NAMES, [1.0, 1.0])
    assert q.step(T0 + DT_NS, NAMES, [math.nan, 1.0]) is None
    assert q.n_errors == 1
    tw = q.step(T0 + 2 * DT_NS, NAMES, [1.0, 1.0])
    assert tw is not None
    # dt spans both periods (the NaN step did not advance time): v = rate * r_believed.
    assert tw.v_mps == pytest.approx(1.0 * BELIEVED.wheel_radius_m, rel=1e-2)


def test_short_velocity_list_skipped() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    q.step(T0, NAMES, [1.0, 1.0])
    assert q.step(T0 + DT_NS, NAMES, [1.0]) is None
    assert q.n_errors == 1
    assert q.step(T0 + 2 * DT_NS, NAMES, [1.0, 1.0]) is not None


def test_joint_order_in_message_is_irrelevant() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    q.step(T0, NAMES[::-1], _wheels(0.5, 0.2)[::-1])
    tw = q.step(T0 + DT_NS, NAMES[::-1], _wheels(0.5, 0.2)[::-1])
    assert tw is not None and tw.omega_radps > 0.0


def test_quantisation_visible_at_tiny_speed() -> None:
    q = WheelQuantizer(_noise(), np.random.default_rng(1))
    twists = _run(q, 0.001, 0.0, 30)
    vs = [tw.v_mps for _, tw in twists]
    assert vs[:5] == [0.0] * 5  # < 1 tick accumulated so far
    nonzero = [v for v in vs if v != 0.0]
    assert nonzero
    # both wheels gain one tick in one 20 ms step, believed radius
    one_tick = (2 * math.pi / 4096) / 0.02 * 0.101
    assert abs(nonzero[0] - one_tick) < 1e-9


def test_noise_one_draw_per_successful_step() -> None:
    noise = _noise(sigma=0.01)
    q = WheelQuantizer(noise, np.random.default_rng(7))
    ref = np.random.default_rng(7)
    q.step(T0, NAMES, [0.0, 0.0])
    q.step(T0 + DT_NS, ["x"], [0.0])  # error: no draw
    tw = q.step(T0 + 2 * DT_NS, NAMES, [0.0, 0.0])
    assert tw is not None
    assert math.isclose(tw.v_mps, ref.normal(0.0, 0.01))
    assert tw.omega_radps == 0.0


def test_integrate_pose_arc_midpoint_exact() -> None:
    v, w = 1.0, 0.5
    twists = [(T0 + i * DT_NS, OdomTwist(v, w)) for i in range(501)]
    pose = integrate_pose(twists)
    assert pose.shape == (501, 3)
    assert np.all(pose[0] == 0.0)
    t = 10.0
    assert math.isclose(pose[-1, 2], w * t, rel_tol=1e-9)
    assert math.isclose(pose[-1, 0], v / w * math.sin(w * t), abs_tol=1e-3)
    assert math.isclose(pose[-1, 1], v / w * (1 - math.cos(w * t)), abs_tol=1e-3)


def test_integrate_pose_empty() -> None:
    assert integrate_pose([]).shape == (0, 3)
