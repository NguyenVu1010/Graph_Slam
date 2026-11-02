"""ADR-029 walking trajectory (backlog L1): still start, gait amplitudes, frequency, consistency."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mss_sim.scenario import Route
from mss_sim.walking import Gait, walk

STRAIGHT = Route(waypoints=((0.0, 0.0), (60.0, 0.0)), speed_mps=1.3, lookahead_m=1.5)
L_TURN = Route(waypoints=((0.0, 0.0), (20.0, 0.0), (20.0, 20.0)), speed_mps=1.3, lookahead_m=1.5)


@pytest.fixture(scope="module")
def straight() -> tuple:
    gait, limits = Gait.load()
    return gait, walk(STRAIGHT, gait, limits, seed=42, t_max_s=60.0)


def test_still_start_has_no_motion(straight: tuple) -> None:
    gait, w = straight
    still = w.t_s < 5.0
    np.testing.assert_allclose(w.p_w_b[still], [[0.0, 0.0, gait.body_height_m]] * still.sum())
    np.testing.assert_allclose(w.v_w_b[still][1:-1], 0.0, atol=1e-12)
    np.testing.assert_allclose(w.R_w_b[still], np.broadcast_to(np.eye(3), w.R_w_b[still].shape))


def test_cruise_amplitudes_and_step_frequency(straight: tuple) -> None:
    gait, w = straight
    cruise = (w.t_s > 15.0) & (w.t_s < 40.0)
    z = w.p_w_b[cruise, 2] - gait.body_height_m
    # +-10 % speed pieces around 1.3 m/s: k = min(v / 1.3, 1) in [0.9, 1].
    assert 0.9 * gait.bounce_amplitude_m <= np.max(np.abs(z)) <= gait.bounce_amplitude_m + 1e-12
    assert np.max(np.abs(w.p_w_b[cruise, 1])) <= gait.sway_amplitude_m + 1e-12
    rpy = Rotation.from_matrix(w.R_w_b[cruise]).as_euler("ZYX")
    assert np.max(np.abs(rpy[:, 2])) == pytest.approx(gait.roll_amplitude_rad, rel=0.1)
    assert np.max(np.abs(rpy[:, 1])) == pytest.approx(gait.pitch_amplitude_rad, rel=0.1)
    spec = np.abs(np.fft.rfft(z - z.mean()))
    f = np.fft.rfftfreq(len(z), d=w.t_s[1] - w.t_s[0])
    f_peak = f[np.argmax(spec)]
    speed = np.linalg.norm(w.v_w_b[cruise, :2], axis=1).mean()
    expect = gait.step_frequency_hz * min(speed / gait.cruise_speed_mps, 1.0)
    assert f_peak == pytest.approx(expect, abs=0.1)
    # Vertical acceleration of the bounce: A (2 pi f)^2, several m/s^2 (ADR-029 context).
    az = np.gradient(w.v_w_b[cruise, 2], w.t_s[1] - w.t_s[0])
    assert np.max(np.abs(az)) > 2.5


def test_velocities_integrate_to_the_poses() -> None:
    gait, limits = Gait.load()
    w = walk(L_TURN, gait, limits, seed=7, t_max_s=40.0, rate_hz=1000.0)
    dt = w.t_s[1] - w.t_s[0]
    p = w.p_w_b[0] + np.concatenate([[[0, 0, 0]], np.cumsum(w.v_w_b[1:] * dt, axis=0)])
    assert np.max(np.linalg.norm(p - w.p_w_b, axis=1)) < 0.01
    # Body rate: R_{i+1} ~ R_i Exp(omega_i dt). A pursuit command step changes the yaw rate by
    # up to omega_max, so one sample may be off by omega_max dt / 2 (0.75 mrad); the rest agree.
    rot = Rotation.from_matrix(w.R_w_b)
    pred = rot[1:-1] * Rotation.from_rotvec(w.omega_b[1:-1] * dt)
    err = (pred.inv() * rot[2:]).magnitude()
    assert np.max(err) < 1e-3
    assert np.percentile(err, 95) < 1e-5
    yaw_rate_integral = float(np.sum(w.omega_b[1:-1, 2] * dt))  # nearly level: ~ yaw change
    # The end samples are filled from their neighbours, not left at zero (review M-10).
    np.testing.assert_array_equal(w.omega_b[0], w.omega_b[1])
    np.testing.assert_array_equal(w.omega_b[-1], w.omega_b[-2])
    yaw = Rotation.from_matrix(w.R_w_b[[1, -2]]).as_euler("ZYX")[:, 0]
    assert yaw_rate_integral == pytest.approx(yaw[1] - yaw[0], abs=0.02)


def test_route_is_followed_and_deterministic() -> None:
    gait, limits = Gait.load()
    a = walk(L_TURN, gait, limits, seed=3, t_max_s=40.0)
    b = walk(L_TURN, gait, limits, seed=3, t_max_s=40.0)
    assert a.done
    np.testing.assert_array_equal(a.p_w_b, b.p_w_b)
    assert np.linalg.norm(a.p_w_b[-1, :2] - [20.0, 20.0]) < 0.2
    yaw_end = Rotation.from_matrix(a.R_w_b[-1]).as_euler("ZYX")[0]
    assert yaw_end == pytest.approx(math.pi / 2, abs=0.1)


def test_ends_are_valid_when_the_walk_stops_mid_motion() -> None:
    gait, limits = Gait.load()
    w = walk(STRAIGHT, gait, limits, seed=1, t_max_s=12.0, rate_hz=1000.0)  # cut while walking
    assert not w.done
    dt = w.t_s[1] - w.t_s[0]
    assert np.linalg.norm(w.v_w_b[-1, :2]) > 0.5  # still moving at the cut
    step = (Rotation.from_matrix(w.R_w_b[-2]).inv() * Rotation.from_matrix(w.R_w_b[-1])).as_rotvec()
    # First order: off by about the angular acceleration x dt / 2 (gait pitch: ~2e-3 rad/s).
    np.testing.assert_allclose(w.omega_b[-1], step / dt, atol=3e-3)


def test_rate_must_divide_the_internal_grid() -> None:
    gait, limits = Gait.load()
    with pytest.raises(ValueError, match="must divide"):
        walk(STRAIGHT, gait, limits, seed=0, t_max_s=1.0, rate_hz=300.0)
