from collections.abc import Sequence

import numpy as np

from mss_sim.geodesy import lla_to_enu
from mss_sim.gnss import GnssDegrader, GnssSample
from mss_sim.profile import GnssNoise
from mss_sim.scenario import GnssSegment, GnssZone

T0 = 1_700_000_000_000_000_000
DT_NS = 200_000_000  # 5 Hz
ORIGIN = (21.0, 105.8, 10.0)
P0 = np.array([5.0, -3.0, 1.0])


def _noise(**kw: float) -> GnssNoise:
    base = dict(
        rate_hz=5.0,
        sigma_h_m=1.0,
        sigma_v_m=2.0,
        drift_sigma_m=0.5,
        drift_tau_s=300.0,
        multipath_amp_min_m=3.0,
        multipath_amp_max_m=10.0,
        multipath_hold_min_s=2.0,
        multipath_hold_max_s=10.0,
        multipath_start_rate_per_s=0.2,
        degraded_factor=3.0,
    )
    base.update(kw)
    return GnssNoise(**base)


def _deg(
    noise: GnssNoise | None = None,
    segments: Sequence[GnssSegment] = (),
    zones: Sequence[GnssZone] = (),
    seed: int = 1,
) -> GnssDegrader:
    return GnssDegrader(
        noise or _noise(), list(segments), list(zones), ORIGIN, np.random.default_rng(seed), T0
    )


def _run(deg: GnssDegrader, n: int, p: np.ndarray = P0) -> list[GnssSample | None]:
    return [deg.step(T0 + i * DT_NS, p) for i in range(n)]


def test_outage_window() -> None:
    deg = _deg(segments=[GnssSegment("outage", 10.0, 20.0)])
    out = _run(deg, 150)
    for i, s in enumerate(out):
        t = i * 0.2
        if 10.0 <= t <= 20.0:
            assert s is None, t
        else:
            assert s is not None, t
            assert s.status == 0


def test_nominal_sigma_and_cov() -> None:
    deg = _deg(_noise(drift_tau_s=1e9), seed=3)
    out = _run(deg, 2000)
    err = np.array([lla_to_enu(s.lla, ORIGIN) - P0 for s in out])
    err -= err.mean(axis=0)
    assert abs(err[:, 0].std() / 1.0 - 1) < 0.1
    assert abs(err[:, 1].std() / 1.0 - 1) < 0.1
    assert abs(err[:, 2].std() / 2.0 - 1) < 0.1
    np.testing.assert_allclose(out[0].cov_enu_diag, [1.0, 1.0, 4.0])


def test_multipath_run_with_nominal_cov() -> None:
    noise = _noise(sigma_h_m=0.1, sigma_v_m=0.1, drift_sigma_m=0.0)
    deg = _deg(noise, [GnssSegment("multipath", 0.0, 200.0)], seed=5)
    out = _run(deg, 1000)
    err = np.array([np.linalg.norm((lla_to_enu(s.lla, ORIGIN) - P0)[:2]) for s in out])
    best = run = 0
    for e in err > 3.0:
        run = run + 1 if e else 0
        best = max(best, run)
    assert (best - 1) * 0.2 >= 2.0
    for s in out:
        np.testing.assert_allclose(s.cov_enu_diag, [0.01, 0.01, 0.01])


def test_degraded_cov_scaled() -> None:
    deg = _deg(segments=[GnssSegment("degraded", 0.0, 10.0)])
    s = deg.step(T0, P0)
    np.testing.assert_allclose(s.cov_enu_diag, np.array([1.0, 1.0, 4.0]) * 9.0)


def test_zone_wins_over_segment() -> None:
    zone = GnssZone("outage", [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)])
    deg = _deg(segments=[GnssSegment("degraded", 0.0, 100.0)], zones=[zone])
    assert deg.mode_at(1.0, np.array([0.0, 0.0, 0.0])) == "outage"
    assert deg.mode_at(1.0, np.array([500.0, 0.0, 0.0])) == "degraded"
    assert deg.step(T0, np.array([0.0, 0.0, 0.0])) is None
    assert deg.step(T0 + DT_NS, np.array([500.0, 0.0, 0.0])) is not None


def test_mode_outside_segments_is_nominal() -> None:
    deg = _deg(segments=[GnssSegment("outage", 10.0, 20.0)])
    assert deg.mode_at(5.0, np.zeros(3)) == "nominal"
    assert deg.mode_at(25.0, np.zeros(3)) == "nominal"


def test_same_seed_identical() -> None:
    segs = [GnssSegment("multipath", 0.0, 30.0), GnssSegment("outage", 30.0, 35.0)]
    a = _run(_deg(segments=segs, seed=9), 200)
    b = _run(_deg(segments=segs, seed=9), 200)
    for x, y in zip(a, b, strict=True):
        if x is None:
            assert y is None
        else:
            assert x.lla == y.lla
            np.testing.assert_array_equal(x.cov_enu_diag, y.cov_enu_diag)
