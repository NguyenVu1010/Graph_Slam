"""Allan deviation helpers: white-noise and random-walk densities recovered from synthetic data."""

import numpy as np

from mss_eval.allan import fit_white_and_walk, oadev, theory_curve

_RATE_HZ = 200.0
_DURATION_S = 2 * 3600
_N = 1.75e-4  # white density, unit/sqrt(Hz)
_K = 1.0e-5  # random-walk density, unit/s/sqrt(Hz)


def _synth(seed: int) -> np.ndarray:
    rng_white, rng_walk = (np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(2))
    n = int(_DURATION_S * _RATE_HZ)
    white = rng_white.normal(0.0, _N * np.sqrt(_RATE_HZ), n)  # discrete sigma = N * sqrt(rate)
    walk = np.cumsum(rng_walk.normal(0.0, _K / np.sqrt(_RATE_HZ), n))  # step sigma = K*sqrt(dt)
    return white + walk


def test_fit_recovers_white_and_walk_densities() -> None:
    taus, adev = oadev(_synth(0), _RATE_HZ)
    n_est, k_est = fit_white_and_walk(taus, adev)
    assert abs(n_est / _N - 1.0) < 0.10, n_est
    assert abs(k_est / _K - 1.0) < 0.10, k_est


def test_theory_curve_matches_formula() -> None:
    taus = np.array([0.01, 1.0, 100.0])
    expected = np.sqrt(_N**2 / taus + _K**2 * taus / 3.0)
    np.testing.assert_allclose(theory_curve(taus, _N, _K), expected)


def test_oadev_of_white_noise_has_minus_half_slope() -> None:
    rng = np.random.default_rng(1)
    x = rng.normal(0.0, _N * np.sqrt(_RATE_HZ), 200_000)
    taus, adev = oadev(x, _RATE_HZ)
    assert taus[0] == 1.0 / _RATE_HZ
    short = taus <= 1.0
    slope = np.polyfit(np.log(taus[short]), np.log(adev[short]), 1)[0]
    assert abs(slope + 0.5) < 0.05
