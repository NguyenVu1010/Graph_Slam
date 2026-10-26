"""WGS-84 geodetic conversions: LLA <-> ECEF <-> local ENU.

Angles are degrees at the API (latitude, longitude), metres for altitude and ECEF/ENU.
Functions accept a single point (a 3-tuple or a ``(3,)`` array) or a batch ``(N, 3)``.
A single-point input returns a single-point output (``Lla`` tuple for LLA results).
"""

import numpy as np

Lla = tuple[float, float, float]  # (latitude deg, longitude deg, altitude m)

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
_B_M = WGS84_A_M * (1.0 - WGS84_F)
_E2 = WGS84_F * (2.0 - WGS84_F)  # first eccentricity squared
_EP2 = _E2 / (1.0 - _E2)  # second eccentricity squared
_BOWRING_ITERATIONS = 5


def _as_2d(x: object) -> tuple[np.ndarray, bool]:
    arr = np.asarray(x, dtype=float)
    if arr.shape == (3,):
        return arr[None, :], True
    if arr.ndim == 2 and arr.shape[1] == 3:
        return arr, False
    raise ValueError(f"expected shape (3,) or (N, 3), got {arr.shape}")


def _to_lla_out(arr: np.ndarray, single: bool) -> np.ndarray | Lla:
    if single:
        return (float(arr[0, 0]), float(arr[0, 1]), float(arr[0, 2]))
    return arr


def lla_to_ecef(lla: Lla | np.ndarray) -> np.ndarray:
    """Geodetic (deg, deg, m) to ECEF (m). Returns ``(3,)`` or ``(N, 3)``."""
    a, single = _as_2d(lla)
    lat = np.deg2rad(a[:, 0])
    lon = np.deg2rad(a[:, 1])
    h = a[:, 2]
    sin_lat = np.sin(lat)
    n = WGS84_A_M / np.sqrt(1.0 - _E2 * sin_lat**2)
    x = (n + h) * np.cos(lat) * np.cos(lon)
    y = (n + h) * np.cos(lat) * np.sin(lon)
    z = (n * (1.0 - _E2) + h) * sin_lat
    out = np.stack([x, y, z], axis=1)
    return out[0] if single else out


def ecef_to_lla(xyz: np.ndarray) -> np.ndarray | Lla:
    """ECEF (m) to geodetic via Bowring iteration. ``(3,)`` -> ``Lla``, ``(N, 3)`` -> array."""
    a, single = _as_2d(xyz)
    x, y, z = a[:, 0], a[:, 1], a[:, 2]
    p = np.hypot(x, y)
    lon = np.arctan2(y, x)
    beta = np.arctan2(z * WGS84_A_M, p * _B_M)
    lat = np.arctan2(z, p * (1.0 - _E2))
    for _ in range(_BOWRING_ITERATIONS):
        lat = np.arctan2(
            z + _EP2 * _B_M * np.sin(beta) ** 3,
            p - _E2 * WGS84_A_M * np.cos(beta) ** 3,
        )
        beta = np.arctan2((1.0 - WGS84_F) * np.sin(lat), np.cos(lat))
    sin_lat = np.sin(lat)
    n = WGS84_A_M / np.sqrt(1.0 - _E2 * sin_lat**2)
    h = p * np.cos(lat) + z * sin_lat - WGS84_A_M**2 / n
    out = np.stack([np.rad2deg(lat), np.rad2deg(lon), h], axis=1)
    return _to_lla_out(out, single)


def _r_enu_ecef(origin: Lla) -> np.ndarray:
    lat = np.deg2rad(origin[0])
    lon = np.deg2rad(origin[1])
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-so, co, 0.0],
            [-sl * co, -sl * so, cl],
            [cl * co, cl * so, sl],
        ]
    )


def lla_to_enu(lla: Lla | np.ndarray, origin: Lla) -> np.ndarray:
    """Geodetic to local ENU (m) about ``origin``. Returns ``(3,)`` or ``(N, 3)``."""
    d = lla_to_ecef(lla) - lla_to_ecef(origin)
    return d @ _r_enu_ecef(origin).T


def enu_to_lla(enu: np.ndarray, origin: Lla) -> np.ndarray | Lla:
    """Local ENU (m) about ``origin`` to geodetic. ``(3,)`` -> ``Lla``, ``(N, 3)`` -> array."""
    a, single = _as_2d(enu)
    ecef = lla_to_ecef(origin) + a @ _r_enu_ecef(origin)
    return ecef_to_lla(ecef[0] if single else ecef)
