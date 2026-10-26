# GNSS: geodesy, residual, chi-square gate, ENU alignment

Implemented in `core/gnss/` (`mss::gnss`). Tested by `tests/unit/test_gnss_*.cpp` and
`tests/component/test_gnss_component.cpp` (G-01 to G-04 in `docs/Test_CI_CD.md`).
Frames and perturbation follow `docs/conventions.md` and `docs/derivations/lie_conventions.md`:
`e` is ENU (x east, y north, z up), `w` the SLAM world frame, `b` body, `g` GNSS antenna;
right perturbation `T <- T * Exp(xi)`, `xi = [rho, phi]`.

## 1. Geodesy (WGS-84)

Code: `lla_to_enu`, `enu_to_lla` in `core/gnss/src/geodesy.cpp`. The algorithm and constants
are copied from `sim/mss_sim/geodesy.py` so C++ and Python agree to about 1e-12 relative.

Constants: `a = 6378137.0` m (`kWgs84AM`), `f = 1/298.257223563` (`kWgs84F`),
`b = a (1 - f)` (`kWgs84BM`), `e2 = f (2 - f)` (`kE2`), `ep2 = e2 / (1 - e2)` (`kEp2`).

LLA to ECEF (`lla_to_ecef`), with `lat`, `lon` in radians and `h` in metres:

    N = a / sqrt(1 - e2 sin^2 lat)
    x = (N + h) cos lat cos lon
    y = (N + h) cos lat sin lon
    z = (N (1 - e2) + h) sin lat

ECEF to LLA (`ecef_to_lla`), Bowring, `kBowringIterations = 5`:

    p    = hypot(x, y)
    lon  = atan2(y, x)                      (in (-pi, pi])
    beta = atan2(z a, p b)
    repeat 5 times:
      lat  = atan2(z + ep2 b sin^3 beta, p - e2 a cos^3 beta)
      beta = atan2((1 - f) sin lat, cos lat)
    h = p cos lat + z sin lat - a^2 / N(lat)

Rotation ECEF to ENU at the origin (`r_e_ecef`), rows:

    [ -sin lon,           cos lon,          0       ]
    [ -sin lat cos lon,  -sin lat sin lon,  cos lat ]
    [  cos lat cos lon,   cos lat sin lon,  sin lat ]

    p_e    = R_e_ecef (x_ecef - x0_ecef)
    x_ecef = x0_ecef + R_e_ecef^T p_e

## 2. GNSS position residual and Jacobian

Code: `detail::gnss_residual` in `core/gnss/src/innovation.hpp`.

Residual (3x1, metres, expressed in e):

    r = p_e_g - T_e_w (T_w_b p_b_g)
      = p_e_g - R_e_w (R_w_b p_b_g + p_w_b) - p_e_w

Variable: `T_w_b`, right perturbation `T_w_b <- T_w_b Exp([rho, phi])`. To first order:

    p_w_b                <- p_w_b + R_w_b rho
    R_w_b Exp(phi) p_b_g ~= R_w_b p_b_g + R_w_b skew(phi) p_b_g
                          = R_w_b p_b_g - R_w_b skew(p_b_g) phi

so

    J_r_xi = [ -R_e_w R_w_b ,  R_e_w R_w_b skew(p_b_g) ]     (3x6, columns [rho, phi])

## 3. Chi-square innovation gate

Code: `detail::innovation_nis`, `detail::chi_square_gate`.

The caller forms the innovation covariance

    S = Sigma_fix + J_r_xi P J_r_xi^T

with `P` the 6x6 `[rho, phi]` covariance of `T_w_b`. The normalised innovation squared is
`NIS = r^T S^-1 r`, computed with an LLT of `S` (fixed size, no heap). Non-finite `r` or
`S` and a `S` that is not positive definite are errors (a NaN NIS would otherwise compare
false against the threshold and pass). The fix is rejected when `NIS > chi2_3(q)`:

| q    | chi2, 3 DoF        |
|------|--------------------|
| 0.95 | 7.814727903251178  |
| 0.99 | 11.344866730144373 |

The module uses q = 0.95 (`detail::kChi2Gate3Dof95`): consistent fixes are accepted 95% of
the time, the middle of the 90-99% band of G-02.

## 4. 4-DoF alignment of w to e

Code: `GnssAligner` in `core/gnss/src/aligner.cpp`.

w and e are both gravity-aligned with z up, so `R_e_w = Rz(yaw)` and `p_e_w` is 3-D.
Pairs: `p_w_g,i = T_w_b,i p_b_g` and `p_e_g,i`. For conditioning (ENU coordinates can be
of order 1e5 m), every pair is shifted by the first one:

    a_i = p_w_g,i - p_w_g,0        b_i = p_e_g,i - p_e_g,0

Running sums: `n`, `sum a`, `sum b`, `C = sum a_h b_h^T` (2x2, horizontal parts),
`sum |a_h|^2`. Centred cross-covariance:

    C_c = C - (sum a_h)(sum b_h)^T / n

Maximising `sum b_c^T Rz(yaw) a_c` gives

    yaw = atan2(C_c(0,1) - C_c(1,0), C_c(0,0) + C_c(1,1))

and the translation, with the un-shifted means,

    p_e_w = mean(p_e_g) - Rz(yaw) mean(p_w_g)
          = (p_e_g,0 + sum b / n) - Rz(yaw) (p_w_g,0 + sum a / n)

Horizontal spread (RMS distance of `p_w_g,h` from its centroid):

    spread = sqrt(max(0, sum |a_h|^2 / n - |sum a_h / n|^2))

The same spread is computed on the e side from `sum b` and `sum |b_h|^2`. If either side is
below `kMinHorizontalSpreadM`, yaw is unobservable and no transform is returned. The e-side
check catches a receiver that repeats or holds one fix while the body moves: then
`C_c ~ 0` and `atan2(~0, ~0)` would give an arbitrary yaw.

Observability: `sigma_yaw ~ sigma_h / sqrt(sum |a_c,h|^2)`. A straight 30 m path at 1 m/s
and 5 Hz has 151 fixes and `sum |a_c,h|^2 ~ n L^2 / 12 ~ 151 * 900 / 12 ~ 1.13e4 m^2`, so at
`sigma_h = 1.0` m `sigma_yaw ~ 1 / 106 rad ~ 0.54 deg`. G-03 therefore uses
`sigma_h = 0.2` m (1-sigma about 0.11 deg) to test the 0.5 deg requirement with margin.
