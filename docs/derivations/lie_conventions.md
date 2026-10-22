# Lie group conventions (SO(3), SE(3))

Implemented in `core/common/include/mss/common/lie.hpp` (wraps Sophus).
Tested by `tests/unit/test_lie.cpp` (C-01, C-02) and `tests/unit/backend/test_lie_gtsam.cpp`.
Every other module uses these functions; nothing else includes Sophus.

## Notation

- `skew(v)` (also written `v^`) is the 3x3 matrix with `skew(v) * u = v x u`.
- SO(3) tangent: `phi` in R^3 (axis times angle, radians), `theta = |phi|`.
- SE(3) tangent: `xi = [rho, phi]` in R^6. **Translation first, rotation second.**
  This order holds for every vector, Jacobian and covariance in `mss`.
- `Pose{R, p}` is `T = [R p; 0 1]`.

## Exp and Log

SO(3), Rodrigues:

    Exp(phi) = I + sin(theta)/theta * phi^ + (1 - cos(theta))/theta^2 * (phi^)^2

`Log` is its inverse on `theta in [0, pi]`. Near `theta = 0` and `theta = pi` the
formulas are evaluated by Sophus (`SO3::exp`, `SO3::log`, quaternion based); `mss` does
not re-implement them.

SE(3):

    Exp([rho, phi]) = [ Exp(phi)  Jl(phi) * rho ]
                      [ 0         1             ]

    Log(T) = [ Jl(Log(R))^-1 * p,  Log(R) ]

## Jacobians of SO(3)

Left Jacobian:

    Jl(phi)   = I + (1 - cos theta)/theta^2 * phi^ + (theta - sin theta)/theta^3 * (phi^)^2
    Jl(phi)^-1 = I - 1/2 phi^ + (1 - theta/2 * cot(theta/2))/theta^2 * (phi^)^2

Right Jacobian, defined by the right perturbation used everywhere in `mss`:

    Exp(phi + dphi) ~= Exp(phi) * Exp(Jr(phi) * dphi)

Relations used by the implementation:

    Jr(phi)    = Jl(-phi)    = Jl(phi)^T
    Jr(phi)^-1 = Jl(-phi)^-1
    Log(Exp(phi) * Exp(dphi)) ~= phi + Jr(phi)^-1 * dphi

`so3_jr(phi)` is `Sophus::SO3d::leftJacobian(-phi)` and `so3_jr_inv(phi)` is
`Sophus::SO3d::leftJacobianInverse(-phi)`.

## Box operators (right perturbation)

    boxplus(T, delta)  = T * Exp(delta)
    boxminus(a, b)     = Log(b^-1 * a)        so boxminus(boxplus(T, d), T) = d

A covariance `Sigma` on pose `T` means `T_true = T * Exp(delta)`, `delta ~ N(0, Sigma)`,
with `delta = [rho, phi]` expressed in the frame of `T`'s source (the body frame).

## Adjoint

For `T = (R, p)` acting on `[rho, phi]`:

    Ad(T) = [ R   p^ R ]
            [ 0   R    ]

    T * Exp(xi) * T^-1 = Exp(Ad(T) * xi)
    T * Exp(xi) = Exp(Ad(T) * xi) * T

`se3_adj(T)` is `Sophus::SE3d::Adj()`, which uses this layout.

Covariance transform: if `Sigma_x` is the covariance of a perturbation in frame `x`,
the same uncertainty expressed in frame `y` is

    Sigma_y = Ad(T_y_x) * Sigma_x * Ad(T_y_x)^T        (transform_covariance)

## Library orders

| Library | `log` / `Logmap` output | `exp` / `Expmap` input |
|---------|-------------------------|------------------------|
| `mss` (`se3_log`, `se3_exp`) | `[rho, phi]` | `[rho, phi]` |
| Sophus `SE3d::log()` / `SE3d::exp()` | `[rho, phi]` (translation first) | `[rho, phi]` |
| GTSAM `Pose3::Logmap` / `Pose3::Expmap` | `[phi, rho]` (**rotation first**) | `[phi, rho]` |

Both libraries use the right perturbation (`T * Exp(delta)`) and the same `Exp` map;
only the order of the 6-vector differs. GTSAM `Pose3` tangents, Jacobians and noise
models are therefore `[phi, rho]`, and `core/backend` must permute at its boundary:

    xi_gtsam = [xi_mss.tail<3>(), xi_mss.head<3>()]
    Sigma_gtsam = P * Sigma_mss * P^T,   P = [0 I; I 0]

`tests/unit/backend/test_lie_gtsam.cpp` checks the vector permutation only (`xi_gtsam`
against GTSAM `Pose3::Expmap`); the covariance/Jacobian matrix check `P * Sigma * P^T` is
planned together with `core/backend` (`graph.cpp`).
