#pragma once

#include <cmath>

#include "mss2d/small_matrix.hpp"

namespace mss2d {

/// Wraps an angle (rad) to [-pi, pi).
template <typename S>
S normalize_angle(S a) {
  const S pi = static_cast<S>(3.14159265358979323846);
  a = std::fmod(a + pi, 2 * pi);
  if (a < 0) {
    a += 2 * pi;
  }
  return a - pi;
}

/// SE(2) element T_a_b: maps points from frame b to frame a, p_a = T_a_b * p_b
/// (docs/conventions.md). (x, y) = p_a_b in m, yaw = rotation of b in a, rad, kept in
/// [-pi, pi). Tangent order [x, y, yaw]; right perturbation T <- T * Exp(delta).
template <typename S>
struct Pose2T {
  S x{0};
  S y{0};
  S yaw{0};

  Pose2T() = default;
  Pose2T(S x_m, S y_m, S yaw_rad) : x(x_m), y(y_m), yaw(normalize_angle(yaw_rad)) {}

  /// T_a_c = T_a_b * T_b_c.
  Pose2T operator*(const Pose2T& o) const {
    const S c = std::cos(yaw);
    const S s = std::sin(yaw);
    return Pose2T(x + c * o.x - s * o.y, y + s * o.x + c * o.y, yaw + o.yaw);
  }

  /// p_a = T_a_b * p_b.
  Vec2T<S> operator*(const Vec2T<S>& p) const {
    const S c = std::cos(yaw);
    const S s = std::sin(yaw);
    return Vec2T<S>{x + c * p.x - s * p.y, y + s * p.x + c * p.y};
  }

  /// T_b_a.
  Pose2T inverse() const {
    const S c = std::cos(yaw);
    const S s = std::sin(yaw);
    return Pose2T(-c * x - s * y, s * x - c * y, -yaw);
  }

  /// Exp([x, y, yaw]) = (V(yaw) [x, y], yaw), V = [[A, -B], [B, A]],
  /// A = sin(yaw)/yaw, B = (1 - cos(yaw))/yaw (Taylor series near 0).
  static Pose2T exp(const Vec3T<S>& d) {
    const S th = d[2];
    S a;
    S b;
    if (std::abs(th) < static_cast<S>(1e-4)) {
      a = 1 - th * th / 6;
      b = th / 2 - th * th * th / 24;
    } else {
      // 1 - cos(th) = 2 sin^2(th/2): no cancellation in float for small th.
      const S sh = std::sin(th / 2);
      a = std::sin(th) / th;
      b = 2 * sh * sh / th;
    }
    return Pose2T(a * d[0] - b * d[1], b * d[0] + a * d[1], th);
  }

  /// Inverse of exp for yaw in [-pi, pi): V^-1 = [[h, th/2], [-th/2, h]],
  /// h = (th/2) cot(th/2).
  Vec3T<S> log() const {
    const S th = yaw;
    S h;
    if (std::abs(th) < static_cast<S>(1e-4)) {
      h = 1 - th * th / 12;
    } else {
      // (th/2) cot(th/2); the form sin(th) / (1 - cos(th)) divides by zero in float for
      // th just above the Taylor branch.
      h = (th / 2) * std::cos(th / 2) / std::sin(th / 2);
    }
    Vec3T<S> d;
    d[0] = h * x + (th / 2) * y;
    d[1] = -(th / 2) * x + h * y;
    d[2] = th;
    return d;
  }
};

using Pose2 = Pose2T<float>;

}  // namespace mss2d
