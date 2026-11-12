#pragma once

namespace mss2d {

// Hand-written fixed-size vectors and matrices (no Eigen on the MCU, ADR-034). Templated on
// the scalar so tests can instantiate the same formulas with double.

template <typename S>
struct Vec2T {
  S x{0};
  S y{0};
};

template <typename S>
struct Vec3T {
  S v[3]{0, 0, 0};
  S& operator[](int i) { return v[i]; }
  const S& operator[](int i) const { return v[i]; }
};

/// Row-major 3x3 matrix, zero-initialised.
template <typename S>
struct Mat3T {
  S m[3][3]{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  S& operator()(int r, int c) { return m[r][c]; }
  const S& operator()(int r, int c) const { return m[r][c]; }
};

using Vec2f = Vec2T<float>;
using Vec3f = Vec3T<float>;
using Mat3f = Mat3T<float>;

}  // namespace mss2d
