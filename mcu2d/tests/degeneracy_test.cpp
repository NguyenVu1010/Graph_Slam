#include "mss2d/degeneracy.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

namespace mss2d {
namespace {

Mat3f from_eig(const float l[3], const Mat3f& v) {
  Mat3f a;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        a(r, c) += v(r, k) * l[k] * v(c, k);
      }
    }
  }
  return a;
}

Mat3f random_rotation(std::mt19937& rng) {
  std::uniform_real_distribution<float> u(-3.0F, 3.0F);
  const float a = u(rng);
  const float b = u(rng);
  const float g = u(rng);
  Mat3f rz;
  rz(0, 0) = std::cos(a);
  rz(0, 1) = -std::sin(a);
  rz(1, 0) = std::sin(a);
  rz(1, 1) = std::cos(a);
  rz(2, 2) = 1;
  Mat3f rx;
  rx(0, 0) = 1;
  rx(1, 1) = std::cos(b);
  rx(1, 2) = -std::sin(b);
  rx(2, 1) = std::sin(b);
  rx(2, 2) = std::cos(b);
  Mat3f ry;
  ry(0, 0) = std::cos(g);
  ry(0, 2) = std::sin(g);
  ry(1, 1) = 1;
  ry(2, 0) = -std::sin(g);
  ry(2, 2) = std::cos(g);
  Mat3f t;
  Mat3f out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        t(r, c) += rz(r, k) * rx(k, c);
      }
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        out(r, c) += t(r, k) * ry(k, c);
      }
    }
  }
  return out;
}

TEST(SymEigen3, RecoversKnownSpectrumAscendingAndOrthonormal) {
  std::mt19937 rng(42);
  const float spectra[][3] = {{500.0F, 0.5F, 80.0F}, {3.0F, 3.0F, 1.0F}, {0.0F, 10.0F, 1e4F}};
  for (const auto& l : spectra) {
    const Mat3f v = random_rotation(rng);
    const SymEigen3 e = sym_eigen3(from_eig(l, v));
    float sorted[3] = {l[0], l[1], l[2]};
    std::sort(sorted, sorted + 3);
    const float scale = sorted[2];
    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(e.values[k], sorted[k], 1e-5F * scale) << k;
    }
    // V^T V = I and A v_k = lambda_k v_k.
    const Mat3f a = from_eig(l, v);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        float dot = 0;
        for (int r = 0; r < 3; ++r) {
          dot += e.vectors(r, i) * e.vectors(r, j);
        }
        EXPECT_NEAR(dot, i == j ? 1.0F : 0.0F, 1e-5F);
      }
      for (int r = 0; r < 3; ++r) {
        float av = 0;
        for (int c = 0; c < 3; ++c) {
          av += a(r, c) * e.vectors(c, i);
        }
        EXPECT_NEAR(av, e.values[i] * e.vectors(r, i), 1e-4F * scale);
      }
    }
  }
}

TEST(Degeneracy, FlagsEigenvaluesBelowThresholdOnly) {
  Mat3f h;
  h(0, 0) = 2.0F;
  h(1, 1) = 300.0F;
  h(2, 2) = 900.0F;
  const DegeneracyReport r = analyse_degeneracy(h, 10.0F);
  EXPECT_EQ(r.n_degenerate, 1);
  EXPECT_TRUE(r.degenerate[0]);
  EXPECT_FALSE(r.degenerate[1]);
  EXPECT_FALSE(r.degenerate[2]);
  EXPECT_NEAR(std::abs(r.eig.vectors(0, 0)), 1.0F, 1e-6F);
  EXPECT_EQ(analyse_degeneracy(h, 0.0F).n_degenerate, 0);  // disabled
}

}  // namespace
}  // namespace mss2d
