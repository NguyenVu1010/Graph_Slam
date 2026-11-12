#include "mss2d/degeneracy.hpp"

#include <cmath>

namespace mss2d {

SymEigen3 sym_eigen3(const Mat3f& in) {
  // Cyclic Jacobi (Numerical Recipes 11.1): rotate away each off-diagonal entry in turn;
  // A <- P^T A P, V <- V P with the plane rotation P in (p, q).
  float a[3][3];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      a[r][c] = r <= c ? in(r, c) : in(c, r);
    }
  }
  SymEigen3 out;
  Mat3f& v = out.vectors;
  v(0, 0) = 1.0F;
  v(1, 1) = 1.0F;
  v(2, 2) = 1.0F;
  constexpr int kPairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
  constexpr int kMaxSweeps = 16;
  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    const float off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
    const float diag = std::abs(a[0][0]) + std::abs(a[1][1]) + std::abs(a[2][2]);
    if (off <= 1e-9F * diag) {  // below float resolution of the diagonal: done
      break;
    }
    for (const auto& pq : kPairs) {
      const int p = pq[0];
      const int q = pq[1];
      const float apq = a[p][q];
      if (apq == 0.0F) {
        continue;
      }
      const float theta = (a[q][q] - a[p][p]) / (2.0F * apq);
      // t = tan(phi), the smaller root; for huge theta, t ~ 1 / (2 theta).
      const float t = std::abs(theta) > 1e15F
                          ? 0.5F / theta
                          : (theta >= 0.0F ? 1.0F : -1.0F) /
                                (std::abs(theta) + std::sqrt(theta * theta + 1.0F));
      const float c = 1.0F / std::sqrt(t * t + 1.0F);
      const float s = t * c;
      for (int k = 0; k < 3; ++k) {  // A P
        const float akp = a[k][p];
        const float akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (int k = 0; k < 3; ++k) {  // P^T (A P)
        const float apk = a[p][k];
        const float aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
      }
      a[p][q] = 0.0F;
      a[q][p] = 0.0F;
      for (int k = 0; k < 3; ++k) {  // V P
        const float vkp = v(k, p);
        const float vkq = v(k, q);
        v(k, p) = c * vkp - s * vkq;
        v(k, q) = s * vkp + c * vkq;
      }
    }
  }
  // Sort ascending (3 elements: insertion sort), moving the columns along.
  int order[3] = {0, 1, 2};
  for (int i = 1; i < 3; ++i) {
    for (int j = i; j > 0 && a[order[j]][order[j]] < a[order[j - 1]][order[j - 1]]; --j) {
      const int tmp = order[j];
      order[j] = order[j - 1];
      order[j - 1] = tmp;
    }
  }
  const Mat3f unsorted = v;
  for (int k = 0; k < 3; ++k) {
    out.values[k] = a[order[k]][order[k]];
    for (int r = 0; r < 3; ++r) {
      v(r, k) = unsorted(r, order[k]);
    }
  }
  return out;
}

DegeneracyReport analyse_degeneracy(const Mat3f& h, float eig_threshold) {
  DegeneracyReport r;
  r.eig = sym_eigen3(h);
  for (int k = 0; k < 3; ++k) {
    r.degenerate[k] = eig_threshold > 0.0F && r.eig.values[k] < eig_threshold;
    r.n_degenerate += r.degenerate[k] ? 1 : 0;
  }
  return r;
}

}  // namespace mss2d
