#include "mss/loop/loop_selector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Cholesky>

#include "mss/common/lie.hpp"

// Design: ADR-028; derivation: docs/derivations/loop_closure.md section 11.
namespace mss::loop {

namespace {

using common::Mat6;
using common::Pose;
using common::PoseCovariance;
using common::Vec6;

int popcount(std::uint64_t v) {
  int n = 0;
  while (v != 0U) {
    v &= v - 1U;
    ++n;
  }
  return n;
}

int lowest_bit(std::uint64_t v) {
  int b = 0;
  while ((v & 1U) == 0U) {
    v >>= 1U;
    ++b;
  }
  return b;
}

std::uint64_t bit(std::size_t s) {
  return std::uint64_t{1} << s;
}

// A pose with the covariance of its right perturbation ([rho, phi]).
struct Uncertain {
  Pose T;
  PoseCovariance C = PoseCovariance::Zero();
};

// (A, Ca) * (B, Cb): T = A B, C = Ad(B^-1) Ca Ad(B^-1)^T + Cb.
Uncertain compose(const Uncertain& a, const Uncertain& b) {
  Uncertain out;
  out.T = a.T * b.T;
  const PoseCovariance Ca_in_b = common::transform_covariance(a.C, b.T.inverse());
  out.C = Ca_in_b + b.C;
  return out;
}

// (A, C)^-1: T = A^-1, C' = Ad(A) C Ad(A)^T ((A Exp(d))^-1 = A^-1 Exp(-Ad(A) d)).
Uncertain inverse(const Uncertain& a) {
  Uncertain out;
  out.T = a.T.inverse();
  out.C = common::transform_covariance(a.C, a.T);
  return out;
}

bool pose_finite(const Pose& T) {
  return T.R.allFinite() && T.p.allFinite();
}

bool spd(const PoseCovariance& C) {
  if (!C.allFinite()) {
    return false;
  }
  const PoseCovariance sym = 0.5 * (C + C.transpose());
  const Eigen::LLT<Mat6> llt(sym);
  return llt.info() == Eigen::Success;
}

}  // namespace

LoopSelector::LoopSelector(const LoopConfig& cfg, std::size_t reserve_keyframes) : cfg_(cfg) {
  if (!cfg.enabled || !cfg.pcm_enabled) {
    return;
  }
  if (!(cfg.pcm_chi2_gate > 0.0) || !std::isfinite(cfg.pcm_chi2_gate)) {
    throw std::invalid_argument("loop_closure.pcm_chi2_gate must be > 0 and finite");
  }
  if (cfg.pcm_max_loops < 1 || cfg.pcm_max_loops > static_cast<int>(kMaxLoops)) {
    throw std::invalid_argument("loop_closure.pcm_max_loops must be in [1, 64]");
  }
  if (!(cfg.pcm_odometry_sigma_scale >= 1.0) || !std::isfinite(cfg.pcm_odometry_sigma_scale)) {
    throw std::invalid_argument("loop_closure.pcm_odometry_sigma_scale must be >= 1 and finite");
  }
  if (cfg.pcm_max_clique_steps <= 0) {
    throw std::invalid_argument("loop_closure.pcm_max_clique_steps must be > 0");
  }
  T_0k_.reserve(reserve_keyframes);
  S_k_.reserve(reserve_keyframes);
  T_0k_.emplace_back();  // keyframe 0: the origin of the chain
  S_k_.push_back(PoseCovariance::Zero());
  to_add_.reserve(kMaxLoops);
  to_remove_.reserve(kMaxLoops);
}

bool LoopSelector::add_odometry(common::KeyId k, const Pose& T_prev_k, const PoseCovariance& cov) {
  if (T_0k_.empty() || k != T_0k_.size() || !pose_finite(T_prev_k) || !cov.allFinite()) {
    return false;
  }
  const Pose T = T_0k_.back() * T_prev_k;
  const double s2 = cfg_.pcm_odometry_sigma_scale * cfg_.pcm_odometry_sigma_scale;
  const PoseCovariance left = common::transform_covariance(s2 * 0.5 * (cov + cov.transpose()), T);
  T_0k_.push_back(T);
  S_k_.push_back(S_k_.back() + left);
  return true;
}

bool LoopSelector::odometry(common::KeyId a,
                            common::KeyId b,
                            Pose& T_ab,
                            PoseCovariance& cov) const {
  if (a >= T_0k_.size() || b >= T_0k_.size()) {
    return false;
  }
  const common::KeyId lo = a < b ? a : b;
  const common::KeyId hi = a < b ? b : a;
  Uncertain u;
  u.T = T_0k_[lo].inverse() * T_0k_[hi];
  const PoseCovariance d = S_k_[hi] - S_k_[lo];
  u.C = common::transform_covariance(0.5 * (d + d.transpose()), T_0k_[hi].inverse());
  if (a > b) {
    u = inverse(u);
  }
  T_ab = u.T;
  cov = u.C;
  return true;
}

double LoopSelector::pair_chi2(const common::LoopConstraint& a,
                               const common::LoopConstraint& b) const {
  Uncertain o_i;
  Uncertain o_j;
  if (!odometry(a.i, b.i, o_i.T, o_i.C) || !odometry(b.j, a.j, o_j.T, o_j.C)) {
    return std::numeric_limits<double>::infinity();
  }
  const Uncertain za{a.T_bi_bj, a.cov};
  const Uncertain zb{b.T_bi_bj, b.cov};
  const Uncertain x = compose(compose(compose(inverse(za), o_i), zb), o_j);
  const Vec6 e = common::se3_log(x.T);
  const Mat6 S = 0.5 * (x.C + x.C.transpose());
  const Eigen::LDLT<Mat6> ldlt(S);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return std::numeric_limits<double>::infinity();
  }
  const Vec6 S_inv_e = ldlt.solve(e);
  return e.dot(S_inv_e);
}

LoopSelector::Decision LoopSelector::propose(const common::LoopConstraint& lc) {
  Decision d;
  to_add_.clear();
  to_remove_.clear();
  if (T_0k_.empty() || !(lc.i < lc.j) || lc.j >= T_0k_.size() || !pose_finite(lc.T_bi_bj) ||
      !spd(lc.cov)) {
    return d;
  }
  // A free slot, or the oldest loop leaves PCM (frozen in the graph when selected).
  const std::size_t cap = static_cast<std::size_t>(cfg_.pcm_max_loops);
  std::size_t slot = cap;
  for (std::size_t s = 0; s < cap; ++s) {
    if ((used_ & bit(s)) == 0U) {
      slot = s;
      break;
    }
  }
  if (slot == cap) {
    slot = 0;
    for (std::size_t s = 1; s < cap; ++s) {
      if (age_[s] < age_[slot]) {
        slot = s;
      }
    }
    d.frozen = (selected_ & bit(slot)) != 0U;
    used_ &= ~bit(slot);
    selected_ &= ~bit(slot);
    for (std::size_t s = 0; s < cap; ++s) {
      adj_[s] &= ~bit(slot);
    }
  }
  loops_[slot] = lc;
  age_[slot] = counter_++;
  adj_[slot] = 0U;
  for (std::size_t s = 0; s < cap; ++s) {
    if ((used_ & bit(s)) == 0U) {
      continue;
    }
    if (pair_chi2(loops_[s], lc) <= cfg_.pcm_chi2_gate) {
      adj_[slot] |= bit(s);
      adj_[s] |= bit(slot);
      ++d.n_consistent;
    }
  }
  used_ |= bit(slot);
  const std::uint64_t before = selected_;
  select();
  const std::uint64_t entered = selected_ & ~before;
  const std::uint64_t left = before & ~selected_;
  // Oldest first, so the graph sees loops in their detection order.
  for (std::uint64_t m = entered; m != 0U;) {
    std::size_t s = static_cast<std::size_t>(lowest_bit(m));
    for (std::uint64_t r = m; r != 0U; r &= r - 1U) {
      const std::size_t t = static_cast<std::size_t>(lowest_bit(r));
      if (age_[t] < age_[s]) {
        s = t;
      }
    }
    to_add_.push_back(loops_[s]);
    m &= ~bit(s);
  }
  for (std::uint64_t m = left; m != 0U; m &= m - 1U) {
    const std::size_t s = static_cast<std::size_t>(lowest_bit(m));
    to_remove_.emplace_back(loops_[s].i, loops_[s].j);
  }
  d.selected = (selected_ & bit(slot)) != 0U;
  d.clique_size = popcount(selected_);
  return d;
}

// Maximum clique by Bron-Kerbosch with pivoting over the used slots; best_ ranks by size,
// then overlap with the current selection, then the oldest loop (smallest age sum). The search
// starts from the current selection: it is still a clique (adjacency among held loops never
// changes, an evicted slot is cleared from it), so a search cut short by pcm_max_clique_steps
// can only improve on it (review H3 m-2).
void LoopSelector::select() {
  best_ = selected_ & used_;
  steps_ = 0;
  clique(0U, used_, 0U);
  selected_ = best_;
}

void LoopSelector::clique(std::uint64_t r, std::uint64_t p, std::uint64_t x) {
  if (++steps_ > cfg_.pcm_max_clique_steps) {
    return;  // bound reached: keep the best clique found so far (ADR-028)
  }
  if (p == 0U && x == 0U) {
    const int n = popcount(r);
    const int nb = popcount(best_);
    bool better = n > nb;
    if (n == nb) {
      const int keep = popcount(r & selected_);
      const int keep_b = popcount(best_ & selected_);
      if (keep != keep_b) {
        better = keep > keep_b;
      } else {
        std::uint64_t age_r = 0;
        std::uint64_t age_b = 0;
        for (std::uint64_t m = r; m != 0U; m &= m - 1U) {
          age_r += age_[static_cast<std::size_t>(lowest_bit(m))];
        }
        for (std::uint64_t m = best_; m != 0U; m &= m - 1U) {
          age_b += age_[static_cast<std::size_t>(lowest_bit(m))];
        }
        better = age_r < age_b;
      }
    }
    if (better || best_ == 0U) {
      best_ = r;
    }
    return;
  }
  // Pivot: the vertex of P | X with the most neighbours in P.
  const std::uint64_t px = p | x;
  int pivot = lowest_bit(px);
  int most = -1;
  for (std::uint64_t m = px; m != 0U; m &= m - 1U) {
    const int u = lowest_bit(m);
    const int c = popcount(p & adj_[static_cast<std::size_t>(u)]);
    if (c > most) {
      most = c;
      pivot = u;
    }
  }
  for (std::uint64_t m = p & ~adj_[static_cast<std::size_t>(pivot)]; m != 0U; m &= m - 1U) {
    const int v = lowest_bit(m);
    const std::uint64_t vb = bit(static_cast<std::size_t>(v));
    const std::uint64_t nv = adj_[static_cast<std::size_t>(v)];
    clique(r | vb, p & nv, x & nv);
    p &= ~vb;
    x |= vb;
  }
}

bool LoopSelector::forget(common::KeyId i, common::KeyId j) {
  std::size_t slot = kMaxLoops;
  for (std::size_t s = 0; s < kMaxLoops; ++s) {
    if ((used_ & bit(s)) != 0U && loops_[s].i == i && loops_[s].j == j &&
        (slot == kMaxLoops || age_[s] > age_[slot])) {
      slot = s;
    }
  }
  if (slot == kMaxLoops) {
    return false;
  }
  used_ &= ~bit(slot);
  selected_ &= ~bit(slot);
  for (std::size_t s = 0; s < kMaxLoops; ++s) {
    adj_[s] &= ~bit(slot);
  }
  adj_[slot] = 0U;
  return true;
}

std::size_t LoopSelector::n_loops() const {
  return static_cast<std::size_t>(popcount(used_));
}

std::size_t LoopSelector::n_selected() const {
  return static_cast<std::size_t>(popcount(selected_));
}

}  // namespace mss::loop
