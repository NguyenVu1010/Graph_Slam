#include "mss/common/types.hpp"
#include "mss/scan_match/scan_matcher.hpp"

// Derivation: docs/derivations/scan_match.md, sections 1-2.
// Residual r = n_w . (T_w_l * p_l - q_w); variable xi = [rho, phi], right perturbation
// T_w_l <- T_w_l * Exp(xi); J_r_xi = [n_l ; p_l x n_l]^T with n_l = R_w_l^T n_w.
namespace mss::scan_match {

double point_to_plane_residual(const common::Vec3& p_l,
                               const common::Vec3& q_w,
                               const common::Vec3& n_w,
                               const common::Pose& T_w_l,
                               common::Vec6* J_r_xi) {
  const common::Vec3 p_w = T_w_l * p_l;
  const double r = n_w.dot(p_w - q_w);
  if (J_r_xi != nullptr) {
    const common::Vec3 n_l = T_w_l.R.transpose() * n_w;
    J_r_xi->head<3>() = n_l;
    J_r_xi->tail<3>() = p_l.cross(n_l);
  }
  return r;
}

}  // namespace mss::scan_match
