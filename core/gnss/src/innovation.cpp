#include "mss/gnss/gnss.hpp"  // own header first (CLAUDE.md)
// The file name differs from the header name, so clang-format cannot detect the main
// include; this comment keeps it in its own block.

#include <Eigen/Cholesky>

#include "mss/common/lie.hpp"

namespace mss::gnss {

namespace {

// Writes r^T S^-1 r to *nis and returns nullptr, or returns the error reason (a string
// literal, so callers wrap it without allocating).
const char* compute_nis(const common::Vec3& r_e, const common::Mat3& s_e, double* nis) {
  if (!r_e.allFinite() || !s_e.allFinite()) {
    return "non-finite innovation";
  }
  const Eigen::LLT<common::Mat3> llt(s_e);
  if (llt.info() != Eigen::Success) {
    return "innovation covariance not positive definite";
  }
  const common::Vec3 s_inv_r = llt.solve(r_e);
  *nis = r_e.dot(s_inv_r);
  return nullptr;
}

}  // namespace

common::Vec3 gnss_residual(const common::Vec3& p_e_g,
                           const common::Pose& T_e_w,
                           const common::Pose& T_w_b,
                           const common::Vec3& p_b_g,
                           Eigen::Matrix<double, 3, 6>* J_r_xi) {
  const common::Vec3 p_w_g = T_w_b * p_b_g;
  const common::Vec3 r = p_e_g - T_e_w * p_w_g;
  if (J_r_xi != nullptr) {
    const common::Mat3 R_e_b = T_e_w.R * T_w_b.R;
    J_r_xi->block<3, 3>(0, 0) = -R_e_b;
    J_r_xi->block<3, 3>(0, 3) = R_e_b * common::skew(p_b_g);
  }
  return r;
}

common::Result<double> innovation_nis(const common::Vec3& r_e, const common::Mat3& s_e) {
  double nis = 0.0;
  const char* reason = compute_nis(r_e, s_e, &nis);
  if (reason != nullptr) {
    return common::Result<double>::error(reason);
  }
  return common::Result<double>::ok(nis);
}

common::Status chi_square_gate(const common::Vec3& r_e, const common::Mat3& s_e, double threshold) {
  double nis = 0.0;
  const char* reason = compute_nis(r_e, s_e, &nis);
  if (reason != nullptr) {
    return common::Status::error(reason);
  }
  if (nis > threshold) {
    return common::Status::error("chi-square gate exceeded");
  }
  return common::Status::ok();
}

}  // namespace mss::gnss
