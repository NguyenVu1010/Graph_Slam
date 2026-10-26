#include "mss/gnss/gnss.hpp"

namespace mss::gnss {

common::Status gate_fix(const common::GnssMeasurement& m, double max_cov_m2) {
  if (m.status < 0) {
    return common::Status::error("no fix");
  }
  if (!m.p_e_g.allFinite() || !m.cov.allFinite()) {
    return common::Status::error("non-finite fix");
  }
  for (int i = 0; i < 3; ++i) {
    if (m.cov(i, i) > max_cov_m2) {
      return common::Status::error("fix covariance too large");
    }
  }
  return common::Status::ok();
}

}  // namespace mss::gnss
