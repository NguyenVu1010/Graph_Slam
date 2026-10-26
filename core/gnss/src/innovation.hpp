#pragma once

// Compatibility aliases: the GNSS innovation helpers are public in mss/gnss/gnss.hpp
// (gnss plan Contract issue 1, approved for the pipeline PR). Existing gnss tests keep
// using the mss::gnss::detail names through this header.

#include "mss/gnss/gnss.hpp"

namespace mss::gnss::detail {

using mss::gnss::chi_square_gate;
using mss::gnss::gnss_residual;
using mss::gnss::innovation_nis;
using mss::gnss::kChi2Gate3Dof95;

}  // namespace mss::gnss::detail
