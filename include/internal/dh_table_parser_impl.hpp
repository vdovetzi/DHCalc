#pragma once

#include "dhcalc/kinematics.hpp"
#include <istream>
#include <vector>

namespace dhcalc {
namespace detail {

// Parses DH table from any input stream. May throw ParseError.
std::vector<DHFrameParameters> parse_dh_table_impl(std::istream &input);

} // namespace detail
} // namespace dhcalc