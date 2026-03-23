#pragma once

#include <filesystem>

#include "dhcalc/kinematics.hpp"

namespace dhcalc {

class ParseError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

std::vector<DHFrameParameters>
parse_dh_table(const std::filesystem::path &path);

} // namespace dhcalc
