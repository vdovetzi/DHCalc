#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dhcalc {

using Matrix4 = Eigen::Matrix4d;

struct DHFrameParameters {
  std::string name;
  double theta_radians{};
  double alpha_radians{};
  double r{};
  double d{};
};

Matrix4 identity_matrix();
Matrix4 compute_H(const DHFrameParameters &row );
Matrix4 compute_FK(const std::vector<DHFrameParameters> &rows);
std::string format_matrix(const Matrix4 &matrix, int precision = 6);

} // namespace dhcalc
