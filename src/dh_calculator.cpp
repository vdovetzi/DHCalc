#include "dhcalc/kinematics.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace dhcalc {

Matrix4 identity_matrix() { return Matrix4::Identity(); }

Matrix4 compute_H(const DHFrameParameters &row) {
  const double ct = std::cos(row.theta_radians);
  const double st = std::sin(row.theta_radians);
  const double ca = std::cos(row.alpha_radians);
  const double sa = std::sin(row.alpha_radians);

  Matrix4 H;
  H << ct, -st * ca, st * sa, row.r * ct, st, ct * ca, -ct * sa, row.r * st,
      0.0, sa, ca, row.d, 0.0, 0.0, 0.0, 1.0;
  return H;
}

Matrix4 compute_FK(const std::vector<DHFrameParameters> &rows) {
  Matrix4 result = Matrix4::Identity();

  for (const DHFrameParameters &row : rows) {
    result *= compute_H(row);
  }

  return result;
}

std::string format_matrix(const Matrix4 &matrix, int precision) {
  if (precision < 0) {
    throw std::invalid_argument("Precision must be non-negative.");
  }

  const int width = precision + 8;
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision);

  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    output << "[";
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      output << ' ' << std::setw(width) << matrix(row, col);
    }
    output << " ]";

    if (row + 1 != matrix.rows()) {
      output << '\n';
    }
  }

  return output.str();
}

} // namespace dhcalc
