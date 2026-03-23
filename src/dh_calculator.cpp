#include "dhcalc/kinematics.hpp"
#include "internal/utility.hpp"
#include "symengine/basic.h"
#include "symengine/dict.h"
#include "symengine/real_double.h"

using SymEngine::DenseMatrix;
using SymEngine::Expression;

namespace dhcalc {

DenseMatrix identity_matrix() {
  DenseMatrix I(4, 4);
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      if (i != j) {
        I.set(i, j, SymEngine::real_double(0));
      } else {
        I.set(i, j, SymEngine::real_double(1));
      }
    }
  }
  return I;
}

DenseMatrix compute_H(const DHFrameParameters &row) {
  auto theta = row.theta;
  auto alpha = row.alpha;
  auto r = row.r;
  auto d = row.d;

  Expression cos_theta = cos(theta);
  Expression sin_theta = sin(theta);
  Expression cos_alpha = cos(alpha);
  Expression sin_alpha = sin(alpha);

  DenseMatrix T(4, 4);
  T.set(0, 0, cos_theta);
  T.set(0, 1, -sin_theta * cos_alpha);
  T.set(0, 2, sin_theta * sin_alpha);
  T.set(0, 3, r * cos_theta);
  T.set(1, 0, sin_theta);
  T.set(1, 1, cos_theta * cos_alpha);
  T.set(1, 2, -cos_theta * sin_alpha);
  T.set(1, 3, r * sin_theta);
  T.set(2, 0, SymEngine::real_double(0));
  T.set(2, 1, sin_alpha);
  T.set(2, 2, cos_alpha);
  T.set(2, 3, d);
  T.set(3, 0, SymEngine::real_double(0));
  T.set(3, 1, SymEngine::real_double(0));
  T.set(3, 2, SymEngine::real_double(0));
  T.set(3, 3, SymEngine::real_double(1));

  return T;
}

DenseMatrix compute_FK(const std::vector<DHFrameParameters> &rows) {
  DenseMatrix result = identity_matrix();

  for (const DHFrameParameters &row : rows) {
    result.mul_matrix(compute_H(row), result);
  }

  return result;
}

std::string format_matrix(const DenseMatrix &M, int precision,
                          double zero_threshold) {
  if (precision < 1) {
    throw std::invalid_argument("Precision must be >= 1.");
  }

  std::ostringstream output;

  for (size_t i = 0; i < M.nrows(); ++i) {
    output << "[";
    for (size_t j = 0; j < M.ncols(); ++j) {
      auto expr = expand(M.get(i, j));
      auto cleaned =
          utilities::clean_expression(expr, zero_threshold, precision);
      output << ' ' << cleaned->__str__();
      if (j + 1 != M.ncols()) {
        output << ',';
      }
    }
    output << " ]";

    if (i + 1 != M.nrows()) {
      output << '\n';
    }
  }

  return output.str();
}

} // namespace dhcalc
