#pragma once

#include "symengine/expression.h"

namespace dhcalc {

struct DHFrameParameters {
  std::string name;
  SymEngine::Expression theta;
  SymEngine::Expression alpha;
  SymEngine::Expression r;
  SymEngine::Expression d;
};

SymEngine::DenseMatrix identity_matrix();
SymEngine::DenseMatrix compute_H(const DHFrameParameters &row);
SymEngine::DenseMatrix compute_FK(const std::vector<DHFrameParameters> &rows);
std::string format_matrix(const SymEngine::DenseMatrix &M, int precision = 6,
                          double zero_threshold = 1e-10);
} // namespace dhcalc
