//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/inference/Conditional-inst.h>

namespace gtsam {
void HybridConditional::print(const std::string &s,
                                     const KeyFormatter &formatter) const {
  Conditional::print(s, formatter);
}

bool HybridConditional::equals(const HybridFactor &other,
                                      double tol) const {
  return false;
}

HybridConditional HybridConditional::operator*(const HybridConditional &other) const {
  return {};
}
}

