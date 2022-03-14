//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/inference/Conditional-inst.h>

namespace gtsam {
void HybridConditional::print(const std::string &s,
                              const KeyFormatter &formatter) const {
  std::cout << s << "P(";
  int index = 0;
  const size_t N = keys().size();
  const size_t contN = N - discreteKeys_.size();
  while (index < N) {
    if (index > 0) {
      if (index == nrFrontals_) std::cout << " | "; else std::cout << ", ";
    }
    if (index < contN) {
      std::cout << formatter(keys()[index]);
    } else {
      auto &dk = discreteKeys_[index - contN];
      std::cout << "(" << formatter(dk.first) << ", " << dk.second << ")";
    }
    index++;
  }
  std::cout << ")\n";
  if (inner) inner->print("", formatter);
}

bool HybridConditional::equals(const HybridFactor &other,
                                      double tol) const {
  return false;
}

HybridConditional HybridConditional::operator*(const HybridConditional &other) const {
  return {};
}
}

