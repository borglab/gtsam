//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/CGMixtureFactor.h>

#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/base/utilities.h>

namespace gtsam {

CGMixtureFactor::CGMixtureFactor(const KeyVector &continuousKeys,
                                 const DiscreteKeys &discreteKeys,
                                 const Factors &factors) : Base(continuousKeys, discreteKeys),
                                                           factors_(factors) {}
bool CGMixtureFactor::equals(const HybridFactor &lf, double tol) const {
  return false;
}

void CGMixtureFactor::print(const std::string &s, const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  factors_.print(
      "mixture = ",
      [&](Key k) {
        return formatter(k);
      }, [&](const GaussianFactor::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        if (!gf->empty()) gf->print("", formatter);
        else return {"nullptr"};
        return rd.str();
      });
}

}