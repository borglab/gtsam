//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/CGMixtureFactor.h>

namespace gtsam {

CGMixtureFactor::CGMixtureFactor(const KeyVector &continuousKeys,
                                 const DiscreteKeys &discreteKeys,
                                 const Factors &factors) : Base(continuousKeys, discreteKeys),
                                                           factors_(factors) {}
bool CGMixtureFactor::equals(const HybridFactor &lf, double tol) const {
  return false;
}

}