/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CLGaussianConditional.cpp
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @date   Mar 12, 2022
 */

#include <gtsam/hybrid/CLGaussianConditional.h>

#include <gtsam/base/utilities.h>

#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/discrete/DecisionTree-inl.h>

namespace gtsam {

CLGaussianConditional::CLGaussianConditional(const KeyVector &continuousFrontals,
                                             const KeyVector &continuousParents,
                                             const DiscreteKeys &discreteParents,
                                             const CLGaussianConditional::Conditionals &conditionals)
    : BaseFactor(
    CollectKeys(continuousFrontals, continuousParents), discreteParents),
      BaseConditional(continuousFrontals.size()), conditionals_(conditionals) {

}

bool CLGaussianConditional::equals(const HybridFactor &lf, double tol) const {
  return false;
}

void CLGaussianConditional::print(const std::string &s, const KeyFormatter &formatter) const {
  std::cout << s << ": ";
  if (isContinuous_) std::cout << "Cont. ";
  if (isDiscrete_) std::cout << "Disc. ";
  if (isHybrid_) std::cout << "Hybr. ";
  BaseConditional::print("", formatter);
  std::cout << "Discrete Keys = ";
  for (auto &dk : discreteKeys_) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << "\n";
  conditionals_.print(
      "",
      [&](Key k) {
        return formatter(k);
      }, [&](const GaussianConditional::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        if (!gf->empty()) gf->print("", formatter);
        else return {"nullptr"};
        return rd.str();
      });
}
}