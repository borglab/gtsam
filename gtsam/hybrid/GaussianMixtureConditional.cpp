/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureConditional.cpp
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/hybrid/GaussianMixtureConditional.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

GaussianMixtureConditional::GaussianMixtureConditional(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const GaussianMixtureConditional::Conditionals &conditionals)
    : BaseFactor(CollectKeys(continuousFrontals, continuousParents),
                 discreteParents),
      BaseConditional(continuousFrontals.size()),
      conditionals_(conditionals) {}

/* *******************************************************************************/
const GaussianMixtureConditional::Conditionals &
GaussianMixtureConditional::conditionals() {
  return conditionals_;
}

/* *******************************************************************************/
GaussianMixtureConditional GaussianMixtureConditional::FromConditionals(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const std::vector<GaussianConditional::shared_ptr> &conditionalsList) {
  Conditionals dt(discreteParents, conditionalsList);

  return GaussianMixtureConditional(continuousFrontals, continuousParents,
                                    discreteParents, dt);
}

/* *******************************************************************************/
GaussianMixtureConditional::Sum GaussianMixtureConditional::add(
    const GaussianMixtureConditional::Sum &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const Sum tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianMixtureConditional::Sum
GaussianMixtureConditional::asGaussianFactorGraphTree() const {
  auto lambda = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {conditionals_, lambda};
}

/* *******************************************************************************/
bool GaussianMixtureConditional::equals(const HybridFactor &lf,
                                        double tol) const {
  return BaseFactor::equals(lf, tol);
}

/* *******************************************************************************/
void GaussianMixtureConditional::print(const std::string &s,
                                       const KeyFormatter &formatter) const {
  std::cout << s << ": ";
  if (isContinuous()) std::cout << "Cont. ";
  if (isDiscrete()) std::cout << "Disc. ";
  if (isHybrid()) std::cout << "Hybr. ";
  BaseConditional::print("", formatter);
  std::cout << "Discrete Keys = ";
  for (auto &dk : discreteKeys()) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << "\n";
  conditionals_.print(
      "", [&](Key k) { return formatter(k); },
      [&](const GaussianConditional::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        if (!gf->empty())
          gf->print("", formatter);
        else
          return {"nullptr"};
        return rd.str();
      });
}
}  // namespace gtsam
