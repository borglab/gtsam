/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureFactor.cpp
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* *******************************************************************************/
GaussianMixtureFactor::GaussianMixtureFactor(const KeyVector &continuousKeys,
                                             const DiscreteKeys &discreteKeys,
                                             const Factors &factors)
    : Base(continuousKeys, discreteKeys), factors_(factors) {}

/* *******************************************************************************/
bool GaussianMixtureFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  return e != nullptr && Base::equals(*e, tol);
}

/* *******************************************************************************/
GaussianMixtureFactor GaussianMixtureFactor::FromFactors(
    const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys,
    const std::vector<GaussianFactor::shared_ptr> &factors) {
  Factors dt(discreteKeys, factors);

  return GaussianMixtureFactor(continuousKeys, discreteKeys, dt);
}

/* *******************************************************************************/
void GaussianMixtureFactor::print(const std::string &s,
                                  const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  std::cout << "{\n";
  factors_.print(
      "", [&](Key k) { return formatter(k); },
      [&](const GaussianFactor::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        std::cout << ":\n";
        if (gf && !gf->empty()) {
          gf->print("", formatter);
          return rd.str();
        } else {
          return "nullptr";
        }
      });
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
const GaussianMixtureFactor::Factors &GaussianMixtureFactor::factors() {
  return factors_;
}

/* *******************************************************************************/
GaussianMixtureFactor::Sum GaussianMixtureFactor::add(
    const GaussianMixtureFactor::Sum &sum) const {
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
GaussianMixtureFactor::Sum GaussianMixtureFactor::asGaussianFactorGraphTree()
    const {
  auto wrap = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {factors_, wrap};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixtureFactor::error(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc =
      [continuousValues](const GaussianFactor::shared_ptr &factor) {
        return factor->error(continuousValues);
      };
  DecisionTree<Key, double> errorTree(factors_, errorFunc);
  return errorTree;
}

/* *******************************************************************************/
double GaussianMixtureFactor::error(
    const VectorValues &continuousValues,
    const DiscreteValues &discreteValues) const {
  auto factor = factors_(discreteValues);
  return factor->error(continuousValues);
}

}  // namespace gtsam
