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
                                             const Mixture &factors)
    : Base(continuousKeys, discreteKeys),
      factors_(factors, [](const GaussianFactor::shared_ptr &gf) {
        return std::make_pair(gf, 0.0);
      }) {}

/* *******************************************************************************/
bool GaussianMixtureFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // This will return false if either factors_ is empty or e->factors_ is empty,
  // but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty()) return false;

  // Check the base and the factors:
  return Base::equals(*e, tol) &&
         factors_.equals(e->factors_,
                         [tol](const GaussianMixtureFactor::FactorAndLogZ &f1,
                               const GaussianMixtureFactor::FactorAndLogZ &f2) {
                           return f1.first->equals(*(f2.first), tol);
                         });
}

/* *******************************************************************************/
void GaussianMixtureFactor::print(const std::string &s,
                                  const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const GaussianMixtureFactor::FactorAndLogZ &gf_z) -> std::string {
          auto gf = gf_z.first;
          RedirectCout rd;
          std::cout << ":\n";
          if (gf && !gf->empty()) {
            gf->print("", formatter);
            return rd.str();
          } else {
            return "nullptr";
          }
        });
  }
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
const GaussianMixtureFactor::Mixture GaussianMixtureFactor::factors() {
  // Unzip to tree of Gaussian factors and tree of log-constants,
  // and return the first tree.
  return unzip(factors_).first;
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
  auto wrap = [](const GaussianMixtureFactor::FactorAndLogZ &factor_z) {
    GaussianFactorGraph result;
    result.push_back(factor_z.first);
    return result;
  };
  return {factors_, wrap};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixtureFactor::error(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc =
      [continuousValues](const GaussianMixtureFactor::FactorAndLogZ &factor_z) {
        GaussianFactor::shared_ptr factor;
        double log_z;
        std::tie(factor, log_z) = factor_z;
        return factor->error(continuousValues) + log_z;
      };
  DecisionTree<Key, double> errorTree(factors_, errorFunc);
  return errorTree;
}

/* *******************************************************************************/
double GaussianMixtureFactor::error(
    const VectorValues &continuousValues,
    const DiscreteValues &discreteValues) const {
  // Directly index to get the conditional, no need to build the whole tree.
  GaussianFactor::shared_ptr factor;
  double log_z;
  std::tie(factor, log_z) = factors_(discreteValues);
  return factor->error(continuousValues) + log_z;
}

}  // namespace gtsam
