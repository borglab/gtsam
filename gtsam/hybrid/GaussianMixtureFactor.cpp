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
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* *******************************************************************************/
GaussianMixtureFactor::GaussianMixtureFactor(const KeyVector &continuousKeys,
                                             const DiscreteKeys &discreteKeys,
                                             const Mixture &factors)
    : Base(continuousKeys, discreteKeys),
      factors_(factors, [](const GaussianFactor::shared_ptr &gf) {
        return FactorAndConstant{gf, 0.0};
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
         factors_.equals(e->factors_, [tol](const FactorAndConstant &f1,
                                            const FactorAndConstant &f2) {
           return f1.factor->equals(*(f2.factor), tol) &&
                  std::abs(f1.constant - f2.constant) < tol;
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
        [&](const FactorAndConstant &gf_z) -> std::string {
          auto gf = gf_z.factor;
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
const GaussianMixtureFactor::Mixture GaussianMixtureFactor::factors() const {
  return Mixture(factors_, [](const FactorAndConstant &factor_z) {
    return factor_z.factor;
  });
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
  auto wrap = [](const FactorAndConstant &factor_z) {
    GaussianFactorGraph result;
    result.push_back(factor_z.factor);
    return result;
  };
  return {factors_, wrap};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixtureFactor::error(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc = [continuousValues](const FactorAndConstant &factor_z) {
    return factor_z.error(continuousValues);
  };
  DecisionTree<Key, double> errorTree(factors_, errorFunc);
  return errorTree;
}

/* *******************************************************************************/
double GaussianMixtureFactor::error(const HybridValues &values) const {
  const FactorAndConstant factor_z = factors_(values.discrete());
  return factor_z.error(values.continuous());
}
/* *******************************************************************************/

}  // namespace gtsam
