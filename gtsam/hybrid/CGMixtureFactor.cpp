/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CGMixtureFactor.cpp
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/hybrid/CGMixtureFactor.h>

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/linear/GaussianFactorGraph.h>
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

const CGMixtureFactor::Factors& CGMixtureFactor::factors() {
  return factors_;
}

/* *******************************************************************************/
CGMixtureFactor::Sum CGMixtureFactor::addTo(const CGMixtureFactor::Sum &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const Sum wrapped = wrappedFactors();
  return sum.empty() ? wrapped : sum.apply(wrapped, add);
}

/* *******************************************************************************/
CGMixtureFactor::Sum CGMixtureFactor::wrappedFactors() const {
  auto wrap = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {factors_, wrap};
}
}