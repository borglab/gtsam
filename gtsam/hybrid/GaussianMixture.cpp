/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.cpp
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

GaussianMixture::GaussianMixture(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const GaussianMixture::Conditionals &conditionals)
    : BaseFactor(CollectKeys(continuousFrontals, continuousParents),
                 discreteParents),
      BaseConditional(continuousFrontals.size()),
      conditionals_(conditionals) {}

const GaussianMixture::Conditionals& GaussianMixture::conditionals() {
  return conditionals_;
}

/* *******************************************************************************/
GaussianMixture::Sum GaussianMixture::addTo(const GaussianMixture::Sum &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const Sum wrapped = wrappedConditionals();
  return sum.empty() ? wrapped : sum.apply(wrapped, add);
}

/* *******************************************************************************/
GaussianMixture::Sum GaussianMixture::wrappedConditionals() const {
  auto wrap = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {conditionals_, wrap};
}

bool GaussianMixture::equals(const HybridFactor &lf, double tol) const {
  return false;
}

void GaussianMixture::print(const std::string &s,
                                  const KeyFormatter &formatter) const {
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