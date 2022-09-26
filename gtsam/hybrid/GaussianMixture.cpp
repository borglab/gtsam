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
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteValues.h>
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

/* *******************************************************************************/
const GaussianMixture::Conditionals &GaussianMixture::conditionals() {
  return conditionals_;
}

/* *******************************************************************************/
GaussianMixture GaussianMixture::FromConditionals(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const std::vector<GaussianConditional::shared_ptr> &conditionalsList) {
  Conditionals dt(discreteParents, conditionalsList);

  return GaussianMixture(continuousFrontals, continuousParents, discreteParents,
                         dt);
}

/* *******************************************************************************/
GaussianMixture::Sum GaussianMixture::add(
    const GaussianMixture::Sum &sum) const {
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
GaussianMixture::Sum GaussianMixture::asGaussianFactorGraphTree() const {
  auto lambda = [](const GaussianFactor::shared_ptr &factor) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return result;
  };
  return {conditionals_, lambda};
}

/* *******************************************************************************/
size_t GaussianMixture::nrComponents() const {
  size_t total = 0;
  conditionals_.visit([&total](const GaussianFactor::shared_ptr &node) {
    if (node) total += 1;
  });
  return total;
}

/* *******************************************************************************/
GaussianConditional::shared_ptr GaussianMixture::operator()(
    const DiscreteValues &discreteVals) const {
  auto &ptr = conditionals_(discreteVals);
  if (!ptr) return nullptr;
  auto conditional = boost::dynamic_pointer_cast<GaussianConditional>(ptr);
  if (conditional)
    return conditional;
  else
    throw std::logic_error(
        "A GaussianMixture unexpectedly contained a non-conditional");
}

/* *******************************************************************************/
bool GaussianMixture::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  return e != nullptr && BaseFactor::equals(*e, tol);
}

/* *******************************************************************************/
void GaussianMixture::print(const std::string &s,
                            const KeyFormatter &formatter) const {
  std::cout << s;
  if (isContinuous()) std::cout << "Continuous ";
  if (isDiscrete()) std::cout << "Discrete ";
  if (isHybrid()) std::cout << "Hybrid ";
  BaseConditional::print("", formatter);
  std::cout << " Discrete Keys = ";
  for (auto &dk : discreteKeys()) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << "\n";
  conditionals_.print(
      "", [&](Key k) { return formatter(k); },
      [&](const GaussianConditional::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        if (gf && !gf->empty())
          gf->print("", formatter);
        else
          return {"nullptr"};
        return rd.str();
      });
}

/* *******************************************************************************/
void GaussianMixture::prune(const DecisionTreeFactor &decisionTree) {
  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner = [&decisionTree](
                    const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    DiscreteValues values(choices);

    if (decisionTree(values) == 0.0) {
      // empty aka null pointer
      boost::shared_ptr<GaussianConditional> null;
      return null;
    } else {
      return conditional;
    }
  };

  auto pruned_conditionals = conditionals_.apply(pruner);
  conditionals_.root_ = pruned_conditionals.root_;
}

}  // namespace gtsam
