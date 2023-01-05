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
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridValues.h>
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
const GaussianMixture::Conditionals &GaussianMixture::conditionals() const {
  return conditionals_;
}

/* *******************************************************************************/
GaussianMixture::GaussianMixture(
    KeyVector &&continuousFrontals, KeyVector &&continuousParents,
    DiscreteKeys &&discreteParents,
    std::vector<GaussianConditional::shared_ptr> &&conditionals)
    : GaussianMixture(continuousFrontals, continuousParents, discreteParents,
                      Conditionals(discreteParents, conditionals)) {}

/* *******************************************************************************/
GaussianFactorGraphTree GaussianMixture::add(
    const GaussianFactorGraphTree &sum) const {
  using Y = GraphAndConstant;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1.graph;
    result.push_back(graph2.graph);
    return Y(result, graph1.constant + graph2.constant);
  };
  const auto tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianFactorGraphTree GaussianMixture::asGaussianFactorGraphTree() const {
  auto lambda = [](const GaussianConditional::shared_ptr &conditional) {
    GaussianFactorGraph result;
    result.push_back(conditional);
    if (conditional) {
      return GraphAndConstant(result, conditional->logNormalizationConstant());
    } else {
      return GraphAndConstant(result, 0.0);
    }
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
    const DiscreteValues &discreteValues) const {
  auto &ptr = conditionals_(discreteValues);
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
  if (e == nullptr) return false;

  // This will return false if either conditionals_ is empty or e->conditionals_
  // is empty, but not if both are empty or both are not empty:
  if (conditionals_.empty() ^ e->conditionals_.empty()) return false;

  // Check the base and the factors:
  return BaseFactor::equals(*e, tol) &&
         conditionals_.equals(e->conditionals_,
                              [tol](const GaussianConditional::shared_ptr &f1,
                                    const GaussianConditional::shared_ptr &f2) {
                                return f1->equals(*(f2), tol);
                              });
}

/* *******************************************************************************/
void GaussianMixture::print(const std::string &s,
                            const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
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
        if (gf && !gf->empty()) {
          gf->print("", formatter);
          return rd.str();
        } else {
          return "nullptr";
        }
      });
}

/* ************************************************************************* */
KeyVector GaussianMixture::continuousParents() const {
  // Get all parent keys:
  const auto range = parents();
  KeyVector continuousParentKeys(range.begin(), range.end());
  // Loop over all discrete keys:
  for (const auto &discreteKey : discreteKeys()) {
    const Key key = discreteKey.first;
    // remove that key from continuousParentKeys:
    continuousParentKeys.erase(std::remove(continuousParentKeys.begin(),
                                           continuousParentKeys.end(), key),
                               continuousParentKeys.end());
  }
  return continuousParentKeys;
}

/* ************************************************************************* */
boost::shared_ptr<GaussianMixtureFactor> GaussianMixture::likelihood(
    const VectorValues &frontals) const {
  // Check that values has all frontals
  for (auto &&kv : frontals) {
    if (frontals.find(kv.first) == frontals.end()) {
      throw std::runtime_error("GaussianMixture: frontals missing factor key.");
    }
  }

  const DiscreteKeys discreteParentKeys = discreteKeys();
  const KeyVector continuousParentKeys = continuousParents();
  const GaussianMixtureFactor::Factors likelihoods(
      conditionals_, [&](const GaussianConditional::shared_ptr &conditional) {
        return GaussianMixtureFactor::FactorAndConstant{
            conditional->likelihood(frontals),
            conditional->logNormalizationConstant()};
      });
  return boost::make_shared<GaussianMixtureFactor>(
      continuousParentKeys, discreteParentKeys, likelihoods);
}

/* ************************************************************************* */
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &discreteKeys) {
  std::set<DiscreteKey> s;
  s.insert(discreteKeys.begin(), discreteKeys.end());
  return s;
}

/* ************************************************************************* */
/**
 * @brief Helper function to get the pruner functional.
 *
 * @param decisionTree The probability decision tree of only discrete keys.
 * @return std::function<GaussianConditional::shared_ptr(
 * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
 */
std::function<GaussianConditional::shared_ptr(
    const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
GaussianMixture::prunerFunc(const DecisionTreeFactor &decisionTree) {
  // Get the discrete keys as sets for the decision tree
  // and the gaussian mixture.
  auto decisionTreeKeySet = DiscreteKeysAsSet(decisionTree.discreteKeys());
  auto gaussianMixtureKeySet = DiscreteKeysAsSet(this->discreteKeys());

  auto pruner = [decisionTree, decisionTreeKeySet, gaussianMixtureKeySet](
                    const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    const DiscreteValues values(choices);

    // Case where the gaussian mixture has the same
    // discrete keys as the decision tree.
    if (gaussianMixtureKeySet == decisionTreeKeySet) {
      if (decisionTree(values) == 0.0) {
        // empty aka null pointer
        boost::shared_ptr<GaussianConditional> null;
        return null;
      } else {
        return conditional;
      }
    } else {
      std::vector<DiscreteKey> set_diff;
      std::set_difference(decisionTreeKeySet.begin(), decisionTreeKeySet.end(),
                          gaussianMixtureKeySet.begin(),
                          gaussianMixtureKeySet.end(),
                          std::back_inserter(set_diff));

      const std::vector<DiscreteValues> assignments =
          DiscreteValues::CartesianProduct(set_diff);
      for (const DiscreteValues &assignment : assignments) {
        DiscreteValues augmented_values(values);
        augmented_values.insert(assignment);

        // If any one of the sub-branches are non-zero,
        // we need this conditional.
        if (decisionTree(augmented_values) > 0.0) {
          return conditional;
        }
      }
      // If we are here, it means that all the sub-branches are 0,
      // so we prune.
      return nullptr;
    }
  };
  return pruner;
}

/* *******************************************************************************/
void GaussianMixture::prune(const DecisionTreeFactor &decisionTree) {
  auto decisionTreeKeySet = DiscreteKeysAsSet(decisionTree.discreteKeys());
  auto gmKeySet = DiscreteKeysAsSet(this->discreteKeys());
  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner = prunerFunc(decisionTree);

  auto pruned_conditionals = conditionals_.apply(pruner);
  conditionals_.root_ = pruned_conditionals.root_;
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixture::error(
    const VectorValues &continuousValues) const {
  // functor to calculate to double error value from GaussianConditional.
  auto errorFunc =
      [continuousValues](const GaussianConditional::shared_ptr &conditional) {
        if (conditional) {
          return conditional->error(continuousValues);
        } else {
          // Return arbitrarily large error if conditional is null
          // Conditional is null if it is pruned out.
          return 1e50;
        }
      };
  DecisionTree<Key, double> errorTree(conditionals_, errorFunc);
  return errorTree;
}

/* *******************************************************************************/
double GaussianMixture::error(const HybridValues &values) const {
  // Directly index to get the conditional, no need to build the whole tree.
  auto conditional = conditionals_(values.discrete());
  return conditional->error(values.continuous());
}

}  // namespace gtsam
