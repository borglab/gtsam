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
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

GaussianMixture::GaussianMixture(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const GaussianMixture::Conditionals &conditionals)
    : BaseFactor(CollectKeys(continuousFrontals, continuousParents),
                 discreteParents),
      BaseConditional(continuousFrontals.size()),
      conditionals_(conditionals) {
  // Calculate logConstant_ as the maximum of the log constants of the
  // conditionals, by visiting the decision tree:
  logConstant_ = -std::numeric_limits<double>::infinity();
  conditionals_.visit(
      [this](const GaussianConditional::shared_ptr &conditional) {
        if (conditional) {
          this->logConstant_ = std::max(
              this->logConstant_, conditional->logNormalizationConstant());
        }
      });
}

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
GaussianMixture::GaussianMixture(
    const KeyVector &continuousFrontals, const KeyVector &continuousParents,
    const DiscreteKeys &discreteParents,
    const std::vector<GaussianConditional::shared_ptr> &conditionals)
    : GaussianMixture(continuousFrontals, continuousParents, discreteParents,
                      Conditionals(discreteParents, conditionals)) {}

/* *******************************************************************************/
// TODO(dellaert): This is copy/paste: GaussianMixture should be derived from
// GaussianMixtureFactor, no?
GaussianFactorGraphTree GaussianMixture::add(
    const GaussianFactorGraphTree &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const auto tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianFactorGraphTree GaussianMixture::asGaussianFactorGraphTree() const {
  auto wrap = [](const GaussianConditional::shared_ptr &gc) {
    return GaussianFactorGraph{gc};
  };
  return {conditionals_, wrap};
}

/* *******************************************************************************/
GaussianBayesNetTree GaussianMixture::add(
    const GaussianBayesNetTree &sum) const {
  using Y = GaussianBayesNet;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    if (graph2.size() == 0) {
      return GaussianBayesNet();
    }
    result.push_back(graph2);
    return result;
  };
  const auto tree = asGaussianBayesNetTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianBayesNetTree GaussianMixture::asGaussianBayesNetTree() const {
  auto wrap = [](const GaussianConditional::shared_ptr &gc) {
    if (gc) {
      return GaussianBayesNet{gc};
    } else {
      return GaussianBayesNet();
    }
  };
  return {conditionals_, wrap};
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
  auto conditional = std::dynamic_pointer_cast<GaussianConditional>(ptr);
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
  std::cout << " logNormalizationConstant: " << logConstant_ << "\n"
            << std::endl;
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
bool GaussianMixture::allFrontalsGiven(const VectorValues &given) const {
  for (auto &&kv : given) {
    if (given.find(kv.first) == given.end()) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
std::shared_ptr<GaussianMixtureFactor> GaussianMixture::likelihood(
    const VectorValues &given) const {
  if (!allFrontalsGiven(given)) {
    throw std::runtime_error(
        "GaussianMixture::likelihood: given values are missing some frontals.");
  }

  const DiscreteKeys discreteParentKeys = discreteKeys();
  const KeyVector continuousParentKeys = continuousParents();
  const GaussianMixtureFactor::Factors likelihoods(
      conditionals_, [&](const GaussianConditional::shared_ptr &conditional) {
        const auto likelihood_m = conditional->likelihood(given);
        const double Cgm_Kgcm =
            logConstant_ - conditional->logNormalizationConstant();
        if (Cgm_Kgcm == 0.0) {
          return likelihood_m;
        } else {
          // Add a constant factor to the likelihood in case the noise models
          // are not all equal.
          GaussianFactorGraph gfg;
          gfg.push_back(likelihood_m);
          Vector c(1);
          c << std::sqrt(2.0 * Cgm_Kgcm);
          auto constantFactor = std::make_shared<JacobianFactor>(c);
          gfg.push_back(constantFactor);
          return std::make_shared<JacobianFactor>(gfg);
        }
      });
  return std::make_shared<GaussianMixtureFactor>(
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
 * @param discreteProbs The probabilities of only discrete keys.
 * @return std::function<GaussianConditional::shared_ptr(
 * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
 */
std::function<GaussianConditional::shared_ptr(
    const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
GaussianMixture::prunerFunc(const DecisionTreeFactor &discreteProbs) {
  // Get the discrete keys as sets for the decision tree
  // and the gaussian mixture.
  auto discreteProbsKeySet = DiscreteKeysAsSet(discreteProbs.discreteKeys());
  auto gaussianMixtureKeySet = DiscreteKeysAsSet(this->discreteKeys());

  auto pruner = [discreteProbs, discreteProbsKeySet, gaussianMixtureKeySet](
                    const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    const DiscreteValues values(choices);

    // Case where the gaussian mixture has the same
    // discrete keys as the decision tree.
    if (gaussianMixtureKeySet == discreteProbsKeySet) {
      if (discreteProbs(values) == 0.0) {
        // empty aka null pointer
        std::shared_ptr<GaussianConditional> null;
        return null;
      } else {
        return conditional;
      }
    } else {
      std::vector<DiscreteKey> set_diff;
      std::set_difference(
          discreteProbsKeySet.begin(), discreteProbsKeySet.end(),
          gaussianMixtureKeySet.begin(), gaussianMixtureKeySet.end(),
          std::back_inserter(set_diff));

      const std::vector<DiscreteValues> assignments =
          DiscreteValues::CartesianProduct(set_diff);
      for (const DiscreteValues &assignment : assignments) {
        DiscreteValues augmented_values(values);
        augmented_values.insert(assignment);

        // If any one of the sub-branches are non-zero,
        // we need this conditional.
        if (discreteProbs(augmented_values) > 0.0) {
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
void GaussianMixture::prune(const DecisionTreeFactor &discreteProbs) {
  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner = prunerFunc(discreteProbs);

  auto pruned_conditionals = conditionals_.apply(pruner);
  conditionals_.root_ = pruned_conditionals.root_;
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixture::logProbability(
    const VectorValues &continuousValues) const {
  // functor to calculate (double) logProbability value from
  // GaussianConditional.
  auto probFunc =
      [continuousValues](const GaussianConditional::shared_ptr &conditional) {
        if (conditional) {
          return conditional->logProbability(continuousValues);
        } else {
          // Return arbitrarily small logProbability if conditional is null
          // Conditional is null if it is pruned out.
          return -1e20;
        }
      };
  return DecisionTree<Key, double>(conditionals_, probFunc);
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> GaussianMixture::errorTree(
    const VectorValues &continuousValues) const {
  auto errorFunc = [&](const GaussianConditional::shared_ptr &conditional) {
    // Check if valid pointer
    if (conditional) {
      return conditional->error(continuousValues) +  //
             logConstant_ - conditional->logNormalizationConstant();
    } else {
      // If not valid, pointer, it means this conditional was pruned,
      // so we return maximum error.
      return std::numeric_limits<double>::max();
    }
  };
  DecisionTree<Key, double> error_tree(conditionals_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double GaussianMixture::error(const HybridValues &values) const {
  // Check if discrete keys in discrete assignment are
  // present in the GaussianMixture
  KeyVector dKeys = this->discreteKeys_.indices();
  bool valid_assignment = false;
  for (auto &&kv : values.discrete()) {
    if (std::find(dKeys.begin(), dKeys.end(), kv.first) != dKeys.end()) {
      valid_assignment = true;
      break;
    }
  }

  // The discrete assignment is not valid so we return 0.0 erorr.
  if (!valid_assignment) {
    return 0.0;
  }

  // Directly index to get the conditional, no need to build the whole tree.
  auto conditional = conditionals_(values.discrete());
  if (conditional) {
    return conditional->error(values.continuous()) +  //
           logConstant_ - conditional->logNormalizationConstant();
  } else {
    // If not valid, pointer, it means this conditional was pruned,
    // so we return maximum error.
    return std::numeric_limits<double>::max();
  }
}

/* *******************************************************************************/
double GaussianMixture::logProbability(const HybridValues &values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->logProbability(values.continuous());
}

/* *******************************************************************************/
double GaussianMixture::evaluate(const HybridValues &values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->evaluate(values.continuous());
}

}  // namespace gtsam
