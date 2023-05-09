/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A Bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @author Frank Dellaert
 * @date   January 2022
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>

// In Wrappers we have no access to this so have a default ready
static std::mt19937_64 kRandomNumberGenerator(42);

namespace gtsam {

/* ************************************************************************* */
void HybridBayesNet::print(const std::string &s,
                           const KeyFormatter &formatter) const {
  Base::print(s, formatter);
}

/* ************************************************************************* */
bool HybridBayesNet::equals(const This &bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
DecisionTreeFactor::shared_ptr HybridBayesNet::discreteConditionals() const {
  AlgebraicDecisionTree<Key> decisionTree;

  // The canonical decision tree factor which will get
  // the discrete conditionals added to it.
  DecisionTreeFactor dtFactor;

  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      // Convert to a DecisionTreeFactor and add it to the main factor.
      DecisionTreeFactor f(*conditional->asDiscrete());
      dtFactor = dtFactor * f;
    }
  }
  return std::make_shared<DecisionTreeFactor>(dtFactor);
}

/* ************************************************************************* */
/**
 * @brief Helper function to get the pruner functional.
 *
 * @param prunedDecisionTree  The prob. decision tree of only discrete keys.
 * @param conditional Conditional to prune. Used to get full assignment.
 * @return std::function<double(const Assignment<Key> &, double)>
 */
std::function<double(const Assignment<Key> &, double)> prunerFunc(
    const DecisionTreeFactor &prunedDecisionTree,
    const HybridConditional &conditional) {
  // Get the discrete keys as sets for the decision tree
  // and the Gaussian mixture.
  std::set<DiscreteKey> decisionTreeKeySet =
      DiscreteKeysAsSet(prunedDecisionTree.discreteKeys());
  std::set<DiscreteKey> conditionalKeySet =
      DiscreteKeysAsSet(conditional.discreteKeys());

  auto pruner = [prunedDecisionTree, decisionTreeKeySet, conditionalKeySet](
                    const Assignment<Key> &choices,
                    double probability) -> double {
    // This corresponds to 0 probability
    double pruned_prob = 0.0;

    // typecast so we can use this to get probability value
    DiscreteValues values(choices);
    // Case where the Gaussian mixture has the same
    // discrete keys as the decision tree.
    if (conditionalKeySet == decisionTreeKeySet) {
      if (prunedDecisionTree(values) == 0) {
        return pruned_prob;
      } else {
        return probability;
      }
    } else {
      // Due to branch merging (aka pruning) in DecisionTree, it is possible we
      // get a `values` which doesn't have the full set of keys.
      std::set<Key> valuesKeys;
      for (auto kvp : values) {
        valuesKeys.insert(kvp.first);
      }
      std::set<Key> conditionalKeys;
      for (auto kvp : conditionalKeySet) {
        conditionalKeys.insert(kvp.first);
      }
      // If true, then values is missing some keys
      if (conditionalKeys != valuesKeys) {
        // Get the keys present in conditionalKeys but not in valuesKeys
        std::vector<Key> missing_keys;
        std::set_difference(conditionalKeys.begin(), conditionalKeys.end(),
                            valuesKeys.begin(), valuesKeys.end(),
                            std::back_inserter(missing_keys));
        // Insert missing keys with a default assignment.
        for (auto missing_key : missing_keys) {
          values[missing_key] = 0;
        }
      }

      // Now we generate the full assignment by enumerating
      // over all keys in the prunedDecisionTree.
      // First we find the differing keys
      std::vector<DiscreteKey> set_diff;
      std::set_difference(decisionTreeKeySet.begin(), decisionTreeKeySet.end(),
                          conditionalKeySet.begin(), conditionalKeySet.end(),
                          std::back_inserter(set_diff));

      // Now enumerate over all assignments of the differing keys
      const std::vector<DiscreteValues> assignments =
          DiscreteValues::CartesianProduct(set_diff);
      for (const DiscreteValues &assignment : assignments) {
        DiscreteValues augmented_values(values);
        augmented_values.insert(assignment);

        // If any one of the sub-branches are non-zero,
        // we need this probability.
        if (prunedDecisionTree(augmented_values) > 0.0) {
          return probability;
        }
      }
      // If we are here, it means that all the sub-branches are 0,
      // so we prune.
      return pruned_prob;
    }
  };
  return pruner;
}

/* ************************************************************************* */
void HybridBayesNet::updateDiscreteConditionals(
    const DecisionTreeFactor &prunedDecisionTree) {
  KeyVector prunedTreeKeys = prunedDecisionTree.keys();

  // Loop with index since we need it later.
  for (size_t i = 0; i < this->size(); i++) {
    HybridConditional::shared_ptr conditional = this->at(i);
    if (conditional->isDiscrete()) {
      auto discrete = conditional->asDiscrete();

      // Apply prunerFunc to the underlying AlgebraicDecisionTree
      auto discreteTree =
          std::dynamic_pointer_cast<DecisionTreeFactor::ADT>(discrete);
      DecisionTreeFactor::ADT prunedDiscreteTree =
          discreteTree->apply(prunerFunc(prunedDecisionTree, *conditional));

      // Create the new (hybrid) conditional
      KeyVector frontals(discrete->frontals().begin(),
                         discrete->frontals().end());
      auto prunedDiscrete = std::make_shared<DiscreteLookupTable>(
          frontals.size(), conditional->discreteKeys(), prunedDiscreteTree);
      conditional = std::make_shared<HybridConditional>(prunedDiscrete);

      // Add it back to the BayesNet
      this->at(i) = conditional;
    }
  }
}

/* ************************************************************************* */
HybridBayesNet HybridBayesNet::prune(size_t maxNrLeaves) {
  // Get the decision tree of only the discrete keys
  auto discreteConditionals = this->discreteConditionals();
  const auto decisionTree = discreteConditionals->prune(maxNrLeaves);

  this->updateDiscreteConditionals(decisionTree);

  /* To Prune, we visitWith every leaf in the GaussianMixture.
   * For each leaf, using the assignment we can check the discrete decision tree
   * for 0.0 probability, then just set the leaf to a nullptr.
   *
   * We can later check the GaussianMixture for just nullptrs.
   */

  HybridBayesNet prunedBayesNetFragment;

  // Go through all the conditionals in the
  // Bayes Net and prune them as per decisionTree.
  for (auto &&conditional : *this) {
    if (auto gm = conditional->asMixture()) {
      // Make a copy of the Gaussian mixture and prune it!
      auto prunedGaussianMixture = std::make_shared<GaussianMixture>(*gm);
      prunedGaussianMixture->prune(decisionTree);  // imperative :-(

      // Type-erase and add to the pruned Bayes Net fragment.
      prunedBayesNetFragment.push_back(prunedGaussianMixture);

    } else {
      // Add the non-GaussianMixture conditional
      prunedBayesNetFragment.push_back(conditional);
    }
  }

  return prunedBayesNetFragment;
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (auto &&conditional : *this) {
    if (auto gm = conditional->asMixture()) {
      // If conditional is hybrid, select based on assignment.
      gbn.push_back((*gm)(assignment));
    } else if (auto gc = conditional->asGaussian()) {
      // If continuous only, add Gaussian conditional.
      gbn.push_back(gc);
    } else if (auto dc = conditional->asDiscrete()) {
      // If conditional is discrete-only, we simply continue.
      continue;
    }
  }

  return gbn;
}

/* ************************************************************************* */
HybridValues HybridBayesNet::optimize() const {
  // Collect all the discrete factors to compute MPE
  DiscreteBayesNet discrete_bn;
  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      discrete_bn.push_back(conditional->asDiscrete());
    }
  }

  // Solve for the MPE
  DiscreteValues mpe = DiscreteFactorGraph(discrete_bn).optimize();

  // Given the MPE, compute the optimal continuous values.
  return HybridValues(optimize(mpe), mpe);
}

/* ************************************************************************* */
VectorValues HybridBayesNet::optimize(const DiscreteValues &assignment) const {
  GaussianBayesNet gbn = choose(assignment);

  // Check if there exists a nullptr in the GaussianBayesNet
  // If yes, return an empty VectorValues
  if (std::find(gbn.begin(), gbn.end(), nullptr) != gbn.end()) {
    return VectorValues();
  }
  return gbn.optimize();
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(const HybridValues &given,
                                    std::mt19937_64 *rng) const {
  DiscreteBayesNet dbn;
  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      // If conditional is discrete-only, we add to the discrete Bayes net.
      dbn.push_back(conditional->asDiscrete());
    }
  }
  // Sample a discrete assignment.
  const DiscreteValues assignment = dbn.sample(given.discrete());
  // Select the continuous Bayes net corresponding to the assignment.
  GaussianBayesNet gbn = choose(assignment);
  // Sample from the Gaussian Bayes net.
  VectorValues sample = gbn.sample(given.continuous(), rng);
  return {sample, assignment};
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(std::mt19937_64 *rng) const {
  HybridValues given;
  return sample(given, rng);
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(const HybridValues &given) const {
  return sample(given, &kRandomNumberGenerator);
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample() const {
  return sample(&kRandomNumberGenerator);
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::logProbability(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> result(0.0);

  // Iterate over each conditional.
  for (auto &&conditional : *this) {
    if (auto gm = conditional->asMixture()) {
      // If conditional is hybrid, select based on assignment and compute
      // logProbability.
      result = result + gm->logProbability(continuousValues);
    } else if (auto gc = conditional->asGaussian()) {
      // If continuous, get the (double) logProbability and add it to the
      // result
      double logProbability = gc->logProbability(continuousValues);
      // Add the computed logProbability to every leaf of the logProbability
      // tree.
      result = result.apply([logProbability](double leaf_value) {
        return leaf_value + logProbability;
      });
    } else if (auto dc = conditional->asDiscrete()) {
      // If discrete, add the discrete logProbability in the right branch
      result = result.apply(
          [dc](const Assignment<Key> &assignment, double leaf_value) {
            return leaf_value + dc->logProbability(DiscreteValues(assignment));
          });
    }
  }

  return result;
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::evaluate(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> tree = this->logProbability(continuousValues);
  return tree.apply([](double log) { return exp(log); });
}

/* ************************************************************************* */
double HybridBayesNet::evaluate(const HybridValues &values) const {
  return exp(logProbability(values));
}

/* ************************************************************************* */
HybridGaussianFactorGraph HybridBayesNet::toFactorGraph(
    const VectorValues &measurements) const {
  HybridGaussianFactorGraph fg;

  // For all nodes in the Bayes net, if its frontal variable is in measurements,
  // replace it by a likelihood factor:
  for (auto &&conditional : *this) {
    if (conditional->frontalsIn(measurements)) {
      if (auto gc = conditional->asGaussian()) {
        fg.push_back(gc->likelihood(measurements));
      } else if (auto gm = conditional->asMixture()) {
        fg.push_back(gm->likelihood(measurements));
      } else {
        throw std::runtime_error("Unknown conditional type");
      }
    } else {
      fg.push_back(conditional);
    }
  }
  return fg;
}

}  // namespace gtsam
