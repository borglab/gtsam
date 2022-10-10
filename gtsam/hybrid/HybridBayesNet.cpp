/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @date   January 2022
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>

namespace gtsam {

/* ************************************************************************* */
/// Return the DiscreteKey vector as a set.
static std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &dkeys) {
  std::set<DiscreteKey> s;
  s.insert(dkeys.begin(), dkeys.end());
  return s;
}

/* ************************************************************************* */
DecisionTreeFactor::shared_ptr HybridBayesNet::discreteConditionals() const {
  AlgebraicDecisionTree<Key> decisionTree;

  // The canonical decision tree factor which will get the discrete conditionals
  // added to it.
  DecisionTreeFactor dtFactor;

  for (size_t i = 0; i < this->size(); i++) {
    HybridConditional::shared_ptr conditional = this->at(i);
    if (conditional->isDiscrete()) {
      // Convert to a DecisionTreeFactor and add it to the main factor.
      DecisionTreeFactor f(*conditional->asDiscreteConditional());
      dtFactor = dtFactor * f;
    }
  }
  return boost::make_shared<DecisionTreeFactor>(dtFactor);
}

/**
 * @brief Helper function to get the pruner functional.
 *
 * @param probDecisionTree The probability decision tree of only discrete keys.
 * @param discreteFactorKeySet Set of DiscreteKeys in probDecisionTree.
 * Pre-computed for efficiency.
 * @param gaussianMixtureKeySet Set of DiscreteKeys in the GaussianMixture.
 * @return std::function<GaussianConditional::shared_ptr(
 * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
 */
std::function<GaussianConditional::shared_ptr(
    const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
PrunerFunc(const DecisionTreeFactor::shared_ptr &probDecisionTree,
           const std::set<DiscreteKey> &discreteFactorKeySet,
           const std::set<DiscreteKey> &gaussianMixtureKeySet) {
  auto pruner = [&](const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    DiscreteValues values(choices);

    // Case where the gaussian mixture has the same
    // discrete keys as the decision tree.
    if (gaussianMixtureKeySet == discreteFactorKeySet) {
      if ((*probDecisionTree)(values) == 0.0) {
        // empty aka null pointer
        boost::shared_ptr<GaussianConditional> null;
        return null;
      } else {
        return conditional;
      }
    } else {
      std::vector<DiscreteKey> set_diff;
      std::set_difference(
          discreteFactorKeySet.begin(), discreteFactorKeySet.end(),
          gaussianMixtureKeySet.begin(), gaussianMixtureKeySet.end(),
          std::back_inserter(set_diff));
      const std::vector<DiscreteValues> assignments =
          DiscreteValues::CartesianProduct(set_diff);
      for (const DiscreteValues &assignment : assignments) {
        DiscreteValues augmented_values(values);
        augmented_values.insert(assignment.begin(), assignment.end());

        // If any one of the sub-branches are non-zero,
        // we need this conditional.
        if ((*probDecisionTree)(augmented_values) > 0.0) {
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

/* ************************************************************************* */
HybridBayesNet HybridBayesNet::prune(size_t maxNrLeaves) const {
  // Get the decision tree of only the discrete keys
  auto discreteConditionals = this->discreteConditionals();
  const DecisionTreeFactor::shared_ptr discreteFactor =
      boost::make_shared<DecisionTreeFactor>(
          discreteConditionals->prune(maxNrLeaves));

  auto discreteFactorKeySet = DiscreteKeysAsSet(discreteFactor->discreteKeys());

  /* To Prune, we visitWith every leaf in the GaussianMixture.
   * For each leaf, using the assignment we can check the discrete decision tree
   * for 0.0 probability, then just set the leaf to a nullptr.
   *
   * We can later check the GaussianMixture for just nullptrs.
   */

  HybridBayesNet prunedBayesNetFragment;

  // Go through all the conditionals in the
  // Bayes Net and prune them as per discreteFactor.
  for (size_t i = 0; i < this->size(); i++) {
    HybridConditional::shared_ptr conditional = this->at(i);

    GaussianMixture::shared_ptr gaussianMixture =
        boost::dynamic_pointer_cast<GaussianMixture>(conditional->inner());

    if (gaussianMixture) {
      // We may have mixtures with less discrete keys than discreteFactor so
      // we skip those since the label assignment does not exist.
      auto gmKeySet = DiscreteKeysAsSet(gaussianMixture->discreteKeys());

      // Get the pruner function.
      auto pruner = PrunerFunc(discreteFactor, discreteFactorKeySet, gmKeySet);

      // Run the pruning to get a new, pruned tree
      GaussianMixture::Conditionals prunedTree =
          gaussianMixture->conditionals().apply(pruner);

      DiscreteKeys discreteKeys = gaussianMixture->discreteKeys();
      // reverse keys to get a natural ordering
      std::reverse(discreteKeys.begin(), discreteKeys.end());

      // Convert from boost::iterator_range to KeyVector
      // so we can pass it to constructor.
      KeyVector frontals(gaussianMixture->frontals().begin(),
                         gaussianMixture->frontals().end()),
          parents(gaussianMixture->parents().begin(),
                  gaussianMixture->parents().end());

      // Create the new gaussian mixture and add it to the bayes net.
      auto prunedGaussianMixture = boost::make_shared<GaussianMixture>(
          frontals, parents, discreteKeys, prunedTree);

      // Type-erase and add to the pruned Bayes Net fragment.
      prunedBayesNetFragment.push_back(
          boost::make_shared<HybridConditional>(prunedGaussianMixture));

    } else {
      // Add the non-GaussianMixture conditional
      prunedBayesNetFragment.push_back(conditional);
    }
  }

  return prunedBayesNetFragment;
}

/* ************************************************************************* */
GaussianMixture::shared_ptr HybridBayesNet::atMixture(size_t i) const {
  return factors_.at(i)->asMixture();
}

/* ************************************************************************* */
GaussianConditional::shared_ptr HybridBayesNet::atGaussian(size_t i) const {
  return factors_.at(i)->asGaussian();
}

/* ************************************************************************* */
DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) const {
  return factors_.at(i)->asDiscreteConditional();
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (size_t idx = 0; idx < size(); idx++) {
    if (factors_.at(idx)->isHybrid()) {
      // If factor is hybrid, select based on assignment.
      GaussianMixture gm = *this->atMixture(idx);
      gbn.push_back(gm(assignment));

    } else if (factors_.at(idx)->isContinuous()) {
      // If continuous only, add gaussian conditional.
      gbn.push_back((this->atGaussian(idx)));

    } else if (factors_.at(idx)->isDiscrete()) {
      // If factor at `idx` is discrete-only, we simply continue.
      continue;
    }
  }

  return gbn;
}

/* ************************************************************************* */
HybridValues HybridBayesNet::optimize() const {
  // Solve for the MPE
  DiscreteBayesNet discrete_bn;
  for (auto &conditional : factors_) {
    if (conditional->isDiscrete()) {
      discrete_bn.push_back(conditional->asDiscreteConditional());
    }
  }

  DiscreteValues mpe = DiscreteFactorGraph(discrete_bn).optimize();

  // Given the MPE, compute the optimal continuous values.
  GaussianBayesNet gbn = this->choose(mpe);
  return HybridValues(mpe, gbn.optimize());
}

/* ************************************************************************* */
VectorValues HybridBayesNet::optimize(const DiscreteValues &assignment) const {
  GaussianBayesNet gbn = this->choose(assignment);
  return gbn.optimize();
}

}  // namespace gtsam
