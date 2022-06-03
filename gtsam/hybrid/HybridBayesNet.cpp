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
 * @date   January 2022
 */

#include <gtsam/hybrid/HybridBayesNet.h>

namespace gtsam {

/* ************************************************************************* */
/// Return the DiscreteKey vector as a set.
static std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &dkeys) {
  std::set<DiscreteKey> s;
  s.insert(dkeys.begin(), dkeys.end());
  return s;
}

/* ************************************************************************* */
HybridBayesNet HybridBayesNet::prune(
    const DecisionTreeFactor::shared_ptr &discreteFactor) const {
  /* To Prune, we visitWith every leaf in the GaussianMixture.
   * For each leaf, using the assignment we can check the discrete decision tree
   * for 0.0 probability, then just set the leaf to a nullptr.
   *
   * We can later check the GaussianMixture for just nullptrs.
   */

  HybridBayesNet prunedBayesNetFragment;

  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner =
      [&](const Assignment<Key> &choices,
          const GaussianFactor::shared_ptr &gf) -> GaussianFactor::shared_ptr {
    // typecast so we can use this to get probability value
    DiscreteValues values(choices);

    if ((*discreteFactor)(values) == 0.0) {
      // empty aka null pointer
      boost::shared_ptr<GaussianFactor> null;
      return null;
    } else {
      return gf;
    }
  };

  // Go through all the conditionals in the
  // Bayes Net and prune them as per discreteFactor.
  for (size_t i = 0; i < this->size(); i++) {
    HybridConditional::shared_ptr conditional = this->at(i);

    GaussianMixture::shared_ptr gaussianMixture =
        boost::dynamic_pointer_cast<GaussianMixture>(conditional->inner());

    if (gaussianMixture) {
      // We may have mixtures with less discrete keys than discreteFactor so we
      // skip those since the label assignment does not exist.
      auto gmKeySet = DiscreteKeysAsSet(gaussianMixture->discreteKeys());
      auto dfKeySet = DiscreteKeysAsSet(discreteFactor->discreteKeys());
      if (gmKeySet != dfKeySet) {
        // Add the gaussianMixture which doesn't have to be pruned.
        prunedBayesNetFragment.push_back(
            boost::make_shared<HybridConditional>(gaussianMixture));
        continue;
      }

      // The GaussianMixture stores all the conditionals and uneliminated
      // factors in the factors tree.
      auto gaussianMixtureTree = gaussianMixture->factors();

      // Run the pruning to get a new, pruned tree
      auto prunedFactors = gaussianMixtureTree.apply(pruner);

      DiscreteKeys discreteKeys = gaussianMixture->discreteKeys();
      // reverse keys to get a natural ordering
      std::reverse(discreteKeys.begin(), discreteKeys.end());

      // Convert to GaussianConditionals
      auto prunedTree = GaussianMixture::Conditionals(
          prunedFactors, [](const GaussianFactor::shared_ptr &factor) {
            return boost::dynamic_pointer_cast<GaussianConditional>(factor);
          });

      // Create the new gaussian mixture and add it to the bayes net.
      auto prunedGaussianMixture = boost::make_shared<GaussianMixture>(
          gaussianMixture->frontals(), gaussianMixture->parents(), discreteKeys,
          prunedTree);

      prunedBayesNetFragment.push_back(
          boost::make_shared<HybridConditional>(prunedGaussianMixture));

    } else {
      // Add the non-GaussianMixture conditional
      prunedBayesNetFragment.push_back(conditional);
    }
  }

  return prunedBayesNetFragment;
}

}  // namespace gtsam
