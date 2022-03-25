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
GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i));
}

/* ************************************************************************* */
void HybridBayesNet::setGaussian(size_t i,
                                 const GaussianMixture::shared_ptr &gaussian) {
  factors_.at(i) = gaussian;
}

/* ************************************************************************* */
DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) const {
  return boost::dynamic_pointer_cast<DiscreteConditional>(factors_.at(i));
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (size_t idx = 0; idx < size(); idx++) {
    GaussianMixture gm = *this->atGaussian(idx);
    gbn.push_back(gm(assignment));
  }
  return gbn;
}

/* ************************************************************************* */
HybridBayesNet::shared_ptr HybridBayesNet::prune(
    const DecisionTreeFactor::shared_ptr &discreteFactor) const {
  // To Prune, we visitWith every leaf in the GaussianMixture. For each
  // leaf, we apply an operation, where using the assignment, we can
  // check the discrete decision tree for an exception and if yes, then just
  // set the leaf to a nullptr. We can later check the GaussianMixture for
  // just nullptrs.

  HybridBayesNet::shared_ptr prunedBayesNetFragment =
      boost::make_shared<HybridBayesNet>(*this);

  // Go through all the conditionals in the
  // Bayes Net and prune them as per discreteFactor.
  for (size_t i = 0; i < this->size(); i++) {
    auto conditional = this->at(i);

    // Container for nodes (or nullptrs) to create a new DecisionTree
    std::vector<GaussianConditional::shared_ptr> nodes;

    // Loop over all assignments and create a set of GaussianConditionals
    std::function<GaussianFactor::shared_ptr(
        const Assignment<Key> &, const GaussianFactor::shared_ptr &)>
        pruner = [&](const Assignment<Key> &choices,
                     const GaussianFactor::shared_ptr &gf) {
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

    GaussianMixture::shared_ptr gaussianMixture =
        boost::dynamic_pointer_cast<GaussianMixture>(conditional);

    if (gaussianMixture) {
      // We may have mixtures with less discrete keys than discreteFactor so we
      // skip those since the label assignment does not exist.
      std::set<DiscreteKey> gmKeySet = gaussianMixture->discreteKeys().asSet();
      std::set<DiscreteKey> dfKeySet = discreteFactor->discreteKeys().asSet();
      if (gmKeySet != dfKeySet) {
        continue;
      }

      // Run the pruning to get a new, pruned tree
      auto prunedFactors = gaussianMixture->factors().apply(pruner);

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
          gaussianMixture->nrFrontals(), gaussianMixture->continuousKeys(),
          discreteKeys, prunedTree);

      prunedBayesNetFragment->setGaussian(i, prunedGaussianMixture);
    }
  }

  return prunedBayesNetFragment;
}

}  // namespace gtsam
