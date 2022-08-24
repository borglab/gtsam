/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridGaussianISAM.h
 * @date March 31, 2022
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianISAM.h>
#include <gtsam/inference/ISAM-inst.h>
#include <gtsam/inference/Key.h>

#include <iterator>

namespace gtsam {

// Instantiate base class
// template class ISAM<HybridBayesTree>;

/* ************************************************************************* */
HybridGaussianISAM::HybridGaussianISAM() {}

/* ************************************************************************* */
HybridGaussianISAM::HybridGaussianISAM(const HybridBayesTree& bayesTree)
    : Base(bayesTree) {}

/* ************************************************************************* */
void HybridGaussianISAM::updateInternal(
    const HybridGaussianFactorGraph& newFactors,
    HybridBayesTree::Cliques* orphans,
    const boost::optional<Ordering>& ordering,
    const HybridBayesTree::Eliminate& function) {
  // Remove the contaminated part of the Bayes tree
  BayesNetType bn;
  const KeySet newFactorKeys = newFactors.keys();
  if (!this->empty()) {
    KeyVector keyVector(newFactorKeys.begin(), newFactorKeys.end());
    this->removeTop(keyVector, &bn, orphans);
  }

  // Add the removed top and the new factors
  FactorGraphType factors;
  factors += bn;
  factors += newFactors;

  // Add the orphaned subtrees
  for (const sharedClique& orphan : *orphans)
    factors += boost::make_shared<BayesTreeOrphanWrapper<Node> >(orphan);

  KeySet allDiscrete;
  for (auto& factor : factors) {
    for (auto& k : factor->discreteKeys()) {
      allDiscrete.insert(k.first);
    }
  }
  KeyVector newKeysDiscreteLast;
  for (auto& k : newFactorKeys) {
    if (!allDiscrete.exists(k)) {
      newKeysDiscreteLast.push_back(k);
    }
  }
  std::copy(allDiscrete.begin(), allDiscrete.end(),
            std::back_inserter(newKeysDiscreteLast));

  // Get an ordering where the new keys are eliminated last
  const VariableIndex index(factors);
  Ordering elimination_ordering;
  if (ordering) {
    elimination_ordering = *ordering;
  } else {
    elimination_ordering = Ordering::ColamdConstrainedLast(
        index,
        KeyVector(newKeysDiscreteLast.begin(), newKeysDiscreteLast.end()),
        true);
  }

  // eliminate all factors (top, added, orphans) into a new Bayes tree
  HybridBayesTree::shared_ptr bayesTree =
      factors.eliminateMultifrontal(elimination_ordering, function, index);

  // Re-add into Bayes tree data structures
  this->roots_.insert(this->roots_.end(), bayesTree->roots().begin(),
                      bayesTree->roots().end());
  this->nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
}

/* ************************************************************************* */
void HybridGaussianISAM::update(const HybridGaussianFactorGraph& newFactors,
                                const boost::optional<Ordering>& ordering,
                                const HybridBayesTree::Eliminate& function) {
  Cliques orphans;
  this->updateInternal(newFactors, &orphans, ordering, function);
}

/* ************************************************************************* */
/**
 * @brief Check if `b` is a subset of `a`.
 * Non-const since they need to be sorted.
 *
 * @param a KeyVector
 * @param b KeyVector
 * @return True if the keys of b is a subset of a, else false.
 */
bool IsSubset(KeyVector a, KeyVector b) {
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  return std::includes(a.begin(), a.end(), b.begin(), b.end());
}

/* ************************************************************************* */
void HybridGaussianISAM::prune(const Key& root, const size_t maxNrLeaves) {
  auto decisionTree = boost::dynamic_pointer_cast<DecisionTreeFactor>(
      this->clique(root)->conditional()->inner());
  DecisionTreeFactor prunedDiscreteFactor = decisionTree->prune(maxNrLeaves);
  decisionTree->root_ = prunedDiscreteFactor.root_;

  std::vector<gtsam::Key> prunedKeys;
  for (auto&& clique : nodes()) {
    // The cliques can be repeated for each frontal so we record it in
    // prunedKeys and check if we have already pruned a particular clique.
    if (std::find(prunedKeys.begin(), prunedKeys.end(), clique.first) !=
        prunedKeys.end()) {
      continue;
    }

    // Add all the keys of the current clique to be pruned to prunedKeys
    for (auto&& key : clique.second->conditional()->frontals()) {
      prunedKeys.push_back(key);
    }

    // Convert parents() to a KeyVector for comparison
    KeyVector parents;
    for (auto&& parent : clique.second->conditional()->parents()) {
      parents.push_back(parent);
    }

    if (IsSubset(parents, decisionTree->keys())) {
      auto gaussianMixture = boost::dynamic_pointer_cast<GaussianMixture>(
          clique.second->conditional()->inner());

      gaussianMixture->prune(prunedDiscreteFactor);
    }
  }
}

}  // namespace gtsam
