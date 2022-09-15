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
 * @author Varun Agrawal
 */

#include <gtsam/base/treeTraversal-inst.h>
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
void HybridGaussianISAM::prune(const size_t maxNrLeaves) {
  auto decisionTree = boost::dynamic_pointer_cast<DecisionTreeFactor>(
      this->roots_.at(0)->conditional()->inner());

  DecisionTreeFactor prunedDiscreteFactor = decisionTree->prune(maxNrLeaves);
  decisionTree->root_ = prunedDiscreteFactor.root_;

  /// Helper struct for pruning the hybrid bayes tree.
  struct HybridPrunerData {
    /// The discrete decision tree after pruning.
    DecisionTreeFactor prunedDiscreteFactor;
    HybridPrunerData(const DecisionTreeFactor& prunedDiscreteFactor,
                     const HybridBayesTree::sharedNode& parentClique)
        : prunedDiscreteFactor(prunedDiscreteFactor) {}

    /**
     * @brief A function used during tree traversal that operates on each node
     * before visiting the node's children.
     *
     * @param node The current node being visited.
     * @param parentData The data from the parent node.
     * @return HybridPrunerData which is passed to the children.
     */
    static HybridPrunerData AssignmentPreOrderVisitor(
        const HybridBayesTree::sharedNode& clique,
        HybridPrunerData& parentData) {
      // Get the conditional
      HybridConditional::shared_ptr conditional = clique->conditional();

      // If conditional is hybrid, we prune it.
      if (conditional->isHybrid()) {
        auto gaussianMixture = conditional->asMixture();

        // Check if the number of discrete keys match,
        // else we get an assignment error.
        // TODO(Varun) Update prune method to handle assignment subset?
        if (gaussianMixture->discreteKeys() ==
            parentData.prunedDiscreteFactor.discreteKeys()) {
          gaussianMixture->prune(parentData.prunedDiscreteFactor);
        }
      }
      return parentData;
    }
  };

  HybridPrunerData rootData(prunedDiscreteFactor, 0);
  {
    treeTraversal::no_op visitorPost;
    // Limits OpenMP threads since we're mixing TBB and OpenMP
    TbbOpenMPMixedScope threadLimiter;
    treeTraversal::DepthFirstForestParallel(
        *this, rootData, HybridPrunerData::AssignmentPreOrderVisitor,
        visitorPost);
  }
}

}  // namespace gtsam
