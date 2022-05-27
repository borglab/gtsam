/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridISAM.h
 * @date March 31, 2022
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridISAM.h>
#include <gtsam/inference/ISAM-inst.h>
#include <gtsam/inference/Key.h>

#include <iterator>

namespace gtsam {

// Instantiate base class
// template class ISAM<HybridBayesTree>;

/* ************************************************************************* */
HybridISAM::HybridISAM() {}

/* ************************************************************************* */
HybridISAM::HybridISAM(const HybridBayesTree& bayesTree) : Base(bayesTree) {}

void HybridISAM::updateInternal(const HybridFactorGraph& newFactors,
                                HybridBayesTree::Cliques* orphans,
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

  // KeyVector new

  // Get an ordering where the new keys are eliminated last
  const VariableIndex index(factors);
  const Ordering ordering = Ordering::ColamdConstrainedLast(
      index, KeyVector(newKeysDiscreteLast.begin(), newKeysDiscreteLast.end()),
      true);

  ordering.print("ORD");

  // eliminate all factors (top, added, orphans) into a new Bayes tree
  auto bayesTree = factors.eliminateMultifrontal(ordering, function, index);

  // Re-add into Bayes tree data structures
  this->roots_.insert(this->roots_.end(), bayesTree->roots().begin(),
                      bayesTree->roots().end());
  this->nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
}

void HybridISAM::update(const HybridFactorGraph& newFactors,
                        const HybridBayesTree::Eliminate& function) {
  Cliques orphans;
  this->updateInternal(newFactors, &orphans, function);
}

}  // namespace gtsam
