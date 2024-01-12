/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM-inst.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
 * @author  Michael Kaess
 */

#pragma once

#include <gtsam/inference/ISAM.h>
#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

/* ************************************************************************* */
template<class BAYESTREE>
void ISAM<BAYESTREE>::updateInternal(const FactorGraphType& newFactors,
    Cliques* orphans, const Eliminate& function) {
  // Remove the contaminated part of the Bayes tree
  BayesNetType bn;
  const KeySet newFactorKeys = newFactors.keys();
  if (!this->empty()) {
    KeyVector keyVector(newFactorKeys.begin(), newFactorKeys.end());
    this->removeTop(keyVector, &bn, orphans);
  }

  // Add the removed top and the new factors
  FactorGraphType factors;
  factors.push_back(bn);
  factors.push_back(newFactors);

  // Add the orphaned subtrees
  for (const sharedClique& orphan : *orphans)
    factors.template emplace_shared<BayesTreeOrphanWrapper<Clique> >(orphan);

  // Get an ordering where the new keys are eliminated last
  const VariableIndex index(factors);
  const Ordering ordering = Ordering::ColamdConstrainedLast(index,
      KeyVector(newFactorKeys.begin(), newFactorKeys.end()));

  // eliminate all factors (top, added, orphans) into a new Bayes tree
  auto bayesTree = factors.eliminateMultifrontal(ordering, function, std::cref(index));

  // Re-add into Bayes tree data structures
  this->roots_.insert(this->roots_.end(), bayesTree->roots().begin(),
      bayesTree->roots().end());
  this->nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
}

/* ************************************************************************* */
template<class BAYESTREE>
void ISAM<BAYESTREE>::update(const FactorGraphType& newFactors,
    const Eliminate& function) {
  Cliques orphans;
  this->updateInternal(newFactors, &orphans, function);
}

}
/// namespace gtsam
