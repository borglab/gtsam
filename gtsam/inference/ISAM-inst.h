/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM-inl.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
 * @author  Michael Kaess
 */

#pragma once

#include <gtsam/inference/ISAM.h>
#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class BAYESTREE>
  void ISAM<BAYESTREE>::update_internal(const FactorGraphType& newFactors, Cliques& orphans, const Eliminate& function)
  {
    // Remove the contaminated part of the Bayes tree
    BayesNetType bn;
    if (!this->empty()) {
      const KeySet newFactorKeys = newFactors.keys();
      this->removeTop(std::vector<Key>(newFactorKeys.begin(), newFactorKeys.end()), bn, orphans);
    }

    // Add the removed top and the new factors
    FactorGraphType factors;
    factors += bn;
    factors += newFactors;

    // Add the orphaned subtrees
    for(const sharedClique& orphan: orphans)
      factors += boost::make_shared<BayesTreeOrphanWrapper<Clique> >(orphan);

    // eliminate into a Bayes net
    const VariableIndex varIndex(factors);
    const KeySet newFactorKeys = newFactors.keys();
    const Ordering constrainedOrdering =
      Ordering::ColamdConstrainedLast(varIndex, std::vector<Key>(newFactorKeys.begin(), newFactorKeys.end()));
    Base bayesTree = *factors.eliminateMultifrontal(constrainedOrdering, function, varIndex);
    this->roots_.insert(this->roots_.end(), bayesTree.roots().begin(), bayesTree.roots().end());
    this->nodes_.insert(bayesTree.nodes().begin(), bayesTree.nodes().end());
  }

  /* ************************************************************************* */
  template<class BAYESTREE>
  void ISAM<BAYESTREE>::update(const FactorGraphType& newFactors, const Eliminate& function)
  {
    Cliques orphans;
    this->update_internal(newFactors, orphans, function);
  }

}
/// namespace gtsam
