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

namespace gtsam {

  /* ************************************************************************* */
  template<class BAYESTREE>
  void ISAM<BAYESTREE>::update_internal(const FactorGraphType& newFactors, Cliques& orphans, const Eliminate& function)
  {
    // Remove the contaminated part of the Bayes tree
    // Throw exception if disconnected
    BayesNetType bn;
    if (!this->empty()) {
      const FastSet<Key> newFactorKeys = newFactors.keys();
      this->removeTop(std::vector<Key>(newFactorKeys.begin(), newFactorKeys.end()), bn, orphans);
      if (bn.empty())
        throw std::runtime_error(
            "ISAM::update_internal(): no variables in common between existing Bayes tree and incoming factors!");
    }

    // Add the removed top and the new factors
    FactorGraphType factors;
    factors += bn;
    factors += newFactors;

    // Add the orphaned subtrees
    BOOST_FOREACH(const sharedClique& orphan, orphans)
      factors += boost::make_shared<BayesTreeOrphanWrapper<Clique> >(orphan);

    // eliminate into a Bayes net
    Base bayesTree = *factors.eliminateMultifrontal(boost::none, function);
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
