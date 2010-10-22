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

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/ISAM.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>

namespace gtsam {

  using namespace std;

  /** Create an empty Bayes Tree */
  template<class CONDITIONAL>
  ISAM<CONDITIONAL>::ISAM() : BayesTree<CONDITIONAL>() {}

  /** Create a Bayes Tree from a Bayes Net */
  template<class CONDITIONAL>
  ISAM<CONDITIONAL>::ISAM(const BayesNet<CONDITIONAL>& bayesNet) :
    BayesTree<CONDITIONAL>(bayesNet) {}

  /* ************************************************************************* */
  template<class CONDITIONAL>
  template<class FACTORGRAPH>
  void ISAM<CONDITIONAL>::update_internal(const FACTORGRAPH& newFactors, Cliques& orphans) {

    // Remove the contaminated part of the Bayes tree
    BayesNet<CONDITIONAL> bn;
    removeTop(newFactors.keys(), bn, orphans);
    FACTORGRAPH factors(bn);

    // add the factors themselves
    factors.push_back(newFactors);

    // eliminate into a Bayes net
    typename BayesNet<CONDITIONAL>::shared_ptr bayesNet = GenericSequentialSolver<typename CONDITIONAL::Factor>(factors).eliminate();

    // insert conditionals back in, straight into the topless bayesTree
    typename BayesNet<CONDITIONAL>::const_reverse_iterator rit;
    for ( rit=bayesNet->rbegin(); rit != bayesNet->rend(); ++rit )
      this->insert(*rit);

    // add orphans to the bottom of the new tree
    BOOST_FOREACH(sharedClique orphan, orphans) {
      this->insert(orphan);
    }

  }

  template<class CONDITIONAL>
  template<class FACTORGRAPH>
  void ISAM<CONDITIONAL>::update(const FACTORGRAPH& newFactors) {
    Cliques orphans;
    this->update_internal(newFactors, orphans);
  }

}
/// namespace gtsam
