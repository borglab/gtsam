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
#include <gtsam/inference/GenericMultifrontalSolver-inl.h>

namespace gtsam {

  using namespace std;

  /** Create an empty Bayes Tree */
  template<class CONDITIONAL>
  ISAM<CONDITIONAL>::ISAM() : BayesTree<CONDITIONAL>() {}

  /** Create a Bayes Tree from a Bayes Net */
//  template<class CONDITIONAL>
//  ISAM<CONDITIONAL>::ISAM(const BayesNet<CONDITIONAL>& bayesNet) :
//    BayesTree<CONDITIONAL>(bayesNet) {}

  /* ************************************************************************* */
  template<class CONDITIONAL>
	template<class FG> void ISAM<CONDITIONAL>::update_internal(
			const FG& newFactors, Cliques& orphans, typename FG::Eliminate function) {

    // Remove the contaminated part of the Bayes tree
    BayesNet<CONDITIONAL> bn;
    removeTop(newFactors.keys(), bn, orphans);
    FG factors(bn);

    // add the factors themselves
    factors.push_back(newFactors);

    // eliminate into a Bayes net
//    GenericSequentialSolver<typename CONDITIONAL::FactorType> solver(factors);
//    typename BayesNet<CONDITIONAL>::shared_ptr bayesNet = solver.eliminate(function);
//
//    // insert conditionals back in, straight into the topless bayesTree
//    typename BayesNet<CONDITIONAL>::const_reverse_iterator rit;
//    for ( rit=bayesNet->rbegin(); rit != bayesNet->rend(); ++rit )
//      BayesTree<CONDITIONAL>::insert(*this,*rit);
    GenericMultifrontalSolver<typename CONDITIONAL::FactorType, JunctionTree<FG> > solver(factors);
    boost::shared_ptr<BayesTree<CONDITIONAL> > bayesTree;
    bayesTree = solver.eliminate(function);
    this->root_ = bayesTree->root();
    this->nodes_ = bayesTree->nodes();

    // add orphans to the bottom of the new tree
    BOOST_FOREACH(sharedClique orphan, orphans)
      this->insert(orphan);
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
	template<class FG>
	void ISAM<CONDITIONAL>::update(const FG& newFactors,
			typename FG::Eliminate function) {
    Cliques orphans;
    this->update_internal(newFactors, orphans, function);
  }

}
/// namespace gtsam
