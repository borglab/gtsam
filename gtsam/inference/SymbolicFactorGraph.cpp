/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * SymbolicFactorGraph.cpp
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/EliminationTree-inl.h>

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class FactorGraph<IndexFactor>;
	template class BayesNet<IndexConditional>;
	template class EliminationTree<IndexFactor>;

  /* ************************************************************************* */
	SymbolicFactorGraph::SymbolicFactorGraph(const BayesNet<IndexConditional>& bayesNet) :
	    FactorGraph<IndexFactor>(bayesNet) {}

	/* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Index key) {
    boost::shared_ptr<IndexFactor> factor(new IndexFactor(key));
    push_back(factor);
  }

  /** Push back binary factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2) {
    boost::shared_ptr<IndexFactor> factor(new IndexFactor(key1,key2));
    push_back(factor);
  }

  /** Push back ternary factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2, Index key3) {
    boost::shared_ptr<IndexFactor> factor(new IndexFactor(key1,key2,key3));
    push_back(factor);
  }

  /** Push back 4-way factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2, Index key3, Index key4) {
    boost::shared_ptr<IndexFactor> factor(new IndexFactor(key1,key2,key3,key4));
    push_back(factor);
  }

  /* ************************************************************************* */
  std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> >
  SymbolicFactorGraph::keys() const {
    std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > keys;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if(factor) keys.insert(factor->begin(), factor->end()); }
    return keys;
  }


//	/* ************************************************************************* */
//	SymbolicBayesNet
//	SymbolicFactorGraph::eliminateFrontals(const Ordering& ordering)
//	{
//		return Inference::Eliminate(ordering);
//	}

	/* ************************************************************************* */
}
