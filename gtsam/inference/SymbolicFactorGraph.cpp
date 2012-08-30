/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.cpp
 * @date Oct 29, 2009
 * @author Frank Dellaert
 */

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/IndexConditional.h>

#include <boost/make_shared.hpp>

namespace gtsam {

	using namespace std;

  /* ************************************************************************* */
	SymbolicFactorGraph::SymbolicFactorGraph(const SymbolicBayesNet& bayesNet) :
	    FactorGraph<IndexFactor>(bayesNet) {}

  /* ************************************************************************* */
  SymbolicFactorGraph::SymbolicFactorGraph(const SymbolicBayesTree& bayesTree) :
      FactorGraph<IndexFactor>(bayesTree) {}

  /* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Index key) {
    push_back(boost::make_shared<IndexFactor>(key));
  }

  /** Push back binary factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2) {
    push_back(boost::make_shared<IndexFactor>(key1,key2));
  }

  /** Push back ternary factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2, Index key3) {
    push_back(boost::make_shared<IndexFactor>(key1,key2,key3));
  }

  /** Push back 4-way factor */
  void SymbolicFactorGraph::push_factor(Index key1, Index key2, Index key3, Index key4) {
    push_back(boost::make_shared<IndexFactor>(key1,key2,key3,key4));
  }

  /* ************************************************************************* */
  FastSet<Index>
  SymbolicFactorGraph::keys() const {
    FastSet<Index> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if(factor) keys.insert(factor->begin(), factor->end()); }
    return keys;
  }

	/* ************************************************************************* */
	std::pair<SymbolicFactorGraph::sharedConditional, SymbolicFactorGraph>
		SymbolicFactorGraph::eliminateFrontals(size_t nFrontals) const
	{
		return FactorGraph<IndexFactor>::eliminateFrontals(nFrontals, EliminateSymbolic);
	}

	/* ************************************************************************* */
	IndexFactor::shared_ptr CombineSymbolic(
			const FactorGraph<IndexFactor>& factors, const FastMap<Index,
					vector<Index> >& variableSlots) {
		IndexFactor::shared_ptr combined(Combine<IndexFactor, Index> (factors, variableSlots));
//		combined->assertInvariants();
		return combined;
	}

	/* ************************************************************************* */
	pair<IndexConditional::shared_ptr, IndexFactor::shared_ptr> //
	EliminateSymbolic(const FactorGraph<IndexFactor>& factors, size_t nrFrontals) {

		FastSet<Index> keys;
		BOOST_FOREACH(const IndexFactor::shared_ptr& factor, factors)
						BOOST_FOREACH(Index var, *factor)
										keys.insert(var);

		if (keys.size() < 1) throw invalid_argument(
				"IndexFactor::CombineAndEliminate called on factors with no variables.");

		vector<Index> newKeys(keys.begin(), keys.end());
		return make_pair(boost::make_shared<IndexConditional>(newKeys, nrFrontals),
				boost::make_shared<IndexFactor>(newKeys.begin() + nrFrontals, newKeys.end()));
	}

	/* ************************************************************************* */
}
