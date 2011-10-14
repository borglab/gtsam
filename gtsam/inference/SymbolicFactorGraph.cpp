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

#include <boost/make_shared.hpp>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>

namespace gtsam {

	using namespace std;

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
  FastSet<Index>
  SymbolicFactorGraph::keys() const {
    FastSet<Index> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if(factor) keys.insert(factor->begin(), factor->end()); }
    return keys;
  }

	/* ************************************************************************* */
	IndexFactor::shared_ptr CombineSymbolic(
			const FactorGraph<IndexFactor>& factors, const FastMap<Index,
					std::vector<Index> >& variableSlots) {
		IndexFactor::shared_ptr combined(Combine<IndexFactor, Index> (factors,
				variableSlots));
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

		pair<IndexConditional::shared_ptr, IndexFactor::shared_ptr> result;
		std::vector<Index> newKeys(keys.begin(),keys.end());
    result.first.reset(new IndexConditional(newKeys, nrFrontals));
		result.second.reset(new IndexFactor(newKeys.begin()+nrFrontals, newKeys.end()));

		return result;
	}

	/* ************************************************************************* */
}
