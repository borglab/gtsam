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
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/inference-inl.h>

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class FactorGraph<Factor>;
	template class BayesNet<Conditional>;

  /* ************************************************************************* */
	SymbolicFactorGraph::SymbolicFactorGraph(const BayesNet<Conditional>& bayesNet) :
	    FactorGraph<Factor>(bayesNet) {}

	/* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(varid_t key) {
    boost::shared_ptr<Factor> factor(new Factor(key));
    push_back(factor);
  }

  /** Push back binary factor */
  void SymbolicFactorGraph::push_factor(varid_t key1, varid_t key2) {
    boost::shared_ptr<Factor> factor(new Factor(key1,key2));
    push_back(factor);
  }

  /** Push back ternary factor */
  void SymbolicFactorGraph::push_factor(varid_t key1, varid_t key2, varid_t key3) {
    boost::shared_ptr<Factor> factor(new Factor(key1,key2,key3));
    push_back(factor);
  }

  /** Push back 4-way factor */
  void SymbolicFactorGraph::push_factor(varid_t key1, varid_t key2, varid_t key3, varid_t key4) {
    boost::shared_ptr<Factor> factor(new Factor(key1,key2,key3,key4));
    push_back(factor);
  }

  /* ************************************************************************* */
  std::set<varid_t, std::less<varid_t>, boost::fast_pool_allocator<varid_t> >
  SymbolicFactorGraph::keys() const {
    std::set<varid_t, std::less<varid_t>, boost::fast_pool_allocator<varid_t> > keys;
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
