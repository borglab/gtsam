/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FactorGraph-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
 * @author Michael Kaess
 */

#pragma once

#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/base/DSF.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include <stdio.h>
#include <list>
#include <sstream>
#include <stdexcept>
#include <functional>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class FACTOR>
	void FactorGraph<FACTOR>::print(const string& s) const {
		cout << s << endl;
		cout << "size: " << size() << endl;
		for (size_t i = 0; i < factors_.size(); i++) {
			stringstream ss;
			ss << "factor " << i << ": ";
			if (factors_[i] != NULL) factors_[i]->print(ss.str());
		}
	}

	/* ************************************************************************* */
	template<class FACTOR>
	bool FactorGraph<FACTOR>::equals(const FactorGraph<FACTOR>& fg, double tol) const {
		/** check whether the two factor graphs have the same number of factors_ */
		if (factors_.size() != fg.size()) return false;

		/** check whether the factors_ are the same */
		for (size_t i = 0; i < factors_.size(); i++) {
			// TODO: Doesn't this force order of factor insertion?
			sharedFactor f1 = factors_[i], f2 = fg.factors_[i];
			if (f1 == NULL && f2 == NULL) continue;
			if (f1 == NULL || f2 == NULL) return false;
			if (!f1->equals(*f2, tol)) return false;
		}
		return true;
	}

	/* ************************************************************************* */
	template<class FACTOR>
	size_t FactorGraph<FACTOR>::nrFactors() const {
		size_t size_ = 0;
		for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
			if (*factor != NULL) size_++;
		return size_;
	}

	/* ************************************************************************* */
	template<class FACTOR>
	void FactorGraph<FACTOR>::replace(size_t index, sharedFactor factor) {
		if (index >= factors_.size()) throw invalid_argument(boost::str(
				boost::format("Factor graph does not contain a factor with index %d.")
						% index));
		// Replace the factor
		factors_[index] = factor;
	}

	/* ************************************************************************* */
	template<class FACTORGRAPH>
	FACTORGRAPH combine(const FACTORGRAPH& fg1, const FACTORGRAPH& fg2) {
		// create new linear factor graph equal to the first one
		FACTORGRAPH fg = fg1;

		// add the second factors_ in the graph
		fg.push_back(fg2);

		return fg;
	}

	/* ************************************************************************* */
	template<class DERIVED, class KEY>
	typename DERIVED::shared_ptr Combine(const FactorGraph<DERIVED>& factors,
			const FastMap<KEY, std::vector<KEY> >& variableSlots) {
		typedef const FastMap<KEY, std::vector<KEY> > VariableSlots;
		typedef typeof(boost::lambda::bind(&VariableSlots::value_type::first, boost::lambda::_1))
				FirstGetter;
		typedef boost::transform_iterator<FirstGetter,
				typename VariableSlots::const_iterator, KEY, KEY> IndexIterator;
		FirstGetter firstGetter(boost::lambda::bind(
				&VariableSlots::value_type::first, boost::lambda::_1));
		IndexIterator keysBegin(variableSlots.begin(), firstGetter);
		IndexIterator keysEnd(variableSlots.end(), firstGetter);
		return typename DERIVED::shared_ptr(new DERIVED(keysBegin, keysEnd));
	}

  /* ************************************************************************* */
	template<class FACTOR, class CONDITIONAL, class CLIQUE>
	void _FactorGraph_BayesTree_adder(
	    vector<typename FactorGraph<FACTOR>::sharedFactor>& factors,
	    const typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique& clique) {

	  if(clique) {
	    // Add factor from this clique
	    factors.push_back((*clique)->toFactor());

	    // Traverse children
	    typedef typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique sharedClique;
	    BOOST_FOREACH(const sharedClique& child, clique->children()) {
	      _FactorGraph_BayesTree_adder<FACTOR,CONDITIONAL,CLIQUE>(factors, child);
	    }
	  }
	}

  /* ************************************************************************* */
  template<class FACTOR>
  template<class CONDITIONAL, class CLIQUE>
  FactorGraph<FACTOR>::FactorGraph(const BayesTree<CONDITIONAL,CLIQUE>& bayesTree) {
    factors_.reserve(bayesTree.size());
    _FactorGraph_BayesTree_adder<FACTOR,CONDITIONAL,CLIQUE>(factors_, bayesTree.root());
  }

	/* ************************************************************************* */
} // namespace gtsam
