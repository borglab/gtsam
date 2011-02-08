/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   BayesNet-inl.h
 * @brief  Bayes net template definitions
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/inference/BayesNet.h>

#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <boost/assign/std/vector.hpp> // for +=
using namespace boost::assign;

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::print(const string& s) const {
		cout << s << ":\n";
		BOOST_REVERSE_FOREACH(sharedConditional conditional,conditionals_)
			conditional->print((boost::format("Node[%1%]") % conditional->key()).str());
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	bool BayesNet<CONDITIONAL>::equals(const BayesNet& cbn, double tol) const {
		if(size() != cbn.size()) return false;
		return equal(conditionals_.begin(),conditionals_.end(),cbn.conditionals_.begin(),equals_star<CONDITIONAL>(tol));
	}

  /* ************************************************************************* */
  template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::permuteWithInverse(const Permutation& inversePermutation) {
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      conditional->permuteWithInverse(inversePermutation);
    }
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  bool BayesNet<CONDITIONAL>::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
    bool separatorChanged = false;
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      if(conditional->permuteSeparatorWithInverse(inversePermutation))
        separatorChanged = true;
    }
    return separatorChanged;
  }

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::push_back(const BayesNet<CONDITIONAL> bn) {
		BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
			push_back(conditional);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::push_front(const BayesNet<CONDITIONAL> bn) {
		BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
			push_front(conditional);
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	FastList<Index> BayesNet<CONDITIONAL>::ordering() const {
		FastList<Index> ord;
		BOOST_FOREACH(sharedConditional conditional,conditionals_)
		   ord.push_back(conditional->key());
		return ord;
	}

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::saveGraph(const std::string &s) const {
		ofstream of(s.c_str());
		of<< "digraph G{\n";
		BOOST_FOREACH(const_sharedConditional conditional,conditionals_) {
			Index child = conditional->key();
			BOOST_FOREACH(Index parent, conditional->parents()) {
				of << parent << "->" << child << endl;
			}
		}
		of<<"}";
		of.close();
	}

	/* ************************************************************************* */

	template<class CONDITIONAL>
	typename BayesNet<CONDITIONAL>::sharedConditional
	BayesNet<CONDITIONAL>::operator[](Index key) const {
		const_iterator it = find_if(conditionals_.begin(), conditionals_.end(), boost::lambda::bind(&CONDITIONAL::key, *boost::lambda::_1) == key);
		if (it == conditionals_.end()) throw(invalid_argument((boost::format(
						"BayesNet::operator['%1%']: not found") % key).str()));
		return *it;
	}

	/* ************************************************************************* */

} // namespace gtsam
