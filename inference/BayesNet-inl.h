/**
 * @file   BayesNet-inl.h
 * @brief  Bayes net template definitions
 * @author Frank Dellaert
 */

#pragma once

#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <boost/assign/std/vector.hpp> // for +=
using namespace boost::assign;

#include <gtsam/inference/BayesNet.h>
//#include "FactorGraph-inl.h"
#include "Conditional.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		BOOST_REVERSE_FOREACH(sharedConditional conditional,conditionals_)
			conditional->print((boost::format("Node[%1%]") % conditional->key()).str());
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesNet<Conditional>::equals(const BayesNet& cbn, double tol) const {
		if(size() != cbn.size()) return false;
		return equal(conditionals_.begin(),conditionals_.end(),cbn.conditionals_.begin(),equals_star<Conditional>(tol));
	}

  /* ************************************************************************* */
  template<class Conditional>
	void BayesNet<Conditional>::permuteWithInverse(const Permutation& inversePermutation) {
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      conditional->permuteWithInverse(inversePermutation);
    }
  }

  /* ************************************************************************* */
  template<class Conditional>
  bool BayesNet<Conditional>::permuteSeparatorWithInverse(const Permutation& inversePermutation) {
    bool separatorChanged = false;
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      if(conditional->permuteSeparatorWithInverse(inversePermutation))
        separatorChanged = true;
    }
    return separatorChanged;
  }

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::push_back(const BayesNet<Conditional> bn) {
		BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
			push_back(conditional);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::push_front(const BayesNet<Conditional> bn) {
		BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
			push_front(conditional);
	}

	/* ************************************************************************* */
	template<class Conditional>
	list<varid_t> BayesNet<Conditional>::ordering() const {
		list<varid_t> ord;
		BOOST_FOREACH(sharedConditional conditional,conditionals_)
		   ord.push_back(conditional->key());
		return ord;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::saveGraph(const std::string &s) const {
		ofstream of(s.c_str());
		of<< "digraph G{\n";
		BOOST_FOREACH(const_sharedConditional conditional,conditionals_) {
			varid_t child = conditional->key();
			BOOST_FOREACH(varid_t parent, conditional->parents()) {
				of << parent << "->" << child << endl;
			}
		}
		of<<"}";
		of.close();
	}

	/* ************************************************************************* */

	template<class Conditional>
	typename BayesNet<Conditional>::sharedConditional
	BayesNet<Conditional>::operator[](varid_t key) const {
		const_iterator it = find_if(conditionals_.begin(), conditionals_.end(), boost::lambda::bind(&Conditional::key, *boost::lambda::_1) == key);
		if (it == conditionals_.end()) throw(invalid_argument((boost::format(
						"BayesNet::operator['%1%']: not found") % key).str()));
		return *it;
	}

	/* ************************************************************************* */

} // namespace gtsam
