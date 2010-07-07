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

#include <boost/assign/std/vector.hpp> // for +=
using namespace boost::assign;

#include "Ordering.h"
#include "BayesNet.h"
//#include "FactorGraph-inl.h"
//#include "Conditional.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		Symbol key;
		BOOST_REVERSE_FOREACH(sharedConditional conditional,conditionals_)
			conditional->print("Node[" + (string)(conditional->key()) + "]");
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesNet<Conditional>::equals(const BayesNet& cbn, double tol) const {
		if(size() != cbn.size()) return false;
		return equal(conditionals_.begin(),conditionals_.end(),cbn.conditionals_.begin(),equals_star<Conditional>(tol));
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
	Ordering BayesNet<Conditional>::ordering() const {
		Ordering ord;
		BOOST_FOREACH(sharedConditional conditional,conditionals_)
		   ord.push_back(conditional->key());
		return ord;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::saveGraph(const std::string &s) const {
		ofstream of(s.c_str());
		of<< "digraph G{\n";
		BOOST_FOREACH(sharedConditional conditional,conditionals_) {
			Symbol child = conditional->key();
			BOOST_FOREACH(const Symbol& parent,conditional->parents()) {
				of << (string)parent << "->" << (string)child << endl;
			}
		}
		of<<"}";
		of.close();
	}

	/* ************************************************************************* */

	template<class Conditional>
	typename BayesNet<Conditional>::sharedConditional
	BayesNet<Conditional>::operator[](const Symbol& key) const {
		const_iterator it = find_if(conditionals_.begin(),conditionals_.end(),onKey<Conditional>(key));
		if (it == conditionals_.end()) throw(invalid_argument(
						"BayesNet::operator['"+(std::string)key+"']: not found"));
		return *it;
	}

	/* ************************************************************************* */

} // namespace gtsam
