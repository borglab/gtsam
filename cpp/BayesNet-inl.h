/**
 * @file   BayesNet-inl.h
 * @brief  Bayes net template definitions
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include <boost/assign/std/vector.hpp> // for +=
using namespace boost::assign;

#include "Ordering.h"
#include "BayesNet.h"
#include "FactorGraph-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		std::string key;
		BOOST_FOREACH(sharedConditional conditional,conditionals_)
			conditional->print("Node[" + conditional->key() + "]");
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
	typename BayesNet<Conditional>::sharedConditional
	BayesNet<Conditional>::operator[](const std::string& key) const {
		const_iterator it = find_if(conditionals_.begin(),conditionals_.end(),onKey<Conditional>(key));
		if (it == conditionals_.end()) throw(invalid_argument(
						"BayesNet::operator['"+key+"']: not found"));
		return *it;
	}

	/* ************************************************************************* */

} // namespace gtsam
