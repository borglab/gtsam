/**
 * @file   BayesNet-inl.h
 * @brief  Bayes chain template definitions
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
	template<class Factor, class Conditional>
	BayesNet<Conditional> marginals(const BayesNet<Conditional>& bn, const Ordering& keys) {
		// Convert to factor graph
		FactorGraph<Factor> factorGraph(bn);

		// Get the keys of all variables and remove all keys we want the marginal for
		Ordering ord = bn.ordering();
		BOOST_FOREACH(string key, keys) ord.remove(key); // TODO: O(n*k), faster possible?

		// add marginal keys at end
		BOOST_FOREACH(string key, keys) ord.push_back(key);

		// eliminate to get joint
		BayesNet<Conditional> joint = _eliminate<Factor,Conditional>(factorGraph,ord);

		// remove all integrands, P(K) = \int_I P(I|K) P(K)
		size_t nrIntegrands = ord.size()-keys.size();
		for(int i=0;i<nrIntegrands;i++) joint.pop_front();

		// joint is now only on keys, return it
		return joint;
		}

	/* ************************************************************************* */

} // namespace gtsam
