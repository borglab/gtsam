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

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		std::string key;
		BOOST_FOREACH(conditional_ptr conditional,conditionals_)
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
	Ordering BayesNet<Conditional>::ordering() const {
		Ordering ord;
		BOOST_FOREACH(conditional_ptr conditional,conditionals_)
		   ord.push_back(conditional->key());
		return ord;
	}

	/* ************************************************************************* */
	// predicate to check whether a conditional has the sought key
	template<class Conditional>
	class HasKey {
		const string& key_;
	public:
		HasKey(const std::string& key):key_(key) {}
		bool operator()(const boost::shared_ptr<Conditional>& conditional) {
			return (conditional->key()==key_);
		}
	};

	template<class Conditional>
	boost::shared_ptr<Conditional> BayesNet<Conditional>::operator[](const std::string& key) const {
		const_iterator it = find_if(conditionals_.begin(),conditionals_.end(),HasKey<Conditional>(key));
		if (it == conditionals_.end()) throw(invalid_argument(
						"BayesNet::operator['"+key+"']: not found"));
		return *it;
	}
	/* ************************************************************************* */

} // namespace gtsam
