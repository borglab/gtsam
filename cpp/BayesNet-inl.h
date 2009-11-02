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
		if(indices_ != cbn.indices_) return false;
		if(size() != cbn.size()) return false;
		return equal(conditionals_.begin(),conditionals_.begin(),conditionals_.begin(),equals_star<Conditional>);
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::push_back
		(const boost::shared_ptr<Conditional>& conditional) {
		indices_.insert(make_pair(conditional->key(),conditionals_.size()));
		conditionals_.push_back(conditional);
	}

	/* ************************************************************************* *
	template<class Conditional>
	void BayesNet<Conditional>::erase(const string& key) {
		list<string>::iterator it;
		for (it=keys_.begin(); it != keys_.end(); ++it){
		  if( strcmp(key.c_str(), (*it).c_str()) == 0 )
				break;
		}
		keys_.erase(it);
		conditionals_.erase(key);
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

} // namespace gtsam
