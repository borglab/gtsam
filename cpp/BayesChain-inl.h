/**
 * @file   BayesChain-inl.h
 * @brief  Bayes chain template definitions
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "BayesChain.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesChain<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		BOOST_FOREACH(string key, keys_) {
			const_iterator it = nodes_.find(key);
			it->second->print("Node[" + key + "]");
		}
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesChain<Conditional>::equals(const BayesChain& cbn, double tol) const {
		const_iterator it1 = nodes_.begin(), it2 = cbn.nodes_.begin();
		if(nodes_.size() != cbn.nodes_.size()) return false;
		for(; it1 != nodes_.end(); it1++, it2++) {
			const string& j1 = it1->first, j2 = it2->first;
			boost::shared_ptr<Conditional> node1 = it1->second, node2 = it2->second;
			if (j1 != j2) return false;
			if (!node1->equals(*node2,tol))
			return false;
		}
		return true;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesChain<Conditional>::insert
		(const string& key, boost::shared_ptr<Conditional> node) {
		keys_.push_front(key);
		nodes_.insert(make_pair(key,node));
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesChain<Conditional>::erase(const string& key) {
		list<string>::iterator it;
		for (it=keys_.begin(); it != keys_.end(); ++it){
		  if( strcmp(key.c_str(), (*it).c_str()) == 0 )
				break;
		}
		keys_.erase(it);
		nodes_.erase(key);
	}

/* ************************************************************************* */

} // namespace gtsam
