/**
 * @file   BayesNet-inl.h
 * @brief  Bayes chain template definitions
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "BayesNet.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::print(const string& s) const {
		cout << s << ":\n";
		BOOST_FOREACH(string key, keys_) {
			const_iterator it = nodes_.find(key);
			it->second->print("Node[" + key + "]");
		}
	}

	/* ************************************************************************* */
	template<class Conditional>
	bool BayesNet<Conditional>::equals(const BayesNet& cbn, double tol) const {
		if(size() != cbn.size()) return false;
		if(keys_ != cbn.keys_) return false;
		string key;
		boost::shared_ptr<Conditional> node;
		FOREACH_PAIR( key, node, nodes_) {
			const_iterator cg = cbn.nodes_.find(key);
			if (cg == nodes_.end()) return false;
			if (!equals_star(node,cg->second,tol)) return false;
		}
		return true;
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::insert
		(const string& key, boost::shared_ptr<Conditional> node) {
		keys_.push_front(key);
		nodes_.insert(make_pair(key,node));
	}

	/* ************************************************************************* */
	template<class Conditional>
	void BayesNet<Conditional>::erase(const string& key) {
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
