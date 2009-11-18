/*
 * SymbolicFactor.cpp
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#include <map>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "SymbolicConditional.h"
#include "SymbolicFactor.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

	/* ************************************************************************* */
	SymbolicFactor::SymbolicFactor(const boost::shared_ptr<SymbolicConditional>& c) {
		// initialize keys_ with parents
		keys_ = c->parents();
		// add key on which conditional is defined
		keys_.push_back(c->key());
	}

	/* ************************************************************************* */
	SymbolicFactor::SymbolicFactor(const vector<shared_ptr> & factors) {

		// store keys in a map to make them unique (set is not portable)
		map<string, string> map;
		BOOST_FOREACH(shared_ptr factor, factors)
			BOOST_FOREACH(string key, factor->keys())
				map.insert(make_pair(key,key));

		// create the unique keys
		string key,val;
		FOREACH_PAIR(key, val, map)
			keys_.push_back(key);
	}

	/* ************************************************************************* */
	void SymbolicFactor::print(const string& s) const {
		cout << s << " ";
		BOOST_FOREACH(string key, keys_) cout << " " << key;
		cout << endl;
	}

	/* ************************************************************************* */
	bool SymbolicFactor::equals(const SymbolicFactor& other, double tol) const {
		return keys_ == other.keys_;
	}

	/* ************************************************************************* */
	pair<SymbolicConditional::shared_ptr, SymbolicFactor::shared_ptr>
	SymbolicFactor::eliminate(const string& key) const
	{
		// get keys from input factor
		list<string> separator;
		BOOST_FOREACH(string j,keys_)
			if (j!=key) separator.push_back(j);

		// start empty remaining factor to be returned
	  boost::shared_ptr<SymbolicFactor> lf(new SymbolicFactor(separator));

	  // create SymbolicConditional on separator
	  SymbolicConditional::shared_ptr cg (new SymbolicConditional(key,separator));

	  return make_pair(cg,lf);
	}

	/* ************************************************************************* */

}
