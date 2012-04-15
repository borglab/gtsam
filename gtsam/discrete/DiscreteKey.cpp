/*
 * DiscreteKey.h
 * @brief specialized key for discrete variables
 * @author Frank Dellaert
 * @date Feb 28, 2011
 */

#include <iostream>
#include <boost/format.hpp> // for key names
#include <boost/foreach.hpp> // FOREACH
#include "DiscreteKey.h"

namespace gtsam {

	using namespace std;

	bool OldDiscreteKey::equals(const OldDiscreteKey& other, double tol) const {
		return (*this == other);
	}

	void OldDiscreteKey::print(const string& s) const {
		cout << s << *this;
	}

	ostream& operator <<(ostream &os, const OldDiscreteKey &key) {
		os << key.name_;
		return os;
	}

	DiscreteKeys::DiscreteKeys(const vector<int>& cs) {
		for (size_t i = 0; i < cs.size(); i++) {
			string name = boost::str(boost::format("v%1%") % i);
			push_back(DiscreteKey(i, cs[i]));
		}
	}

	vector<Index> DiscreteKeys::indices() const {
		vector < Index > js;
		BOOST_FOREACH(const DiscreteKey& key, *this)
			js.push_back(key.first);
		return js;
	}

	map<Index,size_t> DiscreteKeys::cardinalities() const {
		map<Index,size_t> cs;
		cs.insert(begin(),end());
//		BOOST_FOREACH(const DiscreteKey& key, *this)
//			cs.insert(key);
		return cs;
	}

	DiscreteKeys operator&(const DiscreteKey& key1, const DiscreteKey& key2) {
		DiscreteKeys keys(key1);
		return keys & key2;
	}

}
