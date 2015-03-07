/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.cpp
 * @author  Richard Roberts
 * @date    Sep 2, 2010
 */

#include "Ordering.h"

#include <string>
#include <iostream>
#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Ordering::Ordering(const std::list<Key> & L):nVars_(0) {
	int i = 0;
	BOOST_FOREACH( Key s, L )
	  insert(s, i++) ;
}

/* ************************************************************************* */
void Ordering::permuteWithInverse(const Permutation& inversePermutation) {
  BOOST_FOREACH(Ordering::value_type& key_order, *this) {
    key_order.second = inversePermutation[key_order.second];
  }
}

/* ************************************************************************* */
void Ordering::print(const string& str, const KeyFormatter& keyFormatter) const {
  cout << str;
	// Print ordering in index order
	Ordering::InvertedMap inverted = this->invert();
	// Print the ordering with varsPerLine ordering entries printed on each line,
	// for compactness.
	static const size_t varsPerLine = 10;
	bool endedOnNewline = false;
	BOOST_FOREACH(const Ordering::InvertedMap::value_type& index_key, inverted) {
		if(index_key.first % varsPerLine != 0)
			cout << ", ";
		cout << index_key.first << ":" << keyFormatter(index_key.second);
		if(index_key.first % varsPerLine == varsPerLine - 1) {
			cout << "\n";
			endedOnNewline = true;
		} else {
			endedOnNewline = false;
		}
	}
	if(!endedOnNewline)
		cout << "\n";
	cout.flush();
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering& rhs, double tol) const {
  return order_ == rhs.order_;
}

/* ************************************************************************* */
Ordering::value_type Ordering::pop_back() {
	// FIXME: is there a way of doing this without searching over the entire structure?
	for (iterator it=begin(); it!=end(); ++it) {
		if (it->second == nVars_ - 1) {
			value_type result = *it;
			order_.erase(it);
			--nVars_;
			return result;
		}
	}
	return value_type();
}

/* ************************************************************************* */
Index Ordering::pop_back(Key key) {
  Map::iterator item = order_.find(key);
  if(item == order_.end()) {
    throw invalid_argument("Attempting to remove a key from an ordering that does not contain that key");
  } else {
    if(item->second != nVars_ - 1) {
      throw invalid_argument("Attempting to remove a key from an ordering in which that key is not last");
    } else {
      order_.erase(item);
      -- nVars_;
      return nVars_;
    }
  }
}

/* ************************************************************************* */
Ordering::InvertedMap Ordering::invert() const {
	InvertedMap result;
	BOOST_FOREACH(const value_type& p, *this)
		result.insert(make_pair(p.second, p.first));
	return result;
}

/* ************************************************************************* */
void Unordered::print(const string& s) const {
  cout << s << " (" << size() << "):";
  BOOST_FOREACH(Index key, *this)
    cout << " " << key;
  cout << endl;
}

/* ************************************************************************* */
bool Unordered::equals(const Unordered &other, double tol) const {
  return *this == other;
}

}
