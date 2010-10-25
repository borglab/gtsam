/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Sep 2, 2010
 */

#include "Ordering.h"

#include <string>
#include <iostream>
#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Ordering::Ordering(const std::list<Symbol> & L):nVars_(0) {
	int i = 0;
	BOOST_FOREACH( const Symbol& s, L )
	  insert(s, i++) ;
}

/* ************************************************************************* */
void Ordering::permuteWithInverse(const Permutation& inversePermutation) {
  BOOST_FOREACH(Ordering::value_type& key_order, *this) {
    key_order.second = inversePermutation[key_order.second];
  }
}

/* ************************************************************************* */
void Ordering::print(const string& str) const {
  cout << str << " ";
  BOOST_FOREACH(const Ordering::value_type& key_order, *this) {
    if(key_order != *begin())
      cout << ", ";
    cout << (string)key_order.first << ":" << key_order.second;
  }
  cout << endl;
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering& rhs, double tol) const {
  return order_ == rhs.order_;
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
