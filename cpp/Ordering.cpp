/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/foreach.hpp>
#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
Ordering Ordering::subtract(const Ordering& keys) const {
	Ordering newOrdering = *this;
	BOOST_FOREACH(string key, keys) {
		newOrdering.remove(key);
	}
	return newOrdering;
}

/* ************************************************************************* */
void Ordering::print(const string& s) const {
  cout << s;
  BOOST_FOREACH(string key, *this)
    cout << " " << key;
  cout << endl;
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering &other, double tol) const {
	return *this == other;
}
/* ************************************************************************* */



