/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include <gtsam/inference/Ordering.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void Ordering::print(const string& s) const {
  cout << s << " (" << size() << "):";
  BOOST_FOREACH(const Symbol& key, *this)
    cout << " " << (string)key;
  cout << endl;
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering &other, double tol) const {
	return *this == other;
}

/* ************************************************************************* */
Ordering Ordering::subtract(const Ordering& keys) const {
	Ordering newOrdering = *this;
	BOOST_FOREACH(const Symbol& key, keys) {
		newOrdering.remove(key);
	}
	return newOrdering;
}

/* ************************************************************************* */
void Unordered::print(const string& s) const {
  cout << s << " (" << size() << "):";
  BOOST_FOREACH(const Symbol& key, *this)
    cout << " " << (string)key;
  cout << endl;
}

/* ************************************************************************* */
bool Unordered::equals(const Unordered &other, double tol) const {
	return *this == other;
}

/* ************************************************************************* */
