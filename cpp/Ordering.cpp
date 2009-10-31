/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <boost/foreach.hpp>
#include "Ordering.h"

using namespace std;
using namespace gtsam;

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



