/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <stdexcept>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

/* ************************************************************************* */
Ordering::Ordering(const map<string, string>& p_map) {

	// find the root
	string root;
	bool foundRoot = false;
	string child, parent;
	FOREACH_PAIR(child, parent, p_map) {
		if (child.compare(parent) == 0) {
			root = child;
			foundRoot = true;
			break;
		}
	}
	push_front(root);

	if (!foundRoot)
		throw invalid_argument("Ordering: invalid predecessor map!");

	// push front the nodes from the root level to the leaf level
	list<string> parents(1, root);
	while(!parents.empty()){
		list<string> children;
		BOOST_FOREACH(string key, parents){
			FOREACH_PAIR(child, parent, p_map){
				if (parent.compare(key)==0 && child.compare(key)!=0) {
					children.push_back(child);
					push_front(child);
				}
			}
		}

		parents = children;
	}
}

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



