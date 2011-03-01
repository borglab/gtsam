/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TestableAssertions.h
 * @brief Provides additional testing facilities for common data structures
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <iostream>
#include <boost/foreach.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

/**
 * Equals testing for basic types
 */
bool assert_equal(const Index& expected, const Index& actual, double tol = 0.0) {
  if(expected != actual) {
    std::cout << "Not equal:\nexpected: " << expected << "\nactual: " << actual << std::endl;
    return false;
  }
  return true;
}

/**
 * Version of assert_equals to work with vectors
 * TODO: replace this with a more general approach to handle multiple types of containers
 */
template<class V>
bool assert_equal(const std::vector<V>& expected, const std::vector<V>& actual, double tol = 1e-9) {
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  if(match) {
    size_t i = 0;
    BOOST_FOREACH(const V& a, expected) {
      if (!assert_equal(a, expected[i++], tol)) {
        match = false;
        break;
      }
    }
  }
	if(!match) {
	  std::cout << "expected: ";
	  BOOST_FOREACH(const V& a, expected) { std::cout << a << " "; }
	  std::cout << "\nactual: ";
	  BOOST_FOREACH(const V& a, actual) { std::cout << a << " "; }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * General function for comparing containers of testable objects
 */
template<class V>
bool assert_container_equal(const V& expected, const V& actual, double tol = 1e-9) {
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename V::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (!assert_equal(*itExp, *itAct, tol)) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: ";
	  BOOST_FOREACH(const typename V::value_type& a, expected) { a.print("  "); }
	  std::cout << "\nactual: ";
	  BOOST_FOREACH(const typename V::value_type& a, actual) { a.print("  "); }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * General function for comparing containers of objects with operator==
 */
template<class V>
bool assert_container_equality(const V& expected, const V& actual, double tol = 1e-9) {
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename V::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (*itExp != *itAct) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: ";
	  BOOST_FOREACH(const typename V::value_type& a, expected) { std::cout << a << " "; }
	  std::cout << "\nactual: ";
	  BOOST_FOREACH(const typename V::value_type& a, actual) { std::cout << a << " "; }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Allow for testing inequality
 */
template<class V>
bool assert_inequal(const V& expected, const V& actual, double tol = 1e-9) {
	if (!actual.equals(expected, tol))
		return true;
	printf("Erroneously equal:\n");
	expected.print("expected");
	actual.print("actual");
	return false;
}

} // \namespace gtsam
