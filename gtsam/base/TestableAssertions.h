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
#include <map>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

/**
 * Equals testing for basic types
 */
inline bool assert_equal(const Index& expected, const Index& actual, double tol = 0.0) {
  if(expected != actual) {
    std::cout << "Not equal:\nexpected: " << expected << "\nactual: " << actual << std::endl;
    return false;
  }
  return true;
}

/**
 * Comparisons for boost.optional objects that checks whether objects exist
 * before comparing their values. First version allows for both to be
 * boost::none, but the second, with expected given rather than optional
 *
 * Concept requirement: V is testable
 */
template<class V>
bool assert_equal(const boost::optional<V>& expected,
									const boost::optional<V>& actual, double tol = 1e-9) {
	if (!expected && actual) {
		std::cout << "expected is boost::none, while actual is not" << std::endl;
		return false;
	}
	if (expected && !actual) {
		std::cout << "actual is boost::none, while expected is not" << std::endl;
		return false;
	}
	if (!expected && !actual)
		return true;
	return assert_equal(*expected, *actual, tol);
}

template<class V>
bool assert_equal(const V& expected, const boost::optional<V>& actual, double tol = 1e-9) {
	if (!actual) {
		std::cout << "actual is boost::none" << std::endl;
		return false;
	}
	return assert_equal(expected, *actual, tol);
}

template<class V>
bool assert_equal(const V& expected, const boost::optional<const V&>& actual, double tol = 1e-9) {
	if (!actual) {
		std::cout << "actual is boost::none" << std::endl;
		return false;
	}
	return assert_equal(expected, *actual, tol);
}

/**
 * Version of assert_equals to work with vectors
 * \deprecated: use container equals instead
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
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const V& a, expected) { std::cout << a << " "; }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const V& a, actual) { std::cout << a << " "; }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Function for comparing maps of testable->testable
 * TODO: replace with more generalized version
 */
template<class V1, class V2>
bool assert_container_equal(const std::map<V1,V2>& expected, const std::map<V1,V2>& actual, double tol = 1e-9) {
	typedef typename std::map<V1,V2> Map;
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename Map::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (!assert_equal(itExp->first, itAct->first, tol) ||
  				!assert_equal(itExp->second, itAct->second, tol)) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, expected) {
	  	a.first.print("key");
	  	a.second.print("    value");
	  }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, actual)  {
	  	a.first.print("key");
	  	a.second.print("    value");
	  }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Function for comparing maps of size_t->testable
 */
template<class V2>
bool assert_container_equal(const std::map<size_t,V2>& expected, const std::map<size_t,V2>& actual, double tol = 1e-9) {
	typedef typename std::map<size_t,V2> Map;
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename Map::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (itExp->first != itAct->first ||
  				!assert_equal(itExp->second, itAct->second, tol)) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, expected) {
	  	std::cout << "Key: " << a.first << std::endl;
	  	a.second.print("    value");
	  }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, actual)  {
	  	std::cout << "Key: " << a.first << std::endl;
	  	a.second.print("    value");
	  }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Function for comparing vector of pairs (testable, testable)
 */
template<class V1, class V2>
bool assert_container_equal(const std::vector<std::pair<V1,V2> >& expected,
		const std::vector<std::pair<V1,V2> >& actual, double tol = 1e-9) {
	typedef typename std::vector<std::pair<V1,V2> > VectorPair;
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename VectorPair::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (!assert_equal(itExp->first, itAct->first, tol) ||
  				!assert_equal(itExp->second, itAct->second, tol)) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename VectorPair::value_type& a, expected) {
	  	a.first.print( "    first ");
	  	a.second.print("    second");
	  }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename VectorPair::value_type& a, actual)  {
	  	a.first.print( "    first ");
	  	a.second.print("    second");
	  }
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
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename V::value_type& a, expected) { a.print("  "); }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename V::value_type& a, actual) { a.print("  "); }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Function for comparing maps of size_t->testable
 * Types are assumed to have operator ==
 */
template<class V2>
bool assert_container_equality(const std::map<size_t,V2>& expected, const std::map<size_t,V2>& actual) {
	typedef typename std::map<size_t,V2> Map;
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  typename Map::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (itExp->first  != itAct->first || itExp->second != itAct->second) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, expected) {
	  	std::cout << "Key:   " << a.first << std::endl;
	  	std::cout << "Value: " << a.second << std::endl;
	  }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename Map::value_type& a, actual)  {
	  	std::cout << "Key:   " << a.first << std::endl;
	  	std::cout << "Value: " << a.second << std::endl;
	  }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}


/**
 * General function for comparing containers of objects with operator==
 */
template<class V>
bool assert_container_equality(const V& expected, const V& actual) {
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
	  std::cout << "expected: " << std::endl;
	  BOOST_FOREACH(const typename V::value_type& a, expected) { std::cout << a << " "; }
	  std::cout << "\nactual: " << std::endl;
	  BOOST_FOREACH(const typename V::value_type& a, actual) { std::cout << a << " "; }
	  std::cout << std::endl;
	  return false;
	}
	return true;
}

/**
 * Compare strings for unit tests
 */
inline bool assert_equal(const std::string& expected, const std::string& actual) {
	if (expected == actual)
		return true;
	printf("Not equal:\n");
	std::cout << "expected: [" << expected << "]\n";
	std::cout << "actual: [" << actual << "]" << std::endl;
	return false;
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
