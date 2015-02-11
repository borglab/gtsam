/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastSet.h
 * @brief   A thin wrapper around std::set that uses boost's fast_pool_allocator.
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/base/FastDefaultAllocator.h>

#include <set>
#include <iostream>
#include <string>
#include <cmath>
#include <boost/mpl/has_xxx.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/set.hpp>

BOOST_MPL_HAS_XXX_TRAIT_DEF(print)

namespace gtsam {

// This is used internally to allow this container to be Testable even when it
// contains non-testable elements.
template<typename VALUE, class ENABLE = void>
struct FastSetTestableHelper;

/**
 * FastSet is a thin wrapper around std::set that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several %.
 * @addtogroup base
 */
template<typename VALUE, class ENABLE = void>
class FastSet: public std::set<VALUE, std::less<VALUE>, typename internal::FastDefaultAllocator<VALUE>::type> {

public:

  typedef std::set<VALUE, std::less<VALUE>, typename internal::FastDefaultAllocator<VALUE>::type> Base;

  /** Default constructor */
  FastSet() {
  }

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  explicit FastSet(INPUTITERATOR first, INPUTITERATOR last) :
      Base(first, last) {
  }

  /** Constructor from a iterable container, passes through to base class */
  template<typename INPUTCONTAINER>
  explicit FastSet(const INPUTCONTAINER& container) :
      Base(container.begin(), container.end()) {
  }

  /** Copy constructor from another FastSet */
  FastSet(const FastSet<VALUE>& x) :
      Base(x) {
  }

  /** Copy constructor from the base set class */
  FastSet(const Base& x) :
      Base(x) {
  }

#ifdef GTSAM_ALLOCATOR_BOOSTPOOL
  /** Copy constructor from a standard STL container */
  FastSet(const std::set<VALUE>& x) {
    // This if statement works around a bug in boost pool allocator and/or
    // STL vector where if the size is zero, the pool allocator will allocate
    // huge amounts of memory.
    if(x.size() > 0)
      Base::insert(x.begin(), x.end());
  }
#endif

  /** Conversion to a standard STL container */
  operator std::set<VALUE>() const {
    return std::set<VALUE>(this->begin(), this->end());
  }

  /** Handy 'exists' function */
  bool exists(const VALUE& e) const { return this->find(e) != this->end(); }

  /** Print to implement Testable */
  void print(const std::string& str = "") const { FastSetTestableHelper<VALUE>::print(*this, str); }

  /** Check for equality within tolerance to implement Testable */
  bool equals(const FastSet<VALUE>& other, double tol = 1e-9) const { return FastSetTestableHelper<VALUE>::equals(*this, other, tol); }

  /** insert another set: handy for MATLAB access */
  void merge(const FastSet& other) {
    Base::insert(other.begin(),other.end());
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

// This is the default Testable interface for *non*Testable elements, which
// uses stream operators.
template<typename VALUE, class ENABLE>
struct FastSetTestableHelper {

  typedef FastSet<VALUE> Set;

  static void print(const Set& set, const std::string& str) {
    std::cout << str << "\n";
    for (typename Set::const_iterator it = set.begin(); it != set.end(); ++it)
      std::cout << "  " << *it << "\n";
    std::cout.flush();
  }

  static bool equals(const Set& set1, const Set& set2, double tol) {
    typename Set::const_iterator it1 = set1.begin();
    typename Set::const_iterator it2 = set2.begin();
    while (it1 != set1.end()) {
      if (it2 == set2.end() ||
          fabs((double)(*it1) - (double)(*it2)) > tol)
        return false;
      ++it1;
      ++it2;
    }
    return true;
  }
};

// This is the Testable interface for Testable elements
template<typename VALUE>
struct FastSetTestableHelper<VALUE, typename boost::enable_if<has_print<VALUE> >::type> {

  typedef FastSet<VALUE> Set;

  static void print(const Set& set, const std::string& str) {
    std::cout << str << "\n";
    for (typename Set::const_iterator it = set.begin(); it != set.end(); ++it)
      it->print("  ");
    std::cout.flush();
  }

  static bool equals(const Set& set1, const Set& set2, double tol) {
    typename Set::const_iterator it1 = set1.begin();
    typename Set::const_iterator it2 = set2.begin();
    while (it1 != set1.end()) {
      if (it2 == set2.end() ||
          !it1->equals(*it2, tol))
        return false;
      ++it1;
      ++it2;
    }
    return true;
  }
};

}
