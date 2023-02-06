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

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/version.hpp>
#if BOOST_VERSION >= 107400
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/set.hpp>
#endif
#include <gtsam/base/FastDefaultAllocator.h>
#include <gtsam/base/Testable.h>

#include <functional>
#include <set>

namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */

namespace gtsam {

/**
 * FastSet is a thin wrapper around std::set that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several %.
 * @ingroup base
 */
template<typename VALUE>
class FastSet: public std::set<VALUE, std::less<VALUE>,
    typename internal::FastDefaultAllocator<VALUE>::type> {

  BOOST_CONCEPT_ASSERT ((IsTestable<VALUE> ));

public:

  typedef std::set<VALUE, std::less<VALUE>,
  typename internal::FastDefaultAllocator<VALUE>::type> Base;

  using Base::Base;  // Inherit the set constructors

  FastSet() = default; ///< Default constructor

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
  bool exists(const VALUE& e) const {
    return this->find(e) != this->end();
  }

  /** Print to implement Testable: pretty basic */
  void print(const std::string& str = "") const {
    for (typename Base::const_iterator it = this->begin(); it != this->end(); ++it)
    traits<VALUE>::Print(*it, str);
  }

  /** Check for equality within tolerance to implement Testable */
  bool equals(const FastSet<VALUE>& other, double tol = 1e-9) const {
    typename Base::const_iterator it1 = this->begin(), it2 = other.begin();
    while (it1 != this->end()) {
      if (it2 == other.end() || !traits<VALUE>::Equals(*it2, *it2, tol))
      return false;
      ++it1;
      ++it2;
    }
    return true;
  }

  /** insert another set: handy for MATLAB access */
  void merge(const FastSet& other) {
    Base::insert(other.begin(), other.end());
  }

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

}
