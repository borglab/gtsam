/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastMap.h
 * @brief   A thin wrapper around std::map that uses boost's fast_pool_allocator.
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/global_includes.h>

// Change class depending on whether we are using TBB
#ifdef GTSAM_USE_TBB

// Include TBB header
#  include <tbb/concurrent_unordered_map.h>
#  undef min // TBB seems to include Windows.h which defines these macros that cause problems
#  undef max
#  undef ERROR

#include <functional> // std::hash()

// Use TBB concurrent_unordered_map for ConcurrentMap
template <typename KEY, typename VALUE>
using ConcurrentMapBase = tbb::concurrent_unordered_map<
  KEY,
  VALUE,
  std::hash<KEY>
  >;

#else

// If we're not using TBB, use a FastMap for ConcurrentMap
#include <gtsam/base/FastMap.h>
template <typename KEY, typename VALUE>
using ConcurrentMapBase = gtsam::FastMap<KEY, VALUE>;

#endif

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#endif
#include <boost/static_assert.hpp>

#include <gtsam/base/FastVector.h>

namespace gtsam {

/**
 * FastMap is a thin wrapper around std::map that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several
 * percent.
 * @ingroup base
 */
template<typename KEY, typename VALUE>
class ConcurrentMap : public ConcurrentMapBase<KEY,VALUE> {

public:

  typedef ConcurrentMapBase<KEY,VALUE> Base;

  /** Default constructor */
  ConcurrentMap() {}

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  ConcurrentMap(INPUTITERATOR first, INPUTITERATOR last) : Base(first, last) {}

  /** Copy constructor from another ConcurrentMap */
  ConcurrentMap(const ConcurrentMap<KEY,VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  ConcurrentMap(const Base& x) : Base(x) {}

  /** Handy 'exists' function */
  bool exists(const KEY& e) const { return this->count(e); }

#ifndef GTSAM_USE_TBB
  // If we're not using TBB and this is actually a FastMap, we need to add these functions and hide
  // the original erase functions.
  void unsafe_erase(typename Base::iterator position) { ((Base*)this)->erase(position); }
  typename Base::size_type unsafe_erase(const KEY& k) { return ((Base*)this)->erase(k); }
  void unsafe_erase(typename Base::iterator first, typename Base::iterator last) {
    return ((Base*)this)->erase(first, last); }
private:
  void erase() {}
public:
#endif

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const
  {
    // Copy to an STL container and serialize that
    FastVector<std::pair<KEY, VALUE> > map(this->size());
    std::copy(this->begin(), this->end(), map.begin());
    ar & BOOST_SERIALIZATION_NVP(map);
  }
  template<class Archive>
  void load(Archive& ar, const unsigned int /*version*/)
  {
    this->clear();
    // Load into STL container and then fill our map
    FastVector<std::pair<KEY, VALUE> > map;
    ar & BOOST_SERIALIZATION_NVP(map);
    this->insert(map.begin(), map.end());
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
};

}
