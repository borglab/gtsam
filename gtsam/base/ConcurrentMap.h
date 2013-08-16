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

#include <tbb/concurrent_unordered_map.h>
#undef min
#undef max
#undef ERROR

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>

#include <gtsam/base/FastVector.h>

namespace gtsam {

/**
 * FastMap is a thin wrapper around std::map that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several
 * percent.
 * @addtogroup base
 */
template<typename KEY, typename VALUE>
class ConcurrentMap : public tbb::concurrent_unordered_map<KEY, VALUE> {

public:

  typedef tbb::concurrent_unordered_map<KEY, VALUE> Base;

  /** Default constructor */
  ConcurrentMap() {}

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  ConcurrentMap(INPUTITERATOR first, INPUTITERATOR last) : Base(first, last) {}

  /** Copy constructor from another FastMap */
  ConcurrentMap(const ConcurrentMap<KEY,VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  ConcurrentMap(const Base& x) : Base(x) {}

  /** Handy 'exists' function */
  bool exists(const KEY& e) const { return this->count(e); }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void save(Archive& ar, const unsigned int version) const
  {
    // Copy to an STL container and serialize that
    FastVector<std::pair<KEY, VALUE> > map(this->size());
    std::copy(this->begin(), this->end(), map.begin());
    ar & BOOST_SERIALIZATION_NVP(map);
  }
  template<class Archive>
  void load(Archive& ar, const unsigned int version)
  {
    // Load into STL container and then fill our map
    FastVector<std::pair<KEY, VALUE> > map;
    ar & BOOST_SERIALIZATION_NVP(map);
    this->insert(map.begin(), map.end());
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
};

}
