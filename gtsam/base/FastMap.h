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

#include <map>
#include <boost/pool/pool_alloc.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/map.hpp>

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
class FastMap : public std::map<KEY, VALUE, std::less<KEY>, boost::fast_pool_allocator<std::pair<const KEY, VALUE> > > {

public:

  typedef std::map<KEY, VALUE, std::less<KEY>, boost::fast_pool_allocator<std::pair<const KEY, VALUE> > > Base;

  /** Default constructor */
  FastMap() {}

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  explicit FastMap(INPUTITERATOR first, INPUTITERATOR last) : Base(first, last) {}

  /** Copy constructor from another FastMap */
  FastMap(const FastMap<KEY,VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  FastMap(const Base& x) : Base(x) {}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

}
