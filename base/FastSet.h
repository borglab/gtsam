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
 * @created Oct 17, 2010
 */

#pragma once

#include <map>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

/**
 * FastSet is a thin wrapper around std::set that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several
 * percent.
 */
template<typename VALUE>
class FastSet: public std::set<VALUE, std::less<VALUE>, boost::fast_pool_allocator<VALUE> > {

public:

  typedef std::set<VALUE, std::less<VALUE>, boost::fast_pool_allocator<VALUE> > Base;

  /** Constructor from a range, passes through to base class */
  template<typename InputIterator>
  FastSet(InputIterator first, InputIterator last) : Base(first, last) {}

  /** Copy constructor from another FastMap */
  FastSet(const FastSet<VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  FastSet(const Base& x) : Base(x) {}

};

}
