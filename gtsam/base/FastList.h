/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastList.h
 * @brief   A thin wrapper around std::list that uses boost's fast_pool_allocator.
 * @author  Richard Roberts
 * @date    Oct 22, 2010
 */

#pragma once

#include <list>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

/**
 * FastSet is a thin wrapper around std::set that uses the boost
 * fast_pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the fast_pool_allocator can lead to speedups of several
 * percent.
	 * @ingroup base
 */
template<typename VALUE>
class FastList: public std::list<VALUE, boost::fast_pool_allocator<VALUE> > {

public:

  typedef std::list<VALUE, boost::fast_pool_allocator<VALUE> > Base;

  /** Default constructor */
  FastList() {}

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  explicit FastList(INPUTITERATOR first, INPUTITERATOR last) : Base(first, last) {}

  /** Copy constructor from another FastList */
  FastList(const FastList<VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  FastList(const Base& x) : Base(x) {}

};

}
