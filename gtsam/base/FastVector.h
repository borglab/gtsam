/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastVector.h
 * @brief   A thin wrapper around std::vector that uses boost's pool_allocator.
 * @author  Richard Roberts
 * @date    Feb 9, 2011
 */

#pragma once

#include <vector>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

/**
 * FastVector is a thin wrapper around std::vector that uses the boost
 * pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the pool_allocator can lead to speedups of several %
 * @ingroup base
 */
template<typename VALUE>
class FastVector: public std::vector<VALUE, boost::pool_allocator<VALUE> > {

public:

  typedef std::vector<VALUE, boost::pool_allocator<VALUE> > Base;

  /** Default constructor */
  FastVector() {}

  /** Constructor from size */
  explicit FastVector(size_t size) : Base(size) {}

  /** Constructor from size and initial values */
  explicit FastVector(size_t size, const VALUE& initial) : Base(size, initial) {}

  /** Constructor from a range, passes through to base class */
  template<typename INPUTITERATOR>
  explicit FastVector(INPUTITERATOR first, INPUTITERATOR last) : Base(first, last) {}

  /** Copy constructor from another FastSet */
  FastVector(const FastVector<VALUE>& x) : Base(x) {}

  /** Copy constructor from the base map class */
  FastVector(const Base& x) : Base(x) {}

};

}
