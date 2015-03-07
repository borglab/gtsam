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
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

#include <gtsam/base/FastList.h>
#include <gtsam/base/FastSet.h>

namespace gtsam {

/**
 * FastVector is a thin wrapper around std::vector that uses the boost
 * pool_allocator instead of the default STL allocator.  This is just a
 * convenience to avoid having lengthy types in the code.  Through timing,
 * we've seen that the pool_allocator can lead to speedups of several %
 * @addtogroup base
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
  explicit FastVector(INPUTITERATOR first, INPUTITERATOR last) {
    // This if statement works around a bug in boost pool allocator and/or
    // STL vector where if the size is zero, the pool allocator will allocate
    // huge amounts of memory.
    if(first != last)
      Base::assign(first, last);
  }

  /** Copy constructor from another FastVector */
  FastVector(const FastVector<VALUE>& x) : Base(x) {}

  /** Copy constructor from a FastList */
  FastVector(const FastList<VALUE>& x) {
    if(x.size() > 0)
      Base::assign(x.begin(), x.end());
  }

  /** Copy constructor from a FastSet */
  FastVector(const FastSet<VALUE>& x) {
    if(x.size() > 0)
      Base::assign(x.begin(), x.end());
  }

  /** Copy constructor from the base class */
  FastVector(const Base& x) : Base(x) {}

  /** Copy constructor from a standard STL container */
  FastVector(const std::vector<VALUE>& x) {
    // This if statement works around a bug in boost pool allocator and/or
    // STL vector where if the size is zero, the pool allocator will allocate
    // huge amounts of memory.
    if(x.size() > 0)
      Base::assign(x.begin(), x.end());
  }

	/** Conversion to a standard STL container */
	operator std::vector<VALUE>() const {
		return std::vector<VALUE>(this->begin(), this->end());
	}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }

};

}
