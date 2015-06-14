/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastDefaultAllocator.h
 * @brief   An easy way to control which allocator is used for Fast* collections
 * @author  Richard Roberts
 * @date    Aug 15, 2013
 */

#pragma once
#include <gtsam/config.h>      // Configuration from CMake

#if !defined GTSAM_ALLOCATOR_BOOSTPOOL && !defined GTSAM_ALLOCATOR_TBB && !defined GTSAM_ALLOCATOR_STL
#  ifdef GTSAM_USE_TBB
//   Use TBB allocator by default if we have TBB, otherwise boost pool
#    define GTSAM_ALLOCATOR_TBB
#  else
#    define GTSAM_ALLOCATOR_BOOSTPOOL
#  endif
#endif

#if defined GTSAM_ALLOCATOR_BOOSTPOOL
#  include <boost/pool/pool_alloc.hpp>
#elif defined GTSAM_ALLOCATOR_TBB
#  include <tbb/tbb_allocator.h>
#  undef min // TBB seems to include Windows.h which defines these macros that cause problems
#  undef max
#  undef ERROR
#elif defined GTSAM_ALLOCATOR_STL
#  include <memory>
#endif

namespace gtsam
{

  namespace internal
  {
    /// Default allocator for list, map, and set types
    template<typename T>
    struct FastDefaultAllocator
    {
#if defined GTSAM_ALLOCATOR_BOOSTPOOL
      typedef boost::fast_pool_allocator<T> type;
      static const bool isBoost = true;
      static const bool isTBB = false;
      static const bool isSTL = false;
#elif defined GTSAM_ALLOCATOR_TBB
      typedef tbb::tbb_allocator<T> type;
      static const bool isBoost = false;
      static const bool isTBB = true;
      static const bool isSTL = false;
#elif defined GTSAM_ALLOCATOR_STL
      typedef std::allocator<T> type;
      static const bool isBoost = false;
      static const bool isTBB = false;
      static const bool isSTL = true;
#endif
    };

    /// Default allocator for vector types (we never use boost pool for vectors)
    template<typename T>
    struct FastDefaultVectorAllocator
    {
#if defined GTSAM_ALLOCATOR_TBB
      typedef tbb::tbb_allocator<T> type;
      static const bool isBoost = false;
      static const bool isTBB = true;
      static const bool isSTL = false;
#else
      typedef std::allocator<T> type;
      static const bool isBoost = false;
      static const bool isTBB = false;
      static const bool isSTL = true;
#endif
    };
  }

}
