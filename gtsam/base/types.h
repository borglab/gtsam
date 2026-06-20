/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     types.h
 * @brief    Typedefs for easier changing of types
 * @author   Richard Roberts
 * @date     Aug 21, 2010
 * @ingroup base
 */

#pragma once

#include <gtsam/config.h>  // for GTSAM_USE_TBB
#include <gtsam/dllexport.h>

#include <cstddef>
#include <cstdint>
#include <string>

#ifdef GTSAM_USE_TBB
#include <tbb/scalable_allocator.h>
#endif

#ifdef GTSAM_USE_EIGEN_MKL_OPENMP
#include <omp.h>
#endif

namespace gtsam {

  /// Function to demangle type name of variable, e.g. demangle(typeid(x).name())
  std::string GTSAM_EXPORT demangle(const char* name);

  /// Integer nonlinear key type
  typedef std::uint64_t Key;

  /// Integer nonlinear factor index type
  typedef std::uint64_t FactorIndex;

  /// The index type for Eigen objects
  typedef ptrdiff_t DenseIndex;

  /* ************************************************************************* */
  /**
   * Helper struct that encapsulates a value with a default, this is just used
   * as a member object so you don't have to specify defaults in the class
   * constructor.
   */
  template<typename T, T defaultValue>
  struct ValueWithDefault {
    T value;

    /** Default constructor, initialize to default value supplied in template argument */
    ValueWithDefault() : value(defaultValue) {}

    /** Initialize to the given value */
    ValueWithDefault(const T& _value) : value(_value) {}

    /** Operator to access the value */
    T& operator*() { return value; }

    /** Operator to access the value */
    const T& operator*() const { return value; }

    /** Implicit conversion allows use in if statements for bool type, etc. */
    operator T() const { return value; }
  };

    /* ************************************************************************* */
#ifdef __clang__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-private-field" // Clang complains that previousOpenMPThreads is unused in the #else case below
#endif

  /// An object whose scope defines a block where TBB and OpenMP parallelism are mixed.  In such a
  /// block, we use default threads for TBB, and p/2 threads for OpenMP.  If GTSAM is not compiled to
  /// use both TBB and OpenMP, this has no effect.
  class TbbOpenMPMixedScope
  {
    int previousOpenMPThreads;

  public:
#if defined GTSAM_USE_TBB && defined GTSAM_USE_EIGEN_MKL_OPENMP
    TbbOpenMPMixedScope() :
      previousOpenMPThreads(omp_get_num_threads())
    {
      omp_set_num_threads(omp_get_num_procs() / 4);
    }

    ~TbbOpenMPMixedScope()
    {
      omp_set_num_threads(previousOpenMPThreads);
    }
#else
    TbbOpenMPMixedScope() : previousOpenMPThreads(-1) {}
    ~TbbOpenMPMixedScope() {}
#endif
  };

#ifdef __clang__
#  pragma clang diagnostic pop
#endif

}

/* ************************************************************************* */
/** An assertion that throws an exception if NDEBUG is not defined and
* evaluates to an empty statement otherwise. */
#ifdef NDEBUG
#define assert_throw(CONDITION, EXCEPTION) ((void)0)
#else
#define assert_throw(CONDITION, EXCEPTION) \
  if (!(CONDITION)) { \
  throw (EXCEPTION); \
  }
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef ERROR
#undef ERROR
#endif
