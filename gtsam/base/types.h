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
 * @addtogroup base
 */

#pragma once

#include <gtsam/dllexport.h>
#include <boost/concept/assert.hpp>
#include <boost/range/concepts.hpp>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#include <cstddef>
#include <cstdint>

#include <exception>

#ifdef GTSAM_USE_TBB
#include <tbb/scalable_allocator.h>
#endif

#ifdef GTSAM_USE_EIGEN_MKL_OPENMP
#include <omp.h>
#endif

#ifdef __clang__
#  define CLANG_DIAGNOSTIC_PUSH_IGNORE(diag) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"" diag "\"")
#else
#  define CLANG_DIAGNOSTIC_PUSH_IGNORE(diag)
#endif

#ifdef __clang__
#  define CLANG_DIAGNOSTIC_POP() _Pragma("clang diagnostic pop")
#else
#  define CLANG_DIAGNOSTIC_POP()
#endif

namespace gtsam {

  /// Integer nonlinear key type
  typedef std::uint64_t Key;

  /// Integer nonlinear factor index type
  typedef std::uint64_t FactorIndex;

  /// The index type for Eigen objects
  typedef ptrdiff_t DenseIndex;

  /* ************************************************************************* */
  /**
   * Helper class that uses templates to select between two types based on
   * whether TEST_TYPE is const or not.
   */
  template<typename TEST_TYPE, typename BASIC_TYPE, typename AS_NON_CONST,
      typename AS_CONST>
  struct const_selector {
  };

  /** Specialization for the non-const version */
  template<typename BASIC_TYPE, typename AS_NON_CONST, typename AS_CONST>
  struct const_selector<BASIC_TYPE, BASIC_TYPE, AS_NON_CONST, AS_CONST> {
    typedef AS_NON_CONST type;
  };

  /** Specialization for the const version */
  template<typename BASIC_TYPE, typename AS_NON_CONST, typename AS_CONST>
  struct const_selector<const BASIC_TYPE, BASIC_TYPE, AS_NON_CONST, AS_CONST> {
    typedef AS_CONST type;
  };

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
  /** A helper class that behaves as a container with one element, and works with
   * boost::range */
  template<typename T>
  class ListOfOneContainer {
    T element_;
  public:
    typedef T value_type;
    typedef const T* const_iterator;
    typedef T* iterator;
    ListOfOneContainer(const T& element) : element_(element) {}
    const T* begin() const { return &element_; }
    const T* end() const { return &element_ + 1; }
    T* begin() { return &element_; }
    T* end() { return &element_ + 1; }
    size_t size() const { return 1; }
  };

  BOOST_CONCEPT_ASSERT((boost::RandomAccessRangeConcept<ListOfOneContainer<int> >));

  /** Factory function for ListOfOneContainer to enable ListOfOne(e) syntax. */
  template<typename T>
  ListOfOneContainer<T> ListOfOne(const T& element) {
    return ListOfOneContainer<T>(element);
  }

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

#ifdef _MSC_VER

// Define some common g++ functions and macros we use that MSVC does not have

#if (_MSC_VER < 1800)

#include <boost/math/special_functions/fpclassify.hpp>
namespace std {
  template<typename T> inline int isfinite(T a) {
    return (int)boost::math::isfinite(a); }
  template<typename T> inline int isnan(T a) {
    return (int)boost::math::isnan(a); }
  template<typename T> inline int isinf(T a) {
    return (int)boost::math::isinf(a); }
}

#endif

#include <boost/math/constants/constants.hpp>
#ifndef M_PI
#define M_PI (boost::math::constants::pi<double>())
#endif
#ifndef M_PI_2
#define M_PI_2 (boost::math::constants::pi<double>() / 2.0)
#endif
#ifndef M_PI_4
#define M_PI_4 (boost::math::constants::pi<double>() / 4.0)
#endif

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
