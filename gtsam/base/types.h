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

#include <gtsam/dllexport.h>
#include <boost/concept/assert.hpp>
#include <boost/range/concepts.hpp>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#include <cstddef>
#include <cstdint>

#include <exception>
#include <string>

#ifdef GTSAM_USE_TBB
#include <tbb/scalable_allocator.h>
#endif

#if defined(__GNUC__) || defined(__clang__)
#define GTSAM_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define GTSAM_DEPRECATED __declspec(deprecated)
#else
#define GTSAM_DEPRECATED
#endif

#ifdef GTSAM_USE_EIGEN_MKL_OPENMP
#include <omp.h>
#endif

/* Define macros for ignoring compiler warnings.
 * Usage Example:
 * ```
 *  CLANG_DIAGNOSTIC_PUSH_IGNORE("-Wdeprecated-declarations")
 *  GCC_DIAGNOSTIC_PUSH_IGNORE("-Wdeprecated-declarations")
 *  MSVC_DIAGNOSTIC_PUSH_IGNORE(4996)
 *  // ... code you want to suppress deprecation warnings for ...
 *  DIAGNOSTIC_POP()
 * ```
 */
#define DO_PRAGMA(x) _Pragma (#x)
#ifdef __clang__
#  define CLANG_DIAGNOSTIC_PUSH_IGNORE(diag) \
  _Pragma("clang diagnostic push") \
  DO_PRAGMA(clang diagnostic ignored diag)
#else
#  define CLANG_DIAGNOSTIC_PUSH_IGNORE(diag)
#endif

#ifdef __GNUC__
#  define GCC_DIAGNOSTIC_PUSH_IGNORE(diag) \
  _Pragma("GCC diagnostic push") \
  DO_PRAGMA(GCC diagnostic ignored diag)
#else
#  define GCC_DIAGNOSTIC_PUSH_IGNORE(diag)
#endif

#ifdef _MSC_VER
#  define MSVC_DIAGNOSTIC_PUSH_IGNORE(code) \
  _Pragma("warning ( push )") \
  DO_PRAGMA(warning ( disable : code ))
#else
#  define MSVC_DIAGNOSTIC_PUSH_IGNORE(code)
#endif

#if defined(__clang__)
#  define DIAGNOSTIC_POP() _Pragma("clang diagnostic pop")
#elif defined(__GNUC__)
#  define DIAGNOSTIC_POP() _Pragma("GCC diagnostic pop")
#elif defined(_MSC_VER)
#  define DIAGNOSTIC_POP() _Pragma("warning ( pop )")
#else
#  define DIAGNOSTIC_POP()
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

#include <cmath>
namespace std {
  template<typename T> inline int isfinite(T a) {
    return (int)std::isfinite(a); }
  template<typename T> inline int isnan(T a) {
    return (int)std::isnan(a); }
  template<typename T> inline int isinf(T a) {
    return (int)std::isinf(a); }
}

#endif

#include <cmath>
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif
#ifndef M_PI_4
#define M_PI_4 (M_PI / 4.0)
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

namespace gtsam {

  /// Convenience void_t as we assume C++11, it will not conflict the std one in C++17 as this is in `gtsam::`
  template<typename ...> using void_t = void;

  /**
   * A SFINAE trait to mark classes that need special alignment.
   *
   * This is required to make std::make_shared and etc respect alignment, which is essential for the Python
   * wrappers to work properly.
   *
   * Explanation
   * =============
   * When a GTSAM type is not declared with the type alias `_eigen_aligned_allocator_trait = void`, the first template
   * will be taken so `needs_eigen_aligned_allocator` will be resolved to `std::false_type`.
   *
   * Otherwise, it will resolve to the second template, which will be resolved to `std::true_type`.
   *
   * Please refer to `gtsam/base/make_shared.h` for an example.
   */
  template<typename, typename = void_t<>>
  struct needs_eigen_aligned_allocator : std::false_type {
  };
  template<typename T>
  struct needs_eigen_aligned_allocator<T, void_t<typename T::_eigen_aligned_allocator_trait>> : std::true_type {
  };

}

/**
 * This marks a GTSAM object to require alignment. With this macro an object will automatically be allocated in aligned
 * memory when one uses `gtsam::make_shared`. It reduces future misalignment problems that is hard to debug.
 * See https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html for detailed explanation.
 */
#define GTSAM_MAKE_ALIGNED_OPERATOR_NEW \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  using _eigen_aligned_allocator_trait = void;

/**
 * This marks a GTSAM object to require alignment. With this macro an object will automatically be allocated in aligned
 * memory when one uses `gtsam::make_shared`. It reduces future misalignment problems that is hard to debug.
 * See https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html for detailed explanation.
 */
#define GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign) \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign) \
  using _eigen_aligned_allocator_trait = void;
