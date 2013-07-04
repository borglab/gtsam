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

#include <cstddef>

#include <string>
#include <boost/function/function1.hpp>

namespace gtsam {

  /// Integer variable index type
  typedef size_t Index;

  /** A function to convert indices to strings, for example by translating back
   * to a nonlinear key and then to a Symbol. */
  typedef boost::function<std::string(Index)> IndexFormatter;

  GTSAM_EXPORT std::string _defaultIndexFormatter(Index j);

  /** The default IndexFormatter outputs the index */
  static const IndexFormatter DefaultIndexFormatter = &_defaultIndexFormatter;


  /// Integer nonlinear key type
  typedef size_t Key;

  /// Typedef for a function to format a key, i.e. to convert it to a string
  typedef boost::function<std::string(Key)> KeyFormatter;

  // Helper function for DefaultKeyFormatter
  GTSAM_EXPORT std::string _defaultKeyFormatter(Key key);

  /// The default KeyFormatter, which is used if no KeyFormatter is passed to
  /// a nonlinear 'print' function.  Automatically detects plain integer keys
  /// and Symbol keys.
  static const KeyFormatter DefaultKeyFormatter = &_defaultKeyFormatter;


  /// The index type for Eigen objects
  typedef ptrdiff_t DenseIndex;


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

    /** Implicit conversion allows use in if statements for bool type, etc. */
    operator T() const { return value; }
  };

  /** An assertion that throws an exception if NDEBUG is not defined and
   * evaluates to an empty statement otherwise. */
#ifdef NDEBUG
#define assert_throw(CONDITION, EXCEPTION) ((void)0)
#else
#define assert_throw(CONDITION, EXCEPTION) \
  if(!(CONDITION)) { \
    throw (EXCEPTION); \
  }
#endif

}

#ifdef _MSC_VER

// Define some common g++ functions and macros we use that MSVC does not have

#include <boost/math/special_functions/fpclassify.hpp>
namespace std {
  template<typename T> inline int isfinite(T a) {
    return (int)boost::math::isfinite(a); }
  template<typename T> inline int isnan(T a) {
    return (int)boost::math::isnan(a); }
  template<typename T> inline int isinf(T a) {
    return (int)boost::math::isinf(a); }
}

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

