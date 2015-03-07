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

#include <cstddef>

#include <string>
#include <boost/function/function1.hpp>

namespace gtsam {

	/// Integer variable index type
	typedef size_t Index;

  /** A function to convert indices to strings, for example by translating back
   * to a nonlinear key and then to a Symbol. */
  typedef boost::function<std::string(Index)> IndexFormatter;

  std::string _defaultIndexFormatter(Index j);

  /** The default IndexFormatter outputs the index */
  static const IndexFormatter DefaultIndexFormatter = &_defaultIndexFormatter;

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

}

#ifdef _MSC_VER

// Define some common g++ functions and macros that MSVC does not have

#include <boost/math/special_functions/fpclassify.hpp>
namespace std {
  using boost::math::isfinite;
  using boost::math::isnan;
  using boost::math::isinf;
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

