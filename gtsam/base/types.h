/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    types.h
 * @brief   Typedefs for easier changing of types
 * @author  Richard Roberts
 * @created Aug 21, 2010
 */

#pragma once

#include <unistd.h>

namespace gtsam {

	/**
	 * Integer variable index type
	 */
	typedef size_t Index;

	/** Helper class that uses templates to select between two types based on
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

