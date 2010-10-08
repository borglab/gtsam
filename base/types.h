/**
 * @file    types.h
 * @brief   Typedefs for easier changing of types
 * @author  Richard Roberts
 * @created Aug 21, 2010
 */

#pragma once

#include <unistd.h>

namespace gtsam {

typedef size_t varid_t;

/** Helper class that uses templates to select between two types based on
 * whether TEST_TYPE is const or not.
 */
template<typename TEST_TYPE, typename BASIC_TYPE, typename AS_NON_CONST, typename AS_CONST>
struct const_selector {};

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
  ValueWithDefault() : value(defaultValue) {}
  ValueWithDefault(const T& _value) : value(_value) {}
  T& operator*() { return value; }
};

}

