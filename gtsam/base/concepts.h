/*
 * concepts.h
 *
 * @date Dec 4, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

// This is a helper to ease the transition to the new traits defined in this file.
// Uncomment this if you want all methods that are tagged as not implemented
// to cause compilation errors.
#ifdef COMPILE_ERROR_NOT_IMPLEMENTED

#include <boost/static_assert.hpp>
#define CONCEPT_NOT_IMPLEMENTED BOOST_STATIC_ASSERT_MSG(boost::false_type, \
"This method is required by the new concepts framework but has not been implemented yet.");

#else

#include <exception>
#define CONCEPT_NOT_IMPLEMENTED \
  throw std::runtime_error("This method is required by the new concepts framework but has not been implemented yet.");

#endif

namespace gtsam {

template <typename T> struct traits;

}
