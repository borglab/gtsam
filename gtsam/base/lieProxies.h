/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file lieProxies.h
 * @brief Provides convenient mappings of common member functions for testing
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/global_includes.h>

/**
 * Global functions in a separate testing namespace
 *
 * These should not be used outside of tests, as they are just remappings
 * of the original functions.  We use these to avoid needing to do
 * too much std::bind magic or writing a bunch of separate proxy functions.
 *
 * Don't expect all classes to work for all of these functions.
 */
namespace gtsam {
namespace testing {

  /** binary functions */
  template<class T>
  T between(const T& t1, const T& t2) { return t1.between(t2); }

  template<class T>
  T compose(const T& t1, const T& t2) { return t1.compose(t2); }

  /** unary functions */
  template<class T>
  T inverse(const T& t) { return t.inverse(); }

  /** rotation functions */
  template<class T, class P>
  P rotate(const T& r, const P& pt) { return r.rotate(pt); }

  template<class T, class P>
  P unrotate(const T& r, const P& pt) { return r.unrotate(pt); }

} // \namespace testing
} // \namespace gtsam


