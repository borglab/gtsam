/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ring.h
 * @brief   Real Ring definition
 * @author  Varun Agrawal
 * @date    Dec 8, 2024
 */

#pragma once

#include <algorithm>

/** The Real ring with addition and multiplication */
struct Ring {
  static inline double zero() { return 0.0; }
  static inline double one() { return 1.0; }
  static inline double add(const double& a, const double& b) { return a + b; }
  static inline double max(const double& a, const double& b) {
    return std::max(a, b);
  }
  static inline double mul(const double& a, const double& b) { return a * b; }
  static inline double div(const double& a, const double& b) {
    return (a == 0 || b == 0) ? 0 : (a / b);
  }
  static inline double id(const double& x) { return x; }
  static inline double negate(const double& x) { return -x; }
};
