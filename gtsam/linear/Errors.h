/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Errors.h
 * @brief   vector of errors
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/base/FastList.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>

#include <string>

namespace gtsam {

// Forward declarations
class VectorValues;

/// Errors is a vector of errors.
using Errors = FastList<Vector>;

/// Break V into pieces according to its start indices.
GTSAM_EXPORT Errors createErrors(const VectorValues& V);

/// Print an Errors instance.
GTSAM_EXPORT void print(const Errors& e, const std::string& s = "Errors");

// Check equality for unit testing.
GTSAM_EXPORT bool equality(const Errors& actual, const Errors& expected,
                           double tol = 1e-9);

/// Addition.
GTSAM_EXPORT Errors operator+(const Errors& a, const Errors& b);

/// Subtraction.
GTSAM_EXPORT Errors operator-(const Errors& a, const Errors& b);

/// Negation.
GTSAM_EXPORT Errors operator-(const Errors& a);

/// Dot product.
GTSAM_EXPORT double dot(const Errors& a, const Errors& b);

/// BLAS level 2 style AXPY, `y := alpha*x + y`
GTSAM_EXPORT void axpy(double alpha, const Errors& x, Errors& y);

/// traits
template <>
struct traits<Errors> {
  static void Print(const Errors& e, const std::string& str = "") {
    print(e, str);
  }
  static bool Equals(const Errors& actual, const Errors& expected,
                     double tol = 1e-8) {
    return equality(actual, expected, tol);
  }
};

}  // namespace gtsam
