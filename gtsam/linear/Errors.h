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
#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>

#include <string>

namespace gtsam {

  // Forward declarations
  class VectorValues;

  /** vector of errors */
  class Errors : public FastList<Vector> {

  public:

    GTSAM_EXPORT Errors() ;

    /** break V into pieces according to its start indices */
    GTSAM_EXPORT Errors(const VectorValues&V);

    /** print */
    GTSAM_EXPORT void print(const std::string& s = "Errors") const;

    /** equals, for unit testing */
    GTSAM_EXPORT bool equals(const Errors& expected, double tol=1e-9) const;

    /** Addition */
    GTSAM_EXPORT Errors operator+(const Errors& b) const;

    /** subtraction */
    GTSAM_EXPORT Errors operator-(const Errors& b) const;

    /** negation */
    GTSAM_EXPORT Errors operator-() const ;

  }; // Errors

  /**
  * dot product
  */
  GTSAM_EXPORT double dot(const Errors& a, const Errors& b);

  /**
  * BLAS level 2 style
  */
  template <>
  GTSAM_EXPORT void axpy(double alpha, const Errors& x, Errors& y);

  /** print with optional string */
  GTSAM_EXPORT void print(const Errors& a, const std::string& s = "Error");

  /// traits
  template<>
  struct traits<Errors> : public Testable<Errors> {
  };

} //\ namespace gtsam
