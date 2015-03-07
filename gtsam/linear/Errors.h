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

#include <gtsam/linear/VectorValues.h>

namespace gtsam {
	
  /** vector of errors */
  class Errors : public std::list<Vector> {

  public:

    Errors() ;

	/** break V into pieces according to its start indices */
	Errors(const VectorValues &V) ;

  	/** print */
    void print(const std::string& s = "Errors") const;

    /** equals, for unit testing */
    bool equals(const Errors& expected, double tol=1e-9) const;

    /** Addition */
    Errors operator+(const Errors& b) const;

    /** subtraction */
    Errors operator-(const Errors& b) const;

    /** negation */
    Errors operator-() const ;

  }; // Errors

  /**
   * dot product
   */
  double dot(const Errors& a, const Errors& b);

  /**
   * BLAS level 2 style
   */
  template <>
  void axpy<Errors,Errors>(double alpha, const Errors& x, Errors& y);

  /** print with optional string */
  void print(const Errors& a, const std::string& s = "Error");

} // gtsam
