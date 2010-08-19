/**
 * @file    Errors.h
 * @brief   vector of errors
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <vector>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>

namespace gtsam {
	
  /** vector of errors */
  class Errors : public std::list<Vector>, public Testable<Errors> {

  public:

  	/** print */
    void print(const std::string& s = "Errors") const;

    /** equals, for unit testing */
    bool equals(const Errors& expected, double tol=1e-9) const;

    /** subtraction */
    Errors operator-(const Errors& b) const;

  }; // Errors

  /**
   * dot product
   */
  double dot(const Errors& a, const Errors& b);

  /**
   * BLAS level 2 style
   */
  void axpy(double alpha, const Errors& x, Errors& y);

  /** print with optional string */
  void print(const Errors& a, const std::string& s = "Error");

} // gtsam
