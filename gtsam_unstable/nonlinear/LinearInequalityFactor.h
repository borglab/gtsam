/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 LinearInequalityFactor.h
 * @brief  
 * @author   Duy-Nguyen Ta
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam_unstable/nonlinear/LinearEqualityFactor.h>

namespace gtsam {

/* ************************************************************************* */
/**
 * A convenient base class for creating a linear inequality constraint, e.g., Ax <= 0,
 * at the nonlinear level with 1 variable.
 * To derive from this class, implement computeError().
 */
template<class VALUE>
class LinearInequalityFactor1: public LinearEqualityFactor1<VALUE> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:
  typedef LinearEqualityFactor1<VALUE> Base;
  typedef LinearInequalityFactor1<VALUE> This;

public:

  /**
   * Default Constructor for I/O
   */
  LinearInequalityFactor1() {
  }

  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  LinearInequalityFactor1(Key key, Key dualKey) :
      Base(key, dualKey, 1) {
  }

  virtual ~LinearInequalityFactor1() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual double
  computeError(const X&, boost::optional<Matrix&> H1 = boost::none) const = 0;

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector
  evaluateError(const X& x, boost::optional<Matrix&> H1 = boost::none) const {
    return (Vector(1) << computeError(x, H1)).finished();
  }

};
// \class LinearEqualityFactor1

/* ************************************************************************* */
/**
 * A convenient base class for creating a linear inequality constraint, e.g., Ax <= 0,
 * at the nonlinear level with 1 variable.
 * To derive from this class, implement computeError().
 */
template<class VALUE1, class VALUE2>
class LinearInequalityFactor2: public LinearEqualityFactor2<VALUE1, VALUE2> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:
  typedef LinearEqualityFactor2<VALUE1, VALUE2> Base;
  typedef LinearInequalityFactor2<VALUE1, VALUE2> This;

public:

  /**
   * Default Constructor for I/O
   */
  LinearInequalityFactor2() {
  }

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  LinearInequalityFactor2(Key j1, Key j2, Key dualKey) :
    Base(j1, j2, 1) {
  }

  virtual ~LinearInequalityFactor2() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual double
  computeError(const X1&, const X2&, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const = 0;

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector
  evaluateError(const X1& x1, const X2& x2, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const {
    return (Vector(1) << computeError(x1, x2, H1, H2)).finished();
  }
};
// \class LinearEqualityFactor2


} /* namespace gtsam */
