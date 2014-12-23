/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 NonlinearConstraint.h
 * @brief  
 * @author Duy-Nguyen Ta
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>

namespace gtsam {

/* ************************************************************************* */
/** A convenient base class for creating a nonlinear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE>
class NonlinearInequality1: public NonlinearConstraint1<VALUE> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:

  typedef NonlinearConstraint1<VALUE> Base;
  typedef NonlinearInequality1<VALUE> This;

private:
  static const int X1Dim = traits::dimension<VALUE>::value;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearInequality1() {
  }

  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequality1(Key key, Key dualKey) :
      Base(key, dualKey, 1) {
  }

  virtual ~NonlinearInequality1() {
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
//
//  virtual GaussianFactor::shared_ptr multipliedHessian(const Values& x,
//      const VectorValues& duals) const {
//    return Base::multipliedHessian(x, duals);
//  }

};
// \class NonlinearConstraint1

/* ************************************************************************* */
/** A convenient base class for creating your own NonlinearConstraint with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2>
class NonlinearInequality2: public NonlinearConstraint2<VALUE1, VALUE2> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:

  typedef NonlinearConstraint2<VALUE1, VALUE2> Base;
  typedef NonlinearInequality2<VALUE1, VALUE2> This;

private:
  static const int X1Dim = traits::dimension<VALUE1>::value;
  static const int X2Dim = traits::dimension<VALUE2>::value;

public:

  /**
   * Default Constructor for I/O
   */
  NonlinearInequality2() {
  }

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequality2(Key j1, Key j2, Key dualKey) :
    Base(j1, j2, 1) {
  }

  virtual ~NonlinearInequality2() {
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
// \class NonlinearConstraint2


} /* namespace gtsam */
