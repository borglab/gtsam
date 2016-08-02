/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 NonlinearConstraint.h
 * @brief  Nonlinear inequalities allways assumed to be in the form c(X) <= 0
 * @author Duy-Nguyen Ta
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>

namespace gtsam {
//TODO: Make a generic NonlinearinequalityCnstraint class
/* ************************************************************************* */
/** A convenient base class for creating a nonlinear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE>
class NonlinearInequalityConstraint1: public NonlinearEqualityConstraint1<VALUE> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:

  typedef NonlinearEqualityConstraint1<VALUE> Base;
  typedef NonlinearInequalityConstraint1<VALUE> This;

private:
  static const int X1Dim = traits < VALUE > ::dimension;

public:

  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint1(Key key, Key dualKey) :
      Base(key, dualKey, 1) {
  }

  virtual ~NonlinearInequalityConstraint1() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual double
  computeError(const X&, boost::optional<Matrix&> H1 = boost::none) const = 0;

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H1 =
      boost::none) const {
    return Vector1(computeError(x, H1));
  }
//
//  virtual GaussianFactor::shared_ptr multipliedHessian(const Values& x,
//      const VectorValues& duals) const {
//    return Base::multipliedHessian(x, duals);
//  }
  virtual bool isActive_() const override {
    return Base::active_;
  }
};
// \class NonlinearEqualityConstraint1

/* ************************************************************************* */
/** A convenient base class for creating your own NonlinearConstraint with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2>
class NonlinearInequalityConstraint2: public NonlinearEqualityConstraint2<
    VALUE1, VALUE2> {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:

  typedef NonlinearEqualityConstraint2<VALUE1, VALUE2> Base;
  typedef NonlinearInequalityConstraint2<VALUE1, VALUE2> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;

public:

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint2(Key j1, Key j2, Key dualKey) :
      Base(j1, j2, 1) {
  }

  virtual ~NonlinearInequalityConstraint2() {
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
  virtual Vector evaluateError(const X1& x1, const X2& x2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    return Vector1(computeError(x1, x2, H1, H2));
  }

  virtual bool isActive_() const override {
    return Base::active_;
  }
};
// \class NonlinearEqualityConstraint2

/**
 * A convinient base class for creating NonlinearInequalityConstraint with 3 variables. To
 * derive from this class implement computeError.
 */
template<class VALUE1, class VALUE2, class VALUE3>
class NonlinearInequalityConstraint3: public NonlinearEqualityConstraint3<
    VALUE1, VALUE2, VALUE3> {

public:
  // typdefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;

protected:

  typedef NonlinearEqualityConstraint3<VALUE1, VALUE2, VALUE3> Base;
  typedef NonlinearInequalityConstraint3<VALUE1, VALUE2, VALUE3> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;
  static const int X3Dim = traits < VALUE3 > ::dimension;

public:

  /**
   * Constructor.
   * ConstraintDim is the number of dimensions of the constraint error function == 1
   * @param j1
   * @param j3
   * @param dualKey
   * @return
   */
  NonlinearInequalityConstraint3(Key j1, Key j2, Key j3, Key dualKey) :
      Base(j1, j2, j3, dualKey, 1) {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual double
  computeError(const X1&, const X2&, const X3&, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const = 0;

  /** predefine evaluteError to return a 1-dimension vector */
  virtual Vector evaluateError(const X1 &x1, const X2 &x2, const X3 &x3,
      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
      boost::optional<Matrix &> H3) const override {
    return Vector1(computeError(x1, x2, x3, H1, H2, H3));
  }

  virtual bool isActive_() const override {
    return Base::active_;
  }
};
} /* namespace gtsam */
