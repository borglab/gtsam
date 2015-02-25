/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file 	LinearEqualityFactor.h
 * @brief   Linear equality factors at the nonlinear level
 * @author  Duy-Nguyen Ta
 * @date 	Sep 30, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/nonlinear/ConstrainedFactor.h>

namespace gtsam {

/* ************************************************************************* */
/** A convenient base class for creating a linear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError().
 * Warning: It is the user's responsibility to make sure the Hessian is approximately zero
 *
 * TODO: Should we check Hessian = 0 automatically to make sure it's linear, like SNOPT?
 * TODO: Sometimes the true Hessian is not 0, but an approximate one with a different retract is zero
 * and that could also work [Absil07fcm]
 */
template<class VALUE>
class LinearEqualityFactor1: public NoiseModelFactor1<VALUE>,
    public ConstrainedFactor {

public:
  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:
  typedef NoiseModelFactor1<VALUE> Base;
  typedef LinearEqualityFactor1<VALUE> This;

public:
  /**
   * Default Constructor for I/O
   */
  LinearEqualityFactor1() {
  }

  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  LinearEqualityFactor1(Key key, Key dualKey, size_t constraintDim = 1) :
      Base(noiseModel::Constrained::All(constraintDim), key), ConstrainedFactor(
          dualKey) {
  }

  virtual ~LinearEqualityFactor1() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual Vector
  evaluateError(const X&, boost::optional<Matrix&> H1 = boost::none) const = 0;

};
// \class LinearEqualityFactor1

/* ************************************************************************* */
/** A convenient base class for creating a linear equality constraint with 2
 * variables.  To derive from this class, implement evaluateError().
 * Warning: It is the user's responsibility to make sure the Hessian is approximately zero
 *
 * TODO: Should we check Hessian = 0 automatically to make sure it's linear, like SNOPT?
 * TODO: Sometimes the true Hessian is not 0, but an approximate one with a different retract is zero
 * and that could also work [Absil07fcm]
 */
template<class VALUE1, class VALUE2>
class LinearEqualityFactor2: public NoiseModelFactor2<VALUE1, VALUE2>,
    public ConstrainedFactor {

public:
  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:
  typedef NoiseModelFactor2<VALUE1, VALUE2> Base;
  typedef LinearEqualityFactor2<VALUE1, VALUE2> This;

public:
  /**
   * Default Constructor for I/O
   */
  LinearEqualityFactor2() {
  }

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  LinearEqualityFactor2(Key j1, Key j2, Key dualKey, size_t constraintDim = 1) :
      Base(noiseModel::Constrained::All(constraintDim), j1, j2), ConstrainedFactor(
          dualKey) {
  }

  virtual ~LinearEqualityFactor2() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const = 0;

};
// \class LinearEqualityFactor2

/* ************************************************************************* */
/** A convenient base class for creating a linear equality constraint with 3
 * variables.  To derive from this class, implement evaluateError().
 * Warning: It is the user's responsibility to make sure the Hessian is approximately zero
 *
 * TODO: Should we check Hessian = 0 automatically to make sure it's linear, like SNOPT?
 * TODO: Sometimes the true Hessian is not 0, but an approximate one with a different retract is zero
 * and that could also work [Absil07fcm]
 */
template<class VALUE1, class VALUE2, class VALUE3>
class LinearEqualityFactor3: public NoiseModelFactor3<VALUE1, VALUE2, VALUE3>,
    public ConstrainedFactor {

public:
  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;

protected:
  typedef NoiseModelFactor3<VALUE1, VALUE2, VALUE3> Base;
  typedef LinearEqualityFactor3<VALUE1, VALUE2, VALUE3> This;

public:

  /**
   * Default Constructor for I/O
   */
  LinearEqualityFactor3() {
  }

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  LinearEqualityFactor3(Key j1, Key j2, Key j3, Key dualKey,
      size_t constraintDim = 1) :
      Base(noiseModel::Constrained::All(constraintDim), j1, j2, j3), ConstrainedFactor(
          dualKey) {
  }

  virtual ~LinearEqualityFactor3() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual Vector
  evaluateError(const X1&, const X2&, const X3&, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const = 0;

};
// \class LinearEqualityFactor3

} /* namespace gtsam */
