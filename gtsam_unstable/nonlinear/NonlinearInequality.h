/**
 * @file 	 NonlinearConstraint.h
 * @brief  
 * @author Duy-Nguyen Ta
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>

namespace gtsam {

class NonlinearInequality : public NonlinearConstraint {
  bool active_;

  typedef NonlinearConstraint Base;

public:
  typedef boost::shared_ptr<NonlinearInequality> shared_ptr;

public:
  /// Construct with dual key
  NonlinearInequality(Key dualKey) : Base(dualKey), active_(true) {}

  /**
   * compute the HessianFactor of the (-dual * constraintHessian) for the qp subproblem's objective function
   */
  virtual GaussianFactor::shared_ptr multipliedHessian(const Values& x,
      const VectorValues& duals) const = 0;
};

/* ************************************************************************* */
/** A convenient base class for creating a nonlinear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE>
class NonlinearInequality1: public NonlinearConstraint1<VALUE>, public NonlinearInequality {

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
  NonlinearInequality1(Key key, Key dualKey, size_t constraintDim = 1) :
      Base(noiseModel::Constrained::All(constraintDim), key), NonlinearConstraint(dualKey) {
  }

  virtual ~NonlinearInequality1() {
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual Vector
  evaluateError(const X&, boost::optional<Matrix&> H1 = boost::none) const {
    return (Vector(1) << computeError(X, H1));
  }

  /**
   *  Override this method to finish implementing a binary factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2).
   */
  virtual double
  computeError(const X&, boost::optional<Matrix&> H1 = boost::none) const = 0;

};
// \class NonlinearConstraint1

} /* namespace gtsam */
