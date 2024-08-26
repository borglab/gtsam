/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConstrainedOptProblem.h
 * @brief   Nonlinear constrained optimization problem.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/constraint/NonlinearInequalityConstraint.h>

namespace gtsam {

/** Constrained optimization problem, in the form of
 *    argmin_x 0.5||f(X)||^2
 *    s.t.     h(X) = 0
 *             g(X) <= 0
 * where X represents the variables, 0.5||f(X)||^2 represents the quadratic cost
 * functions, h(X)=0 represents the nonlinear equality constraints, g(x)<=0 represents the
 * inequality constraints.
 */
class GTSAM_EXPORT ConstrainedOptProblem {
 public:
  typedef ConstrainedOptProblem This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  NonlinearFactorGraph costs_;                    // cost function, ||f(X)||^2
  NonlinearEqualityConstraints e_constraints_;    // equality constraints, h(X)=0
  NonlinearInequalityConstraints i_constraints_;  // inequality constraints, g(X)<=0
  Values values_;                                 // initial value estimates of X

 public:
  /** Constructor with both equality and inequality constraints. */
  ConstrainedOptProblem(const NonlinearFactorGraph& costs,
                        const NonlinearEqualityConstraints& e_constraints,
                        const NonlinearInequalityConstraints& i_constraints,
                        const Values& values = Values());

  /** Constructor with equality constraints only. */
  static ConstrainedOptProblem EqConstrainedOptProblem(
      const NonlinearFactorGraph& costs,
      const NonlinearEqualityConstraints& e_constraints,
      const Values& values = Values()) {
    return ConstrainedOptProblem(costs, e_constraints, NonlinearInequalityConstraints(), values);
  }

  /** Member variable access functions. */
  const NonlinearFactorGraph& costs() const { return costs_; }
  const NonlinearEqualityConstraints& eConstraints() const { return e_constraints_; }
  const NonlinearInequalityConstraints& iConstraints() const { return i_constraints_; }
  const Values& initialValues() const { return values_; }

  /** Evaluate cost and constraint violations.
   * Return a tuple representing (cost, e-constraint violation, i-constraint violation).
   */
  std::tuple<double, double, double> evaluate(const Values& values) const;

  /** Return the dimension of the problem, as a tuple of
   * total dimension of cost factors,
   * total dimension of equality constraints,
   * total dimension of inequality constraints,
   * total dimension of variables.
   */
  std::tuple<size_t, size_t, size_t, size_t> dim() const;

  /** Base class to generate keys for auxiliary variables. */
  class GTSAM_EXPORT AuxiliaryKeyGenerator {
   public:
    AuxiliaryKeyGenerator() {}
    virtual ~AuxiliaryKeyGenerator() {}

    virtual Key generate(const size_t k) const { return Symbol('u', k); }

    /** generate the next available auxiliary key that is not in values. */
    Key generate(size_t& k, const Values& values) const {
      Key key = generate(k++);
      while (values.exists(key)) {
        key = generate(k++);
      }
      return key;
    }
  };

  /// Equivalent equality-constrained optimization probelm with auxiliary
  /// variables z. Inequality constraints g(x)<=0 are transformed into equality
  /// constraints g(x)+z^2=0.
  ConstrainedOptProblem auxiliaryProblem(const AuxiliaryKeyGenerator& generator) const;
};

}  // namespace gtsam
