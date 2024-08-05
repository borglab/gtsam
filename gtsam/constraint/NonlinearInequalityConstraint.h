/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearInequalityConstraint.h
 * @brief   Nonlinear inequality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/constraint/RampFunction.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

/**
 * Inequality constraint base class.
 */
class NonlinearInequalityConstraint : public NonlinearConstraint {
 public:
  typedef NonlinearConstraint Base;
  typedef NonlinearInequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  using Base::Base;

  /** Destructor. */
  virtual ~NonlinearInequalityConstraint() {}

  virtual Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const = 0;

  virtual Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  virtual bool active(const Values& x) const override;

  virtual NonlinearEqualityConstraint::shared_ptr createEqualityConstraint() const = 0;

  /** Smooth approximation of the ramp function. */
  virtual NoiseModelFactor::shared_ptr penaltyFactorSmooth(SmoothRampFunction::shared_ptr func,
                                                           const double mu = 1.0) const = 0;

  /** penalty function as if the constraint is equality, 0.5 * mu * ||g(x)||^2 */
  virtual NoiseModelFactor::shared_ptr penaltyFactorEquality(const double mu = 1.0) const;
};

/** Inequality constraint that force g(x) <= 0, where g(x) is a scalar-valued
 * function. */
class ScalarExpressionInequalityConstraint : public NonlinearInequalityConstraint {
 public:
  typedef NonlinearInequalityConstraint Base;
  typedef ScalarExpressionInequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  Double_ expression_;
  FastVector<int> dims_;

 public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param sigma   scalar representing sigma.
   */
  ScalarExpressionInequalityConstraint(const Double_& expression, const double& sigma);

  // Create an inequality constraint g(x)>=0.
  static ScalarExpressionInequalityConstraint::shared_ptr GeqZero(const Double_& expression,
                                                                  const double& sigma);

  // Create an inequality constraint g(x)<=0.
  static ScalarExpressionInequalityConstraint::shared_ptr LeqZero(const Double_& expression,
                                                                  const double& sigma);

  virtual Vector unwhitenedExpr(const Values& x, OptionalMatrixVecType H = nullptr) const override;

  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint() const override;

  NoiseModelFactor::shared_ptr penaltyFactor(const double mu = 1.0) const override;

  NoiseModelFactor::shared_ptr penaltyFactorSmooth(SmoothRampFunction::shared_ptr func,
                                                   const double mu = 1.0) const override;

  virtual NoiseModelFactor::shared_ptr penaltyFactorEquality(const double mu = 1.0) const override;

  const Double_& expression() const { return expression_; }
};

/// Container of NonlinearInequalityConstraint.
class NonlinearInequalityConstraints : public FactorGraph<NonlinearInequalityConstraint> {
 public:
  typedef FactorGraph<NonlinearInequalityConstraint> Base;
  typedef NonlinearInequalityConstraints This;
  typedef std::shared_ptr<This> shared_ptr;

  using Base::Base;

  /// Return the total dimension of constraints.
  size_t dim() const;

  /// Evaluate the constraint violation as a vector
  Vector violationVector(const Values& values, bool whiten = true) const;

  /// Evaluate the constraint violation (as L2 norm).
  double violationNorm(const Values& values) const;

  NonlinearFactorGraph penaltyGraph(const double mu = 1.0) const;

  NonlinearFactorGraph penaltyGraphSmooth(SmoothRampFunction::shared_ptr func,
                                          const double mu = 1.0) const;
};

}  // namespace gtsam
