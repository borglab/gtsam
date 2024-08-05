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

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

/// Ramp function for create penalty factors.
double RampFunction(const double x, OptionalJacobian<1, 1> H = {});

/** Base class for smooth approximation of the ramp function. */
class SmoothRampFunction {
 public:
  typedef std::shared_ptr<SmoothRampFunction> shared_ptr;
  typedef std::function<double(const double& x, OptionalJacobian<1, 1> H)> UnaryScalarFunc;

  /** Constructor. */
  SmoothRampFunction() {}

  /** Destructor. */
  virtual ~SmoothRampFunction() {}

  virtual double operator()(const double& x, OptionalJacobian<1, 1> H = {}) const = 0;

  UnaryScalarFunc function() const;
};

/** Ramp function approximated with a polynomial of degree 2.
 * Function f(x) =  0                   for   x <= 0
 *                  x^2/(2*e) + e/2     for   0 < x < epsilon
 *                  x                   for   x >= epsilon
 */
class PolynomialRampFunction : public SmoothRampFunction {
 public:
  typedef SmoothRampFunction Base;

 protected:
  double epsilon_;

 public:
  PolynomialRampFunction(const double epsilon = 1) : Base(), epsilon_(epsilon) {}

  virtual double operator()(const double& x, OptionalJacobian<1, 1> H = {}) const override;
};

/** Softplus function that implements f(x) = log(1 + exp(k*x)) / k. */
class SoftPlusFunction : public SmoothRampFunction {
 public:
  typedef SmoothRampFunction Base;

 protected:
  double k_;

 public:
  SoftPlusFunction(const double k = 1) : Base(), k_(k) {}

  virtual double operator()(const double& x, OptionalJacobian<1, 1> H = {}) const override;
};

}  // namespace gtsam
