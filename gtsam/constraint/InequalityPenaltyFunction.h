/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    InequalityPenaltyFunction.h
 * @brief   Ramp function to compute inequality constraint violations.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

/** Base class for smooth approximation of the ramp function. */
class GTSAM_EXPORT InequalityPenaltyFunction {
 public:
  typedef std::shared_ptr<InequalityPenaltyFunction> shared_ptr;
  typedef std::function<double(const double& x, OptionalJacobian<1, 1> H)>
      UnaryScalarFunc;

  /** Constructor. */
  InequalityPenaltyFunction() {}

  /** Destructor. */
  virtual ~InequalityPenaltyFunction() {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const = 0;

  virtual UnaryScalarFunc function() const;
};

/** Ramp function f : R -> R.
 *  f(x) =  0     for   x <= 0
 *          x     for   x > 0
 */
class GTSAM_EXPORT RampFunction : public InequalityPenaltyFunction {
 public:
  typedef InequalityPenaltyFunction Base;
  typedef RampFunction This;
  typedef std::shared_ptr<This> shared_ptr;

 public:
  RampFunction() : Base() {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const override {
    return Ramp(x, H);
  }

  virtual UnaryScalarFunc function() const override { return Ramp; }

  static double Ramp(const double x, OptionalJacobian<1, 1> H = {});
};

/** Ramp function approximated with a polynomial of degree 2 in [0, epsilon].
 * The coefficients are computed as
 *      a = 1 / (2 * epsilon)
 * Function f(x) =  0               for   x <= 0
 *                  a * x^2         for   0 < x < epsilon
 *                  x - epsilon/2   for   x >= epsilon
 */
class GTSAM_EXPORT SmoothRampPoly2 : public InequalityPenaltyFunction {
 public:
  typedef InequalityPenaltyFunction Base;
  typedef SmoothRampPoly2 This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double epsilon_;
  double a_;

 public:
  SmoothRampPoly2(const double epsilon = 1)
      : Base(), epsilon_(epsilon), a_(0.5 / epsilon) {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const override;
};

/** Ramp function approximated with a polynomial of degree 3 in [0, epsilon].
 * The coefficients are computed as
 *      a = -1 / epsilon^2
 *      b = 2 / epsilon
 * Function f(x) =  0                   for   x <= 0
 *                  a * x^3 + b * x^2   for   0 < x < epsilon
 *                  x                   for   x >= epsilon
 */
class GTSAM_EXPORT SmoothRampPoly3 : public InequalityPenaltyFunction {
 public:
  typedef InequalityPenaltyFunction Base;
  typedef SmoothRampPoly3 This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double epsilon_;
  double a_;
  double b_;

 public:
  SmoothRampPoly3(const double epsilon = 1)
      : Base(),
        epsilon_(epsilon),
        a_(-1 / (epsilon * epsilon)),
        b_(2 / epsilon) {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const override;
};

/** Softplus function that implements f(x) = log(1 + exp(k*x)) / k. */
class GTSAM_EXPORT SoftPlusFunction : public InequalityPenaltyFunction {
 public:
  typedef InequalityPenaltyFunction Base;
  typedef SoftPlusFunction This;
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  double k_;

 public:
  SoftPlusFunction(const double k = 1) : Base(), k_(k) {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const override;
};

}  // namespace gtsam
