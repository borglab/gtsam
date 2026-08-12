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
  using shared_ptr = std::shared_ptr<InequalityPenaltyFunction>;
  using UnaryScalarFunc =
      std::function<double(const double& x, OptionalJacobian<1, 1> H)>;

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
  using Base = InequalityPenaltyFunction;
  using This = RampFunction;
  using shared_ptr = std::shared_ptr<This>;

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
  using Base = InequalityPenaltyFunction;
  using This = SmoothRampPoly2;
  using shared_ptr = std::shared_ptr<This>;

 protected:
  double epsilon_;
  double a_;

 public:
  /** Constructor.
   * @param epsilon parameter for adjusting the smoothness of the function.
   */
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
  using Base = InequalityPenaltyFunction;
  using This = SmoothRampPoly3;
  using shared_ptr = std::shared_ptr<This>;

 protected:
  double epsilon_;
  double a_;
  double b_;

 public:
  /** Constructor.
   * @param epsilon parameter for adjusting the smoothness of the function.
   */
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
  using Base = InequalityPenaltyFunction;
  using This = SoftPlusFunction;
  using shared_ptr = std::shared_ptr<This>;

 protected:
  double k_;

 public:
  /** Constructor.
   * @param k parameter for adjusting the smoothness of the function.
   */
  SoftPlusFunction(const double k = 1) : Base(), k_(k) {}

  virtual double operator()(const double& x,
                            OptionalJacobian<1, 1> H = {}) const override;
};

}  // namespace gtsam
