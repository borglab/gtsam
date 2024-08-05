/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    constrainedExample.h
 * @brief   Simple constrained optimization scenarios.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/constraint/NonlinearInequalityConstraint.h>


namespace constrained_example {

using namespace gtsam;

/// Exponential function e^x.
double exp_func(const double& x, gtsam::OptionalJacobian<1, 1> H1 = {}) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

/// Exponential expression e^x.
Double_ exp(const Double_& x) { return Double_(exp_func, x); }

/// Pow functor used for pow function.
class PowFunctor {
 private:
  double c_;

 public:
  PowFunctor(const double& c) : c_(c) {}

  double operator()(const double& x,
                    gtsam::OptionalJacobian<1, 1> H1 = {}) const {
    if (H1) H1->setConstant(c_ * pow(x, c_ - 1));
    return pow(x, c_);
  }
};

/// Pow function.
Double_ pow(const Double_& x, const double& c) {
  PowFunctor pow_functor(c);
  return Double_(pow_functor, x);
}

/// Plus between Double expression and double.
Double_ operator+(const Double_& x, const double& d) { return x + Double_(d); }

/// Negative sign operator.
Double_ operator-(const Double_& x) { return Double_(0.0) - x; }

/// Keys for creating expressions.
Symbol x1_key('x', 1);
Symbol x2_key('x', 2);
Double_ x1(x1_key), x2(x2_key);

}  // namespace constrained_example

/* ************************************************************************* */
/**
 * Constrained optimization example in L. Vandenberghe slides:
 * https://www.seas.ucla.edu/~vandenbe/133B/lectures/nllseq.pdf
 * f(x) = 0.5*||x1 + e^(-x2)||^2 + 0.5*||x1^2 + 2*x2 + 1||^2
 * h(x) = x1 + x1^3 + x2 + x2^2
 */
namespace e_constrained_example {
using namespace constrained_example;
NonlinearFactorGraph GetCost() {
  NonlinearFactorGraph graph;
  auto f1 = x1 + exp(-x2);
  auto f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));
  return graph;
}

NonlinearEqualityConstraints GetConstraints() {
  NonlinearEqualityConstraints constraints;
  Vector scale = Vector1(0.1);
  auto h1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  constraints.push_back(NonlinearEqualityConstraint::shared_ptr(
      new ExpressionEqualityConstraint<double>(h1, 0.0, scale)));
  return constraints;
}

NonlinearFactorGraph cost = GetCost();
NonlinearEqualityConstraints constraints = GetConstraints();
} // namespace e_constrained_example

/* ************************************************************************* */
/**
 * Constrained optimization example with inequality constraints
 * f(x) = 0.5 * ||x1-1||^2 + 0.5 * ||x2-1||^2
 * g(x) = 1 - x1^2 - x2^2
 */
namespace i_constrained_example {
using namespace constrained_example;
NonlinearFactorGraph GetCost() {
  NonlinearFactorGraph graph;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.addPrior(x1_key, 1.0, cost_noise);
  graph.addPrior(x2_key, 1.0, cost_noise);
  return graph;
}

// InequalityConstraints GetIConstraints() {
//   InequalityConstraints i_constraints;
//   Double_ g1 = Double_(1.0) - x1 * x1 - x2 * x2;
//   double tolerance = 0.2;
//   i_constraints.emplace_shared<DoubleExpressionInequality>(g1, tolerance);
//   return i_constraints;
// }

NonlinearFactorGraph cost = GetCost();
NonlinearEqualityConstraints e_constraints;
// InequalityConstraints i_constraints = GetIConstraints();
} // namespace i_constrained_example

/* ************************************************************************* */
/**
 * Constrained optimization example with inequality constraints
 * f(x) = 0.5 * ||x1||^2 + 0.5 * ||x2||^2
 * h(x) = x1 + x2 - 1
 */
namespace e_constrained_example2 {
using namespace constrained_example;
NonlinearFactorGraph GetCost() {
  NonlinearFactorGraph graph;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.addPrior(x1_key, 1.0, cost_noise);
  // graph.addPrior(x2_key, 0.0, cost_noise);
  // graph.emplace_shared<gtsam::BetweenFactor<double>>(x1_key, x2_key, 1.0, cost_noise);
  return graph;
}

NonlinearEqualityConstraints GetEConstraints() {
  NonlinearEqualityConstraints e_constraints;
  // Double_ h1 = x1 + x2 - Double_(1.0);
  // double tolerance = 0.2;
  // e_constraints.emplace_shared<DoubleExpressionEquality>(h1, tolerance);
  return e_constraints;
}

NonlinearFactorGraph cost = GetCost();
NonlinearEqualityConstraints e_constraints = GetEConstraints();
} // namespace i_constrained_example
