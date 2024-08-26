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

// #include <gtsam/constraint/ConstrainedOptProblem.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/constraint/NonlinearInequalityConstraint.h>

namespace constrained_example {

using namespace gtsam;

/// Exponential function e^x.
inline double exp_func(const double& x, gtsam::OptionalJacobian<1, 1> H1 = {}) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

/// Exponential expression e^x.
inline Double_ exp(const Double_& x) { return Double_(exp_func, x); }

/// Pow functor used for pow function.
class PowFunctor {
 private:
  double c_;

 public:
  PowFunctor(const double& c) : c_(c) {}

  double operator()(const double& x, gtsam::OptionalJacobian<1, 1> H1 = {}) const {
    if (H1) H1->setConstant(c_ * pow(x, c_ - 1));
    return pow(x, c_);
  }
};

/// Pow function.
inline Double_ pow(const Double_& x, const double& c) {
  PowFunctor pow_functor(c);
  return Double_(pow_functor, x);
}

/// Plus between Double expression and double.
inline Double_ operator+(const Double_& x, const double& d) { return x + Double_(d); }

/// Negative sign operator.
inline Double_ operator-(const Double_& x) { return Double_(0.0) - x; }

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
 * h(x) = x1 + x1^3 + x2 + x2^2 = 0
 */
// namespace constrained_example1 {
// using namespace constrained_example;
// NonlinearFactorGraph Cost() {
//   NonlinearFactorGraph graph;
//   auto f1 = x1 + exp(-x2);
//   auto f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
//   auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
//   graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
//   graph.add(ExpressionFactor<double>(cost_noise, 0., f2));
//   return graph;
// }

// NonlinearEqualityConstraints EqConstraints() {
//   NonlinearEqualityConstraints constraints;
//   Vector sigmas = Vector1(1.0);
//   auto h1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
//   constraints.push_back(NonlinearEqualityConstraint::shared_ptr(
//       new ExpressionEqualityConstraint<double>(h1, 0.0, sigmas)));
//   return constraints;
// }

// Values InitValues() {
//   Values values;
//   values.insert(x1_key, -0.2);
//   values.insert(x2_key, -0.2);
//   return values;
// }

// Values OptimalValues() {
//   Values values;
//   values.insert(x1_key, 0.0);
//   values.insert(x2_key, 0.0);
//   return values;
// }

// NonlinearFactorGraph costs = Cost();
// NonlinearEqualityConstraints e_constraints = EqConstraints();
// NonlinearInequalityConstraints i_constraints;
// Values init_values = InitValues();
// ConstrainedOptProblem::shared_ptr problem =
//     std::make_shared<ConstrainedOptProblem>(costs, e_constraints, i_constraints, init_values);
// Values optimal_values = OptimalValues();
// }  // namespace constrained_example1

// /* ************************************************************************* */
// /**
//  * Constrained optimization example with inequality constraints
//  * Approach a point while staying on unit circle, and within an ellipse.
//  * f(x) = 0.5 * ||x1-1||^2 + 0.5 * ||x2-1||^2
//  * h(x) = x1^2 + x2^2 - 1 = 0
//  * g(x) = 4*x1^2 + 0.25*x2^2 - 1 <= 0
//  */
// namespace constrained_example2 {
// using namespace constrained_example;
// NonlinearFactorGraph Cost() {
//   NonlinearFactorGraph graph;
//   auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
//   graph.addPrior(x1_key, 1.0, cost_noise);
//   graph.addPrior(x2_key, 1.0, cost_noise);
//   return graph;
// }

// NonlinearEqualityConstraints EqConstraints() {
//   NonlinearEqualityConstraints constraints;
//   Vector1 sigmas(1.0);
//   Double_ h1 = x1 * x1 + x2 * x2;
//   constraints.emplace_shared<ExpressionEqualityConstraint<double>>(h1, 1.0, sigmas);
//   return constraints;
// }

// NonlinearInequalityConstraints IneqConstraints() {
//   NonlinearInequalityConstraints constraints;
//   Double_ g1 = 4 * x1 * x1 + 0.25 * x2 * x2 - Double_(1.0);
//   double sigma = 1;
//   constraints.emplace_shared<ScalarExpressionInequalityConstraint>(g1, sigma);
//   return constraints;
// }

// Values InitValues() {
//   Values values;
//   values.insert(x1_key, -1.0);
//   values.insert(x2_key, 2.0);
//   return values;
// }

// Values OptimalValues() {
//   Values values;
//   values.insert(x1_key, 1.0 / sqrt(5));
//   values.insert(x2_key, 2.0 / sqrt(5));
//   return values;
// }

// NonlinearFactorGraph costs = Cost();
// NonlinearEqualityConstraints e_constraints = EqConstraints();
// NonlinearInequalityConstraints i_constraints = IneqConstraints();
// Values init_values = InitValues();
// ConstrainedOptProblem::shared_ptr problem =
//     std::make_shared<ConstrainedOptProblem>(costs, e_constraints, i_constraints, init_values);
// Values optimal_values = OptimalValues();

// }  // namespace constrained_example2
