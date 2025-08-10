/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testAugmentedLagrangianOptimizr.cpp
 * @brief Test augmented Lagrangian method optimzier for equality constrained
 * optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>

#include "constrainedExample.h"
#include "gtsam/constrained/NonlinearEqualityConstraint.h"

using namespace gtsam;

/* ********************************************************************************************* */
double EvaluateLagrangeTerm(const std::vector<Vector>& lambdas,
                            const NonlinearEqualityConstraints& constraints,
                            const Values& values) {
  double s = 0;
  for (size_t i = 0; i < constraints.size(); i++) {
    const auto& constraint = constraints.at(i);
    s += lambdas.at(i).dot(constraint->whitenedError(values));
  }
  return s;
}

/* ********************************************************************************************* */
double EvaluateLagrangeTerm(const std::vector<double>& lambdas,
                            const NonlinearInequalityConstraints& constraints,
                            const Values& values) {
  double s = 0;
  for (size_t i = 0; i < constraints.size(); i++) {
    const auto& constraint = constraints.at(i);
    s += lambdas.at(i) * constraint->whitenedExpr(values)(0);
  }
  return s;
}

/* ********************************************************************************************* */
double ComputeBias(const std::vector<Vector>& lambdas, double mu) {
  double norm_squared = 0;
  for (const auto& lambda : lambdas) {
    norm_squared += pow(lambda.norm(), 2);
  }
  return 0.5 / mu * norm_squared;
}

/* ********************************************************************************************* */
double ComputeBias(const std::vector<double>& lambdas, double epsilon) {
  double norm_squared = 0;
  for (const auto& lambda : lambdas) {
    norm_squared += pow(lambda, 2);
  }
  return 0.5 / epsilon * norm_squared;
}

/* ********************************************************************************************* */
TEST(AugmentedLagrangian, constrained_example1) {
  using namespace constrained_example1;

  // Construct optimizer
  auto params = std::make_shared<AugmentedLagrangianParams>();
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);

  AugmentedLagrangianState state;
  state.lambdaEq.emplace_back(Vector1(0.3));
  state.muEq = 0.2;
  NonlinearFactorGraph augmentedLagrangian = optimizer.augmentedLagrangianFunction(state);

  const Values& values = init_values;
  double expected_cost = costs.error(values);
  double expected_l2_penalty = eqConstraints.penaltyGraph(state.muEq).error(values);
  double expected_lagrange_term = EvaluateLagrangeTerm(state.lambdaEq, eqConstraints, values);
  double bias = ComputeBias(state.lambdaEq, state.muEq);
  double expected_error = expected_cost + expected_l2_penalty + expected_lagrange_term + bias;
  EXPECT_DOUBLES_EQUAL(expected_error, augmentedLagrangian.error(values), 1e-6);
}

/* ********************************************************************************************* */
TEST(AugmentedLagrangian, constrained_example2) {
  using namespace constrained_example2;

  // Construct optimizer
  auto params = std::make_shared<AugmentedLagrangianParams>();
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);

  AugmentedLagrangianState state;
  state.lambdaEq.emplace_back(Vector1(0.3));
  state.lambdaIneq.emplace_back(-2.0);
  state.muEq = 0.2;
  state.muIneq = 0.3;
  double epsilon = 1.0;
  NonlinearFactorGraph augmentedLagrangian = optimizer.augmentedLagrangianFunction(state, epsilon);

  const Values& values = init_values;
  double expected_cost = costs.error(values);
  double expected_l2_penalty_e = eqConstraints.penaltyGraph(state.muEq).error(values);
  double expected_lagrange_term_e = EvaluateLagrangeTerm(state.lambdaEq, eqConstraints, values);
  double bias_e = ComputeBias(state.lambdaEq, state.muEq);
  double expected_penalty_i = ineqConstraints.penaltyGraph(state.muIneq).error(values);
  double expected_lagrange_term_i = EvaluateLagrangeTerm(state.lambdaIneq, ineqConstraints, values);
  double bias_i = ComputeBias(state.lambdaIneq, epsilon);
  double expected_error = expected_cost + expected_l2_penalty_e + expected_penalty_i +
                          expected_lagrange_term_e + expected_lagrange_term_i + bias_e + bias_i;

  EXPECT_DOUBLES_EQUAL(expected_error, augmentedLagrangian.error(values), 1e-6);
}

/* ********************************************************************************************* */
TEST(AugmentedLagrangianOptimizer, constrained_example1) {
  using namespace constrained_example1;

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->verbose = true;
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

/* ********************************************************************************************* */
TEST(AugmentedLagrangianOptimizer, constrained_example2) {
  using namespace constrained_example2;

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->verbose = true;
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

/* ********************************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
