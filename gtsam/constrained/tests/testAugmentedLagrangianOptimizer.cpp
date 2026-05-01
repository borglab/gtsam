/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testAugmentedLagrangianOptimizer.cpp
 * @brief Test augmented Lagrangian method optimizer for equality constrained
 * optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

#include "constrainedExample.h"
#include "gtsam/constrained/NonlinearEqualityConstraint.h"

using namespace gtsam;

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
double ComputeBias(const std::vector<Vector>& lambdas, double mu) {
  double norm_squared = 0;
  for (const auto& lambda : lambdas) {
    norm_squared += pow(lambda.norm(), 2);
  }
  return 0.5 / mu * norm_squared;
}

/* ************************************************************************* */
double ComputeBias(const std::vector<double>& lambdas, double epsilon) {
  double norm_squared = 0;
  for (const auto& lambda : lambdas) {
    norm_squared += pow(lambda, 2);
  }
  return 0.5 / epsilon * norm_squared;
}

/* ************************************************************************* */
TEST(AugmentedLagrangian, constrained_example1) {
  using namespace constrained_example1;

  // Construct optimizer
  auto params = std::make_shared<AugmentedLagrangianParams>();
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);

  AugmentedLagrangianState state;
  state.lambdaEq.emplace_back(Vector1(0.3));
  state.muEq = 0.2;
  NonlinearFactorGraph augmentedLagrangian =
      optimizer.augmentedLagrangianFunction(state);

  const Values& values = init_values;
  double expected_cost = costs.error(values);
  double expected_l2_penalty =
      eqConstraints.penaltyGraph(state.muEq).error(values);
  double expected_lagrange_term =
      EvaluateLagrangeTerm(state.lambdaEq, eqConstraints, values);
  double bias = ComputeBias(state.lambdaEq, state.muEq);
  double expected_error =
      expected_cost + expected_l2_penalty + expected_lagrange_term + bias;
  EXPECT_DOUBLES_EQUAL(expected_error, augmentedLagrangian.error(values), 1e-6);
}

/* ************************************************************************* */
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
  NonlinearFactorGraph augmentedLagrangian =
      optimizer.augmentedLagrangianFunction(state, epsilon);

  const Values& values = init_values;
  double expected_cost = costs.error(values);
  double expected_l2_penalty_e =
      eqConstraints.penaltyGraph(state.muEq).error(values);
  double expected_lagrange_term_e =
      EvaluateLagrangeTerm(state.lambdaEq, eqConstraints, values);
  double bias_e = ComputeBias(state.lambdaEq, state.muEq);
  double expected_penalty_i =
      ineqConstraints.penaltyGraph(state.muIneq).error(values);
  double expected_lagrange_term_i =
      EvaluateLagrangeTerm(state.lambdaIneq, ineqConstraints, values);
  double bias_i = ComputeBias(state.lambdaIneq, epsilon);
  double expected_error = expected_cost + expected_l2_penalty_e +
                          expected_penalty_i + expected_lagrange_term_e +
                          expected_lagrange_term_i + bias_e + bias_i;

  EXPECT_DOUBLES_EQUAL(expected_error, augmentedLagrangian.error(values), 1e-6);
}

/* ************************************************************************* */
TEST(AugmentedLagrangianOptimizer, constrained_example1) {
  using namespace constrained_example1;

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->verbose = true;
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

/* ************************************************************************* */
TEST(AugmentedLagrangianOptimizer, constrained_example2) {
  using namespace constrained_example2;

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->verbose = true;
  AugmentedLagrangianOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

TEST(AugmentedLagrangian, VectorBiasUsesElementwiseSigmas) {
  using namespace gtsam;

  const Symbol x_key('x', 0);
  const Vector3_ x(x_key);

  // Non-uniform sigmas are essential to detect the bug in Release.
  Vector sigmas(3);
  sigmas << 1.0, 2.0, 4.0;

  NonlinearEqualityConstraints eq;
  eq.emplace_shared<ExpressionEqualityConstraint<Vector3>>(x, Vector3::Zero(),
                                                           sigmas);

  // Empty cost graph keeps the factor indexing simple.
  NonlinearFactorGraph costs;
  const auto problem =
      ConstrainedOptProblem::EqConstrainedOptProblem(costs, eq);

  Values values;
  values.insert<Vector3>(
      x_key,
      Vector3::Zero());  // satisfy constraint => penalty factor error is zero

  auto params = std::make_shared<AugmentedLagrangianParams>();
  AugmentedLagrangianOptimizer optimizer(problem, values, params);

  AugmentedLagrangianState state;
  state.muEq = 0.2;
  state.lambdaEq.emplace_back((Vector(3) << 0.3, -0.5, 1.2).finished());

  // Buggy code aborts here in Debug (Eigen assert) and gives wrong bias in
  // Release.
  const NonlinearFactorGraph graph =
      optimizer.augmentedLagrangianFunction(state);
  EXPECT_LONGS_EQUAL(1, static_cast<long>(graph.size()));

  auto nf = graph.at(0);
  auto factor = std::dynamic_pointer_cast<NoiseModelFactor>(nf);
  EXPECT(factor);

  const Vector bias_from_factor = factor->unwhitenedError(values);
  const Vector expected =
      (state.lambdaEq.at(0) / state.muEq).cwiseProduct(sigmas);
  EXPECT(assert_equal(expected, bias_from_factor, 1e-12));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
