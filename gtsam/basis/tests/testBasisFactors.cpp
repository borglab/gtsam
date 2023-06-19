/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1-------------------------------------------
 */

/**
 * @file testBasisFactors.cpp
 * @date May 31, 2020
 * @author Varun Agrawal
 * @brief unit tests for factors in BasisFactors.h
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/basis/Basis.h>
#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/factorTesting.h>

using gtsam::Chebyshev2;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::NonlinearOptimizerParams;
using gtsam::ParameterMatrix;
using gtsam::Pose2;
using gtsam::Values;
using gtsam::Vector;
using gtsam::noiseModel::Isotropic;

constexpr size_t N = 2;

// Key used in all tests
const gtsam::Symbol key('X', 0);

//******************************************************************************
TEST(BasisFactors, EvaluationFactor) {
  using gtsam::EvaluationFactor;

  double measured = 0;

  auto model = Isotropic::Sigma(1, 1.0);
  EvaluationFactor<Chebyshev2> factor(key, measured, model, N, 0);

  NonlinearFactorGraph graph;
  graph.add(factor);

  Vector functionValues(N);
  functionValues.setZero();

  Values initial;
  initial.insert<Vector>(key, functionValues);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(BasisFactors, VectorEvaluationFactor) {
  using gtsam::VectorEvaluationFactor;
  const size_t M = 4;

  const Vector measured = Vector::Zero(M);

  auto model = Isotropic::Sigma(M, 1.0);
  VectorEvaluationFactor<Chebyshev2, M> factor(key, measured, model, N, 0);

  NonlinearFactorGraph graph;
  graph.add(factor);

  ParameterMatrix stateMatrix(M, N);

  Values initial;
  initial.insert<ParameterMatrix>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(BasisFactors, Print) {
  using gtsam::VectorEvaluationFactor;
  const size_t M = 1;

  const Vector measured = Vector::Ones(M) * 42;

  auto model = Isotropic::Sigma(M, 1.0);
  VectorEvaluationFactor<Chebyshev2, M> factor(key, measured, model, N, 0);

  std::string expected =
      "  keys = { X0 }\n"
      "  noise model: unit (1) \n"
      "FunctorizedFactor(X0)\n"
      "  measurement: [\n"
      "	42\n"
      "]\n"
      "  noise model sigmas: 1\n";

  EXPECT(assert_print_equal(expected, factor));
}

//******************************************************************************
TEST(BasisFactors, VectorComponentFactor) {
  using gtsam::VectorComponentFactor;
  const int P = 4;
  const size_t i = 2;
  const double measured = 0.0, t = 3.0, a = 2.0, b = 4.0;
  auto model = Isotropic::Sigma(1, 1.0);
  VectorComponentFactor<Chebyshev2, P> factor(key, measured, model, N, i, t, a,
                                              b);

  NonlinearFactorGraph graph;
  graph.add(factor);

  ParameterMatrix stateMatrix(P, N);

  Values initial;
  initial.insert<ParameterMatrix>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(BasisFactors, ManifoldEvaluationFactor) {
  using gtsam::ManifoldEvaluationFactor;
  const Pose2 measured;
  const double t = 3.0, a = 2.0, b = 4.0;
  auto model = Isotropic::Sigma(3, 1.0);
  ManifoldEvaluationFactor<Chebyshev2, Pose2> factor(key, measured, model, N, t,
                                                     a, b);

  NonlinearFactorGraph graph;
  graph.add(factor);

  ParameterMatrix stateMatrix(3, N);

  Values initial;
  initial.insert<ParameterMatrix>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, initial, 1e-7, 1e-5);
}

//******************************************************************************
TEST(BasisFactors, VecDerivativePrior) {
  using gtsam::VectorDerivativeFactor;
  const size_t M = 4;

  const Vector measured = Vector::Zero(M);
  auto model = Isotropic::Sigma(M, 1.0);
  VectorDerivativeFactor<Chebyshev2, M> vecDPrior(key, measured, model, N, 0);

  NonlinearFactorGraph graph;
  graph.add(vecDPrior);

  ParameterMatrix stateMatrix(M, N);

  Values initial;
  initial.insert<ParameterMatrix>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(BasisFactors, ComponentDerivativeFactor) {
  using gtsam::ComponentDerivativeFactor;
  const size_t M = 4;

  double measured = 0;
  auto model = Isotropic::Sigma(1, 1.0);
  ComponentDerivativeFactor<Chebyshev2, M> controlDPrior(key, measured, model,
                                                         N, 0, 0);

  NonlinearFactorGraph graph;
  graph.add(controlDPrior);

  Values initial;
  ParameterMatrix stateMatrix(M, N);
  initial.insert<ParameterMatrix>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
