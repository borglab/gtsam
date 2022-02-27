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

#include <gtsam/basis/Basis.h>
#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>

#include <CppUnitLite/TestHarness.h>

using gtsam::noiseModel::Isotropic;
using gtsam::Vector;
using gtsam::Values;
using gtsam::Chebyshev2;
using gtsam::ParameterMatrix;
using gtsam::LevenbergMarquardtParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph;
using gtsam::NonlinearOptimizerParams;

const size_t N = 2;

// Key for FunctorizedFactor
gtsam::Key key = gtsam::Symbol('X', 0);

//******************************************************************************
TEST(FunctorizedFactor, Print2) {
  using gtsam::VectorEvaluationFactor;
  const size_t M = 1;

  Vector measured = Vector::Ones(M) * 42;

  auto model = Isotropic::Sigma(M, 1.0);
  VectorEvaluationFactor<Chebyshev2, M> priorFactor(key, measured, model, N, 0);

  std::string expected =
      "  keys = { X0 }\n"
      "  noise model: unit (1) \n"
      "FunctorizedFactor(X0)\n"
      "  measurement: [\n"
      "	42\n"
      "]\n"
      "  noise model sigmas: 1\n";

  EXPECT(assert_print_equal(expected, priorFactor));
}

//******************************************************************************
TEST(FunctorizedFactor, VectorEvaluationFactor) {
  using gtsam::VectorEvaluationFactor;
  const size_t M = 4;

  Vector measured = Vector::Zero(M);

  auto model = Isotropic::Sigma(M, 1.0);
  VectorEvaluationFactor<Chebyshev2, M> priorFactor(key, measured, model, N, 0);

  NonlinearFactorGraph graph;
  graph.add(priorFactor);

  ParameterMatrix<M> stateMatrix(N);

  Values initial;
  initial.insert<ParameterMatrix<M>>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::SILENT;
  parameters.verbosityLM = LevenbergMarquardtParams::SILENT;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(FunctorizedFactor, VectorComponentFactor) {
  using gtsam::VectorComponentFactor;
  const int P = 4;
  const size_t i = 2;
  const double measured = 0.0, t = 3.0, a = 2.0, b = 4.0;
  auto model = Isotropic::Sigma(1, 1.0);
  VectorComponentFactor<Chebyshev2, P> controlPrior(key, measured, model, N, i,
                                                    t, a, b);

  NonlinearFactorGraph graph;
  graph.add(controlPrior);

  ParameterMatrix<P> stateMatrix(N);

  Values initial;
  initial.insert<ParameterMatrix<P>>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::SILENT;
  parameters.verbosityLM = LevenbergMarquardtParams::SILENT;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(FunctorizedFactor, VecDerivativePrior) {
  using gtsam::VectorDerivativeFactor;
  const size_t M = 4;

  Vector measured = Vector::Zero(M);
  auto model = Isotropic::Sigma(M, 1.0);
  VectorDerivativeFactor<Chebyshev2, M> vecDPrior(key, measured, model, N, 0);

  NonlinearFactorGraph graph;
  graph.add(vecDPrior);

  ParameterMatrix<M> stateMatrix(N);

  Values initial;
  initial.insert<ParameterMatrix<M>>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::SILENT;
  parameters.verbosityLM = LevenbergMarquardtParams::SILENT;
  parameters.setMaxIterations(20);
  Values result =
      LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-9);
}

//******************************************************************************
TEST(FunctorizedFactor, ComponentDerivativeFactor) {
  using gtsam::ComponentDerivativeFactor;
  const size_t M = 4;

  double measured = 0;
  auto model = Isotropic::Sigma(1, 1.0);
  ComponentDerivativeFactor<Chebyshev2, M> controlDPrior(key, measured, model,
                                                         N, 0, 0);

  NonlinearFactorGraph graph;
  graph.add(controlDPrior);

  Values initial;
  ParameterMatrix<M> stateMatrix(N);
  initial.insert<ParameterMatrix<M>>(key, stateMatrix);

  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::SILENT;
  parameters.verbosityLM = LevenbergMarquardtParams::SILENT;
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
