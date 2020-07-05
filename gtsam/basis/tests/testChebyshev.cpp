/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testChebyshev.cpp
 * @date July 4, 2020
 * @author Varun Agrawal
 * @brief Unit tests for Chebyshev Basis Decompositions
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "../Chebyshev.h"
#include "../FitBasis.h"

using namespace std;
using namespace gtsam;

auto model = noiseModel::Unit::Create(1);

const size_t N = 3;

//******************************************************************************
TEST(Chebyshev, Chebyshev1) {
  typedef Chebyshev1Basis::EvaluationFunctor Synth;
  Vector c(N);
  double x;
  c << 12, 3, 1;
  x = -1.0;
  EXPECT_DOUBLES_EQUAL(12 + 3 * x + 2 * x * x - 1, Synth(N, x)(c), 1e-9);
  x = -0.5;
  EXPECT_DOUBLES_EQUAL(12 + 3 * x + 2 * x * x - 1, Synth(N, x)(c), 1e-9);
  x = 0.3;
  EXPECT_DOUBLES_EQUAL(12 + 3 * x + 2 * x * x - 1, Synth(N, x)(c), 1e-9);
}

//******************************************************************************
TEST(Chebyshev, Chebyshev2) {
  typedef Chebyshev2Basis::EvaluationFunctor Synth;
  Vector c(N);
  double x;
  c << 12, 3, 1;
  x = -1.0;
  EXPECT_DOUBLES_EQUAL(12 + 6 * x + 4 * x * x - 1, Synth(N, x)(c), 1e-9);
  x = -0.5;
  EXPECT_DOUBLES_EQUAL(12 + 6 * x + 4 * x * x - 1, Synth(N, x)(c), 1e-9);
  x = 0.3;
  EXPECT_DOUBLES_EQUAL(12 + 6 * x + 4 * x * x - 1, Synth(N, x)(c), 1e-9);
}

//******************************************************************************
TEST(Chebyshev, Prediction) {
  Chebyshev1Basis::EvaluationFunctor fx(N, 0.5);
  Vector c(N);
  c << 3, 5, -12;
  EXPECT_DOUBLES_EQUAL(11.5, fx(c), 1e-9);
}

//******************************************************************************
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
TEST(Chebyshev, Expression) {
  // Create linear factor graph
  NonlinearFactorGraph graph;
  Key key(1);

  // Let's pretend we have 6 GPS measurements (we just do x coordinate)
  // at times
  const size_t m = 6;
  Vector t(m);
  t << -0.7, -0.4, 0.1, 0.3, 0.7, 0.9;
  Vector x(m);
  x << -0.7, -0.4, 0.1, 0.3, 0.7, 0.9;

  for (size_t i = 0; i < m; i++) {
    graph.emplace_shared<PredictFactor<Chebyshev1Basis>>(key, x(i), model, N,
                                                         t(i));
  }

  // Solve
  Values initial;
  initial.insert<Vector>(key, Vector::Zero(N));  // initial does not matter

  // ... and optimize
  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check
  Vector expected(N);
  expected << 0, 1, 0;
  Vector actual_c = result.at<Vector>(key);
  EXPECT(assert_equal(expected, actual_c));

  // Calculate and print covariances
  Marginals marginals(graph, result);
  Matrix3 cov = marginals.marginalCovariance(key);
  EXPECT_DOUBLES_EQUAL(0.626, cov(1, 1), 1e-3);

  // Predict x at time 1.0
  Chebyshev1Basis::EvaluationFunctor f(N, 1.0);
  Matrix H;
  double actual = f(actual_c, H);
  EXPECT_DOUBLES_EQUAL(1.0, actual, 1e-9);

  // Calculate predictive variance on prediction
  double actual_variance_on_prediction = (H * cov * H.transpose())(0);
  EXPECT_DOUBLES_EQUAL(1.1494, actual_variance_on_prediction, 1e-4);
}

//******************************************************************************
TEST(Chebyshev, Decomposition) {
  const size_t M = 16;

  // Create example sequence
  Sequence sequence;
  for (size_t i = 0; i < M; i++) {
    double x = ((double)i / M);  // - 0.99;
    double y = x;
    sequence[x] = y;
  }

  // Do Chebyshev Decomposition
  FitBasis<Chebyshev1Basis> actual(N, sequence, model);

  // Check
  Vector expected = Vector::Zero(N);
  expected(1) = 1;
  EXPECT(assert_equal(expected, (Vector)actual.parameters(), 1e-4));
}

//******************************************************************************
TEST(Chebyshev1, Derivative) {
  typedef Chebyshev1Basis::Derivative D;
  Vector c(N);
  double x;
  c << 12, 3, 1;
  // D[12 + 3*x + 2*x*x-1,x] = 3 + 4*x
  x = -1.0;
  EXPECT_DOUBLES_EQUAL(3 + 4 * x, D(N, x)(c), 1e-9);
  x = -0.5;
  EXPECT_DOUBLES_EQUAL(3 + 4 * x, D(N, x)(c), 1e-9);
  x = 0.3;
  EXPECT_DOUBLES_EQUAL(3 + 4 * x, D(N, x)(c), 1e-9);
}

//******************************************************************************
Vector3 f(1, 0, 1);

double proxy1(double x, size_t N) {
  return Chebyshev1Basis::EvaluationFunctor(N, x)(Vector(f));
}

TEST(Chebyshev1, Derivative2) {
  const double x = 0.5;
  Chebyshev1Basis::Derivative dTdx(N, x);
  double analytic_dTdx = f(1) + 4 * f(2) * x;  // not too hard to do
  Matrix numeric_dTdx =
      numericalDerivative21<double, double, double>(proxy1, x, N);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), analytic_dTdx, 1e-9);
  EXPECT_DOUBLES_EQUAL(analytic_dTdx, dTdx(f), 1e-9);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
