/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1-------------------------------------------
 */

/**
 * @file testFunctorizedFactor.cpp
 * @date May 31, 2020
 * @author Varun Agrawal
 * @brief unit tests for FunctorizedFactor class
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/basis/Basis.h>
#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace std;
using namespace gtsam;

// Key for FunctorizedFactor
Key key = Symbol('X', 0);

// Keys for FunctorizedFactor2
Key keyA = Symbol('A', 0);
Key keyx = Symbol('x', 0);

auto model = noiseModel::Isotropic::Sigma(9, 1);
auto model2 = noiseModel::Isotropic::Sigma(3, 1);

/// Functor that takes a matrix and multiplies every element by m
class MultiplyFunctor {
  double m_;  ///< simple multiplier

 public:
  MultiplyFunctor(double m) : m_(m) {}

  Matrix operator()(const Matrix &X,
                    OptionalJacobian<-1, -1> H = boost::none) const {
    if (H) *H = m_ * Matrix::Identity(X.rows() * X.cols(), X.rows() * X.cols());
    return m_ * X;
  }
};

/// Functor that performs Ax where A is a matrix and x is a vector.
class ProjectionFunctor {
 public:
  Vector operator()(const Matrix &A, const Vector &x,
                    OptionalJacobian<-1, -1> H1 = boost::none,
                    OptionalJacobian<-1, -1> H2 = boost::none) const {
    if (H1) {
      H1->resize(x.size(), A.size());
      *H1 << I_3x3, I_3x3, I_3x3;
    }
    if (H2) *H2 = A;
    return A * x;
  }
};

/* ************************************************************************* */
// Test identity operation for FunctorizedFactor.
TEST(FunctorizedFactor, Identity) {
  Matrix X = Matrix::Identity(3, 3), measurement = Matrix::Identity(3, 3);

  double multiplier = 1.0;
  auto functor = MultiplyFunctor(multiplier);
  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model, functor);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test FunctorizedFactor with multiplier value of 2.
TEST(FunctorizedFactor, Multiply2) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                              MultiplyFunctor(multiplier));

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test equality function for FunctorizedFactor.
TEST(FunctorizedFactor, Equality) {
  Matrix measurement = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  auto factor1 = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                               MultiplyFunctor(multiplier));
  auto factor2 = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                               MultiplyFunctor(multiplier));

  EXPECT(factor1.equals(factor2));
}

/* ************************************************************************* */
// Test Jacobians of FunctorizedFactor.
TEST(FunctorizedFactor, Jacobians) {
  Matrix X = Matrix::Identity(3, 3);
  Matrix actualH;

  double multiplier = 2.0;

  auto factor =
      MakeFunctorizedFactor<Matrix>(key, X, model, MultiplyFunctor(multiplier));

  Values values;
  values.insert<Matrix>(key, X);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Test print result of FunctorizedFactor.
TEST(FunctorizedFactor, Print) {
  Matrix X = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  auto factor =
      MakeFunctorizedFactor<Matrix>(key, X, model, MultiplyFunctor(multiplier));

  string expected =
      "  keys = { X0 }\n"
      "  noise model: unit (9) \n"
      "FunctorizedFactor(X0)\n"
      "  measurement: [\n"
      "	1, 0;\n"
      "	0, 1\n"
      "]\n"
      "  noise model sigmas: 1 1 1 1 1 1 1 1 1\n";

  EXPECT(assert_print_equal(expected, factor));
}

/* ************************************************************************* */
// Test FunctorizedFactor using a std::function type.
TEST(FunctorizedFactor, Functional) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  std::function<Matrix(Matrix, boost::optional<Matrix &>)> functional =
      MultiplyFunctor(multiplier);
  auto factor =
      MakeFunctorizedFactor<Matrix>(key, measurement, model, functional);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test FunctorizedFactor with a lambda function.
TEST(FunctorizedFactor, Lambda) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  auto lambda = [multiplier](const Matrix &X,
                             OptionalJacobian<-1, -1> H = boost::none) {
    if (H)
      *H = multiplier *
           Matrix::Identity(X.rows() * X.cols(), X.rows() * X.cols());
    return multiplier * X;
  };
  // FunctorizedFactor<Matrix> factor(key, measurement, model, lambda);
  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model, lambda);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test identity operation for FunctorizedFactor2.
TEST(FunctorizedFactor, Identity2) {
  // x = Ax since A is I_3x3
  Matrix A = Matrix::Identity(3, 3);
  Vector x = Vector::Ones(3);

  auto functor = ProjectionFunctor();
  auto factor =
      MakeFunctorizedFactor2<Matrix, Vector>(keyA, keyx, x, model2, functor);

  Vector error = factor.evaluateError(A, x);

  EXPECT(assert_equal(Vector::Zero(3), error, 1e-9));
}

/* ************************************************************************* */
// Test Jacobians of FunctorizedFactor2.
TEST(FunctorizedFactor, Jacobians2) {
  Matrix A = Matrix::Identity(3, 3);
  Vector x = Vector::Ones(3);
  Matrix actualH1, actualH2;

  auto factor = MakeFunctorizedFactor2<Matrix, Vector>(keyA, keyx, x, model2,
                                                       ProjectionFunctor());

  Values values;
  values.insert<Matrix>(keyA, A);
  values.insert<Vector>(keyx, x);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Test FunctorizedFactor2 using a std::function type.
TEST(FunctorizedFactor, Functional2) {
  Matrix A = Matrix::Identity(3, 3);
  Vector3 x(1, 2, 3);
  Vector measurement = A * x;

  std::function<Matrix(Matrix, Matrix, boost::optional<Matrix &>,
                       boost::optional<Matrix &>)>
      functional = ProjectionFunctor();
  auto factor = MakeFunctorizedFactor2<Matrix, Vector>(keyA, keyx, measurement,
                                                       model2, functional);

  Vector error = factor.evaluateError(A, x);

  EXPECT(assert_equal(Vector::Zero(3), error, 1e-9));
}

/* ************************************************************************* */
// Test FunctorizedFactor2 with a lambda function.
TEST(FunctorizedFactor, Lambda2) {
  Matrix A = Matrix::Identity(3, 3);
  Vector3 x = Vector3(1, 2, 3);
  Matrix measurement = A * x;

  auto lambda = [](const Matrix &A, const Vector &x,
                   OptionalJacobian<-1, -1> H1 = boost::none,
                   OptionalJacobian<-1, -1> H2 = boost::none) {
    if (H1) {
      H1->resize(x.size(), A.size());
      *H1 << I_3x3, I_3x3, I_3x3;
    }
    if (H2) *H2 = A;
    return A * x;
  };
  // FunctorizedFactor<Matrix> factor(key, measurement, model, lambda);
  auto factor = MakeFunctorizedFactor2<Matrix, Vector>(keyA, keyx, measurement,
                                                       model2, lambda);

  Vector error = factor.evaluateError(A, x);

  EXPECT(assert_equal(Vector::Zero(3), error, 1e-9));
}

const size_t N = 2;

//******************************************************************************
TEST(FunctorizedFactor, Print2) {
  const size_t M = 1;

  Vector measured = Vector::Ones(M) * 42;

  auto model = noiseModel::Isotropic::Sigma(M, 1.0);
  VectorEvaluationFactor<Chebyshev2, M> priorFactor(key, measured, model, N, 0);

  string expected =
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
  const size_t M = 4;

  Vector measured = Vector::Zero(M);

  auto model = noiseModel::Isotropic::Sigma(M, 1.0);
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
  const int P = 4;
  const size_t i = 2;
  const double measured = 0.0, t = 3.0, a = 2.0, b = 4.0;
  auto model = noiseModel::Isotropic::Sigma(1, 1.0);
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
  const size_t M = 4;

  Vector measured = Vector::Zero(M);
  auto model = noiseModel::Isotropic::Sigma(M, 1.0);
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
  const size_t M = 4;

  double measured = 0;
  auto model = noiseModel::Isotropic::Sigma(1, 1.0);
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
