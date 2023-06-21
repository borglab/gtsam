/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testFourier.cpp
 * @date July 4, 2020
 * @author Frank Dellaert, Varun Agrawal
 * @brief Unit tests for Fourier Basis Decompositions with Expressions
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Fourier.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace std;
using namespace gtsam;

auto model = noiseModel::Unit::Create(1);

// Coefficients for testing, respectively 3 and 7 parameter Fourier basis.
// They correspond to best approximation of TestFunction(x)
const Vector k3Coefficients = (Vector3() << 1.5661, 1.2717, 1.2717).finished();
const Vector7 k7Coefficients =
    (Vector7() << 1.5661, 1.2717, 1.2717, -0.0000, 0.5887, -0.0943, 0.0943)
        .finished();

// The test-function used below
static double TestFunction(double x) { return exp(sin(x) + cos(x)); }

//******************************************************************************
TEST(Basis, BasisEvaluationFunctor) {
  // fx(0) takes coefficients c to calculate the value f(c;x==0)
  FourierBasis::EvaluationFunctor fx(3, 0);
  EXPECT_DOUBLES_EQUAL(k3Coefficients[0] + k3Coefficients[1],
                       fx(k3Coefficients), 1e-9);
}

//******************************************************************************
TEST(Basis, BasisEvaluationFunctorDerivative) {
  // If we give the H argument, we get the derivative of fx(0) wrpt coefficients
  // Needs to be Matrix so it can be used by OptionalJacobian.
  Matrix H(1, 3);
  FourierBasis::EvaluationFunctor fx(3, 0);
  EXPECT_DOUBLES_EQUAL(k3Coefficients[0] + k3Coefficients[1],
                       fx(k3Coefficients, H), 1e-9);

  Matrix13 expectedH(1, 1, 0);
  EXPECT(assert_equal(expectedH, H));
}

//******************************************************************************
TEST(Basis, Manual) {
  const size_t N = 3;

  // We will create a linear factor graph
  GaussianFactorGraph graph;

  // We create an unknown Vector expression for the coefficients
  Key key(1);

  // We will need values below to test the Jacobians
  Values values;
  values.insert<Vector>(key, Vector::Zero(N));  // value does not really matter

  // At 16 different samples points x, check Predict_ expression
  for (size_t i = 0; i < 16; i++) {
    const double x = i * M_PI / 8;
    const double desiredValue = TestFunction(x);

    // Manual JacobianFactor
    Matrix A(1, N);
    A << 1, cos(x), sin(x);
    Vector b(1);
    b << desiredValue;
    JacobianFactor linearFactor(key, A, b);
    graph.add(linearFactor);

    // Create factor to predict value at x
    EvaluationFactor<FourierBasis> predictFactor(key, desiredValue, model, N,
                                                 x);

    // Check expression Jacobians
    EXPECT_CORRECT_FACTOR_JACOBIANS(predictFactor, values, 1e-5, 1e-9);

    auto linearizedFactor = predictFactor.linearize(values);
    auto linearizedJacobianFactor =
        std::dynamic_pointer_cast<JacobianFactor>(linearizedFactor);
    CHECK(linearizedJacobianFactor);  // makes sure it's indeed a JacobianFactor
    EXPECT(assert_equal(linearFactor, *linearizedJacobianFactor, 1e-9));
  }

  // Solve linear graph
  VectorValues actual = graph.optimize();
  EXPECT(assert_equal((Vector)k3Coefficients, actual.at(key), 1e-4));
}

//******************************************************************************
TEST(Basis, EvaluationFactor) {
  // Check fitting a function with a 7-parameter Fourier basis

  // Create linear factor graph
  NonlinearFactorGraph graph;
  Key key(1);
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, desiredValue = TestFunction(x);
    graph.emplace_shared<EvaluationFactor<FourierBasis>>(key, desiredValue,
                                                         model, 7, x);
  }

  // Solve FourierFactorGraph
  Values values;
  values.insert<Vector>(key, Vector::Zero(7));
  GaussianFactorGraph::shared_ptr lfg = graph.linearize(values);
  VectorValues actual = lfg->optimize();

  EXPECT(assert_equal((Vector)k7Coefficients, actual.at(key), 1e-4));
}

//******************************************************************************
TEST(Basis, WeightMatrix) {
  // The WeightMatrix creates an m*n matrix, where m is the number of sample
  // points, and n is the number of parameters.
  Matrix expected(2, 3);
  expected.row(0) << 1, cos(1), sin(1);
  expected.row(1) << 1, cos(2), sin(2);
  Vector2 X(1, 2);
  Matrix actual = FourierBasis::WeightMatrix(3, X);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

//******************************************************************************
TEST(Basis, Decomposition) {
  // Create example sequence
  Sequence sequence;
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, y = TestFunction(x);
    sequence[x] = y;
  }

  // Do Fourier Decomposition
  FitBasis<FourierBasis> actual(sequence, model, 3);

  // Check
  EXPECT(assert_equal((Vector)k3Coefficients, actual.parameters(), 1e-4));
}

//******************************************************************************
// Check derivative in two different ways: numerical and using D on f
double proxy(double x) {
  return FourierBasis::EvaluationFunctor(7, x)(k7Coefficients);
}

TEST(Basis, Derivative7) {
  // Check Derivative evaluation at point x=0.2

  // Calculate expected values by numerical derivative of proxy.
  const double x = 0.2;
  Matrix numeric_dTdx = numericalDerivative11<double, double>(proxy, x);

  // Calculate derivatives at Chebyshev points using D7, interpolate
  Matrix D7 = FourierBasis::DifferentiationMatrix(7);
  Vector derivativeCoefficients = D7 * k7Coefficients;
  FourierBasis::EvaluationFunctor fx(7, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), fx(derivativeCoefficients), 1e-8);

  // Do directly
  FourierBasis::DerivativeFunctor dfdx(7, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), dfdx(k7Coefficients), 1e-8);
}

//******************************************************************************
TEST(Basis, VecDerivativeFunctor) {
  using DotShape = typename FourierBasis::VectorDerivativeFunctor;
  const size_t N = 3;

  // MATLAB example, Dec 27 2019, commit 014eded5
  double h = 2 * M_PI / 16;
  Vector2 dotShape(0.5556, -0.8315);  // at h/2
  DotShape dotShapeFunction(2, N, h / 2);
  Matrix23 theta_mat = (Matrix32() << 0, 0, 0.7071, 0.7071, 0.7071, -0.7071)
                           .finished()
                           .transpose();
  ParameterMatrix theta(theta_mat);
  EXPECT(assert_equal(Vector(dotShape), dotShapeFunction(theta), 1e-4));
}

//******************************************************************************
// Suppose we want to parameterize a periodic function with function values at
// specific times, rather than coefficients. Can we do it? This would be a
// generalization of the Fourier transform, and constitute a "pseudo-spectral"
// parameterization. One way to do this is to establish hard constraints that
// express the relationship between the new parameters and the coefficients.
// For example, below I'd like the parameters to be the function values at
// X = {0.1,0.2,0.3}, rather than a 3-vector of coefficients.
// Because the values f(X) = at these points can be written as f(X) = W(X)*c,
// we can simply express the coefficents c as c=inv(W(X))*f, which is a
// generalized Fourier transform. That also means we can create factors with the
// unknown f-values, as done manually below.
TEST(Basis, PseudoSpectral) {
  // We will create a linear factor graph
  GaussianFactorGraph graph;

  const size_t N = 3;
  const Key key(1);

  // The correct values at X = {0.1,0.2,0.3} are simply W*c
  const Vector X = (Vector3() << 0.1, 0.2, 0.3).finished();
  const Matrix W = FourierBasis::WeightMatrix(N, X);
  const Vector expected = W * k3Coefficients;

  // Check those values are indeed correct values of Fourier approximation
  using Eval = FourierBasis::EvaluationFunctor;
  EXPECT_DOUBLES_EQUAL(Eval(N, 0.1)(k3Coefficients), expected(0), 1e-9);
  EXPECT_DOUBLES_EQUAL(Eval(N, 0.2)(k3Coefficients), expected(1), 1e-9);
  EXPECT_DOUBLES_EQUAL(Eval(N, 0.3)(k3Coefficients), expected(2), 1e-9);

  // Calculate "inverse Fourier transform" matrix
  const Matrix invW = W.inverse();

  // At 16 different samples points x, add a factor on fExpr
  for (size_t i = 0; i < 16; i++) {
    const double x = i * M_PI / 8;
    const double desiredValue = TestFunction(x);

    // Manual JacobianFactor
    Matrix A(1, 3);
    A << 1, cos(x), sin(x);
    Vector b(1);
    b << desiredValue;
    JacobianFactor linearFactor(key, A * invW, b);
    graph.add(linearFactor);
  }

  // Solve linear graph
  VectorValues actual = graph.optimize();
  EXPECT(assert_equal((Vector)expected, actual.at(key), 1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
