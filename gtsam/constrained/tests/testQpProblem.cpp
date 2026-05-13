/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQpProblem.cpp
 * @brief   Unit tests for QP constrained optimization problems.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <stdexcept>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace QpCostFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

Values VectorValue(const Vector& vector) {
  Values values;
  values.insert(x0, vector);
  return values;
}

Values MatrixValuesForTwoKeys(const Matrix& matrix0, const Matrix& matrix1) {
  Values values;
  values.insert(x0, matrix0);
  values.insert(x1, matrix1);
  return values;
}

double DirectTraceCost(const SymmetricBlockMatrix& Q, const Matrix& X0,
                       const Matrix& X1) {
  return 0.5 * ((X0.transpose() * Q.block(0, 0) * X0).trace() +
                (X0.transpose() * Q.block(0, 1) * X1).trace() +
                (X1.transpose() * Q.block(1, 0) * X0).trace() +
                (X1.transpose() * Q.block(1, 1) * X1).trace());
}

// Verifies affine Hessian factors evaluate direct Vector values.
TEST(QpCost, HessianVectorError) {
  const Matrix G = (Matrix(2, 2) << 2.0, 0.5, 0.5, 3.0).finished();
  const Vector g = (Vector(2) << 0.2, -0.3).finished();
  const QpCost cost(HessianFactor(x0, G, g, 1.7));
  const Vector x = (Vector(2) << 1.0, 2.0).finished();
  const double expected = 0.5 * (1.7 - 2.0 * x.dot(g) + x.dot(G * x));

  EXPECT_DOUBLES_EQUAL(expected, cost.error(VectorValue(x)), 1e-12);
}

// Verifies row-space matrix costs match the trace objective.
TEST(QpCost, RowSpaceMatrixError) {
  Matrix Q = Matrix::Zero(5, 5);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0, 5.0;
  Q.block<2, 3>(0, 2) << 0.2, -0.1, 0.4, 0.3, 0.5, -0.2;
  Q.block<3, 2>(2, 0) = Q.block<2, 3>(0, 2).transpose();

  const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{2, 3}, Q);
  const QpCost cost(KeyVector{x0, x1}, blockQ, 2);
  const Matrix X0 = (Matrix(2, 2) << 1.0, 2.0, -0.5, 0.25).finished();
  const Matrix X1 = (Matrix(3, 2) << 0.2, -0.4, 1.5, 0.7, -1.0, 0.3).finished();

  EXPECT_DOUBLES_EQUAL(DirectTraceCost(blockQ, X0, X1),
                       cost.error(MatrixValuesForTwoKeys(X0, X1)), 1e-12);
}

// Verifies QpCost linearizes exactly for direct Matrix values.
TEST(QpCost, LinearizeExact) {
  const Matrix G = (Matrix(2, 2) << 2.0, 0.5, 0.5, 3.0).finished();
  const Vector g = (Vector(2) << 0.2, -0.3).finished();
  const QpCost cost(HessianFactor(x0, G, g, 1.7));

  Values linearizationPoint;
  linearizationPoint.insert(x0, (Matrix(2, 1) << 1.5, 0.25).finished());
  Values perturbed;
  perturbed.insert(x0, (Matrix(2, 1) << 1.2, 0.5).finished());

  const LinearContainerFactor container(cost.linearize(linearizationPoint),
                                        linearizationPoint);

  EXPECT_DOUBLES_EQUAL(cost.error(perturbed), container.error(perturbed),
                       1e-12);
}

}  // namespace QpCostFixture
/* ************************************************************************* */
namespace LinearConstraintFixture {

const Key x0 = Symbol('x', 0);

Values VectorValue(double first, double second) {
  Values values;
  values.insert(x0, (Vector(2) << first, second).finished());
  return values;
}

// Verifies linear equalities evaluate direct Vector values.
TEST(LinearConstraint, EqualityError) {
  const Matrix A = (Matrix(1, 2) << 1.0, 2.0).finished();
  const LinearConstraint constraint =
      LinearConstraint::Equal(JacobianFactor(x0, A, Vector1(5.0)));
  const auto factor = constraint.createEqualityFactor();
  std::vector<Matrix> H(1);

  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(VectorValue(1.0, 2.0))(0),
                       1e-12);
  factor->unwhitenedError(VectorValue(1.0, 2.0), &H);
  EXPECT(assert_equal(A, H[0], 1e-12));
}

// Verifies <= constraints ramp only positive signed violations.
TEST(LinearConstraint, LessEqualViolation) {
  const Matrix A = (Matrix(1, 2) << 1.0, 0.0).finished();
  const LinearConstraint constraint =
      LinearConstraint::LessEqual(JacobianFactor(x0, A, Vector1(1.0)));
  const auto factor = constraint.createInequalityFactor();

  EXPECT_DOUBLES_EQUAL(-1.0, factor->unwhitenedExpr(VectorValue(0.0, 0.0))(0),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(VectorValue(0.0, 0.0))(0),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, factor->unwhitenedError(VectorValue(2.0, 0.0))(0),
                       1e-12);
}

// Verifies >= constraints are represented by negating the stored expression.
TEST(LinearConstraint, GreaterEqualViolation) {
  const Matrix A = (Matrix(1, 2) << 1.0, 0.0).finished();
  const LinearConstraint constraint =
      LinearConstraint::GreaterEqual(JacobianFactor(x0, A, Vector1(1.0)));
  const auto factor = constraint.createInequalityFactor();

  EXPECT_DOUBLES_EQUAL(1.0, factor->unwhitenedError(VectorValue(0.0, 0.0))(0),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(VectorValue(2.0, 0.0))(0),
                       1e-12);
}

// Verifies dimension mismatches are rejected at evaluation time.
TEST(LinearConstraint, DimensionMismatch) {
  const Matrix A = (Matrix(1, 3) << 1.0, 2.0, 3.0).finished();
  const LinearConstraint constraint =
      LinearConstraint::Equal(JacobianFactor(x0, A, Vector1(0.0)));
  const auto factor = constraint.createEqualityFactor();

  CHECK_EXCEPTION(factor->unwhitenedError(VectorValue(1.0, 2.0)),
                  std::invalid_argument);
}

// Verifies non-Vector and non-Matrix Values entries are rejected.
TEST(LinearConstraint, RejectNonVectorMatrix) {
  const LinearConstraint constraint = LinearConstraint::Equal(
      JacobianFactor(x0, (Matrix(1, 1) << 1.0).finished(), Vector1(0.0)));
  const auto factor = constraint.createEqualityFactor();
  Values values;
  values.insertDouble(x0, 1.0);

  CHECK_EXCEPTION(factor->unwhitenedError(values), std::invalid_argument);
}

}  // namespace LinearConstraintFixture
/* ************************************************************************* */
namespace QpProblemFixture {

const Key x0 = Symbol('x', 0);

Values VectorValue(double first, double second) {
  Values values;
  values.insert(x0, (Vector(2) << first, second).finished());
  return values;
}

Values ScalarValue(double value) {
  Values values;
  values.insert(x0, (Vector(1) << value).finished());
  return values;
}

// Verifies QpProblem evaluates QP costs and linear constraints directly.
TEST(QpProblem, Evaluate) {
  QpProblem problem;

  const Matrix G = (Matrix(2, 2) << 2.0, 0.5, 0.5, 3.0).finished();
  const Vector g = (Vector(2) << 0.2, -0.3).finished();
  problem.addCost(HessianFactor(x0, G, g, 1.7));

  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x0, (Matrix(1, 2) << 1.0, 2.0).finished(), Vector1(5.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x0, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(3.0))));

  const Values values = VectorValue(1.0, 2.0);
  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(values);
  const Vector x = (Vector(2) << 1.0, 2.0).finished();
  const double expectedCost = 0.5 * (1.7 - 2.0 * x.dot(g) + x.dot(G * x));

  EXPECT_DOUBLES_EQUAL(expectedCost, cost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, eqViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, ineqViolation, 1e-12);
}

// Verifies a QpProblem can be solved through the Augmented Lagrangian
// optimizer.
TEST(QpProblem, AugmentedLagrangianOptimize) {
  QpProblem problem;
  problem.addCost(HessianFactor(x0, Matrix11::Identity(), Vector1(2.0), 4.0));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x0, Matrix11::Identity(), Vector1(1.0))));

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->absoluteViolationTolerance = 1e-8;
  params->absoluteCostTolerance = 1e-8;
  params->relativeCostTolerance = 1e-8;
  AugmentedLagrangianOptimizer optimizer(problem, ScalarValue(0.0), params);

  const Values result = optimizer.optimize();

  EXPECT(assert_equal(ScalarValue(1.0), result, 1e-5));
}

}  // namespace QpProblemFixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
