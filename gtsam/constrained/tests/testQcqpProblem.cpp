/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQcqpProblem.cpp
 * @brief   Unit tests for QCQP constrained optimization problems.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace RowSpaceQpCostFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

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

// Verifies vector values match the usual quadratic error.
TEST(QpCost, RowSpaceVectorError) {
  Matrix Q = Matrix::Zero(8, 8);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0, 1.5, 2.5, 3.5, 4.5;
  Q(0, 5) = Q(5, 0) = 0.2;
  Q(2, 7) = Q(7, 2) = -0.1;

  const QpCost factor(KeyVector{x0, x1},
                      SymmetricBlockMatrix(std::vector<DenseIndex>{4, 4}, Q));
  const Vector vector0 = (Vector(4) << 1.0, 2.0, 3.0, 4.0).finished();
  const Vector vector1 = (Vector(4) << -1.0, 0.5, 2.0, -0.5).finished();
  Values values;
  values.insert(x0, vector0);
  values.insert(x1, vector1);
  const Vector x = (Vector(8) << vector0, vector1).finished();

  EXPECT_DOUBLES_EQUAL(0.5 * x.dot(Q * x), factor.error(values), 1e-12);
}

// Verifies two-column matrix values use the row-space trace formula.
TEST(QpCost, RowSpaceMatrixErrorD2) {
  Matrix Q = Matrix::Zero(5, 5);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0, 5.0;
  Q.block<2, 3>(0, 2) << 0.2, -0.1, 0.4, 0.3, 0.5, -0.2;
  Q.block<3, 2>(2, 0) = Q.block<2, 3>(0, 2).transpose();

  const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{2, 3}, Q);
  const QpCost factor(KeyVector{x0, x1}, blockQ, 2);
  const Matrix X0 = (Matrix(2, 2) << 1.0, 2.0, -0.5, 0.25).finished();
  const Matrix X1 = (Matrix(3, 2) << 0.2, -0.4, 1.5, 0.7, -1.0, 0.3).finished();
  const Values values = MatrixValuesForTwoKeys(X0, X1);

  EXPECT_DOUBLES_EQUAL(DirectTraceCost(blockQ, X0, X1), factor.error(values),
                       1e-12);
}

// Verifies three-column matrix values use the same row-space trace formula.
TEST(QpCost, RowSpaceMatrixErrorD3) {
  Matrix Q = Matrix::Zero(4, 4);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0;
  Q.block<2, 2>(0, 2) << 0.2, -0.1, 0.3, 0.5;
  Q.block<2, 2>(2, 0) = Q.block<2, 2>(0, 2).transpose();

  const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{2, 2}, Q);
  const QpCost factor(KeyVector{x0, x1}, blockQ, 3);
  const Matrix X0 =
      (Matrix(2, 3) << 1.0, 0.2, -0.5, -0.25, 0.4, 0.7).finished();
  const Matrix X1 = (Matrix(2, 3) << -0.1, 1.2, 0.3, 0.6, -0.8, 0.5).finished();
  const Values values = MatrixValuesForTwoKeys(X0, X1);

  EXPECT_DOUBLES_EQUAL(DirectTraceCost(blockQ, X0, X1), factor.error(values),
                       1e-12);
}

// Verifies matrix-valued QCQP costs linearize to an exact Hessian factor.
TEST(QpCost, RowSpaceLinearizeExact) {
  Matrix Q = Matrix::Zero(4, 4);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0;
  Q(0, 2) = Q(2, 0) = 0.3;

  const QpCost factor(KeyVector{x0},
                      SymmetricBlockMatrix(std::vector<DenseIndex>{4}, Q));
  Values linearizationPoint;
  linearizationPoint.insert(x0,
                            (Matrix(4, 1) << 1.0, 0.1, -0.2, 0.7).finished());

  Values perturbed;
  perturbed.insert(x0, (Matrix(4, 1) << 0.9, 0.3, -0.4, 0.8).finished());

  const auto linearized = factor.linearize(linearizationPoint);
  const LinearContainerFactor container(linearized, linearizationPoint);

  EXPECT_DOUBLES_EQUAL(factor.error(perturbed), container.error(perturbed),
                       1e-12);
}

}  // namespace RowSpaceQpCostFixture
/* ************************************************************************* */
namespace QuadraticConstraintFixture {

const Key x0 = Symbol('x', 0);

Values MatrixValue(const Matrix& X) {
  Values values;
  values.insert(x0, X);
  return values;
}

Values VectorValue(const Vector& x) {
  Values values;
  values.insert(x0, x);
  return values;
}

// Verifies vector-valued quadratic constraints use the usual x' A x form.
TEST(QuadraticConstraint, VectorFeasible) {
  const Matrix A = Matrix::Identity(2, 2);
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 5.0);
  const auto factor = constraint.createEqualityFactor();
  const Values values = VectorValue((Vector(2) << 1.0, 2.0).finished());

  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(values)(0), 1e-12);
}

// Verifies a quadratic matrix equality evaluates to zero when satisfied.
TEST(QuadraticConstraint, Feasible) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 1.0);
  const auto factor = constraint.createEqualityFactor();
  const Values values = MatrixValue(Matrix::Identity(2, 3));

  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(values)(0), 1e-12);
}

// Verifies a quadratic matrix equality reports the expected scalar violation.
TEST(QuadraticConstraint, Infeasible) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 1.0);
  const auto factor = constraint.createEqualityFactor();
  const Values values =
      MatrixValue((Matrix(2, 2) << 1.0, 1.0, 0.0, 1.0).finished());

  EXPECT_DOUBLES_EQUAL(1.0, factor->unwhitenedError(values)(0), 1e-12);
}

// Verifies <= constraints ramp only positive signed violations.
TEST(QuadraticConstraint, LessEqualViolation) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint =
      QuadraticConstraint::LessEqual(x0, A, 1.0);
  const auto factor = constraint.createInequalityFactor();

  EXPECT_DOUBLES_EQUAL(
      -1.0, factor->unwhitenedExpr(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(
      0.0, factor->unwhitenedError(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(3.0,
                       factor->unwhitenedError(MatrixValue(
                           (Matrix(2, 1) << 2.0, 0.0).finished()))(0),
                       1e-12);
}

// Verifies >= constraints are represented by negating the stored expression.
TEST(QuadraticConstraint, GreaterEqualViolation) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint =
      QuadraticConstraint::GreaterEqual(x0, A, 1.0);
  const auto factor = constraint.createInequalityFactor();

  EXPECT_DOUBLES_EQUAL(
      1.0, factor->unwhitenedError(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0,
                       factor->unwhitenedError(MatrixValue(
                           (Matrix(2, 1) << 2.0, 0.0).finished()))(0),
                       1e-12);
}

}  // namespace QuadraticConstraintFixture
/* ************************************************************************* */
namespace QcqpProblemFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

Values ProblemValues() {
  Values values;
  values.insert(x0, (Matrix(2, 2) << 1.0, 0.0, 0.0, 1.0).finished());
  values.insert(x1, (Matrix(2, 2) << 0.2, -0.4, 1.5, 0.7).finished());
  return values;
}

// Verifies QcqpProblem evaluates direct vector-valued costs and constraints.
TEST(QcqpProblem, EvaluateVectorValues) {
  const Matrix Q = Matrix::Identity(2, 2);
  QcqpProblem problem;
  problem.addCost(QpCost(KeyVector{x0},
                         SymmetricBlockMatrix(std::vector<DenseIndex>{2}, Q)));
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 1.0));

  Values values;
  values.insert(x0, (Vector(2) << 1.0, 0.0).finished());

  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(values);
  EXPECT_DOUBLES_EQUAL(0.5, cost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, eqViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, ineqViolation, 1e-12);
}

// Verifies QcqpProblem evaluates manually assembled costs and constraints.
TEST(QcqpProblem, Evaluate) {
  Matrix Q = Matrix::Zero(4, 4);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0;
  Q.block<2, 2>(0, 2) << 0.2, -0.1, 0.3, 0.5;
  Q.block<2, 2>(2, 0) = Q.block<2, 2>(0, 2).transpose();

  const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{2, 2}, Q);
  NonlinearFactorGraph costs;
  costs.emplace_shared<QpCost>(KeyVector{x0, x1}, blockQ, 2);

  NonlinearEqualityConstraints constraints;
  constraints.push_back(
      QuadraticConstraint::Equal(x0, Matrix::Identity(2, 2), 2.0)
          .createEqualityFactor());

  const QcqpProblem problem(costs, constraints);
  const Values values = ProblemValues();
  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(values);
  const Matrix X0 = values.at<Matrix>(x0);
  const Matrix X1 = values.at<Matrix>(x1);

  EXPECT_DOUBLES_EQUAL(RowSpaceQpCostFixture::DirectTraceCost(blockQ, X0, X1),
                       cost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, eqViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, ineqViolation, 1e-12);
}

// Verifies ALM optimizes QCQPs with mixed linear/quadratic constraints.
TEST(QcqpProblem, OptimizeAugmentedLagrangianMixedConstraints) {
  QcqpProblem problem;

  const Matrix Q = Matrix::Identity(2, 2);
  problem.addCost(QpCost(KeyVector{x0},
                         SymmetricBlockMatrix(std::vector<DenseIndex>{2}, Q)));

  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x0, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x0, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.9))));
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 1.0));

  Matrix yBound = Matrix::Zero(2, 2);
  yBound(1, 1) = 1.0;
  problem.addConstraint(QuadraticConstraint::LessEqual(x0, yBound, 0.01));

  Values initialValues;
  initialValues.insert(x0, (Matrix(2, 1) << 0.8, 0.4).finished());

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 50;
  params->absoluteViolationTolerance = 1e-8;
  params->relativeViolationTolerance = 1e-8;
  params->relativeCostTolerance = 1e-8;

  const Values result =
      AugmentedLagrangianOptimizer(problem, initialValues, params).optimize();
  const Matrix expected = (Matrix(2, 1) << 1.0, 0.0).finished();
  EXPECT(assert_equal(expected, result.at<Matrix>(x0), 1e-4));

  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(result);
  EXPECT_DOUBLES_EQUAL(0.5, cost, 1e-4);
  EXPECT(eqViolation < 1e-4);
  EXPECT(ineqViolation < 1e-4);
}

}  // namespace QcqpProblemFixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
