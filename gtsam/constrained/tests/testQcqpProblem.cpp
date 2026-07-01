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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <string>
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
namespace QcqpSingleFactorFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

Values SingleFactorManifoldValues() {
  Values values;
  values.insert(x0, Rot2::fromAngle(0.3));
  values.insert(x1, Rot2::fromAngle(-0.2));
  return values;
}

Values SingleFactorQcqpValues() {
  Values values;
  InsertQcqpValue<Rot2, 1>(x0, Rot2::fromAngle(0.3), &values);
  InsertQcqpValue<Rot2, 1>(x1, Rot2::fromAngle(-0.2), &values);
  return values;
}

bool ThrowsMissingQcqpTraits(const NonlinearFactorGraph& graph) {
  try {
    QcqpProblem problem(graph);
  } catch (const std::runtime_error& exception) {
    return std::string(exception.what()).find("QCQP variable traits") !=
           std::string::npos;
  }
  return false;
}

// Verifies unsupported factors still reject QCQP conversion.
TEST(QcqpProblem, UnsupportedFactorThrows) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot2>>(x0, x1, Rot2::fromAngle(0.1));

  CHECK_EXCEPTION({ QcqpProblem problem(graph); }, std::runtime_error);
}

// Verifies missing QCQP variable traits produce a generic conversion error.
TEST(QcqpProblem, MissingQcqpTraitsThrows) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<SO3>>(
      x0, x1, SO3::Expmap((Vector3() << 0.1, 0.2, 0.3).finished()));

  EXPECT(ThrowsMissingQcqpTraits(graph));
}

// Verifies one Rot2 Frobenius factor converts exactly with D=1 QCQP variables.
TEST(QcqpProblem, SingleFrobeniusBetweenFactor) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1,
                                                     Rot2::fromAngle(0.4));

  const Values values = SingleFactorManifoldValues();
  const QcqpProblem problem(graph);
  const Values qcqpValues = SingleFactorQcqpValues();

  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(qcqpValues),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies hard Rot2 Frobenius priors become D=1 lifted linear equalities.
TEST(QcqpProblem, HardFrobeniusPriorRot2D1) {
  const Rot2 measured = Rot2::fromAngle(0.25);
  const auto hardNoise = noiseModel::Constrained::All(4);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<Rot2>>(x0, measured.matrix(), hardNoise);

  const QcqpProblem problem(graph);

  bool foundLinearEquality = false;
  for (const auto& factor : problem.eConstraints()) {
    if (dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get())) {
      foundLinearEquality = true;
      break;
    }
  }

  LONGS_EQUAL(0, problem.costs().size());
  EXPECT(foundLinearEquality);

  Values qcqpValues;
  InsertQcqpValue<Rot2, 1>(x0, measured, &qcqpValues);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies the deferred non-constrained Frobenius prior cost path rejects.
TEST(QcqpProblem, FrobeniusPriorRot2D1GaussianRejected) {
  const Rot2 measured = Rot2::fromAngle(0.25);
  const auto gaussianNoise = noiseModel::Isotropic::Sigma(4, 0.1);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<Rot2>>(x0, measured.matrix(),
                                             gaussianNoise);

  CHECK_EXCEPTION({ QcqpProblem problem(graph); }, std::runtime_error);
}

}  // namespace QcqpSingleFactorFixture
/* ************************************************************************* */
namespace QcqpRingFixture {

NonlinearFactorGraph RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

Values RingValues(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    values.insert(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)));
  }
  return values;
}

Values RingQcqpValues(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 1>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

// Verifies a Rot2 ring graph preserves cost and constraints for D=1 variables.
TEST(QcqpProblem, Rot2RingPgo) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values values = RingValues(N, delta, 0.01);
  const QcqpProblem problem(graph);
  const Values qcqpValues = RingQcqpValues(N, delta, 0.01);

  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(qcqpValues),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies the constrained optimizer still reduces a Rot2 QCQP ring problem.
TEST(QcqpProblem, AugmentedLagrangianOptimizerRot2Ring) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem problem(graph);
  const Values initialValues = RingQcqpValues(N, delta, 0.03);
  const double initialCost = problem.costs().error(initialValues);

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 5;
  params->initialMuEq = 10.0;
  params->muEqIncreaseRate = 5.0;
  params->absoluteViolationTolerance = 1e-6;
  params->absoluteCostTolerance = 1e-12;
  params->verbose = false;

  const AugmentedLagrangianOptimizer optimizer(problem, initialValues, params);
  const Values result = optimizer.optimize();

  EXPECT(problem.eConstraints().violationNorm(result) < 1e-5);
  EXPECT(problem.costs().error(result) <= initialCost + 1e-9);
}

}  // namespace QcqpRingFixture
/* ************************************************************************* */
namespace QcqpRot2VariableFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

template <int D>
Values QcqpValues(const Rot2& R0, const Rot2& R1) {
  Values values;
  InsertQcqpValue<Rot2, D>(x0, R0, &values);
  InsertQcqpValue<Rot2, D>(x1, R1, &values);
  return values;
}

template <int D>
double DirectQcqpBetweenCost(const Rot2& R0, const Rot2& R1,
                             const Rot2& measured) {
  const Matrix X0 = traits<Rot2>::template QcqpValue<D>(R0);
  const Matrix X1 = traits<Rot2>::template QcqpValue<D>(R1);
  const Matrix residual = X1 - measured.matrix().transpose() * X0;
  return 0.5 * residual.squaredNorm();
}

// Verifies the D=2 Rot2 QCQP variable is row-orthonormal and feasible.
TEST(QcqpProblem, Rot2D2QcqpValueConstraints) {
  const Rot2 R = Rot2::fromAngle(0.3);
  const Matrix X = traits<Rot2>::QcqpValue<2>(R);
  LONGS_EQUAL(2, X.rows());
  LONGS_EQUAL(2, X.cols());
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X * X.transpose(), 1e-12));

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot2, 2>(x0, &constraints);
  Values values;
  values.insert(x0, X);

  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Verifies the D=3 Rot2 QCQP variable reuses the D=2 row constraints.
TEST(QcqpProblem, Rot2D3QcqpValueConstraints) {
  const Rot2 R = Rot2::fromAngle(-0.4);
  const Matrix X = traits<Rot2>::QcqpValue<3>(R);
  LONGS_EQUAL(2, X.rows());
  LONGS_EQUAL(3, X.cols());
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X * X.transpose(), 1e-12));

  const auto constraintsD2 = traits<Rot2>::QcqpConstraints<2>();
  const auto constraintsD3 = traits<Rot2>::QcqpConstraints<3>();
  LONGS_EQUAL(constraintsD2.size(), constraintsD3.size());
  for (size_t i = 0; i < constraintsD2.size(); ++i) {
    EXPECT(assert_equal(constraintsD2[i].first, constraintsD3[i].first, 1e-12));
    EXPECT_DOUBLES_EQUAL(constraintsD2[i].second, constraintsD3[i].second,
                         1e-12);
  }

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot2, 3>(x0, &constraints);
  Values values;
  values.insert(x0, X);

  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Verifies D=2 row-space Frobenius conversion matches the manifold factor.
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD2) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1, measured);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  const QcqpProblem problem(graph, 2);
  const Values qcqpValues = QcqpValues<2>(R0, R1);

  EXPECT_DOUBLES_EQUAL(graph.error(manifoldValues),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies D=3 Frobenius conversion matches the manifold factor.
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD3) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1, measured);

  const QcqpProblem problem(graph, 3);
  const Values qcqpValues = QcqpValues<3>(R0, R1);
  const double qcqpCost = problem.costs().error(qcqpValues);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  EXPECT(std::isfinite(qcqpCost));
  EXPECT_DOUBLES_EQUAL(graph.error(manifoldValues), qcqpCost, 1e-12);
  EXPECT_DOUBLES_EQUAL(DirectQcqpBetweenCost<3>(R0, R1, measured), qcqpCost,
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

}  // namespace QcqpRot2VariableFixture
/* ************************************************************************* */
namespace QcqpTraitExtensionsFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

// Verifies Rot2 QcqpValue at D=4 zero-pads beyond the intrinsic dim.
TEST(QcqpProblem, Rot2QcqpValueD4Padding) {
  const Rot2 R = Rot2::fromAngle(0.7);
  const Matrix X4 = traits<Rot2>::template QcqpValue<4>(R);
  LONGS_EQUAL(2, X4.rows());
  LONGS_EQUAL(4, X4.cols());
  EXPECT(assert_equal(Matrix(R.matrix().transpose()),
                      Matrix(X4.leftCols<2>()), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix::Zero(2, 2)),
                      Matrix(X4.rightCols<2>()), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X4 * X4.transpose(), 1e-12));
}

// Verifies Rot2 matrix-form constraints are equal across D = 2, 3, 5.
TEST(QcqpProblem, Rot2QcqpConstraintsAreDIndependent) {
  const auto cs2 = traits<Rot2>::template QcqpConstraints<2>();
  const auto cs3 = traits<Rot2>::template QcqpConstraints<3>();
  const auto cs5 = traits<Rot2>::template QcqpConstraints<5>();
  LONGS_EQUAL(3, cs2.size());
  LONGS_EQUAL(cs2.size(), cs3.size());
  LONGS_EQUAL(cs2.size(), cs5.size());
  for (size_t i = 0; i < cs2.size(); ++i) {
    EXPECT(assert_equal(cs2[i].first, cs3[i].first, 1e-12));
    EXPECT(assert_equal(cs2[i].first, cs5[i].first, 1e-12));
    EXPECT_DOUBLES_EQUAL(cs2[i].second, cs3[i].second, 1e-12);
    EXPECT_DOUBLES_EQUAL(cs2[i].second, cs5[i].second, 1e-12);
  }
}

// Verifies Rot3 QcqpValue and QcqpConstraints at D=3.
TEST(QcqpProblem, Rot3D3QcqpValueConstraints) {
  const Rot3 R = Rot3::Expmap((Vector3() << 0.4, -0.1, 0.7).finished());
  const Matrix X = traits<Rot3>::template QcqpValue<3>(R);
  LONGS_EQUAL(3, X.rows());
  LONGS_EQUAL(3, X.cols());
  EXPECT(assert_equal(Matrix(R.matrix().transpose()), X, 1e-12));
  EXPECT(assert_equal(Matrix(Matrix3::Identity()), X * X.transpose(), 1e-12));

  const auto cs = traits<Rot3>::template QcqpConstraints<3>();
  LONGS_EQUAL(6, cs.size());

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot3, 3>(x0, &constraints);
  Values values;
  values.insert(x0, X);
  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Verifies Rot3 rejects D=2 for QcqpValue and QcqpConstraints.
TEST(QcqpProblem, Rot3D2Throws) {
  const Rot3 R = Rot3::Expmap(Vector3::Zero());
  CHECK_EXCEPTION(traits<Rot3>::template QcqpValue<2>(R),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Rot3>::template QcqpConstraints<2>(),
                  std::invalid_argument);
}

// Verifies FrobeniusBetweenFactor<Rot3> matches the manifold form at K=3.
TEST(QcqpProblem, Rot3FrobeniusBetweenFactorD3) {
  const Rot3 measured = Rot3::Expmap((Vector3() << 0.2, 0.1, -0.3).finished());
  const Rot3 R0 = Rot3::Expmap((Vector3() << 0.05, 0.10, 0.15).finished());
  const Rot3 R1 = Rot3::Expmap((Vector3() << 0.10, 0.20, 0.30).finished());

  NonlinearFactorGraph graph;
  auto noise = noiseModel::Isotropic::Sigma(Rot3::dimension, 1.0);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(x0, x1, measured, noise);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  const QcqpProblem problem(graph, 3);
  Values qcqpValues;
  InsertQcqpValue<Rot3, 3>(x0, R0, &qcqpValues);
  InsertQcqpValue<Rot3, 3>(x1, R1, &qcqpValues);
  const double qcqpCost = problem.costs().error(qcqpValues);

  EXPECT(std::isfinite(qcqpCost));
  EXPECT_DOUBLES_EQUAL(graph.error(manifoldValues), qcqpCost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies FrobeniusBetweenFactor<Rot2> matches the manifold form at K=4.
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD4) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  auto noise = noiseModel::Isotropic::Sigma(1, 1.0);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1, measured, noise);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  const QcqpProblem problem(graph, 4);
  Values qcqpValues;
  InsertQcqpValue<Rot2, 4>(x0, R0, &qcqpValues);
  InsertQcqpValue<Rot2, 4>(x1, R1, &qcqpValues);
  const double qcqpCost = problem.costs().error(qcqpValues);

  EXPECT(std::isfinite(qcqpCost));
  EXPECT_DOUBLES_EQUAL(graph.error(manifoldValues), qcqpCost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Verifies FrobeniusBetweenFactor<Rot3> rejects K=2 at QCQP construction.
TEST(QcqpProblem, Rot3FrobeniusBetweenFactorK2Rejected) {
  NonlinearFactorGraph graph;
  auto noise = noiseModel::Isotropic::Sigma(Rot3::dimension, 1.0);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
      x0, x1, Rot3::Expmap(Vector3::Zero()), noise);
  CHECK_EXCEPTION({ QcqpProblem problem(graph, /*K=*/2); },
                  std::invalid_argument);
}

}  // namespace QcqpTraitExtensionsFixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
