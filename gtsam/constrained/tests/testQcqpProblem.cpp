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
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

using namespace gtsam;

// Mathematical reading guide:
// 1. RowSpaceQpCostFixture and QuadraticConstraintFixture establish the
//    trace-form primitives used by both tracks.
// 2. QcqpSingleFactorFixture and QcqpRingFixture cover the exact homogeneous
//    D=1 Rot2 lift and its global sign ambiguity.
// 3. QcqpRot2VariableFixture and QcqpTraitExtensionsFixture develop the
//    D>=N Stiefel track, right-O(D) gauge, and supported factor boundary.
// 4. QcqpConstraintInsertionFixture and QcqpExtractionFixture document the
//    conversion and best-effort recovery contracts.

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
  const Vector vector0{{1.0, 2.0, 3.0, 4.0}};
  const Vector vector1{{-1.0, 0.5, 2.0, -0.5}};
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
  const Matrix X0{{1.0, 2.0}, {-0.5, 0.25}};
  const Matrix X1{{0.2, -0.4}, {1.5, 0.7}, {-1.0, 0.3}};
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
  const Matrix X0{{1.0, 0.2, -0.5}, {-0.25, 0.4, 0.7}};
  const Matrix X1{{-0.1, 1.2, 0.3}, {0.6, -0.8, 0.5}};
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
  linearizationPoint.insert(x0, Matrix{{1.0}, {0.1}, {-0.2}, {0.7}});

  Values perturbed;
  perturbed.insert(x0, Matrix{{0.9}, {0.3}, {-0.4}, {0.8}});

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
  const Values values = VectorValue(Vector{{1.0, 2.0}});

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
  const Values values = MatrixValue(Matrix{{1.0, 1.0}, {0.0, 1.0}});

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
  EXPECT_DOUBLES_EQUAL(
      3.0, factor->unwhitenedError(MatrixValue(Matrix{{2.0}, {0.0}}))(0),
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
  EXPECT_DOUBLES_EQUAL(
      0.0, factor->unwhitenedError(MatrixValue(Matrix{{2.0}, {0.0}}))(0),
      1e-12);
}

}  // namespace QuadraticConstraintFixture
/* ************************************************************************* */
namespace QcqpProblemFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

Values ProblemValues() {
  Values values;
  values.insert(x0, Matrix{{1.0, 0.0}, {0.0, 1.0}});
  values.insert(x1, Matrix{{0.2, -0.4}, {1.5, 0.7}});
  return values;
}

// A runtime BM column dimension must remain positive even for an empty graph.
TEST(QcqpProblem, EmptyGraphRejectsZeroColumnDimension) {
  const NonlinearFactorGraph graph;
  CHECK_EXCEPTION({ QcqpProblem problem(graph, 0); }, std::invalid_argument);
}

// Verifies QcqpProblem evaluates direct vector-valued costs and constraints.
TEST(QcqpProblem, EvaluateVectorValues) {
  const Matrix Q = Matrix::Identity(2, 2);
  QcqpProblem problem;
  problem.addCost(QpCost(KeyVector{x0},
                         SymmetricBlockMatrix(std::vector<DenseIndex>{2}, Q)));
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 1.0));

  Values values;
  values.insert(x0, Vector{{1.0, 0.0}});

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
      JacobianFactor(x0, Matrix{{0.0, 1.0}}, Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x0, Matrix{{1.0, 0.0}}, Vector1(0.9))));
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 1.0));

  Matrix yBound = Matrix::Zero(2, 2);
  yBound(1, 1) = 1.0;
  problem.addConstraint(QuadraticConstraint::LessEqual(x0, yBound, 0.01));

  Values initialValues;
  initialValues.insert(x0, Matrix{{0.8}, {0.4}});

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 50;
  params->absoluteViolationTolerance = 1e-8;
  params->relativeViolationTolerance = 1e-8;
  params->relativeCostTolerance = 1e-8;

  const Values result =
      AugmentedLagrangianOptimizer(problem, initialValues, params).optimize();
  const Matrix expected{{1.0}, {0.0}};
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
      x0, x1, SO3::Expmap(Vector3{0.1, 0.2, 0.3}));

  EXPECT(ThrowsMissingQcqpTraits(graph));
}

// For x_i=[1;vec(R_i)], verifies the lifted quadratic cost equals
// 0.5*||R_2-R_1*M||_F^2 and all homogeneous SO(2) constraints are feasible.
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

template <typename T>
std::array<double, 4> D1FrobeniusBetweenFactorErrors(const T& value1,
                                                     const T& value2,
                                                     const T& measurement,
                                                     const Vector& sigmas) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<T>>(
      x0, x1, measurement, noiseModel::Diagonal::Sigmas(sigmas));

  Values values;
  values.insert(x0, value1);
  values.insert(x1, value2);

  Values qcqpValues;
  InsertQcqpValue<T, 1>(x0, value1, &qcqpValues);
  InsertQcqpValue<T, 1>(x1, value2, &qcqpValues);

  const QcqpProblem problem(graph);

  const T predicted = traits<T>::Compose(value1, measurement);
  Values predictedQcqpValues;
  InsertQcqpValue<T, 1>(x0, value1, &predictedQcqpValues);
  InsertQcqpValue<T, 1>(x1, predicted, &predictedQcqpValues);

  return {graph.error(values), problem.costs().error(qcqpValues),
          problem.eConstraints().violationNorm(qcqpValues),
          problem.costs().error(predictedQcqpValues)};
}

TEST(QcqpProblem, FrobeniusBetweenFactorRot3D1) {
  const Rot3 value1 = Rot3::Expmap(Vector3{0.2, -0.3, 0.4});
  const Rot3 value2 = Rot3::Expmap(Vector3{-0.1, 0.5, 0.2});
  const Rot3 measurement = Rot3::Expmap(Vector3{0.3, 0.1, -0.2});
  const auto errors = D1FrobeniusBetweenFactorErrors(
      value1, value2, measurement,
      (Vector9() << 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2).finished());
  EXPECT_DOUBLES_EQUAL(errors[0], errors[1], 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0, errors[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, errors[3], 1e-12);
}

TEST(QcqpProblem, FrobeniusBetweenFactorPose2D1) {
  const Pose2 value1(Rot2::fromAngle(0.2), Point2(1.0, -2.0));
  const Pose2 value2(Rot2::fromAngle(-0.4), Point2(-3.0, 0.5));
  const Pose2 measurement(Rot2::fromAngle(0.3), Point2(2.0, -1.0));
  const auto errors = D1FrobeniusBetweenFactorErrors(
      value1, value2, measurement,
      Vector9{0.4, 1.7, 0.5, 0.6, 1.8, 0.7, 0.8, 1.9, 0.9});
  EXPECT_DOUBLES_EQUAL(errors[0], errors[1], 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0, errors[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, errors[3], 1e-12);
}

TEST(QcqpProblem, FrobeniusBetweenFactorPose3D1) {
  const Pose3 value1(Rot3::Expmap(Vector3{0.2, -0.3, 0.4}),
                     Point3(1.0, -2.0, 0.5));
  const Pose3 value2(Rot3::Expmap(Vector3{-0.1, 0.5, 0.2}),
                     Point3(-3.0, 0.5, 2.0));
  const Pose3 measurement(Rot3::Expmap(Vector3{0.3, 0.1, -0.2}),
                          Point3(2.0, -1.0, 3.0));
  const auto errors = D1FrobeniusBetweenFactorErrors(
      value1, value2, measurement,
      Eigen::Matrix<double, 16, 1>{0.4, 0.5, 0.6, 1.7, 0.7, 0.8, 0.9, 1.8, 1.0,
                                   1.1, 1.2, 1.9, 1.3, 1.4, 1.5, 2.0});
  EXPECT_DOUBLES_EQUAL(errors[0], errors[1], 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0, errors[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, errors[3], 1e-12);
}

// A hard prior imposes x=[1;vec(M)], not merely a homogeneous direction.
// Since ||x||^2=1+||M||_F^2=3, the negated lift has violation 2*sqrt(3).
TEST(QcqpProblem, HardFrobeniusPriorRot2D1) {
  const Rot2 measured = Rot2::fromAngle(0.25);
  const auto hardNoise = noiseModel::Constrained::All(4);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<Rot2>>(x0, measured.matrix(), hardNoise);

  const QcqpProblem problem(graph);

  const LinearEqualityConstraintFactor* priorConstraint = nullptr;
  for (const auto& factor : problem.eConstraints()) {
    if (const auto* linear =
            dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get())) {
      priorConstraint = linear;
      break;
    }
  }

  LONGS_EQUAL(0, problem.costs().size());
  EXPECT(priorConstraint != nullptr);
  if (!priorConstraint) return;

  const Vector5 expectedTarget = traits<Rot2>::QcqpValue<1>(measured).col(0);
  const JacobianFactor& priorJacobian =
      priorConstraint->linearConstraint().factor();
  EXPECT(assert_equal(Matrix(Matrix5::Identity()), Matrix(priorJacobian.getA()),
                      1e-12));
  EXPECT(assert_equal(Vector(expectedTarget), Vector(priorJacobian.getb()),
                      1e-12));

  Values qcqpValues;
  InsertQcqpValue<Rot2, 1>(x0, measured, &qcqpValues);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);

  Values negatedQcqpValues;
  negatedQcqpValues.insert(x0, -traits<Rot2>::QcqpValue<1>(measured));
  EXPECT_DOUBLES_EQUAL(2.0 * std::sqrt(3.0),
                       problem.eConstraints().violationNorm(negatedQcqpValues),
                       1e-12);
}

template <typename T>
struct HardFrobeniusPriorD1Result {
  size_t costCount;
  bool foundPrior;
  Matrix A;
  Vector b;
  Matrix expectedTarget;
  double violation;
};

template <typename T>
HardFrobeniusPriorD1Result<T> HardFrobeniusPriorD1(const T& measured) {
  const auto hardNoise = noiseModel::Constrained::All(measured.matrix().size());

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<T>>(x0, measured.matrix(), hardNoise);
  const QcqpProblem problem(graph);

  const LinearEqualityConstraintFactor* priorConstraint = nullptr;
  for (const auto& factor : problem.eConstraints()) {
    if (const auto* linear =
            dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get())) {
      priorConstraint = linear;
      break;
    }
  }

  const Matrix expectedTarget = traits<T>::template QcqpValue<1>(measured);

  Values qcqpValues;
  InsertQcqpValue<T, 1>(x0, measured, &qcqpValues);
  if (!priorConstraint) {
    return {problem.costs().size(),
            false,
            Matrix(),
            Vector(),
            expectedTarget,
            problem.eConstraints().violationNorm(qcqpValues)};
  }

  const JacobianFactor& priorJacobian =
      priorConstraint->linearConstraint().factor();
  return {problem.costs().size(),
          true,
          Matrix(priorJacobian.getA()),
          Vector(priorJacobian.getb()),
          expectedTarget,
          problem.eConstraints().violationNorm(qcqpValues)};
}

TEST(QcqpProblem, HardFrobeniusPriorRot3D1) {
  const auto result =
      HardFrobeniusPriorD1(Rot3::Expmap(Vector3{0.2, -0.3, 0.4}));
  LONGS_EQUAL(0, result.costCount);
  EXPECT(result.foundPrior);
  EXPECT(assert_equal(Matrix::Identity(10, 10), result.A, 1e-12));
  EXPECT(assert_equal(Vector(result.expectedTarget.col(0)), result.b, 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, result.violation, 1e-12);
}

TEST(QcqpProblem, HardFrobeniusPriorPose2D1) {
  const auto result =
      HardFrobeniusPriorD1(Pose2(Rot2::fromAngle(0.2), Point2(1.0, -2.0)));
  LONGS_EQUAL(0, result.costCount);
  EXPECT(result.foundPrior);
  EXPECT(assert_equal(Matrix::Identity(7, 7), result.A, 1e-12));
  EXPECT(assert_equal(Vector(result.expectedTarget.col(0)), result.b, 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, result.violation, 1e-12);
}

TEST(QcqpProblem, HardFrobeniusPriorPose3D1) {
  const auto result = HardFrobeniusPriorD1(
      Pose3(Rot3::Expmap(Vector3{0.2, -0.3, 0.4}), Point3(1.0, -2.0, 0.5)));
  LONGS_EQUAL(0, result.costCount);
  EXPECT(result.foundPrior);
  EXPECT(assert_equal(Matrix::Identity(13, 13), result.A, 1e-12));
  EXPECT(assert_equal(Vector(result.expectedTarget.col(0)), result.b, 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, result.violation, 1e-12);
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

template <typename T>
NonlinearFactorGraph AnchoredPoseRingGraph(const std::vector<T>& poses) {
  constexpr size_t matrixDimension = T::LieAlgebra::RowsAtCompileTime;
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<T>>(
      Symbol('x', 0), poses[0].matrix(),
      noiseModel::Constrained::All(matrixDimension * matrixDimension));
  for (size_t i = 0; i < poses.size(); ++i) {
    const size_t j = (i + 1) % poses.size();
    graph.emplace_shared<FrobeniusBetweenFactor<T>>(
        Symbol('x', i), Symbol('x', j), poses[i].between(poses[j]));
  }
  return graph;
}

template <typename T>
Values PoseRingQcqpValues(const std::vector<T>& poses) {
  Values values;
  for (size_t i = 0; i < poses.size(); ++i) {
    InsertQcqpValue<T, 1>(Symbol('x', i), poses[i], &values);
  }
  return values;
}

template <typename T>
struct PoseRingOptimizationResult {
  double violation;
  double initialCost;
  double finalCost;
  std::vector<std::pair<Key, T>> recovered;
};

template <typename T>
PoseRingOptimizationResult<T> OptimizePoseRing(
    const std::vector<T>& groundTruth, const std::vector<T>& initialPoses) {
  const QcqpProblem problem(AnchoredPoseRingGraph(groundTruth));
  const Values initialValues = PoseRingQcqpValues(initialPoses);
  const double initialCost = problem.costs().error(initialValues);

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 10;
  params->absoluteViolationTolerance = 1e-6;
  params->verbose = false;

  const Values result =
      AugmentedLagrangianOptimizer(problem, initialValues, params).optimize();
  const auto recovered = ExtractQcqpValues<T, 1>(result);
  return {problem.eConstraints().violationNorm(result), initialCost,
          problem.costs().error(result), recovered};
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
  params->absoluteViolationTolerance = 1e-6;
  params->verbose = false;

  const AugmentedLagrangianOptimizer optimizer(problem, initialValues, params);
  const Values result = optimizer.optimize();

  EXPECT(problem.eConstraints().violationNorm(result) < 1e-5);
  EXPECT(problem.costs().error(result) <= initialCost + 1e-9);
}

// Verifies direct Pose2 QCQP optimization and conversion back to poses.
TEST(QcqpProblem, AugmentedLagrangianOptimizerPose2Ring) {
  constexpr size_t N = 3;
  const Pose2 step(2.0, 0.0, 2.0 * M_PI / static_cast<double>(N));
  std::vector<Pose2> groundTruth(N), initialPoses(N);
  for (size_t i = 1; i < N; ++i) {
    groundTruth[i] = groundTruth[i - 1].compose(step);
  }
  initialPoses = groundTruth;
  for (size_t i = 1; i < N; ++i) {
    const double scale = static_cast<double>(i);
    initialPoses[i] = groundTruth[i].retract(
        Vector3(0.02 * scale, -0.01 * scale, 0.015 * scale));
  }

  const auto result = OptimizePoseRing(groundTruth, initialPoses);
  EXPECT(result.violation < 1e-5);
  EXPECT(result.finalCost <= result.initialCost + 1e-9);
  LONGS_EQUAL(groundTruth.size(), result.recovered.size());
  for (size_t i = 0; i < result.recovered.size(); ++i) {
    LONGS_EQUAL(Symbol('x', i), result.recovered[i].first);
    EXPECT(groundTruth[i].localCoordinates(result.recovered[i].second).norm() <
           1e-4);
  }
}

// Verifies direct Pose3 QCQP optimization and conversion back to poses.
TEST(QcqpProblem, AugmentedLagrangianOptimizerPose3Ring) {
  constexpr size_t N = 3;
  const Pose3 step(Rot3::Rz(2.0 * M_PI / static_cast<double>(N)),
                   Point3(2.0, 0.0, 0.0));
  std::vector<Pose3> groundTruth(N), initialPoses(N);
  for (size_t i = 1; i < N; ++i) {
    groundTruth[i] = groundTruth[i - 1].compose(step);
  }
  initialPoses = groundTruth;
  for (size_t i = 1; i < N; ++i) {
    const double scale = static_cast<double>(i);
    Vector6 perturbation;
    perturbation << 0.004 * scale, -0.002 * scale, 0.003 * scale, 0.02 * scale,
        -0.01 * scale, 0.008 * scale;
    initialPoses[i] = groundTruth[i].retract(perturbation);
  }

  const auto result = OptimizePoseRing(groundTruth, initialPoses);
  EXPECT(result.violation < 1e-5);
  EXPECT(result.finalCost <= result.initialCost + 1e-9);
  LONGS_EQUAL(groundTruth.size(), result.recovered.size());
  for (size_t i = 0; i < result.recovered.size(); ++i) {
    LONGS_EQUAL(Symbol('x', i), result.recovered[i].first);
    EXPECT(groundTruth[i].localCoordinates(result.recovered[i].second).norm() <
           1e-4);
  }
}

// Every D=1 between cost and quadratic constraint is invariant under the
// global sign flip x_i -> -x_i; one hard prior rejects that second solution.
TEST(QcqpProblem, HardPriorPinsRot2RingSign) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  NonlinearFactorGraph graph = RingGraph(N, delta);
  graph.emplace_shared<FrobeniusPrior<Rot2>>(
      Symbol('x', 0), Matrix2::Identity(), noiseModel::Constrained::All(4));
  const QcqpProblem problem(graph);
  const Values canonical = RingQcqpValues(N, delta, 0.0);

  Values negated;
  for (size_t i = 0; i < N; ++i) {
    const Key key = Symbol('x', i);
    negated.insert(key, -canonical.at<Matrix>(key));
  }

  EXPECT_DOUBLES_EQUAL(problem.costs().error(canonical),
                       problem.costs().error(negated), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(canonical),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(2.0 * std::sqrt(3.0),
                       problem.eConstraints().violationNorm(negated), 1e-12);
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

// At D=N=2 the canonical lift X=R' satisfies XX'=I, hence X lies in O(2).
// These quadratic constraints alone do not distinguish rotations/reflections.
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

// At D=3 the canonical X=[R',0] still satisfies XX'=I; the three row-space
// equations are independent of D and define the row Stiefel manifold St(2,D).
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

// On canonical D=2 lifts, 0.5*||X_1-M'*X_0||_F^2 equals the original
// manifold Frobenius between-factor error.
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

// Zero padding at D=3 leaves 0.5*||X_1-M'*X_0||_F^2 unchanged, so the
// row-space QCQP and manifold errors agree exactly on canonical lifts.
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
  EXPECT(assert_equal(Matrix(R.matrix().transpose()), Matrix(X4.leftCols<2>()),
                      1e-12));
  EXPECT(assert_equal(Matrix(Matrix::Zero(2, 2)), Matrix(X4.rightCols<2>()),
                      1e-12));
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

// For Rot3 at D=1, x=[1; vec(R)] uses column-major matrix coordinates and
// satisfies the ten homogeneous SO(3) constraints.
TEST(QcqpProblem, Rot3D1QcqpValueConstraints) {
  const Rot3 R = Rot3::Expmap(Vector3{0.4, -0.1, 0.7});
  const Matrix3 rotation = R.matrix();
  const Matrix X = traits<Rot3>::template QcqpValue<1>(R);
  LONGS_EQUAL(10, X.rows());
  LONGS_EQUAL(1, X.cols());

  Vector10 expected;
  expected(0) = 1.0;
  expected.tail<9>() = Eigen::Map<const Vector9>(rotation.data());
  EXPECT(assert_equal(Vector(expected), Vector(X.col(0)), 1e-12));

  const auto constraints = traits<Rot3>::template QcqpConstraints<1>();
  LONGS_EQUAL(10, constraints.size());
  for (const auto& [A, b] : constraints) {
    LONGS_EQUAL(10, A.rows());
    LONGS_EQUAL(10, A.cols());
    EXPECT(assert_equal(A, Matrix(A.transpose()), 1e-12));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X).trace(), 1e-12);
  }

  Matrix3 raw{{1.0, 2.0, 3.0}, {4.0, 5.0, 7.0}, {8.0, 9.0, 11.0}};
  Vector10 rawX;
  rawX(0) = 1.0;
  rawX.tail<9>() = Eigen::Map<const Vector9>(raw.data());
  const Vector3 expectedOrientation = raw.col(1).cross(raw.col(2)) - raw.col(0);
  for (int k = 0; k < 3; ++k) {
    EXPECT_DOUBLES_EQUAL(
        expectedOrientation(k),
        (rawX.transpose() * constraints[k + 1].first * rawX)(0, 0), 1e-12);
  }

  const Matrix3 RRt = raw * raw.transpose();
  const std::array<std::pair<int, int>, 6> upperTriangle = {
      std::pair<int, int>{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 2}};
  for (size_t k = 0; k < upperTriangle.size(); ++k) {
    const auto [row, col] = upperTriangle[k];
    EXPECT_DOUBLES_EQUAL(
        RRt(row, col),
        (rawX.transpose() * constraints[k + 4].first * rawX)(0, 0), 1e-12);
  }

  NonlinearEqualityConstraints insertedConstraints;
  InsertQcqpConstraints<Rot3, 1>(x0, &insertedConstraints);
  Values values;
  values.insert(x0, X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(values), 1e-12);

  Values negatedValues;
  negatedValues.insert(x0, -X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(negatedValues),
                       1e-12);

  Matrix3 reflection = Matrix3::Identity();
  reflection(2, 2) = -1.0;
  Vector10 reflectedX;
  reflectedX(0) = 1.0;
  reflectedX.tail<9>() = Eigen::Map<const Vector9>(reflection.data());
  EXPECT(std::abs((reflectedX.transpose() * constraints[1].first * reflectedX)(
                      0, 0) -
                  constraints[1].second) > 1e-12);
  for (size_t k = 4; k < constraints.size(); ++k) {
    EXPECT_DOUBLES_EQUAL(
        constraints[k].second,
        (reflectedX.transpose() * constraints[k].first * reflectedX)(0, 0),
        1e-12);
  }
}

// Pose2 D=1 retains the first two homogeneous rows in column-major order and
// embeds the five SO(2) constraints without constraining translation.
TEST(QcqpProblem, Pose2D1QcqpValueConstraints) {
  const Pose2 pose(Rot2::fromAngle(0.4), Point2(2.0, -3.0));
  const Matrix3 T = pose.matrix();
  const Matrix X = traits<Pose2>::template QcqpValue<1>(pose);
  LONGS_EQUAL(7, X.rows());
  LONGS_EQUAL(1, X.cols());

  Vector7 expected{1.0, T(0, 0), T(1, 0), T(0, 1), T(1, 1), T(0, 2), T(1, 2)};
  EXPECT(assert_equal(Vector(expected), Vector(X.col(0)), 1e-12));

  const auto constraints = traits<Pose2>::template QcqpConstraints<1>();
  LONGS_EQUAL(5, constraints.size());
  for (const auto& [A, b] : constraints) {
    LONGS_EQUAL(7, A.rows());
    LONGS_EQUAL(7, A.cols());
    EXPECT(assert_equal(A, Matrix(A.transpose()), 1e-12));
    EXPECT(A.bottomRows(2).isZero(0.0));
    EXPECT(A.rightCols(2).isZero(0.0));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X).trace(), 1e-12);
  }

  Matrix2 rawR{{1.0, 2.0}, {3.0, 5.0}};
  Vector7 rawX{1.0, rawR(0, 0), rawR(1, 0), rawR(0, 1), rawR(1, 1), 7.0, 11.0};
  const Matrix2 RRt = rawR * rawR.transpose();
  const std::array<double, 5> expectedForms = {1.0, rawR.determinant(),
                                               RRt(0, 0), RRt(0, 1), RRt(1, 1)};
  for (size_t k = 0; k < constraints.size(); ++k) {
    EXPECT_DOUBLES_EQUAL(expectedForms[k],
                         (rawX.transpose() * constraints[k].first * rawX)(0, 0),
                         1e-12);
  }

  Vector7 translatedX = rawX;
  translatedX.tail<2>() << -13.0, 17.0;
  for (const auto& [A, b] : constraints) {
    (void)b;
    EXPECT_DOUBLES_EQUAL((rawX.transpose() * A * rawX)(0, 0),
                         (translatedX.transpose() * A * translatedX)(0, 0),
                         1e-12);
  }

  NonlinearEqualityConstraints insertedConstraints;
  InsertQcqpConstraints<Pose2, 1>(x0, &insertedConstraints);
  Values values;
  values.insert(x0, X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(values), 1e-12);

  Values negatedValues;
  negatedValues.insert(x0, -X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(negatedValues),
                       1e-12);

  Vector7 reflectedX{1.0, 1.0, 0.0, 0.0, -1.0, 2.0, -3.0};
  EXPECT(std::abs((reflectedX.transpose() * constraints[1].first *
                   reflectedX)(0, 0) -
                  constraints[1].second) >
         1e-12);
  for (size_t k = 2; k < constraints.size(); ++k) {
    EXPECT_DOUBLES_EQUAL(
        constraints[k].second,
        (reflectedX.transpose() * constraints[k].first * reflectedX)(0, 0),
        1e-12);
  }
}

// Pose3 D=1 retains the first three homogeneous rows in column-major order and
// embeds the ten SO(3) constraints without constraining translation.
TEST(QcqpProblem, Pose3D1QcqpValueConstraints) {
  const Pose3 pose(Rot3::Expmap(Vector3{0.4, -0.1, 0.7}),
                   Point3(2.0, -3.0, 4.0));
  const Matrix4 T = pose.matrix();
  const Matrix X = traits<Pose3>::template QcqpValue<1>(pose);
  LONGS_EQUAL(13, X.rows());
  LONGS_EQUAL(1, X.cols());

  Eigen::Matrix<double, 13, 1> expected;
  expected(0) = 1.0;
  expected.segment<3>(1) = T.col(0).head<3>();
  expected.segment<3>(4) = T.col(1).head<3>();
  expected.segment<3>(7) = T.col(2).head<3>();
  expected.segment<3>(10) = T.col(3).head<3>();
  EXPECT(assert_equal(Vector(expected), Vector(X.col(0)), 1e-12));

  const auto constraints = traits<Pose3>::template QcqpConstraints<1>();
  LONGS_EQUAL(10, constraints.size());
  for (const auto& [A, b] : constraints) {
    LONGS_EQUAL(13, A.rows());
    LONGS_EQUAL(13, A.cols());
    EXPECT(assert_equal(A, Matrix(A.transpose()), 1e-12));
    EXPECT(A.bottomRows(3).isZero(0.0));
    EXPECT(A.rightCols(3).isZero(0.0));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X).trace(), 1e-12);
  }

  Matrix3 rawR{{1.0, 2.0, 3.0}, {4.0, 5.0, 7.0}, {8.0, 9.0, 11.0}};
  Eigen::Matrix<double, 13, 1> rawX;
  rawX(0) = 1.0;
  rawX.segment<9>(1) = Eigen::Map<const Vector9>(rawR.data());
  rawX.tail<3>() << 12.0, 13.0, 14.0;
  const Vector3 expectedOrientation =
      rawR.col(1).cross(rawR.col(2)) - rawR.col(0);
  for (int k = 0; k < 3; ++k) {
    EXPECT_DOUBLES_EQUAL(
        expectedOrientation(k),
        (rawX.transpose() * constraints[k + 1].first * rawX)(0, 0), 1e-12);
  }

  const Matrix3 RRt = rawR * rawR.transpose();
  const std::array<std::pair<int, int>, 6> upperTriangle = {
      std::pair<int, int>{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 2}};
  for (size_t k = 0; k < upperTriangle.size(); ++k) {
    const auto [row, col] = upperTriangle[k];
    EXPECT_DOUBLES_EQUAL(
        RRt(row, col),
        (rawX.transpose() * constraints[k + 4].first * rawX)(0, 0), 1e-12);
  }

  Eigen::Matrix<double, 13, 1> translatedX = rawX;
  translatedX.tail<3>() << -17.0, 19.0, -23.0;
  for (const auto& [A, b] : constraints) {
    (void)b;
    EXPECT_DOUBLES_EQUAL((rawX.transpose() * A * rawX)(0, 0),
                         (translatedX.transpose() * A * translatedX)(0, 0),
                         1e-12);
  }

  NonlinearEqualityConstraints insertedConstraints;
  InsertQcqpConstraints<Pose3, 1>(x0, &insertedConstraints);
  Values values;
  values.insert(x0, X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(values), 1e-12);

  Values negatedValues;
  negatedValues.insert(x0, -X);
  EXPECT_DOUBLES_EQUAL(0.0, insertedConstraints.violationNorm(negatedValues),
                       1e-12);

  Eigen::Matrix<double, 13, 1> reflectedX{1.0, 1.0, 0.0,  0.0, 0.0,  1.0, 0.0,
                                          0.0, 0.0, -1.0, 2.0, -3.0, 4.0};
  EXPECT(std::abs((reflectedX.transpose() * constraints[1].first *
                   reflectedX)(0, 0) -
                  constraints[1].second) >
         1e-12);
  for (size_t k = 4; k < constraints.size(); ++k) {
    EXPECT_DOUBLES_EQUAL(
        constraints[k].second,
        (reflectedX.transpose() * constraints[k].first * reflectedX)(0, 0),
        1e-12);
  }
}

// For Rot3 at D=N=3, X=R' satisfies the six scalar equations equivalent to
// XX'=I: three unit-row equations and three row-orthogonality equations.
TEST(QcqpProblem, Rot3D3QcqpValueConstraints) {
  const Rot3 R = Rot3::Expmap(Vector3{0.4, -0.1, 0.7});
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

// Rot3 uses its exact D=1 variable for the D=1 between-cost lowering.
TEST(QcqpProblem, Rot3FrobeniusBetweenFactorD1Accepted) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(x0, x1, Rot3::Identity());
  const QcqpProblem problem(graph, 1);
  LONGS_EQUAL(1, problem.costs().size());
}

// For Rot3 canonical lifts, the D=3 row-space between cost is exactly the
// original 0.5*||R_2-R_1*M||_F^2 manifold cost.
TEST(QcqpProblem, Rot3FrobeniusBetweenFactorD3) {
  const Rot3 measured = Rot3::Expmap(Vector3{0.2, 0.1, -0.3});
  const Rot3 R0 = Rot3::Expmap(Vector3{0.05, 0.10, 0.15});
  const Rot3 R1 = Rot3::Expmap(Vector3{0.10, 0.20, 0.30});

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
  CHECK_EXCEPTION(
      { QcqpProblem problem(graph, /*K=*/2); }, std::invalid_argument);
}

// A fixed target ||X-Xbar|| is not right-O(D)-invariant and cannot be written
// only in terms of XX'; matrix priors await a BM-compatible anchor block.
TEST(QcqpProblem, MatrixFrobeniusPriorRejected) {
  NonlinearFactorGraph rot2Graph;
  rot2Graph.emplace_shared<FrobeniusPrior<Rot2>>(
      x0, Rot2::fromAngle(0.2).matrix(), noiseModel::Isotropic::Sigma(4, 1.0));
  CHECK_EXCEPTION({ QcqpProblem problem(rot2Graph, 2); }, std::runtime_error);

  NonlinearFactorGraph rot3Graph;
  rot3Graph.emplace_shared<FrobeniusPrior<Rot3>>(
      x0, Rot3::Identity().matrix(), noiseModel::Isotropic::Sigma(9, 1.0));
  CHECK_EXCEPTION({ QcqpProblem problem(rot3Graph, 3); }, std::runtime_error);
}

}  // namespace QcqpTraitExtensionsFixture

/* ************************************************************************* */
namespace QcqpConstraintInsertionFixture {

const Key x0 = Symbol('x', 0);

// Deduplication compares the actual trace(X'AX)=b equations: an unrelated
// unary linear constraint stays, all three St(2,D) equations appear once.
TEST(QcqpProblem, InsertQcqpConstraintsMatchesExactQuadratics) {
  NonlinearEqualityConstraints constraints;
  const Matrix selector{{1.0, 0.0, 0.0, 0.0}};
  constraints.push_back(
      LinearConstraint::Equal(JacobianFactor(x0, selector, Vector1(1.0)))
          .createEqualityFactor());

  InsertQcqpConstraints<Rot2, 2>(x0, &constraints);
  LONGS_EQUAL(4, constraints.size());
  InsertQcqpConstraints<Rot2, 2>(x0, &constraints);
  LONGS_EQUAL(4, constraints.size());
}

// Repeated graph factors retain every cost but register the six Rot3
// row-Stiefel constraints only once for each unique variable.
TEST(QcqpProblem, RepeatedFactorsRegisterUniqueVariableConstraints) {
  constexpr size_t kFactorCount = 50;
  const Key x1 = Symbol('x', 1);
  const auto noise = noiseModel::Isotropic::Sigma(Rot3::dimension, 1.0);
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < kFactorCount; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
        x0, x1, Rot3::RzRyRx(0.0, 0.0, 0.001 * i), noise);
  }

  const QcqpProblem problem(graph, 3);
  LONGS_EQUAL(kFactorCount, problem.costs().size());
  LONGS_EQUAL(12, problem.eConstraints().size());

  for (size_t i = 0; i < problem.eConstraints().size(); ++i) {
    const auto quadratic =
        std::dynamic_pointer_cast<QuadraticEqualityConstraintFactor>(
            problem.eConstraints().at(i));
    CHECK(quadratic != nullptr);
    LONGS_EQUAL(i < 6 ? x0 : x1, quadratic->quadraticConstraint().key());
  }

  Values manifoldValues;
  manifoldValues.insert(x0, Rot3::Identity());
  manifoldValues.insert(x1, Rot3::Identity());
  Values qcqpValues;
  InsertQcqpValue<Rot3, 3>(x0, Rot3::Identity(), &qcqpValues);
  InsertQcqpValue<Rot3, 3>(x1, Rot3::Identity(), &qcqpValues);
  EXPECT_DOUBLES_EQUAL(graph.error(manifoldValues),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

class QuadraticConstraintEmitter : public NonlinearFactor {
 public:
  explicit QuadraticConstraintEmitter(
      const std::vector<QuadraticConstraint>& constraints)
      : NonlinearFactor(KeyVector{x0}), constraints_(constraints) {}

  size_t dim() const override { return 0; }

  GaussianFactor::shared_ptr linearize(const Values&) const override {
    return nullptr;
  }

  void qcqpFactors(NonlinearFactorGraph*,
                   NonlinearEqualityConstraints* constraints,
                   size_t) const override {
    for (const auto& constraint : constraints_) {
      constraints->push_back(constraint.createEqualityFactor());
    }
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<QuadraticConstraintEmitter>(*this);
  }

 private:
  std::vector<QuadraticConstraint> constraints_;
};

class NonquadraticConstraintEmitter : public NonlinearFactor {
 public:
  explicit NonquadraticConstraintEmitter(
      const NonlinearEqualityConstraint::shared_ptr& constraint)
      : NonlinearFactor(KeyVector{x0}), constraint_(constraint) {}

  size_t dim() const override { return 0; }

  GaussianFactor::shared_ptr linearize(const Values&) const override {
    return nullptr;
  }

  void qcqpFactors(NonlinearFactorGraph*,
                   NonlinearEqualityConstraints* constraints,
                   size_t) const override {
    constraints->push_back(constraint_);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<NonquadraticConstraintEmitter>(*this);
  }

 private:
  NonlinearEqualityConstraint::shared_ptr constraint_;
};

// Indexed merging removes exact repeats across source factors without
// suppressing distinct quadratic equations on the same key or reordering them.
TEST(QcqpProblem, IndexedMergePreservesDistinctQuadratics) {
  const Matrix A0{{1.0, 0.0}, {0.0, 0.0}};
  const Matrix A1{{0.0, 0.0}, {0.0, 1.0}};
  const Matrix A2 = Matrix2::Identity();
  const QuadraticConstraint c0 = QuadraticConstraint::Equal(x0, A0, 1.0);
  const QuadraticConstraint c1 = QuadraticConstraint::Equal(x0, A1, 1.0);
  const QuadraticConstraint c2 = QuadraticConstraint::Equal(x0, A2, 2.0);

  NonlinearFactorGraph graph;
  graph.emplace_shared<QuadraticConstraintEmitter>(
      std::vector<QuadraticConstraint>{c0, c1});
  graph.emplace_shared<QuadraticConstraintEmitter>(
      std::vector<QuadraticConstraint>{c0, c2});

  const QcqpProblem problem(graph, 2);
  LONGS_EQUAL(3, problem.eConstraints().size());
  const std::array<Matrix, 3> expected{A0, A1, A2};
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto quadratic =
        std::dynamic_pointer_cast<QuadraticEqualityConstraintFactor>(
            problem.eConstraints().at(i));
    CHECK(quadratic != nullptr);
    EXPECT(
        assert_equal(expected[i], quadratic->quadraticConstraint().A(), 0.0));
  }

  Values values;
  values.insert(x0, Matrix(Matrix2::Identity()));
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(values),
                       1e-12);
}

// Nonquadratic equalities are never deduplicated as a side effect of the
// quadratic index, even when two source factors emit the same shared factor.
TEST(QcqpProblem, IndexedMergePreservesNonquadraticConstraints) {
  const Matrix A{{1.0, 0.0}};
  const auto constraint =
      LinearConstraint::Equal(JacobianFactor(x0, A, Vector1(1.0)))
          .createEqualityFactor();
  NonlinearFactorGraph graph;
  graph.emplace_shared<NonquadraticConstraintEmitter>(constraint);
  graph.emplace_shared<NonquadraticConstraintEmitter>(constraint);

  const QcqpProblem problem(graph, 2);
  LONGS_EQUAL(2, problem.eConstraints().size());
  EXPECT(problem.eConstraints().at(0) == constraint);
  EXPECT(problem.eConstraints().at(1) == constraint);
}

}  // namespace QcqpConstraintInsertionFixture
/* ************************************************************************* */
namespace QcqpExtractionFixture {

// Verify the published D=1 QCQP vector dimensions for each supported group.
TEST(QcqpProblem, QcqpVectorDimensions) {
  LONGS_EQUAL(5, traits<Rot2>::QcqpVectorDim);
  LONGS_EQUAL(10, traits<Rot3>::QcqpVectorDim);
  LONGS_EQUAL(7, traits<Pose2>::QcqpVectorDim);
  LONGS_EQUAL(13, traits<Pose3>::QcqpVectorDim);
}

// Verify that a D=1 QCQP value remains recoverable after homogeneous scaling.
template <typename T>
T ScaledD1RoundTrip(const T& value) {
  const Matrix X = -2.5 * traits<T>::template QcqpValue<1>(value);
  return traits<T>::template FromQcqpValue<1>(X);
}

// Rot2 D=1 conversion and recovery use the same column-major coordinates.
TEST(QcqpProblem, Rot2D1QcqpValueRoundTrip) {
  const Rot2 value = Rot2::fromAngle(0.4);
  EXPECT(assert_equal(value, ScaledD1RoundTrip(value), 1e-12));
}

// Rot3 D=1 conversion and recovery use the same column-major coordinates.
TEST(QcqpProblem, Rot3D1QcqpValueRoundTrip) {
  const Rot3 value = Rot3::RzRyRx(0.2, -0.3, 0.5);
  EXPECT(assert_equal(value, ScaledD1RoundTrip(value), 1e-12));
}

// Pose2 D=1 recovery preserves both rotation and translation.
TEST(QcqpProblem, Pose2D1QcqpValueRoundTrip) {
  const Pose2 value(Rot2::fromAngle(0.4), Point2(2.0, -3.0));
  EXPECT(assert_equal(value, ScaledD1RoundTrip(value), 1e-12));
}

// Pose3 D=1 recovery preserves both rotation and translation.
TEST(QcqpProblem, Pose3D1QcqpValueRoundTrip) {
  const Pose3 value(Rot3::RzRyRx(0.2, -0.3, 0.5), Point3(2.0, -3.0, 4.0));
  EXPECT(assert_equal(value, ScaledD1RoundTrip(value), 1e-12));
}

// D=1 recovery rejects incorrect dimensions and zero homogenization entries.
TEST(QcqpProblem, D1QcqpValueRecoveryRejectsInvalidVectors) {
  CHECK_EXCEPTION(traits<Rot2>::template FromQcqpValue<1>(Matrix::Zero(4, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Rot3>::template FromQcqpValue<1>(Matrix::Zero(9, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Pose2>::template FromQcqpValue<1>(Matrix::Zero(6, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Pose3>::template FromQcqpValue<1>(Matrix::Zero(12, 1)),
                  std::invalid_argument);

  Matrix rot2 = traits<Rot2>::template QcqpValue<1>(Rot2());
  Matrix rot3 = traits<Rot3>::template QcqpValue<1>(Rot3());
  Matrix pose2 = traits<Pose2>::template QcqpValue<1>(Pose2());
  Matrix pose3 = traits<Pose3>::template QcqpValue<1>(Pose3());
  rot2(0, 0) = 0.0;
  rot3(0, 0) = 0.0;
  pose2(0, 0) = 0.0;
  pose3(0, 0) = 0.0;
  CHECK_EXCEPTION(traits<Rot2>::template FromQcqpValue<1>(rot2),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Rot3>::template FromQcqpValue<1>(rot3),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Pose2>::template FromQcqpValue<1>(pose2),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Pose3>::template FromQcqpValue<1>(pose3),
                  std::invalid_argument);
}

// D=1 extraction selects each group's exact homogenized vector dimension.
TEST(QcqpProblem, ExtractD1QcqpValues) {
  Values values;
  const Rot2 rot2 = Rot2::fromAngle(0.4);
  const Rot3 rot3 = Rot3::RzRyRx(0.2, -0.3, 0.5);
  const Pose2 pose2(Rot2::fromAngle(-0.2), Point2(1.0, -2.0));
  const Pose3 pose3(Rot3::RzRyRx(-0.1, 0.3, -0.4), Point3(1.0, -2.0, 3.0));
  InsertQcqpValue<Rot2, 1>(Symbol('r', 2), rot2, &values);
  InsertQcqpValue<Rot3, 1>(Symbol('r', 3), rot3, &values);
  InsertQcqpValue<Pose2, 1>(Symbol('p', 2), pose2, &values);
  InsertQcqpValue<Pose3, 1>(Symbol('p', 3), pose3, &values);
  const Matrix foreignValue = Matrix::Zero(6, 1);
  values.insert(Symbol('z', 0), foreignValue);

  const auto rot2Values = ExtractQcqpValues<Rot2, 1>(values);
  const auto rot3Values = ExtractQcqpValues<Rot3, 1>(values);
  const auto pose2Values = ExtractQcqpValues<Pose2, 1>(values);
  const auto pose3Values = ExtractQcqpValues<Pose3, 1>(values);
  LONGS_EQUAL(1, rot2Values.size());
  LONGS_EQUAL(1, rot3Values.size());
  LONGS_EQUAL(1, pose2Values.size());
  LONGS_EQUAL(1, pose3Values.size());
  EXPECT(assert_equal(rot2, rot2Values.front().second, 1e-12));
  EXPECT(assert_equal(rot3, rot3Values.front().second, 1e-12));
  EXPECT(assert_equal(pose2, pose2Values.front().second, 1e-12));
  EXPECT(assert_equal(pose3, pose3Values.front().second, 1e-12));
}

// For canonical X=R', the leading-block projection returns R exactly.
TEST(QcqpProblem, ExtractQcqpValuesRot2) {
  Values values;
  const Rot2 R0 = Rot2::fromAngle(0.25);
  const Rot2 R1 = Rot2::fromAngle(-1.10);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 0), R0, &values);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 1), R1, &values);

  const auto extracted = ExtractQcqpValues<Rot2, 2>(values);
  LONGS_EQUAL(2, extracted.size());

  Values out;
  for (auto& [key, R] : extracted) out.insert(key, R);
  EXPECT(assert_equal(R0, out.at<Rot2>(Symbol('x', 0)), 1e-12));
  EXPECT(assert_equal(R1, out.at<Rot2>(Symbol('x', 1)), 1e-12));
}

// For canonical Rot3 X=R', the leading-block projection returns R exactly.
TEST(QcqpProblem, ExtractQcqpValuesRot3) {
  Values values;
  const Rot3 R0 = Rot3::Rz(0.25);
  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.4, 0.7);
  InsertQcqpValue<Rot3, 3>(Symbol('x', 0), R0, &values);
  InsertQcqpValue<Rot3, 3>(Symbol('x', 1), R1, &values);

  const auto extracted = ExtractQcqpValues<Rot3, 3>(values);
  LONGS_EQUAL(2, extracted.size());

  Values out;
  for (auto& [key, R] : extracted) out.insert(key, R);
  EXPECT(assert_equal(R0, out.at<Rot3>(Symbol('x', 0)), 1e-12));
  EXPECT(assert_equal(R1, out.at<Rot3>(Symbol('x', 1)), 1e-12));
}

// Extraction scans mixed Values but accepts only exact N-by-D matrix slices;
// matching row count alone is insufficient because D defines the lift.
TEST(QcqpProblem, ExtractQcqpValuesSkipsForeignSlices) {
  Values values;
  const Rot2 R2 = Rot2::fromAngle(0.3);
  const Rot3 R3 = Rot3::Rz(0.5);
  InsertQcqpValue<Rot2, 3>(Symbol('a', 0), R2, &values);
  InsertQcqpValue<Rot2, 3>(Symbol('a', 1), R2, &values);
  InsertQcqpValue<Rot3, 3>(Symbol('b', 0), R3, &values);
  values.insert(Symbol('b', 1), Matrix(Matrix::Identity(3, 4)));

  const auto rot3s = ExtractQcqpValues<Rot3, 3>(values);
  LONGS_EQUAL(1, rot3s.size());
  EXPECT(rot3s.front().first == Symbol('b', 0));
  EXPECT(assert_equal(R3, rot3s.front().second, 1e-12));
}

// For D=2 and G in SO(2), X_iG extracts as G^{-1}R_i: absolute rotations
// change, while (G^{-1}R_0)^{-1}(G^{-1}R_1)=R_0^{-1}R_1 remains invariant.
TEST(QcqpProblem, UnanchoredExtractionIsGaugeDependent) {
  const Rot2 R0 = Rot2::fromAngle(0.25);
  const Rot2 R1 = Rot2::fromAngle(-0.7);
  const Rot2 gaugeRotation = Rot2::fromAngle(0.4);
  const Matrix gauge = gaugeRotation.matrix();

  Values canonical;
  InsertQcqpValue<Rot2, 2>(Symbol('x', 0), R0, &canonical);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 1), R1, &canonical);
  Values gauged;
  gauged.insert(Symbol('x', 0),
                (canonical.at<Matrix>(Symbol('x', 0)) * gauge).eval());
  gauged.insert(Symbol('x', 1),
                (canonical.at<Matrix>(Symbol('x', 1)) * gauge).eval());

  const auto extracted = ExtractQcqpValues<Rot2, 2>(gauged);
  LONGS_EQUAL(2, extracted.size());
  const Rot2 expected0 = gaugeRotation.inverse().compose(R0);
  const Rot2 expected1 = gaugeRotation.inverse().compose(R1);
  EXPECT(assert_equal(expected0, extracted[0].second, 1e-12));
  EXPECT(assert_equal(expected1, extracted[1].second, 1e-12));
  EXPECT(assert_equal(R0.between(R1),
                      extracted[0].second.between(extracted[1].second), 1e-12));
}

// Direct trait projection rejects matrices whose column count differs from D.
TEST(QcqpProblem, FromQcqpValueRequiresExactDimensions) {
  CHECK_EXCEPTION(
      traits<Rot2>::template FromQcqpValue<2>(Matrix::Identity(2, 3)),
      std::invalid_argument);
  CHECK_EXCEPTION(
      traits<Rot3>::template FromQcqpValue<3>(Matrix::Identity(3, 4)),
      std::invalid_argument);
}

}  // namespace QcqpExtractionFixture

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
