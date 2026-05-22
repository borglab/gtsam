/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQcqpProblem.cpp
 * @brief   Unit tests for QcqpProblem and the QCQP trait API.
 *
 * Covers:
 *  - QpCost row-space trace formula (vector and matrix variants);
 *  - QuadraticConstraint equality / ≤ / ≥ semantics;
 *  - QcqpProblem evaluate + ALM on mixed linear / quadratic constraints;
 *  - FrobeniusBetweenFactor<Rot2|Rot3> → QCQP conversion at multiple K;
 *  - The D-templated QcqpValue / QcqpConstraints / QcqpIntrinsicDim traits,
 *    including D-padding, D-independence of constraints, vec form (D=1),
 *    and rejection of nonsensical D (Rot3 at D=2).
 *
 * @author  Frank Dellaert
 * @author  Zhexin Xu
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

// QpCost on a single Vector key reduces to the standard 0.5*x'Qx quadratic
// — i.e. when K=1 the row-space trace formula collapses to the scalar form.
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

// At K=2 the per-key block becomes a 2-column matrix; QpCost must use the
// row-space trace formula 0.5*Σ trace(X_i' Q_ij X_j), not naively dot the
// flattened vector against Q. Cross-check against the explicit double-sum.
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

// The row-space trace formula is K-independent. A K=3 check guards against
// a stride bug that would only show up when the column count differs from
// the K=2 case above (e.g. a Kronecker that wires in K, not the block dim).
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

// QpCost is exactly quadratic in X, so linearize() must return a Hessian
// factor whose error agrees with the original at any perturbation around the
// linearization point — to machine precision. ALM relies on this exactness.
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

// QuadraticConstraint encodes h(X) = 0.5·trace(X'AX) - b. The 0.5 is the
// Lagrangian convention (h's gradient at X is then A·X, not 2·A·X) — easy to
// get wrong. Check that ‖x‖² = 5 must be stored as b = 2.5 to evaluate
// feasible.
TEST(QuadraticConstraint, VectorFeasible) {
  const Matrix A = Matrix::Identity(2, 2);
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 2.5);
  const auto factor = constraint.createEqualityFactor();
  const Values values = VectorValue((Vector(2) << 1.0, 2.0).finished());

  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(values)(0), 1e-12);
}

// Sparse-A case: A picks out one row. ‖row 0‖² = 1 ⇒ b = 0.5. Catches a
// regression where the trace formula sums over all rows instead of the
// row(s) selected by A.
TEST(QuadraticConstraint, Feasible) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 0.5);
  const auto factor = constraint.createEqualityFactor();
  const Values values = MatrixValue(Matrix::Identity(2, 3));

  EXPECT_DOUBLES_EQUAL(0.0, factor->unwhitenedError(values)(0), 1e-12);
}

// The signed violation must equal 0.5·(actual − target). ‖row 0‖² = 2 with
// b = 0.5 ⇒ h = 0.5·(2 − 1) = 0.5. Guards against an off-by-2 from confusing
// h with the unnormalized residual trace(X'AX) − 2b.
TEST(QuadraticConstraint, Infeasible) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint = QuadraticConstraint::Equal(x0, A, 0.5);
  const auto factor = constraint.createEqualityFactor();
  const Values values =
      MatrixValue((Matrix(2, 2) << 1.0, 1.0, 0.0, 1.0).finished());

  EXPECT_DOUBLES_EQUAL(0.5, factor->unwhitenedError(values)(0), 1e-12);
}

// Inequality factor ramps only the *signed* expression: satisfied side reads
// out as the raw value (negative when feasible), but `error()` clips it to 0.
// Catches a sign flip that would penalize the satisfied side.
TEST(QuadraticConstraint, LessEqualViolation) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint =
      QuadraticConstraint::LessEqual(x0, A, 0.5);
  const auto factor = constraint.createInequalityFactor();

  // X = 0: h = -0.5 (satisfied), error ramps to 0.
  EXPECT_DOUBLES_EQUAL(
      -0.5, factor->unwhitenedExpr(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(
      0.0, factor->unwhitenedError(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  // X = (2, 0): h = 0.5*4 - 0.5 = 1.5.
  EXPECT_DOUBLES_EQUAL(1.5,
                       factor->unwhitenedError(MatrixValue(
                           (Matrix(2, 1) << 2.0, 0.0).finished()))(0),
                       1e-12);
}

// `≥` is represented as the negation of the `≤` expression — so the same
// b=0.5 constraint flips which side ramps. Mirrors the test above to catch
// a missing sign on the GreaterEqual factory.
TEST(QuadraticConstraint, GreaterEqualViolation) {
  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 1.0;
  const QuadraticConstraint constraint =
      QuadraticConstraint::GreaterEqual(x0, A, 0.5);
  const auto factor = constraint.createInequalityFactor();

  // X = 0: 0.5*0 - 0.5 = -0.5; with sign = -1 and ramp this gives 0.5.
  EXPECT_DOUBLES_EQUAL(
      0.5, factor->unwhitenedError(MatrixValue(Matrix::Zero(2, 1)))(0), 1e-12);
  // X = (2, 0): 0.5*4 - 0.5 = 1.5 >= 0, satisfies, ramps to 0.
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

// QcqpProblem.evaluate() must thread through the cost graph and *both*
// constraint containers (eq + ineq), reporting separate violation norms.
// Use the trivial unit-vector-on-the-sphere case to keep expected values
// hand-computable.
TEST(QcqpProblem, EvaluateVectorValues) {
  const Matrix Q = Matrix::Identity(2, 2);
  QcqpProblem problem;
  problem.addCost(QpCost(KeyVector{x0},
                         SymmetricBlockMatrix(std::vector<DenseIndex>{2}, Q)));
  // Constraint ||x||^2 = 1, encoded as 0.5*x'Ix = 0.5.
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 0.5));

  Values values;
  values.insert(x0, (Vector(2) << 1.0, 0.0).finished());

  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(values);
  EXPECT_DOUBLES_EQUAL(0.5, cost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, eqViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, ineqViolation, 1e-12);
}

// Same as above, but with a manually assembled matrix-form cost and a 2-key
// trace-1 constraint on x0. Verifies the matrix-form path against the
// reference DirectTraceCost helper.
TEST(QcqpProblem, Evaluate) {
  Matrix Q = Matrix::Zero(4, 4);
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0;
  Q.block<2, 2>(0, 2) << 0.2, -0.1, 0.3, 0.5;
  Q.block<2, 2>(2, 0) = Q.block<2, 2>(0, 2).transpose();

  const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{2, 2}, Q);
  NonlinearFactorGraph costs;
  costs.emplace_shared<QpCost>(KeyVector{x0, x1}, blockQ, 2);

  NonlinearEqualityConstraints constraints;
  // Constraint trace(X0' I X0) = 2, encoded as 0.5*trace = 1.
  constraints.push_back(
      QuadraticConstraint::Equal(x0, Matrix::Identity(2, 2), 1.0)
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

// End-to-end ALM smoke test on a small QCQP that mixes both constraint
// kinds (linear eq / linear ineq / quadratic eq / quadratic ineq). The
// minimizer is (1, 0). Confirms ALM converges to feasibility on *all four*
// containers — a regression here usually means a constraint container was
// missed in the AL gradient assembly.
TEST(QcqpProblem, OptimizeAugmentedLagrangianMixedConstraints) {
  QcqpProblem problem;

  const Matrix Q = Matrix::Identity(2, 2);
  problem.addCost(QpCost(KeyVector{x0},
                         SymmetricBlockMatrix(std::vector<DenseIndex>{2}, Q)));

  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x0, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x0, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.9))));
  // Constraint ||x||^2 = 1, encoded as 0.5*trace = 0.5.
  problem.addConstraint(QuadraticConstraint::Equal(x0, Q, 0.5));

  Matrix yBound = Matrix::Zero(2, 2);
  yBound(1, 1) = 1.0;
  // Constraint y^2 <= 0.01, encoded as 0.5*y^2 <= 0.005.
  problem.addConstraint(QuadraticConstraint::LessEqual(x0, yBound, 0.005));

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
  InsertQcqpValue<Rot2, 2>(x0, Rot2::fromAngle(0.3), &values);
  InsertQcqpValue<Rot2, 2>(x1, Rot2::fromAngle(-0.2), &values);
  return values;
}

bool ThrowsMissingQcqpTraits(const NonlinearFactorGraph& graph) {
  try {
    QcqpProblem problem(graph, /*K=*/2);
  } catch (const std::runtime_error& exception) {
    return std::string(exception.what()).find("QCQP variable traits") !=
           std::string::npos;
  }
  return false;
}

// A factor that does not implement `qcqpFactors` (here: stock
// BetweenFactor<Rot2>) must be rejected at construction — silently dropping
// it would yield a QCQP missing measurement terms.
TEST(QcqpProblem, UnsupportedFactorThrows) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot2>>(x0, x1, Rot2::fromAngle(0.1));

  CHECK_EXCEPTION({ QcqpProblem problem(graph, /*K=*/2); }, std::runtime_error);
}

// FrobeniusBetweenFactor<SO3> defines qcqpFactors() but SO3 has no QCQP
// trait (we only specialize on Rot2 / Rot3). The conversion must surface a
// trait-specific error, not a generic linearization failure.
TEST(QcqpProblem, MissingQcqpTraitsThrows) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<SO3>>(
      x0, x1, SO3::Expmap((Vector3() << 0.1, 0.2, 0.3).finished()));

  EXPECT(ThrowsMissingQcqpTraits(graph));
}

// QCQP-conversion invariant: at K = d (the manifold's intrinsic dim), the
// matrix-form cost must equal the manifold-form Frobenius cost. The 2× on
// `graph.error(values)` recovers the paper convention from GTSAM's ½-NLL.
TEST(QcqpProblem, SingleFrobeniusBetweenFactor) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1,
                                                     Rot2::fromAngle(0.4));

  const Values values = SingleFactorManifoldValues();
  const QcqpProblem problem(graph, /*K=*/2);
  const Values qcqpValues = SingleFactorQcqpValues();

  // 2 * graph.error: paper convention (no 0.5) vs. graph's 0.5*||r||^2.
  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(values),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
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
    InsertQcqpValue<Rot2, 2>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

// Multi-key, multi-factor invariant: on a Rot2 ring at K=2, the assembled
// QCQP cost equals the manifold cost and the row-orthonormality constraints
// are already feasible at the manifold-form values. Tests the multi-factor
// assembly path that the single-factor test doesn't exercise.
TEST(QcqpProblem, Rot2RingPgo) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values values = RingValues(N, delta, 0.01);
  const QcqpProblem problem(graph, /*K=*/2);
  const Values qcqpValues = RingQcqpValues(N, delta, 0.01);

  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(values),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Cheap ALM smoke on the Rot2 ring: just 5 iterations should already drive
// the constraint norm down without growing the cost. A regression here
// usually means the ALM gradient is missing the QcqpProblem's constraint
// terms entirely.
TEST(QcqpProblem, AugmentedLagrangianOptimizerRot2Ring) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem problem(graph, /*K=*/2);
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
  // Paper convention: ||residual||^2 (no 0.5).
  const Matrix X0 = traits<Rot2>::template QcqpValue<D>(R0);
  const Matrix X1 = traits<Rot2>::template QcqpValue<D>(R1);
  const Matrix residual = X1 - measured.matrix().transpose() * X0;
  return residual.squaredNorm();
}

// QcqpValue<Rot2,2>(R) returns R^T (row-orthonormal). At this natural rank,
// the trait's constraints reduce to X X^T = I and must report zero
// violation. This is the contract every higher-K case extends.
TEST(QcqpProblem, Rot2D2QcqpValueConstraints) {
  const Rot2 R = Rot2::fromAngle(0.3);
  const Matrix X = traits<Rot2>::template QcqpValue<2>(R);
  LONGS_EQUAL(2, X.rows());
  LONGS_EQUAL(2, X.cols());
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X * X.transpose(), 1e-12));

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot2, 2>(x0, &constraints);
  Values values;
  values.insert(x0, X);

  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Staircase climb invariant: the set of small 2×2 row-ortho constraints
// returned by QcqpConstraints<Rot2,K> must NOT depend on K. The K
// dependence belongs in QpCost's Kronecker, not in the constraint set —
// otherwise the constraint count would grow with the rank.
TEST(QcqpProblem, Rot2D3QcqpValueConstraints) {
  const Rot2 R = Rot2::fromAngle(-0.4);
  const Matrix X = traits<Rot2>::template QcqpValue<3>(R);
  LONGS_EQUAL(2, X.rows());
  LONGS_EQUAL(3, X.cols());
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X * X.transpose(), 1e-12));

  const auto cs2 = traits<Rot2>::template QcqpConstraints<2>();
  const auto cs3 = traits<Rot2>::template QcqpConstraints<3>();
  LONGS_EQUAL(cs2.size(), cs3.size());
  for (size_t i = 0; i < cs2.size(); ++i) {
    EXPECT(assert_equal(cs2[i].first, cs3[i].first, 1e-12));
    EXPECT_DOUBLES_EQUAL(cs2[i].second, cs3[i].second, 1e-12);
  }

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot2, 3>(x0, &constraints);
  Values values;
  values.insert(x0, X);

  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Same conversion invariant as SingleFrobeniusBetweenFactor above, but
// asserted against named R0/R1 and a non-trivial measurement, so the test
// also doubles as a worked example of how to lift a Rot2 manifold value
// into the QCQP-form matrix variable.
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD2) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1, measured);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  const QcqpProblem problem(graph, /*K=*/2);
  const Values qcqpValues = QcqpValues<2>(R0, R1);

  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(manifoldValues),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// K = d + 1 (= 3 here): the staircase has lifted by one. The QCQP cost
// must still equal the manifold cost because the lift only pads with zeros.
// Cross-check both against the GTSAM manifold form and against the direct
// row-space residual formula (catches a wrong-axis Kronecker).
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD3) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(x0, x1, measured);

  const QcqpProblem problem(graph, /*K=*/3);
  const Values qcqpValues = QcqpValues<3>(R0, R1);
  const double qcqpCost = problem.costs().error(qcqpValues);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  EXPECT(std::isfinite(qcqpCost));
  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(manifoldValues), qcqpCost, 1e-12);
  EXPECT_DOUBLES_EQUAL(DirectQcqpBetweenCost<3>(R0, R1, measured), qcqpCost,
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

}  // namespace QcqpRot2VariableFixture
/* ************************************************************************* */
namespace QcqpRot3VariableFixture {

const Key x0 = Symbol('x', 0);
const Key x1 = Symbol('x', 1);

template <int D>
Values Rot3QcqpValues(const Rot3& R0, const Rot3& R1) {
  Values values;
  InsertQcqpValue<Rot3, D>(x0, R0, &values);
  InsertQcqpValue<Rot3, D>(x1, R1, &values);
  return values;
}

template <int D>
double DirectRot3QcqpBetweenCost(const Rot3& R0, const Rot3& R1,
                                 const Rot3& measured) {
  // Paper convention: ||residual||^2 (see DirectQcqpBetweenCost above).
  const Matrix X0 = traits<Rot3>::template QcqpValue<D>(R0);
  const Matrix X1 = traits<Rot3>::template QcqpValue<D>(R1);
  const Matrix residual = X1 - measured.matrix().transpose() * X0;
  return residual.squaredNorm();
}

// Rot3 trait contract at the natural rank d=3: QcqpValue is R^T, the trait
// emits exactly 6 row-orthonormality constraints (3 unit norms + 3
// orthogonalities), and the matrix form starts feasible.
TEST(QcqpProblem, Rot3D3QcqpValueConstraints) {
  const Rot3 R = Rot3::Expmap((Vector3() << 0.4, -0.1, 0.7).finished());
  const Matrix X = traits<Rot3>::template QcqpValue<3>(R);
  LONGS_EQUAL(3, X.rows());
  LONGS_EQUAL(3, X.cols());
  // X = R', so X X' = R'R = I.
  EXPECT(assert_equal(Matrix(Matrix3::Identity()), X * X.transpose(), 1e-12));

  const auto cs = traits<Rot3>::template QcqpConstraints<3>();
  LONGS_EQUAL(6, cs.size());

  NonlinearEqualityConstraints constraints;
  InsertQcqpConstraints<Rot3, 3>(x0, &constraints);
  Values values;
  values.insert(x0, X);
  EXPECT_DOUBLES_EQUAL(0.0, constraints.violationNorm(values), 1e-12);
}

// Rot3 conversion at the natural rank. Same contract as the Rot2 D=2 case
// — the matrix-form Frobenius cost must equal the manifold Frobenius cost
// (× 2 for paper convention). Cross-check against the direct row-space
// residual to catch tangent-axis-vs-row-axis swaps.
TEST(QcqpProblem, Rot3FrobeniusBetweenFactorD3) {
  const Rot3 measured = Rot3::Expmap((Vector3() << 0.2, 0.1, -0.3).finished());
  const Rot3 R0 = Rot3::Expmap((Vector3() << 0.05, 0.10, 0.15).finished());
  const Rot3 R1 = Rot3::Expmap((Vector3() << 0.10, 0.20, 0.30).finished());

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(x0, x1, measured);

  Values manifoldValues;
  manifoldValues.insert(x0, R0);
  manifoldValues.insert(x1, R1);

  const QcqpProblem problem(graph, /*K=*/3);
  const Values qcqpValues = Rot3QcqpValues<3>(R0, R1);
  const double qcqpCost = problem.costs().error(qcqpValues);

  EXPECT(std::isfinite(qcqpCost));
  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(manifoldValues), qcqpCost, 1e-12);
  EXPECT_DOUBLES_EQUAL(DirectRot3QcqpBetweenCost<3>(R0, R1, measured), qcqpCost,
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// pMin = d for the staircase to even start. For Rot3 that means K ≥ 3 —
// at K = 2 the matrix form would be 3-by-2, which can't host R^T (3×3)
// and can't satisfy the row-orthonormality X X^T = I_3 (rank 2 < 3). The
// trait must surface this at QCQP construction, not later inside ALM
// where the diagnosis would be obscure.
TEST(QcqpProblem, Rot3K2Rejected) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
      x0, x1, Rot3::Expmap(Vector3::Zero()));
  CHECK_EXCEPTION({ QcqpProblem problem(graph, /*K=*/2); },
                  std::invalid_argument);
}

}  // namespace QcqpRot3VariableFixture
/* ************************************************************************* */
namespace QcqpDApiFixture {

// The QCQP trait API has two distinct branches:
//   - Vec form (D = 1): vectorize R into a 4×1 (Rot2) or 9×1 (Rot3) variable
//     with constraints encoded in 4×4 / 9×9 matrices. Older form, kept for
//     non-Frobenius polynomial relaxations.
//   - Matrix form (D ≥ d): d×D row-orthonormal representation. The
//     staircase walks this branch — QcqpProblem dispatches the runtime
//     column count K onto a compile-time D via the switch in
//     FrobeniusBetweenFactor::qcqpFactors.
// The tests below pin both branches' contracts.

const Key x0 = Symbol('x', 0);

// Sanity: the trait's compile-time intrinsic dim must match SO(d)'s ambient
// dimension. The staircase reads this to size pMin and to pick the rounding
// rank d. A wrong value here silently breaks both.
TEST(QcqpProblem, IntrinsicDimConstants) {
  LONGS_EQUAL(2, traits<Rot2>::QcqpIntrinsicDim);
  LONGS_EQUAL(3, traits<Rot3>::QcqpIntrinsicDim);
}

// Climbing the staircase lifts X by appending a zero column. Verify across
// D = 2, 3, 5 that (a) the left d cols equal R^T, (b) the remaining cols are
// exactly zero, and (c) X X^T = I_d at every D. This is the *only* lift
// behavior the staircase's escapeSaddleAndLift relies on.
TEST(QcqpProblem, Rot2QcqpValueDPaddingD2D3D5) {
  const Rot2 R = Rot2::fromAngle(0.7);
  const Matrix2 Rt = R.matrix().transpose();

  const Matrix X2 = traits<Rot2>::template QcqpValue<2>(R);
  const Matrix X3 = traits<Rot2>::template QcqpValue<3>(R);
  const Matrix X5 = traits<Rot2>::template QcqpValue<5>(R);

  LONGS_EQUAL(2, X2.rows());
  LONGS_EQUAL(2, X2.cols());
  EXPECT(assert_equal(Matrix(Rt), X2, 1e-12));

  LONGS_EQUAL(2, X3.rows());
  LONGS_EQUAL(3, X3.cols());
  EXPECT(assert_equal(Matrix(Rt), Matrix(X3.leftCols<2>()), 1e-12));
  EXPECT(assert_equal(Matrix(Vector2::Zero()), Matrix(X3.col(2)), 1e-12));

  LONGS_EQUAL(2, X5.rows());
  LONGS_EQUAL(5, X5.cols());
  EXPECT(assert_equal(Matrix(Rt), Matrix(X5.leftCols<2>()), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix::Zero(2, 3)), Matrix(X5.rightCols<3>()),
                      1e-12));

  // Row-orthonormality at every D >= 2.
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X2 * X2.transpose(), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X3 * X3.transpose(), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix2::Identity()), X5 * X5.transpose(), 1e-12));
}

TEST(QcqpProblem, Rot3QcqpValueDPaddingD3D4D6) {
  const Rot3 R = Rot3::Expmap((Vector3() << 0.2, -0.1, 0.5).finished());
  const Matrix3 Rt = R.matrix().transpose();

  const Matrix X3 = traits<Rot3>::template QcqpValue<3>(R);
  const Matrix X4 = traits<Rot3>::template QcqpValue<4>(R);
  const Matrix X6 = traits<Rot3>::template QcqpValue<6>(R);

  LONGS_EQUAL(3, X3.rows());
  LONGS_EQUAL(3, X3.cols());
  EXPECT(assert_equal(Matrix(Rt), X3, 1e-12));

  LONGS_EQUAL(3, X4.rows());
  LONGS_EQUAL(4, X4.cols());
  EXPECT(assert_equal(Matrix(Rt), Matrix(X4.leftCols<3>()), 1e-12));
  EXPECT(assert_equal(Matrix(Vector3::Zero()), Matrix(X4.col(3)), 1e-12));

  LONGS_EQUAL(3, X6.rows());
  LONGS_EQUAL(6, X6.cols());
  EXPECT(assert_equal(Matrix(Rt), Matrix(X6.leftCols<3>()), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix::Zero(3, 3)), Matrix(X6.rightCols<3>()),
                      1e-12));

  EXPECT(assert_equal(Matrix(Matrix3::Identity()), X3 * X3.transpose(), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix3::Identity()), X4 * X4.transpose(), 1e-12));
  EXPECT(assert_equal(Matrix(Matrix3::Identity()), X6 * X6.transpose(), 1e-12));
}

// The constraint set in QcqpConstraints<RotT, D> describes X X^T = I_d and
// is therefore D-independent in *structure* — only QpCost's Kronecker pulls
// in D. Verify the d×d A blocks and their `b` targets are byte-equal across
// D for Rot2 (and for Rot3 in the next test).
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

TEST(QcqpProblem, Rot3QcqpConstraintsAreDIndependent) {
  const auto cs3 = traits<Rot3>::template QcqpConstraints<3>();
  const auto cs4 = traits<Rot3>::template QcqpConstraints<4>();
  const auto cs6 = traits<Rot3>::template QcqpConstraints<6>();
  LONGS_EQUAL(6, cs3.size());
  LONGS_EQUAL(cs3.size(), cs4.size());
  LONGS_EQUAL(cs3.size(), cs6.size());
  for (size_t i = 0; i < cs3.size(); ++i) {
    EXPECT(assert_equal(cs3[i].first, cs4[i].first, 1e-12));
    EXPECT(assert_equal(cs3[i].first, cs6[i].first, 1e-12));
  }
}

// Vec form (D = 1) is a separate code path from the matrix form. Verify it
// still produces a 4-vector for Rot2 with the documented constraint count
// (2 col-unit-norm + 1 col-ortho + 1 det = +1) and that the column-major
// reshape round-trips back to R. Catches regressions where matrix-form
// changes accidentally rewire the vec path.
TEST(QcqpProblem, Rot2VecFormD1) {
  const Rot2 R = Rot2::fromAngle(0.3);
  const Matrix X = traits<Rot2>::template QcqpValue<1>(R);
  LONGS_EQUAL(4, X.rows());
  LONGS_EQUAL(1, X.cols());
  // Reshape back to 2x2 (column-major) and verify equality with R.
  const Matrix Rback = Eigen::Map<const Matrix2>(X.data());
  EXPECT(assert_equal(Matrix(R.matrix()), Rback, 1e-12));

  const auto cs = traits<Rot2>::template QcqpConstraints<1>();
  LONGS_EQUAL(4, cs.size());  // 2 col-unit-norm + 1 col-ortho + 1 det=+1
  for (const auto& [A, b] : cs) {
    LONGS_EQUAL(4, A.rows());
    LONGS_EQUAL(4, A.cols());
    const double xAx = (X.transpose() * A * X)(0, 0);
    EXPECT_DOUBLES_EQUAL(b, xAx, 1e-12);
  }
}

TEST(QcqpProblem, Rot3VecFormD1) {
  const Rot3 R = Rot3::Expmap((Vector3() << 0.1, 0.2, 0.3).finished());
  const Matrix X = traits<Rot3>::template QcqpValue<1>(R);
  LONGS_EQUAL(9, X.rows());
  LONGS_EQUAL(1, X.cols());
  const Matrix Rback = Eigen::Map<const Matrix3>(X.data());
  EXPECT(assert_equal(Matrix(R.matrix()), Rback, 1e-12));

  const auto cs = traits<Rot3>::template QcqpConstraints<1>();
  LONGS_EQUAL(6, cs.size());  // 3 col-unit-norm + 3 col-ortho (det=1 is cubic)
  for (const auto& [A, b] : cs) {
    LONGS_EQUAL(9, A.rows());
    LONGS_EQUAL(9, A.cols());
    const double xAx = (X.transpose() * A * X)(0, 0);
    EXPECT_DOUBLES_EQUAL(b, xAx, 1e-12);
  }
}

// Direct trait-level rejection of D = 2 for Rot3. Complements Rot3K2Rejected
// (which tests the same thing one level up at QcqpProblem construction); if
// only the trait test fails, the higher-level guard is masking the bug.
TEST(QcqpProblem, Rot3D2Throws) {
  const Rot3 R = Rot3::Expmap(Vector3::Zero());
  CHECK_EXCEPTION(traits<Rot3>::template QcqpValue<2>(R),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Rot3>::template QcqpConstraints<2>(),
                  std::invalid_argument);
}

// Runtime K → compile-time D dispatch lives inside qcqpFactors's switch.
// We've already tested K = d and K = d + 1; this one pins K = d + 2 (= 4)
// to make sure the switch covers the wider rungs of the ladder for both
// rotation types. A typo in one switch case would show up here.
TEST(QcqpProblem, Rot2FrobeniusBetweenFactorD4) {
  const Rot2 measured = Rot2::fromAngle(0.4);
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.2);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(Symbol('x', 0),
                                                     Symbol('x', 1), measured);

  Values manifoldValues;
  manifoldValues.insert(Symbol('x', 0), R0);
  manifoldValues.insert(Symbol('x', 1), R1);

  Values qcqpValues;
  InsertQcqpValue<Rot2, 4>(Symbol('x', 0), R0, &qcqpValues);
  InsertQcqpValue<Rot2, 4>(Symbol('x', 1), R1, &qcqpValues);

  const QcqpProblem problem(graph, /*K=*/4);
  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(manifoldValues),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

TEST(QcqpProblem, Rot3FrobeniusBetweenFactorD4) {
  const Rot3 measured = Rot3::Expmap((Vector3() << 0.2, 0.1, -0.3).finished());
  const Rot3 R0 = Rot3::Expmap((Vector3() << 0.05, 0.10, 0.15).finished());
  const Rot3 R1 = Rot3::Expmap((Vector3() << 0.10, 0.20, 0.30).finished());

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(Symbol('x', 0),
                                                     Symbol('x', 1), measured);

  Values manifoldValues;
  manifoldValues.insert(Symbol('x', 0), R0);
  manifoldValues.insert(Symbol('x', 1), R1);

  Values qcqpValues;
  InsertQcqpValue<Rot3, 4>(Symbol('x', 0), R0, &qcqpValues);
  InsertQcqpValue<Rot3, 4>(Symbol('x', 1), R1, &qcqpValues);

  const QcqpProblem problem(graph, /*K=*/4);
  EXPECT_DOUBLES_EQUAL(2.0 * graph.error(manifoldValues),
                       problem.costs().error(qcqpValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

}  // namespace QcqpDApiFixture

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
