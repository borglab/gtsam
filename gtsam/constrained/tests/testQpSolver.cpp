/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQpSolver.cpp
 * @brief   Unit tests for constrained-module QP solver.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/QpsParser.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <cmath>
#include <stdexcept>
#include <string>

using namespace gtsam;

/* ************************************************************************* */
namespace dense_native_curvature {

// Exact diagonal packing preserves weak curvature and rejects negative costs.
TEST(ActiveSetSolver, DenseDiagonalNativeCurvature) {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  params->regularization = 0;
  const Matrix curvature{{1e-20, 0}, {0, 2}};
  const Vector expected{{4, -1}};
  QpProblem problem;
  problem.addCost(HessianFactor(0, curvature, Vector(curvature * expected), 0));
  const auto result = ActiveSetSolver(problem, params).optimizeWithState();
  CHECK(result.second.converged);
  EXPECT(assert_equal(expected, result.first.at<Vector>(0), 1e-10));
  QpProblem indefinite;
  indefinite.addCost(HessianFactor(0, Matrix{{1, 0}, {0, -.25}}, Vector::Zero(2), 0));
  CHECK_EXCEPTION((void)ActiveSetSolver(indefinite, params), std::runtime_error);
}

// Small off-diagonal curvature must use the full square root, even in mixed costs.
TEST(ActiveSetSolver, DenseRetainsOffDiagonalAndJacobianCurvature) {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  params->regularization = 0;
  const Vector expected{{2, -1}};
  for (double coupling : {0.0, 1e-8}) {
    const Matrix curvature{{1, coupling}, {coupling, 2}};
    QpProblem problem;
    problem.addCost(HessianFactor(0, curvature, Vector(curvature * expected), 0));
    problem.addCost(JacobianFactor(0, Matrix::Identity(2, 2), expected));
    const auto result = ActiveSetSolver(problem, params).optimizeWithState();
    CHECK(result.second.converged);
    EXPECT(assert_equal(expected, result.first.at<Vector>(0), 1e-10));
  }
}

}  // namespace dense_native_curvature
/* ************************************************************************* */

/* ************************************************************************* */
namespace scalar_qp_examples {

const Key x1 = Symbol('x', 1);
const Key x2 = Symbol('x', 2);

Matrix Matrix1(double value) { return Matrix{{value}}; }

Vector Vector1D(double value) { return Vector{{value}}; }

Values Values2(double first, double second) {
  Values values;
  values.insert(x1, Vector1D(first));
  values.insert(x2, Vector1D(second));
  return values;
}

ActiveSetSolverParams::shared_ptr DenseQpParams() {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  return params;
}

bool Values2Equal(const Values& values, double first, double second,
                  double tolerance = 1e-7) {
  const double actualFirst = values.at<Vector>(x1)(0);
  const double actualSecond = values.at<Vector>(x2)(0);
  return std::abs(actualFirst - first) <= tolerance &&
         std::abs(actualSecond - second) <= tolerance;
}

QpProblem CreateForstExample() {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, x2, 2.0 * Matrix1(1.0), -Matrix1(1.0),
                                3.0 * Vector1D(1.0), 2.0 * Matrix1(1.0),
                                Vector1D(0.0), 10.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, Matrix1(1.0), x2, Matrix1(1.0), Vector1D(2.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, -Matrix1(1.0), Vector1D(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x2, -Matrix1(1.0), Vector1D(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, Matrix1(1.0), Vector1D(1.5))));
  return problem;
}

QpProblem CreateEqualityConstrainedExample() {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, x2, 2.0 * Matrix1(1.0), Matrix1(0.0),
                                Vector1D(0.0), 2.0 * Matrix1(1.0),
                                Vector1D(0.0), 0.0));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x1, Matrix1(1.0), x2, Matrix1(1.0), Vector1D(-1.0))));
  return problem;
}

QpProblem CreateMatlabExample() {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, x2, Matrix1(1.0), -Matrix1(1.0),
                                2.0 * Vector1D(1.0), 2.0 * Matrix1(1.0),
                                6.0 * Vector1D(1.0), 1000.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, Matrix1(1.0), x2, Matrix1(1.0), Vector1D(2.0))));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x1, -Matrix1(1.0), x2, 2.0 * Matrix1(1.0), Vector1D(2.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, 2.0 * Matrix1(1.0), x2, Matrix1(1.0), Vector1D(3.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, -Matrix1(1.0), Vector1D(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x2, -Matrix1(1.0), Vector1D(0.0))));
  return problem;
}

QpProblem CreateNocedalExample() {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(1.0), 1.0));
  problem.addCost(HessianFactor(x2, Matrix1(1.0), 2.5 * Vector1D(1.0), 6.25));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x1, -Matrix1(1.0), x2, 2.0 * Matrix1(1.0), Vector1D(2.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, Matrix1(1.0), x2, 2.0 * Matrix1(1.0), Vector1D(6.0))));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x1, Matrix1(1.0), x2, -2.0 * Matrix1(1.0), Vector1D(2.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x1, -Matrix1(1.0), Vector1D(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x2, -Matrix1(1.0), Vector1D(0.0))));
  return problem;
}

// Verifies the active-set solve on the textbook Forst example.
TEST(ActiveSetSolver, DenseForstExample) {
  const Values result =
      CreateForstExample().optimize(Values2(0.0, 0.0), QpSolverType::Dense);

  CHECK(Values2Equal(result, 1.5, 0.5));
}

// Verifies equality constraints are included in every KKT solve.
TEST(ActiveSetSolver, DenseEqualityConstrained) {
  ActiveSetSolver solver(CreateEqualityConstrainedExample(), DenseQpParams());
  const auto [result, state] = solver.optimizeWithState(Values2(1.0, 1.0));

  CHECK(Values2Equal(result, -0.5, -0.5));
  CHECK_EQUAL(1, state.equalityMultipliers.size());
  CHECK_EQUAL(1, state.equalityMultipliers.front().size());
}

// A positive proximal weight must not change the original QP optimum.
TEST(ActiveSetSolver, DenseRegularizationIsRecentered) {
  auto params = DenseQpParams();
  params->regularization = 1.0;
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(2.0), 0.0));

  Values initial;
  initial.insert(x1, Vector1D(0.0));
  const Values result = ActiveSetSolver(problem, params).optimize(initial);

  EXPECT_DOUBLES_EQUAL(2.0, result.at<Vector>(x1)(0), 1e-7);
}

// Redundant equalities remain enforced without requiring unique multipliers.
TEST(ActiveSetSolver, DenseDependentEqualityConstraints) {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(0.0), 0.0));
  const JacobianFactor equality(x1, Matrix1(1.0), Vector1D(1.0));
  problem.addConstraint(LinearConstraint::Equal(equality));
  problem.addConstraint(LinearConstraint::Equal(equality));

  Values initial;
  initial.insert(x1, Vector1D(1.0));
  const auto result =
      ActiveSetSolver(problem, DenseQpParams()).optimizeWithState(initial);
  CHECK(result.second.converged);
  EXPECT_DOUBLES_EQUAL(1, result.first.at<Vector>(x1)(0), 1e-10);
}

// Verifies the Matlab QP example from the old solver tests.
TEST(ActiveSetSolver, DenseMatlabExample) {
  const Values result =
      CreateMatlabExample().optimize(Values2(0.0, 0.0), QpSolverType::Dense);

  CHECK(Values2Equal(result, 2.0 / 3.0, 4.0 / 3.0));
}

// Verifies no-initial solve uses vector-valued phase-I initialization.
TEST(ActiveSetSolver, DenseMatlabExampleNoInitialValues) {
  const Values result = CreateMatlabExample().optimize(QpSolverType::Dense);

  CHECK(Values2Equal(result, 2.0 / 3.0, 4.0 / 3.0));
}

// Verifies the Nocedal and Wright example from the old solver tests.
TEST(ActiveSetSolver, DenseNocedalExample) {
  const Values result =
      CreateNocedalExample().optimize(Values2(2.0, 0.0), QpSolverType::Dense);

  CHECK(Values2Equal(result, 1.4, 1.7));
}

// Verifies active constraints can block a failed full KKT step.
TEST(ActiveSetSolver, DenseFailedSubproblem) {
  const Key x = Symbol('x', 0);
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector::Zero(2), 0.0));
  problem.addCost(HessianFactor(x, Matrix::Zero(2, 2), Vector::Zero(2), 100.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix{{-1.0, 0.0}}, Vector1D(-1.0))));

  Values initial;
  initial.insert(x, Vector{{10.0, 100.0}});
  const Values result = problem.optimize(initial, QpSolverType::Dense);

  EXPECT(assert_equal(Vector{{1.0, 0.0}}, result.at<Vector>(x), 1e-7));
}

// Verifies infeasible caller-provided initial values are rejected.
TEST(ActiveSetSolver, DenseInfeasibleInitialValues) {
  const Key x = Symbol('x', 0);
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector::Zero(2), 0.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix{{-1.0, 0.0}}, Vector1D(-1.0))));

  Values initial;
  initial.insert(x, Vector{{-10.0, 100.0}});

  CHECK_EXCEPTION(ActiveSetSolver(problem, DenseQpParams()).optimize(initial),
                  std::invalid_argument);
}

// Verifies a previous dense QP state can warm-start the active set.
TEST(ActiveSetSolver, DenseWarmStart) {
  ActiveSetSolver solver(CreateMatlabExample(), DenseQpParams());
  const auto [firstValues, firstState] =
      solver.optimizeWithState(Values2(0.0, 0.0));
  const auto [secondValues, secondState] =
      solver.optimizeWithState(firstState.values, firstState);

  CHECK(Values2Equal(firstValues, 2.0 / 3.0, 4.0 / 3.0));
  CHECK(Values2Equal(secondValues, 2.0 / 3.0, 4.0 / 3.0));
  CHECK(secondState.iterations <= firstState.iterations);
}

// Verifies QpProblem::optimize defaults to sparse active-set QP solving.
TEST(QpProblem, DefaultOptimizeUsesSparseSolver) {
  const Values result = CreateMatlabExample().optimize(Values2(0.0, 0.0));

  CHECK(Values2Equal(result, 2.0 / 3.0, 4.0 / 3.0));
}

// Verifies QpProblem can select the dense QP solver.
TEST(QpProblem, DenseOptimizeSelection) {
  const Values result =
      CreateMatlabExample().optimize(Values2(0.0, 0.0), QpSolverType::Dense);

  CHECK(Values2Equal(result, 2.0 / 3.0, 4.0 / 3.0));
}

// Verifies sparse no-initial QP solve uses phase-I initialization.
TEST(QpProblem, SparseOptimizeNoInitialValues) {
  const Values result = CreateMatlabExample().optimize(QpSolverType::Sparse);

  CHECK(Values2Equal(result, 2.0 / 3.0, 4.0 / 3.0));
}

// Verifies no-initial sparse QP solve handles unconstrained variables.
TEST(QpProblem, SparseUnconstrainedNoInitialValues) {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(2.0), 0.0));

  const Values result = problem.optimize(QpSolverType::Sparse);

  EXPECT_DOUBLES_EQUAL(2.0, result.at<Vector>(x1)(0), 1e-7);
}

}  // namespace scalar_qp_examples
/* ************************************************************************* */
namespace constrained_qp_features {

const Key x = Symbol('x', 0);

Vector Vector1D(double value) { return Vector{{value}}; }

Matrix Matrix1D(double value) { return Matrix{{value}}; }

ActiveSetSolverParams::shared_ptr DenseQpParams() {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  return params;
}

QpProblem CreateSparseChainQp(size_t variables) {
  QpProblem problem;
  const Vector target = Vector1D(1.0);
  const Vector zero = Vector1D(0.0);
  for (size_t i = 0; i < variables; ++i) {
    const Key key = Symbol('c', i);
    problem.addCost(HessianFactor(key, Matrix1D(1.0), target, 0.0));
    if (i % 4 == 0) {
      problem.addConstraint(LinearConstraint::LessEqual(
          JacobianFactor(key, Matrix1D(1.0), zero)));
    }
  }
  for (size_t i = 0; i + 1 < variables; ++i) {
    const Key key1 = Symbol('c', i);
    const Key key2 = Symbol('c', i + 1);
    problem.addCost(
        JacobianFactor(key1, Matrix1D(0.5), key2, Matrix1D(-0.5), zero));
  }
  return problem;
}

Values SparseChainInitialValues(size_t variables) {
  Values values;
  for (size_t i = 0; i < variables; ++i) {
    values.insert(Symbol('c', i), Vector1D(0.0));
  }
  return values;
}

// Verifies the solver preserves Matrix value shape from the initial point.
TEST(ActiveSetSolver, DenseMatrixValues) {
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector{{2.0, 2.0}}, 8.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix{{1.0, 1.0}}, Vector1D(2.0))));

  Values initial;
  initial.insert(x, Matrix{{0.0}, {0.0}});
  const Values result = problem.optimize(initial, QpSolverType::Dense);

  const Matrix actual = result.at<Matrix>(x);
  EXPECT_LONGS_EQUAL(2, actual.rows());
  EXPECT_LONGS_EQUAL(1, actual.cols());
  EXPECT(assert_equal(Matrix{{1.0}, {1.0}}, actual, 1e-7));
}

// Verifies GreaterEqual constraints are solved through the constrained API.
TEST(ActiveSetSolver, DenseGreaterEqualConstraint) {
  QpProblem problem;
  problem.addCost(HessianFactor(x, Matrix::Identity(1, 1), Vector1D(2.0), 4.0));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, Matrix::Identity(1, 1), Vector1D(3.0))));

  Values initial;
  initial.insert(x, Vector1D(3.0));
  const Values result = problem.optimize(initial, QpSolverType::Dense);

  EXPECT_DOUBLES_EQUAL(3.0, result.at<Vector>(x)(0), 1e-7);
}

// Verifies multi-row inequality constraints expose row-ordered multipliers.
TEST(ActiveSetSolver, DenseMultiRowInequalityConstraint) {
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector{{2.0, 3.0}}, 13.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix::Identity(2, 2), Vector{{1.0, 1.0}})));

  Values initial;
  initial.insert(x, Vector{{0.0, 0.0}});
  const auto [result, state] =
      ActiveSetSolver(problem, DenseQpParams()).optimizeWithState(initial);

  EXPECT(assert_equal(Vector{{1.0, 1.0}}, result.at<Vector>(x), 1e-7));
  CHECK_EQUAL(1, state.inequalityMultipliers.size());
  CHECK_EQUAL(2, state.inequalityMultipliers.front().size());
  CHECK_EQUAL(2, state.activeInequalityRows.size());
  CHECK(state.activeInequalityRows[0]);
  CHECK(state.activeInequalityRows[1]);
  EXPECT(assert_equal(Vector{{-1.0, -2.0}}, state.inequalityMultipliers.front(),
                      1e-7));
}

// Verifies active-set QP state exposes multipliers by constraint row order.
TEST(ActiveSetSolver, QpMultipliersAreRowOrdered) {
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector{{2.0, 3.0}}, 13.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix::Identity(2, 2), Vector{{1.0, 1.0}})));

  Values initial;
  initial.insert(x, Vector{{0.0, 0.0}});
  const auto [result, state] =
      ActiveSetSolver(problem).optimizeWithState(initial);

  EXPECT(assert_equal(Vector{{1.0, 1.0}}, result.at<Vector>(x), 1e-7));
  CHECK_EQUAL(1, state.inequalityMultipliers.size());
  CHECK_EQUAL(2, state.inequalityMultipliers.front().size());
  CHECK_EQUAL(2, state.activeInequalityRows.size());
  CHECK(state.activeInequalityRows[0]);
  CHECK(state.activeInequalityRows[1]);
  EXPECT(assert_equal(Vector{{-1.0, -2.0}}, state.inequalityMultipliers.front(),
                      1e-7));
}

// Verifies dense QP mode matches the QpProblem dense convenience path.
TEST(ActiveSetSolver, DenseMatchesQpProblemDense) {
  const QpProblem problem = scalar_qp_examples::CreateMatlabExample();
  const Values initial = scalar_qp_examples::Values2(0.0, 0.0);
  const auto [denseValues, denseState] =
      ActiveSetSolver(problem, DenseQpParams()).optimizeWithState(initial);
  const Values problemValues = problem.optimize(initial, QpSolverType::Dense);

  CHECK(assert_equal(problemValues, denseValues, 1e-7));
  CHECK(denseState.converged);
}

// Verifies the active-set solver can warm-start from row-ordered state.
TEST(ActiveSetSolver, QpWarmStartRestoresActiveRows) {
  ActiveSetSolver solver(scalar_qp_examples::CreateMatlabExample());
  const auto [firstValues, firstState] =
      solver.optimizeWithState(scalar_qp_examples::Values2(0.0, 0.0));
  const auto [secondValues, secondState] =
      solver.optimizeWithState(firstState.values, firstState);

  CHECK(scalar_qp_examples::Values2Equal(firstValues, 2.0 / 3.0, 4.0 / 3.0));
  CHECK(scalar_qp_examples::Values2Equal(secondValues, 2.0 / 3.0, 4.0 / 3.0));
  CHECK_EQUAL(firstState.activeInequalityRows.size(),
              secondState.activeInequalityRows.size());
  for (size_t i = 0; i < firstState.activeInequalityRows.size(); ++i) {
    CHECK_EQUAL(firstState.activeInequalityRows[i],
                secondState.activeInequalityRows[i]);
  }
  CHECK(secondState.iterations <= firstState.iterations);
}

// Verifies sparse and dense QP solving agree on chain QPs.
TEST(ActiveSetSolver, SparseChainMatchesDense) {
  constexpr size_t variables = 32;
  const QpProblem problem = CreateSparseChainQp(variables);
  const Values initial = SparseChainInitialValues(variables);

  const Values dense = problem.optimize(initial, QpSolverType::Dense);
  const Values sparse = problem.optimize(initial, QpSolverType::Sparse);

  for (size_t i = 0; i < variables; ++i) {
    const Key key = Symbol('c', i);
    EXPECT(assert_equal(dense.at<Vector>(key), sparse.at<Vector>(key), 1e-6));
  }
}

}  // namespace constrained_qp_features
/* ************************************************************************* */
namespace qps_file_examples {

const Key x1 = Symbol('X', 1);
const Key x2 = Symbol('X', 2);
const QpSolverType kSolverTypes[] = {QpSolverType::Sparse, QpSolverType::Dense};

Vector Vector1D(double value) { return Vector{{value}}; }

Values Values2(double first, double second) {
  Values values;
  values.insert(x1, Vector1D(first));
  values.insert(x2, Vector1D(second));
  return values;
}

QpProblem ParseQpsProblem(const std::string& name) {
  return QpsParser::parseFile(findExampleDataFile(name)).problem;
}

Values OptimizeQps(const std::string& name, QpSolverType solverType) {
  return ParseQpsProblem(name).optimize(solverType);
}

double ObjectiveAtSolution(const std::string& name, QpSolverType solverType) {
  const QpProblem problem = ParseQpsProblem(name);
  const Values actual = problem.optimize(solverType);
  return problem.costs().error(actual);
}

// Verifies QPExample solves to the legacy optimum with both QP solvers.
TEST(QpSolver, QPS_QPExample) {
  for (const QpSolverType solverType : kSolverTypes) {
    CHECK(assert_equal(Values2(0.7625, 0.4750),
                       OptimizeQps("QPExample.QPS", solverType), 1e-7));
  }
}

// Verifies HS21 solves to the legacy solution and objective with both solvers.
TEST(QpSolver, QPS_HS21) {
  const QpProblem problem = ParseQpsProblem("HS21.QPS");
  for (const QpSolverType solverType : kSolverTypes) {
    const Values actual = problem.optimize(solverType);
    CHECK(assert_equal(Values2(2.0, 0.0), actual, 1e-7));
    EXPECT_DOUBLES_EQUAL(-99.9599999, problem.costs().error(actual), 1e-7);
  }
}

// Verifies HS35 matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_HS35) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(1.11111111e-01,
                         ObjectiveAtSolution("HS35.QPS", solverType), 1e-7);
  }
}

// Verifies HS35MOD matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_HS35MOD) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(2.50000001e-01,
                         ObjectiveAtSolution("HS35MOD.QPS", solverType), 1e-7);
  }
}

// Verifies HS51 matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_HS51) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(8.88178420e-16,
                         ObjectiveAtSolution("HS51.QPS", solverType), 1e-7);
  }
}

// Verifies HS52 matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_HS52) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(5.32664756,
                         ObjectiveAtSolution("HS52.QPS", solverType), 1e-5);
  }
}

// Verifies HS268 matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_HS268) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(5.73107049e-07,
                         ObjectiveAtSolution("HS268.QPS", solverType), 1e-6);
  }
}

// Verifies QPTEST matches the legacy objective with both QP solvers.
TEST(QpSolver, QPS_QPTEST) {
  for (const QpSolverType solverType : kSolverTypes) {
    EXPECT_DOUBLES_EQUAL(0.437187500e01,
                         ObjectiveAtSolution("QPTEST.QPS", solverType), 1e-7);
  }
}

}  // namespace qps_file_examples
/* ************************************************************************* */

/* ************************************************************************* */
namespace redundant_sparse_multipliers {

// Opposing bounds fix a variable although their multipliers are nonunique.
// Either gradient sign must eventually produce dual-feasible multipliers.
TEST(ActiveSetSolver, SparseOpposingBounds) {
  for (double sign : {-1.0, 1.0}) {
    QpProblem problem;
    problem.addCost(
        HessianFactor(0, Matrix::Ones(1, 1), Vector::Constant(1, sign), 0));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(0, Matrix::Ones(1, 1), Vector::Zero(1))));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(0, -Matrix::Ones(1, 1), Vector::Zero(1))));
    Values initial;
    initial.insert(0, Vector::Zero(1).eval());

    const auto solved = ActiveSetSolver(problem).optimizeWithState(initial);
    CHECK(solved.second.converged);
    EXPECT_DOUBLES_EQUAL(0, solved.first.at<Vector>(0)(0), 1e-8);
    const auto& duals = solved.second.inequalityMultipliers;
    CHECK(duals[0](0) <= 1e-8 && duals[1](0) <= 1e-8);
    EXPECT_DOUBLES_EQUAL(-sign, duals[0](0) - duals[1](0), 1e-8);
  }
}

// Duplicate equality rows need a valid multiplier combination, not uniqueness.
TEST(ActiveSetSolver, SparseDuplicateEqualities) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Ones(1, 1), Vector::Ones(1), 0));
  const auto equality = LinearConstraint::Equal(
      JacobianFactor(0, Matrix::Ones(1, 1), Vector::Zero(1)));
  problem.addConstraint(equality);
  problem.addConstraint(equality);
  Values initial;
  initial.insert(0, Vector::Zero(1).eval());

  const auto solved = ActiveSetSolver(problem).optimizeWithState(initial);
  CHECK(solved.second.converged);
  const auto& duals = solved.second.equalityMultipliers;
  EXPECT_DOUBLES_EQUAL(-1, duals[0](0) + duals[1](0), 1e-8);
}

// Cold feasibility search and QP optimization tolerate duplicate normals.
TEST(ActiveSetSolver, SparseColdSolveDuplicateBounds) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(2, 2), Vector::Ones(2), 0));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      0, Matrix{{1, 0}, {2, 0}, {0, 1}}, Vector3{0, 0, 0.5})));

  const auto solved = ActiveSetSolver(problem).optimizeWithState();
  CHECK(solved.second.converged);
  CHECK(assert_equal(Vector2{0, 0.5}, solved.first.at<Vector>(0), 1e-7));
}

// A dependency spanning a cycle must survive elimination of earlier cliques.
// Changing row units must preserve stationarity in the original equations.
TEST(ActiveSetSolver, SparseDependentEqualityCycle) {
  constexpr size_t count = 8;
  for (double scale : {1e-3, 1.0, 1e3}) {
    QpProblem problem;
    Values initial;
    for (Key key = 0; key < count; ++key) {
      problem.addCost(HessianFactor(key, Matrix::Ones(1, 1),
                                    Vector::Constant(1, key + 1), 0));
      problem.addConstraint(LinearConstraint::Equal(
          JacobianFactor(key, scale * Matrix::Ones(1, 1), (key + 1) % count,
                         -scale * Matrix::Ones(1, 1), Vector::Zero(1))));
      initial.insert(key, Vector::Zero(1).eval());
    }
    problem.addConstraint(LinearConstraint::Equal(
        JacobianFactor(0, Matrix::Ones(1, 1), Vector::Zero(1))));

    const auto [values, state] =
        ActiveSetSolver(problem).optimizeWithState(initial);
    CHECK(state.converged);
    Vector residual = -Vector::LinSpaced(count, 1, count);
    for (Key key = 0; key < count; ++key) {
      EXPECT_DOUBLES_EQUAL(0.0, values.at<Vector>(key)(0), 1e-8);
      const double multiplier = scale * state.equalityMultipliers[key](0);
      residual(key) -= multiplier;
      residual((key + 1) % count) += multiplier;
    }
    residual(0) -= state.equalityMultipliers.back()(0);
    CHECK(residual.lpNorm<Eigen::Infinity>() < 1e-7);
  }
}

// Gauging a redundant row must retain a weak but independent normal direction.
TEST(ActiveSetSolver, SparseDependentAndWeakEqualityNormals) {
  const Matrix normals{{1, 0}, {1, 1e-5}, {2, 0}};
  const Vector2 gradient{-1, -1e-5};
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(2, 2), -gradient, 0));
  problem.addConstraint(
      LinearConstraint::Equal(JacobianFactor(0, normals, Vector3::Zero())));
  Values initial;
  initial.insert(0, Vector::Zero(2).eval());

  const auto [values, state] =
      ActiveSetSolver(problem).optimizeWithState(initial);
  CHECK(state.converged);
  CHECK(assert_equal(Vector2::Zero(), values.at<Vector>(0), 1e-8));
  const Vector2 residual =
      gradient - normals.transpose() * state.equalityMultipliers[0];
  CHECK(residual.lpNorm<Eigen::Infinity>() < 1e-8);
}

}  // namespace redundant_sparse_multipliers
/* ************************************************************************* */

/* ************************************************************************* */
namespace sparse_proximal_qp {

ActiveSetSolverParams::shared_ptr Parameters(double regularization) {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->qpSubproblemSolver =
      ActiveSetSolverParams::QpSubproblemSolver::Sparse;
  params->regularization = regularization;
  params->stepTolerance = 1e-8;
  params->multiplierTolerance = 1e-9;
  return params;
}

// Proximal regularization must not shrink the unconstrained optimum.
TEST(ActiveSetSolver, SparseProximalOriginalOptimum) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(10), 0));
  Values initial;
  initial.insert(0, Vector(Vector1(0)));

  const auto solved =
      ActiveSetSolver(problem, Parameters(1)).optimizeWithState(initial);
  CHECK(solved.second.converged);
  EXPECT_DOUBLES_EQUAL(10, solved.first.at<Vector>(0)(0), 1e-9);
  CHECK(solved.second.iterations > 2);
}

// Active-bound multipliers must describe the original QP gradient.
TEST(ActiveSetSolver, SparseProximalOriginalBoundMultiplier) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(10), 0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(3))));

  const auto solved =
      ActiveSetSolver(problem, Parameters(1)).optimizeWithState();
  EXPECT_DOUBLES_EQUAL(3, solved.first.at<Vector>(0)(0), 1e-9);
  EXPECT_DOUBLES_EQUAL(-7, solved.second.inequalityMultipliers[0](0), 1e-9);
}

// A free direction retains its current value under a proximal solve.
TEST(ActiveSetSolver, SparseProximalPreservesFreeDirection) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix{{1, 0}, {0, 0}}, Vector2{10, 0}, 0));
  Values initial;
  initial.insert(0, Vector(Vector2{0, 7}));

  const auto solved =
      ActiveSetSolver(problem, Parameters(0.1)).optimizeWithState(initial);
  CHECK(assert_equal(Vector2{10, 7}, solved.first.at<Vector>(0), 1e-9));
}

// Tiny steps caused by large regularization do not prove original optimality.
TEST(ActiveSetSolver, SparseProximalRejectsPrematureConvergence) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(1), 0));
  auto params = Parameters(1e6);
  params->stepTolerance = 1e-5;
  params->maxIterations = 3;
  Values initial;
  initial.insert(0, Vector(Vector1(0)));

  CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimizeWithState(initial),
                  std::runtime_error);
}

// Original stationarity is absolute while the gap scales with objective magnitude.
TEST(ActiveSetSolver, SparseProximalOriginalGap) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(1000), 0));
  Values initial;
  initial.insert(0, Vector(Vector1(0)));

  const auto solved =
      ActiveSetSolver(problem, Parameters(1e-3)).optimizeWithState(initial);
  const double x = solved.first.at<Vector>(0)(0);
  CHECK(std::abs(x - 1000) <= 1e-9);
  CHECK(std::abs(x * (x - 1000)) <= 1e-9 * (1 + 0.5 * x * x));
}

}  // namespace sparse_proximal_qp
/* ************************************************************************* */

/* ************************************************************************* */
namespace sparse_equality_first {

// A feasible equality projection avoids the iterative cold-start LP.
TEST(ActiveSetSolver, SparseEqualityFirstFeasiblePoint) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(2), 0));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2))));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->maxIterations = 1;

  const auto solved = ActiveSetSolver(problem, params).optimizeWithState();
  CHECK(solved.second.converged);
  EXPECT_DOUBLES_EQUAL(2, solved.first.at<Vector>(0)(0), 1e-9);
}

// A projected point that violates an inequality still uses phase one.
TEST(ActiveSetSolver, SparseEqualityFirstFallsBack) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(2, 2), Vector2::Zero(), 0));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix{{1, 1}}, Vector1(1))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix{{1, 0}}, Vector1(0.8))));

  const auto solved = ActiveSetSolver(problem).optimizeWithState();
  CHECK(solved.second.converged);
  CHECK(assert_equal(Vector2{0.8, 0.2}, solved.first.at<Vector>(0), 1e-7));
}

// Inconsistent equality and inequality rows still report infeasibility.
TEST(ActiveSetSolver, SparseEqualityFirstInfeasible) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix::Identity(1, 1), Vector1(0), 0));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(1))));

  CHECK_EXCEPTION(ActiveSetSolver(problem).optimizeWithState(),
                  std::runtime_error);
}

}  // namespace sparse_equality_first
/* ************************************************************************* */

/* ************************************************************************* */
namespace square_root_qp {

// Releasing a bound must preserve a weak finite least-squares direction.
TEST(ActiveSetSolver, SquareRootWeakDirection) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix{{1e8, 1e8}, {1.0, 0.0}},
                                Vector2{1e8, 1.0}));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, Matrix{{-1.0, 0.0}}, Vector1(0.0))));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->maxIterations = 10;
  Values initial;
  initial.insert(0, Vector(Vector2::Zero()));
  const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
  CHECK(result.second.converged);
  CHECK(assert_equal(Vector2{1.0, 0.0}, result.first.at<Vector>(0), 1e-7));
}

// A feasible exact subproblem minimum needs no duplicate factorization.
TEST(ActiveSetSolver, InspectUnregularizedMinimumImmediately) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2.0)));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->maxIterations = 1;
  Values initial;
  initial.insert(0, Vector(Vector1(0.0)));
  const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
  CHECK(result.second.converged);
  CHECK(assert_equal(Vector1(2.0), result.first.at<Vector>(0), 1e-9));
  CHECK(result.second.iterations == 1);
}

// Releasing a bound requires one constrained solve and one free solve.
TEST(ActiveSetSolver, ReleaseUnregularizedBoundWithoutDuplicateSolve) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(0.0))));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->maxIterations = 2;
  Values initial;
  initial.insert(0, Vector(Vector1(0.0)));
  const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
  CHECK(result.second.converged);
  CHECK(assert_equal(Vector1(2.0), result.first.at<Vector>(0), 1e-9));
  CHECK(result.second.iterations == 2);
}

}  // namespace square_root_qp
/* ************************************************************************* */

/* ************************************************************************* */
namespace sparse_ordering {

// Activating a row can introduce a variable absent from the objective graph.
TEST(ActiveSetSolver, OrderingIncludesNewConstraintVariable) {
  QpProblem problem;
  problem.addCost(JacobianFactor(10, Matrix::Identity(1, 1), Vector1(2.0)));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      10, Matrix::Identity(1, 1), 30, Matrix::Identity(1, 1), Vector1(1.0))));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->maxIterations = 2;
  Values initial;
  initial.insert(10, Vector(Vector1(0.0)));
  initial.insert(30, Vector(Vector1(0.0)));
  const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
  CHECK(result.second.converged);
  CHECK(assert_equal(Vector1(2.0), result.first.at<Vector>(10), 1e-9));
  CHECK(assert_equal(Vector1(-1.0), result.first.at<Vector>(30), 1e-9));
  CHECK(result.second.iterations == 2);
  CHECK(result.second.activeInequalityRows[0]);
  EXPECT_DOUBLES_EQUAL(0, result.second.inequalityMultipliers[0](0), 1e-9);
}

// Reuse the primal ordering as rows change, but replace a dual ordering when
// one bound replaces another, even though the multiplier key count is unchanged.
TEST(ActiveSetSolver, OrderingAcrossBoundSwap) {
  QpProblem problem;
  problem.addCost(JacobianFactor(10, Matrix::Identity(1, 1), Vector1(2.0)));
  problem.addCost(JacobianFactor(30, Matrix::Identity(1, 1), Vector1(3.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(10, Matrix::Identity(1, 1), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(10, Matrix::Identity(1, 1), Vector1(1.0))));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->maxIterations = 3;
  const ActiveSetSolver solver(problem, params);
  for (double start : {0.0, 1.0}) {
    Values initial;
    initial.insert(10, Vector(Vector1(start)));
    initial.insert(30, Vector(Vector1(0.0)));
    const auto result = solver.optimizeWithState(initial);
    CHECK(result.second.converged);
    CHECK(assert_equal(Vector1(1.0), result.first.at<Vector>(10), 1e-9));
    CHECK(assert_equal(Vector1(3.0), result.first.at<Vector>(30), 1e-9));
    EXPECT_DOUBLES_EQUAL(0, result.second.inequalityMultipliers[0](0), 1e-9);
    EXPECT_DOUBLES_EQUAL(-1, result.second.inequalityMultipliers[1](0), 1e-9);
    CHECK(!result.second.activeInequalityRows[0]);
    CHECK(result.second.activeInequalityRows[1]);
    CHECK(result.second.iterations == (start == 0.0 ? 3 : 1));
  }
}

}  // namespace sparse_ordering
/* ************************************************************************* */

/* ************************************************************************* */
namespace incremental_sparse_qp {

// Batch and incremental solves agree when bounds swap or a row introduces a key.
TEST(ActiveSetSolver, IncrementalWorkingSetMatchesBatch) {
  for (bool constraintOnlyKey : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(10, Matrix::Identity(1, 1), Vector1(2.0)));
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(10, Matrix::Identity(1, 1), Vector1(0.0))));
    const JacobianFactor upper = constraintOnlyKey
        ? JacobianFactor(10, Matrix::Identity(1, 1), 30,
                         Matrix::Identity(1, 1), Vector1(1.0))
        : JacobianFactor(10, Matrix::Identity(1, 1), Vector1(1.0));
    problem.addConstraint(LinearConstraint::LessEqual(upper));
    Values initial, batch;
    initial.insert(10, Vector(Vector1(0.0)));
    if (constraintOnlyKey) initial.insert(30, Vector(Vector1(0.0)));
    for (bool incremental : {false, true}) {
      auto params = std::make_shared<ActiveSetSolverParams>();
      params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
      params->regularization = 0;
      params->useIncrementalSparseQp = incremental;
      params->maxIterations = 3;
      const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
      CHECK(result.second.converged);
      CHECK(result.second.iterations == 3);
      CHECK(assert_equal(Vector1(constraintOnlyKey ? 2.0 : 1.0),
                          result.first.at<Vector>(10), 1e-9));
      if (incremental) {
        CHECK(assert_equal(batch, result.first, 1e-9));
      } else {
        batch = result.first;
      }
    }
  }
}

// Removing redundant active bounds retains feasibility and valid multipliers.
TEST(ActiveSetSolver, IncrementalOpposingAndDuplicateBounds) {
  for (double target : {-1.0, 1.0}) {
    QpProblem problem;
    problem.addCost(HessianFactor(0, Matrix::Ones(1, 1), Vector1(target), 0));
    const auto upper = LinearConstraint::LessEqual(
        JacobianFactor(0, Matrix::Ones(1, 1), Vector1(0.0)));
    problem.addConstraint(upper);
    problem.addConstraint(upper);
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(0, Matrix::Ones(1, 1), Vector1(0.0))));
    Values initial;
    initial.insert(0, Vector(Vector1(0.0)));
    for (bool incremental : {false, true}) {
      auto params = std::make_shared<ActiveSetSolverParams>();
      params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
      params->regularization = 0;
      params->useIncrementalSparseQp = incremental;
      const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
      CHECK(result.second.converged);
      CHECK(assert_equal(Vector1(0.0), result.first.at<Vector>(0), 1e-9));
      const auto& duals = result.second.inequalityMultipliers;
      for (const Vector& dual : duals) CHECK(dual(0) <= 1e-9);
      EXPECT_DOUBLES_EQUAL(-target, duals[0](0) + duals[1](0) - duals[2](0), 1e-9);
    }
  }
}


// Proximal centers move repeatedly while affine bounds activate and release.
TEST(ActiveSetSolver, IncrementalProximalMatchesBatch) {
  for (double regularization : {0.0, 1e-9, 0.25}) {
    for (bool jacobian : {false, true}) {
      QpProblem problem;
      const JacobianFactor cost(0, Matrix::Identity(2, 2), Vector2{2.0, -1.0});
      if (jacobian) problem.addCost(cost);
      else problem.addCost(HessianFactor(cost));
      problem.addConstraint(LinearConstraint::Equal(
          JacobianFactor(0, Matrix{{1.0, 1.0}}, Vector1{1.0})));
      problem.addConstraint(LinearConstraint::GreaterEqual(
          JacobianFactor(0, Matrix{{1.0, 0.0}}, Vector1{0.0})));
      problem.addConstraint(LinearConstraint::LessEqual(
          JacobianFactor(0, Matrix{{1.0, 0.0}}, Vector1{1.25})));
      Values initial, batch;
      initial.insert(0, Vector(Vector2{0.0, 1.0}));
      for (bool incremental : {false, true}) {
        auto params = sparse_proximal_qp::Parameters(regularization);
        params->useIncrementalSparseQp = incremental;
        const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
        CHECK(result.second.converged);
        CHECK(assert_equal(Vector2{1.25, -0.25}, result.first.at<Vector>(0), 1e-8));
        if (incremental) {
          CHECK(assert_equal(batch, result.first, 1e-8));
        } else {
          batch = result.first;
        }
      }
    }
  }
}

// Positive regularization preserves dependent/opposing working-row robustness.
TEST(ActiveSetSolver, IncrementalProximalDependentRows) {
  for (double regularization : {1e-9, 0.5}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{2.0, 3.0}));
    const JacobianFactor fixed(0, Matrix{{1.0, 0.0}}, Vector1{1.0});
    problem.addConstraint(LinearConstraint::LessEqual(fixed));
    problem.addConstraint(LinearConstraint::GreaterEqual(fixed));
    problem.addConstraint(LinearConstraint::LessEqual(fixed));
    Values initial, batch;
    initial.insert(0, Vector(Vector2{1.0, 0.0}));
    for (bool incremental : {false, true}) {
      auto params = sparse_proximal_qp::Parameters(regularization);
      params->useIncrementalSparseQp = incremental;
      const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
      CHECK(result.second.converged);
      CHECK(assert_equal(Vector2{1.0, 3.0}, result.first.at<Vector>(0), 1e-8));
      if (incremental) {
        CHECK(assert_equal(batch, result.first, 1e-8));
      } else {
        batch = result.first;
      }
    }
  }
}


// Weak finite information survives both proximal-center and working-row changes.
TEST(ActiveSetSolver, IncrementalProximalWeakWeightedDirection) {
  for (double regularization : {0.0, 1e-9}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(
        0, Matrix{{1e-10, 0.0}, {0.0, 1e-15}}, Vector2{1e-10, 2e-15},
        noiseModel::Isotropic::Sigma(2, 1e-10)));
    problem.addConstraint(LinearConstraint::Equal(
        JacobianFactor(0, Matrix{{1.0, 0.0}}, Vector1{0.25})));
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(0, Matrix{{0.0, 1.0}}, Vector1{0.0})));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(0, Matrix{{0.0, 1.0}}, Vector1{1.5})));
    Values initial, batch;
    initial.insert(0, Vector(Vector2{0.25, 0.0}));
    for (bool incremental : {false, true}) {
      auto params = sparse_proximal_qp::Parameters(regularization);
      params->multiplierTolerance = 1e-13;
      params->stepTolerance = 1e-12;
      params->useIncrementalSparseQp = incremental;
      const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
      CHECK(result.second.converged);
      CHECK(assert_equal(Vector2{0.25, 1.5}, result.first.at<Vector>(0), 1e-8));
      if (incremental) {
        CHECK(assert_equal(batch, result.first, 1e-8));
      } else {
        batch = result.first;
      }
    }
  }
}

}  // namespace incremental_sparse_qp
/* ************************************************************************* */

/* ************************************************************************* */
namespace projected_gradient_initialization {

ActiveSetSolverParams::shared_ptr Parameters(bool projected = true) {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->qpInitialization =
      projected ? ActiveSetSolverParams::QpInitialization::ProjectedGradient
                : ActiveSetSolverParams::QpInitialization::TightConstraints;
  return params;
}

// Independent bounds can be released together despite a coupled objective.
TEST(ActiveSetSolver, ProjectedGradientIndependentBounds) {
  QpProblem problem;
  Values initial;
  for (Key key = 0; key < 6; ++key) {
    initial.insert(key, Vector::Zero(1).eval());
    problem.addCost(JacobianFactor(key, Matrix::Identity(1, 1), Vector1(2)));
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(key, Matrix::Identity(1, 1), Vector1(0))));
    if (key > 0)
      problem.addCost(JacobianFactor(key - 1, Matrix::Identity(1, 1), key,
                                     -Matrix::Identity(1, 1), Vector1(0)));
  }
  const auto baseline =
      ActiveSetSolver(problem, Parameters(false)).optimizeWithState(initial);
  for (bool incremental : {false, true}) {
    auto params = Parameters();
    params->useIncrementalSparseQp = incremental;
    params->maxIterations = 3;
    const auto result =
        ActiveSetSolver(problem, params).optimizeWithState(initial);
    CHECK(result.second.converged);
    CHECK(result.second.iterations == 1);
    CHECK(baseline.second.iterations > result.second.iterations);
    CHECK(assert_equal(baseline.first, result.first, 1e-9));
  }
}

// Affine equalities join projection components; an initially slack upper bound
// still blocks the main QP. Constraint scales and sigmas do not change the
// seed.
TEST(ActiveSetSolver, ProjectedGradientAffineEqualityAndInactiveBound) {
  for (double scale : {1e-5, 1.0, 1e5}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix::Identity(1, 1), Vector1(3)));
    problem.addCost(JacobianFactor(1, Matrix::Identity(1, 1), Vector1(0)));
    problem.addConstraint(LinearConstraint::Equal(JacobianFactor(
        0, Matrix::Identity(1, 1), 1, Matrix::Identity(1, 1), Vector1(3))));
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(0, scale * Matrix::Identity(1, 1), Vector1(scale)),
        Vector1(7)));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2)), Vector1(3)));
    Values initial;
    initial.insert(0, Vector(Vector1(1)));
    initial.insert(1, Vector(Vector1(2)));
    const auto result =
        ActiveSetSolver(problem, Parameters()).optimizeWithState(initial);
    CHECK(result.second.converged);
    CHECK(result.second.iterations == 2);
    CHECK(assert_equal(Vector1(2), result.first.at<Vector>(0), 1e-8));
    CHECK(assert_equal(Vector1(1), result.first.at<Vector>(1), 1e-8));
    CHECK(!result.second.activeInequalityRows[0]);
    CHECK(result.second.activeInequalityRows[1]);
  }
}

// A vector block releases one row while dependent/opposing rows remain
// feasible; an unconstrained matrix value retains its shape.
TEST(ActiveSetSolver, ProjectedGradientDependentRowsAndMatrixValues) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(2, 2), Vector2(2, -1)));
  problem.addCost(JacobianFactor(1, Matrix::Identity(2, 2), Vector2(3, 4)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix::Identity(2, 2), Vector2::Zero()),
      Vector2(2, 3)));
  const Matrix row = (Matrix(1, 2) << 0, 1).finished();
  problem.addConstraint(
      LinearConstraint::GreaterEqual(JacobianFactor(0, 2 * row, Vector1(0))));
  problem.addConstraint(
      LinearConstraint::LessEqual(JacobianFactor(0, row, Vector1(0))));
  Values initial;
  initial.insert(0, Vector(Vector2::Zero()));
  initial.insert(1, Matrix::Zero(1, 2).eval());
  const auto result =
      ActiveSetSolver(problem, Parameters()).optimizeWithState(initial);
  CHECK(result.second.converged);
  CHECK(assert_equal(Vector2(2, 0), result.first.at<Vector>(0), 1e-8));
  CHECK(assert_equal((Matrix(1, 2) << 3, 4).finished(),
                     result.first.at<Matrix>(1), 1e-8));
  CHECK(!result.second.activeInequalityRows[0]);
  for (const auto& multiplier : result.second.inequalityMultipliers)
    CHECK(multiplier.maxCoeff() <= 1e-7);
}

// Warm starts bypass projection; an exhausted auxiliary solve fails explicitly.
TEST(ActiveSetSolver, ProjectedGradientWarmStartAndFailure) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix::Identity(1, 1), Vector1(0))));
  Values initial;
  initial.insert(0, Vector(Vector1(0)));
  auto params = Parameters();
  params->maxIterations = 1;
  ActiveSetSolverState warm;
  warm.activeInequalityRows = {false};
  const ActiveSetSolver solver(problem, params);
  CHECK(solver.optimizeWithState(initial, warm).second.converged);
  bool failedProjection = false;
  try {
    solver.optimizeWithState(initial);
  } catch (const std::runtime_error& error) {
    failedProjection = std::string(error.what())
                           .find("projected-gradient initialization failed") !=
                       std::string::npos;
  }
  CHECK(failedProjection);
}

// No tight inequalities require a projection, including an unconstrained QP.
TEST(ActiveSetSolver, ProjectedGradientNoTightInequalities) {
  for (bool addSlack : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix::Identity(1, 1), Vector1(2)));
    if (addSlack)
      problem.addConstraint(LinearConstraint::GreaterEqual(
          JacobianFactor(0, Matrix::Identity(1, 1), Vector1(-1))));
    Values initial;
    initial.insert(0, Vector(Vector1(0)));
    auto params = Parameters();
    params->maxIterations = 1;
    const auto result =
        ActiveSetSolver(problem, params).optimizeWithState(initial);
    CHECK(result.second.iterations == 1);
    CHECK(assert_equal(Vector1(2), result.first.at<Vector>(0), 1e-9));
  }
}

// Standard fixtures retain their solutions, including cold feasibility recovery
// and both main QP backends; projection always uses the sparse backend.
TEST(ActiveSetSolver, ProjectedGradientQpsFixtures) {
  for (const std::string name :
       {"QPExample.QPS", "HS21.QPS", "HS35.QPS", "HS35MOD.QPS", "HS51.QPS",
        "HS52.QPS", "HS268.QPS", "QPTEST.QPS"}) {
    const auto problem = qps_file_examples::ParseQpsProblem(name);
    for (auto backend : {ActiveSetSolverParams::QpSubproblemSolver::Sparse,
                         ActiveSetSolverParams::QpSubproblemSolver::Dense}) {
      auto params = Parameters(false);
      params->qpSubproblemSolver = backend;
      params->regularization = 1e-9;  // Match the existing QPS fixture defaults.
      const auto baseline =
          ActiveSetSolver(problem, params).optimizeWithState();
      params->qpInitialization =
          ActiveSetSolverParams::QpInitialization::ProjectedGradient;
      const auto projected =
          ActiveSetSolver(problem, params).optimizeWithState();
      CHECK(projected.second.converged);
      CHECK(assert_equal(baseline.first, projected.first, 1e-5));
      EXPECT_DOUBLES_EQUAL(problem.costs().error(baseline.first),
                           problem.costs().error(projected.first), 1e-6);
    }
  }
}

// The two-stage mode preserves sparse standard-fixture solutions and phase one.
TEST(ActiveSetSolver, FeasibleThenProjectedGradientQpsFixtures) {
  for (const std::string name :
       {"QPExample.QPS", "HS21.QPS", "HS35.QPS", "HS35MOD.QPS", "HS51.QPS",
        "HS52.QPS", "HS268.QPS", "QPTEST.QPS"}) {
    const auto problem = qps_file_examples::ParseQpsProblem(name);
    auto params = Parameters(false);
    const auto baseline = ActiveSetSolver(problem, params).optimizeWithState();
    params->qpInitialization = ActiveSetSolverParams::QpInitialization::
        FeasibleStepThenProjectedGradient;
    const auto initialized =
        ActiveSetSolver(problem, params).optimizeWithState();
    CHECK(initialized.second.converged);
    CHECK(assert_equal(baseline.first, initialized.first, 1e-5));
  }
}

// Both dense initialization modes must certify HS52's original equalities.
TEST(ActiveSetSolver, FeasibleInitializationChecksDenseCandidate) {
  const auto problem = qps_file_examples::ParseQpsProblem("HS52.QPS");
  auto params = Parameters(false);
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  params->regularization = 1e-9;
  const auto baseline = ActiveSetSolver(problem, params).optimize();
  params->qpInitialization = ActiveSetSolverParams::QpInitialization::
      FeasibleStepThenProjectedGradient;
  const auto result = ActiveSetSolver(problem, params).optimize();
  CHECK(problem.eConstraints().violationVector(result, false)
            .lpNorm<Eigen::Infinity>() <= params->feasibilityTolerance);
  CHECK(assert_equal(baseline, result, 1e-5));
}

}  // namespace projected_gradient_initialization
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
