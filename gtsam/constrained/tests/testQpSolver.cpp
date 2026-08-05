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
namespace scalar_qp_examples {

const Key x1 = Symbol('x', 1);
const Key x2 = Symbol('x', 2);

Matrix Matrix1(double value) { return (Matrix(1, 1) << value).finished(); }

Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

Values Values2(double first, double second) {
  Values values;
  values.insert(x1, Vector1D(first));
  values.insert(x2, Vector1D(second));
  return values;
}

ActiveSetSolverParams::shared_ptr DenseQpParams() {
  auto params = std::make_shared<ActiveSetSolverParams>();
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

// Verifies dense QP mode applies the configured Hessian regularization.
TEST(ActiveSetSolver, DenseRegularizationAffectsSolution) {
  auto params = DenseQpParams();
  params->regularization = 1.0;
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(2.0), 0.0));

  Values initial;
  initial.insert(x1, Vector1D(0.0));
  const Values result = ActiveSetSolver(problem, params).optimize(initial);

  EXPECT_DOUBLES_EQUAL(1.0, result.at<Vector>(x1)(0), 1e-7);
}

// Verifies dependent active constraints fail deterministically in dense mode.
TEST(ActiveSetSolver, DenseDependentEqualityConstraintsThrow) {
  QpProblem problem;
  problem.addCost(HessianFactor(x1, Matrix1(1.0), Vector1D(0.0), 0.0));
  const JacobianFactor equality(x1, Matrix1(1.0), Vector1D(1.0));
  problem.addConstraint(LinearConstraint::Equal(equality));
  problem.addConstraint(LinearConstraint::Equal(equality));

  Values initial;
  initial.insert(x1, Vector1D(1.0));
  CHECK_EXCEPTION(ActiveSetSolver(problem, DenseQpParams()).optimize(initial),
                  std::runtime_error);
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
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x, (Matrix(1, 2) << -1.0, 0.0).finished(), Vector1D(-1.0))));

  Values initial;
  initial.insert(x, (Vector(2) << 10.0, 100.0).finished());
  const Values result = problem.optimize(initial, QpSolverType::Dense);

  EXPECT(assert_equal((Vector(2) << 1.0, 0.0).finished(), result.at<Vector>(x),
                      1e-7));
}

// Verifies infeasible caller-provided initial values are rejected.
TEST(ActiveSetSolver, DenseInfeasibleInitialValues) {
  const Key x = Symbol('x', 0);
  QpProblem problem;
  problem.addCost(
      HessianFactor(x, Matrix::Identity(2, 2), Vector::Zero(2), 0.0));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x, (Matrix(1, 2) << -1.0, 0.0).finished(), Vector1D(-1.0))));

  Values initial;
  initial.insert(x, (Vector(2) << -10.0, 100.0).finished());

  CHECK_EXCEPTION(problem.optimize(initial, QpSolverType::Dense),
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

Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

Matrix Matrix1D(double value) { return (Matrix(1, 1) << value).finished(); }

ActiveSetSolverParams::shared_ptr DenseQpParams() {
  auto params = std::make_shared<ActiveSetSolverParams>();
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
  problem.addCost(HessianFactor(x, Matrix::Identity(2, 2),
                                (Vector(2) << 2.0, 2.0).finished(), 8.0));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 1.0).finished(), Vector1D(2.0))));

  Values initial;
  initial.insert(x, (Matrix(2, 1) << 0.0, 0.0).finished());
  const Values result = problem.optimize(initial, QpSolverType::Dense);

  const Matrix actual = result.at<Matrix>(x);
  EXPECT_LONGS_EQUAL(2, actual.rows());
  EXPECT_LONGS_EQUAL(1, actual.cols());
  EXPECT(assert_equal((Matrix(2, 1) << 1.0, 1.0).finished(), actual, 1e-7));
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
  problem.addCost(HessianFactor(x, Matrix::Identity(2, 2),
                                (Vector(2) << 2.0, 3.0).finished(), 13.0));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x, Matrix::Identity(2, 2), (Vector(2) << 1.0, 1.0).finished())));

  Values initial;
  initial.insert(x, (Vector(2) << 0.0, 0.0).finished());
  const auto [result, state] =
      ActiveSetSolver(problem, DenseQpParams()).optimizeWithState(initial);

  EXPECT(assert_equal((Vector(2) << 1.0, 1.0).finished(), result.at<Vector>(x),
                      1e-7));
  CHECK_EQUAL(1, state.inequalityMultipliers.size());
  CHECK_EQUAL(2, state.inequalityMultipliers.front().size());
  CHECK_EQUAL(2, state.activeInequalityRows.size());
  CHECK(state.activeInequalityRows[0]);
  CHECK(state.activeInequalityRows[1]);
  EXPECT(assert_equal((Vector(2) << -1.0, -2.0).finished(),
                      state.inequalityMultipliers.front(), 1e-7));
}

// Verifies active-set QP state exposes multipliers by constraint row order.
TEST(ActiveSetSolver, QpMultipliersAreRowOrdered) {
  QpProblem problem;
  problem.addCost(HessianFactor(x, Matrix::Identity(2, 2),
                                (Vector(2) << 2.0, 3.0).finished(), 13.0));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      x, Matrix::Identity(2, 2), (Vector(2) << 1.0, 1.0).finished())));

  Values initial;
  initial.insert(x, (Vector(2) << 0.0, 0.0).finished());
  const auto [result, state] =
      ActiveSetSolver(problem).optimizeWithState(initial);

  EXPECT(assert_equal((Vector(2) << 1.0, 1.0).finished(), result.at<Vector>(x),
                      1e-7));
  CHECK_EQUAL(1, state.inequalityMultipliers.size());
  CHECK_EQUAL(2, state.inequalityMultipliers.front().size());
  CHECK_EQUAL(2, state.activeInequalityRows.size());
  CHECK(state.activeInequalityRows[0]);
  CHECK(state.activeInequalityRows[1]);
  EXPECT(assert_equal((Vector(2) << -1.0, -2.0).finished(),
                      state.inequalityMultipliers.front(), 1e-7));
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

Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

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
                         ObjectiveAtSolution("HS52.QPS", solverType), 1e-7);
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
