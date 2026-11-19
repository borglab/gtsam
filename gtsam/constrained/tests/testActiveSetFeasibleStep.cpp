/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/linear/linearExceptions.h>

#include <limits>

using namespace gtsam;
using Status = ActiveSetFeasibleStepResult::Status;

/* ************************************************************************* */
namespace affine_steps {

const Matrix11 unit{{1.0}};

Values point(double x, double y = 0) {
  Values result;
  result.insert(0, Vector(Vector2{x, y}));
  return result;
}

ActiveSetSolverParams::shared_ptr params(bool incremental = true) {
  auto result = std::make_shared<ActiveSetSolverParams>();
  result->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  result->regularization = 0;
  result->useIncrementalSparseQp = incremental;
  return result;
}

void upper(QpProblem* problem, const Matrix12& row, double bound) {
  problem->addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, row, Vector1{bound})));
}

// A slack affine bound truncates descent while an affine equality stays exact.
TEST(ActiveSetFeasibleStep, AffineEqualityAndBlockingBound) {
  for (bool incremental : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{4, -1}));
    problem.addConstraint(LinearConstraint::Equal(
        JacobianFactor(0, Matrix12{{1, 1}}, Vector1{3})));
    upper(&problem, Matrix12{{1, 0}}, 2);
    upper(&problem, Matrix12{{0, -1}}, 1);
    ActiveSetSolver solver(problem, params(incremental));
    const auto result = solver.computeFeasibleStep(point(1, 2));
    EXPECT(result.status == Status::FeasibleProgress);
    EXPECT(assert_equal(Vector2{2, 1}, result.values.at<Vector>(0), 1e-9));
    EXPECT(result.objective < result.initialObjective);
    EXPECT(result.constraintViolation < 1e-9);
    EXPECT(result.activeInequalityRows == std::vector<bool>({true, false}));
    EXPECT_LONGS_EQUAL(1, result.iterations);
  }
}

// Violated affine boundaries x=1 and x=2 are never imposed simultaneously.
TEST(ActiveSetFeasibleStep, IncompatibleAffineBoundaries) {
  for (auto backend : {ActiveSetSolverParams::QpSubproblemSolver::Sparse,
                       ActiveSetSolverParams::QpSubproblemSolver::Dense}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{3, 0}));
    upper(&problem, Matrix12{{1, 0}}, 1);
    upper(&problem, Matrix12{{1, 0}}, 2);
    auto settings = params();
    settings->qpSubproblemSolver = backend;
    const auto result = ActiveSetSolver(problem, settings).computeFeasibleStep(point(0));
    EXPECT(result.status == Status::FeasibleProgress);
    EXPECT(assert_equal(Vector2{1, 0}, result.values.at<Vector>(0), 1e-9));
    EXPECT(result.activeInequalityRows == std::vector<bool>({true, false}));
  }
}

// A first feasible descent step may stop before the QP optimum.
TEST(ActiveSetFeasibleStep, FeasibleDoesNotMeanOptimal) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{3, 3}));
  upper(&problem, Matrix12{{1, 0}}, 1);
  upper(&problem, Matrix12{{0, 1}}, 2);
  ActiveSetSolver solver(problem, params());
  const auto step = solver.computeFeasibleStep(point(0));
  EXPECT(step.status == Status::FeasibleProgress);
  EXPECT_LONGS_EQUAL(1, step.iterations);
  EXPECT(assert_equal(Vector2{1, 1}, step.values.at<Vector>(0), 1e-9));
  const auto optimum = solver.optimizeWithState(point(0));
  EXPECT(optimum.second.converged);
  EXPECT(assert_equal(Vector2{1, 2}, optimum.first.at<Vector>(0), 1e-9));
}

// Explicitly enabling projection releases a binding row without affecting optimize.
TEST(ActiveSetFeasibleStep, ReleasesInwardBindingRow) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{2, 0}));
  upper(&problem, Matrix12{{-1, 0}}, 0);
  upper(&problem, Matrix12{{1, 0}}, 1);
  auto settings = params();
  EXPECT(settings->qpInitialization ==
         ActiveSetSolverParams::QpInitialization::TightConstraints);
  settings->qpInitialization =
      ActiveSetSolverParams::QpInitialization::ProjectedGradient;
  ActiveSetSolver solver(problem, settings);
  const auto before = solver.optimizeWithState(point(0));
  const auto result = solver.computeFeasibleStep(point(0));
  const auto after = solver.optimizeWithState(point(0));
  EXPECT(result.status == Status::FeasibleProgress);
  EXPECT(assert_equal(Vector2{1, 0}, result.values.at<Vector>(0), 1e-9));
  EXPECT(result.activeInequalityRows == std::vector<bool>({false, true}));
  EXPECT(before.second.converged && after.second.converged);
  EXPECT_LONGS_EQUAL(before.second.iterations, after.second.iterations);
  EXPECT(assert_equal(before.first, after.first));
}

// Dependent and opposing rows fix one coordinate while another can descend.
TEST(ActiveSetFeasibleStep, DependentRowsAndNoProgress) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{1, 2}));
  upper(&problem, Matrix12{{1, 0}}, 0);
  upper(&problem, Matrix12{{-1, 0}}, 0);
  upper(&problem, Matrix12{{2, 0}}, 0);
  ActiveSetSolver solver(problem, params());
  const auto first = solver.computeFeasibleStep(point(0));
  EXPECT(first.status == Status::FeasibleProgress);
  EXPECT(assert_equal(Vector2{0, 2}, first.values.at<Vector>(0), 1e-9));
  const auto second = solver.computeFeasibleStep(first.values);
  EXPECT(second.status == Status::NoProgress);
  EXPECT(assert_equal(first.values, second.values));
  EXPECT_DOUBLES_EQUAL(second.initialObjective, second.objective, 1e-12);
}

// Reject invalid starts rather than repairing them or running a feasibility phase.
TEST(ActiveSetFeasibleStep, InvalidStartingPoint) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{1, 0}));
  upper(&problem, Matrix12{{-1, 0}}, 0);
  ActiveSetSolver solver(problem, params());
  CHECK_EXCEPTION(solver.computeFeasibleStep(point(-1)), std::invalid_argument);
  CHECK_EXCEPTION(solver.computeFeasibleStep(point(std::numeric_limits<double>::quiet_NaN())), std::exception);
  Values wrong;
  wrong.insert(0, Vector(Vector1{0}));
  CHECK_EXCEPTION(solver.computeFeasibleStep(wrong), std::invalid_argument);
}

// Already optimal points and exhausted projection budgets have explicit statuses.
TEST(ActiveSetFeasibleStep, OptimalAndProjectionExhaustion) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{2, 0}));
  auto settings = params();
  auto result = ActiveSetSolver(problem, settings).computeFeasibleStep(point(2));
  EXPECT(result.status == Status::NoProgress);
  settings->maxIterations = 0;
  result = ActiveSetSolver(problem, settings).computeFeasibleStep(point(0));
  EXPECT(result.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(0, result.iterations);
  upper(&problem, Matrix12{{-1, 0}}, 0);
  settings->qpInitialization =
      ActiveSetSolverParams::QpInitialization::ProjectedGradient;
  settings->maxIterations = 1;
  result = ActiveSetSolver(problem, settings).computeFeasibleStep(point(0));
  EXPECT(result.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(0, result.iterations);
  EXPECT(result.message.find("projected-gradient") != std::string::npos);
}

// Singular working systems return numerical failure and preserve the input.
TEST(ActiveSetFeasibleStep, NumericalFailure) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix12{{1, 1}}, Vector1{1}));
  ActiveSetSolver solver(problem, params());
  const Values initial = point(0);
  const auto result = solver.computeFeasibleStep(initial);
  EXPECT(result.status == Status::NumericalFailure);
  EXPECT(!result.message.empty());
  EXPECT(assert_equal(initial, result.values));
  CHECK_EXCEPTION(solver.optimize(initial), IndeterminateSystemException);
}

// Matrix values keep their original shape; LP input is explicitly unsupported.
TEST(ActiveSetFeasibleStep, MatrixValuesAndLpPrecondition) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{1, 2}));
  Values initial;
  initial.insert(0, Matrix(Matrix12{{0, 0}}));
  const auto result = ActiveSetSolver(problem, params()).computeFeasibleStep(initial);
  EXPECT(result.status == Status::FeasibleProgress);
  EXPECT(assert_equal(Matrix12{{1, 2}}, result.values.at<Matrix>(0)));
  LpProblem lp;
  lp.addCost(JacobianFactor(0, unit, Vector1{0}));
  Values zero;
  zero.insert(0, Vector(Vector1{0}));
  CHECK_EXCEPTION(ActiveSetSolver(lp).computeFeasibleStep(zero), std::invalid_argument);
}

}  // namespace affine_steps
/* ************************************************************************* */
namespace zero_length_blocks {

QpProblem problem() {
  QpProblem result;
  for (Key x : {0, 2}) {
    result.addCost(JacobianFactor(x, Matrix21{{1}, {0}}, x + 1,
                                  Matrix21{{2}, {1}}, Vector2{1, 3}));
    result.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(x, Matrix11{{1}}, Vector1{0})));
  }
  return result;
}

// Zero-length blocks add one boundary at a time, then permit a feasible move.
TEST(ActiveSetFeasibleStep, ZeroBlocksAndMainExhaustion) {
  Values initial;
  for (Key key : {0, 1, 2, 3}) initial.insert(key, Vector(Vector1{0}));
  const QpProblem qp = problem();
  auto settings = std::make_shared<ActiveSetSolverParams>();
  settings->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  settings->regularization = 0;
  settings->qpInitialization =
      ActiveSetSolverParams::QpInitialization::ProjectedGradient;
  settings->maxIterations = 2;
  const auto exhausted = ActiveSetSolver(qp, settings).computeFeasibleStep(initial);
  EXPECT(exhausted.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(2, exhausted.iterations);
  EXPECT(assert_equal(initial, exhausted.values));
  settings->maxIterations = 3;
  const auto progress = ActiveSetSolver(qp, settings).computeFeasibleStep(initial);
  EXPECT(progress.status == Status::FeasibleProgress);
  EXPECT_LONGS_EQUAL(3, progress.iterations);
  EXPECT(progress.objective < progress.initialObjective);
  EXPECT(progress.constraintViolation < 1e-9);
  for (Key key : {0, 2}) EXPECT_DOUBLES_EQUAL(0, progress.values.at<Vector>(key)(0), 1e-9);
}

}  // namespace zero_length_blocks
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
