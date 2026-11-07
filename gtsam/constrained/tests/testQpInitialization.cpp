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

using namespace gtsam;

/* ************************************************************************* */
namespace two_stage_initialization {

using Initialization = ActiveSetSolverParams::QpInitialization;
using Backend = ActiveSetSolverParams::QpSubproblemSolver;
using Status = ActiveSetFeasibleStepResult::Status;

QpProblem problem() {
  QpProblem result;
  result.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{2, 3}));
  result.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix12{{1, 0}}, Vector1{0})));
  return result;
}

Values point(double x = 0, double y = 0) {
  Values result;
  result.insert(0, Vector(Vector2{x, y}));
  return result;
}

ActiveSetSolverParams::shared_ptr params() {
  auto result = std::make_shared<ActiveSetSolverParams>();
  result->regularization = 0;
  result->qpInitialization = Initialization::FeasibleStepThenProjectedGradient;
  result->setOrdering(Ordering{0});
  return result;
}

// The integrated pipeline matches two calls, including its preliminary count.
TEST(QpInitialization, MatchesExplicitComposition) {
  const QpProblem qp = problem();
  for (auto backend : {Backend::Sparse, Backend::Dense}) {
    for (bool incremental : {false, true}) {
      auto settings = params();
      settings->qpSubproblemSolver = backend;
      settings->useIncrementalSparseQp = incremental;
      const auto combined =
          ActiveSetSolver(qp, settings).optimizeWithState(point());
      settings->qpInitialization = Initialization::TightConstraints;
      const auto first =
          ActiveSetSolver(qp, settings).computeFeasibleStep(point());
      EXPECT(first.status == Status::FeasibleProgress);
      settings->qpInitialization = Initialization::ProjectedGradient;
      const auto refined =
          ActiveSetSolver(qp, settings).optimizeWithState(first.values);
      EXPECT(combined.second.converged);
      EXPECT(assert_equal(refined.first, combined.first, 1e-9));
      EXPECT_LONGS_EQUAL(first.iterations + refined.second.iterations,
                         combined.second.iterations);
      EXPECT(combined.second.activeInequalityRows ==
             refined.second.activeInequalityRows);
    }
  }
}

// Feasible-step returns preliminary descent, but its mask uses the new
// gradient.
TEST(QpInitialization, SuppliedPointAndFeasibleStep) {
  const QpProblem qp = problem();
  const auto result =
      ActiveSetSolver(qp, params()).computeFeasibleStep(point(0, 1));
  EXPECT(result.status == Status::FeasibleProgress);
  EXPECT_LONGS_EQUAL(1, result.iterations);
  EXPECT(assert_equal(Vector2{0, 3}, result.values.at<Vector>(0), 1e-9));
  EXPECT_DOUBLES_EQUAL(4, result.initialObjective, 1e-9);
  EXPECT_DOUBLES_EQUAL(2, result.objectiveDecrease, 1e-9);
  EXPECT(result.activeInequalityRows == std::vector<bool>({false}));
  EXPECT(result.constraintViolation <= 1e-9);
}

// Initialization preserves backend-specific regularization semantics.
TEST(QpInitialization, ConfiguredRegularization) {
  const QpProblem qp = problem();
  for (auto backend : {Backend::Sparse, Backend::Dense}) {
    auto settings = params();
    settings->qpSubproblemSolver = backend;
    settings->regularization = 1e-3;
    const auto initialized =
        ActiveSetSolver(qp, settings).optimizeWithState(point());
    settings->qpInitialization = Initialization::TightConstraints;
    const auto baseline =
        ActiveSetSolver(qp, settings).optimizeWithState(point());
    EXPECT(initialized.second.converged);
    EXPECT(assert_equal(baseline.first, initialized.first, 1e-7));
  }
}

// Affine equalities and incompatible bounds survive first-blocking truncation.
TEST(QpInitialization, AffineConstraintsAndCoincidentRows) {
  QpProblem qp;
  qp.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{4, -1}));
  qp.addConstraint(
      LinearConstraint::Equal(JacobianFactor(0, Matrix12{{1, 1}}, Vector1{3})));
  for (double bound : {2., 4., 2.})
    qp.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(0, Matrix12{{1, 0}}, Vector1{bound})));
  qp.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix12{{0, 1}}, Vector1{1})));
  for (bool incremental : {false, true}) {
    auto settings = params();
    settings->useIncrementalSparseQp = incremental;
    const ActiveSetSolver solver(qp, settings);
    const auto step = solver.computeFeasibleStep(point(1, 2));
    EXPECT(step.status == Status::FeasibleProgress);
    EXPECT(assert_equal(Vector2{2, 1}, step.values.at<Vector>(0), 1e-9));
    EXPECT(step.activeInequalityRows ==
           std::vector<bool>({true, false, true, true}));
    EXPECT(step.constraintViolation <= 1e-9);
    EXPECT(solver.optimizeWithState(point(1, 2)).second.converged);
    CHECK_EXCEPTION(solver.optimize(point()), std::invalid_argument);
  }
}

// Opposing and dependent rows fix one direction without dropping its original
// rows.
TEST(QpInitialization, OpposingRowsAndMatrixValues) {
  QpProblem qp = problem();
  qp.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, Matrix12{{1, 0}}, Vector1{0})));
  qp.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix12{{2, 0}}, Vector1{0})));
  Values initial;
  initial.insert(0, Matrix(Matrix12{{0, 0}}));
  const auto result = ActiveSetSolver(qp, params()).optimizeWithState(initial);
  EXPECT(result.second.converged);
  EXPECT(assert_equal(Matrix12{{0, 3}}, result.first.at<Matrix>(0), 1e-9));
  EXPECT_LONGS_EQUAL(3, result.second.inequalityMultipliers.size());
}

// A caller's row mask bypasses both the preliminary solve and cone projection.
TEST(QpInitialization, WarmStartAndSharedIterationBudget) {
  const QpProblem qp = problem();
  auto settings = params();
  settings->maxIterations = 1;
  ActiveSetSolver::State warm;
  warm.activeInequalityRows = {false};
  const ActiveSetSolver solver(qp, settings);
  const auto result = solver.optimizeWithState(point(), warm);
  EXPECT(result.second.converged);
  EXPECT_LONGS_EQUAL(1, result.second.iterations);
  const auto exhausted = solver.computeFeasibleStep(point());
  EXPECT(exhausted.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(1, exhausted.iterations);
  EXPECT(assert_equal(point(), exhausted.values));
  EXPECT(exhausted.message.find("projected-gradient initialization failed") !=
         std::string::npos);
  settings->maxIterations = 0;
  const auto zero = ActiveSetSolver(qp, settings).computeFeasibleStep(point());
  EXPECT(zero.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(0, zero.iterations);
}

// Preliminary descent does not certify optimality or grant another main budget.
TEST(QpInitialization, FeasibleProgressIsNotConvergence) {
  QpProblem qp;
  qp.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{2, 3}));
  auto settings = params();
  settings->maxIterations = 1;
  const ActiveSetSolver solver(qp, settings);
  const auto step = solver.computeFeasibleStep(point());
  EXPECT(step.status == Status::FeasibleProgress);
  EXPECT_LONGS_EQUAL(1, step.iterations);
  CHECK_EXCEPTION(solver.optimize(point()), std::runtime_error);
  settings->maxIterations = 2;
  EXPECT_LONGS_EQUAL(2, ActiveSetSolver(qp, settings)
                            .optimizeWithState(point())
                            .second.iterations);
  const auto optimal =
      ActiveSetSolver(qp, settings).computeFeasibleStep(point(2, 3));
  EXPECT(optimal.status == Status::NoProgress);
  EXPECT(assert_equal(point(2, 3), optimal.values));
}

// An unhelpfully short blocked step cannot be advertised as feasible progress.
TEST(QpInitialization, ShortBlockingStep) {
  QpProblem qp;
  qp.addCost(JacobianFactor(0, Matrix22::Identity(), Vector2{-1, 0}));
  qp.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix12{{1, 0}}, Vector1{0})));
  auto settings = params();
  settings->activeTolerance = 0;
  settings->maxIterations = 2;
  const Values initial = point(1e-8);
  const auto result =
      ActiveSetSolver(qp, settings).computeFeasibleStep(initial);
  EXPECT(result.status == Status::IterationLimit);
  EXPECT_LONGS_EQUAL(2, result.iterations);
  EXPECT(assert_equal(initial, result.values));
}

// Numerical failure preserves the input and reports the attempted face solve.
TEST(QpInitialization, NumericalFailureAndInvalidModes) {
  QpProblem qp;
  qp.addCost(JacobianFactor(0, Matrix12{{1, 1}}, Vector1{1}));
  const ActiveSetSolver solver(qp, params());
  const auto result = solver.computeFeasibleStep(point());
  EXPECT(result.status == Status::NumericalFailure);
  EXPECT_LONGS_EQUAL(1, result.iterations);
  EXPECT(assert_equal(point(), result.values));
  CHECK_EXCEPTION(solver.optimize(point()), IndeterminateSystemException);
  auto settings = params();
  settings->qpInitialization = static_cast<Initialization>(-1);
  CHECK_EXCEPTION(ActiveSetSolver(qp, settings), std::invalid_argument);
  LpProblem lp;
  lp.addCost(JacobianFactor(0, Matrix11{{1}}, Vector1{0}));
  CHECK_EXCEPTION(ActiveSetSolver(lp, params()), std::invalid_argument);
}

}  // namespace two_stage_initialization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
