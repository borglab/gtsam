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
  result->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
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

namespace dense_equality_initialization {

using Initialization = ActiveSetSolverParams::QpInitialization;

ActiveSetSolverParams::shared_ptr parameters(Initialization initialization) {
  auto result = std::make_shared<ActiveSetSolverParams>();
  result->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  result->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  result->qpInitialization = initialization;
  result->regularization = 0;
  return result;
}

// Affine equalities keep one per-call preparation for cold, supplied, and warm starts.
TEST(QpInitialization, DenseReusedAffinePreparation) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(3, 3), Vector{{-2, 3, 2}}));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix{{1, 1, 0}, {2, 2, 0}}, Vector{{1, 2}})));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix{{1, 0, 0}, {2, 0, 0}}, Vector::Zero(2))));
  Values initial;
  initial.insert(0, Vector{{0, 1, 0}});
  for (auto initialization : {Initialization::TightConstraints,
       Initialization::ProjectedGradient, Initialization::FeasibleStepThenProjectedGradient}) {
    const ActiveSetSolver solver(problem, parameters(initialization));
    const auto cold = solver.optimizeWithState();
    const auto supplied = solver.optimizeWithState(initial);
    const auto warm = solver.optimizeWithState(cold.first, cold.second);
    EXPECT(cold.second.converged && supplied.second.converged && warm.second.converged);
    EXPECT(assert_equal(Vector{{0, 1, 2}}, cold.first.at<Vector>(0), 1e-9));
    EXPECT(assert_equal(cold.first, supplied.first, 1e-9));
    EXPECT(assert_equal(cold.first, warm.first, 1e-9));
  }
}

// Independent cone components retain separate scales despite a shared dense basis.
TEST(QpInitialization, DenseProjectedComponentsKeepOriginalScales) {
  QpProblem problem;
  Values initial;
  for (Key key : {Key(10), Key(20)}) {
    const double scale = key == 10 ? 1 : 1e6;
    problem.addCost(JacobianFactor(key, scale * Matrix::Identity(2, 2),
                                    scale * Vector2{-1, 2}));
    problem.addConstraint(LinearConstraint::Equal(
        JacobianFactor(key, Matrix{{1, 1}}, Vector1{1})));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, Matrix{{1, 0}}, Vector1{0})));
    initial.insert(key, Vector{{0, 1}});
  }
  auto settings = parameters(Initialization::ProjectedGradient);
  settings->setOrdering(Ordering{20, 10});
  settings->maxIterations = 2;
  const auto result = ActiveSetSolver(problem, settings).computeFeasibleStep(initial);
  EXPECT(result.status == ActiveSetFeasibleStepResult::Status::FeasibleProgress);
  EXPECT(result.activeInequalityRows == std::vector<bool>({false, false}));
  EXPECT_LONGS_EQUAL(1, result.iterations);
  EXPECT(assert_equal(Vector2{-1, 2}, result.values.at<Vector>(10), 1e-9));
  EXPECT(assert_equal(Vector2{-1, 2}, result.values.at<Vector>(20), 1e-9));
}

// A cached affine point outside the inequalities still uses the existing phase one.
TEST(QpInitialization, DenseEqualityReuseRetainsPhaseOneFallback) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(2, 2), Vector2{3, -2}));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix{{1, 1}}, Vector1{1})));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix{{1, 0}}, Vector1{2})));
  const auto result = ActiveSetSolver(problem,
      parameters(Initialization::FeasibleStepThenProjectedGradient)).optimizeWithState();
  EXPECT(result.second.converged);
  EXPECT(assert_equal(Vector2{3, -2}, result.first.at<Vector>(0), 1e-9));
}

// Opposing rows constrain one tangent direction while another row releases.
TEST(QpInitialization, DenseProjectedOpposingRows) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(3, 3), Vector{{2, 3, 4}}));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix{{1, 1, 0}}, Vector1{1})));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix{{1, 0, 0}, {2, 0, 0}, {0, 0, 1}},
                      Vector{{0, 0, .25}})));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, Matrix{{1, 0, 0}}, Vector1{0})));
  Values initial;
  initial.insert(0, Vector{{0, 1, .25}});
  const auto result = ActiveSetSolver(problem,
      parameters(Initialization::ProjectedGradient)).optimizeWithState(initial);
  EXPECT(result.second.converged);
  EXPECT(assert_equal(Vector{{0, 1, 4}}, result.first.at<Vector>(0), 1e-9));
  for (const auto& multipliers : result.second.inequalityMultipliers)
    EXPECT((multipliers.array() <= 1e-9).all());
}

// Fully fixed coordinates have a zero tangent cone and report no false progress.
TEST(QpInitialization, DenseFullyFixedProjectionHasNoProgress) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix::Identity(2, 2), Vector2{3, 4}));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(0, Matrix::Identity(2, 2), Vector2{1, 2})));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(0, Matrix{{1, 0}, {0, 1}}, Vector2{1, 2})));
  const ActiveSetSolver solver(problem, parameters(Initialization::ProjectedGradient));
  const auto optimized = solver.optimizeWithState();
  EXPECT(optimized.second.converged);
  const auto step = solver.computeFeasibleStep(optimized.first);
  EXPECT(step.status == ActiveSetFeasibleStepResult::Status::NoProgress);
  EXPECT(assert_equal(optimized.first, step.values, 0));
}

}  // namespace dense_equality_initialization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
