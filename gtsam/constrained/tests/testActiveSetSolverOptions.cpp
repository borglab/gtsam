/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <limits>

using namespace gtsam;

/* ************************************************************************* */
namespace numerical_contracts {

// A representable optimum must not fail because x amplifies gradient roundoff.
TEST(ActiveSetSolverOptions, LargeRoundedOptimum) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix11{{0.3}}, Vector1{100000}, 0));
  Values initial;
  initial.insert(0, Vector(Vector1{0}));
  for (bool incremental : {false, true}) {
    auto params = std::make_shared<ActiveSetSolverParams>();
    params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
    params->regularization = 0;
    params->maxIterations = 3;
    params->useIncrementalSparseQp = incremental;
    const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial);
    EXPECT(result.second.converged);
    const double x = result.first.at<Vector>(0)(0);
    EXPECT(std::abs(0.3 * x - 100000) < params->multiplierTolerance);
  }
}

// The initializer flag must control both full optimization and feasible steps.
TEST(ActiveSetSolverOptions, FeasibleStepHonorsInitializer) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix11{{1}}, Vector1{2}));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(0, Matrix11{{1}}, Vector1{0})));
  Values initial;
  initial.insert(0, Vector(Vector1{0}));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  EXPECT(ActiveSetSolver(problem, params).computeFeasibleStep(initial).status ==
         ActiveSetFeasibleStepResult::Status::NoProgress);
  params->qpInitialization =
      ActiveSetSolverParams::QpInitialization::ProjectedGradient;
  EXPECT(ActiveSetSolver(problem, params).computeFeasibleStep(initial).status ==
         ActiveSetFeasibleStepResult::Status::FeasibleProgress);
}

// An arbitrary objective constant cannot hide a genuine quadratic decrease.
TEST(ActiveSetSolverOptions, FeasibleDecreaseIgnoresConstantOffset) {
  QpProblem problem;
  problem.addCost(HessianFactor(0, Matrix11{{0.3}}, Vector1{100000}, 1e30));
  Values initial;
  initial.insert(0, Vector(Vector1{0}));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  const auto result = ActiveSetSolver(problem, params).computeFeasibleStep(initial);
  EXPECT(result.status == ActiveSetFeasibleStepResult::Status::FeasibleProgress);
  EXPECT_DOUBLES_EQUAL(result.initialObjective, result.objective, 0);
  EXPECT(std::abs(result.objectiveDecrease - 1e10 / 0.6) < 1e-4);
}

// Invalid tolerances and warm-start shapes cannot silently alter certificates.
TEST(ActiveSetSolverOptions, RejectInvalidOptionsAndWarmStart) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix11{{1}}, Vector1{1}));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->multiplierTolerance = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(ActiveSetSolver(problem, params), std::invalid_argument);
  params->multiplierTolerance = 1e-7;
  params->regularization = -1;
  CHECK_EXCEPTION(ActiveSetSolver(problem, params), std::invalid_argument);
  params->regularization = 0;
  Values initial;
  initial.insert(0, Vector(Vector1{0}));
  ActiveSetSolver::State wrong;
  wrong.activeInequalityRows = {true};
  ActiveSetSolver solver(problem, params);
  CHECK_EXCEPTION(solver.optimizeWithState(initial, wrong), std::invalid_argument);
  initial.update(0, Vector(Vector1{std::numeric_limits<double>::infinity()}));
  CHECK_EXCEPTION(solver.optimize(initial), std::invalid_argument);
}

}  // namespace numerical_contracts
/* ************************************************************************* */
namespace custom_ordering {

QpProblem problem() {
  QpProblem result;
  result.addCost(JacobianFactor(10, Matrix11{{1}}, Vector1{2}));
  result.addCost(JacobianFactor(30, Matrix11{{1}}, Vector1{3}));
  result.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(10, Matrix11{{1}}, Vector1{0})));
  result.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(10, Matrix11{{1}}, Vector1{1})));
  return result;
}

Values zero() {
  Values result;
  result.insert(10, Vector(Vector1{0}));
  result.insert(30, Vector(Vector1{0}));
  return result;
}

// Sparse batch, incremental, dense, and projection solves honor a complete order.
TEST(ActiveSetSolverOptions, CustomOrderingAcrossBackends) {
  const QpProblem qp = problem();
  for (auto backend : {ActiveSetSolverParams::QpSubproblemSolver::Sparse,
                       ActiveSetSolverParams::QpSubproblemSolver::Dense}) {
    for (bool incremental : {false, true}) {
      auto params = std::make_shared<ActiveSetSolverParams>();
      params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
      params->regularization = 0;
      params->qpSubproblemSolver = backend;
      params->useIncrementalSparseQp = incremental;
      params->qpInitialization =
          ActiveSetSolverParams::QpInitialization::ProjectedGradient;
      params->setOrdering(Ordering{30, 10});
      EXPECT(params->hasOrdering());
      ActiveSetSolver solver(qp, params);
      const auto full = solver.optimizeWithState(zero());
      EXPECT(full.second.converged);
      EXPECT_DOUBLES_EQUAL(1, full.first.at<Vector>(10)(0), 1e-9);
      EXPECT_DOUBLES_EQUAL(3, full.first.at<Vector>(30)(0), 1e-9);
      const auto step = solver.computeFeasibleStep(zero());
      EXPECT(step.status == ActiveSetFeasibleStepResult::Status::FeasibleProgress);
      // Caller mutations cannot invalidate cached dense factors or orderings.
      params->maxIterations = 0;
      params->setOrdering(Ordering{999});
      EXPECT(solver.optimizeWithState(zero()).second.converged);
    }
  }
}

// Wrong keys, repeated keys, and incomplete permutations are rejected up front.
TEST(ActiveSetSolverOptions, InvalidOrdering) {
  const QpProblem qp = problem();
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  for (const Ordering& order : {Ordering{10}, Ordering{10, 10},
                                Ordering{10, 99}, Ordering{}}) {
    params->setOrdering(order);
    CHECK_EXCEPTION(ActiveSetSolver(qp, params), std::invalid_argument);
  }
  params->clearOrdering();
  EXPECT(!params->hasOrdering());
  EXPECT(ActiveSetSolver(qp, params).optimizeWithState(zero()).second.converged);
}

// A constraint-only key enters and leaves the graph without changing user order.
TEST(ActiveSetSolverOptions, WorkingGraphKeyChanges) {
  QpProblem qp;
  qp.addCost(JacobianFactor(10, Matrix11{{1}}, Vector1{2}));
  qp.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(10, Matrix11{{1}}, Vector1{0})));
  qp.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(10, Matrix11{{1}}, 30, Matrix11{{1}}, Vector1{1})));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->setOrdering(Ordering{30, 10});
  for (auto initialization : {
           ActiveSetSolverParams::QpInitialization::TightConstraints,
           ActiveSetSolverParams::QpInitialization::ProjectedGradient,
           ActiveSetSolverParams::QpInitialization::FeasibleStepThenProjectedGradient}) {
    params->qpInitialization = initialization;
    const auto full = ActiveSetSolver(qp, params).optimizeWithState(zero());
    EXPECT(full.second.converged);
    EXPECT_DOUBLES_EQUAL(2, full.first.at<Vector>(10)(0), 1e-9);
    EXPECT_DOUBLES_EQUAL(-1, full.first.at<Vector>(30)(0), 1e-9);
  }
}

// Automatic initialization restricts the order in phase one and appends slack.
TEST(ActiveSetSolverOptions, PhaseOneOrdering) {
  QpProblem qp;
  qp.addCost(JacobianFactor(10, Matrix11{{1}}, Vector1{0}));
  qp.addCost(JacobianFactor(30, Matrix11{{1}}, Vector1{0}));
  qp.addConstraint(LinearConstraint::Equal(
      JacobianFactor(10, Matrix11{{1}}, Vector1{0})));
  qp.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(30, Matrix11{{1}}, Vector1{1})));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpFormulation = ActiveSetSolverParams::QpFormulation::Primal;
  params->regularization = 0;
  params->setOrdering(Ordering{30, 10});
  const auto full = ActiveSetSolver(qp, params).optimizeWithState();
  EXPECT(full.second.converged);
  EXPECT_DOUBLES_EQUAL(1, full.first.at<Vector>(30)(0), 1e-7);
}

}  // namespace custom_ordering
/* ************************************************************************* */
int main() { TestResult result; return TestRegistry::runAllTests(result); }
