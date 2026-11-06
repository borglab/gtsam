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
  params->regularization = 0;
  EXPECT(ActiveSetSolver(problem, params).computeFeasibleStep(initial).status ==
         ActiveSetFeasibleStepResult::Status::NoProgress);
  params->useProjectedGradientInitialization = true;
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
  params->regularization = 0;
  const auto result = ActiveSetSolver(problem, params).computeFeasibleStep(initial);
  EXPECT(result.status == ActiveSetFeasibleStepResult::Status::FeasibleProgress);
  EXPECT_DOUBLES_EQUAL(result.initialObjective, result.objective, 0);
  EXPECT(std::abs(result.objectiveDecrease - 1e10 / 0.6) < 1e-4);
}

}  // namespace numerical_contracts
/* ************************************************************************* */
int main() { TestResult result; return TestRegistry::runAllTests(result); }
