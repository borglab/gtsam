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

}  // namespace numerical_contracts
/* ************************************************************************* */
int main() { TestResult result; return TestRegistry::runAllTests(result); }
