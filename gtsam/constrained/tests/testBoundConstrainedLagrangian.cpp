/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testBoundConstrainedLagrangian.cpp
 * @brief Test bound constrained augmented Lagrangian method optimzier for equality constrained
 * optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/BoundConstrainedLagrangian.h>

#include "constrainedExample.h"
#include "gtsam/constrained/NonlinearEqualityConstraint.h"

using namespace gtsam;

/* ********************************************************************************************* */
TEST(BoundConstrainedLagrangianOptimizer, constrained_example1) {
  using namespace constrained_example1;

  auto params = std::make_shared<BoundConstrainedLagrangianParams>();
  params->verbose = true;
  BoundConstrainedLagrangian optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

/* ********************************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
