/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testPenaltyOptimizr.cpp
 * @brief Test penalty method optimzier for constrained optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/PenaltyOptimizer.h>

#include "constrainedExample.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(PenaltyOptimizer, constrained_example1) {
  using namespace constrained_example1;

  auto params = std::make_shared<PenaltyOptimizerParams>();
  params->verbose = true;
  PenaltyOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  EXPECT(assert_equal(optimal_values, results, 1e-4));
}

/* ************************************************************************* */
TEST(PenaltyOptimizer, constrained_example2) {
  using namespace constrained_example2;

  auto params = std::make_shared<PenaltyOptimizerParams>();
  params->verbose = true;

  {
    PenaltyOptimizer optimizer(problem, init_values, params);
    Values results = optimizer.optimize();

    /// Check the result is correct within tolerance.
    EXPECT(assert_equal(optimal_values, results, 1e-4));
  }

  // Check constructor from a single factor graph works
  {
    NonlinearFactorGraph graph = costs;
    for (const auto& factor: eqConstraints) {
      graph.push_back(factor);
    }
    for (const auto& factor: ineqConstraints) {
      graph.push_back(factor);
    }
    PenaltyOptimizer optimizer(graph, init_values, params);
    Values results = optimizer.optimize();

    /// Check the result is correct within tolerance.
    EXPECT(assert_equal(optimal_values, results, 1e-4));
  }

}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
