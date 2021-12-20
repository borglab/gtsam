/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testExecutionTrace.cpp
 * @date May 11, 2015
 * @author Frank Dellaert
 * @brief unit tests for Expression internals
 */

#include <gtsam/nonlinear/internal/CallRecord.h>
#include <gtsam/nonlinear/internal/ExecutionTrace.h>
#include <gtsam/geometry/Point2.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Constant
TEST(ExecutionTrace, construct) {
  internal::ExecutionTrace<Point2> trace;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

