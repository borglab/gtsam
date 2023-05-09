/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVariableSlots.cpp
 * @brief
 * @author  Richard Roberts
 * @date Oct 5, 2010
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/VariableSlots.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <boost/assign/std/vector.hpp>

using namespace gtsam;
using namespace std;
using namespace boost::assign;

/* ************************************************************************* */
TEST(VariableSlots, constructor) {

  SymbolicFactorGraph fg;
  fg.push_factor(2, 3);
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(5, 9);

  VariableSlots actual(fg);

  static const size_t none = numeric_limits<size_t>::max();
  VariableSlots expected((SymbolicFactorGraph()));
  expected[0] += none, 0, 0, none;
  expected[1] += none, 1, none, none;
  expected[2] += 0, none, 1, none;
  expected[3] += 1, none, none, none;
  expected[5] += none, none, none, 0;
  expected[9] += none, none, none, 1;

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

