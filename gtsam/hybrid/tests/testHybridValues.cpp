/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file testHybridValues.cpp
 *  @date Jul 28, 2022
 *  @author Shangjie Xue
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

TEST(HybridValues, basics) {
  HybridValues values;
  values.insert(99, Vector2(2, 3));
  values.insert(100, 3);
  EXPECT(assert_equal(values.at(99), Vector2(2, 3)));
  EXPECT(assert_equal(values.atDiscrete(100), int(3)));

  values.print();

  HybridValues values2;
  values2.insert(100, 3);
  values2.insert(99, Vector2(2, 3));
  EXPECT(assert_equal(values2, values));

  values2.insert(98, Vector2(2, 3));
  EXPECT(!assert_equal(values2, values));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
