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

static const HybridValues kExample{{{99, Vector2(2, 3)}}, {{100, 3}}};

/* ************************************************************************* */
TEST(HybridValues, Basics) {
  HybridValues values;
  values.insert(99, Vector2(2, 3));
  values.insert(100, 3);
  EXPECT(assert_equal(kExample, values));
  EXPECT(assert_equal(values.at(99), Vector2(2, 3)));
  EXPECT(assert_equal(values.atDiscrete(100), int(3)));

  HybridValues values2;
  values2.insert(100, 3);
  values2.insert(99, Vector2(2, 3));
  EXPECT(assert_equal(kExample, values2));
}

/* ************************************************************************* */
// Check insert
TEST(HybridValues, Insert) {
  HybridValues actual;
  EXPECT(assert_equal({{}, {{100, 3}}},  //
                      actual.insert(DiscreteValues{{100, 3}})));
  EXPECT(assert_equal(kExample,  //
                      actual.insert(VectorValues{{99, Vector2(2, 3)}})));
  HybridValues actual2;
  EXPECT(assert_equal(kExample, actual2.insert(kExample)));
}

/* ************************************************************************* */
// Check update.
TEST(HybridValues, Update) {
  HybridValues actual(kExample);
  EXPECT(assert_equal({{{99, Vector2(2, 3)}}, {{100, 2}}},
                      actual.update(DiscreteValues{{100, 2}})));
  EXPECT(assert_equal({{{99, Vector1(4)}}, {{100, 2}}},
                      actual.update(VectorValues{{99, Vector1(4)}})));
  HybridValues actual2(kExample);
  EXPECT(assert_equal(kExample, actual2.update(kExample)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
