/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressions.cpp
 * @date May 2019
 * @author Frank Dellaert
 * @brief unit tests for navigation expression helpers
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/expressions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

// A NavState unknown expression wXb with key 42
Expression<NavState> wXb_(42);

/* ************************************************************************* */
// Example: absolute position measurement
TEST(Expressions, Position) {
  auto absolutePosition_ = position(wXb_);

  // Test with some values
  Values values;
  values.insert(42, NavState(Rot3(), Point3(1, 2, 3), Velocity3(4, 5, 6)));
  EXPECT(assert_equal(Point3(1, 2, 3), absolutePosition_.value(values)));
}

/* ************************************************************************* */
// Example: velocity measurement in body frame
TEST(Expressions, Velocity) {
  // We want to predict h(wXb) = velocity in body frame
  // h(wXb) = bRw(wXb) * wV(wXb)
  auto bodyVelocity_ = unrotate(attitude(wXb_), velocity(wXb_));

  // Test with some values
  Values values;
  values.insert(42, NavState(Rot3(), Point3(1, 2, 3), Velocity3(4, 5, 6)));
  EXPECT(assert_equal(Velocity3(4, 5, 6), bodyVelocity_.value(values)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
