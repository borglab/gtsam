/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testScenario.cpp
 * @brief   Unit test Scenario class
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/Scenario.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

static const double degree = M_PI / 180.0;

/* ************************************************************************* */
TEST(Scenario, Forward) {
  const double v = 2;  // m/s
  Scenario forward(Vector3::Zero(), Vector3(v, 0, 0));

  const Pose3 T15 = forward.pose(15);
  EXPECT(assert_equal(Vector3(0, 0, 0), T15.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(30, 0, 0), T15.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, Circle) {
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, omega = 6 * degree;
  Scenario circle(Vector3(0, 0, omega), Vector3(v, 0, 0));

  // R = v/omega, so test if circle is of right size
  const double R = v / omega;
  const Pose3 T15 = circle.pose(15);
  EXPECT(assert_equal(Vector3(0, 0, 90 * degree), T15.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(R, R, 0), T15.translation(), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
