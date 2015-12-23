/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testScenarioRunner.cpp
 * @brief   test ImuFacor with ScenarioRunner class
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/ScenarioRunner.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

static const double degree = M_PI / 180.0;

/* ************************************************************************* */
TEST(ScenarioRunner, Forward) {
  const double v = 2;  // m/s
  Scenario forward(Vector3::Zero(), Vector3(v, 0, 0), 1e-2, 0.000001, 1);

  ScenarioRunner runner(forward);
  const double T = 1;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(forward.pose(T), runner.predict(pim).pose, 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-9));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, w = 6 * degree;
  Scenario circle(Vector3(0, 0, w), Vector3(v, 0, 0));

  ScenarioRunner runner(circle);
  const double T = 15;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(circle.pose(T), runner.predict(pim).pose, 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Loop) {
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 degree/sec (negative in FLU)
  const double v = 2, w = 6 * degree;
  Scenario loop(Vector3(0, -w, 0), Vector3(v, 0, 0));

  ScenarioRunner runner(loop);
  const double T = 30;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(loop.pose(T), runner.predict(pim).pose, 0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
