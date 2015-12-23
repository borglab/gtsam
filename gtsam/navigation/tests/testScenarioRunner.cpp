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

static const double kDegree = M_PI / 180.0;
static const double kDeltaT = 1e-2;
static const double kGyroSigma = 0.02;
static const double kAccelerometerSigma = 0.1;

/* ************************************************************************* */
TEST(ScenarioRunner, Forward) {
  const double v = 2;  // m/s
  ExpmapScenario scenario(Vector3::Zero(), Vector3(v, 0, 0));

  ScenarioRunner runner(&scenario, kDeltaT, kGyroSigma, kAccelerometerSigma);
  const double T = 0.1;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  // Forward velocity 2m/s, angular velocity 6 kDegree/sec
  const double v = 2, w = 6 * kDegree;
  ExpmapScenario scenario(Vector3(0, 0, w), Vector3(v, 0, 0));

  ScenarioRunner runner(&scenario, kDeltaT, kGyroSigma, kAccelerometerSigma);
  const double T = 0.1;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 0.1));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Loop) {
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 kDegree/sec (negative in FLU)
  const double v = 2, w = 6 * kDegree;
  ExpmapScenario scenario(Vector3(0, -w, 0), Vector3(v, 0, 0));

  ScenarioRunner runner(&scenario, kDeltaT, kGyroSigma, kAccelerometerSigma);
  const double T = 0.1;  // seconds

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 0.1));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Accelerating) {
  // Set up body pointing towards y axis, and start at 10,20,0 with velocity
  // going in X. The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 P0(10, 20, 0);
  const Vector3 V0(50, 0, 0);

  const double a_b = 0.2;  // m/s^2
  const AcceleratingScenario scenario(nRb, P0, V0, Vector3(a_b, 0, 0));

  const double T = 3;  // seconds
  ScenarioRunner runner(&scenario, T / 10, kGyroSigma, kAccelerometerSigma);

  ImuFactor::PreintegratedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
