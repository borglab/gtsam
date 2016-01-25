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
#include <gtsam/base/timing.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

static const double kDegree = M_PI / 180.0;
static const double kDeltaT = 1e-2;
static const double kGyroSigma = 0.02;
static const double kAccelSigma = 0.1;

static const Vector3 kAccBias(0.2, 0, 0), kRotBias(0.1, 0, 0.3);
static const imuBias::ConstantBias kNonZeroBias(kAccBias, kRotBias);

// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegrationParams> defaultParams() {
  auto p = PreintegrationParams::MakeSharedD(10);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0000001 * I_3x3;
  return p;
}

/* ************************************************************************* */
namespace forward {
const double v = 2;  // m/s
ConstantTwistScenario scenario(Vector3::Zero(), Vector3(v, 0, 0));
}
/* ************************************************************************* */
TEST(ScenarioRunner, Forward) {
  using namespace forward;
  ScenarioRunner runner(&scenario, defaultParams(), kDeltaT);
  const double T = 0.1;  // seconds

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, ForwardWithBias) {
  using namespace forward;
  ScenarioRunner runner(&scenario, defaultParams(), kDeltaT);
  const double T = 0.1;  // seconds

  PreintegratedImuMeasurements pim = runner.integrate(T, kNonZeroBias);
  Matrix6 estimatedCov = runner.estimatePoseCovariance(T, 1000, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  // Forward velocity 2m/s, angular velocity 6 kDegree/sec
  const double v = 2, w = 6 * kDegree;
  ConstantTwistScenario scenario(Vector3(0, 0, w), Vector3(v, 0, 0));

  ScenarioRunner runner(&scenario, defaultParams(), kDeltaT);
  const double T = 0.1;  // seconds

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 0.1));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Loop) {
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 kDegree/sec (negative in FLU)
  const double v = 2, w = 6 * kDegree;
  ConstantTwistScenario scenario(Vector3(0, -w, 0), Vector3(v, 0, 0));

  ScenarioRunner runner(&scenario, defaultParams(), kDeltaT);
  const double T = 0.1;  // seconds

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 0.1));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 1e-5));
}

/* ************************************************************************* */
namespace initial {
// Set up body pointing towards y axis, and start at 10,20,0 with velocity
// going in X. The body itself has Z axis pointing down
const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
const Point3 P0(10, 20, 0);
const Vector3 V0(50, 0, 0);
}

/* ************************************************************************* */
namespace accelerating {
using namespace initial;
const double a = 0.2;  // m/s^2
const Vector3 A(0, a, 0);
const AcceleratingScenario scenario(nRb, P0, V0, A);

const double T = 3;  // seconds
}

/* ************************************************************************* */
TEST(ScenarioRunner, Accelerating) {
  using namespace accelerating;
  ScenarioRunner runner(&scenario, defaultParams(), T / 10);

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
}

/* ************************************************************************* */
// TODO(frank):Fails !
// TEST(ScenarioRunner, AcceleratingWithBias) {
//  using namespace accelerating;
//  ScenarioRunner runner(&scenario, T / 10, kGyroSigma, kAccelSigma,
//                        kNonZeroBias);
//
//  PreintegratedImuMeasurements pim = runner.integrate(T,
//  kNonZeroBias);
//  Matrix6 estimatedCov = runner.estimatePoseCovariance(T, 10000,
//  kNonZeroBias);
//  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
//}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingAndRotating) {
  using namespace initial;
  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0, 0.1, 0);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 3;  // seconds
  ScenarioRunner runner(&scenario, defaultParams(), T / 10);

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  auto result = TestRegistry::runAllTests(tr);
  tictoc_print_();
  return result;
}
/* ************************************************************************* */
