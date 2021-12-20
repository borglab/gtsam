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

// #define ENABLE_TIMING // uncomment for timing results

#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/base/timing.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

static const double kDegree = M_PI / 180.0;
static const double kDt = 1e-2;

// realistic white noise strengths are 0.5 deg/sqrt(hr) and 0.1 (m/s)/sqrt(h)
static const double kGyroSigma = 0.5 * kDegree / 60;
static const double kAccelSigma = 0.1 / 60.0;

static const Vector3 kAccBias(0.2, 0, 0), kRotBias(0.1, 0, 0.3);
static const imuBias::ConstantBias kNonZeroBias(kAccBias, kRotBias);

// Create default parameters with Z-up and above noise parameters
static boost::shared_ptr<PreintegrationParams> defaultParams() {
  auto p = PreintegrationParams::MakeSharedU(10);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0000001 * I_3x3;
  return p;
}

#define EXPECT_NEAR(a, b, c) EXPECT(assert_equal(Vector(a), Vector(b), c));

/* ************************************************************************* */
TEST(ScenarioRunner, Spin) {
  gttic(Spin);
  //  angular velocity 6 degree/sec
  const double w = 6 * kDegree;
  const Vector3 W(0, 0, w), V(0, 0, 0);
  const ConstantTwistScenario scenario(W, V);

  auto p = defaultParams();
  ScenarioRunner runner(scenario, p, kDt);
  const double T = 2 * kDt;  // seconds

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

#if 0
  // Check sampled noise is kosher
  Matrix6 expected;
  expected << p->accelerometerCovariance / kDt, Z_3x3,  //
      Z_3x3, p->gyroscopeCovariance / kDt;
  Matrix6 actual = runner.estimateNoiseCovariance(1000);
  EXPECT(assert_equal(expected, actual, 1e-2));
#endif

  // Check calculated covariance against Monte Carlo estimate
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 1e-5));
}

/* ************************************************************************* */
namespace forward {
const double v = 2;  // m/s
ConstantTwistScenario scenario(Z_3x1, Vector3(v, 0, 0));
}
/* ************************************************************************* */
TEST(ScenarioRunner, Forward) {
  gttic(Forward);
  using namespace forward;
  ScenarioRunner runner(scenario, defaultParams(), kDt);
  const double T = 0.1;  // seconds

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, ForwardWithBias) {
  gttic(ForwardWithBias);
  using namespace forward;
  ScenarioRunner runner(scenario, defaultParams(), kDt);
  const double T = 0.1;  // seconds

  auto pim = runner.integrate(T, kNonZeroBias);
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  gttic(Circle);
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, w = 6 * kDegree;
  ConstantTwistScenario scenario(Vector3(0, 0, w), Vector3(v, 0, 0));

  ScenarioRunner runner(scenario, defaultParams(), kDt);
  const double T = 0.1;  // seconds

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 0.1));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 1e-5));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Loop) {
  gttic(Loop);
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 degree/sec (negative in FLU)
  const double v = 2, w = 6 * kDegree;
  ConstantTwistScenario scenario(Vector3(0, -w, 0), Vector3(v, 0, 0));

  ScenarioRunner runner(scenario, defaultParams(), kDt);
  const double T = 0.1;  // seconds

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 0.1));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 1e-5));
}

/* ************************************************************************* */
namespace initial {
const Rot3 nRb;
const Point3 P0(0,0,0);
const Vector3 V0(0, 0, 0);
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
  gttic(Accelerating);
  using namespace accelerating;
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingWithBias) {
  gttic(AcceleratingWithBias);
  using namespace accelerating;
  ScenarioRunner runner(scenario, defaultParams(), T / 10, kNonZeroBias);

  auto pim = runner.integrate(T, kNonZeroBias);
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingAndRotating) {
  gttic(AcceleratingAndRotating);
  using namespace initial;
  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0, 0.1, 0);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 10;  // seconds
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
namespace initial2 {
// No rotation, but non-zero position and velocities
const Rot3 nRb;
const Point3 P0(10, 20, 0);
const Vector3 V0(50, 0, 0);
}

/* ************************************************************************* */
namespace accelerating2 {
using namespace initial2;
const double a = 0.2;  // m/s^2
const Vector3 A(0, a, 0);
const AcceleratingScenario scenario(nRb, P0, V0, A);

const double T = 3;  // seconds
}

/* ************************************************************************* */
TEST(ScenarioRunner, Accelerating2) {
  gttic(Accelerating2);
  using namespace accelerating2;
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingWithBias2) {
  gttic(AcceleratingWithBias2);
  using namespace accelerating2;
  ScenarioRunner runner(scenario, defaultParams(), T / 10, kNonZeroBias);

  auto pim = runner.integrate(T, kNonZeroBias);
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingAndRotating2) {
  gttic(AcceleratingAndRotating2);
  using namespace initial2;
  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0, 0.1, 0);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 10;  // seconds
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
namespace initial3 {
// Rotation only
// Set up body pointing towards y axis. The body itself has Z axis pointing down
const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
const Point3 P0(0,0,0);
const Vector3 V0(0, 0, 0);
}

/* ************************************************************************* */
namespace accelerating3 {
using namespace initial3;
const double a = 0.2;  // m/s^2
const Vector3 A(0, a, 0);
const AcceleratingScenario scenario(nRb, P0, V0, A);

const double T = 3;  // seconds
}

/* ************************************************************************* */
TEST(ScenarioRunner, Accelerating3) {
  gttic(Accelerating3);
  using namespace accelerating3;
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingWithBias3) {
  gttic(AcceleratingWithBias3);
  using namespace accelerating3;
  ScenarioRunner runner(scenario, defaultParams(), T / 10, kNonZeroBias);

  auto pim = runner.integrate(T, kNonZeroBias);
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingAndRotating3) {
  gttic(AcceleratingAndRotating3);
  using namespace initial3;
  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0, 0.1, 0);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 10;  // seconds
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
namespace initial4 {
// Both rotation and translation
// Set up body pointing towards y axis, and start at 10,20,0 with velocity
// going in X. The body itself has Z axis pointing down
const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
const Point3 P0(10, 20, 0);
const Vector3 V0(50, 0, 0);
}

/* ************************************************************************* */
namespace accelerating4 {
using namespace initial4;
const double a = 0.2;  // m/s^2
const Vector3 A(0, a, 0);
const AcceleratingScenario scenario(nRb, P0, V0, A);

const double T = 3;  // seconds
}

/* ************************************************************************* */
TEST(ScenarioRunner, Accelerating4) {
  gttic(Accelerating4);
  using namespace accelerating4;
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingWithBias4) {
  gttic(AcceleratingWithBias4);
  using namespace accelerating4;
  ScenarioRunner runner(scenario, defaultParams(), T / 10, kNonZeroBias);

  auto pim = runner.integrate(T, kNonZeroBias);
  Matrix9 estimatedCov = runner.estimateCovariance(T, 100, kNonZeroBias);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ScenarioRunner, AcceleratingAndRotating4) {
  gttic(AcceleratingAndRotating4);
  using namespace initial4;
  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0, 0.1, 0);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 10;  // seconds
  ScenarioRunner runner(scenario, defaultParams(), T / 10);

  auto pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T, 100);
  EXPECT_NEAR(estimatedCov.diagonal(), pim.preintMeasCov().diagonal(), 0.1);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  auto result = TestRegistry::runAllTests(tr);
#ifdef ENABLE_TIMING
  tictoc_print_();
#endif
  return result;
}
/* ************************************************************************* */
