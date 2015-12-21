/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ScenarioRunner.h
 * @brief   Simple class to test navigation scenarios
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>

namespace gtsam {

double accNoiseVar = 0.01;
double omegaNoiseVar = 0.03;
double intNoiseVar = 0.0001;
const Matrix3 kMeasuredAccCovariance = accNoiseVar * I_3x3;
const Matrix3 kMeasuredOmegaCovariance = omegaNoiseVar * I_3x3;
const Matrix3 kIntegrationErrorCovariance = intNoiseVar * I_3x3;

/// Simple class to test navigation scenarios
class ScenarioRunner {
 public:
  ScenarioRunner(const Scenario& scenario) : scenario_(scenario) {}

  // Integrate measurements for T seconds
  ImuFactor::PreintegratedMeasurements integrate(double T) {
    // TODO(frank): allow non-zero
    const imuBias::ConstantBias zeroBias;
    const bool use2ndOrderCoriolis = true;

    ImuFactor::PreintegratedMeasurements result(
        zeroBias, kMeasuredAccCovariance, kMeasuredOmegaCovariance,
        kIntegrationErrorCovariance, use2ndOrderCoriolis);

    const Vector3 measuredAcc = scenario_.groundTruthAcc();
    const Vector3 measuredOmega = scenario_.groundTruthGyro();
    double deltaT = scenario_.imuSampleTime();
    for (double t = 0; t <= T; t += deltaT) {
      result.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    }

    return result;
  }

  // Predict mean
  Pose3 mean(const ImuFactor::PreintegratedMeasurements& integrated) {
    // TODO(frank): allow non-standard
    const imuBias::ConstantBias zeroBias;
    const Pose3 pose_i = Pose3::identity();
    const Vector3 vel_i = Vector3::Zero();
    const Vector3 gravity(0, 0, 9.81);
    const Vector3 omegaCoriolis = Vector3::Zero();
    const bool use2ndOrderCoriolis = true;
    const PoseVelocityBias prediction = integrated.predict(
        pose_i, vel_i, zeroBias, gravity, omegaCoriolis, use2ndOrderCoriolis);
    return prediction.pose;
  }

 private:
  Scenario scenario_;
};

}  // namespace gtsam

/**
 * @file    testScenario.cpp
 * @brief   test ImuFacor with ScenarioRunner class
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/Scenario.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

static const double degree = M_PI / 180.0;

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, omega = 6 * degree;
  Scenario circle(Vector3(0, 0, omega), Vector3(v, 0, 0));

  ScenarioRunner runner(circle);
  const double T = 15;  // seconds
  ImuFactor::PreintegratedMeasurements integrated = runner.integrate(T);
  EXPECT(assert_equal(circle.poseAtTime(T), runner.mean(integrated), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
