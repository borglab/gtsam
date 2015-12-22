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

#include <iostream>

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

    const Vector3 measuredOmega = scenario_.angularVelocityInBody();
    const double deltaT = scenario_.imuSampleTime();
    const size_t nrSteps = T / deltaT;
    double t = 0;
    Vector3 v0 = scenario_.velocity(0);
    Vector3 v = Vector3::Zero();
    Vector3 p = Vector3::Zero();
    for (size_t k = 0; k < nrSteps; k++, t += deltaT) {
      std::cout << t << ", " << deltaT << ": ";
      p += deltaT * v;
      v += deltaT * scenario_.acceleration(t);
      const Vector3 measuredAcc = scenario_.accelerationInBody(t);
      result.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
      std::cout << " P:" << result.deltaPij().transpose();
      std::cout << " p:" << p.transpose();
      std::cout << " p0:" << (p + v0 * t).transpose();
      std::cout << std::endl;
    }

    return result;
  }

  // Predict mean
  Pose3 mean(const ImuFactor::PreintegratedMeasurements& integrated) {
    // TODO(frank): allow non-standard
    const imuBias::ConstantBias zeroBias;
    const Pose3 pose_i = Pose3::identity();
    const Vector3 vel_i = scenario_.velocity(0);
    const Vector3 omegaCoriolis = Vector3::Zero();
    const bool use2ndOrderCoriolis = true;
    const PoseVelocityBias prediction =
        integrated.predict(pose_i, vel_i, zeroBias, scenario_.gravity(),
                           omegaCoriolis, use2ndOrderCoriolis);
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
TEST(ScenarioRunner, Forward) {
  const double v = 2;  // m/s
  Scenario forward(Vector3::Zero(), Vector3(v, 0, 0));

  ScenarioRunner runner(forward);
  const double T = 1;  // seconds
  ImuFactor::PreintegratedMeasurements integrated = runner.integrate(T);
  EXPECT(assert_equal(forward.pose(T), runner.mean(integrated), 1e-9));
}

/* ************************************************************************* */
TEST(ScenarioRunner, Circle) {
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, omega = 6 * degree;
  Scenario circle(Vector3(0, 0, omega), Vector3(v, 0, 0), 0.01);

  ScenarioRunner runner(circle);
  const double T = 15;  // seconds
  ImuFactor::PreintegratedMeasurements integrated = runner.integrate(T);
  EXPECT(assert_equal(circle.pose(T), runner.mean(integrated), 0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
