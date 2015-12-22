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

#pragma once
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
    for (size_t k = 0; k < nrSteps; k++, t += deltaT) {
      const Vector3 measuredAcc = scenario_.accelerationInBody(t);
      result.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
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

