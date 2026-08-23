/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ClockRecoveryContinuousTrajectory.cpp
 * @brief Sketch clock-offset estimation with a continuous pose trajectory.
 * @author Brett Downing
 */

#include <gtsam/basis/CumulativeSplineTrajectory.h>
#include <gtsam/basis/IrwinHall.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>

#include <cmath>
#include <iostream>

using namespace gtsam;

namespace {

struct ShutterEvent {
  Key cameraPose;
  double timestamp;
};

struct AccelerationMeasurement {
  Vector3 acceleration;
  double timestamp;
};

struct AngularRateMeasurement {
  Vector3 angularVelocity;
  double timestamp;
};

ShutterEvent sampleCamera() { return {Symbol('x', 0), 4.95}; }

AccelerationMeasurement sampleAccelerometer() {
  return {Vector3(0.0, 0.0, 9.81), 5.02};
}

AngularRateMeasurement sampleGyroscope() { return {Vector3::Zero(), 5.01}; }

Vector3 rotationComponentValue(const Vector6& derivative,
                               OptionalJacobian<3, 6> H = {}) {
  if (H) *H << Matrix3::Identity(), Matrix3::Zero();
  return derivative.head<3>();
}

Vector3 translationComponentValue(const Vector6& derivative,
                                  OptionalJacobian<3, 6> H = {}) {
  if (H) *H << Matrix3::Zero(), Matrix3::Identity();
  return derivative.tail<3>();
}

Vector3_ rotationComponent(const Vector6_& derivative) {
  return Vector3_(&rotationComponentValue, derivative);
}

Vector3_ translationComponent(const Vector6_& derivative) {
  return Vector3_(&translationComponentValue, derivative);
}

Vector6 applyAdjoint(const Pose3& pose, const Vector6& tangent,
                     OptionalJacobian<6, 6> Hpose = {},
                     OptionalJacobian<6, 6> Htangent = {}) {
  return pose.Adjoint(tangent, Hpose, Htangent);
}

Vector6_ adjoint(const Pose3_& pose, const Vector6_& tangent) {
  return Vector6_(&applyAdjoint, pose, tangent);
}

enum class TimeDerivative : size_t {
  kVelocity = 1,
  kAcceleration = 2,
  kJerk = 3,
  kSnap = 4,
  kCrackle = 5,
  kPop = 6,
};

size_t derivativeOrder(TimeDerivative derivative) {
  return static_cast<size_t>(derivative);
}

}  // namespace

int main() {
  Values initialValues;
  NonlinearFactorGraph graph;

  constexpr double trajectorySampleRate = 20.0;
  constexpr double clockDriftSampleRate = 0.1;
  constexpr double maximumTimestamp = 10.0;

  CumulativeSplineTrajectory<Pose3> trajectoryModel(trajectorySampleRate,
                                                    kernels::IrwinHallCDF2);
  CumulativeSplineTrajectory<double> accelerometerClockModel(
      clockDriftSampleRate, kernels::IrwinHallCDF1);
  CumulativeSplineTrajectory<double> gyroscopeClockModel(
      clockDriftSampleRate, kernels::IrwinHallCDF1);

  const Pose3_ accelerometerPose{Pose3()};
  const Pose3_ gyroscopePose{Pose3()};
  const Vector3_ gravity(Vector3(0.0, 0.0, 9.81));

  const Key accelerometerBiasKey = Symbol('b', 0);
  const Key gyroscopeBiasKey = Symbol('b', 1);
  const Vector3_ accelerometerBias(accelerometerBiasKey);
  const Vector3_ gyroscopeBias(gyroscopeBiasKey);
  initialValues.insert(accelerometerBiasKey, Vector3(Vector3::Zero()));
  initialValues.insert(gyroscopeBiasKey, Vector3(Vector3::Zero()));

  const size_t poseCount =
      static_cast<size_t>(std::ceil(maximumTimestamp * trajectorySampleRate) +
                          trajectoryModel.kernel().getLength());
  for (size_t index = 0; index < poseCount; ++index) {
    const Key poseKey = Symbol('p', index);
    trajectoryModel.addControlPoint(Pose3_(poseKey));
    initialValues.insert(poseKey, Pose3());
  }

  const size_t clockPointCount =
      static_cast<size_t>(std::ceil(maximumTimestamp * clockDriftSampleRate) +
                          accelerometerClockModel.kernel().getLength());
  for (size_t index = 0; index < clockPointCount; ++index) {
    const Key accelerometerClockKey = Symbol('a', index);
    const Key gyroscopeClockKey = Symbol('g', index);
    initialValues.insert(accelerometerClockKey, 0.0);
    initialValues.insert(gyroscopeClockKey, 0.0);
    accelerometerClockModel.addControlPoint(Double_(accelerometerClockKey));
    gyroscopeClockModel.addControlPoint(Double_(gyroscopeClockKey));
  }

  const ShutterEvent shutterEvent = sampleCamera();
  const AccelerationMeasurement acceleration = sampleAccelerometer();
  const AngularRateMeasurement angularRate = sampleGyroscope();
  initialValues.insert(shutterEvent.cameraPose, Pose3());

  const Double_ gyroscopeClockOffset =
      gyroscopeClockModel.sampleTrajectory(Double_(angularRate.timestamp));
  const Double_ accelerometerClockOffset =
      accelerometerClockModel.sampleTrajectory(Double_(acceleration.timestamp));

  const Double_ cameraTime(shutterEvent.timestamp);
  const Double_ gyroscopeTime =
      Double_(angularRate.timestamp) - gyroscopeClockOffset;
  const Double_ accelerometerTime =
      Double_(acceleration.timestamp) - accelerometerClockOffset;

  constexpr double windowStart = 4.5;
  constexpr double windowEnd = 5.5;
  const Pose3_ cameraPose =
      trajectoryModel.sampleTrajectory(cameraTime, windowStart, windowEnd);
  const Vector6_ cameraVelocity = trajectoryModel.sampleTrajectoryDerivative(
      gyroscopeTime, windowStart, windowEnd,
      derivativeOrder(TimeDerivative::kVelocity));
  const Vector6_ cameraAcceleration =
      trajectoryModel.sampleTrajectoryDerivative(
          accelerometerTime, windowStart, windowEnd,
          derivativeOrder(TimeDerivative::kAcceleration));

  const Vector6_ gyroscopeVelocity = adjoint(gyroscopePose, cameraVelocity);
  const Vector6_ accelerometerAcceleration =
      adjoint(accelerometerPose, cameraAcceleration);

  const auto poseNoise = noiseModel::Isotropic::Sigma(6, 0.1);
  const auto imuNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  const auto smoothnessNoise = noiseModel::Isotropic::Sigma(6, 1.0);

  graph.emplace_shared<ExpressionFactor<Pose3>>(
      poseNoise, Pose3(), between(cameraPose, Pose3_(shutterEvent.cameraPose)));
  graph.emplace_shared<ExpressionFactor<Vector3>>(
      imuNoise, angularRate.angularVelocity,
      rotationComponent(gyroscopeVelocity) + gyroscopeBias);
  graph.emplace_shared<ExpressionFactor<Vector3>>(
      imuNoise, acceleration.acceleration,
      translationComponent(accelerometerAcceleration) + gravity +
          accelerometerBias);

  const Double_ smoothnessTime(5.0);
  graph.emplace_shared<ExpressionFactor<Vector6>>(
      smoothnessNoise, Vector6::Zero(),
      trajectoryModel.sampleTrajectoryDerivative(
          smoothnessTime, windowStart, windowEnd,
          derivativeOrder(TimeDerivative::kCrackle)));

  std::cout << "Constructed " << graph.size() << " factors over "
            << initialValues.size() << " initial values.\n";
  return 0;
}
