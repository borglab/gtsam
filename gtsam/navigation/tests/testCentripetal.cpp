/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCentripetal.cpp
 * @brief   Unit test for centripetal accelerations on PreintegratedBase
 * @author  Scott Baker
 * @date   10/6/2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/Scenario.h>

#include "imuFactorTesting.h"

using namespace gtsam;

/* ************************************************************************* */
// For offset IMUs, acceleration is given by
//   fb + omega × (lever arm) + (omega × (omega × lever arm))
//   = fb +  doublecross(w0, lever arm)
// We then can rotate by body -> sensor frame to ensure it is in the sensor
// measurement Angular velocity is also rotated by body -> sensor frame
static std::pair<Vector3, Vector3> computeSensorMeasurements(
    const Vector3& fb, const Vector3& wb, const Pose3& bTs,
    const imuBias::ConstantBias& bias = imuBias::ConstantBias()) {
  // Force on sensor as if in the same orientation as body
  const Vector3 f_b_sensor = fb + gtsam::doubleCross(wb, bTs.translation());

  // Find body -> sensor frame given by sRb = bRs^T
  const Rot3 sRb = bTs.rotation().inverse();

  // Rotate forces and angular velocities to sensor frame
  const Vector3 f_sensor = sRb * f_b_sensor + bias.accelerometer();
  const Vector3 w_sensor = sRb * wb + bias.gyroscope();

  return std::make_pair(f_sensor, w_sensor);
}

/* ************************************************************************* */
namespace gravity {
const double g = 9.81;
const Vector3 g_n = Vector3(0, 0, -g);
}  // namespace gravity

/* ************************************************************************* */
/// Simple helper function that integrates a single IMU
template <typename PIM>
static NavState integrate(const ConstantTwistScenario& scenario,
                          const Pose3& bTs, const imuBias::ConstantBias& bias) {
  using namespace gravity;

  // Create preintegration parameters for the IMU
  auto p = PreintegrationParams::MakeSharedU(g);
  p->setBodyPSensor(bTs);

  PIM pim(p, bias);

  // Integrate for 5 seconds at 10 Hz
  const double dt = 0.1;
  const double T = 5.0;
  for (int k = 0; k * dt < T; k++) {
    const double t = k * dt;

    // Force at body frame = a - g_b
    const Vector3 fb =
        scenario.acceleration_b(t) - scenario.rotation(t).unrotate(g_n);
    const Vector3 wb = scenario.omega_b(t);

    // Compute sensor measurements using the helper function
    auto [f_sensor, w_sensor] = computeSensorMeasurements(fb, wb, bTs, bias);

    // Integrate IMU
    pim.integrateMeasurement(f_sensor, w_sensor, dt);
  }

  // Predict the final state
  return pim.predict(NavState(), bias);
}

/* ************************************************************************* */
namespace car_in_circle {
// We set up a scenario where a car is driving in a 10m circle at 10 m/s with
// a yaw rate of 1 rad/s
const double v = 10, w = 1;
const Vector3 W(0, 0, w), V(v, 0, 0);
const ConstantTwistScenario scenario(W, V);
}  // namespace car_in_circle

/* ************************************************************************* */
namespace aligned_imus {
// Set up the three IMUs; imu0 at body frame, imu1 3m left of body frame, imu2
// 3m right in same orientation Using FRD; right +3m, left -3m
const double b = 3.0;                        // Lever arm in m
const Pose3 bTs_imu0;                        // IMU at body frame
const Pose3 bTs_imu1({}, Point3(0, -b, 0));  // IMU 1 3m left of body frame
const Pose3 bTs_imu2({}, Point3(0, b, 0));   // IMU 2 3m right of body
}  // namespace aligned_imus

/* ************************************************************************* */
namespace rotated_imus {
// Set up the three IMUs; imu0 at body frame, imu1 3m left of body frame, imu2
// 3m right. Set IMU on left to be rotated on yaw by -90, IMU on right by +90
const double b = 3.0;  // Lever arm in m
const Pose3 bTs_imu0;  // IMU at body frame
const Pose3 bTs_imu1(Rot3::Rz(-M_PI_2),
                     Point3(0, -b, 0));  // IMU 1 3m left of body frame
const Pose3 bTs_imu2(Rot3::Rz(M_PI_2),
                     Point3(0, b, 0));  // IMU 2 3m right of body frame
}  // namespace rotated_imus

/* ************************************************************************* */
TEST_PIM(CentripetalCompensation, Circle3IMUsSameOrientation) {
  using namespace aligned_imus;
  using namespace car_in_circle;

  const imuBias::ConstantBias zero;  // zero bias

  // Integrate imu0 to get state0
  auto state0 = integrate<PIM>(scenario, bTs_imu0, zero);

  // Integrate imu1 and imu2 should yield the same result
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu1, zero), 1e-6));
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu2, zero), 1e-6));
}

/* ************************************************************************* */
TEST_PIM(CentripetalCompensation, Circle3IMUsRotated) {
  using namespace rotated_imus;
  using namespace car_in_circle;

  const imuBias::ConstantBias zero;  // zero bias

  // Integrate imu0 to get state0
  auto state0 = integrate<PIM>(scenario, bTs_imu0, zero);

  // Integrate imu1 and imu2 should yield the same result
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu1, zero), 1e-6));
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu2, zero), 1e-6));
}
/* ************************************************************************* */
TEST_PIM(CentripetalCompensation, Circle3BiasedIMUsSameOrientation) {
  using namespace aligned_imus;
  using namespace car_in_circle;

  // Define IMU biases in sensor frame
  const Vector3 gyro_bias_s(0.1, 0.2, 0.3);
  const Vector3 accel_bias_s(0.1, 0.2, 0.3);
  const imuBias::ConstantBias bias(accel_bias_s, gyro_bias_s);

  // Integrate imu0 to get state0
  auto state0 = integrate<PIM>(scenario, bTs_imu0, bias);

  // Integrate imu1 and imu2 should yield the same result
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu1, bias), 1e-6));
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu2, bias), 1e-6));
}

/* ************************************************************************* */
TEST_PIM(CentripetalCompensation, Circle3BiasedIMUsRotated) {
  using namespace rotated_imus;
  using namespace car_in_circle;

  // Define IMU biases in sensor frame
  const Vector3 gyro_bias_s(0.1, 0.2, 0.3);
  const Vector3 accel_bias_s(0.1, 0.2, 0.3);
  const imuBias::ConstantBias bias(accel_bias_s, gyro_bias_s);

  // Integrate imu0 to get state0
  auto state0 = integrate<PIM>(scenario, bTs_imu0, bias);

  // Integrate imu1 and imu2 should yield the same result
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu1, bias), 1e-6));
  EXPECT(assert_equal(state0, integrate<PIM>(scenario, bTs_imu2, bias), 1e-6));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************