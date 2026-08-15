/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPreintegratedRotation.cpp
 * @brief  Unit test for PreintegratedRotation
 * @author Frank Dellaert
 */

// GCC bug workaround
#if  defined(__GNUC__) && __GNUC__ == 16
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/PreintegratedRotation.h>

#include <initializer_list>
#include <memory>
#include <stdexcept>

#include "gtsam/base/Matrix.h"
#include "gtsam/base/Vector.h"

using namespace gtsam;

//******************************************************************************
// Example where gyro measures small rotation about x-axis, with bias.
namespace biased_x_rotation {
const double omega = 0.1;
const Vector3 trueOmega(omega, 0, 0);
const Vector3 bias(1, 2, 3);
const Vector3 measuredOmega = trueOmega + bias;
const double deltaT = 0.5;
}  // namespace biased_x_rotation

//******************************************************************************
TEST(PreintegratedRotation, integrateGyroMeasurement) {
  // Example where IMU is identical to body frame, then omega is roll
  using namespace biased_x_rotation;
  auto p = std::make_shared<PreintegratedRotationParams>();

  // Check the value.
  Matrix3 H_bias;
  const internal::IncrementalRotation f{measuredOmega, deltaT, p->getBodyPSensor()};
  const Rot3 incrR = f(bias, H_bias);
  const Rot3 expected = Rot3::Roll(omega * deltaT);
  EXPECT(assert_equal(expected, incrR, 1e-9))

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias))

  // Check value of deltaRij() after integration.
  Matrix3 F;
  PreintegratedRotation pim(p);
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT, F);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9))

  // Check that system matrix F is the first derivative of compose:
  EXPECT(assert_equal<Matrix3>(pim.deltaRij().inverse().AdjointMap(), F))

  // Make sure delRdelBiasOmega is H_bias after integration.
  EXPECT(assert_equal<Matrix3>(H_bias, pim.delRdelBiasOmega()))

  // Check if we make a correction to the bias, the value and Jacobian are
  // correct. Note that the bias is subtracted from the measurement, and the
  // integration time is taken into account, so we expect -deltaT*delta change.
  Matrix3 H;
  const double delta = 0.05;
  const Vector3 biasOmegaIncr(delta, 0, 0);
  Rot3 corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  EQUALITY(Vector3(-deltaT * delta, 0, 0), expected.logmap(corrected))
  EXPECT(assert_equal(Rot3::Roll((omega - delta) * deltaT), corrected, 1e-9))

  // Check the derivative matches the numerical one
  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  Matrix3 expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  EXPECT(assert_equal(expectedH, H));
  
  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  EXPECT(assert_equal(expectedH, H));
}

/* ************************************************************************* */
namespace gyro_integration_methods {

Vector times(const std::initializer_list<double>& values) {
  Vector result(values.size());
  Eigen::Index index = 0;
  for (const double value : values) result(index++) = value;
  return result;
}

Matrix omegas(const std::initializer_list<Vector3>& rows) {
  Matrix result(rows.size(), 3);
  Eigen::Index index = 0;
  for (const Vector3& row : rows) result.row(index++) = row.transpose();
  return result;
}

Matrix constantOmegas(const Vector3& omega, size_t count) {
  Matrix result(count, 3);
  for (size_t row = 0; row < count; ++row) {
    result.row(row) = omega.transpose();
  }
  return result;
}

// Checks constant angular velocity for both gyroscope integrators.
TEST(PreintegratedRotation, GyroIntegrationConstantOmega) {
  const Vector sampleTimes = times({0.0, 0.25, 0.5, 0.75, 1.0});
  const Vector3 omega{0.1, -0.2, 0.3};
  const Matrix measurements = constantOmegas(omega, sampleTimes.size());
  const Rot3 expected = Rot3::Expmap(omega);

  EXPECT(assert_equal(
      expected, integrateSequentialRotations(sampleTimes, measurements),
      1e-12));
  EXPECT(assert_equal(
      expected, integrateSingleSpeedConing(sampleTimes, measurements), 1e-12));
}

// Checks that both functions subtract the supplied gyroscope bias.
TEST(PreintegratedRotation, GyroIntegrationBiasCorrection) {
  const Vector sampleTimes = times({0.0, 0.5, 1.0});
  const Vector3 omega{0.15, 0.0, -0.05};
  const Vector3 bias{0.3, -0.4, 0.5};
  const Matrix measurements = constantOmegas(omega + bias, sampleTimes.size());
  const Rot3 expected = Rot3::Expmap(omega);

  EXPECT(assert_equal(
      expected, integrateSequentialRotations(sampleTimes, measurements, bias),
      1e-12));
  EXPECT(assert_equal(
      expected, integrateSingleSpeedConing(sampleTimes, measurements, bias),
      1e-12));
}

// Checks that samples are rotated from the sensor frame into the body frame.
TEST(PreintegratedRotation, GyroIntegrationSensorRotation) {
  const Vector sampleTimes = times({0.0, 0.5, 1.0});
  const Rot3 body_R_sensor = Rot3::Yaw(M_PI_2);
  const Vector3 sensorOmega{0.2, 0.0, 0.0};
  const Vector3 bodyOmega = body_R_sensor * sensorOmega;
  const Matrix measurements =
      constantOmegas(sensorOmega, sampleTimes.size());
  const Rot3 expected = Rot3::Expmap(bodyOmega);

  EXPECT(assert_equal(expected,
                      integrateSequentialRotations(
                          sampleTimes, measurements, Vector3::Zero(),
                          body_R_sensor),
                      1e-12));
  EXPECT(assert_equal(expected,
                      integrateSingleSpeedConing(
                          sampleTimes, measurements, Vector3::Zero(),
                          body_R_sensor),
                      1e-12));
}

// Checks sequential rotations against explicit trapezoidal composition.
TEST(PreintegratedRotation, GyroIntegrationSequentialRotations) {
  const Vector sampleTimes = times({0.0, 1.0, 2.0});
  const Matrix measurements =
      omegas({Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
              Vector3{0.0, 0.0, 1.0}});
  const Vector3 theta0{0.5, 0.5, 0.0};
  const Vector3 theta1{0.0, 0.5, 0.5};
  const Rot3 expected =
      Rot3::Expmap(theta0).compose(Rot3::Expmap(theta1));

  EXPECT(assert_equal(
      expected, integrateSequentialRotations(sampleTimes, measurements),
      1e-12));
}

// Checks the single-speed coning correction against its defining formula.
TEST(PreintegratedRotation, GyroIntegrationSingleSpeedConing) {
  const Vector sampleTimes = times({0.0, 1.0, 2.0});
  const Matrix measurements =
      omegas({Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
              Vector3{0.0, 0.0, 1.0}});
  const Vector3 theta0{0.5, 0.5, 0.0};
  const Vector3 theta1{0.0, 0.5, 0.5};
  const Vector3 correctedTheta1 =
      theta1 + (1.0 / 12.0) * theta0.cross(theta1);
  const Rot3 expected =
      Rot3::Expmap(theta0).compose(Rot3::Expmap(correctedTheta1));

  EXPECT(assert_equal(
      expected, integrateSingleSpeedConing(sampleTimes, measurements), 1e-12));
}

// Checks that invalid sample layouts and timestamps are rejected.
TEST(PreintegratedRotation, GyroIntegrationInvalidInputs) {
  CHECK_EXCEPTION(
      integrateSequentialRotations(times({0.0}), Matrix::Zero(1, 3)),
      std::invalid_argument);
  CHECK_EXCEPTION(integrateSequentialRotations(times({0.0, 1.0}),
                                               Matrix::Zero(2, 2)),
                  std::invalid_argument);
  CHECK_EXCEPTION(integrateSequentialRotations(times({0.0, 0.0}),
                                               Matrix::Zero(2, 3)),
                  std::invalid_argument);
  CHECK_EXCEPTION(integrateSingleSpeedConing(times({0.0, 1.0, 0.5}),
                                             Matrix::Zero(3, 3)),
                  std::invalid_argument);
}

}  // namespace gyro_integration_methods
/* ************************************************************************* */

//******************************************************************************

// Create params where x and y axes are exchanged.
static std::shared_ptr<PreintegratedRotationParams> paramsWithTransform() {
  auto p = std::make_shared<PreintegratedRotationParams>();
  p->setBodyPSensor({Rot3::Yaw(M_PI_2), {0, 0, 0}});
  return p;
}

TEST(PreintegratedRotation, integrateGyroMeasurementWithTransform) {
  // Example where IMU is rotated, so measured omega indicates pitch.
  using namespace biased_x_rotation;
  auto p = paramsWithTransform();

  // Check the value.
  Matrix3 H_bias;
  const internal::IncrementalRotation f{measuredOmega, deltaT, p->getBodyPSensor()};
  const Rot3 expected = Rot3::Pitch(omega * deltaT); // Pitch, because of sensor-IMU rotation!
  EXPECT(assert_equal(expected, f(bias, H_bias), 1e-9))

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias))

  // Check value of deltaRij() after integration.
  Matrix3 F;
  PreintegratedRotation pim(p);
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT, F);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9))

  // Check that system matrix F is the first derivative of compose:
  EXPECT(assert_equal<Matrix3>(pim.deltaRij().inverse().AdjointMap(), F))

  // Make sure delRdelBiasOmega is H_bias after integration.
  EXPECT(assert_equal<Matrix3>(H_bias, pim.delRdelBiasOmega()))

  // Check the bias correction in same way, but will now yield pitch change.
  Matrix3 H;
  const double delta = 0.05;
  const Vector3 biasOmegaIncr(delta, 0, 0);
  Rot3 corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  EQUALITY(Vector3(0, -deltaT * delta, 0), expected.logmap(corrected))
  EXPECT(assert_equal(Rot3::Pitch((omega - delta) * deltaT), corrected, 1e-9))

  // Check the derivative matches the *expectedH* one
  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  Matrix3 expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  EXPECT(assert_equal(expectedH, H));

  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  EXPECT(assert_equal(expectedH, H));
}

// Create params we have a non-axis-aligned rotation and even an offset.
static std::shared_ptr<PreintegratedRotationParams> paramsWithArbitraryTransform() {
  auto p = std::make_shared<PreintegratedRotationParams>();
  p->setBodyPSensor({Rot3::Expmap({1,2,3}), {4,5,6}});
  return p;
}

TEST(PreintegratedRotation, integrateGyroMeasurementWithArbitraryTransform) {
  // Example with a non-axis-aligned transform and some position.
  using namespace biased_x_rotation;
  auto p = paramsWithArbitraryTransform();

  // Check the derivative:
  Matrix3 H_bias;
  const internal::IncrementalRotation f{measuredOmega, deltaT, p->getBodyPSensor()};
  f(bias, H_bias);
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias))

  // Check derivative of deltaRij() after integration.
  Matrix3 F;
  PreintegratedRotation pim(p);
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT, F);

  // Check that system matrix F is the first derivative of compose:
  EXPECT(assert_equal<Matrix3>(pim.deltaRij().inverse().AdjointMap(), F))

  // Make sure delRdelBiasOmega is H_bias after integration.
  EXPECT(assert_equal<Matrix3>(H_bias, pim.delRdelBiasOmega()))

  // Check the bias correction in same way, but will now yield pitch change.
  Matrix3 H;
  const double delta = 0.05;
  const Vector3 biasOmegaIncr(delta, 0, 0);
  Rot3 corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);

  // Check the derivative matches the numerical one
  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  Matrix3 expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  EXPECT(assert_equal(expectedH, H));

  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  expectedH = numericalDerivative11<Rot3, Vector3>(g, biasOmegaIncr);
  EXPECT(assert_equal(expectedH, H));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
