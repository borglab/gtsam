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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/PreintegratedRotation.h>

#include <memory>

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

  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  const Matrix3 sane = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane, pim.delRdelBiasOmega()));
  // TODO(frank): the derivative H we compute in biascorrectedDeltaRij is not the *sane* one!
  // EXPECT(assert_equal(sane, H));

  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  const Matrix3 sane2 = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane2, pim.delRdelBiasOmega()));
}

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

  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  const Matrix3 sane = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane, pim.delRdelBiasOmega()));
  // TODO(frank): the derivative H we compute in biascorrectedDeltaRij is not the *sane* one!
  // EXPECT(assert_equal(sane, H));

  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  const Matrix3 sane2 = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane2, pim.delRdelBiasOmega()));
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

  auto g = [&](const Vector3& increment) {
    return pim.biascorrectedDeltaRij(increment, {});
  };
  const Matrix3 sane = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane, pim.delRdelBiasOmega()));
  // TODO(frank): the derivative H we compute in biascorrectedDeltaRij is not the *sane* one!
  // EXPECT(assert_equal(sane, H));

  // Let's integrate a second IMU measurement and check the Jacobian update:
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  const Matrix3 sane2 = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
  EXPECT(assert_equal(sane2, pim.delRdelBiasOmega()));

}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
