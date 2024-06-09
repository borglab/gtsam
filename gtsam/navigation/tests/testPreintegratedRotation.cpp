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

// Create params where x and y axes are exchanged.
static std::shared_ptr<PreintegratedRotationParams> paramsWithTransform() {
  auto p = std::make_shared<PreintegratedRotationParams>();
  p->setBodyPSensor({Rot3::Yaw(M_PI_2), {0, 0, 0}});
  return p;
}

//******************************************************************************
TEST(PreintegratedRotation, integrateGyroMeasurement) {
  // Example where IMU is identical to body frame, then omega is roll
  using namespace biased_x_rotation;
  auto p = std::make_shared<PreintegratedRotationParams>();
  PreintegratedRotation pim(p);

  // Check the value.
  Matrix3 H_bias;
  PreintegratedRotation::IncrementalRotation f{measuredOmega, deltaT,
                                               p->getBodyPSensor()};
  const Rot3 incrR = f(bias, H_bias);
  Rot3 expected = Rot3::Roll(omega * deltaT);
  EXPECT(assert_equal(expected, incrR, 1e-9));

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias));

  // Check value of deltaRij() after integration.
  Matrix3 F;
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT, F);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9));

  // Check that system matrix F is the first derivative of compose:
  EXPECT(assert_equal<Matrix3>(pim.deltaRij().inverse().AdjointMap(), F));

  // Make sure delRdelBiasOmega is H_bias after integration.
  EXPECT(assert_equal<Matrix3>(H_bias, pim.delRdelBiasOmega()));
}

//******************************************************************************
TEST(PreintegratedRotation, integrateGyroMeasurementWithTransform) {
  // Example where IMU is rotated, so measured omega indicates pitch.
  using namespace biased_x_rotation;
  auto p = paramsWithTransform();
  PreintegratedRotation pim(p);

  // Check the value.
  Matrix3 H_bias;
  PreintegratedRotation::IncrementalRotation f{measuredOmega, deltaT,
                                               p->getBodyPSensor()};
  Rot3 expected = Rot3::Pitch(omega * deltaT);
  EXPECT(assert_equal(expected, f(bias, H_bias), 1e-9));

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias));

  // Check value of deltaRij() after integration.
  Matrix3 F;
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT, F);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9));

  // Check that system matrix F is the first derivative of compose:
  EXPECT(assert_equal<Matrix3>(pim.deltaRij().inverse().AdjointMap(), F));

  // Make sure delRdelBiasOmega is H_bias after integration.
  EXPECT(assert_equal<Matrix3>(H_bias, pim.delRdelBiasOmega()));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
