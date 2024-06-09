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
namespace simple_roll {
auto p = std::make_shared<PreintegratedRotationParams>();
PreintegratedRotation pim(p);
const double omega = 0.1;
const Vector3 measuredOmega(omega, 0, 0);
const Vector3 bias(0, 0, 0);
const double deltaT = 0.5;
}  // namespace simple_roll

//******************************************************************************
TEST(PreintegratedRotation, IncrementalRotation) {
  using namespace simple_roll;

  // Check the value.
  Matrix3 H_bias;
  PreintegratedRotation::IncrementalRotation f{measuredOmega, deltaT,
                                               p->getBodyPSensor()};
  const Rot3 incrR = f(bias, H_bias);
  Rot3 expected = Rot3::Roll(omega * deltaT);
  EXPECT(assert_equal(expected, incrR, 1e-9));

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias));

  // Ephemeral test for deprecated Jacobian:
  Matrix3 D_incrR_integratedOmega;
  (void)pim.incrementalRotation(measuredOmega, bias, deltaT,
                                D_incrR_integratedOmega);
  auto g = [&](const Vector3& x, const Vector3& y) {
    return pim.incrementalRotation(x, y, deltaT, {});
  };
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative22<Rot3, Vector3, Vector3>(g, measuredOmega, bias),
      -deltaT * D_incrR_integratedOmega));
}

//******************************************************************************
static std::shared_ptr<PreintegratedRotationParams> paramsWithTransform() {
  auto p = std::make_shared<PreintegratedRotationParams>();
  p->setBodyPSensor({Rot3::Yaw(M_PI_2), {0, 0, 0}});
  return p;
}

namespace roll_in_rotated_frame {
auto p = paramsWithTransform();
PreintegratedRotation pim(p);
const double omega = 0.1;
const Vector3 measuredOmega(omega, 0, 0);
const Vector3 bias(0, 0, 0);
const double deltaT = 0.5;
}  // namespace roll_in_rotated_frame

//******************************************************************************
TEST(PreintegratedRotation, IncrementalRotationWithTransform) {
  using namespace roll_in_rotated_frame;

  // Check the value.
  Matrix3 H_bias;
  PreintegratedRotation::IncrementalRotation f{measuredOmega, deltaT,
                                               p->getBodyPSensor()};
  Rot3 expected = Rot3::Pitch(omega * deltaT);
  EXPECT(assert_equal(expected, f(bias, H_bias), 1e-9));

  // Check the derivative:
  EXPECT(assert_equal(numericalDerivative11<Rot3, Vector3>(f, bias), H_bias));

  // Ephemeral test for deprecated Jacobian:
  Matrix3 D_incrR_integratedOmega;
  (void)pim.incrementalRotation(measuredOmega, bias, deltaT,
                                D_incrR_integratedOmega);
  auto g = [&](const Vector3& x, const Vector3& y) {
    return pim.incrementalRotation(x, y, deltaT, {});
  };
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative22<Rot3, Vector3, Vector3>(g, measuredOmega, bias),
      -deltaT * D_incrR_integratedOmega));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
