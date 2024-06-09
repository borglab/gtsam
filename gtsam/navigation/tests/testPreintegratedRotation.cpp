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
const double roll = 0.1;
const Vector3 measuredOmega(roll, 0, 0);
const Vector3 biasHat(0, 0, 0);
const double deltaT = 0.5;
}  // namespace simple_roll

//******************************************************************************
TEST(PreintegratedRotation, incrementalRotation) {
  using namespace simple_roll;

  // Check the value.
  Matrix3 D_incrR_integratedOmega;
  const Rot3 incrR = pim.incrementalRotation(measuredOmega, biasHat, deltaT,
                                             D_incrR_integratedOmega);
  Rot3 expected = Rot3::Roll(roll * deltaT);
  EXPECT(assert_equal(expected, incrR, 1e-9));

  // Lambda for numerical derivative:
  auto f = [&](const Vector3& x, const Vector3& y) {
    return pim.incrementalRotation(x, y, deltaT, {});
  };

  // NOTE(frank): these derivatives as computed by the function violate the
  // "Jacobian contract". We should refactor this. It's not clear that the
  // deltaT factor is actually understood in calling code.

  // Check derivative with respect to measuredOmega
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative21<Rot3, Vector3, Vector3>(f, measuredOmega, biasHat),
      deltaT * D_incrR_integratedOmega));

  // Check derivative with respect to biasHat
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative22<Rot3, Vector3, Vector3>(f, measuredOmega, biasHat),
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
const double roll = 0.1;
const Vector3 measuredOmega(roll, 0, 0);
const Vector3 biasHat(0, 0, 0);
const double deltaT = 0.5;
}  // namespace roll_in_rotated_frame

//******************************************************************************
TEST(PreintegratedRotation, incrementalRotationWithTransform) {
  using namespace roll_in_rotated_frame;

  // Check the value.
  Matrix3 D_incrR_integratedOmega;
  const Rot3 incrR = pim.incrementalRotation(measuredOmega, biasHat, deltaT,
                                             D_incrR_integratedOmega);
  Rot3 expected = Rot3::Pitch(roll * deltaT);
  EXPECT(assert_equal(expected, incrR, 1e-9));

  // Lambda for numerical derivative:
  auto f = [&](const Vector3& x, const Vector3& y) {
    return pim.incrementalRotation(x, y, deltaT, {});
  };

  // NOTE(frank): Here, once again, the derivatives are weird, as they do not
  // take the rotation into account. They *are* the derivatives of the rotated
  // omegas, but not the derivatives with respect to the function arguments.

  // Check derivative with respect to measuredOmega
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative21<Rot3, Vector3, Vector3>(f, measuredOmega, biasHat),
      deltaT * D_incrR_integratedOmega));

  // Check derivative with respect to biasHat
  EXPECT(assert_equal<Matrix3>(
      numericalDerivative22<Rot3, Vector3, Vector3>(f, measuredOmega, biasHat),
      -deltaT * D_incrR_integratedOmega));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
