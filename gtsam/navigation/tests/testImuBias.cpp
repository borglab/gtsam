/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor.cpp
 * @brief   Unit test for the InertialNavFactor
 * @author  Vadim Indelman, Stephen Williams
 */

#include <gtsam/navigation/ImuBias.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

Vector biasAcc1(Vector3(0.2, -0.1, 0));
Vector biasGyro1(Vector3(0.1, -0.3, -0.2));
imuBias::ConstantBias bias1(biasAcc1, biasGyro1);

Vector biasAcc2(Vector3(0.1, 0.2, 0.04));
Vector biasGyro2(Vector3(-0.002, 0.005, 0.03));
imuBias::ConstantBias bias2(biasAcc2, biasGyro2);

/* ************************************************************************* */
TEST( ImuBias, Constructor) {
  // Default Constructor
  imuBias::ConstantBias bias1;

  // Acc + Gyro Constructor
  imuBias::ConstantBias bias2(biasAcc2, biasGyro2);

  // Copy Constructor
  imuBias::ConstantBias bias3(bias2);
}

/* ************************************************************************* */
TEST( ImuBias, inverse) {
  imuBias::ConstantBias biasActual = bias1.inverse();
  imuBias::ConstantBias biasExpected = imuBias::ConstantBias(-biasAcc1,
      -biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, compose) {
  imuBias::ConstantBias biasActual = bias2.compose(bias1);
  imuBias::ConstantBias biasExpected = imuBias::ConstantBias(
      biasAcc1 + biasAcc2, biasGyro1 + biasGyro2);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, between) {
  // p.between(q) == q - p
  imuBias::ConstantBias biasActual = bias2.between(bias1);
  imuBias::ConstantBias biasExpected = imuBias::ConstantBias(
      biasAcc1 - biasAcc2, biasGyro1 - biasGyro2);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, localCoordinates) {
  Vector deltaActual = Vector(bias2.localCoordinates(bias1));
  Vector deltaExpected = (imuBias::ConstantBias(biasAcc1 - biasAcc2,
      biasGyro1 - biasGyro2)).vector();
  EXPECT(assert_equal(deltaExpected, deltaActual));
}

/* ************************************************************************* */
TEST( ImuBias, retract) {
  Vector6 delta;
  delta << 0.1, 0.2, -0.3, 0.1, -0.1, 0.2;
  imuBias::ConstantBias biasActual = bias2.retract(delta);
  imuBias::ConstantBias biasExpected = imuBias::ConstantBias(
      biasAcc2 + delta.block<3, 1>(0, 0), biasGyro2 + delta.block<3, 1>(3, 0));
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, Logmap) {
  Vector deltaActual = bias2.Logmap(bias1);
  Vector deltaExpected = bias1.vector();
  EXPECT(assert_equal(deltaExpected, deltaActual));
}

/* ************************************************************************* */
TEST( ImuBias, Expmap) {
  Vector6 delta;
  delta << 0.1, 0.2, -0.3, 0.1, -0.1, 0.2;
  imuBias::ConstantBias biasActual = bias2.Expmap(delta);
  imuBias::ConstantBias biasExpected = imuBias::ConstantBias(delta);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, operatorSub) {
  imuBias::ConstantBias biasActual = -bias1;
  imuBias::ConstantBias biasExpected(-biasAcc1, -biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, operatorAdd) {
  imuBias::ConstantBias biasActual = bias2 + bias1;
  imuBias::ConstantBias biasExpected(biasAcc2 + biasAcc1,
      biasGyro2 + biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST( ImuBias, operatorSubB) {
  imuBias::ConstantBias biasActual = bias2 - bias1;
  imuBias::ConstantBias biasExpected(biasAcc2 - biasAcc1,
      biasGyro2 - biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
