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
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef imuBias::ConstantBias Bias;

Vector biasAcc1(Vector3(0.2, -0.1, 0));
Vector biasGyro1(Vector3(0.1, -0.3, -0.2));
Bias bias1(biasAcc1, biasGyro1);

Vector biasAcc2(Vector3(0.1, 0.2, 0.04));
Vector biasGyro2(Vector3(-0.002, 0.005, 0.03));
Bias bias2(biasAcc2, biasGyro2);

/* ************************************************************************* */
TEST(ImuBias, Constructor) {
  // Default Constructor
  Bias bias1;

  // Acc + Gyro Constructor
  Bias bias2(biasAcc2, biasGyro2);

  // Copy Constructor
  Bias bias3(bias2);
}

/* ************************************************************************* */
TEST(ImuBias, operatorSub) {
  Bias biasActual = -bias1;
  Bias biasExpected(-biasAcc1, -biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST(ImuBias, operatorAdd) {
  Bias biasActual = bias2 + bias1;
  Bias biasExpected(biasAcc2 + biasAcc1, biasGyro2 + biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST(ImuBias, operatorSubB) {
  Bias biasActual = bias2 - bias1;
  Bias biasExpected(biasAcc2 - biasAcc1, biasGyro2 - biasGyro1);
  EXPECT(assert_equal(biasExpected, biasActual));
}

/* ************************************************************************* */
TEST(ImuBias, Correct1) {
  Matrix aH1, aH2;
  const Vector3 measurement(1, 2, 3);
  std::function<Vector3(const Bias&, const Vector3&)> f =
      std::bind(&Bias::correctAccelerometer, std::placeholders::_1,
                std::placeholders::_2, boost::none, boost::none);
  bias1.correctAccelerometer(measurement, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(f, bias1, measurement), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, bias1, measurement), aH2));
}

/* ************************************************************************* */
TEST(ImuBias, Correct2) {
  Matrix aH1, aH2;
  const Vector3 measurement(1, 2, 3);
  std::function<Vector3(const Bias&, const Vector3&)> f =
      std::bind(&Bias::correctGyroscope, std::placeholders::_1,
                std::placeholders::_2, boost::none, boost::none);
  bias1.correctGyroscope(measurement, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(f, bias1, measurement), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, bias1, measurement), aH2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
