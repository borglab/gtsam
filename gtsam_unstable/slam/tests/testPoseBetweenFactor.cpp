/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testProjectionFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2009
 */

#include <gtsam_unstable/slam/PoseBetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef PoseBetweenFactor<Pose3> TestPoseBetweenFactor;

/// traits
namespace gtsam {
template<>
struct traits<TestPoseBetweenFactor> : public Testable<TestPoseBetweenFactor> {
};
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, Constructor) {
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model);
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, ConstructorWithTransform) {
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model, body_P_sensor);
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPoseBetweenFactor factor1(poseKey1, poseKey2, measurement, model);
  TestPoseBetweenFactor factor2(poseKey1, poseKey2, measurement, model);

  CHECK(assert_equal(factor1, factor2));

  // Create a third, different factor and check for inequality
  Pose3 measurement2(Rot3::RzRyRx(0.20, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  TestPoseBetweenFactor factor3(poseKey1, poseKey2, measurement2, model);

  CHECK(assert_inequal(factor1, factor3));
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPoseBetweenFactor factor1(poseKey1, poseKey2, measurement, model, body_P_sensor);
  TestPoseBetweenFactor factor2(poseKey1, poseKey2, measurement, model, body_P_sensor);

  CHECK(assert_equal(factor1, factor2));

  // Create a third, different factor and check for inequality
  Pose3 body_P_sensor2(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.30, -0.10, 1.0));
  TestPoseBetweenFactor factor3(poseKey1, poseKey2, measurement, model, body_P_sensor2);

  CHECK(assert_inequal(factor1, factor3));
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, Error ) {
  // Create the measurement and linearization point
  Pose3 measurement(Rot3::RzRyRx(0.15, 0.15, -0.20), Point3(+0.5, -1.0, +1.0));
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(Rot3::RzRyRx(0.15,  0.00, 0.20), Point3(-3.5, 6.0,  -9.0));

  // The expected error
  Vector expectedError(6);
  // The solution depends on choice of Pose3 and Rot3 Expmap mode!
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  expectedError << -0.0298135267953815,
                    0.0131341515747393,
                    0.0968868439682154,
#if defined(GTSAM_POSE3_EXPMAP)
                   -0.145701634472172,
                   -0.134898525569125,
                   -0.0421026389164264;
#else
                   -0.13918755,
                   -0.142346243,
                   -0.0390885321;
#endif
#else
  expectedError << -0.029839512616488,
                    0.013145599455949,
                    0.096971291682578,
                   -0.139187549519821,
                   -0.142346243063553,
                   -0.039088532100977;
#endif

  // Create a factor and calculate the error
  Key poseKey1(1);
  Key poseKey2(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model);
  Vector actualError(factor.evaluateError(pose1, pose2));

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, ErrorWithTransform ) {
  // Create the measurement and linearization point
  Pose3 measurement(Rot3::RzRyRx(-0.15, 0.10, 0.15), Point3(+1.25, -0.90, +.45));
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(Rot3::RzRyRx(0.15,  0.00, 0.20), Point3(-3.5, 6.0,  -9.0));
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  // The expected error
  Vector expectedError(6);
  // The solution depends on choice of Pose3 and Rot3 Expmap mode!
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  expectedError <<    0.0173358202010741,
                      0.0222210698409755,
                     -0.0125032003886145,
#if defined(GTSAM_POSE3_EXPMAP)
                      0.0263800787416566,
                      0.00540285006310398,
                      0.000175859555693563;
#else
                      0.0264132886,
                      0.0052376953,
                      -7.16127036e-05;
#endif
#else
  expectedError <<    0.017337193670445,
                      0.022222830355243,
                     -0.012504190982804,
                      0.026413288603739,
                      0.005237695303536,
                     -0.000071612703633;
#endif


  // Create a factor and calculate the error
  Key poseKey1(1);
  Key poseKey2(2);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model, body_P_sensor);
  Vector actualError(factor.evaluateError(pose1, pose2));

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, Jacobian ) {
  // Create a factor
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(0.15, 0.15, -0.20), Point3(+0.5, -1.0, +1.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model);

  // Create a linearization point at the zero-error point
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(Rot3::RzRyRx(0.179693265735950, 0.002945368776519, 0.102274823253840),
              Point3(-3.37493895, 6.14660244, -8.93650986));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor, &pose2](const Pose3& p) { return factor.evaluateError(p, pose2); }, pose1);
  Matrix expectedH2 = numericalDerivative11<Vector, Pose3>(
		  [&factor, &pose1](const Pose3& p) { return factor.evaluateError(pose1, p); }, pose2);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  Matrix actualH2;
  factor.evaluateError(pose1, pose2, actualH1, actualH2);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
  CHECK(assert_equal(expectedH2, actualH2, 1e-6));
}

/* ************************************************************************* */
TEST( PoseBetweenFactor, JacobianWithTransform ) {
  // Create a factor
  Key poseKey1(1);
  Key poseKey2(2);
  Pose3 measurement(Rot3::RzRyRx(-0.15, 0.10, 0.15), Point3(+1.25, -0.90, +.45));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPoseBetweenFactor factor(poseKey1, poseKey2, measurement, model, body_P_sensor);

  // Create a linearization point at the zero-error point
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(Rot3::RzRyRx(0.162672458989103, 0.013665177349534, 0.224649482928184),
              Point3(-3.5257579, 6.02637531, -8.98382384));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor, &pose2](const Pose3& p) { return factor.evaluateError(p, pose2); }, pose1);
  Matrix expectedH2 = numericalDerivative11<Vector, Pose3>(
		  [&factor, &pose1](const Pose3& p) { return factor.evaluateError(pose1, p); }, pose2);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  Matrix actualH2;
  Vector error = factor.evaluateError(pose1, pose2, actualH1, actualH2);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-6));
  CHECK(assert_equal(expectedH2, actualH2, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

