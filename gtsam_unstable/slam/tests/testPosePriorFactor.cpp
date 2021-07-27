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

#include <gtsam_unstable/slam/PosePriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef PosePriorFactor<Pose3> TestPosePriorFactor;

/// traits
namespace gtsam {
template<>
struct traits<TestPosePriorFactor> : public Testable<TestPosePriorFactor> {
};
}

/* ************************************************************************* */
TEST( PosePriorFactor, Constructor) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPosePriorFactor factor(poseKey, measurement, model);
}

/* ************************************************************************* */
TEST( PosePriorFactor, ConstructorWithTransform) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPosePriorFactor factor(poseKey, measurement, model, body_P_sensor);
}

/* ************************************************************************* */
TEST( PosePriorFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPosePriorFactor factor1(poseKey, measurement, model);
  TestPosePriorFactor factor2(poseKey, measurement, model);

  CHECK(assert_equal(factor1, factor2));

  // Create a third, different factor and check for inequality
  Pose3 measurement2(Rot3::RzRyRx(0.20, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  TestPosePriorFactor factor3(poseKey, measurement2, model);

  CHECK(assert_inequal(factor1, factor3));
}

/* ************************************************************************* */
TEST( PosePriorFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPosePriorFactor factor1(poseKey, measurement, model, body_P_sensor);
  TestPosePriorFactor factor2(poseKey, measurement, model, body_P_sensor);

  CHECK(assert_equal(factor1, factor2));

  // Create a third, different factor and check for inequality
  Pose3 body_P_sensor2(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.30, -0.10, 1.0));
  TestPosePriorFactor factor3(poseKey, measurement, model, body_P_sensor2);

  CHECK(assert_inequal(factor1, factor3));
}

/* ************************************************************************* */
TEST( PosePriorFactor, Error ) {
  // Create the measurement and linearization point
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  Pose3 pose(Rot3::RzRyRx(0.0, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));

  // The expected error
  Vector expectedError(6);
  // The solution depends on choice of Pose3 and Rot3 Expmap mode!
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  expectedError << -0.182948257976108,
                    0.13851858011118,
                   -0.157375974517456,
#if defined(GTSAM_POSE3_EXPMAP)
                    0.766913166076379,
                   -1.22976117053126,
                    0.949345561430261;
#else
                    0.740211734,
                   -1.19821028,
                    1.00815609;
#endif
#else
  expectedError << -0.184137861505414,
                    0.139419283914526,
                   -0.158399296722242,
                    0.740211733817804,
                   -1.198210282560946,
                    1.008156093015192;
#endif


  // Create a factor and calculate the error
  Key poseKey(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPosePriorFactor factor(poseKey, measurement, model);
  Vector actualError(factor.evaluateError(pose));

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-8));
}

/* ************************************************************************* */
TEST( PosePriorFactor, ErrorWithTransform ) {
  // Create the measurement and linearization point
  Pose3 measurement(Rot3::RzRyRx(-M_PI_2+0.15, -0.30, -M_PI_2+0.45), Point3(-4.75, 7.90, -10.0));
  Pose3 pose(Rot3::RzRyRx(0.0, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  // The expected error
  Vector expectedError(6);
  // The solution depends on choice of Pose3 and Rot3 Expmap mode!
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  expectedError << -0.0224998729281528,
                    0.191947887288328,
                    0.273826035236257,
#if defined(GTSAM_POSE3_EXPMAP)
                    1.36483391560855,
                   -0.754590051075035,
                    0.585710674473659;
#else
                    1.49751986,
                   -0.549375791,
                    0.452761203;
#endif
#else
  expectedError << -0.022712885347328,
                    0.193765110165872,
                    0.276418420819283,
                    1.497519863757366,
                   -0.549375791422721,
                    0.452761203187666;
#endif

  // Create a factor and calculate the error
  Key poseKey(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPosePriorFactor factor(poseKey, measurement, model, body_P_sensor);
  Vector actualError(factor.evaluateError(pose));

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-8));
}

/* ************************************************************************* */
TEST( PosePriorFactor, Jacobian ) {
  // Create a factor
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  TestPosePriorFactor factor(poseKey, measurement, model);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      std::bind(&TestPosePriorFactor::evaluateError, &factor,
                std::placeholders::_1, boost::none),
      pose);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST( PosePriorFactor, JacobianWithTransform ) {
  // Create a factor
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(-M_PI_2+0.15, -0.30, -M_PI_2+0.45), Point3(-4.75, 7.90, -10.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 0.25);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestPosePriorFactor factor(poseKey, measurement, model, body_P_sensor);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(-0.303202977317447, -0.143253159173011, 0.494633847678171),
             Point3(-4.74767676, 7.67044942, -11.00985));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      std::bind(&TestPosePriorFactor::evaluateError, &factor,
                std::placeholders::_1, boost::none),
      pose);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

