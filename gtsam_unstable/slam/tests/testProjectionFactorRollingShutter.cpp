/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ProjectionFactorRollingShutterRollingShutter.cpp
 *  @brief Unit tests for ProjectionFactorRollingShutter Class
 *  @author Luca Carlone
 *  @date July 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60;  // degrees
static size_t w = 640, h = 480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov, w, h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::L;
using symbol_shorthand::T;
using symbol_shorthand::X;

// Convenience to define common variables across many tests
static Key poseKey1(X(1));
static Key poseKey2(X(2));
static Key pointKey(L(1));
static double interp_params = 0.5;
static Point2 measurement(323.0, 240.0);
static Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
                           Point3(0.25, -0.10, 1.0));

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, Constructor) {
  ProjectionFactorRollingShutter factor(measurement, interp_params, model,
                                        poseKey1, poseKey2, pointKey, K);
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, ConstructorWithTransform) {
  ProjectionFactorRollingShutter factor(measurement, interp_params, model,
                                        poseKey1, poseKey2, pointKey, K,
                                        body_P_sensor);
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, Equals) {
  {  // factors are equal
    ProjectionFactorRollingShutter factor1(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K);
    ProjectionFactorRollingShutter factor2(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K);
    CHECK(assert_equal(factor1, factor2));
  }
  {  // factors are NOT equal (keys are different)
    ProjectionFactorRollingShutter factor1(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K);
    ProjectionFactorRollingShutter factor2(measurement, interp_params, model,
                                           poseKey1, poseKey1, pointKey, K);
    CHECK(!assert_equal(factor1, factor2));  // not equal
  }
  {  // factors are NOT equal (different interpolation)
    ProjectionFactorRollingShutter factor1(measurement, 0.1, model, poseKey1,
                                           poseKey1, pointKey, K);
    ProjectionFactorRollingShutter factor2(measurement, 0.5, model, poseKey1,
                                           poseKey2, pointKey, K);
    CHECK(!assert_equal(factor1, factor2));  // not equal
  }
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, EqualsWithTransform) {
  {  // factors are equal
    ProjectionFactorRollingShutter factor1(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K,
                                           body_P_sensor);
    ProjectionFactorRollingShutter factor2(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K,
                                           body_P_sensor);
    CHECK(assert_equal(factor1, factor2));
  }
  {  // factors are NOT equal
    ProjectionFactorRollingShutter factor1(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K,
                                           body_P_sensor);
    Pose3 body_P_sensor2(
        Rot3::RzRyRx(0.0, 0.0, 0.0),
        Point3(0.25, -0.10, 1.0));  // rotation different from body_P_sensor
    ProjectionFactorRollingShutter factor2(measurement, interp_params, model,
                                           poseKey1, poseKey2, pointKey, K,
                                           body_P_sensor2);
    CHECK(!assert_equal(factor1, factor2));
  }
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, Error) {
  {
    // Create the factor with a measurement that is 3 pixels off in x
    // Camera pose corresponds to the first camera
    double t = 0.0;
    ProjectionFactorRollingShutter factor(measurement, t, model, poseKey1,
                                          poseKey2, pointKey, K);

    // Set the linearization point
    Pose3 pose1(Rot3(), Point3(0, 0, -6));
    Pose3 pose2(Rot3(), Point3(0, 0, -4));
    Point3 point(0.0, 0.0, 0.0);

    // Use the factor to calculate the error
    Vector actualError(factor.evaluateError(pose1, pose2, point));

    // The expected error is (-3.0, 0.0) pixels / UnitCovariance
    Vector expectedError = Vector2(-3.0, 0.0);

    // Verify we get the expected error
    CHECK(assert_equal(expectedError, actualError, 1e-9));
  }
  {
    // Create the factor with a measurement that is 3 pixels off in x
    // Camera pose is actually interpolated now
    double t = 0.5;
    ProjectionFactorRollingShutter factor(measurement, t, model, poseKey1,
                                          poseKey2, pointKey, K);

    // Set the linearization point
    Pose3 pose1(Rot3(), Point3(0, 0, -8));
    Pose3 pose2(Rot3(), Point3(0, 0, -4));
    Point3 point(0.0, 0.0, 0.0);

    // Use the factor to calculate the error
    Vector actualError(factor.evaluateError(pose1, pose2, point));

    // The expected error is (-3.0, 0.0) pixels / UnitCovariance
    Vector expectedError = Vector2(-3.0, 0.0);

    // Verify we get the expected error
    CHECK(assert_equal(expectedError, actualError, 1e-9));
  }
  {
    // Create measurement by projecting 3D landmark
    double t = 0.3;
    Pose3 pose1(Rot3::RzRyRx(0.1, 0.0, 0.1), Point3(0, 0, 0));
    Pose3 pose2(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0, 1));
    Pose3 poseInterp = interpolate<Pose3>(pose1, pose2, t);
    PinholeCamera<Cal3_S2> camera(poseInterp, *K);
    Point3 point(0.0, 0.0, 5.0);  // 5 meters in front of the camera
    Point2 measured = camera.project(point);

    // create factor
    ProjectionFactorRollingShutter factor(measured, t, model, poseKey1,
                                          poseKey2, pointKey, K);

    // Use the factor to calculate the error
    Vector actualError(factor.evaluateError(pose1, pose2, point));

    // The expected error is zero
    Vector expectedError = Vector2(0.0, 0.0);

    // Verify we get the expected error
    CHECK(assert_equal(expectedError, actualError, 1e-9));
  }
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, ErrorWithTransform) {
  // Create measurement by projecting 3D landmark
  double t = 0.3;
  Pose3 pose1(Rot3::RzRyRx(0.1, 0.0, 0.1), Point3(0, 0, 0));
  Pose3 pose2(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0, 1));
  Pose3 poseInterp = interpolate<Pose3>(pose1, pose2, t);
  Pose3 body_P_sensor3(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0.2, 0.1));
  PinholeCamera<Cal3_S2> camera(poseInterp * body_P_sensor3, *K);
  Point3 point(0.0, 0.0, 5.0);  // 5 meters in front of the camera
  Point2 measured = camera.project(point);

  // create factor
  ProjectionFactorRollingShutter factor(measured, t, model, poseKey1, poseKey2,
                                        pointKey, K, body_P_sensor3);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose1, pose2, point));

  // The expected error is zero
  Vector expectedError = Vector2(0.0, 0.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, Jacobian) {
  // Create measurement by projecting 3D landmark
  double t = 0.3;
  Pose3 pose1(Rot3::RzRyRx(0.1, 0.0, 0.1), Point3(0, 0, 0));
  Pose3 pose2(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0, 1));
  Pose3 poseInterp = interpolate<Pose3>(pose1, pose2, t);
  PinholeCamera<Cal3_S2> camera(poseInterp, *K);
  Point3 point(0.0, 0.0, 5.0);  // 5 meters in front of the camera
  Point2 measured = camera.project(point);

  // create factor
  ProjectionFactorRollingShutter factor(measured, t, model, poseKey1, poseKey2,
                                        pointKey, K);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual;
  factor.evaluateError(pose1, pose2, point, H1Actual, H2Actual, H3Actual);

  auto f = [&factor](const Pose3& p1, const Pose3& p2, const Point3& p3) {
	  return factor.evaluateError(p1, p2, p3); 
  };
  // Expected Jacobians via numerical derivatives
  Matrix H1Expected = numericalDerivative31<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  Matrix H2Expected = numericalDerivative32<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  Matrix H3Expected = numericalDerivative33<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  CHECK(assert_equal(H1Expected, H1Actual, 1e-5));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-5));
  CHECK(assert_equal(H3Expected, H3Actual, 1e-5));
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, JacobianWithTransform) {
  // Create measurement by projecting 3D landmark
  double t = 0.6;
  Pose3 pose1(Rot3::RzRyRx(0.1, 0.0, 0.1), Point3(0, 0, 0));
  Pose3 pose2(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0, 1));
  Pose3 poseInterp = interpolate<Pose3>(pose1, pose2, t);
  Pose3 body_P_sensor3(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0.2, 0.1));
  PinholeCamera<Cal3_S2> camera(poseInterp * body_P_sensor3, *K);
  Point3 point(0.0, 0.0, 5.0);  // 5 meters in front of the camera
  Point2 measured = camera.project(point);

  // create factor
  ProjectionFactorRollingShutter factor(measured, t, model, poseKey1, poseKey2,
                                        pointKey, K, body_P_sensor3);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual;
  factor.evaluateError(pose1, pose2, point, H1Actual, H2Actual, H3Actual);

  auto f = [&factor](const Pose3& p1, const Pose3& p2, const Point3& p3) {
	  return factor.evaluateError(p1, p2, p3); 
  };
  // Expected Jacobians via numerical derivatives
  Matrix H1Expected = numericalDerivative31<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  Matrix H2Expected = numericalDerivative32<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  Matrix H3Expected = numericalDerivative33<Vector, Pose3, Pose3, Point3>(f, pose1, pose2, point);

  CHECK(assert_equal(H1Expected, H1Actual, 1e-5));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-5));
  CHECK(assert_equal(H3Expected, H3Actual, 1e-5));
}

/* ************************************************************************* */
TEST(ProjectionFactorRollingShutter, cheirality) {
  // Create measurement by projecting 3D landmark behind camera
  double t = 0.3;
  Pose3 pose1(Rot3::RzRyRx(0.1, 0.0, 0.1), Point3(0, 0, 0));
  Pose3 pose2(Rot3::RzRyRx(-0.1, -0.1, 0.0), Point3(0, 0, 1));
  Pose3 poseInterp = interpolate<Pose3>(pose1, pose2, t);
  PinholeCamera<Cal3_S2> camera(poseInterp, *K);
  Point3 point(0.0, 0.0, -5.0);  // 5 meters behind the camera

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  Point2 measured = Point2(0.0, 0.0);  // project would throw an exception
  {  // check that exception is thrown if we set throwCheirality = true
    bool throwCheirality = true;
    bool verboseCheirality = true;
    ProjectionFactorRollingShutter factor(measured, t, model, poseKey1,
                                          poseKey2, pointKey, K,
                                          throwCheirality, verboseCheirality);
    CHECK_EXCEPTION(factor.evaluateError(pose1, pose2, point),
                    CheiralityException);
  }
  {  // check that exception is NOT thrown if we set throwCheirality = false,
     // and outputs are correct
    bool throwCheirality = false;    // default
    bool verboseCheirality = false;  // default
    ProjectionFactorRollingShutter factor(measured, t, model, poseKey1,
                                          poseKey2, pointKey, K,
                                          throwCheirality, verboseCheirality);

    // Use the factor to calculate the error
    Matrix H1Actual, H2Actual, H3Actual;
    Vector actualError(factor.evaluateError(pose1, pose2, point, H1Actual,
                                            H2Actual, H3Actual));

    // The expected error is zero
    Vector expectedError = Vector2::Constant(
        2.0 * K->fx());  // this is what we return when point is behind camera

    // Verify we get the expected error
    CHECK(assert_equal(expectedError, actualError, 1e-9));
    CHECK(assert_equal(Matrix::Zero(2, 6), H1Actual, 1e-5));
    CHECK(assert_equal(Matrix::Zero(2, 6), H2Actual, 1e-5));
    CHECK(assert_equal(Matrix::Zero(2, 3), H3Actual, 1e-5));
  }
#else
  {
    // everything is well defined, hence this matches the test "Jacobian" above:
    Point2 measured = camera.project(point);

    // create factor
    ProjectionFactorRollingShutter factor(measured, t, model, poseKey1,
                                          poseKey2, pointKey, K);

    // Use the factor to calculate the Jacobians
    Matrix H1Actual, H2Actual, H3Actual;
    factor.evaluateError(pose1, pose2, point, H1Actual, H2Actual, H3Actual);

    // Expected Jacobians via numerical derivatives
    Matrix H1Expected = numericalDerivative31<Vector, Pose3, Pose3, Point3>(
        std::function<Vector(const Pose3&, const Pose3&, const Point3&)>(
            std::bind(&ProjectionFactorRollingShutter::evaluateError, &factor,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, {}, {},
                      {})),
        pose1, pose2, point);

    Matrix H2Expected = numericalDerivative32<Vector, Pose3, Pose3, Point3>(
        std::function<Vector(const Pose3&, const Pose3&, const Point3&)>(
            std::bind(&ProjectionFactorRollingShutter::evaluateError, &factor,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, {}, {},
                      {})),
        pose1, pose2, point);

    Matrix H3Expected = numericalDerivative33<Vector, Pose3, Pose3, Point3>(
        std::function<Vector(const Pose3&, const Pose3&, const Point3&)>(
            std::bind(&ProjectionFactorRollingShutter::evaluateError, &factor,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, {}, {},
                      {})),
        pose1, pose2, point);

    CHECK(assert_equal(H1Expected, H1Actual, 1e-5));
    CHECK(assert_equal(H2Expected, H2Actual, 1e-5));
    CHECK(assert_equal(H3Expected, H3Actual, 1e-5));
  }
#endif
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
