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

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam_unstable/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
//using symbol_shorthand::X;
//using symbol_shorthand::L;

//typedef GenericProjectionFactor<Pose3, Point3> TestProjectionFactor;


///* ************************************************************************* */
TEST( MultiProjectionFactor, noiseless ){
  cout << " ************************ MultiProjectionFactor: noiseless ****************************" << endl;
  Values theta;
  NonlinearFactorGraph graph;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
//  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2; //, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  Values value;
  value.insert(x1, level_pose);
  value.insert(x2, level_pose_right);

//  poses += level_pose, level_pose_right;
  vector<Point2> measurements;
  measurements += level_uv, level_uv_right;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2> smartFactor(measurements, noiseProjection, views, K);

  double actualError = smartFactor.error(value);
  double expectedError = 0.0;

  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

///* ************************************************************************* */
TEST( MultiProjectionFactor, noisy ){
  cout << " ************************ MultiProjectionFactor: noisy ****************************" << endl;

  Values theta;
  NonlinearFactorGraph graph;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
//  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2; //, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 pixelError(0.2,0.2);
  Point2 level_uv = level_camera.project(landmark) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark);

  Values value;
  value.insert(x1, level_pose);
  value.insert(x2, level_pose_right);

//  poses += level_pose, level_pose_right;
  vector<Point2> measurements;
  measurements += level_uv, level_uv_right;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2> smartFactor(measurements, noiseProjection, views, K);

  double actualError = smartFactor.error(value);
  double expectedError = sqrt(0.08);

  // we do not expect to be able to predict the error, since the error on the pixel will change
  // the triangulation of the landmark which is internal to the factor.
  //  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

///* ************************************************************************* */
//TEST( ProjectionFactor, nonStandard ) {
//  GenericProjectionFactor<Pose3, Point3, Cal3DS2> f;
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, Constructor) {
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//
//  Point2 measurement(323.0, 240.0);
//
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K);
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, ConstructorWithTransform) {
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//
//  Point2 measurement(323.0, 240.0);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, Equals ) {
//  // Create two identical factors and make sure they're equal
//  Point2 measurement(323.0, 240.0);
//
//  TestProjectionFactor factor1(measurement, model, X(1), L(1), K);
//  TestProjectionFactor factor2(measurement, model, X(1), L(1), K);
//
//  CHECK(assert_equal(factor1, factor2));
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, EqualsWithTransform ) {
//  // Create two identical factors and make sure they're equal
//  Point2 measurement(323.0, 240.0);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//
//  TestProjectionFactor factor1(measurement, model, X(1), L(1), K, body_P_sensor);
//  TestProjectionFactor factor2(measurement, model, X(1), L(1), K, body_P_sensor);
//
//  CHECK(assert_equal(factor1, factor2));
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, Error ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//  Point2 measurement(323.0, 240.0);
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K);
//
//  // Set the linearization point
//  Pose3 pose(Rot3(), Point3(0,0,-6));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the error
//  Vector actualError(factor.evaluateError(pose, point));
//
//  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
//  Vector expectedError = Vector_(2, -3.0, 0.0);
//
//  // Verify we get the expected error
//  CHECK(assert_equal(expectedError, actualError, 1e-9));
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, ErrorWithTransform ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//  Point2 measurement(323.0, 240.0);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);
//
//  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//  Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the error
//  Vector actualError(factor.evaluateError(pose, point));
//
//  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
//  Vector expectedError = Vector_(2, -3.0, 0.0);
//
//  // Verify we get the expected error
//  CHECK(assert_equal(expectedError, actualError, 1e-9));
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, Jacobian ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//  Point2 measurement(323.0, 240.0);
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K);
//
//  // Set the linearization point
//  Pose3 pose(Rot3(), Point3(0,0,-6));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the Jacobians
//  Matrix H1Actual, H2Actual;
//  factor.evaluateError(pose, point, H1Actual, H2Actual);
//
//  // The expected Jacobians
//  Matrix H1Expected = Matrix_(2, 6, 0., -554.256, 0., -92.376, 0., 0., 554.256, 0., 0., 0., -92.376, 0.);
//  Matrix H2Expected = Matrix_(2, 3, 92.376, 0., 0., 0., 92.376, 0.);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
//}
//
///* ************************************************************************* */
//TEST( ProjectionFactor, JacobianWithTransform ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  Key poseKey(X(1));
//  Key pointKey(L(1));
//  Point2 measurement(323.0, 240.0);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//  TestProjectionFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);
//
//  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//  Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the Jacobians
//  Matrix H1Actual, H2Actual;
//  factor.evaluateError(pose, point, H1Actual, H2Actual);
//
//  // The expected Jacobians
//  Matrix H1Expected = Matrix_(2, 6, -92.376, 0., 577.350, 0., 92.376, 0., -9.2376, -577.350, 0., 0., 0., 92.376);
//  Matrix H2Expected = Matrix_(2, 3, 0., -92.376, 0., 0., 0., -92.376);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

