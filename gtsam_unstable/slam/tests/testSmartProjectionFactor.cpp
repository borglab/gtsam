/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TestSmartProjectionFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2009
 */

#include <CppUnitLite/TestHarness.h>
#include <iostream>

TEST(SmartProjectionFactor, disabled)
{
  CHECK(("*** testSmartProjectionFactor is disabled *** - Needs conversion for unordered", 0));
}

#if 0

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam_unstable/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef SmartProjectionFactor<Pose3, Point3> TestSmartProjectionFactor;

/* ************************************************************************* */
TEST( SmartProjectionFactor, Constructor) {
  Key poseKey(X(1));

  std::vector<Key> views;
  views += poseKey;

  std::vector<Point2> measurements;
  measurements.push_back(Point2(323.0, 240.0));

  TestSmartProjectionFactor factor(measurements, model, views, K);
}

/* ************************************************************************* */
TEST( SmartProjectionFactor, ConstructorWithTransform) {
  Key poseKey(X(1));

  std::vector<Key> views;
  views += poseKey;

  std::vector<Point2> measurements;
  measurements.push_back(Point2(323.0, 240.0));
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestSmartProjectionFactor factor(measurements, model, views, K, body_P_sensor);
}

/* ************************************************************************* */
TEST( SmartProjectionFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  std::vector<Point2> measurements;
  measurements.push_back(Point2(323.0, 240.0));

  std::vector<Key> views;
  views += X(1);
  TestSmartProjectionFactor factor1(measurements, model, views, K);
  TestSmartProjectionFactor factor2(measurements, model, views, K);

  CHECK(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( SmartProjectionFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  std::vector<Point2> measurements;
  measurements.push_back(Point2(323.0, 240.0));
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  std::vector<Key> views;
  views += X(1);
  TestSmartProjectionFactor factor1(measurements, model, views, K, body_P_sensor);
  TestSmartProjectionFactor factor2(measurements, model, views, K, body_P_sensor);

  CHECK(assert_equal(factor1, factor2));
}


/* ************************************************************************* */
TEST( SmartProjectionFactor, noisy ){
  cout << " ************************ MultiProjectionFactor: noisy ****************************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);

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

  Values values;
  values.insert(x1, level_pose);
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  values.insert(x2, level_pose_right.compose(noise_pose));

  vector<Point2> measurements;
  measurements += level_uv, level_uv_right;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr
    smartFactor(new SmartProjectionFactor<Pose3, Point3, Cal3_S2>(measurements, noiseProjection, views, K));

  double actualError = smartFactor->error(values);
  std::cout << "Error: " << actualError << std::endl;

  // we do not expect to be able to predict the error, since the error on the pixel will change
  // the triangulation of the landmark which is internal to the factor.
  // DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}


/* ************************************************************************* */
TEST( SmartProjectionFactor, 3poses_smart_projection_factor ){
  cout << " ************************ SmartProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2 cam3_uv1 = cam3.project(landmark1);
  measurements_cam1 += cam1_uv1, cam2_uv1, cam3_uv1;

  //
  Point2 cam1_uv2 = cam1.project(landmark2);
  Point2 cam2_uv2 = cam2.project(landmark2);
  Point2 cam3_uv2 = cam3.project(landmark2);
  measurements_cam2 += cam1_uv2, cam2_uv2, cam3_uv2;


  Point2 cam1_uv3 = cam1.project(landmark3);
  Point2 cam2_uv3 = cam2.project(landmark3);
  Point2 cam3_uv3 = cam3.project(landmark3);
  measurements_cam3 += cam1_uv3, cam2_uv3, cam3_uv3;

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(measurements_cam1, noiseProjection, views, K));
  SmartFactor::shared_ptr smartFactor2(new SmartFactor(measurements_cam2, noiseProjection, views, K));
  SmartFactor::shared_ptr smartFactor3(new SmartFactor(measurements_cam3, noiseProjection, views, K));

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose2, noisePrior));

//  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3*noise_pose);
  values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
  tictoc_print_();

}


/* ************************************************************************* */
TEST( SmartProjectionFactor, 3poses_projection_factor ){
//  cout << " ************************ Normal ProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  NonlinearFactorGraph graph;

  // 1. Project three landmarks into three cameras and triangulate
  graph.add(ProjectionFactor(cam1.project(landmark1), noiseProjection, x1, L(1), K));
  graph.add(ProjectionFactor(cam2.project(landmark1), noiseProjection, x2, L(1), K));
  graph.add(ProjectionFactor(cam3.project(landmark1), noiseProjection, x3, L(1), K));

  //
  graph.add(ProjectionFactor(cam1.project(landmark2), noiseProjection, x1, L(2), K));
  graph.add(ProjectionFactor(cam2.project(landmark2), noiseProjection, x2, L(2), K));
  graph.add(ProjectionFactor(cam3.project(landmark2), noiseProjection, x3, L(2), K));

  graph.add(ProjectionFactor(cam1.project(landmark3), noiseProjection, x1, L(3), K));
  graph.add(ProjectionFactor(cam2.project(landmark3), noiseProjection, x2, L(3), K));
  graph.add(ProjectionFactor(cam3.project(landmark3), noiseProjection, x3, L(3), K));

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  graph.add(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.add(PriorFactor<Pose3>(x2, pose2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3* noise_pose);
  values.insert(L(1), landmark1);
  values.insert(L(2), landmark2);
  values.insert(L(3), landmark3);
//  values.at<Pose3>(x3).print("Pose3 before optimization: ");

  LevenbergMarquardtParams params;
//  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
//  params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

//  result.at<Pose3>(x3).print("Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));

}


/* ************************************************************************* */
TEST( SmartProjectionFactor, Hessian ){
  cout << " ************************ SmartProjectionFactor: Hessian **********************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  vector<Point2> measurements_cam1;
  measurements_cam1 += cam1_uv1, cam2_uv1;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2> smartFactor(measurements_cam1, noiseProjection, views, K);

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  //  values.insert(L(1), landmark1);

  Ordering ordering;
  ordering.push_back(x1);
  ordering.push_back(x2);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor.linearize(values, ordering);
  hessianFactor->print("Hessian factor \n");

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with hessianFactor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]





}

#endif

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

