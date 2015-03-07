/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TestSmartProjectionPoseFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @date   Sept 2013
 */

#include "../SmartProjectionPoseFactor.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <boost/assign/std/vector.hpp>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

static bool isDebugTest = false;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;

static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));
static Cal3_S2::shared_ptr K2(new Cal3_S2(1500, 1200, 0, 640, 480));
static boost::shared_ptr<Cal3Bundler> Kbundler(new Cal3Bundler(500, 1e-3, 1e-3, 1000, 2000));

static double rankTol = 1.0;
static double linThreshold = -1.0;
static bool manageDegeneracy = true;
// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

// tests data
static Symbol x1('X',  1);
static Symbol x2('X',  2);
static Symbol x3('X',  3);

static Key poseKey1(x1);
static Point2 measurement1(323.0, 240.0);
static Pose3 body_P_sensor1(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

typedef SmartProjectionPoseFactor<Pose3,Point3,Cal3_S2> SmartFactor;
typedef SmartProjectionPoseFactor<Pose3,Point3,Cal3Bundler> SmartFactorBundler;

void projectToMultipleCameras(
    SimpleCamera cam1, SimpleCamera cam2, SimpleCamera cam3, Point3 landmark,
    vector<Point2>& measurements_cam){

  Point2 cam1_uv1 = cam1.project(landmark);
  Point2 cam2_uv1 = cam2.project(landmark);
  Point2 cam3_uv1 = cam3.project(landmark);
  measurements_cam.push_back(cam1_uv1);
  measurements_cam.push_back(cam2_uv1);
  measurements_cam.push_back(cam3_uv1);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor) {
  SmartFactor::shared_ptr factor1(new SmartFactor());
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor2) {
  SmartFactor factor1(rankTol, linThreshold);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor3) {
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, poseKey1, model, K);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor4) {
  SmartFactor factor1(rankTol, linThreshold);
  factor1.add(measurement1, poseKey1, model, K);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, ConstructorWithTransform) {
  bool manageDegeneracy = true;
  bool enableEPI = false;
  SmartFactor factor1(rankTol, linThreshold, manageDegeneracy, enableEPI, body_P_sensor1);
  factor1.add(measurement1, poseKey1, model, K);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Equals ) {
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, poseKey1, model, K);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  factor2->add(measurement1, poseKey1, model, K);

  CHECK(assert_equal(*factor1, *factor2));
}

/* *************************************************************************/
TEST_UNSAFE( SmartProjectionPoseFactor, noiseless ){
  // cout << " ************************ SmartProjectionPoseFactor: noisy ****************************" << endl;

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, level_pose_right);

  SmartFactor factor1;
  factor1.add(level_uv, x1, model, K);
  factor1.add(level_uv_right, x2, model, K);

  double actualError = factor1.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartFactor::Cameras cameras = factor1.cameras(values);
  double actualError2 = factor1.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // test vector of errors
  //Vector actual = factor1.unwhitenedError(values);
  //EXPECT(assert_equal(zero(4),actual,1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, noisy ){
  // cout << " ************************ SmartProjectionPoseFactor: noisy ****************************" << endl;

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K2);

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

  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(level_uv, x1, model, K);
  factor1->add(level_uv_right, x2, model, K);

  double actualError1= factor1->error(values);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  vector<Point2> measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  std::vector< SharedNoiseModel > noises;
  noises.push_back(model);
  noises.push_back(model);

  std::vector< boost::shared_ptr<Cal3_S2> > Ks;  ///< shared pointer to calibration object (one for each camera)
  Ks.push_back(K);
  Ks.push_back(K);

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  factor2->add(measurements, views, noises, Ks);

  double actualError2= factor2->error(values);

  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}


/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_smart_projection_factor ){
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks **********************" << endl;

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, model, K2);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model, K2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model, K2);

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
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

//  GaussianFactorGraph::shared_ptr GFG = graph.linearize(values);
//  VectorValues delta = GFG->optimize();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3),1e-8));
  if(isDebugTest) tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_iterative_smart_projection_factor ){
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks **********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

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
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model, K);

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
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3), 1e-7));
  if(isDebugTest) tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, jacobianSVD ){

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

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
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor3->add(measurements_cam3, views, model, K);

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
  values.insert(x3, pose3*noise_pose);

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3), 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, landmarkDistance ){

  double excludeLandmarksFutherThanDist = 2;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

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
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD, excludeLandmarksFutherThanDist));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD, excludeLandmarksFutherThanDist));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD, excludeLandmarksFutherThanDist));
  smartFactor3->add(measurements_cam3, views, model, K);

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
  values.insert(x3, pose3*noise_pose);

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3),result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, dynamicOutlierRejection ){

  double excludeLandmarksFutherThanDist = 1e10;
  double dynamicOutlierRejectionThreshold = 1; // max 1 pixel of average reprojection error

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

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
  Point3 landmark4(5, -0.5, 1);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3, measurements_cam4;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  measurements_cam4.at(0) = measurements_cam4.at(0) + Point2(10,10); // add outlier

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(1, -1, false, false, boost::none,
      JACOBIAN_SVD, excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
      excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
      excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor3->add(measurements_cam3, views, model, K);

  SmartFactor::shared_ptr smartFactor4(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
      excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor4->add(measurements_cam4, views, model, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, jacobianQ ){

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

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
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor3->add(measurements_cam3, views, model, K);

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
  values.insert(x3, pose3*noise_pose);

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3), 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_projection_factor ){
  //  cout << " ************************ Normal ProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  NonlinearFactorGraph graph;

  // 1. Project three landmarks into three cameras and triangulate
  graph.push_back(ProjectionFactor(cam1.project(landmark1), model, x1, L(1), K2));
  graph.push_back(ProjectionFactor(cam2.project(landmark1), model, x2, L(1), K2));
  graph.push_back(ProjectionFactor(cam3.project(landmark1), model, x3, L(1), K2));

  graph.push_back(ProjectionFactor(cam1.project(landmark2), model, x1, L(2), K2));
  graph.push_back(ProjectionFactor(cam2.project(landmark2), model, x2, L(2), K2));
  graph.push_back(ProjectionFactor(cam3.project(landmark2), model, x3, L(2), K2));

  graph.push_back(ProjectionFactor(cam1.project(landmark3), model, x1, L(3), K2));
  graph.push_back(ProjectionFactor(cam2.project(landmark3), model, x2, L(3), K2));
  graph.push_back(ProjectionFactor(cam3.project(landmark3), model, x3, L(3), K2));

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3* noise_pose);
  values.insert(L(1), landmark1);
  values.insert(L(2), landmark2);
  values.insert(L(3), landmark3);
  if(isDebugTest)  values.at<Pose3>(x3).print("Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest)  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if(isDebugTest)  params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  if(isDebugTest)  result.at<Pose3>(x3).print("Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3), 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, CheckHessian){

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  double rankTol = 10;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(rankTol));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(rankTol));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(rankTol));
  smartFactor3->add(measurements_cam3, views, model, K);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3*noise_pose);
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  boost::shared_ptr<GaussianFactor> hessianFactor1 = smartFactor1->linearize(values);
  boost::shared_ptr<GaussianFactor> hessianFactor2 = smartFactor2->linearize(values);
  boost::shared_ptr<GaussianFactor> hessianFactor3 = smartFactor3->linearize(values);

  Matrix CumulativeInformation = hessianFactor1->information() +  hessianFactor2->information() + hessianFactor3->information();

  boost::shared_ptr<GaussianFactorGraph> GaussianGraph = graph.linearize(values);
  Matrix GraphInformation = GaussianGraph->hessian().first;

  // Check Hessian
  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-8));

  Matrix AugInformationMatrix = hessianFactor1->augmentedInformation() +
      hessianFactor2->augmentedInformation() + hessianFactor3->augmentedInformation();

  // Check Information vector
  // cout << AugInformationMatrix.size() << endl;
  Vector InfoVector = AugInformationMatrix.block(0,18,18,1); // 18x18 Hessian + information vector

  // Check Hessian
  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_2land_rotation_only_smart_projection_factor ){
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 2 landmarks: Rotation Only**********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam2(pose2, *K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam3(pose3, *K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);

  double rankTol = 50;
  SmartFactor::shared_ptr smartFactor1(new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, K2);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, K2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
  Point3 positionPrior = gtsam::Point3(0,0,1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2*noise_pose);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3*noise_pose*noise_pose);
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  std::cout << "TEST COMMENTED: rotation only version of smart factors has been deprecated " << std::endl;
  // EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
  if(isDebugTest) tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_rotation_only_smart_projection_factor ){
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks: Rotation Only**********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  double rankTol = 10;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, K);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, K);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
  Point3 positionPrior = gtsam::Point3(0,0,1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3*noise_pose);
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  std::cout << "TEST COMMENTED: rotation only version of smart factors has been deprecated " << std::endl;
  // EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
  if(isDebugTest) tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Hessian ){
  // cout << " ************************ SmartProjectionPoseFactor: Hessian **********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  vector<Point2> measurements_cam1;
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1,views, model, K2);

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor1->linearize(values);
  if(isDebugTest) hessianFactor->print("Hessian factor \n");

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with hessianFactor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
}


/* *************************************************************************/
TEST( SmartProjectionPoseFactor, HessianWithRotation ){
  // cout << " ************************ SmartProjectionPoseFactor: rotated Hessian **********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K);

  Point3 landmark1(5, 0.5, 1.2);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);

  SmartFactor::shared_ptr smartFactorInstance(new SmartFactor());
  smartFactorInstance->add(measurements_cam1, views, model, K);

  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactorInstance->linearize(values);
  // hessianFactor->print("Hessian factor \n");

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(pose1));
  rotValues.insert(x2, poseDrift.compose(pose2));
  rotValues.insert(x3, poseDrift.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRot = smartFactorInstance->linearize(rotValues);
  // hessianFactorRot->print("Hessian factor \n");

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(assert_equal(hessianFactor->information(), hessianFactorRot->information(), 1e-8) );

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI/2, -M_PI/3, -M_PI/2), gtsam::Point3(10,-4,5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(pose1));
  tranValues.insert(x2, poseDrift2.compose(pose2));
  tranValues.insert(x3, poseDrift2.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran = smartFactorInstance->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(assert_equal(hessianFactor->information(), hessianFactorRotTran->information(), 1e-8) );
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, HessianWithRotationDegenerate ){
  // cout << " ************************ SmartProjectionPoseFactor: rotated Hessian (degenerate) **********************" << endl;

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(0,0,0));
  SimpleCamera cam2(pose2, *K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,0,0));
  SimpleCamera cam3(pose3, *K2);

  Point3 landmark1(5, 0.5, 1.2);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);

  SmartFactor::shared_ptr smartFactor(new SmartFactor());
  smartFactor->add(measurements_cam1, views, model, K2);


  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor->linearize(values);
  if(isDebugTest)  hessianFactor->print("Hessian factor \n");

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(pose1));
  rotValues.insert(x2, poseDrift.compose(pose2));
  rotValues.insert(x3, poseDrift.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRot = smartFactor->linearize(rotValues);
  if(isDebugTest)  hessianFactorRot->print("Hessian factor \n");

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(assert_equal(hessianFactor->information(), hessianFactorRot->information(), 1e-8) );

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI/2, -M_PI/3, -M_PI/2), gtsam::Point3(10,-4,5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(pose1));
  tranValues.insert(x2, poseDrift2.compose(pose2));
  tranValues.insert(x3, poseDrift2.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran = smartFactor->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(assert_equal(hessianFactor->information(), hessianFactorRotTran->information(), 1e-8) );
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, ConstructorWithCal3Bundler) {
  SmartProjectionPoseFactor<Pose3,Point3,Cal3Bundler> factor1(rankTol, linThreshold);
  boost::shared_ptr<Cal3Bundler> Kbundler(new Cal3Bundler(500, 1e-3, 1e-3, 1000, 2000));
  factor1.add(measurement1, poseKey1, model, Kbundler);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Cal3Bundler ){
  // cout << " ************************ SmartProjectionPoseFactor: Cal3Bundler **********************" << endl;

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  PinholeCamera<Cal3Bundler> cam1(pose1, *Kbundler);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  PinholeCamera<Cal3Bundler> cam2(pose2, *Kbundler);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  PinholeCamera<Cal3Bundler> cam3(pose3, *Kbundler);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2 cam3_uv1 = cam3.project(landmark1);
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);
  measurements_cam1.push_back(cam3_uv1);

  Point2 cam1_uv2 = cam1.project(landmark2);
  Point2 cam2_uv2 = cam2.project(landmark2);
  Point2 cam3_uv2 = cam3.project(landmark2);
  measurements_cam2.push_back(cam1_uv2);
  measurements_cam2.push_back(cam2_uv2);
  measurements_cam2.push_back(cam3_uv2);

  Point2 cam1_uv3 = cam1.project(landmark3);
  Point2 cam2_uv3 = cam2.project(landmark3);
  Point2 cam3_uv3 = cam3.project(landmark3);
  measurements_cam3.push_back(cam1_uv3);
  measurements_cam3.push_back(cam2_uv3);
  measurements_cam3.push_back(cam3_uv3);

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartFactorBundler::shared_ptr smartFactor1(new SmartFactorBundler());
  smartFactor1->add(measurements_cam1, views, model, Kbundler);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler());
  smartFactor2->add(measurements_cam2, views, model, Kbundler);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler());
  smartFactor3->add(measurements_cam3, views, model, Kbundler);

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
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose3,result.at<Pose3>(x3), 1e-6));
  if(isDebugTest) tictoc_print_();
 }

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Cal3BundlerRotationOnly  ){

  std::vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  PinholeCamera<Cal3Bundler> cam1(pose1, *Kbundler);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  PinholeCamera<Cal3Bundler> cam2(pose2, *Kbundler);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
  PinholeCamera<Cal3Bundler> cam3(pose3, *Kbundler);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2 cam3_uv1 = cam3.project(landmark1);
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);
  measurements_cam1.push_back(cam3_uv1);

  Point2 cam1_uv2 = cam1.project(landmark2);
  Point2 cam2_uv2 = cam2.project(landmark2);
  Point2 cam3_uv2 = cam3.project(landmark2);
  measurements_cam2.push_back(cam1_uv2);
  measurements_cam2.push_back(cam2_uv2);
  measurements_cam2.push_back(cam3_uv2);

  Point2 cam1_uv3 = cam1.project(landmark3);
  Point2 cam2_uv3 = cam2.project(landmark3);
  Point2 cam3_uv3 = cam3.project(landmark3);
  measurements_cam3.push_back(cam1_uv3);
  measurements_cam3.push_back(cam2_uv3);
  measurements_cam3.push_back(cam3_uv3);

  double rankTol = 10;

  SmartFactorBundler::shared_ptr smartFactor1(new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, Kbundler);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, Kbundler);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model, Kbundler);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
  Point3 positionPrior = gtsam::Point3(0,0,1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.push_back(PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3*noise_pose);
  if(isDebugTest) values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if(isDebugTest) params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if(isDebugTest) params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if(isDebugTest) result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  std::cout << "TEST COMMENTED: rotation only version of smart factors has been deprecated " << std::endl;
  // EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
  if(isDebugTest) tictoc_print_();
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


