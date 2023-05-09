/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TestSmartStereoProjectionPoseFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @date   Sept 2013
 */

#include <gtsam/slam/tests/smartFactorScenarios.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <boost/assign/std/vector.hpp>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

// make a realistic calibration matrix
static double b = 1;

static Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fov, w, h, b));
static Cal3_S2Stereo::shared_ptr K2(
    new Cal3_S2Stereo(1500, 1200, 0, 640, 480, b));


static SmartStereoProjectionParams params;

// static bool manageDegeneracy = true;
// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);

static Key poseKey1(x1);
static StereoPoint2 measurement1(323.0, 300.0, 240.0); //potentially use more reasonable measurement value?
static Pose3 body_P_sensor1(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
    Point3(0.25, -0.10, 1.0));

static double missing_uR = std::numeric_limits<double>::quiet_NaN();

vector<StereoPoint2> stereo_projectToMultipleCameras(const StereoCamera& cam1,
    const StereoCamera& cam2, const StereoCamera& cam3, Point3 landmark) {

  vector<StereoPoint2> measurements_cam;

  StereoPoint2 cam1_uv1 = cam1.project(landmark);
  StereoPoint2 cam2_uv1 = cam2.project(landmark);
  StereoPoint2 cam3_uv1 = cam3.project(landmark);
  measurements_cam.push_back(cam1_uv1);
  measurements_cam.push_back(cam2_uv1);
  measurements_cam.push_back(cam3_uv1);

  return measurements_cam;
}

LevenbergMarquardtParams lm_params;

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, params) {
  SmartStereoProjectionParams p;

  // check default values and "get"
  EXPECT(p.getLinearizationMode() == HESSIAN);
  EXPECT(p.getDegeneracyMode() == IGNORE_DEGENERACY);
  EXPECT_DOUBLES_EQUAL(p.getRetriangulationThreshold(), 1e-5, 1e-9);
  EXPECT(p.getVerboseCheirality() == false);
  EXPECT(p.getThrowCheirality() == false);

  // check "set"
  p.setLinearizationMode(JACOBIAN_SVD);
  p.setDegeneracyMode(ZERO_ON_DEGENERACY);
  p.setRankTolerance(100);
  p.setEnableEPI(true);
  p.setLandmarkDistanceThreshold(200);
  p.setDynamicOutlierRejectionThreshold(3);
  p.setRetriangulationThreshold(1e-2);

  EXPECT(p.getLinearizationMode() == JACOBIAN_SVD);
  EXPECT(p.getDegeneracyMode() == ZERO_ON_DEGENERACY);
  EXPECT_DOUBLES_EQUAL(p.getTriangulationParameters().rankTolerance, 100, 1e-5);
  EXPECT(p.getTriangulationParameters().enableEPI == true);
  EXPECT_DOUBLES_EQUAL(p.getTriangulationParameters().landmarkDistanceThreshold, 200, 1e-5);
  EXPECT_DOUBLES_EQUAL(p.getTriangulationParameters().dynamicOutlierRejectionThreshold, 3, 1e-5);
  EXPECT_DOUBLES_EQUAL(p.getRetriangulationThreshold(), 1e-2, 1e-5);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, Constructor) {
  SmartStereoProjectionPoseFactor::shared_ptr factor1(new SmartStereoProjectionPoseFactor(model));
}

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, Constructor2) {
  SmartStereoProjectionPoseFactor factor1(model, params);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, Constructor3) {
  SmartStereoProjectionPoseFactor::shared_ptr factor1(new SmartStereoProjectionPoseFactor(model));
  factor1->add(measurement1, poseKey1, K);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, Constructor4) {
  SmartStereoProjectionPoseFactor factor1(model, params);
  factor1.add(measurement1, poseKey1, K);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionPoseFactor, Equals ) {
  SmartStereoProjectionPoseFactor::shared_ptr factor1(new SmartStereoProjectionPoseFactor(model));
  factor1->add(measurement1, poseKey1, K);

  SmartStereoProjectionPoseFactor::shared_ptr factor2(new SmartStereoProjectionPoseFactor(model));
  factor2->add(measurement1, poseKey1, K);

  CHECK(assert_equal(*factor1, *factor2));
}

/* *************************************************************************/
TEST_UNSAFE( SmartStereoProjectionPoseFactor, noiseless ) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  StereoCamera level_camera(level_pose, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera level_camera_right(level_pose_right, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  StereoPoint2 level_uv = level_camera.project(landmark);
  StereoPoint2 level_uv_right = level_camera_right.project(landmark);

  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, level_pose_right);

  SmartStereoProjectionPoseFactor factor1(model);
  factor1.add(level_uv, x1, K2);
  factor1.add(level_uv_right, x2, K2);

  double actualError = factor1.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartStereoProjectionPoseFactor::Cameras cameras = factor1.cameras(values);
  double actualError2 = factor1.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // test vector of errors
  //Vector actual = factor1.unwhitenedError(values);
  //EXPECT(assert_equal(zero(4),actual,1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, noiselessWithMissingMeasurements ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
   Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
       Point3(0, 0, 1));
   StereoCamera level_camera(level_pose, K2);

   // create second camera 1 meter to the right of first camera
   Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
   StereoCamera level_camera_right(level_pose_right, K2);

   // landmark ~5 meters in front of camera
   Point3 landmark(5, 0.5, 1.2);

   // 1. Project two landmarks into two cameras and triangulate
   StereoPoint2 level_uv = level_camera.project(landmark);
   StereoPoint2 level_uv_right = level_camera_right.project(landmark);
   StereoPoint2 level_uv_right_missing(level_uv_right.uL(),missing_uR,level_uv_right.v());

   Values values;
   values.insert(x1, level_pose);
   values.insert(x2, level_pose_right);

   SmartStereoProjectionPoseFactor factor1(model);
   factor1.add(level_uv, x1, K2);
   factor1.add(level_uv_right_missing, x2, K2);

   double actualError = factor1.error(values);
   double expectedError = 0.0;
   EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

   // TEST TRIANGULATION WITH MISSING VALUES: i) right pixel of second camera is missing:
   SmartStereoProjectionPoseFactor::Cameras cameras = factor1.cameras(values);
   double actualError2 = factor1.totalReprojectionError(cameras);
   EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

   CameraSet<StereoCamera> cams;
   cams += level_camera;
   cams += level_camera_right;
   TriangulationResult result = factor1.triangulateSafe(cams);
   CHECK(result);
   EXPECT(assert_equal(landmark, *result, 1e-7));

   // TEST TRIANGULATION WITH MISSING VALUES: ii) right pixels of both cameras are missing:
   SmartStereoProjectionPoseFactor factor2(model);
   StereoPoint2 level_uv_missing(level_uv.uL(),missing_uR,level_uv.v());
   factor2.add(level_uv_missing, x1, K2);
   factor2.add(level_uv_right_missing, x2, K2);
   result = factor2.triangulateSafe(cams);
   CHECK(result);
   EXPECT(assert_equal(landmark, *result, 1e-7));
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, noisy ) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  StereoCamera level_camera(level_pose, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera level_camera_right(level_pose_right, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  StereoPoint2 pixelError(0.2, 0.2, 0);
  StereoPoint2 level_uv = level_camera.project(landmark) + pixelError;
  StereoPoint2 level_uv_right = level_camera_right.project(landmark);

  Values values;
  values.insert(x1, level_pose);
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  values.insert(x2, level_pose_right.compose(noise_pose));

  SmartStereoProjectionPoseFactor::shared_ptr factor1(new SmartStereoProjectionPoseFactor(model));
  factor1->add(level_uv, x1, K);
  factor1->add(level_uv_right, x2, K);

  double actualError1 = factor1->error(values);

  SmartStereoProjectionPoseFactor::shared_ptr factor2(new SmartStereoProjectionPoseFactor(model));
  vector<StereoPoint2> measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  vector<boost::shared_ptr<Cal3_S2Stereo> > Ks; ///< shared pointer to calibration object (one for each camera)
  Ks.push_back(K);
  Ks.push_back(K);

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);

  factor2->add(measurements, views, Ks);

  double actualError2 = factor2->error(values);

  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, 3poses_smart_projection_factor ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // 1. Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_l1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_l2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_l3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(model, smart_params));
  smartFactor1->add(measurements_l1, views, K2);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, smart_params));
  smartFactor2->add(measurements_l2, views, K2);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, smart_params));
  smartFactor3->add(measurements_l3, views, K2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3 * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));

  //  cout << std::setprecision(10) << "\n----SmartStereoFactor graph initial error: " << graph.error(values) << endl;
  EXPECT_DOUBLES_EQUAL(833953.92789459578, graph.error(values), 1e-7); // initial error

  // get triangulated landmarks from smart factors
  Point3 landmark1_smart = *smartFactor1->point();
  Point3 landmark2_smart = *smartFactor2->point();
  Point3 landmark3_smart = *smartFactor3->point();

  Values result;
  gttic_(SmartStereoProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionPoseFactor);
  tictoc_finishedIteration_();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);

//  cout << std::setprecision(10) << "SmartStereoFactor graph optimized error: " << graph.error(result) << endl;

  GaussianFactorGraph::shared_ptr GFG = graph.linearize(result);
  VectorValues delta = GFG->optimize();
  VectorValues expected = VectorValues::Zero(delta);
  EXPECT(assert_equal(expected, delta, 1e-6));

  // result.print("results of 3 camera, 3 landmark optimization \n");
  EXPECT(assert_equal(pose3, result.at<Pose3>(x3)));

  /* ***************************************************************
   * Same problem with regular Stereo factors, expect same error!
   * ****************************************************************/

//  landmark1_smart.print("landmark1_smart");
//  landmark2_smart.print("landmark2_smart");
//  landmark3_smart.print("landmark3_smart");

  // add landmarks to values
  values.insert(L(1), landmark1_smart);
  values.insert(L(2), landmark2_smart);
  values.insert(L(3), landmark3_smart);

  // add factors
  NonlinearFactorGraph graph2;

  graph2.addPrior(x1, pose1, noisePrior);
  graph2.addPrior(x2, pose2, noisePrior);

  typedef GenericStereoFactor<Pose3, Point3> ProjectionFactor;

  bool verboseCheirality = true;

  graph2.push_back(ProjectionFactor(measurements_l1[0], model, x1, L(1), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l1[1], model, x2, L(1), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l1[2], model, x3, L(1), K2, false, verboseCheirality));

  graph2.push_back(ProjectionFactor(measurements_l2[0], model, x1, L(2), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l2[1], model, x2, L(2), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l2[2], model, x3, L(2), K2, false, verboseCheirality));

  graph2.push_back(ProjectionFactor(measurements_l3[0], model, x1, L(3), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l3[1], model, x2, L(3), K2, false, verboseCheirality));
  graph2.push_back(ProjectionFactor(measurements_l3[2], model, x3, L(3), K2, false, verboseCheirality));

//  cout << std::setprecision(10) << "\n----StereoFactor graph initial error: " << graph2.error(values) << endl;
  EXPECT_DOUBLES_EQUAL(833953.92789459578, graph2.error(values), 1e-7);

  LevenbergMarquardtOptimizer optimizer2(graph2, values, lm_params);
  Values result2 = optimizer2.optimize();
  EXPECT_DOUBLES_EQUAL(0, graph2.error(result2), 1e-5);

//  cout << std::setprecision(10) << "StereoFactor graph optimized error: " << graph2.error(result2) << endl;

}
/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, body_P_sensor ) {

  // camera has some displacement
  Pose3 body_P_sensor = Pose3(Rot3::Ypr(-0.01, 0., -0.05), Point3(0.1, 0, 0.1));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1.compose(body_P_sensor), K2);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2.compose(body_P_sensor), K2);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3.compose(body_P_sensor), K2);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // 1. Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_l1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_l2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_l3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(model, smart_params, body_P_sensor));
  smartFactor1->add(measurements_l1, views, K2);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, smart_params, body_P_sensor));
  smartFactor2->add(measurements_l2, views, K2);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, smart_params, body_P_sensor));
  smartFactor3->add(measurements_l3, views, K2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, pose3 * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));

  //  cout << std::setprecision(10) << "\n----SmartStereoFactor graph initial error: " << graph.error(values) << endl;
  EXPECT_DOUBLES_EQUAL(953392.32838422502, graph.error(values), 1e-7); // initial error

  Values result;
  gttic_(SmartStereoProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionPoseFactor);
  tictoc_finishedIteration_();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);

  // result.print("results of 3 camera, 3 landmark optimization \n");
  EXPECT(assert_equal(pose3, result.at<Pose3>(x3)));
}
/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, body_P_sensor_monocular ){
  // make a realistic calibration matrix
  double fov = 60; // degrees
  size_t w=640,h=480;

  Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 cameraPose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1)); // body poses
  Pose3 cameraPose2 = cameraPose1 * Pose3(Rot3(), Point3(1,0,0));
  Pose3 cameraPose3 = cameraPose1 * Pose3(Rot3(), Point3(0,-1,0));

  PinholeCamera<Cal3_S2> cam1(cameraPose1, *K); // with camera poses
  PinholeCamera<Cal3_S2> cam2(cameraPose2, *K);
  PinholeCamera<Cal3_S2> cam3(cameraPose3, *K);

  // create arbitrary body_Pose_sensor (transforms from sensor to body)
  Pose3 sensor_to_body =  Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(1, 1, 1)); // Pose3(); //

  // These are the poses we want to estimate, from camera measurements
  Pose3 bodyPose1 = cameraPose1.compose(sensor_to_body.inverse());
  Pose3 bodyPose2 = cameraPose2.compose(sensor_to_body.inverse());
  Pose3 bodyPose3 = cameraPose3.compose(sensor_to_body.inverse());

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(5, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create smart factors
  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // convert measurement to (degenerate) stereoPoint2 (with right pixel being NaN)
  vector<StereoPoint2> measurements_cam1_stereo, measurements_cam2_stereo, measurements_cam3_stereo;
  for(size_t k=0; k<measurements_cam1.size();k++)
	  measurements_cam1_stereo.push_back(StereoPoint2(measurements_cam1[k].x() , missing_uR , measurements_cam1[k].y()));

  for(size_t k=0; k<measurements_cam2.size();k++)
	  measurements_cam2_stereo.push_back(StereoPoint2(measurements_cam2[k].x() , missing_uR , measurements_cam2[k].y()));

  for(size_t k=0; k<measurements_cam3.size();k++)
	  measurements_cam3_stereo.push_back(StereoPoint2(measurements_cam3[k].x() , missing_uR , measurements_cam3[k].y()));

  SmartStereoProjectionParams params;
  params.setRankTolerance(1.0);
  params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
  params.setEnableEPI(false);

  Cal3_S2Stereo::shared_ptr Kmono(new Cal3_S2Stereo(fov,w,h,b));
  SmartStereoProjectionPoseFactor smartFactor1(model, params, sensor_to_body);
  smartFactor1.add(measurements_cam1_stereo, views, Kmono);

  SmartStereoProjectionPoseFactor smartFactor2(model, params, sensor_to_body);
  smartFactor2.add(measurements_cam2_stereo, views, Kmono);

  SmartStereoProjectionPoseFactor smartFactor3(model, params, sensor_to_body);
  smartFactor3.add(measurements_cam3_stereo, views, Kmono);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Put all factors in factor graph, adding priors
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, bodyPose1, noisePrior);
  graph.addPrior(x2, bodyPose2, noisePrior);

  // Check errors at ground truth poses
  Values gtValues;
  gtValues.insert(x1, bodyPose1);
  gtValues.insert(x2, bodyPose2);
  gtValues.insert(x3, bodyPose3);
  double actualError = graph.error(gtValues);
  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-7)

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/100, 0., -M_PI/100), gtsam::Point3(0.1,0.1,0.1));
  Values values;
  values.insert(x1, bodyPose1);
  values.insert(x2, bodyPose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(x3, bodyPose3*noise_pose);

  LevenbergMarquardtParams lmParams;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(bodyPose3,result.at<Pose3>(x3)));
}
/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, jacobianSVD ) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);
  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);
  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // 1. Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_cam2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_cam3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  SmartStereoProjectionParams params;
  params.setLinearizationMode(JACOBIAN_SVD);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1( new SmartStereoProjectionPoseFactor(model, params));
  smartFactor1->add(measurements_cam1, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor2->add(measurements_cam2, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor3->add(measurements_cam3, views, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3 * noise_pose);

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3, result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, jacobianSVDwithMissingValues ) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);
  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);
  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // 1. Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_cam2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_cam3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  // DELETE SOME MEASUREMENTS
  StereoPoint2 sp = measurements_cam1[1];
  measurements_cam1[1] = StereoPoint2(sp.uL(), missing_uR, sp.v());
  sp = measurements_cam2[2];
  measurements_cam2[2] = StereoPoint2(sp.uL(), missing_uR, sp.v());

  SmartStereoProjectionParams params;
  params.setLinearizationMode(JACOBIAN_SVD);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1( new SmartStereoProjectionPoseFactor(model, params));
  smartFactor1->add(measurements_cam1, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor2->add(measurements_cam2, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor3->add(measurements_cam3, views, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3 * noise_pose);

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3, result.at<Pose3>(x3),1e-7));
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, landmarkDistance ) {

//  double excludeLandmarksFutherThanDist = 2;

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);
  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);
  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // 1. Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_cam2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_cam3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  SmartStereoProjectionParams params;
  params.setLinearizationMode(JACOBIAN_SVD);
  params.setLandmarkDistanceThreshold(2);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor1->add(measurements_cam1, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor2->add(measurements_cam2, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor3->add(measurements_cam3, views, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3 * noise_pose);

  // All factors are disabled and pose should remain where it is
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, dynamicOutlierRejection ) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);
  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);
  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);
  Point3 landmark4(5, -0.5, 1);

  // 1. Project four landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_cam2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_cam3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);
  vector<StereoPoint2> measurements_cam4 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark4);

  measurements_cam4.at(0) = measurements_cam4.at(0) + StereoPoint2(10, 10, 1); // add outlier

  SmartStereoProjectionParams params;
  params.setLinearizationMode(JACOBIAN_SVD);
  params.setDynamicOutlierRejectionThreshold(1);


  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor1->add(measurements_cam1, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor2->add(measurements_cam2, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor3->add(measurements_cam3, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor4(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor4->add(measurements_cam4, views, K);

  // same as factor 4, but dynamic outlier rejection is off
  SmartStereoProjectionPoseFactor::shared_ptr smartFactor4b(new SmartStereoProjectionPoseFactor(model));
  smartFactor4b->add(measurements_cam4, views, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  EXPECT_DOUBLES_EQUAL(0, smartFactor1->error(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(0, smartFactor2->error(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(0, smartFactor3->error(values), 1e-9);
  // zero error due to dynamic outlier rejection
  EXPECT_DOUBLES_EQUAL(0, smartFactor4->error(values), 1e-9);

  // dynamic outlier rejection is off
  EXPECT_DOUBLES_EQUAL(6147.3947317473921, smartFactor4b->error(values), 1e-9);

  // Factors 1-3 should have valid point, factor 4 should not
  EXPECT(smartFactor1->point());
  EXPECT(smartFactor2->point());
  EXPECT(smartFactor3->point());
  EXPECT(smartFactor4->point().outlier());
  EXPECT(smartFactor4b->point());

  // Factor 4 is disabled, pose 3 stays put
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose3, result.at<Pose3>(x3)));
}
//
///* *************************************************************************/
//TEST( SmartStereoProjectionPoseFactor, jacobianQ ){
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//  views.push_back(x3);
//
//  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
//  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), Point3(0,0,1));
//  StereoCamera cam1(pose1, K);
//  // create second camera 1 meter to the right of first camera
//  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
//  StereoCamera cam2(pose2, K);
//  // create third camera 1 meter above the first camera
//  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
//  StereoCamera cam3(pose3, K);
//
//  // three landmarks ~5 meters infront of camera
//  Point3 landmark1(5, 0.5, 1.2);
//  Point3 landmark2(5, -0.5, 1.2);
//  Point3 landmark3(3, 0, 3.0);
//
//  vector<StereoPoint2> measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // 1. Project three landmarks into three cameras and triangulate
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(1, -1, false, false, JACOBIAN_Q));
//  smartFactor1->add(measurements_cam1, views, model, K);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(1, -1, false, false, JACOBIAN_Q));
//  smartFactor2->add(measurements_cam2, views, model, K);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(1, -1, false, false, JACOBIAN_Q));
//  smartFactor3->add(measurements_cam3, views, model, K);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
//  graph.push_back(PriorFactor<Pose3>(x2, pose2, noisePrior));
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/100, 0., -M_PI/100), Point3(0.1,0.1,0.1)); // smaller noise
//  Values values;
//  values.insert(x1, pose1);
//  values.insert(x2, pose2);
//  values.insert(x3, pose3*noise_pose);
//
////  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartStereoProjectionPoseFactor, 3poses_projection_factor ){
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//  views.push_back(x3);
//
//  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
//  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), Point3(0,0,1));
//  StereoCamera cam1(pose1, K2);
//
//  // create second camera 1 meter to the right of first camera
//  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
//  StereoCamera cam2(pose2, K2);
//
//  // create third camera 1 meter above the first camera
//  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
//  StereoCamera cam3(pose3, K2);
//
//  // three landmarks ~5 meters infront of camera
//  Point3 landmark1(5, 0.5, 1.2);
//  Point3 landmark2(5, -0.5, 1.2);
//  Point3 landmark3(3, 0, 3.0);
//
//  typedef GenericStereoFactor<Pose3, Point3> ProjectionFactor;
//  NonlinearFactorGraph graph;
//
//  // 1. Project three landmarks into three cameras and triangulate
//  graph.push_back(ProjectionFactor(cam1.project(landmark1), model, x1, L(1), K2));
//  graph.push_back(ProjectionFactor(cam2.project(landmark1), model, x2, L(1), K2));
//  graph.push_back(ProjectionFactor(cam3.project(landmark1), model, x3, L(1), K2));
//
//  graph.push_back(ProjectionFactor(cam1.project(landmark2), model, x1, L(2), K2));
//  graph.push_back(ProjectionFactor(cam2.project(landmark2), model, x2, L(2), K2));
//  graph.push_back(ProjectionFactor(cam3.project(landmark2), model, x3, L(2), K2));
//
//  graph.push_back(ProjectionFactor(cam1.project(landmark3), model, x1, L(3), K2));
//  graph.push_back(ProjectionFactor(cam2.project(landmark3), model, x2, L(3), K2));
//  graph.push_back(ProjectionFactor(cam3.project(landmark3), model, x3, L(3), K2));
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
//  graph.push_back(PriorFactor<Pose3>(x2, pose2, noisePrior));
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3));
//  Values values;
//  values.insert(x1, pose1);
//  values.insert(x2, pose2);
//  values.insert(x3, pose3* noise_pose);
//  values.insert(L(1), landmark1);
//  values.insert(L(2), landmark2);
//  values.insert(L(3), landmark3);
//
//  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
//  Values result = optimizer.optimize();
//
//  EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
//}
//
/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, CheckHessian) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);

  // create second camera
  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  StereoCamera cam2(pose2, K);

  // create third camera
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  StereoCamera cam3(pose3, K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  // Project three landmarks into three cameras and triangulate
  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);
  vector<StereoPoint2> measurements_cam2 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark2);
  vector<StereoPoint2> measurements_cam3 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark3);

  SmartStereoProjectionParams params;
  params.setRankTolerance(10);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor1->add(measurements_cam1, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor2->add(measurements_cam2, views, K);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(model, params));
  smartFactor3->add(measurements_cam3, views, K);

  // Create graph to optimize
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  values.insert(x3, pose3 * noise_pose);

  // TODO: next line throws Cheirality exception on Mac
  boost::shared_ptr<GaussianFactor> hessianFactor1 = smartFactor1->linearize(
      values);
  boost::shared_ptr<GaussianFactor> hessianFactor2 = smartFactor2->linearize(
      values);
  boost::shared_ptr<GaussianFactor> hessianFactor3 = smartFactor3->linearize(
      values);

  Matrix CumulativeInformation = hessianFactor1->information()
      + hessianFactor2->information() + hessianFactor3->information();

  boost::shared_ptr<GaussianFactorGraph> GaussianGraph = graph.linearize(
      values);
  Matrix GraphInformation = GaussianGraph->hessian().first;

  // Check Hessian
  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-8));

  Matrix AugInformationMatrix = hessianFactor1->augmentedInformation()
      + hessianFactor2->augmentedInformation()
      + hessianFactor3->augmentedInformation();

  // Check Information vector
  Vector InfoVector = AugInformationMatrix.block(0, 18, 18, 1); // 18x18 Hessian + information vector

  // Check Hessian
  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-8));
}
//
///* *************************************************************************/
//TEST( SmartStereoProjectionPoseFactor, 3poses_2land_rotation_only_smart_projection_factor ){
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//  views.push_back(x3);
//
//  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
//  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), Point3(0,0,1));
//  StereoCamera cam1(pose1, K2);
//
//  // create second camera 1 meter to the right of first camera
//  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  StereoCamera cam2(pose2, K2);
//
//  // create third camera 1 meter above the first camera
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  StereoCamera cam3(pose3, K2);
//
//  // three landmarks ~5 meters infront of camera
//  Point3 landmark1(5, 0.5, 1.2);
//  Point3 landmark2(5, -0.5, 1.2);
//
//  vector<StereoPoint2> measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // 1. Project three landmarks into three cameras and triangulate
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//
//  double rankTol = 50;
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(rankTol, linThreshold, manageDegeneracy));
//  smartFactor1->add(measurements_cam1, views, model, K2);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(rankTol, linThreshold, manageDegeneracy));
//  smartFactor2->add(measurements_cam2, views, model, K2);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
//  Point3 positionPrior = Point3(0,0,1);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
//  graph.push_back(PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
//  graph.push_back(PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.1,0.1,0.1)); // smaller noise
//  Values values;
//  values.insert(x1, pose1);
//  values.insert(x2, pose2*noise_pose);
//  // initialize third pose with some noise, we expect it to move back to original pose3
//  values.insert(x3, pose3*noise_pose*noise_pose);
//
//  Values result;
//  gttic_(SmartStereoProjectionPoseFactor);
//  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
//  result = optimizer.optimize();
//  gttoc_(SmartStereoProjectionPoseFactor);
//  tictoc_finishedIteration_();
//
//  // result.print("results of 3 camera, 3 landmark optimization \n");
//  // EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartStereoProjectionPoseFactor, 3poses_rotation_only_smart_projection_factor ){
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//  views.push_back(x3);
//
//  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
//  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), Point3(0,0,1));
//  StereoCamera cam1(pose1, K);
//
//  // create second camera 1 meter to the right of first camera
//  Pose3 pose2 = pose1 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  StereoCamera cam2(pose2, K);
//
//  // create third camera 1 meter above the first camera
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  StereoCamera cam3(pose3, K);
//
//  // three landmarks ~5 meters infront of camera
//  Point3 landmark1(5, 0.5, 1.2);
//  Point3 landmark2(5, -0.5, 1.2);
//  Point3 landmark3(3, 0, 3.0);
//
//  vector<StereoPoint2> measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // 1. Project three landmarks into three cameras and triangulate
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  stereo_projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  double rankTol = 10;
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor(rankTol, linThreshold, manageDegeneracy));
//  smartFactor1->add(measurements_cam1, views, model, K);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor2(new SmartStereoProjectionPoseFactor(rankTol, linThreshold, manageDegeneracy));
//  smartFactor2->add(measurements_cam2, views, model, K);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor3(new SmartStereoProjectionPoseFactor(rankTol, linThreshold, manageDegeneracy));
//  smartFactor3->add(measurements_cam3, views, model, K);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
//  Point3 positionPrior = Point3(0,0,1);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.push_back(PriorFactor<Pose3>(x1, pose1, noisePrior));
//  graph.push_back(PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
//  graph.push_back(PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/100, 0., -M_PI/100), Point3(0.1,0.1,0.1)); // smaller noise
//  Values values;
//  values.insert(x1, pose1);
//  values.insert(x2, pose2);
//  // initialize third pose with some noise, we expect it to move back to original pose3
//  values.insert(x3, pose3*noise_pose);
//
//  Values result;
//  gttic_(SmartStereoProjectionPoseFactor);
//  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
//  result = optimizer.optimize();
//  gttoc_(SmartStereoProjectionPoseFactor);
//  tictoc_finishedIteration_();
//
//  // result.print("results of 3 camera, 3 landmark optimization \n");
//  // EXPECT(assert_equal(pose3,result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartStereoProjectionPoseFactor, Hessian ){
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//
//  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
//  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), Point3(0,0,1));
//  StereoCamera cam1(pose1, K2);
//
//  // create second camera 1 meter to the right of first camera
//  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
//  StereoCamera cam2(pose2, K2);
//
//  // three landmarks ~5 meters infront of camera
//  Point3 landmark1(5, 0.5, 1.2);
//
//  // 1. Project three landmarks into three cameras and triangulate
//  StereoPoint2 cam1_uv1 = cam1.project(landmark1);
//  StereoPoint2 cam2_uv1 = cam2.project(landmark1);
//  vector<StereoPoint2> measurements_cam1;
//  measurements_cam1.push_back(cam1_uv1);
//  measurements_cam1.push_back(cam2_uv1);
//
//  SmartStereoProjectionPoseFactor::shared_ptr smartFactor1(new SmartStereoProjectionPoseFactor());
//  smartFactor1->add(measurements_cam1,views, model, K2);
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3));
//  Values values;
//  values.insert(x1, pose1);
//  values.insert(x2, pose2);
//
//  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor1->linearize(values);
//
//  // compute triangulation from linearization point
//  // compute reprojection errors (sum squared)
//  // compare with hessianFactor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
//  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
//}
//

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, HessianWithRotation ) {
  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  Point3 landmark1(5, 0.5, 1.2);

  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactorInstance(new SmartStereoProjectionPoseFactor(model));
  smartFactorInstance->add(measurements_cam1, views, K);

  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  boost::shared_ptr<GaussianFactor> hessianFactor =
      smartFactorInstance->linearize(values);
  // hessianFactor->print("Hessian factor \n");

  Pose3 poseDrift = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(pose1));
  rotValues.insert(x2, poseDrift.compose(pose2));
  rotValues.insert(x3, poseDrift.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRot =
      smartFactorInstance->linearize(rotValues);
  // hessianFactorRot->print("Hessian factor \n");

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRot->information(), 1e-7));

  Pose3 poseDrift2 = Pose3(Rot3::Ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(pose1));
  tranValues.insert(x2, poseDrift2.compose(pose2));
  tranValues.insert(x3, poseDrift2.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran =
      smartFactorInstance->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRotTran->information(), 1e-6));
}

/* *************************************************************************/
TEST( SmartStereoProjectionPoseFactor, HessianWithRotationNonDegenerate ) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K2);

  // Second and third cameras in same place, which is a degenerate configuration
  Pose3 pose2 = pose1;
  Pose3 pose3 = pose1;
  StereoCamera cam2(pose2, K2);
  StereoCamera cam3(pose3, K2);

  Point3 landmark1(5, 0.5, 1.2);

  vector<StereoPoint2> measurements_cam1 = stereo_projectToMultipleCameras(cam1,
      cam2, cam3, landmark1);

  SmartStereoProjectionPoseFactor::shared_ptr smartFactor(new SmartStereoProjectionPoseFactor(model));
  smartFactor->add(measurements_cam1, views, K2);

  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor->linearize(
      values);

  // check that it is non degenerate
  EXPECT(smartFactor->isValid());

  Pose3 poseDrift = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(pose1));
  rotValues.insert(x2, poseDrift.compose(pose2));
  rotValues.insert(x3, poseDrift.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRot = smartFactor->linearize(
      rotValues);

  // check that it is non degenerate
  EXPECT(smartFactor->isValid());

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRot->information(), 1e-6));

  Pose3 poseDrift2 = Pose3(Rot3::Ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(pose1));
  tranValues.insert(x2, poseDrift2.compose(pose2));
  tranValues.insert(x3, poseDrift2.compose(pose3));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran =
      smartFactor->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the degenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
#ifdef GTSAM_USE_EIGEN_MKL
          hessianFactorRotTran->information(), 1e-5));
#else
          hessianFactorRotTran->information(), 1e-6));
#endif
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

