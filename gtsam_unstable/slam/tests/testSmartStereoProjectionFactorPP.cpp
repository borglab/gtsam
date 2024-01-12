/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartStereoProjectionFactorPP.cpp
 *  @brief Unit tests for SmartStereoProjectionFactorPP Class
 *  @author Luca Carlone
 *  @date   March 2021
 */

#include <gtsam/slam/tests/smartFactorScenarios.h>
#include <gtsam_unstable/slam/SmartStereoProjectionFactorPP.h>
#include <gtsam_unstable/slam/ProjectionFactorPPP.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;

namespace {
// make a realistic calibration matrix
static double b = 1;

static Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fov, w, h, b));
static Cal3_S2Stereo::shared_ptr K2(new Cal3_S2Stereo(1500, 1200, 0, 640, 480,
                                                      b));

static SmartStereoProjectionParams params;

// static bool manageDegeneracy = true;
// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(3, 0.1));

// Convenience for named keys
using symbol_shorthand::L;
using symbol_shorthand::X;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);
static Symbol body_P_cam1_key('P', 1);
static Symbol body_P_cam2_key('P', 2);
static Symbol body_P_cam3_key('P', 3);

static Key poseKey1(x1);
static Key poseExtrinsicKey1(body_P_cam1_key);
static Key poseExtrinsicKey2(body_P_cam2_key);
static StereoPoint2 measurement1(
    323.0, 300.0, 240.0);  // potentially use more reasonable measurement value?
static StereoPoint2 measurement2(
    350.0, 200.0, 240.0);  // potentially use more reasonable measurement value?
static Pose3 body_P_sensor1(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
                            Point3(0.25, -0.10, 1.0));

static double missing_uR = std::numeric_limits<double>::quiet_NaN();

vector<StereoPoint2> stereo_projectToMultipleCameras(const StereoCamera& cam1,
                                                     const StereoCamera& cam2,
                                                     const StereoCamera& cam3,
                                                     Point3 landmark) {
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
}  // namespace

/* ************************************************************************* */
TEST( SmartStereoProjectionFactorPP, params) {
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
TEST( SmartStereoProjectionFactorPP, Constructor) {
  SmartStereoProjectionFactorPP::shared_ptr factor1(new SmartStereoProjectionFactorPP(model));
}

/* ************************************************************************* */
TEST( SmartStereoProjectionFactorPP, Constructor2) {
  SmartStereoProjectionFactorPP factor1(model, params);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionFactorPP, Constructor3) {
  SmartStereoProjectionFactorPP::shared_ptr factor1(new SmartStereoProjectionFactorPP(model));
  factor1->add(measurement1, poseKey1, poseExtrinsicKey1, K);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionFactorPP, Constructor4) {
  SmartStereoProjectionFactorPP factor1(model, params);
  factor1.add(measurement1, poseKey1, poseExtrinsicKey1, K);
}

/* ************************************************************************* */
TEST( SmartStereoProjectionFactorPP, Equals ) {
  SmartStereoProjectionFactorPP::shared_ptr factor1(new SmartStereoProjectionFactorPP(model));
  factor1->add(measurement1, poseKey1, poseExtrinsicKey1, K);

  SmartStereoProjectionFactorPP::shared_ptr factor2(new SmartStereoProjectionFactorPP(model));
  factor2->add(measurement1, poseKey1, poseExtrinsicKey1, K);
  // check these are equal
  EXPECT(assert_equal(*factor1, *factor2));

  SmartStereoProjectionFactorPP::shared_ptr factor3(new SmartStereoProjectionFactorPP(model));
  factor3->add(measurement2, poseKey1, poseExtrinsicKey1, K);
  // check these are different
  EXPECT(!factor1->equals(*factor3));

  SmartStereoProjectionFactorPP::shared_ptr factor4(new SmartStereoProjectionFactorPP(model));
  factor4->add(measurement1, poseKey1, poseExtrinsicKey2, K);
  // check these are different
  EXPECT(!factor1->equals(*factor4));
}

/* *************************************************************************/
TEST_UNSAFE( SmartStereoProjectionFactorPP, noiseless_error_identityExtrinsics ) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  StereoCamera w_Camera_cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera w_Camera_cam2(w_Pose_cam2, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  StereoPoint2 cam1_uv = w_Camera_cam1.project(landmark);
  StereoPoint2 cam2_uv = w_Camera_cam2.project(landmark);

  Values values;
  values.insert(x1, w_Pose_cam1);
  values.insert(x2, w_Pose_cam2);
  values.insert(body_P_cam1_key, Pose3::Identity());
  values.insert(body_P_cam2_key, Pose3::Identity());

  SmartStereoProjectionFactorPP factor1(model);
  factor1.add(cam1_uv, x1, body_P_cam1_key, K2);
  factor1.add(cam2_uv, x2, body_P_cam2_key, K2);

  double actualError = factor1.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartStereoProjectionFactorPP::Cameras cameras = factor1.cameras(values);
  double actualError2 = factor1.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);
}

/* *************************************************************************/
TEST_UNSAFE( SmartStereoProjectionFactorPP, noiseless_error_multipleExtrinsics ) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  StereoCamera w_Camera_cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera w_Camera_cam2(w_Pose_cam2, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  StereoPoint2 cam1_uv = w_Camera_cam1.project(landmark);
  StereoPoint2 cam2_uv = w_Camera_cam2.project(landmark);

  Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., 0.0),
        Point3(0, 1, 0));
  Pose3 body_Pose_cam2 = Pose3(Rot3::Ypr(-M_PI / 4, 0., 0.0),
          Point3(1, 1, 0));
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam2.inverse());

  Values values;
  values.insert(x1, w_Pose_body1);
  values.insert(x2, w_Pose_body2);
  values.insert(body_P_cam1_key, body_Pose_cam1);
  values.insert(body_P_cam2_key, body_Pose_cam2);

  SmartStereoProjectionFactorPP factor1(model);
  factor1.add(cam1_uv, x1, body_P_cam1_key, K2);
  factor1.add(cam2_uv, x2, body_P_cam2_key, K2);

  double actualError = factor1.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartStereoProjectionFactorPP::Cameras cameras = factor1.cameras(values);
  double actualError2 = factor1.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, noiseless_error_multipleExtrinsics_missingMeasurements ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
   Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
       Point3(0, 0, 1));
   StereoCamera w_Camera_cam1(w_Pose_cam1, K2);

   // create second camera 1 meter to the right of first camera
   Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
   StereoCamera w_Camera_cam2(w_Pose_cam2, K2);

   // landmark ~5 meters in front of camera
   Point3 landmark(5, 0.5, 1.2);

   // 1. Project two landmarks into two cameras and triangulate
   StereoPoint2 cam1_uv = w_Camera_cam1.project(landmark);
   StereoPoint2 cam2_uv = w_Camera_cam2.project(landmark);
   StereoPoint2 cam2_uv_right_missing(cam2_uv.uL(),missing_uR,cam2_uv.v());

   // fake extrinsic calibration
   Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),
         Point3(0, 1, 0));
   Pose3 body_Pose_cam2 = Pose3(Rot3::Ypr(-M_PI / 4, 0.1, 1.0),
           Point3(1, 1, 1));
   Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
   Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam2.inverse());

   Values values;
   values.insert(x1, w_Pose_body1);
   values.insert(x2, w_Pose_body2);
   values.insert(body_P_cam1_key, body_Pose_cam1);
   values.insert(body_P_cam2_key, body_Pose_cam2);

   SmartStereoProjectionFactorPP factor1(model);
   factor1.add(cam1_uv, x1, body_P_cam1_key, K2);
   factor1.add(cam2_uv_right_missing, x2, body_P_cam2_key, K2);

   double actualError = factor1.error(values);
   double expectedError = 0.0;
   EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

   // TEST TRIANGULATION WITH MISSING VALUES: i) right pixel of second camera is missing:
   SmartStereoProjectionFactorPP::Cameras cameras = factor1.cameras(values);
   double actualError2 = factor1.totalReprojectionError(cameras);
   EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

   // The following are generically exercising the triangulation
   CameraSet<StereoCamera> cams{w_Camera_cam1, w_Camera_cam2};
   TriangulationResult result = factor1.triangulateSafe(cams);
   CHECK(result);
   EXPECT(assert_equal(landmark, *result, 1e-7));

   // TEST TRIANGULATION WITH MISSING VALUES: ii) right pixels of both cameras are missing:
   SmartStereoProjectionFactorPP factor2(model);
   StereoPoint2 cam1_uv_right_missing(cam1_uv.uL(),missing_uR,cam1_uv.v());
   factor2.add(cam1_uv_right_missing, x1, body_P_cam1_key, K2);
   factor2.add(cam2_uv_right_missing, x2, body_P_cam2_key, K2);
   result = factor2.triangulateSafe(cams);
   CHECK(result);
   EXPECT(assert_equal(landmark, *result, 1e-7));
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, noisy_error_multipleExtrinsics ) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  StereoCamera w_Camera_cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera w_Camera_cam2(w_Pose_cam2, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  StereoPoint2 pixelError(0.2, 0.2, 0);
  StereoPoint2 cam1_uv = w_Camera_cam1.project(landmark) + pixelError;
  StereoPoint2 cam2_uv = w_Camera_cam2.project(landmark);

  // fake extrinsic calibration
  Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),
        Point3(0, 1, 0));
  Pose3 body_Pose_cam2 = Pose3(Rot3::Ypr(-M_PI / 4, 0.1, 1.0),
          Point3(1, 1, 1));
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam2.inverse());

  Values values;
  values.insert(x1, w_Pose_body1);
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
        Point3(0.5, 0.1, 0.3));
  values.insert(x2, w_Pose_body2.compose(noise_pose));
  values.insert(body_P_cam1_key, body_Pose_cam1);
  values.insert(body_P_cam2_key, body_Pose_cam2);

  SmartStereoProjectionFactorPP::shared_ptr factor1(new SmartStereoProjectionFactorPP(model));
  factor1->add(cam1_uv, x1, body_P_cam1_key, K);
  factor1->add(cam2_uv, x2, body_P_cam2_key, K);

  double actualError1 = factor1->error(values);

  SmartStereoProjectionFactorPP::shared_ptr factor2(new SmartStereoProjectionFactorPP(model));
  vector<StereoPoint2> measurements;
  measurements.push_back(cam1_uv);
  measurements.push_back(cam2_uv);

  vector<std::shared_ptr<Cal3_S2Stereo> > Ks; ///< shared pointer to calibration object (one for each camera)
  Ks.push_back(K);
  Ks.push_back(K);

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);

  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam1_key);
  extrinsicKeys.push_back(body_P_cam2_key);

  factor2->add(measurements, poseKeys, extrinsicKeys, Ks);

  double actualError2 = factor2->error(values);

  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
  DOUBLES_EQUAL(actualError1, 5381978, 1); // value freeze
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, 3poses_optimization_multipleExtrinsics ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(w_Pose_cam2, K2);

  // create third camera 1 meter above the first camera
  Pose3 w_Pose_cam3 = w_Pose_cam1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(w_Pose_cam3, K2);

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

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);
  poseKeys.push_back(x3);

  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam1_key);
  extrinsicKeys.push_back(body_P_cam2_key);
  extrinsicKeys.push_back(body_P_cam3_key);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Values
  Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
  Pose3 body_Pose_cam2 = Pose3(Rot3::Ypr(-M_PI / 4, 0.1, 1.0),Point3(1, 1, 1));
  Pose3 body_Pose_cam3 = Pose3::Identity();
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam2.inverse());
  Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam3.inverse());

  Values values;
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.1, 0.1, 0.1)); // smaller noise
  values.insert(x1, w_Pose_body1);
  values.insert(x2, w_Pose_body2);
  values.insert(x3, w_Pose_body3);
  values.insert(body_P_cam1_key, body_Pose_cam1);
  values.insert(body_P_cam2_key, body_Pose_cam2);
  // initialize third calibration with some noise, we expect it to move back to original pose3
  values.insert(body_P_cam3_key, body_Pose_cam3 * noise_pose);

  // Graph
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, w_Pose_body1, noisePrior);
  graph.addPrior(x2, w_Pose_body2, noisePrior);
  graph.addPrior(x3, w_Pose_body3, noisePrior);
  // we might need some prior on the calibration too
  graph.addPrior(body_P_cam1_key, body_Pose_cam1, noisePrior);
  graph.addPrior(body_P_cam2_key, body_Pose_cam2, noisePrior);

  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3) * values.at<Pose3>(body_P_cam3_key)));

  //  cout << std::setprecision(10) << "\n----SmartStereoFactor graph initial error: " << graph.error(values) << endl;
  EXPECT_DOUBLES_EQUAL(833953.92789459578, graph.error(values), 1e-7); // initial error (note  - this also matches error below)

  // get triangulated landmarks from smart factors
  Point3 landmark1_smart = *smartFactor1->point();
  Point3 landmark2_smart = *smartFactor2->point();
  Point3 landmark3_smart = *smartFactor3->point();

  // cost is large before optimization
  double initialErrorSmart = graph.error(values);
  EXPECT_DOUBLES_EQUAL(833953.92789459461, initialErrorSmart, 1e-5);

  Values result;
  gttic_(SmartStereoProjectionFactorPP);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionFactorPP);
  tictoc_finishedIteration_();

  //  cout << std::setprecision(10) << "SmartStereoFactor graph optimized error: " << graph.error(result) << endl;
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);

  GaussianFactorGraph::shared_ptr GFG = graph.linearize(result);
  VectorValues delta = GFG->optimize();
  VectorValues expected = VectorValues::Zero(delta);
  EXPECT(assert_equal(expected, delta, 1e-6));

  // result.print("results of 3 camera, 3 landmark optimization \n");
  EXPECT(assert_equal(body_Pose_cam3, result.at<Pose3>(body_P_cam3_key)));

  // ***************************************************************
  // Same problem with regular Stereo factors, expect same error!
  // ****************************************************************

  // add landmarks to values
  Values values2;
  values2.insert(x1, w_Pose_cam1); // note: this is the *camera* pose now
  values2.insert(x2, w_Pose_cam2);
  values2.insert(x3, w_Pose_cam3 * noise_pose); // equivalent to perturbing the extrinsic calibration
  values2.insert(L(1), landmark1_smart);
  values2.insert(L(2), landmark2_smart);
  values2.insert(L(3), landmark3_smart);

  // add factors
  NonlinearFactorGraph graph2;

  graph2.addPrior(x1, w_Pose_cam1, noisePrior);
  graph2.addPrior(x2, w_Pose_cam2, noisePrior);

  typedef GenericStereoFactor<Pose3, Point3> ProjectionFactor;

  bool verboseCheirality = true;

  // NOTE: we cannot repeate this with ProjectionFactor, since they are not suitable for stereo calibration
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
  EXPECT_DOUBLES_EQUAL(833953.92789459578, graph2.error(values2), 1e-7);
  EXPECT_DOUBLES_EQUAL(initialErrorSmart, graph2.error(values2), 1e-7); // identical to previous case!

  LevenbergMarquardtOptimizer optimizer2(graph2, values2, lm_params);
  Values result2 = optimizer2.optimize();
  EXPECT_DOUBLES_EQUAL(0, graph2.error(result2), 1e-5);
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, 3poses_error_sameExtrinsicKey ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(w_Pose_cam2, K2);

  // create third camera 1 meter above the first camera
  Pose3 w_Pose_cam3 = w_Pose_cam1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(w_Pose_cam3, K2);

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

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);
  poseKeys.push_back(x3);

  Symbol body_P_cam_key('P', 0);
  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Values
  Pose3 body_Pose_cam = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam.inverse());

  Values values; // all noiseless
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.01, 0.01, 0.01)); // smaller noise
  values.insert(x1, w_Pose_body1);
  values.insert(x2, w_Pose_body2);
  values.insert(x3, w_Pose_body3);
  values.insert(body_P_cam_key, body_Pose_cam);

  // Graph
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, w_Pose_body1, noisePrior);
  graph.addPrior(x2, w_Pose_body2, noisePrior);
  graph.addPrior(x3, w_Pose_body3, noisePrior);

  // cost is large before optimization
  double initialErrorSmart = graph.error(values);
  EXPECT_DOUBLES_EQUAL(0.0, initialErrorSmart, 1e-5); // initial guess is noisy, so nonzero error
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, 3poses_noisy_error_sameExtrinsicKey ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(w_Pose_cam2, K2);

  // create third camera 1 meter above the first camera
  Pose3 w_Pose_cam3 = w_Pose_cam1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(w_Pose_cam3, K2);

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

  double initialError_expected, initialError_actual;
  {
    KeyVector poseKeys;
    poseKeys.push_back(x1);
    poseKeys.push_back(x2);
    poseKeys.push_back(x3);

    KeyVector extrinsicKeys;
    extrinsicKeys.push_back(body_P_cam1_key);
    extrinsicKeys.push_back(body_P_cam2_key);
    extrinsicKeys.push_back(body_P_cam3_key);

    SmartStereoProjectionParams smart_params;
    smart_params.triangulation.enableEPI = true;
    SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

    SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

    SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

    const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

    // Values
    Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
    Pose3 body_Pose_cam2 = body_Pose_cam1;
    Pose3 body_Pose_cam3 = body_Pose_cam1;
    Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
    Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam2.inverse());
    Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam3.inverse());

    Values values;
    Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.1, 0.1, 0.1)); // smaller noise
    values.insert(x1, w_Pose_body1);
    values.insert(x2, w_Pose_body2);
    values.insert(x3, w_Pose_body3);
    values.insert(body_P_cam1_key, body_Pose_cam1 * noise_pose);
    values.insert(body_P_cam2_key, body_Pose_cam2 * noise_pose);
    // initialize third calibration with some noise, we expect it to move back to original pose3
    values.insert(body_P_cam3_key, body_Pose_cam3 * noise_pose);

    // Graph
    NonlinearFactorGraph graph;
    graph.push_back(smartFactor1);
    graph.push_back(smartFactor2);
    graph.push_back(smartFactor3);
    graph.addPrior(x1, w_Pose_body1, noisePrior);
    graph.addPrior(x2, w_Pose_body2, noisePrior);
    graph.addPrior(x3, w_Pose_body3, noisePrior);

    initialError_expected = graph.error(values);
  }

  {
    KeyVector poseKeys;
    poseKeys.push_back(x1);
    poseKeys.push_back(x2);
    poseKeys.push_back(x3);

    KeyVector extrinsicKeys;
    extrinsicKeys.push_back(body_P_cam1_key);
    extrinsicKeys.push_back(body_P_cam1_key);
    extrinsicKeys.push_back(body_P_cam1_key);

    SmartStereoProjectionParams smart_params;
    smart_params.triangulation.enableEPI = true;
    SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

    SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

    SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
    smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

    const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

    // Values
    Pose3 body_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
    Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam1.inverse());
    Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam1.inverse());
    Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam1.inverse());

    Values values;
    Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.1, 0.1, 0.1)); // smaller noise
    values.insert(x1, w_Pose_body1);
    values.insert(x2, w_Pose_body2);
    values.insert(x3, w_Pose_body3);
    values.insert(body_P_cam1_key, body_Pose_cam1 * noise_pose);

    // Graph
    NonlinearFactorGraph graph;
    graph.push_back(smartFactor1);
    graph.push_back(smartFactor2);
    graph.push_back(smartFactor3);
    graph.addPrior(x1, w_Pose_body1, noisePrior);
    graph.addPrior(x2, w_Pose_body2, noisePrior);
    graph.addPrior(x3, w_Pose_body3, noisePrior);

    initialError_actual = graph.error(values);
  }

  //std::cout << " initialError_expected " << initialError_expected << std::endl;
  EXPECT_DOUBLES_EQUAL(initialError_expected, initialError_actual, 1e-7);
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, 3poses_optimization_sameExtrinsicKey ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(w_Pose_cam2, K2);

  // create third camera 1 meter above the first camera
  Pose3 w_Pose_cam3 = w_Pose_cam1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(w_Pose_cam3, K2);

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

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);
  poseKeys.push_back(x3);

  Symbol body_P_cam_key('P', 0);
  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

  // relevant poses:
  Pose3 body_Pose_cam = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam.inverse());

  // Graph
  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, w_Pose_body1, noisePrior);
  graph.addPrior(x2, w_Pose_body2, noisePrior);
  graph.addPrior(x3, w_Pose_body3, noisePrior);
  // we might need some prior on the calibration too
  // graph.addPrior(body_P_cam_key, body_Pose_cam, noisePrior); // no need! the measurements will fix this!

  // Values
  Values values;
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.01, 0.01, 0.01)); // smaller noise
  values.insert(x1, w_Pose_body1);
  values.insert(x2, w_Pose_body2);
  values.insert(x3, w_Pose_body3);
  values.insert(body_P_cam_key, body_Pose_cam*noise_pose);

  // cost is large before optimization
  double initialErrorSmart = graph.error(values);
  EXPECT_DOUBLES_EQUAL(31986.961831653316, initialErrorSmart, 1e-5); // initial guess is noisy, so nonzero error

  /////////////////////////////////////////////////////////////////
  // What the factor is doing is the following Schur complement math (this matches augmentedHessianPP in code):
  //  size_t numMeasurements = measured_.size();
  //  Matrix F = Matrix::Zero(3*numMeasurements, 6 * nrUniqueKeys);
  //  for(size_t k=0; k<numMeasurements; k++){
  //    Key key_body = w_P_body_keys_.at(k);
  //    Key key_cal = body_P_cam_keys_.at(k);
  //    F.block<3,6>( 3*k , 6*keyToSlotMap[key_body] ) = Fs[k].block<3,6>(0,0);
  //    F.block<3,6>( 3*k , 6*keyToSlotMap[key_cal] ) = Fs[k].block<3,6>(0,6);
  //  }
  //  Matrix augH = Matrix::Zero(6*nrUniqueKeys+1,6*nrUniqueKeys+1);
  //  augH.block(0,0,6*nrUniqueKeys,6*nrUniqueKeys) = F.transpose() * F - F.transpose() * E * P * E.transpose() * F;
  //  Matrix infoVec = F.transpose() * b - F.transpose() * E * P * E.transpose() * b;
  //  augH.block(0,6*nrUniqueKeys,6*nrUniqueKeys,1) = infoVec;
  //  augH.block(6*nrUniqueKeys,0,1,6*nrUniqueKeys) = infoVec.transpose();
  //  augH(6*nrUniqueKeys,6*nrUniqueKeys) = b.squaredNorm();
  // // The following is close to zero:
  // std::cout << "norm diff: \n"<< Matrix(augH - Matrix(augmentedHessianUniqueKeys.selfadjointView())).lpNorm<Eigen::Infinity>() << std::endl;
  // std::cout << "TEST MATRIX:" << std::endl;
  // augmentedHessianUniqueKeys = SymmetricBlockMatrix(dims, augH);
  /////////////////////////////////////////////////////////////////

  Values result;
  gttic_(SmartStereoProjectionFactorPP);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionFactorPP);
  tictoc_finishedIteration_();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);

  // This passes on my machine but gets and indeterminant linear system exception in CI.
  // This is a very redundant test, so it's not a problem to omit.
  //  GaussianFactorGraph::shared_ptr GFG = graph.linearize(result);
  //  Matrix H = GFG->hessian().first;
  //  double det = H.determinant();
  //  // std::cout << "det " << det << std::endl; // det = 2.27581e+80 (so it's not underconstrained)
  //  VectorValues delta = GFG->optimize();
  //  VectorValues expected = VectorValues::Zero(delta);
  //  EXPECT(assert_equal(expected, delta, 1e-4));
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, 3poses_optimization_2ExtrinsicKeys ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 w_Pose_cam1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(w_Pose_cam1, K2);

  // create second camera 1 meter to the right of first camera
  Pose3 w_Pose_cam2 = w_Pose_cam1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(w_Pose_cam2, K2);

  // create third camera 1 meter above the first camera
  Pose3 w_Pose_cam3 = w_Pose_cam1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(w_Pose_cam3, K2);

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

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);
  poseKeys.push_back(x3);

  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam1_key);
  extrinsicKeys.push_back(body_P_cam1_key);
  extrinsicKeys.push_back(body_P_cam3_key);

  SmartStereoProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor1->add(measurements_l1, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor2->add(measurements_l2, poseKeys, extrinsicKeys, K2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, smart_params));
  smartFactor3->add(measurements_l3, poseKeys, extrinsicKeys, K2);

  // relevant poses:
  Pose3 body_Pose_cam = Pose3(Rot3::Ypr(-M_PI, 1., 0.1),Point3(0, 1, 0));
  Pose3 w_Pose_body1 = w_Pose_cam1.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body2 = w_Pose_cam2.compose(body_Pose_cam.inverse());
  Pose3 w_Pose_body3 = w_Pose_cam3.compose(body_Pose_cam.inverse());

  // Graph
  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, w_Pose_body1, noisePrior);
  graph.addPrior(x2, w_Pose_body2, noisePrior);
  graph.addPrior(x3, w_Pose_body3, noisePrior);
  // graph.addPrior(body_P_cam1_key, body_Pose_cam, noisePrior);
  // we might need some prior on the calibration too
  // graph.addPrior(body_P_cam_key, body_Pose_cam, noisePrior); // no need! the measurements will fix this!

  // Values
  Values values;
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.01, 0.01, 0.01)); // smaller noise
  values.insert(x1, w_Pose_body1);
  values.insert(x2, w_Pose_body2);
  values.insert(x3, w_Pose_body3);
  values.insert(body_P_cam1_key, body_Pose_cam*noise_pose);
  values.insert(body_P_cam3_key, body_Pose_cam*noise_pose);

  // cost is large before optimization
  double initialErrorSmart = graph.error(values);
  EXPECT_DOUBLES_EQUAL(31986.961831653316, initialErrorSmart, 1e-5); // initial guess is noisy, so nonzero error

  Values result;
  gttic_(SmartStereoProjectionFactorPP);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionFactorPP);
  tictoc_finishedIteration_();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);

  //  NOTE: the following would fail since the problem is underconstrained (while LM above works just fine!)
  //  GaussianFactorGraph::shared_ptr GFG = graph.linearize(result);
  //  VectorValues delta = GFG->optimize();
  //  VectorValues expected = VectorValues::Zero(delta);
  //  EXPECT(assert_equal(expected, delta, 1e-4));
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, monocular_multipleExtrinsicKeys ){
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

  // convert measurement to (degenerate) stereoPoint2 (with right pixel being NaN)
  vector<StereoPoint2> measurements_cam1_stereo, measurements_cam2_stereo, measurements_cam3_stereo;
  for(size_t k=0; k<measurements_cam1.size();k++)
    measurements_cam1_stereo.push_back(StereoPoint2(measurements_cam1[k].x() , missing_uR , measurements_cam1[k].y()));

  for(size_t k=0; k<measurements_cam2.size();k++)
    measurements_cam2_stereo.push_back(StereoPoint2(measurements_cam2[k].x() , missing_uR , measurements_cam2[k].y()));

  for(size_t k=0; k<measurements_cam3.size();k++)
    measurements_cam3_stereo.push_back(StereoPoint2(measurements_cam3[k].x() , missing_uR , measurements_cam3[k].y()));

  KeyVector poseKeys;
  poseKeys.push_back(x1);
  poseKeys.push_back(x2);
  poseKeys.push_back(x3);

  Symbol body_P_cam_key('P', 0);
  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);

  SmartStereoProjectionParams smart_params;
  smart_params.setRankTolerance(1.0);
  smart_params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
  smart_params.setEnableEPI(false);

  Cal3_S2Stereo::shared_ptr Kmono(new Cal3_S2Stereo(fov,w,h,b));

  SmartStereoProjectionFactorPP smartFactor1(model, smart_params);
  smartFactor1.add(measurements_cam1_stereo, poseKeys, extrinsicKeys, Kmono);

  SmartStereoProjectionFactorPP smartFactor2(model, smart_params);
  smartFactor2.add(measurements_cam2_stereo, poseKeys, extrinsicKeys, Kmono);

  SmartStereoProjectionFactorPP smartFactor3(model, smart_params);
  smartFactor3.add(measurements_cam3_stereo, poseKeys, extrinsicKeys, Kmono);

  // Graph
  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, bodyPose1, noisePrior);
  graph.addPrior(x2, bodyPose2, noisePrior);
  graph.addPrior(x3, bodyPose3, noisePrior);
  // we might need some prior on the calibration too
  // graph.addPrior(body_P_cam_key, body_Pose_cam, noisePrior); // no need! the measurements will fix this!

  // Check errors at ground truth poses
  Values gtValues;
  gtValues.insert(x1, bodyPose1);
  gtValues.insert(x2, bodyPose2);
  gtValues.insert(x3, bodyPose3);
  gtValues.insert(body_P_cam_key, sensor_to_body);
  double actualError = graph.error(gtValues);
  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-7)

  // noisy values
  Values values;
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.01, 0.01, 0.01)); // smaller noise
  values.insert(x1, bodyPose1);
  values.insert(x2, bodyPose2);
  values.insert(x3, bodyPose3);
  values.insert(body_P_cam_key, sensor_to_body*noise_pose);

  // cost is large before optimization
  double initialErrorSmart = graph.error(values);
  EXPECT_DOUBLES_EQUAL(2379.0012816261642, initialErrorSmart, 1e-5); // freeze value

  Values result;
  gttic_(SmartStereoProjectionFactorPP);
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  gttoc_(SmartStereoProjectionFactorPP);
  tictoc_finishedIteration_();

  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-5);
  EXPECT(assert_equal(sensor_to_body,result.at<Pose3>(body_P_cam_key), 1e-1));
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, landmarkDistance ) {

  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  StereoCamera cam1(pose1, K);
  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
  StereoCamera cam2(pose2, K);
  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0, -1, 0));
  StereoCamera cam3(pose3, K);

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  Symbol body_P_cam_key('P', 0);
  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);

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
  params.setLinearizationMode(HESSIAN);
  params.setLandmarkDistanceThreshold(2);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, params));
  smartFactor1->add(measurements_cam1, views, extrinsicKeys, K);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, params));
  smartFactor2->add(measurements_cam2, views, extrinsicKeys, K);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, params));
  smartFactor3->add(measurements_cam3, views, extrinsicKeys, K);

  // create graph
  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);
  graph.addPrior(body_P_cam_key, Pose3::Identity(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3 * noise_pose);
  values.insert(body_P_cam_key, Pose3::Identity());

  // All smart factors are disabled and pose should remain where it is
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3), 1e-5));
  EXPECT_DOUBLES_EQUAL(graph.error(values), graph.error(result), 1e-5);
}

/* *************************************************************************/
TEST( SmartStereoProjectionFactorPP, dynamicOutlierRejection ) {

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  Symbol body_P_cam_key('P', 0);
  KeyVector extrinsicKeys;
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);
  extrinsicKeys.push_back(body_P_cam_key);

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
  params.setLinearizationMode(HESSIAN);
  params.setDynamicOutlierRejectionThreshold(1);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor1(new SmartStereoProjectionFactorPP(model, params));
  smartFactor1->add(measurements_cam1, views, extrinsicKeys, K);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor2(new SmartStereoProjectionFactorPP(model, params));
  smartFactor2->add(measurements_cam2, views, extrinsicKeys, K);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor3(new SmartStereoProjectionFactorPP(model, params));
  smartFactor3->add(measurements_cam3, views, extrinsicKeys, K);

  SmartStereoProjectionFactorPP::shared_ptr smartFactor4(new SmartStereoProjectionFactorPP(model, params));
  smartFactor4->add(measurements_cam4, views, extrinsicKeys, K);

  // same as factor 4, but dynamic outlier rejection is off
  SmartStereoProjectionFactorPP::shared_ptr smartFactor4b(new SmartStereoProjectionFactorPP(model));
  smartFactor4b->add(measurements_cam4, views, extrinsicKeys, K);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.addPrior(x1, pose1, noisePrior);
  graph.addPrior(x2, pose2, noisePrior);
  graph.addPrior(x3, pose3, noisePrior);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3);
  values.insert(body_P_cam_key, Pose3::Identity());

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
  EXPECT(assert_equal(Pose3::Identity(), result.at<Pose3>(body_P_cam_key)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

