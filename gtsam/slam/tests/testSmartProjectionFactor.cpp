/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionFactor.cpp
 *  @brief Unit tests for SmartProjectionFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @author Frank Dellaert
 *  @date   Sept 2013
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;

static const bool isDebugTest = false;
static const Symbol l1('l', 1), l2('l', 2), l3('l', 3);
static const Key c1 = 1, c2 = 2, c3 = 3;
static const Point2 measurement1(323.0, 240.0);
static const double rankTol = 1.0;

template<class CALIBRATION>
PinholeCamera<CALIBRATION> perturbCameraPoseAndCalibration(
    const PinholeCamera<CALIBRATION>& camera) {
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 cameraPose = camera.pose();
  Pose3 perturbedCameraPose = cameraPose.compose(noise_pose);
  typename gtsam::traits<CALIBRATION>::TangentVector d;
  d.setRandom();
  d *= 0.1;
  CALIBRATION perturbedCalibration = camera.calibration().retract(d);
  return PinholeCamera<CALIBRATION>(perturbedCameraPose, perturbedCalibration);
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, perturbCameraPose) {
  using namespace vanilla;
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 perturbed_level_pose = level_pose.compose(noise_pose);
  Camera actualCamera(perturbed_level_pose, K2);

  Camera expectedCamera = perturbCameraPose(level_camera);
  CHECK(assert_equal(expectedCamera, actualCamera));
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, Constructor) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, Constructor2) {
  using namespace vanilla;
  params.setRankTolerance(rankTol);
  SmartFactor factor1(unit2, params);
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, Constructor3) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(measurement1, c1);
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, Constructor4) {
  using namespace vanilla;
  params.setRankTolerance(rankTol);
  SmartFactor factor1(unit2, params);
  factor1.add(measurement1, c1);
}

/* ************************************************************************* */
TEST(SmartProjectionFactor, Equals ) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(measurement1, c1);

  SmartFactor::shared_ptr factor2(new SmartFactor(unit2));
  factor2->add(measurement1, c1);
}

/* *************************************************************************/
TEST(SmartProjectionFactor, noiseless ) {
  using namespace vanilla;

  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);

  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, factor1->error(values), 1e-7);
  CHECK(
      assert_equal(Z_4x1,
          factor1->reprojectionErrorAfterTriangulation(values), 1e-7));
}

/* *************************************************************************/
TEST(SmartProjectionFactor, noisy ) {

  using namespace vanilla;

  // Project one landmark into two cameras and add noise on first
  Point2 level_uv = level_camera.project(landmark1) + Point2(0.2, 0.2);
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(c1, level_camera);
  Camera perturbed_level_camera_right = perturbCameraPose(level_camera_right);
  values.insert(c2, perturbed_level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);

  // Point is now uninitialized before a triangulation event
  EXPECT(!factor1->point());

  double expectedError = 58640;
  double actualError1 = factor1->error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError1, 1);

  // Check triangulated point
  CHECK(factor1->point());
  EXPECT(
      assert_equal(Point3(13.7587, 1.43851, -1.14274), *factor1->point(), 1e-4));

  // Check whitened errors
  Vector expected(4);
  expected << -7, 235, 58, -242;
  SmartFactor::Cameras cameras1 = factor1->cameras(values);
  Point3 point1 = *factor1->point();
  Vector actual = factor1->whitenedError(cameras1, point1);
  EXPECT(assert_equal(expected, actual, 1));

  SmartFactor::shared_ptr factor2(new SmartFactor(unit2));
  Point2Vector measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  KeyVector views {c1, c2};

  factor2->add(measurements, views);

  double actualError2 = factor2->error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1);
}

/* *************************************************************************/
TEST(SmartProjectionFactor, perturbPoseAndOptimize ) {

  using namespace vanilla;

  // Project three landmarks into three cameras
  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create and fill smartfactors
  SmartFactor::shared_ptr smartFactor1(new SmartFactor(unit2));
  SmartFactor::shared_ptr smartFactor2(new SmartFactor(unit2));
  SmartFactor::shared_ptr smartFactor3(new SmartFactor(unit2));
  KeyVector views {c1, c2, c3};
  smartFactor1->add(measurements_cam1, views);
  smartFactor2->add(measurements_cam2, views);
  smartFactor3->add(measurements_cam3, views);

  // Create factor graph and add priors on two cameras
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);
  graph.addPrior(c1, cam1, noisePrior);
  graph.addPrior(c2, cam2, noisePrior);

  // Create initial estimate
  Values initial;
  initial.insert(c1, cam1);
  initial.insert(c2, cam2);
  initial.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    initial.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  // Points are now uninitialized before a triangulation event
  EXPECT(!smartFactor1->point());
  EXPECT(!smartFactor2->point());
  EXPECT(!smartFactor3->point());

  EXPECT_DOUBLES_EQUAL(75711, smartFactor1->error(initial), 1);
  EXPECT_DOUBLES_EQUAL(58524, smartFactor2->error(initial), 1);
  EXPECT_DOUBLES_EQUAL(77564, smartFactor3->error(initial), 1);

  // Error should trigger this and initialize the points, abort if not so
  CHECK(smartFactor1->point());
  CHECK(smartFactor2->point());
  CHECK(smartFactor3->point());

  EXPECT(
      assert_equal(Point3(2.57696, -0.182566, 1.04085), *smartFactor1->point(),
          1e-4));
  EXPECT(
      assert_equal(Point3(2.80114, -0.702153, 1.06594), *smartFactor2->point(),
          1e-4));
  EXPECT(
      assert_equal(Point3(1.82593, -0.289569, 2.13438), *smartFactor3->point(),
          1e-4));

  // Check whitened errors
  Vector expected(6);
  expected << 256, 29, -26, 29, -206, -202;
  Point3 point1 = *smartFactor1->point();
  SmartFactor::Cameras cameras1 = smartFactor1->cameras(initial);
  Vector reprojectionError = cameras1.reprojectionError(point1,
      measurements_cam1);
  EXPECT(assert_equal(expected, reprojectionError, 1));
  Vector actual = smartFactor1->whitenedError(cameras1, point1);
  EXPECT(assert_equal(expected, actual, 1));

  // Optimize
  LevenbergMarquardtParams lmParams;
  if (isDebugTest) {
    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
    lmParams.verbosity = NonlinearOptimizerParams::ERROR;
  }
  LevenbergMarquardtOptimizer optimizer(graph, initial, lmParams);
  Values result = optimizer.optimize();

  EXPECT(assert_equal(landmark1, *smartFactor1->point(), 1e-5));
  EXPECT(assert_equal(landmark2, *smartFactor2->point(), 1e-5));
  EXPECT(assert_equal(landmark3, *smartFactor3->point(), 1e-5));

  //  GaussianFactorGraph::shared_ptr GFG = graph.linearize(initial);
  //  VectorValues delta = GFG->optimize();

  if (isDebugTest)
    result.at<Camera>(c3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(cam1, result.at<Camera>(c1)));
  EXPECT(assert_equal(cam2, result.at<Camera>(c2)));
  EXPECT(assert_equal(result.at<Camera>(c3).pose(), cam3.pose(), 1e-4));
  EXPECT(
      assert_equal(result.at<Camera>(c3).calibration(), cam3.calibration(), 2));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST(SmartProjectionFactor, perturbPoseAndOptimizeFromSfM_tracks ) {

  using namespace vanilla;

  // Project three landmarks into three cameras
  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  KeyVector views {c1, c2, c3};

  SfmTrack track1;
  for (size_t i = 0; i < 3; ++i) {
    track1.measurements.emplace_back(i + 1, measurements_cam1.at(i));
  }
  SmartFactor::shared_ptr smartFactor1(new SmartFactor(unit2));
  smartFactor1->add(track1);

  SfmTrack track2;
  for (size_t i = 0; i < 3; ++i) {
    track2.measurements.emplace_back(i + 1, measurements_cam2.at(i));
  }
  SmartFactor::shared_ptr smartFactor2(new SmartFactor(unit2));
  smartFactor2->add(track2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(unit2));
  smartFactor3->add(measurements_cam3, views);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(c1, cam1, noisePrior);
  graph.addPrior(c2, cam2, noisePrior);

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  values.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams lmParams;
  if (isDebugTest)
    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    lmParams.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();

  //  GaussianFactorGraph::shared_ptr GFG = graph.linearize(values);
  //  VectorValues delta = GFG->optimize();

  if (isDebugTest)
    result.at<Camera>(c3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(cam1, result.at<Camera>(c1)));
  EXPECT(assert_equal(cam2, result.at<Camera>(c2)));
  EXPECT(assert_equal(result.at<Camera>(c3).pose(), cam3.pose(), 1e-4));
  EXPECT(
      assert_equal(result.at<Camera>(c3).calibration(), cam3.calibration(), 2));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST(SmartProjectionFactor, perturbCamerasAndOptimize ) {

  using namespace vanilla;

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  KeyVector views {c1, c2, c3};

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(unit2));
  smartFactor1->add(measurements_cam1, views);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(unit2));
  smartFactor2->add(measurements_cam2, views);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(unit2));
  smartFactor3->add(measurements_cam3, views);

  SmartFactor::shared_ptr smartFactor4(new SmartFactor(unit2));
  smartFactor4->add(measurements_cam4, views);

  SmartFactor::shared_ptr smartFactor5(new SmartFactor(unit2));
  smartFactor5->add(measurements_cam5, views);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.push_back(smartFactor5);
  graph.addPrior(c1, cam1, noisePrior);
  graph.addPrior(c2, cam2, noisePrior);

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  values.insert(c3, perturbCameraPoseAndCalibration(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams lmParams;
  lmParams.relativeErrorTol = 1e-8;
  lmParams.absoluteErrorTol = 0;
  lmParams.maxIterations = 20;
  if (isDebugTest)
    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    lmParams.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();

  //  GaussianFactorGraph::shared_ptr GFG = graph.linearize(values);
  //  VectorValues delta = GFG->optimize();

  if (isDebugTest)
    result.at<Camera>(c3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(cam1, result.at<Camera>(c1)));
  EXPECT(assert_equal(cam2, result.at<Camera>(c2)));
  EXPECT(assert_equal(result.at<Camera>(c3).pose(), cam3.pose(), 1e-1));
  EXPECT(
      assert_equal(result.at<Camera>(c3).calibration(), cam3.calibration(), 20));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST(SmartProjectionFactor, Cal3Bundler ) {

  using namespace bundler;

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  KeyVector views {c1, c2, c3};

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(unit2));
  smartFactor1->add(measurements_cam1, views);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(unit2));
  smartFactor2->add(measurements_cam2, views);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(unit2));
  smartFactor3->add(measurements_cam3, views);

  SmartFactor::shared_ptr smartFactor4(new SmartFactor(unit2));
  smartFactor4->add(measurements_cam4, views);

  SmartFactor::shared_ptr smartFactor5(new SmartFactor(unit2));
  smartFactor5->add(measurements_cam5, views);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(9, 1e-6);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(c1, cam1, noisePrior);
  graph.addPrior(c2, cam2, noisePrior);

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams lmParams;
  lmParams.relativeErrorTol = 1e-8;
  lmParams.absoluteErrorTol = 0;
  lmParams.maxIterations = 20;
  if (isDebugTest)
    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    lmParams.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();

  if (isDebugTest)
    result.at<Camera>(c3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(cam1, result.at<Camera>(c1), 1e-4));
  EXPECT(assert_equal(cam2, result.at<Camera>(c2), 1e-4));
  EXPECT(assert_equal(result.at<Camera>(c3).pose(), cam3.pose(), 1e-1));
  EXPECT(
      assert_equal(result.at<Camera>(c3).calibration(), cam3.calibration(), 1));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST(SmartProjectionFactor, Cal3Bundler2 ) {

  using namespace bundler;

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  KeyVector views {c1, c2, c3};

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(unit2));
  smartFactor1->add(measurements_cam1, views);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(unit2));
  smartFactor2->add(measurements_cam2, views);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(unit2));
  smartFactor3->add(measurements_cam3, views);

  SmartFactor::shared_ptr smartFactor4(new SmartFactor(unit2));
  smartFactor4->add(measurements_cam4, views);

  SmartFactor::shared_ptr smartFactor5(new SmartFactor(unit2));
  smartFactor5->add(measurements_cam5, views);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(9, 1e-6);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(c1, cam1, noisePrior);
  graph.addPrior(c2, cam2, noisePrior);

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(c3, perturbCameraPoseAndCalibration(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams lmParams;
  lmParams.relativeErrorTol = 1e-8;
  lmParams.absoluteErrorTol = 0;
  lmParams.maxIterations = 20;
  if (isDebugTest)
    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    lmParams.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();

  if (isDebugTest)
    result.at<Camera>(c3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(cam1, result.at<Camera>(c1), 1e-4));
  EXPECT(assert_equal(cam2, result.at<Camera>(c2), 1e-4));
  EXPECT(assert_equal(result.at<Camera>(c3).pose(), cam3.pose(), 1e-1));
  EXPECT(
      assert_equal(result.at<Camera>(c3).calibration(), cam3.calibration(), 2));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST(SmartProjectionFactor, noiselessBundler ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);

  double actualError = factor1->error(values);

  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-3);

  Point3 oldPoint(0,0,0); // this takes the point stored in the factor (we are not interested in this)
  if (factor1->point())
    oldPoint = *(factor1->point());

  Point3 expectedPoint(0,0,0);
  if (factor1->point(values))
    expectedPoint = *(factor1->point(values));

  EXPECT(assert_equal(expectedPoint, landmark1, 1e-3));
}

/* *************************************************************************/
TEST(SmartProjectionFactor, comparisonGeneralSfMFactor ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  NonlinearFactorGraph smartGraph;
  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);
  smartGraph.push_back(factor1);
  double expectedError = factor1->error(values);
  double expectedErrorGraph = smartGraph.error(values);
  Point3 expectedPoint(0,0,0);
  if (factor1->point())
    expectedPoint = *(factor1->point());

  // COMMENTS:
  // 1) triangulation introduces small errors, then for a fair comparison we use expectedPoint as
  // value in the generalGrap
  NonlinearFactorGraph generalGraph;
  SFMFactor sfm1(level_uv, unit2, c1, l1);
  SFMFactor sfm2(level_uv_right, unit2, c2, l1);
  generalGraph.push_back(sfm1);
  generalGraph.push_back(sfm2);
  values.insert(l1, expectedPoint); // note: we get rid of possible errors in the triangulation
  Vector e1 = sfm1.evaluateError(values.at<Camera>(c1), values.at<Point3>(l1));
  Vector e2 = sfm2.evaluateError(values.at<Camera>(c2), values.at<Point3>(l1));
  double actualError = 0.5 * (e1.squaredNorm() + e2.squaredNorm());
  double actualErrorGraph = generalGraph.error(values);

  DOUBLES_EQUAL(expectedErrorGraph, actualErrorGraph, 1e-7);
  DOUBLES_EQUAL(expectedErrorGraph, expectedError, 1e-7);
  DOUBLES_EQUAL(actualErrorGraph, actualError, 1e-7);
  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionFactor, comparisonGeneralSfMFactor1 ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  NonlinearFactorGraph smartGraph;
  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);
  smartGraph.push_back(factor1);
  Matrix expectedHessian = smartGraph.linearize(values)->hessian().first;
  Vector expectedInfoVector = smartGraph.linearize(values)->hessian().second;
  Point3 expectedPoint(0,0,0);
  if (factor1->point())
    expectedPoint = *(factor1->point());

  // COMMENTS:
  // 1) triangulation introduces small errors, then for a fair comparison we use expectedPoint as
  // value in the generalGrap
  NonlinearFactorGraph generalGraph;
  SFMFactor sfm1(level_uv, unit2, c1, l1);
  SFMFactor sfm2(level_uv_right, unit2, c2, l1);
  generalGraph.push_back(sfm1);
  generalGraph.push_back(sfm2);
  values.insert(l1, expectedPoint); // note: we get rid of possible errors in the triangulation
  Matrix actualFullHessian = generalGraph.linearize(values)->hessian().first;
  Matrix actualFullInfoVector = generalGraph.linearize(values)->hessian().second;
  Matrix actualHessian = actualFullHessian.block(0, 0, 18, 18)
      - actualFullHessian.block(0, 18, 18, 3)
          * (actualFullHessian.block(18, 18, 3, 3)).inverse()
          * actualFullHessian.block(18, 0, 3, 18);
  Vector actualInfoVector = actualFullInfoVector.block(0, 0, 18, 1)
      - actualFullHessian.block(0, 18, 18, 3)
          * (actualFullHessian.block(18, 18, 3, 3)).inverse()
          * actualFullInfoVector.block(18, 0, 3, 1);

  EXPECT(assert_equal(expectedHessian, actualHessian, 1e-7));
  EXPECT(assert_equal(expectedInfoVector, actualInfoVector, 1e-7));
}

/* *************************************************************************/
// Have to think about how to compare multiplyHessianAdd in generalSfMFactor and smartFactors
//TEST(SmartProjectionFactor, comparisonGeneralSfMFactor2 ){
//
//  Values values;
//  values.insert(c1, level_camera);
//  values.insert(c2, level_camera_right);
//
//  NonlinearFactorGraph smartGraph;
//  SmartFactor::shared_ptr factor1(new SmartFactor());
//  factor1->add(level_uv, c1, unit2);
//  factor1->add(level_uv_right, c2, unit2);
//  smartGraph.push_back(factor1);
//  GaussianFactorGraph::shared_ptr gfgSmart = smartGraph.linearize(values);
//
//  Point3 expectedPoint(0,0,0);
//  if(factor1->point())
//    expectedPoint = *(factor1->point());
//
//  // COMMENTS:
//  // 1) triangulation introduces small errors, then for a fair comparison we use expectedPoint as
//  // value in the generalGrap
//  NonlinearFactorGraph generalGraph;
//  SFMFactor sfm1(level_uv, unit2, c1, l1);
//  SFMFactor sfm2(level_uv_right, unit2, c2, l1);
//  generalGraph.push_back(sfm1);
//  generalGraph.push_back(sfm2);
//  values.insert(l1, expectedPoint); // note: we get rid of possible errors in the triangulation
//  GaussianFactorGraph::shared_ptr gfg = generalGraph.linearize(values);
//
//  double alpha = 1.0;
//
//  VectorValues yExpected, yActual, ytmp;
//  VectorValues xtmp = map_list_of
//      (c1, (Vec(9) << 0,0,0,0,0,0,0,0,0))
//      (c2, (Vec(9) << 0,0,0,0,0,0,0,0,0))
//      (l1, (Vec(3) << 5.5, 0.5, 1.2));
//  gfg ->multiplyHessianAdd(alpha, xtmp, ytmp);
//
//  VectorValues x = map_list_of
//      (c1, (Vec(9) << 1,2,3,4,5,6,7,8,9))
//      (c2, (Vec(9) << 11,12,13,14,15,16,17,18,19))
//      (l1, (Vec(3) << 5.5, 0.5, 1.2));
//
//  gfgSmart->multiplyHessianAdd(alpha, ytmp + x, yActual);
//  gfg ->multiplyHessianAdd(alpha, x, yExpected);
//
//  EXPECT(assert_equal(yActual,yExpected, 1e-7));
//}
/* *************************************************************************/
TEST(SmartProjectionFactor, computeImplicitJacobian ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor(unit2));
  factor1->add(level_uv, c1);
  factor1->add(level_uv_right, c2);
  Matrix expectedE;
  Vector expectedb;

  CameraSet<Camera> cameras;
  cameras.push_back(level_camera);
  cameras.push_back(level_camera_right);

  factor1->error(values); // this is important to have a triangulation of the point
  Point3 point(0,0,0);
  if (factor1->point())
    point = *(factor1->point());
  SmartFactor::FBlocks Fs;
  factor1->computeJacobians(Fs, expectedE, expectedb, cameras, point);

  NonlinearFactorGraph generalGraph;
  SFMFactor sfm1(level_uv, unit2, c1, l1);
  SFMFactor sfm2(level_uv_right, unit2, c2, l1);
  generalGraph.push_back(sfm1);
  generalGraph.push_back(sfm2);
  values.insert(l1, point); // note: we get rid of possible errors in the triangulation
  Matrix actualFullHessian = generalGraph.linearize(values)->hessian().first;
  Matrix actualVinv = (actualFullHessian.block(18, 18, 3, 3)).inverse();

  Matrix3 expectedVinv = factor1->PointCov(expectedE);
  EXPECT(assert_equal(expectedVinv, actualVinv, 1e-7));
}

/* *************************************************************************/
TEST(SmartProjectionFactor, implicitJacobianFactor ) {

  using namespace bundler;

  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);
  double rankTol = 1;
  bool useEPI = false;

  // Hessian version
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
  params.setEnableEPI(useEPI);

  SmartFactor::shared_ptr explicitFactor(
      new SmartFactor(unit2, params));
  explicitFactor->add(level_uv, c1);
  explicitFactor->add(level_uv_right, c2);

  GaussianFactor::shared_ptr gaussianHessianFactor = explicitFactor->linearize(
      values);
  HessianFactor& hessianFactor =
      dynamic_cast<HessianFactor&>(*gaussianHessianFactor);

  // Implicit Schur version
  params.setLinearizationMode(gtsam::IMPLICIT_SCHUR);
  SmartFactor::shared_ptr implicitFactor(
      new SmartFactor(unit2, params));
  implicitFactor->add(level_uv, c1);
  implicitFactor->add(level_uv_right, c2);
  GaussianFactor::shared_ptr gaussianImplicitSchurFactor =
      implicitFactor->linearize(values);
  CHECK(gaussianImplicitSchurFactor);
  typedef RegularImplicitSchurFactor<Camera> Implicit9;
  Implicit9& implicitSchurFactor =
      dynamic_cast<Implicit9&>(*gaussianImplicitSchurFactor);

  VectorValues x = map_list_of(c1,
      (Vector(9) << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished())(c2,
      (Vector(9) << 11, 12, 13, 14, 15, 16, 17, 18, 19).finished());

  VectorValues yExpected, yActual;
  double alpha = 1.0;
  hessianFactor.multiplyHessianAdd(alpha, x, yActual);
  implicitSchurFactor.multiplyHessianAdd(alpha, x, yExpected);
  EXPECT(assert_equal(yActual, yExpected, 1e-7));
}

/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

TEST(SmartProjectionFactor, serialize) {
  using namespace vanilla;
  using namespace gtsam::serializationTestHelpers;
  SmartFactor factor(unit2);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

