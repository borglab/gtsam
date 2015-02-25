/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionCameraFactor.cpp
 *  @brief Unit tests for SmartProjectionCameraFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @date   Sept 2013
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/SmartProjectionCameraFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;

static bool isDebugTest = false;

static double rankTol = 1.0;
static double linThreshold = -1.0;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

static Key x1(1);
Symbol l1('l', 1), l2('l', 2), l3('l', 3);
Key c1 = 1, c2 = 2, c3 = 3;

static Point2 measurement1(323.0, 240.0);
static Pose3 body_P_sensor1(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
    Point3(0.25, -0.10, 1.0));

typedef SmartProjectionCameraFactor<Cal3_S2> SmartFactor;
typedef SmartProjectionCameraFactor<Cal3Bundler> SmartFactorBundler;

template<class CALIBRATION>
PinholeCamera<CALIBRATION> perturbCameraPoseAndCalibration(
    const PinholeCamera<CALIBRATION>& camera) {
  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
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
TEST( SmartProjectionCameraFactor, perturbCameraPose) {
  using namespace vanilla;
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Pose3 perturbed_level_pose = level_pose.compose(noise_pose);
  Camera actualCamera(perturbed_level_pose, K2);

  Camera expectedCamera = perturbCameraPose(level_camera);
  CHECK(assert_equal(expectedCamera, actualCamera));
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, Constructor) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor());
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, Constructor2) {
  using namespace vanilla;
  SmartFactor factor1(rankTol, linThreshold);
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, Constructor3) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, x1, unit2);
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, Constructor4) {
  using namespace vanilla;
  SmartFactor factor1(rankTol, linThreshold);
  factor1.add(measurement1, x1, unit2);
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, ConstructorWithTransform) {
  using namespace vanilla;
  bool manageDegeneracy = true;
  bool isImplicit = false;
  bool enableEPI = false;
  SmartFactor factor1(rankTol, linThreshold, manageDegeneracy, isImplicit,
      enableEPI, body_P_sensor1);
  factor1.add(measurement1, x1, unit2);
}

/* ************************************************************************* */
TEST( SmartProjectionCameraFactor, Equals ) {
  using namespace vanilla;
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, x1, unit2);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  factor2->add(measurement1, x1, unit2);
}

/* *************************************************************************/
TEST( SmartProjectionCameraFactor, noiseless ) {
  // cout << " ************************ SmartProjectionCameraFactor: noisy ****************************" << endl;
  using namespace vanilla;

  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);

  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, factor1->error(values), 1e-7);
  CHECK(
      assert_equal(zero(4),
          factor1->reprojectionErrorAfterTriangulation(values), 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionCameraFactor, noisy ) {

  using namespace vanilla;

  // 1. Project two landmarks into two cameras and triangulate
  Point2 pixelError(0.2, 0.2);
  Point2 level_uv = level_camera.project(landmark1) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(c1, level_camera);
  Camera perturbed_level_camera_right = perturbCameraPose(level_camera_right);
  values.insert(c2, perturbed_level_camera_right);

  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);

  double actualError1 = factor1->error(values);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  vector<Point2> measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  vector<SharedNoiseModel> noises;
  noises.push_back(unit2);
  noises.push_back(unit2);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);

  factor2->add(measurements, views, noises);

  double actualError2 = factor2->error(values);

  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionCameraFactor, perturbPoseAndOptimize ) {

  using namespace vanilla;

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);
  views.push_back(c3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, unit2);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, unit2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, unit2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(c1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(c2, cam2, noisePrior));

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  values.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionCameraFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionCameraFactor);
  tictoc_finishedIteration_();

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
TEST( SmartProjectionCameraFactor, perturbPoseAndOptimizeFromSfM_tracks ) {

  using namespace vanilla;

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;
  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);
  views.push_back(c3);

  SfM_Track track1;
  for (size_t i = 0; i < 3; ++i) {
    SfM_Measurement measures;
    measures.first = i + 1; // cameras are from 1 to 3
    measures.second = measurements_cam1.at(i);
    track1.measurements.push_back(measures);
  }
  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(track1, unit2);

  SfM_Track track2;
  for (size_t i = 0; i < 3; ++i) {
    SfM_Measurement measures;
    measures.first = i + 1; // cameras are from 1 to 3
    measures.second = measurements_cam2.at(i);
    track2.measurements.push_back(measures);
  }
  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(track2, unit2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, unit2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(c1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(c2, cam2, noisePrior));

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  values.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionCameraFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionCameraFactor);
  tictoc_finishedIteration_();

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
TEST( SmartProjectionCameraFactor, perturbCamerasAndOptimize ) {

  using namespace vanilla;

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);
  views.push_back(c3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, unit2);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, unit2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, unit2);

  SmartFactor::shared_ptr smartFactor4(new SmartFactor());
  smartFactor4->add(measurements_cam4, views, unit2);

  SmartFactor::shared_ptr smartFactor5(new SmartFactor());
  smartFactor5->add(measurements_cam5, views, unit2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6 + 5, 1e-5);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.push_back(smartFactor5);
  graph.push_back(PriorFactor<Camera>(c1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(c2, cam2, noisePrior));

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  values.insert(c3, perturbCameraPoseAndCalibration(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-8;
  params.absoluteErrorTol = 0;
  params.maxIterations = 20;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionCameraFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionCameraFactor);
  tictoc_finishedIteration_();

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
TEST( SmartProjectionCameraFactor, Cal3Bundler ) {

  using namespace bundler;

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);
  views.push_back(c3);

  SmartFactorBundler::shared_ptr smartFactor1(new SmartFactorBundler());
  smartFactor1->add(measurements_cam1, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler());
  smartFactor2->add(measurements_cam2, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler());
  smartFactor3->add(measurements_cam3, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor4(new SmartFactorBundler());
  smartFactor4->add(measurements_cam4, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor5(new SmartFactorBundler());
  smartFactor5->add(measurements_cam5, views, unit2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(9, 1e-6);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(c1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(c2, cam2, noisePrior));

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(c3, perturbCameraPose(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-8;
  params.absoluteErrorTol = 0;
  params.maxIterations = 20;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionCameraFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionCameraFactor);
  tictoc_finishedIteration_();

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
TEST( SmartProjectionCameraFactor, Cal3Bundler2 ) {

  using namespace bundler;

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4, measurements_cam5;

  // 1. Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  projectToMultipleCameras(cam1, cam2, cam3, landmark5, measurements_cam5);

  vector<Key> views;
  views.push_back(c1);
  views.push_back(c2);
  views.push_back(c3);

  SmartFactorBundler::shared_ptr smartFactor1(new SmartFactorBundler());
  smartFactor1->add(measurements_cam1, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler());
  smartFactor2->add(measurements_cam2, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler());
  smartFactor3->add(measurements_cam3, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor4(new SmartFactorBundler());
  smartFactor4->add(measurements_cam4, views, unit2);

  SmartFactorBundler::shared_ptr smartFactor5(new SmartFactorBundler());
  smartFactor5->add(measurements_cam5, views, unit2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(9, 1e-6);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(c1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(c2, cam2, noisePrior));

  Values values;
  values.insert(c1, cam1);
  values.insert(c2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose3
  values.insert(c3, perturbCameraPoseAndCalibration(cam3));
  if (isDebugTest)
    values.at<Camera>(c3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 1e-8;
  params.absoluteErrorTol = 0;
  params.maxIterations = 20;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionCameraFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionCameraFactor);
  tictoc_finishedIteration_();

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
TEST( SmartProjectionCameraFactor, noiselessBundler ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactorBundler::shared_ptr factor1(new SmartFactorBundler());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);

  double actualError = factor1->error(values);

  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-3);

  Point3 oldPoint; // this takes the point stored in the factor (we are not interested in this)
  if (factor1->point())
    oldPoint = *(factor1->point());

  Point3 expectedPoint;
  if (factor1->point(values))
    expectedPoint = *(factor1->point(values));

  EXPECT(assert_equal(expectedPoint, landmark1, 1e-3));
}

/* *************************************************************************/
TEST( SmartProjectionCameraFactor, comparisonGeneralSfMFactor ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  NonlinearFactorGraph smartGraph;
  SmartFactorBundler::shared_ptr factor1(new SmartFactorBundler());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);
  smartGraph.push_back(factor1);
  double expectedError = factor1->error(values);
  double expectedErrorGraph = smartGraph.error(values);
  Point3 expectedPoint;
  if (factor1->point())
    expectedPoint = *(factor1->point());
  // cout << "expectedPoint " << expectedPoint.vector() << endl;

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
  double actualError = 0.5
      * (norm_2(e1) * norm_2(e1) + norm_2(e2) * norm_2(e2));
  double actualErrorGraph = generalGraph.error(values);

  DOUBLES_EQUAL(expectedErrorGraph, actualErrorGraph, 1e-7);
  DOUBLES_EQUAL(expectedErrorGraph, expectedError, 1e-7);
  DOUBLES_EQUAL(actualErrorGraph, actualError, 1e-7);
  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionCameraFactor, comparisonGeneralSfMFactor1 ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  NonlinearFactorGraph smartGraph;
  SmartFactorBundler::shared_ptr factor1(new SmartFactorBundler());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);
  smartGraph.push_back(factor1);
  Matrix expectedHessian = smartGraph.linearize(values)->hessian().first;
  Vector expectedInfoVector = smartGraph.linearize(values)->hessian().second;
  Point3 expectedPoint;
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
//TEST( SmartProjectionCameraFactor, comparisonGeneralSfMFactor2 ){
//
//  Values values;
//  values.insert(c1, level_camera);
//  values.insert(c2, level_camera_right);
//
//  NonlinearFactorGraph smartGraph;
//  SmartFactorBundler::shared_ptr factor1(new SmartFactorBundler());
//  factor1->add(level_uv, c1, unit2);
//  factor1->add(level_uv_right, c2, unit2);
//  smartGraph.push_back(factor1);
//  GaussianFactorGraph::shared_ptr gfgSmart = smartGraph.linearize(values);
//
//  Point3 expectedPoint;
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
TEST( SmartProjectionCameraFactor, computeImplicitJacobian ) {

  using namespace bundler;
  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);

  SmartFactorBundler::shared_ptr factor1(new SmartFactorBundler());
  factor1->add(level_uv, c1, unit2);
  factor1->add(level_uv_right, c2, unit2);
  Matrix expectedF, expectedE;
  Vector expectedb;

  CameraSet<Camera> cameras;
  cameras.push_back(level_camera);
  cameras.push_back(level_camera_right);

  factor1->error(values); // this is important to have a triangulation of the point
  Point3 point;
  if (factor1->point())
    point = *(factor1->point());
  factor1->computeJacobians(expectedF, expectedE, expectedb, cameras, point);

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
TEST( SmartProjectionCameraFactor, implicitJacobianFactor ) {

  using namespace bundler;

  Values values;
  values.insert(c1, level_camera);
  values.insert(c2, level_camera_right);
  double rankTol = 1;
  double linThreshold = -1;
  bool manageDegeneracy = false;
  bool useEPI = false;
  bool isImplicit = false;

  // Hessian version
  SmartFactorBundler::shared_ptr explicitFactor(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy, useEPI,
          isImplicit));
  explicitFactor->add(level_uv, c1, unit2);
  explicitFactor->add(level_uv_right, c2, unit2);

  GaussianFactor::shared_ptr gaussianHessianFactor = explicitFactor->linearize(
      values);
  HessianFactor& hessianFactor =
      dynamic_cast<HessianFactor&>(*gaussianHessianFactor);

  // Implicit Schur version
  isImplicit = true;
  SmartFactorBundler::shared_ptr implicitFactor(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy, useEPI,
          isImplicit));
  implicitFactor->add(level_uv, c1, unit2);
  implicitFactor->add(level_uv_right, c2, unit2);
  GaussianFactor::shared_ptr gaussianImplicitSchurFactor =
      implicitFactor->linearize(values);
  typedef RegularImplicitSchurFactor<9> Implicit9;
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

