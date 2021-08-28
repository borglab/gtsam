/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionFactorP.cpp
 *  @brief Unit tests for SmartProjectionFactorP Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @author Frank Dellaert
 *  @date   August 2021
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;
using namespace std::placeholders;

static const double rankTol = 1.0;
// Create a noise model for the pixel error
static const double sigma = 0.1;
static SharedIsotropic model(noiseModel::Isotropic::Sigma(2, sigma));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);

static Point2 measurement1(323.0, 240.0);

LevenbergMarquardtParams lmParams;

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor2) {
  using namespace vanillaPose;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorP factor1(model, params);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor3) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
  factor1->add(measurement1, x1, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor4) {
  using namespace vanillaPose;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorP factor1(model, params);
  factor1.add(measurement1, x1, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Equals ) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
  factor1->add(measurement1, x1, sharedK);

  SmartFactorP::shared_ptr factor2(new SmartFactorP(model));
  factor2->add(measurement1, x1, sharedK);

  CHECK(assert_equal(*factor1, *factor2));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, noiseless ) {

  using namespace vanillaPose;

  // Project two landmarks into two cameras
  Point2 level_uv = level_camera.project(landmark1);
  Point2 level_uv_right = level_camera_right.project(landmark1);

  SmartFactorP factor(model);
  factor.add(level_uv, x1, sharedK);
  factor.add(level_uv_right, x2, sharedK);

  Values values;  // it's a pose factor, hence these are poses
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartFactorP::Cameras cameras = factor.cameras(values);
  double actualError2 = factor.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // Calculate expected derivative for point (easiest to check)
  std::function<Vector(Point3)> f =  //
      std::bind(&SmartFactorP::whitenedError<Point3>, factor, cameras,
                std::placeholders::_1);

  // Calculate using computeEP
  Matrix actualE;
  factor.triangulateAndComputeE(actualE, values);

  // get point
  boost::optional<Point3> point = factor.point();
  CHECK(point);

  // calculate numerical derivative with triangulated point
  Matrix expectedE = sigma * numericalDerivative11<Vector, Point3>(f, *point);
  EXPECT(assert_equal(expectedE, actualE, 1e-7));

  // Calculate using reprojectionError
  SmartFactorP::Cameras::FBlocks F;
  Matrix E;
  Vector actualErrors = factor.unwhitenedError(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  EXPECT(assert_equal(Z_4x1, actualErrors, 1e-7));

  // Calculate using computeJacobians
  Vector b;
  SmartFactorP::FBlocks Fs;
  factor.computeJacobians(Fs, E, b, cameras, *point);
  double actualError3 = b.squaredNorm();
  EXPECT(assert_equal(expectedE, E, 1e-7));
  EXPECT_DOUBLES_EQUAL(expectedError, actualError3, 1e-6);
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, noisy ) {

  using namespace vanillaPose;

  // Project two landmarks into two cameras
  Point2 pixelError(0.2, 0.2);
  Point2 level_uv = level_camera.project(landmark1) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(x1, cam1.pose());
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
                           Point3(0.5, 0.1, 0.3));
  values.insert(x2, pose_right.compose(noise_pose));

  SmartFactorP::shared_ptr factor(new SmartFactorP(model));
  factor->add(level_uv, x1, sharedK);
  factor->add(level_uv_right, x2, sharedK);

  double actualError1 = factor->error(values);

  SmartFactorP::shared_ptr factor2(new SmartFactorP(model));
  Point2Vector measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  KeyVector views { x1, x2 };

  factor2->add(measurements, views, sharedKs);
  double actualError2 = factor2->error(values);
  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionFactorP, smartFactorWithSensorBodyTransform) {
  using namespace vanillaPose;

  // create arbitrary body_T_sensor (transforms from sensor to body)
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2),
                              Point3(1, 1, 1));

  // These are the poses we want to estimate, from camera measurements
  const Pose3 sensor_T_body = body_T_sensor.inverse();
  Pose3 wTb1 = cam1.pose() * sensor_T_body;
  Pose3 wTb2 = cam2.pose() * sensor_T_body;
  Pose3 wTb3 = cam3.pose() * sensor_T_body;

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2), landmark2(5, -0.5, 1.2), landmark3(5, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create smart factors
  KeyVector views { x1, x2, x3 };

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setDegeneracyMode(IGNORE_DEGENERACY);
  params.setEnableEPI(false);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  std::vector<Pose3> body_T_sensors;
  body_T_sensors.push_back(body_T_sensor);
  body_T_sensors.push_back(body_T_sensor);
  body_T_sensors.push_back(body_T_sensor);

  SmartFactorP smartFactor1(model, params);
  smartFactor1.add(measurements_cam1, views, sharedKs, body_T_sensors);

  SmartFactorP smartFactor2(model, params);
  smartFactor2.add(measurements_cam2, views, sharedKs, body_T_sensors);

  SmartFactorP smartFactor3(model, params);
  smartFactor3.add(measurements_cam3, views, sharedKs, body_T_sensors);
  ;

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Put all factors in factor graph, adding priors
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, wTb1, noisePrior);
  graph.addPrior(x2, wTb2, noisePrior);

  // Check errors at ground truth poses
  Values gtValues;
  gtValues.insert(x1, wTb1);
  gtValues.insert(x2, wTb2);
  gtValues.insert(x3, wTb3);
  double actualError = graph.error(gtValues);
  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-7)

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));
  Values values;
  values.insert(x1, wTb1);
  values.insert(x2, wTb2);
  // initialize third pose with some noise, we expect it to move back to
  // original pose3
  values.insert(x3, wTb3 * noise_pose);

  LevenbergMarquardtParams lmParams;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
//  graph.print("graph\n");
  EXPECT(assert_equal(wTb3, result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, 3poses_smart_projection_factor ) {

  using namespace vanillaPose2;
  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  KeyVector views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedK2s;
  sharedK2s.push_back(sharedK2);
  sharedK2s.push_back(sharedK2);
  sharedK2s.push_back(sharedK2);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_cam1, views, sharedK2s);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model));
  smartFactor2->add(measurements_cam2, views, sharedK2s);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model));
  smartFactor3->add(measurements_cam3, views, sharedK2s);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  Values groundTruth;
  groundTruth.insert(x1, cam1.pose());
  groundTruth.insert(x2, cam2.pose());
  groundTruth.insert(x3, cam3.pose());
  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                   -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)),
          values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, Factors ) {

  using namespace vanillaPose;

  // Default cameras for simple derivatives
  Rot3 R;
  static Cal3_S2::shared_ptr sharedK(new Cal3_S2(100, 100, 0, 0, 0));
  Camera cam1(Pose3(R, Point3(0, 0, 0)), sharedK), cam2(
      Pose3(R, Point3(1, 0, 0)), sharedK);

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_cam1;

  // Project 2 landmarks into 2 cameras
  measurements_cam1.push_back(cam1.project(landmark1));
  measurements_cam1.push_back(cam2.project(landmark1));

  // Create smart factors
  KeyVector views { x1, x2 };

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  SmartFactorP::shared_ptr smartFactor1 = boost::make_shared < SmartFactorP
      > (model);
  smartFactor1->add(measurements_cam1, views, sharedKs);

  SmartFactorP::Cameras cameras;
  cameras.push_back(cam1);
  cameras.push_back(cam2);

  // Make sure triangulation works
  CHECK(smartFactor1->triangulateSafe(cameras));
  CHECK(!smartFactor1->isDegenerate());
  CHECK(!smartFactor1->isPointBehindCamera());
  boost::optional<Point3> p = smartFactor1->point();
  CHECK(p);
  EXPECT(assert_equal(landmark1, *p));

  VectorValues zeroDelta;
  Vector6 delta;
  delta.setZero();
  zeroDelta.insert(x1, delta);
  zeroDelta.insert(x2, delta);

  VectorValues perturbedDelta;
  delta.setOnes();
  perturbedDelta.insert(x1, delta);
  perturbedDelta.insert(x2, delta);
  double expectedError = 2500;

  // After eliminating the point, A1 and A2 contain 2-rank information on cameras:
  Matrix16 A1, A2;
  A1 << -10, 0, 0, 0, 1, 0;
  A2 << 10, 0, 1, 0, -1, 0;
  A1 *= 10. / sigma;
  A2 *= 10. / sigma;
  Matrix expectedInformation;  // filled below
  {
    // createHessianFactor
    Matrix66 G11 = 0.5 * A1.transpose() * A1;
    Matrix66 G12 = 0.5 * A1.transpose() * A2;
    Matrix66 G22 = 0.5 * A2.transpose() * A2;

    Vector6 g1;
    g1.setZero();
    Vector6 g2;
    g2.setZero();

    double f = 0;

    RegularHessianFactor<6> expected(x1, x2, G11, G12, g1, G22, g2, f);
    expectedInformation = expected.information();

    Values values;
    values.insert(x1, cam1.pose());
    values.insert(x2, cam2.pose());

    boost::shared_ptr < RegularHessianFactor<6> > actual = smartFactor1
        ->createHessianFactor(values, 0.0);
    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
    EXPECT(assert_equal(expected, *actual, 1e-6));
    EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
    EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
  }
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, 3poses_iterative_smart_projection_factor ) {

  using namespace vanillaPose;

  KeyVector views { x1, x2, x3 };

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_cam1, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model));
  smartFactor2->add(measurements_cam2, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model));
  smartFactor3->add(measurements_cam3, views, sharedKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(1.11022302e-16, -0.0314107591, 0.99950656, -0.99950656,
                   -0.0313952598, -0.000986635786, 0.0314107591, -0.999013364,
                   -0.0313952598),
              Point3(0.1, -0.1, 1.9)),
          values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, landmarkDistance ) {

  using namespace vanillaPose;

  double excludeLandmarksFutherThanDist = 2;

  KeyVector views { x1, x2, x3 };

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setLinearizationMode(gtsam::JACOBIAN_SVD);
  params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setEnableEPI(false);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, params));
  smartFactor1->add(measurements_cam1, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, params));
  smartFactor2->add(measurements_cam2, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, params));
  smartFactor3->add(measurements_cam3, views, sharedKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  values.insert(x3, pose_above * noise_pose);

  // All factors are disabled and pose should remain where it is
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, dynamicOutlierRejection ) {

  using namespace vanillaPose;

  double excludeLandmarksFutherThanDist = 1e10;
  double dynamicOutlierRejectionThreshold = 1;  // max 1 pixel of average reprojection error

  KeyVector views { x1, x2, x3 };

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  // add fourth landmark
  Point3 landmark4(5, -0.5, 1);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4;

  // Project 4 landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  measurements_cam4.at(0) = measurements_cam4.at(0) + Point2(10, 10);  // add outlier

  SmartProjectionParams params;
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setDynamicOutlierRejectionThreshold(dynamicOutlierRejectionThreshold);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, params));
  smartFactor1->add(measurements_cam1, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, params));
  smartFactor2->add(measurements_cam2, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, params));
  smartFactor3->add(measurements_cam3, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor4(new SmartFactorP(model, params));
  smartFactor4->add(measurements_cam4, views, sharedKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  values.insert(x3, cam3.pose());

  // All factors are disabled and pose should remain where it is
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(cam3.pose(), result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, CheckHessian) {

  KeyVector views { x1, x2, x3 };

  using namespace vanillaPose;

  // Two slightly different cameras
  Pose3 pose2 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedK);
  Camera cam3(pose3, sharedK);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  SmartProjectionParams params;
  params.setRankTolerance(10);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, params));  // HESSIAN, by default
  smartFactor1->add(measurements_cam1, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, params));  // HESSIAN, by default
  smartFactor2->add(measurements_cam2, views, sharedKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, params));  // HESSIAN, by default
  smartFactor3->add(measurements_cam3, views, sharedKs);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose3 * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                   -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                   -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          values.at<Pose3>(x3)));

  boost::shared_ptr<GaussianFactor> factor1 = smartFactor1->linearize(values);
  boost::shared_ptr<GaussianFactor> factor2 = smartFactor2->linearize(values);
  boost::shared_ptr<GaussianFactor> factor3 = smartFactor3->linearize(values);

  Matrix CumulativeInformation = factor1->information() + factor2->information()
      + factor3->information();

  boost::shared_ptr<GaussianFactorGraph> GaussianGraph = graph.linearize(
      values);
  Matrix GraphInformation = GaussianGraph->hessian().first;

  // Check Hessian
  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-6));

  Matrix AugInformationMatrix = factor1->augmentedInformation()
      + factor2->augmentedInformation() + factor3->augmentedInformation();

  // Check Information vector
  Vector InfoVector = AugInformationMatrix.block(0, 18, 18, 1);  // 18x18 Hessian + information vector

  // Check Hessian
  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-6));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, Hessian ) {

  using namespace vanillaPose2;

  KeyVector views { x1, x2 };

  // Project three landmarks into 2 cameras
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2Vector measurements_cam1;
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedK2s;
  sharedK2s.push_back(sharedK2);
  sharedK2s.push_back(sharedK2);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_cam1, views, sharedK2s);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
                           Point3(0.5, 0.1, 0.3));
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());

  boost::shared_ptr<GaussianFactor> factor = smartFactor1->linearize(values);

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with factor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, ConstructorWithCal3Bundler) {
  using namespace bundlerPose;
  SmartProjectionParams params;
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  SmartFactorP factor(model, params);
  factor.add(measurement1, x1, sharedBundlerK);
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, Cal3Bundler ) {

  using namespace bundlerPose;

  // three landmarks ~5 meters in front of camera
  Point3 landmark3(3, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  KeyVector views { x1, x2, x3 };

  std::vector < boost::shared_ptr < Cal3Bundler >> sharedBundlerKs;
  sharedBundlerKs.push_back(sharedBundlerK);
  sharedBundlerKs.push_back(sharedBundlerK);
  sharedBundlerKs.push_back(sharedBundlerK);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_cam1, views, sharedBundlerKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model));
  smartFactor2->add(measurements_cam2, views, sharedBundlerKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model));
  smartFactor3->add(measurements_cam3, views, sharedBundlerKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                   -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)),
          values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(cam3.pose(), result.at<Pose3>(x3), 1e-6));
}

#include <gtsam/slam/ProjectionFactor.h>
typedef GenericProjectionFactor<Pose3, Point3> TestProjectionFactor;
static Symbol l0('L', 0);
/* *************************************************************************/
TEST( SmartProjectionFactorP, hessianComparedToProjFactors_measurementsFromSamePose) {
  // in this test we make sure the fact works even if we have multiple pixel measurements of the same landmark
  // at a single pose, a setup that occurs in multi-camera systems

  using namespace vanillaPose;
  Point2Vector measurements_lmk1;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);

  // create redundant measurements:
  Camera::MeasurementVector measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(measurements_lmk1.at(0));  // we readd the first measurement

  // create inputs
  std::vector<Key> keys;
  keys.push_back(x1);
  keys.push_back(x2);
  keys.push_back(x3);
  keys.push_back(x1);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_lmk1_redundant, keys, sharedKs);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise to get a nontrivial linearization point
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal( Pose3( Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598, -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598), Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));

  // linearization point for the poses
  Pose3 pose1 = level_pose;
  Pose3 pose2 = pose_right;
  Pose3 pose3 = pose_above * noise_pose;

  // ==== check Hessian of smartFactor1 =====
  // -- compute actual Hessian
  boost::shared_ptr<GaussianFactor> linearfactor1 = smartFactor1->linearize(
      values);
  Matrix actualHessian = linearfactor1->information();

  // -- compute expected Hessian from manual Schur complement from Jacobians
  // linearization point for the 3D point
  smartFactor1->triangulateSafe(smartFactor1->cameras(values));
  TriangulationResult point = smartFactor1->point();
  EXPECT(point.valid());  // check triangulated point is valid

  // Use standard ProjectionFactor factor to calculate the Jacobians
  Matrix F = Matrix::Zero(2 * 4, 6 * 3);
  Matrix E = Matrix::Zero(2 * 4, 3);
  Vector b = Vector::Zero(2 * 4);

  // create projection factors rolling shutter
  TestProjectionFactor factor11(measurements_lmk1_redundant[0], model, x1, l0,
                                sharedK);
  Matrix HPoseActual, HEActual;
  // note: b is minus the reprojection error, cf the smart factor jacobian computation
  b.segment<2>(0) = -factor11.evaluateError(pose1, *point, HPoseActual,
                                            HEActual);
  F.block<2, 6>(0, 0) = HPoseActual;
  E.block<2, 3>(0, 0) = HEActual;

  TestProjectionFactor factor12(measurements_lmk1_redundant[1], model, x2, l0,
                                sharedK);
  b.segment<2>(2) = -factor12.evaluateError(pose2, *point, HPoseActual,
                                            HEActual);
  F.block<2, 6>(2, 6) = HPoseActual;
  E.block<2, 3>(2, 0) = HEActual;

  TestProjectionFactor factor13(measurements_lmk1_redundant[2], model, x3, l0,
                                sharedK);
  b.segment<2>(4) = -factor13.evaluateError(pose3, *point, HPoseActual,
                                            HEActual);
  F.block<2, 6>(4, 12) = HPoseActual;
  E.block<2, 3>(4, 0) = HEActual;

  TestProjectionFactor factor14(measurements_lmk1_redundant[3], model, x1, l0,
                                sharedK);
  b.segment<2>(6) = -factor11.evaluateError(pose1, *point, HPoseActual,
                                            HEActual);
  F.block<2, 6>(6, 0) = HPoseActual;
  E.block<2, 3>(6, 0) = HEActual;

  // whiten
  F = (1 / sigma) * F;
  E = (1 / sigma) * E;
  b = (1 / sigma) * b;
  //* G = F' * F - F' * E * P * E' * F
  Matrix P = (E.transpose() * E).inverse();
  Matrix expectedHessian = F.transpose() * F
      - (F.transpose() * E * P * E.transpose() * F);
  EXPECT(assert_equal(expectedHessian, actualHessian, 1e-6));

  // ==== check Information vector of smartFactor1 =====
  GaussianFactorGraph gfg;
  gfg.add(linearfactor1);
  Matrix actualHessian_v2 = gfg.hessian().first;
  EXPECT(assert_equal(actualHessian_v2, actualHessian, 1e-6));  // sanity check on hessian

  // -- compute actual information vector
  Vector actualInfoVector = gfg.hessian().second;

  // -- compute expected information vector from manual Schur complement from Jacobians
  //* g = F' * (b - E * P * E' * b)
  Vector expectedInfoVector = F.transpose() * (b - E * P * E.transpose() * b);
  EXPECT(assert_equal(expectedInfoVector, actualInfoVector, 1e-6));

  // ==== check error of smartFactor1 (again) =====
  NonlinearFactorGraph nfg_projFactors;
  nfg_projFactors.add(factor11);
  nfg_projFactors.add(factor12);
  nfg_projFactors.add(factor13);
  nfg_projFactors.add(factor14);
  values.insert(l0, *point);

  double actualError = smartFactor1->error(values);
  double expectedError = nfg_projFactors.error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, optimization_3poses_measurementsFromSamePose ) {

  using namespace vanillaPose;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<Key> keys;
  keys.push_back(x1);
  keys.push_back(x2);
  keys.push_back(x3);

  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  // For first factor, we create redundant measurement (taken by the same keys as factor 1, to
  // make sure the redundancy in the keys does not create problems)
  Camera::MeasurementVector& measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(measurements_lmk1.at(0)); // we readd the first measurement
  std::vector<Key> keys_redundant = keys;
  keys_redundant.push_back(keys.at(0)); // we readd the first key
  std::vector < boost::shared_ptr < Cal3_S2 >> sharedKs_redundant = sharedKs;
  sharedKs_redundant.push_back(sharedK);// we readd the first calibration

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model));
  smartFactor1->add(measurements_lmk1_redundant, keys_redundant, sharedKs_redundant);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model));
  smartFactor2->add(measurements_lmk2, keys, sharedKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model));
  smartFactor3->add(measurements_lmk3, keys, sharedKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, level_pose, noisePrior);
  graph.addPrior(x2, pose_right, noisePrior);

  Values groundTruth;
  groundTruth.insert(x1, level_pose);
  groundTruth.insert(x2, pose_right);
  groundTruth.insert(x3, pose_above);
  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT( // check that the pose is actually noisy
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                   -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
                   Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-5));
}

#ifndef DISABLE_TIMING
#include <gtsam/base/timing.h>
// this factor is actually slightly faster (but comparable) to original SmartProjectionPoseFactor
//-Total: 0 CPU (0 times, 0 wall, 0.01 children, min: 0 max: 0)
//|   -SmartFactorP LINEARIZE: 0 CPU (1000 times, 0.005481 wall, 0 children, min: 0 max: 0)
//|   -SmartPoseFactor LINEARIZE: 0.01 CPU (1000 times, 0.007318 wall, 0.01 children, min: 0 max: 0)
/* *************************************************************************/
TEST( SmartProjectionFactorP, timing ) {

  using namespace vanillaPose;

  // Default cameras for simple derivatives
  static Cal3_S2::shared_ptr sharedKSimple(new Cal3_S2(100, 100, 0, 0, 0));

  Rot3 R = Rot3::identity();
  Pose3 pose1 = Pose3(R, Point3(0, 0, 0));
  Pose3 pose2 = Pose3(R, Point3(1, 0, 0));
  Camera cam1(pose1, sharedKSimple), cam2(pose2, sharedKSimple);
  Pose3 body_P_sensorId = Pose3::identity();

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_lmk1;

  // Project 2 landmarks into 2 cameras
  measurements_lmk1.push_back(cam1.project(landmark1));
  measurements_lmk1.push_back(cam2.project(landmark1));

  size_t nrTests = 1000;

  for(size_t i = 0; i<nrTests; i++){
    SmartFactorP::shared_ptr smartFactorP(new SmartFactorP(model));
    smartFactorP->add(measurements_lmk1[0], x1, sharedKSimple, body_P_sensorId);
    smartFactorP->add(measurements_lmk1[1], x1, sharedKSimple, body_P_sensorId);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SmartFactorP_LINEARIZE);
    smartFactorP->linearize(values);
    gttoc_(SmartFactorP_LINEARIZE);
  }

  for(size_t i = 0; i<nrTests; i++){
    SmartFactor::shared_ptr smartFactor(new SmartFactor(model, sharedKSimple));
    smartFactor->add(measurements_lmk1[0], x1);
    smartFactor->add(measurements_lmk1[1], x2);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SmartPoseFactor_LINEARIZE);
    smartFactor->linearize(values);
    gttoc_(SmartPoseFactor_LINEARIZE);
  }
  tictoc_print_();
}
#endif

/* *************************************************************************/
TEST( SmartProjectionFactorP, optimization_3poses_sphericalCamera ) {

  using namespace sphericalCamera;
  Camera::MeasurementVector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<Key> keys;
  keys.push_back(x1);
  keys.push_back(x2);
  keys.push_back(x3);

  std::vector<EmptyCal::shared_ptr> emptyKs;
  emptyKs.push_back(emptyK);
  emptyKs.push_back(emptyK);
  emptyKs.push_back(emptyK);

  SmartProjectionParams params;
  params.setRankTolerance(0.1);

  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model,params));
  smartFactor1->add(measurements_lmk1, keys, emptyKs);

  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model,params));
  smartFactor2->add(measurements_lmk2, keys, emptyKs);

  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model,params));
  smartFactor3->add(measurements_lmk3, keys, emptyKs);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, level_pose, noisePrior);
  graph.addPrior(x2, pose_right, noisePrior);

  Values groundTruth;
  groundTruth.insert(x1, level_pose);
  groundTruth.insert(x2, pose_right);
  groundTruth.insert(x3, pose_above);
  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT( // check that the pose is actually noisy
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                   -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
                   Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));

  DOUBLES_EQUAL(0.15734109864597129, graph.error(values), 1e-9);

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-5));
}

#ifndef DISABLE_TIMING
#include <gtsam/base/timing.h>
// using spherical camera is slightly slower (but comparable) to PinholePose<Cal3_S2>
//|   -SmartFactorP spherical LINEARIZE: 0.01 CPU (1000 times, 0.00752 wall, 0.01 children, min: 0 max: 0)
//|   -SmartFactorP pinhole LINEARIZE: 0 CPU (1000 times, 0.00523 wall, 0 children, min: 0 max: 0)
/* *************************************************************************/
TEST( SmartProjectionFactorP, timing_sphericalCamera ) {

  // create common data
  Rot3 R = Rot3::identity();
  Pose3 pose1 = Pose3(R, Point3(0, 0, 0));
  Pose3 pose2 = Pose3(R, Point3(1, 0, 0));
  Pose3 body_P_sensorId = Pose3::identity();
  Point3 landmark1(0, 0, 10);

  // create spherical data
  EmptyCal::shared_ptr emptyK;
  SphericalCamera cam1_sphere(pose1, emptyK), cam2_sphere(pose2, emptyK);
  // Project 2 landmarks into 2 cameras
  std::vector<Unit3> measurements_lmk1_sphere;
  measurements_lmk1_sphere.push_back(cam1_sphere.project(landmark1));
  measurements_lmk1_sphere.push_back(cam2_sphere.project(landmark1));

  // create Cal3_S2 data
  static Cal3_S2::shared_ptr sharedKSimple(new Cal3_S2(100, 100, 0, 0, 0));
  PinholePose<Cal3_S2> cam1(pose1, sharedKSimple), cam2(pose2, sharedKSimple);
  // Project 2 landmarks into 2 cameras
  std::vector<Point2> measurements_lmk1;
  measurements_lmk1.push_back(cam1.project(landmark1));
  measurements_lmk1.push_back(cam2.project(landmark1));

  size_t nrTests = 1000;

  for(size_t i = 0; i<nrTests; i++){
    SmartProjectionFactorP<SphericalCamera>::shared_ptr smartFactorP(new SmartProjectionFactorP<SphericalCamera>(model));
    smartFactorP->add(measurements_lmk1_sphere[0], x1, emptyK, body_P_sensorId);
    smartFactorP->add(measurements_lmk1_sphere[1], x1, emptyK, body_P_sensorId);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SmartFactorP_spherical_LINEARIZE);
    smartFactorP->linearize(values);
    gttoc_(SmartFactorP_spherical_LINEARIZE);
  }

  for(size_t i = 0; i<nrTests; i++){
    SmartProjectionFactorP< PinholePose<Cal3_S2> >::shared_ptr smartFactorP2(new SmartProjectionFactorP< PinholePose<Cal3_S2> >(model));
    smartFactorP2->add(measurements_lmk1[0], x1, sharedKSimple, body_P_sensorId);
    smartFactorP2->add(measurements_lmk1[1], x1, sharedKSimple, body_P_sensorId);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SmartFactorP_pinhole_LINEARIZE);
    smartFactorP2->linearize(values);
    gttoc_(SmartFactorP_pinhole_LINEARIZE);
  }
  tictoc_print_();
}
#endif

/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

TEST(SmartProjectionFactorP, serialize) {
  using namespace vanillaPose;
  using namespace gtsam::serializationTestHelpers;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorP factor(model, params);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

// SERIALIZATION TEST FAILS: to be fixed
//TEST(SmartProjectionFactorP, serialize2) {
//  using namespace vanillaPose;
//  using namespace gtsam::serializationTestHelpers;
//  SmartProjectionParams params;
//  params.setRankTolerance(rankTol);
//  SmartFactorP factor(model, params);
//
//  // insert some measurements
//  KeyVector key_view;
//  Point2Vector meas_view;
//  std::vector<boost::shared_ptr<Cal3_S2>> sharedKs;
//
//
//  key_view.push_back(Symbol('x', 1));
//  meas_view.push_back(Point2(10, 10));
//  sharedKs.push_back(sharedK);
//  factor.add(meas_view, key_view, sharedKs);
//
//  EXPECT(equalsObj(factor));
//  EXPECT(equalsXML(factor));
//  EXPECT(equalsBinary(factor));
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

