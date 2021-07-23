/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionPoseFactorRollingShutterRollingShutter.cpp
 *  @brief Unit tests for SmartProjectionPoseFactorRollingShutterRollingShutter Class
 *  @author Luca Carlone
 *  @date   July 2021
 */

#include "gtsam/slam/tests/smartFactorScenarios.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_unstable/slam/SmartProjectionPoseFactorRollingShutter.h>
#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace gtsam;
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
static Symbol x4('X', 4);
static Symbol l0('L', 0);
static Pose3 body_P_sensor = Pose3(Rot3::Ypr(-0.1, 0.2, -0.2),
                                   Point3(0.1, 0.0, 0.0));

static Point2 measurement1(323.0, 240.0);
static Point2 measurement2(200.0, 220.0);
static Point2 measurement3(320.0, 10.0);
static double interp_factor = 0.5;
static double interp_factor1 = 0.3;
static double interp_factor2 = 0.4;
static double interp_factor3 = 0.5;

/* ************************************************************************* */
// default Cal3_S2 poses with rolling shutter effect
namespace vanillaPoseRS {
typedef PinholePose<Cal3_S2> Camera;
static Cal3_S2::shared_ptr sharedK(new Cal3_S2(fov, w, h));
Pose3 interp_pose1 = interpolate<Pose3>(level_pose,pose_right,interp_factor1);
Pose3 interp_pose2 = interpolate<Pose3>(pose_right,pose_above,interp_factor2);
Pose3 interp_pose3 = interpolate<Pose3>(pose_above,level_pose,interp_factor3);
Camera cam1(interp_pose1, sharedK);
Camera cam2(interp_pose2, sharedK);
Camera cam3(interp_pose3, sharedK);
}

LevenbergMarquardtParams lmParams;
typedef SmartProjectionPoseFactorRollingShutter<Cal3_S2> SmartFactorRS;

/* ************************************************************************* */
TEST( SmartProjectionPoseFactorRollingShutter, Constructor) {
  SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactorRollingShutter, Constructor2) {
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorRS factor1(model, params);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactorRollingShutter, add) {
  using namespace vanillaPose;
  SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
  factor1->add(measurement1, x1, x2, interp_factor, sharedK, body_P_sensor);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactorRollingShutter, Equals ) {
  using namespace vanillaPose;

  // create fake measurements
  Point2Vector measurements;
  measurements.push_back(measurement1);
  measurements.push_back(measurement2);
  measurements.push_back(measurement3);

  std::vector<std::pair<Key,Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1,x2));
  key_pairs.push_back(std::make_pair(x2,x3));
  key_pairs.push_back(std::make_pair(x3,x4));

  std::vector<boost::shared_ptr<Cal3_S2>> intrinsicCalibrations;
  intrinsicCalibrations.push_back(sharedK);
  intrinsicCalibrations.push_back(sharedK);
  intrinsicCalibrations.push_back(sharedK);

  std::vector<Pose3> extrinsicCalibrations;
  extrinsicCalibrations.push_back(body_P_sensor);
  extrinsicCalibrations.push_back(body_P_sensor);
  extrinsicCalibrations.push_back(body_P_sensor);

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  // create by adding a batch of measurements with a bunch of calibrations
  SmartFactorRS::shared_ptr factor2(new SmartFactorRS(model));
  factor2->add(measurements, key_pairs, interp_factors, intrinsicCalibrations, extrinsicCalibrations);

  // create by adding a batch of measurements with a single calibrations
  SmartFactorRS::shared_ptr factor3(new SmartFactorRS(model));
  factor3->add(measurements, key_pairs, interp_factors, sharedK, body_P_sensor);

  { // create equal factors and show equal returns true
    SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
    factor1->add(measurement1, x1, x2, interp_factor1, sharedK, body_P_sensor);
    factor1->add(measurement2, x2, x3, interp_factor2, sharedK, body_P_sensor);
    factor1->add(measurement3, x3, x4, interp_factor3, sharedK, body_P_sensor);

    CHECK(assert_equal(*factor1, *factor2));
    CHECK(assert_equal(*factor1, *factor3));
  }
  { // create slightly different factors (different keys) and show equal returns false
    SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
    factor1->add(measurement1, x1, x2, interp_factor1, sharedK, body_P_sensor);
    factor1->add(measurement2, x2, x2, interp_factor2, sharedK, body_P_sensor); // different!
    factor1->add(measurement3, x3, x4, interp_factor3, sharedK, body_P_sensor);

    CHECK(!assert_equal(*factor1, *factor2));
    CHECK(!assert_equal(*factor1, *factor3));
  }
  { // create slightly different factors (different extrinsics) and show equal returns false
    SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
    factor1->add(measurement1, x1, x2, interp_factor1, sharedK, body_P_sensor);
    factor1->add(measurement2, x2, x3, interp_factor2, sharedK, body_P_sensor*body_P_sensor); // different!
    factor1->add(measurement3, x3, x4, interp_factor3, sharedK, body_P_sensor);

    CHECK(!assert_equal(*factor1, *factor2));
    CHECK(!assert_equal(*factor1, *factor3));
  }
  { // create slightly different factors (different interp factors) and show equal returns false
    SmartFactorRS::shared_ptr factor1(new SmartFactorRS(model));
    factor1->add(measurement1, x1, x2, interp_factor1, sharedK, body_P_sensor);
    factor1->add(measurement2, x2, x3, interp_factor1, sharedK, body_P_sensor); // different!
    factor1->add(measurement3, x3, x4, interp_factor3, sharedK, body_P_sensor);

    CHECK(!assert_equal(*factor1, *factor2));
    CHECK(!assert_equal(*factor1, *factor3));
  }
}

static const int DimBlock = 12;  ///< size of the variable stacking 2 poses from which the observation pose is interpolated
static const int ZDim = 2;  ///< Measurement dimension (Point2)
typedef Eigen::Matrix<double, ZDim, DimBlock> MatrixZD;  // F blocks (derivatives wrt camera)
typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks;  // vector of F blocks

/* *************************************************************************/
TEST( SmartProjectionPoseFactorRollingShutter, noiselessErrorAndJacobians ) {
  using namespace vanillaPoseRS;

  // Project two landmarks into two cameras
  Point2 level_uv = cam1.project(landmark1);
  Point2 level_uv_right = cam2.project(landmark1);
  Pose3 body_P_sensorId = Pose3::identity();

  SmartFactorRS factor(model);
  factor.add(level_uv, x1, x2, interp_factor1, sharedK, body_P_sensorId);
  factor.add(level_uv_right, x2, x3, interp_factor2, sharedK, body_P_sensorId);

  Values values; // it's a pose factor, hence these are poses
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  // Check triangulation
  factor.triangulateSafe(factor.cameras(values));
  TriangulationResult point = factor.point();
  CHECK(point.valid()); // check triangulated point is valid
  CHECK(assert_equal(landmark1, *point)); // check triangulation result matches expected 3D landmark

  // Check Jacobians
  // -- actual Jacobians
  FBlocks actualFs;
  Matrix actualE;
  Vector actualb;
  factor.computeJacobiansWithTriangulatedPoint(actualFs, actualE, actualb, values);
  CHECK(actualE.rows() == 4); CHECK(actualE.cols() == 3);
  CHECK(actualb.rows() == 4); CHECK(actualb.cols() == 1);
  CHECK(actualFs.size() == 2);

  // -- expected Jacobians from ProjectionFactorsRollingShutter
  ProjectionFactorRollingShutter factor1(level_uv, interp_factor1, model, x1, x2, l0, sharedK, body_P_sensorId);
  Matrix expectedF11, expectedF12, expectedE1;
  Vector expectedb1 = factor1.evaluateError(level_pose, pose_right, landmark1, expectedF11, expectedF12, expectedE1);
  CHECK(assert_equal( expectedF11, Matrix(actualFs[0].block(0,0,2,6)), 1e-5));
  CHECK(assert_equal( expectedF12, Matrix(actualFs[0].block(0,6,2,6)), 1e-5));
  CHECK(assert_equal( expectedE1, Matrix(actualE.block(0,0,2,3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus reprojectionError
  CHECK(assert_equal( expectedb1, -Vector(actualb.segment<2>(0)), 1e-5));

  ProjectionFactorRollingShutter factor2(level_uv_right, interp_factor2, model, x2, x3, l0, sharedK, body_P_sensorId);
  Matrix expectedF21, expectedF22, expectedE2;
  Vector expectedb2 = factor2.evaluateError(pose_right, pose_above, landmark1, expectedF21, expectedF22, expectedE2);
  CHECK(assert_equal( expectedF21, Matrix(actualFs[1].block(0,0,2,6)), 1e-5));
  CHECK(assert_equal( expectedF22, Matrix(actualFs[1].block(0,6,2,6)), 1e-5));
  CHECK(assert_equal( expectedE2, Matrix(actualE.block(2,0,2,3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus reprojectionError
  CHECK(assert_equal( expectedb2, -Vector(actualb.segment<2>(2)), 1e-5));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactorRollingShutter, noisyErrorAndJacobians ) {
  // also includes non-identical extrinsic calibration
  using namespace vanillaPoseRS;

  // Project two landmarks into two cameras
  Point2 pixelError(0.5, 1.0);
  Point2 level_uv = cam1.project(landmark1) + pixelError;
  Point2 level_uv_right = cam2.project(landmark1);
  Pose3 body_P_sensorNonId = body_P_sensor;

  SmartFactorRS factor(model);
  factor.add(level_uv, x1, x2, interp_factor1, sharedK, body_P_sensorNonId);
  factor.add(level_uv_right, x2, x3, interp_factor2, sharedK, body_P_sensorNonId);

  Values values; // it's a pose factor, hence these are poses
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  // Perform triangulation
  factor.triangulateSafe(factor.cameras(values));
  TriangulationResult point = factor.point();
  CHECK(point.valid()); // check triangulated point is valid
  Point3 landmarkNoisy = *point;

  // Check Jacobians
  // -- actual Jacobians
  FBlocks actualFs;
  Matrix actualE;
  Vector actualb;
  factor.computeJacobiansWithTriangulatedPoint(actualFs, actualE, actualb, values);
  CHECK(actualE.rows() == 4); CHECK(actualE.cols() == 3);
  CHECK(actualb.rows() == 4); CHECK(actualb.cols() == 1);
  CHECK(actualFs.size() == 2);

  // -- expected Jacobians from ProjectionFactorsRollingShutter
  ProjectionFactorRollingShutter factor1(level_uv, interp_factor1, model, x1, x2, l0, sharedK, body_P_sensorNonId);
  Matrix expectedF11, expectedF12, expectedE1;
  Vector expectedb1 = factor1.evaluateError(level_pose, pose_right, landmarkNoisy, expectedF11, expectedF12, expectedE1);
  CHECK(assert_equal( expectedF11, Matrix(actualFs[0].block(0,0,2,6)), 1e-5));
  CHECK(assert_equal( expectedF12, Matrix(actualFs[0].block(0,6,2,6)), 1e-5));
  CHECK(assert_equal( expectedE1, Matrix(actualE.block(0,0,2,3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus reprojectionError
  CHECK(assert_equal( expectedb1, -Vector(actualb.segment<2>(0)), 1e-5));

  ProjectionFactorRollingShutter factor2(level_uv_right, interp_factor2, model, x2, x3, l0, sharedK, body_P_sensorNonId);
  Matrix expectedF21, expectedF22, expectedE2;
  Vector expectedb2 = factor2.evaluateError(pose_right, pose_above, landmarkNoisy, expectedF21, expectedF22, expectedE2);
  CHECK(assert_equal( expectedF21, Matrix(actualFs[1].block(0,0,2,6)), 1e-5));
  CHECK(assert_equal( expectedF22, Matrix(actualFs[1].block(0,6,2,6)), 1e-5));
  CHECK(assert_equal( expectedE2, Matrix(actualE.block(2,0,2,3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus reprojectionError
  CHECK(assert_equal( expectedb2, -Vector(actualb.segment<2>(2)), 1e-5));

  // Check errors
  double actualError = factor.error(values); // from smart factor
  NonlinearFactorGraph nfg;
  nfg.add(factor1);
  nfg.add(factor2);
  values.insert(l0, landmarkNoisy);
  double expectedError = nfg.error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactorRollingShutter, optimization_3poses ) {

  using namespace vanillaPoseRS;
  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // create inputs
  std::vector<std::pair<Key,Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1,x2));
  key_pairs.push_back(std::make_pair(x2,x3));
  key_pairs.push_back(std::make_pair(x3,x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  SmartFactorRS smartFactor1(model);
  smartFactor1.add(measurements_cam1, key_pairs, interp_factors, sharedK);

  SmartFactorRS smartFactor2(model);
  smartFactor2.add(measurements_cam2, key_pairs, interp_factors, sharedK);

  SmartFactorRS smartFactor3(model);
  smartFactor3.add(measurements_cam3, key_pairs, interp_factors, sharedK);

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
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactorRollingShutter, hessian_simple_2poses ) {
  // here we replicate a test in SmartProjectionPoseFactor by setting interpolation
  // factors to 0 and 1 (such that the rollingShutter measurements falls back to standard pixel measurements)
  // Note: this is a quite extreme test since in typical camera you would not have more than
  // 1 measurement per landmark at each interpolated pose
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

  Point2Vector measurements_cam1;

  // Project 2 landmarks into 2 cameras
  measurements_cam1.push_back(cam1.project(landmark1));
  measurements_cam1.push_back(cam2.project(landmark1));

  SmartFactorRS::shared_ptr smartFactor1(new SmartFactorRS(model));
  double interp_factor = 0;  // equivalent to measurement taken at pose 1
  smartFactor1->add(measurements_cam1[0], x1, x2, interp_factor, sharedKSimple,
                    body_P_sensorId);
  interp_factor = 1;  // equivalent to measurement taken at pose 2
  smartFactor1->add(measurements_cam1[1], x1, x2, interp_factor, sharedKSimple,
                    body_P_sensorId);

  SmartFactor::Cameras cameras;
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
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  boost::shared_ptr < RegularHessianFactor<6> > actual = smartFactor1
      ->createHessianFactor(values);
  EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
  EXPECT(assert_equal(expected, *actual, 1e-6));
  EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
  EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
}

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, 3poses_iterative_smart_projection_factor ) {
  std::cout << "===================" << std::endl;
 using namespace vanillaPose;

 KeyVector views {x1, x2, x3};

 Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

 // Project three landmarks into three cameras
 projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
 projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
 projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

 SmartFactor::shared_ptr smartFactor1(new SmartFactor(model, sharedK));
 smartFactor1->add(measurements_cam1, views);

 SmartFactor::shared_ptr smartFactor2(new SmartFactor(model, sharedK));
 smartFactor2->add(measurements_cam2, views);

 SmartFactor::shared_ptr smartFactor3(new SmartFactor(model, sharedK));
 smartFactor3->add(measurements_cam3, views);

 const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

 NonlinearFactorGraph graph;
 graph.push_back(smartFactor1);
 graph.push_back(smartFactor2);
 graph.push_back(smartFactor3);
 graph.addPrior(x1, cam1.pose(), noisePrior);
 graph.addPrior(x2, cam2.pose(), noisePrior);

 //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
 Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
 Point3(0.1, 0.1, 0.1)); // smaller noise
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
 -0.0313952598), Point3(0.1, -0.1, 1.9)),
 values.at<Pose3>(x3)));

 Values result;
 LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
 result = optimizer.optimize();
 EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
 }

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, landmarkDistance ) {

 using namespace vanillaPose;

 double excludeLandmarksFutherThanDist = 2;

 KeyVector views {x1, x2, x3};

 Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

 // Project three landmarks into three cameras
 projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
 projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
 projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

 SmartProjectionParams params;
 params.setRankTolerance(1.0);
 params.setLinearizationMode(gtsam::JACOBIAN_SVD);
 params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
 params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
 params.setEnableEPI(false);

 SmartFactor::shared_ptr smartFactor1(
 new SmartFactor(model, sharedK, params));
 smartFactor1->add(measurements_cam1, views);

 SmartFactor::shared_ptr smartFactor2(
 new SmartFactor(model, sharedK, params));
 smartFactor2->add(measurements_cam2, views);

 SmartFactor::shared_ptr smartFactor3(
 new SmartFactor(model, sharedK, params));
 smartFactor3->add(measurements_cam3, views);

 const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

 NonlinearFactorGraph graph;
 graph.push_back(smartFactor1);
 graph.push_back(smartFactor2);
 graph.push_back(smartFactor3);
 graph.addPrior(x1, cam1.pose(), noisePrior);
 graph.addPrior(x2, cam2.pose(), noisePrior);

 //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
 Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
 Point3(0.1, 0.1, 0.1)); // smaller noise
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

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, dynamicOutlierRejection ) {

 using namespace vanillaPose;

 double excludeLandmarksFutherThanDist = 1e10;
 double dynamicOutlierRejectionThreshold = 1; // max 1 pixel of average reprojection error

 KeyVector views {x1, x2, x3};

 // add fourth landmark
 Point3 landmark4(5, -0.5, 1);

 Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
 measurements_cam4;

 // Project 4 landmarks into three cameras
 projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
 projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
 projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
 projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
 measurements_cam4.at(0) = measurements_cam4.at(0) + Point2(10, 10); // add outlier

 SmartProjectionParams params;
 params.setLinearizationMode(gtsam::JACOBIAN_SVD);
 params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
 params.setDynamicOutlierRejectionThreshold(dynamicOutlierRejectionThreshold);

 SmartFactor::shared_ptr smartFactor1(
 new SmartFactor(model, sharedK, params));
 smartFactor1->add(measurements_cam1, views);

 SmartFactor::shared_ptr smartFactor2(
 new SmartFactor(model, sharedK, params));
 smartFactor2->add(measurements_cam2, views);

 SmartFactor::shared_ptr smartFactor3(
 new SmartFactor(model, sharedK, params));
 smartFactor3->add(measurements_cam3, views);

 SmartFactor::shared_ptr smartFactor4(
 new SmartFactor(model, sharedK, params));
 smartFactor4->add(measurements_cam4, views);

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

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, 3poses_projection_factor ) {

 using namespace vanillaPose2;

 KeyVector views {x1, x2, x3};

 typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
 NonlinearFactorGraph graph;

 // Project three landmarks into three cameras
 graph.emplace_shared<ProjectionFactor>(cam1.project(landmark1), model, x1, L(1), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam2.project(landmark1), model, x2, L(1), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam3.project(landmark1), model, x3, L(1), sharedK2);

 graph.emplace_shared<ProjectionFactor>(cam1.project(landmark2), model, x1, L(2), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam2.project(landmark2), model, x2, L(2), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam3.project(landmark2), model, x3, L(2), sharedK2);

 graph.emplace_shared<ProjectionFactor>(cam1.project(landmark3), model, x1, L(3), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam2.project(landmark3), model, x2, L(3), sharedK2);
 graph.emplace_shared<ProjectionFactor>(cam3.project(landmark3), model, x3, L(3), sharedK2);

 const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
 graph.addPrior(x1, level_pose, noisePrior);
 graph.addPrior(x2, pose_right, noisePrior);

 Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
 Point3(0.5, 0.1, 0.3));
 Values values;
 values.insert(x1, level_pose);
 values.insert(x2, pose_right);
 values.insert(x3, pose_above * noise_pose);
 values.insert(L(1), landmark1);
 values.insert(L(2), landmark2);
 values.insert(L(3), landmark3);

 DOUBLES_EQUAL(48406055, graph.error(values), 1);

 LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
 Values result = optimizer.optimize();

 DOUBLES_EQUAL(0, graph.error(result), 1e-9);

 EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
 }

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, CheckHessian) {

 KeyVector views {x1, x2, x3};

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

 SmartProjectionParams params;
 params.setRankTolerance(10);

 SmartFactor::shared_ptr smartFactor1(
 new SmartFactor(model, sharedK, params)); // HESSIAN, by default
 smartFactor1->add(measurements_cam1, views);

 SmartFactor::shared_ptr smartFactor2(
 new SmartFactor(model, sharedK, params)); // HESSIAN, by default
 smartFactor2->add(measurements_cam2, views);

 SmartFactor::shared_ptr smartFactor3(
 new SmartFactor(model, sharedK, params)); // HESSIAN, by default
 smartFactor3->add(measurements_cam3, views);

 NonlinearFactorGraph graph;
 graph.push_back(smartFactor1);
 graph.push_back(smartFactor2);
 graph.push_back(smartFactor3);

 //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
 Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
 Point3(0.1, 0.1, 0.1)); // smaller noise
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
 Vector InfoVector = AugInformationMatrix.block(0, 18, 18, 1); // 18x18 Hessian + information vector

 // Check Hessian
 EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-6));
 }

 /* *************************************************************************
 TEST( SmartProjectionPoseFactorRollingShutter, Hessian ) {

 using namespace vanillaPose2;

 KeyVector views {x1, x2};

 // Project three landmarks into 2 cameras
 Point2 cam1_uv1 = cam1.project(landmark1);
 Point2 cam2_uv1 = cam2.project(landmark1);
 Point2Vector measurements_cam1;
 measurements_cam1.push_back(cam1_uv1);
 measurements_cam1.push_back(cam2_uv1);

 SmartFactor::shared_ptr smartFactor1(new SmartFactor(model, sharedK2));
 smartFactor1->add(measurements_cam1, views);

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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

