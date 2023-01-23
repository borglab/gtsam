/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionPoseFactorRollingShutter.cpp
 *  @brief Unit tests for SmartProjectionPoseFactorRollingShutter Class
 *  @author Luca Carlone
 *  @date   July 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>
#include <gtsam_unstable/slam/SmartProjectionPoseFactorRollingShutter.h>

#include <iostream>

#include "gtsam/slam/tests/smartFactorScenarios.h"
#define DISABLE_TIMING

using namespace gtsam;
using namespace std::placeholders;

static const double rankTol = 1.0;
// Create a noise model for the pixel error
static const double sigma = 0.1;
static SharedIsotropic model(noiseModel::Isotropic::Sigma(2, sigma));

// Convenience for named keys
using symbol_shorthand::L;
using symbol_shorthand::X;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);
static Symbol x4('X', 4);
static Symbol l0('L', 0);
static Pose3 body_P_sensor =
    Pose3(Rot3::Ypr(-0.1, 0.2, -0.2), Point3(0.1, 0.0, 0.0));

static Point2 measurement1(323.0, 240.0);
static Point2 measurement2(200.0, 220.0);
static Point2 measurement3(320.0, 10.0);
static double interp_factor = 0.5;
static double interp_factor1 = 0.3;
static double interp_factor2 = 0.4;
static double interp_factor3 = 0.5;

static size_t cameraId1 = 0;

/* ************************************************************************* */
// default Cal3_S2 poses with rolling shutter effect
namespace vanillaPoseRS {
typedef PinholePose<Cal3_S2> Camera;
typedef CameraSet<Camera> Cameras;
static Cal3_S2::shared_ptr sharedK(new Cal3_S2(fov, w, h));
Pose3 interp_pose1 = interpolate<Pose3>(level_pose, pose_right, interp_factor1);
Pose3 interp_pose2 = interpolate<Pose3>(pose_right, pose_above, interp_factor2);
Pose3 interp_pose3 = interpolate<Pose3>(pose_above, level_pose, interp_factor3);
Camera cam1(interp_pose1, sharedK);
Camera cam2(interp_pose2, sharedK);
Camera cam3(interp_pose3, sharedK);
SmartProjectionParams params(
    gtsam::HESSIAN,
    gtsam::ZERO_ON_DEGENERACY);  // only config that works with RS factors
}  // namespace vanillaPoseRS

LevenbergMarquardtParams lmParams;
typedef SmartProjectionPoseFactorRollingShutter<PinholePose<Cal3_S2>>
    SmartFactorRS;

/* ************************************************************************* */
TEST(SmartProjectionPoseFactorRollingShutter, Constructor) {
  using namespace vanillaPoseRS;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));
  SmartFactorRS::shared_ptr factor1(
      new SmartFactorRS(model, cameraRig, params));
}

/* ************************************************************************* */
TEST(SmartProjectionPoseFactorRollingShutter, Constructor2) {
  using namespace vanillaPoseRS;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));
  params.setRankTolerance(rankTol);
  SmartFactorRS factor1(model, cameraRig, params);
}

/* ************************************************************************* */
TEST(SmartProjectionPoseFactorRollingShutter, add) {
  using namespace vanillaPoseRS;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));
  SmartFactorRS::shared_ptr factor1(
      new SmartFactorRS(model, cameraRig, params));
  factor1->add(measurement1, x1, x2, interp_factor);
}

/* ************************************************************************* */
TEST(SmartProjectionPoseFactorRollingShutter, Equals) {
  using namespace vanillaPoseRS;

  // create fake measurements
  Point2Vector measurements;
  measurements.push_back(measurement1);
  measurements.push_back(measurement2);
  measurements.push_back(measurement3);

  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x4));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  FastVector<size_t> cameraIds{0, 0, 0};

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_P_sensor, sharedK));

  // create by adding a batch of measurements with a bunch of calibrations
  SmartFactorRS::shared_ptr factor2(
      new SmartFactorRS(model, cameraRig, params));
  factor2->add(measurements, key_pairs, interp_factors, cameraIds);

  // create by adding a batch of measurements with a single calibrations
  SmartFactorRS::shared_ptr factor3(
      new SmartFactorRS(model, cameraRig, params));
  factor3->add(measurements, key_pairs, interp_factors, cameraIds);

  {  // create equal factors and show equal returns true
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig, params));
    factor1->add(measurement1, x1, x2, interp_factor1, cameraId1);
    factor1->add(measurement2, x2, x3, interp_factor2, cameraId1);
    factor1->add(measurement3, x3, x4, interp_factor3, cameraId1);

    EXPECT(factor1->equals(*factor2));
    EXPECT(factor1->equals(*factor3));
  }
  {  // create equal factors and show equal returns true (use default cameraId)
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig, params));
    factor1->add(measurement1, x1, x2, interp_factor1);
    factor1->add(measurement2, x2, x3, interp_factor2);
    factor1->add(measurement3, x3, x4, interp_factor3);

    EXPECT(factor1->equals(*factor2));
    EXPECT(factor1->equals(*factor3));
  }
  {  // create equal factors and show equal returns true (use default cameraId)
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig, params));
    factor1->add(measurements, key_pairs, interp_factors);

    EXPECT(factor1->equals(*factor2));
    EXPECT(factor1->equals(*factor3));
  }
  {  // create slightly different factors (different keys) and show equal
     // returns false (use default cameraIds)
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig, params));
    factor1->add(measurement1, x1, x2, interp_factor1, cameraId1);
    factor1->add(measurement2, x2, x2, interp_factor2,
                 cameraId1);  // different!
    factor1->add(measurement3, x3, x4, interp_factor3, cameraId1);

    EXPECT(!factor1->equals(*factor2));
    EXPECT(!factor1->equals(*factor3));
  }
  {  // create slightly different factors (different extrinsics) and show equal
     // returns false
    boost::shared_ptr<Cameras> cameraRig2(new Cameras());
    cameraRig2->push_back(Camera(body_P_sensor * body_P_sensor, sharedK));
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig2, params));
    factor1->add(measurement1, x1, x2, interp_factor1, cameraId1);
    factor1->add(measurement2, x2, x3, interp_factor2,
                 cameraId1);  // different!
    factor1->add(measurement3, x3, x4, interp_factor3, cameraId1);

    EXPECT(!factor1->equals(*factor2));
    EXPECT(!factor1->equals(*factor3));
  }
  {  // create slightly different factors (different interp factors) and show
     // equal returns false
    SmartFactorRS::shared_ptr factor1(
        new SmartFactorRS(model, cameraRig, params));
    factor1->add(measurement1, x1, x2, interp_factor1, cameraId1);
    factor1->add(measurement2, x2, x3, interp_factor1,
                 cameraId1);  // different!
    factor1->add(measurement3, x3, x4, interp_factor3, cameraId1);

    EXPECT(!factor1->equals(*factor2));
    EXPECT(!factor1->equals(*factor3));
  }
}

static const int DimBlock = 12;  ///< size of the variable stacking 2 poses from
                                 ///< which the observation pose is interpolated
static const int ZDim = 2;       ///< Measurement dimension (Point2)
typedef Eigen::Matrix<double, ZDim, DimBlock>
    MatrixZD;  // F blocks (derivatives wrt camera)
typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD>>
    FBlocks;  // vector of F blocks

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, noiselessErrorAndJacobians) {
  using namespace vanillaPoseRS;

  // Project two landmarks into two cameras
  Point2 level_uv = cam1.project(landmark1);
  Point2 level_uv_right = cam2.project(landmark1);
  Pose3 body_P_sensorId = Pose3::Identity();

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_P_sensorId, sharedK));

  SmartFactorRS factor(model, cameraRig, params);
  factor.add(level_uv, x1, x2, interp_factor1);
  factor.add(level_uv_right, x2, x3, interp_factor2);

  Values values;  // it's a pose factor, hence these are poses
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  // Check triangulation
  factor.triangulateSafe(factor.cameras(values));
  TriangulationResult point = factor.point();
  EXPECT(point.valid());  // check triangulated point is valid
  EXPECT(assert_equal(
      landmark1,
      *point));  // check triangulation result matches expected 3D landmark

  // Check Jacobians
  // -- actual Jacobians
  FBlocks actualFs;
  Matrix actualE;
  Vector actualb;
  factor.computeJacobiansWithTriangulatedPoint(actualFs, actualE, actualb,
                                               values);
  EXPECT(actualE.rows() == 4);
  EXPECT(actualE.cols() == 3);
  EXPECT(actualb.rows() == 4);
  EXPECT(actualb.cols() == 1);
  EXPECT(actualFs.size() == 2);

  // -- expected Jacobians from ProjectionFactorsRollingShutter
  ProjectionFactorRollingShutter factor1(level_uv, interp_factor1, model, x1,
                                         x2, l0, sharedK, body_P_sensorId);
  Matrix expectedF11, expectedF12, expectedE1;
  Vector expectedb1 = factor1.evaluateError(
      level_pose, pose_right, landmark1, expectedF11, expectedF12, expectedE1);
  EXPECT(
      assert_equal(expectedF11, Matrix(actualFs[0].block(0, 0, 2, 6)), 1e-5));
  EXPECT(
      assert_equal(expectedF12, Matrix(actualFs[0].block(0, 6, 2, 6)), 1e-5));
  EXPECT(assert_equal(expectedE1, Matrix(actualE.block(0, 0, 2, 3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus
  // reprojectionError
  EXPECT(assert_equal(expectedb1, -Vector(actualb.segment<2>(0)), 1e-5));

  ProjectionFactorRollingShutter factor2(level_uv_right, interp_factor2, model,
                                         x2, x3, l0, sharedK, body_P_sensorId);
  Matrix expectedF21, expectedF22, expectedE2;
  Vector expectedb2 = factor2.evaluateError(
      pose_right, pose_above, landmark1, expectedF21, expectedF22, expectedE2);
  EXPECT(
      assert_equal(expectedF21, Matrix(actualFs[1].block(0, 0, 2, 6)), 1e-5));
  EXPECT(
      assert_equal(expectedF22, Matrix(actualFs[1].block(0, 6, 2, 6)), 1e-5));
  EXPECT(assert_equal(expectedE2, Matrix(actualE.block(2, 0, 2, 3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus
  // reprojectionError
  EXPECT(assert_equal(expectedb2, -Vector(actualb.segment<2>(2)), 1e-5));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, noisyErrorAndJacobians) {
  // also includes non-identical extrinsic calibration
  using namespace vanillaPoseRS;

  // Project two landmarks into two cameras
  Point2 pixelError(0.5, 1.0);
  Point2 level_uv = cam1.project(landmark1) + pixelError;
  Point2 level_uv_right = cam2.project(landmark1);
  Pose3 body_P_sensorNonId = body_P_sensor;

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_P_sensorNonId, sharedK));

  SmartFactorRS factor(model, cameraRig, params);
  factor.add(level_uv, x1, x2, interp_factor1);
  factor.add(level_uv_right, x2, x3, interp_factor2);

  Values values;  // it's a pose factor, hence these are poses
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  // Perform triangulation
  factor.triangulateSafe(factor.cameras(values));
  TriangulationResult point = factor.point();
  EXPECT(point.valid());  // check triangulated point is valid
  Point3 landmarkNoisy = *point;

  // Check Jacobians
  // -- actual Jacobians
  FBlocks actualFs;
  Matrix actualE;
  Vector actualb;
  factor.computeJacobiansWithTriangulatedPoint(actualFs, actualE, actualb,
                                               values);
  EXPECT(actualE.rows() == 4);
  EXPECT(actualE.cols() == 3);
  EXPECT(actualb.rows() == 4);
  EXPECT(actualb.cols() == 1);
  EXPECT(actualFs.size() == 2);

  // -- expected Jacobians from ProjectionFactorsRollingShutter
  ProjectionFactorRollingShutter factor1(level_uv, interp_factor1, model, x1,
                                         x2, l0, sharedK, body_P_sensorNonId);
  Matrix expectedF11, expectedF12, expectedE1;
  Vector expectedb1 =
      factor1.evaluateError(level_pose, pose_right, landmarkNoisy, expectedF11,
                            expectedF12, expectedE1);
  EXPECT(
      assert_equal(expectedF11, Matrix(actualFs[0].block(0, 0, 2, 6)), 1e-5));
  EXPECT(
      assert_equal(expectedF12, Matrix(actualFs[0].block(0, 6, 2, 6)), 1e-5));
  EXPECT(assert_equal(expectedE1, Matrix(actualE.block(0, 0, 2, 3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus
  // reprojectionError
  EXPECT(assert_equal(expectedb1, -Vector(actualb.segment<2>(0)), 1e-5));

  ProjectionFactorRollingShutter factor2(level_uv_right, interp_factor2, model,
                                         x2, x3, l0, sharedK,
                                         body_P_sensorNonId);
  Matrix expectedF21, expectedF22, expectedE2;
  Vector expectedb2 =
      factor2.evaluateError(pose_right, pose_above, landmarkNoisy, expectedF21,
                            expectedF22, expectedE2);
  EXPECT(
      assert_equal(expectedF21, Matrix(actualFs[1].block(0, 0, 2, 6)), 1e-5));
  EXPECT(
      assert_equal(expectedF22, Matrix(actualFs[1].block(0, 6, 2, 6)), 1e-5));
  EXPECT(assert_equal(expectedE2, Matrix(actualE.block(2, 0, 2, 3)), 1e-5));
  // by definition computeJacobiansWithTriangulatedPoint returns minus
  // reprojectionError
  EXPECT(assert_equal(expectedb2, -Vector(actualb.segment<2>(2)), 1e-5));

  // Check errors
  double actualError = factor.error(values);  // from smart factor
  NonlinearFactorGraph nfg;
  nfg.add(factor1);
  nfg.add(factor2);
  values.insert(l0, landmarkNoisy);
  double expectedError = nfg.error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, optimization_3poses) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor2(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor3(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors);

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

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, optimization_3poses_multiCam) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_P_sensor, sharedK));
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors, {1, 1, 1});

  SmartFactorRS::shared_ptr smartFactor2(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors, {1, 1, 1});

  SmartFactorRS::shared_ptr smartFactor3(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors, {1, 1, 1});

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
  groundTruth.insert(x3, pose_above);  // pose above is the pose of the camera
  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, optimization_3poses_multiCam2) {
  using namespace vanillaPoseRS;

  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // create arbitrary body_T_sensor (transforms from sensor to body)
  Pose3 body_T_sensor1 = Pose3(Rot3::Ypr(-0.03, 0., 0.01), Point3(1, 1, 1));
  Pose3 body_T_sensor2 = Pose3(Rot3::Ypr(-0.1, 0., 0.05), Point3(0, 0, 1));
  Pose3 body_T_sensor3 = Pose3(Rot3::Ypr(-0.3, 0., -0.05), Point3(0, 1, 1));

  Camera camera1(interp_pose1 * body_T_sensor1, sharedK);
  Camera camera2(interp_pose2 * body_T_sensor2, sharedK);
  Camera camera3(interp_pose3 * body_T_sensor3, sharedK);

  // Project three landmarks into three cameras
  projectToMultipleCameras(camera1, camera2, camera3, landmark1,
                           measurements_lmk1);
  projectToMultipleCameras(camera1, camera2, camera3, landmark2,
                           measurements_lmk2);
  projectToMultipleCameras(camera1, camera2, camera3, landmark3,
                           measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_T_sensor1, sharedK));
  cameraRig->push_back(Camera(body_T_sensor2, sharedK));
  cameraRig->push_back(Camera(body_T_sensor3, sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors, {0, 1, 2});

  SmartFactorRS::shared_ptr smartFactor2(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors, {0, 1, 2});

  SmartFactorRS::shared_ptr smartFactor3(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors, {0, 1, 2});

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
  groundTruth.insert(x3, pose_above);  // pose above is the pose of the camera
  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-4));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, hessian_simple_2poses) {
  // here we replicate a test in SmartProjectionPoseFactor by setting
  // interpolation factors to 0 and 1 (such that the rollingShutter measurements
  // falls back to standard pixel measurements) Note: this is a quite extreme
  // test since in typical camera you would not have more than 1 measurement per
  // landmark at each interpolated pose
  using namespace vanillaPoseRS;

  // Default cameras for simple derivatives
  static Cal3_S2::shared_ptr sharedKSimple(new Cal3_S2(100, 100, 0, 0, 0));

  Rot3 R = Rot3::Identity();
  Pose3 pose1 = Pose3(R, Point3(0, 0, 0));
  Pose3 pose2 = Pose3(R, Point3(1, 0, 0));
  Camera cam1(pose1, sharedKSimple), cam2(pose2, sharedKSimple);
  Pose3 body_P_sensorId = Pose3::Identity();

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_lmk1;

  // Project 2 landmarks into 2 cameras
  measurements_lmk1.push_back(cam1.project(landmark1));
  measurements_lmk1.push_back(cam2.project(landmark1));

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(body_P_sensorId, sharedKSimple));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  double interp_factor = 0;  // equivalent to measurement taken at pose 1
  smartFactor1->add(measurements_lmk1[0], x1, x2, interp_factor);
  interp_factor = 1;  // equivalent to measurement taken at pose 2
  smartFactor1->add(measurements_lmk1[1], x1, x2, interp_factor);

  SmartFactorRS::Cameras cameras;
  cameras.push_back(cam1);
  cameras.push_back(cam2);

  // Make sure triangulation works
  EXPECT(smartFactor1->triangulateSafe(cameras));
  EXPECT(!smartFactor1->isDegenerate());
  EXPECT(!smartFactor1->isPointBehindCamera());
  std::optional<Point3> p = smartFactor1->point();
  EXPECT(p);
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

  // After eliminating the point, A1 and A2 contain 2-rank information on
  // cameras:
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
  boost::shared_ptr<RegularHessianFactor<6>> actual =
      smartFactor1->createHessianFactor(values);
  EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
  EXPECT(assert_equal(expected, *actual, 1e-6));
  EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
  EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, optimization_3poses_EPI) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  double excludeLandmarksFutherThanDist = 1e10;  // very large
  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setEnableEPI(true);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS smartFactor1(model, cameraRig, params);
  smartFactor1.add(measurements_lmk1, key_pairs, interp_factors);

  SmartFactorRS smartFactor2(model, cameraRig, params);
  smartFactor2.add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS smartFactor3(model, cameraRig, params);
  smartFactor3.add(measurements_lmk3, key_pairs, interp_factors);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, level_pose, noisePrior);
  graph.addPrior(x2, pose_right, noisePrior);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);

  // Optimization should correct 3rd pose
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     optimization_3poses_landmarkDistance) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  double excludeLandmarksFutherThanDist = 2;
  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setLinearizationMode(gtsam::HESSIAN);
  // params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY); // this would give an
  // exception as expected
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setEnableEPI(false);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS smartFactor1(model, cameraRig, params);
  smartFactor1.add(measurements_lmk1, key_pairs, interp_factors);

  SmartFactorRS smartFactor2(model, cameraRig, params);
  smartFactor2.add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS smartFactor3(model, cameraRig, params);
  smartFactor3.add(measurements_lmk3, key_pairs, interp_factors);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, level_pose, noisePrior);
  graph.addPrior(x2, pose_right, noisePrior);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);

  // All factors are disabled (due to the distance threshold) and pose should
  // remain where it is
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     optimization_3poses_dynamicOutlierRejection) {
  using namespace vanillaPoseRS;
  // add fourth landmark
  Point3 landmark4(5, -0.5, 1);

  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3,
      measurements_lmk4;
  // Project 4 landmarks into cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_lmk4);
  measurements_lmk4.at(0) =
      measurements_lmk4.at(0) + Point2(10, 10);  // add outlier

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  double excludeLandmarksFutherThanDist = 1e10;
  double dynamicOutlierRejectionThreshold =
      3;  // max 3 pixel of average reprojection error

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setDynamicOutlierRejectionThreshold(dynamicOutlierRejectionThreshold);
  params.setEnableEPI(false);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor2(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor3(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor4(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor4->add(measurements_lmk4, key_pairs, interp_factors);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.addPrior(x1, level_pose, noisePrior);
  graph.addPrior(x2, pose_right, noisePrior);

  Pose3 noise_pose = Pose3(
      Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.01, 0.01,
             0.01));  // smaller noise, otherwise outlier rejection will kick in
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);

  // Optimization should correct 3rd pose
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     hessianComparedToProjFactorsRollingShutter) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise to get a nontrivial linearization
  // point
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  // linearization point for the poses
  Pose3 pose1 = level_pose;
  Pose3 pose2 = pose_right;
  Pose3 pose3 = pose_above * noise_pose;

  // ==== check Hessian of smartFactor1 =====
  // -- compute actual Hessian
  boost::shared_ptr<GaussianFactor> linearfactor1 =
      smartFactor1->linearize(values);
  Matrix actualHessian = linearfactor1->information();

  // -- compute expected Hessian from manual Schur complement from Jacobians
  // linearization point for the 3D point
  smartFactor1->triangulateSafe(smartFactor1->cameras(values));
  TriangulationResult point = smartFactor1->point();
  EXPECT(point.valid());  // check triangulated point is valid

  // Use the factor to calculate the Jacobians
  Matrix F = Matrix::Zero(2 * 3, 6 * 3);
  Matrix E = Matrix::Zero(2 * 3, 3);
  Vector b = Vector::Zero(6);

  // create projection factors rolling shutter
  ProjectionFactorRollingShutter factor11(measurements_lmk1[0], interp_factor1,
                                          model, x1, x2, l0, sharedK);
  Matrix H1Actual, H2Actual, H3Actual;
  // note: b is minus the reprojection error, cf the smart factor jacobian
  // computation
  b.segment<2>(0) = -factor11.evaluateError(pose1, pose2, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(0, 0) = H1Actual;
  F.block<2, 6>(0, 6) = H2Actual;
  E.block<2, 3>(0, 0) = H3Actual;

  ProjectionFactorRollingShutter factor12(measurements_lmk1[1], interp_factor2,
                                          model, x2, x3, l0, sharedK);
  b.segment<2>(2) = -factor12.evaluateError(pose2, pose3, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(2, 6) = H1Actual;
  F.block<2, 6>(2, 12) = H2Actual;
  E.block<2, 3>(2, 0) = H3Actual;

  ProjectionFactorRollingShutter factor13(measurements_lmk1[2], interp_factor3,
                                          model, x3, x1, l0, sharedK);
  b.segment<2>(4) = -factor13.evaluateError(pose3, pose1, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(4, 12) = H1Actual;
  F.block<2, 6>(4, 0) = H2Actual;
  E.block<2, 3>(4, 0) = H3Actual;

  // whiten
  F = (1 / sigma) * F;
  E = (1 / sigma) * E;
  b = (1 / sigma) * b;
  //* G = F' * F - F' * E * P * E' * F
  Matrix P = (E.transpose() * E).inverse();
  Matrix expectedHessian =
      F.transpose() * F - (F.transpose() * E * P * E.transpose() * F);
  EXPECT(assert_equal(expectedHessian, actualHessian, 1e-6));

  // ==== check Information vector of smartFactor1 =====
  GaussianFactorGraph gfg;
  gfg.add(linearfactor1);
  Matrix actualHessian_v2 = gfg.hessian().first;
  EXPECT(assert_equal(actualHessian_v2, actualHessian,
                      1e-6));  // sanity check on hessian

  // -- compute actual information vector
  Vector actualInfoVector = gfg.hessian().second;

  // -- compute expected information vector from manual Schur complement from
  // Jacobians
  //* g = F' * (b - E * P * E' * b)
  Vector expectedInfoVector = F.transpose() * (b - E * P * E.transpose() * b);
  EXPECT(assert_equal(expectedInfoVector, actualInfoVector, 1e-6));

  // ==== check error of smartFactor1 (again) =====
  NonlinearFactorGraph nfg_projFactorsRS;
  nfg_projFactorsRS.add(factor11);
  nfg_projFactorsRS.add(factor12);
  nfg_projFactorsRS.add(factor13);
  values.insert(l0, *point);

  double actualError = smartFactor1->error(values);
  double expectedError = nfg_projFactorsRS.error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     hessianComparedToProjFactorsRollingShutter_measurementsFromSamePose) {
  // in this test we make sure the fact works even if we have multiple pixel
  // measurements of the same landmark at a single pose, a setup that occurs in
  // multi-camera systems

  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);

  // create redundant measurements:
  Camera::MeasurementVector measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(
      measurements_lmk1.at(0));  // we readd the first measurement

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));
  key_pairs.push_back(std::make_pair(x1, x2));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);
  interp_factors.push_back(interp_factor1);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1_redundant, key_pairs, interp_factors);

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise to get a nontrivial linearization
  // point
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  // linearization point for the poses
  Pose3 pose1 = level_pose;
  Pose3 pose2 = pose_right;
  Pose3 pose3 = pose_above * noise_pose;

  // ==== check Hessian of smartFactor1 =====
  // -- compute actual Hessian
  boost::shared_ptr<GaussianFactor> linearfactor1 =
      smartFactor1->linearize(values);
  Matrix actualHessian = linearfactor1->information();

  // -- compute expected Hessian from manual Schur complement from Jacobians
  // linearization point for the 3D point
  smartFactor1->triangulateSafe(smartFactor1->cameras(values));
  TriangulationResult point = smartFactor1->point();
  EXPECT(point.valid());  // check triangulated point is valid

  // Use standard ProjectionFactorRollingShutter factor to calculate the
  // Jacobians
  Matrix F = Matrix::Zero(2 * 4, 6 * 3);
  Matrix E = Matrix::Zero(2 * 4, 3);
  Vector b = Vector::Zero(8);

  // create projection factors rolling shutter
  ProjectionFactorRollingShutter factor11(measurements_lmk1_redundant[0],
                                          interp_factor1, model, x1, x2, l0,
                                          sharedK);
  Matrix H1Actual, H2Actual, H3Actual;
  // note: b is minus the reprojection error, cf the smart factor jacobian
  // computation
  b.segment<2>(0) = -factor11.evaluateError(pose1, pose2, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(0, 0) = H1Actual;
  F.block<2, 6>(0, 6) = H2Actual;
  E.block<2, 3>(0, 0) = H3Actual;

  ProjectionFactorRollingShutter factor12(measurements_lmk1_redundant[1],
                                          interp_factor2, model, x2, x3, l0,
                                          sharedK);
  b.segment<2>(2) = -factor12.evaluateError(pose2, pose3, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(2, 6) = H1Actual;
  F.block<2, 6>(2, 12) = H2Actual;
  E.block<2, 3>(2, 0) = H3Actual;

  ProjectionFactorRollingShutter factor13(measurements_lmk1_redundant[2],
                                          interp_factor3, model, x3, x1, l0,
                                          sharedK);
  b.segment<2>(4) = -factor13.evaluateError(pose3, pose1, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(4, 12) = H1Actual;
  F.block<2, 6>(4, 0) = H2Actual;
  E.block<2, 3>(4, 0) = H3Actual;

  ProjectionFactorRollingShutter factor14(measurements_lmk1_redundant[3],
                                          interp_factor1, model, x1, x2, l0,
                                          sharedK);
  b.segment<2>(6) = -factor11.evaluateError(pose1, pose2, *point, H1Actual,
                                            H2Actual, H3Actual);
  F.block<2, 6>(6, 0) = H1Actual;
  F.block<2, 6>(6, 6) = H2Actual;
  E.block<2, 3>(6, 0) = H3Actual;

  // whiten
  F = (1 / sigma) * F;
  E = (1 / sigma) * E;
  b = (1 / sigma) * b;
  //* G = F' * F - F' * E * P * E' * F
  Matrix P = (E.transpose() * E).inverse();
  Matrix expectedHessian =
      F.transpose() * F - (F.transpose() * E * P * E.transpose() * F);
  EXPECT(assert_equal(expectedHessian, actualHessian, 1e-6));

  // ==== check Information vector of smartFactor1 =====
  GaussianFactorGraph gfg;
  gfg.add(linearfactor1);
  Matrix actualHessian_v2 = gfg.hessian().first;
  EXPECT(assert_equal(actualHessian_v2, actualHessian,
                      1e-6));  // sanity check on hessian

  // -- compute actual information vector
  Vector actualInfoVector = gfg.hessian().second;

  // -- compute expected information vector from manual Schur complement from
  // Jacobians
  //* g = F' * (b - E * P * E' * b)
  Vector expectedInfoVector = F.transpose() * (b - E * P * E.transpose() * b);
  EXPECT(assert_equal(expectedInfoVector, actualInfoVector, 1e-6));

  // ==== check error of smartFactor1 (again) =====
  NonlinearFactorGraph nfg_projFactorsRS;
  nfg_projFactorsRS.add(factor11);
  nfg_projFactorsRS.add(factor12);
  nfg_projFactorsRS.add(factor13);
  nfg_projFactorsRS.add(factor14);
  values.insert(l0, *point);

  double actualError = smartFactor1->error(values);
  double expectedError = nfg_projFactorsRS.error(values);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     optimization_3poses_measurementsFromSamePose) {
  using namespace vanillaPoseRS;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  // For first factor, we create redundant measurement (taken by the same keys
  // as factor 1, to make sure the redundancy in the keys does not create
  // problems)
  Camera::MeasurementVector& measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(
      measurements_lmk1.at(0));  // we readd the first measurement
  std::vector<std::pair<Key, Key>> key_pairs_redundant = key_pairs;
  key_pairs_redundant.push_back(
      key_pairs.at(0));  // we readd the first pair of keys
  std::vector<double> interp_factors_redundant = interp_factors;
  interp_factors_redundant.push_back(
      interp_factors.at(0));  // we readd the first interp factor

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), sharedK));

  SmartFactorRS::shared_ptr smartFactor1(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1_redundant, key_pairs_redundant,
                    interp_factors_redundant);

  SmartFactorRS::shared_ptr smartFactor2(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS::shared_ptr smartFactor3(
      new SmartFactorRS(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors);

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

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-5));
}

#ifndef DISABLE_TIMING
#include <gtsam/base/timing.h>
//-Total: 0 CPU (0 times, 0 wall, 0.21 children, min: 0 max: 0)
//|   -SF RS LINEARIZE: 0.14 CPU
//(10000 times, 0.131202 wall, 0.14 children, min: 0 max: 0)
//|   -RS LINEARIZE: 0.06 CPU
//(10000 times, 0.066951 wall, 0.06 children, min: 0 max: 0)
/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter, timing) {
  using namespace vanillaPose;

  // Default cameras for simple derivatives
  static Cal3_S2::shared_ptr sharedKSimple(new Cal3_S2(100, 100, 0, 0, 0));
  SmartProjectionParams params(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with RS factors

  Rot3 R = Rot3::Identity();
  Pose3 pose1 = Pose3(R, Point3(0, 0, 0));
  Pose3 pose2 = Pose3(R, Point3(1, 0, 0));
  Camera cam1(pose1, sharedKSimple), cam2(pose2, sharedKSimple);
  Pose3 body_P_sensorId = Pose3::Identity();

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_lmk1;

  // Project 2 landmarks into 2 cameras
  measurements_lmk1.push_back(cam1.project(landmark1));
  measurements_lmk1.push_back(cam2.project(landmark1));

  size_t nrTests = 10000;

  for (size_t i = 0; i < nrTests; i++) {
    boost::shared_ptr<Cameras> cameraRig(new Cameras());
    cameraRig->push_back(Camera(body_P_sensorId, sharedKSimple));

    SmartFactorRS::shared_ptr smartFactorRS(new SmartFactorRS(
        model, cameraRig, params));
    double interp_factor = 0;  // equivalent to measurement taken at pose 1
    smartFactorRS->add(measurements_lmk1[0], x1, x2, interp_factor);
    interp_factor = 1;  // equivalent to measurement taken at pose 2
    smartFactorRS->add(measurements_lmk1[1], x1, x2, interp_factor);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SF_RS_LINEARIZE);
    smartFactorRS->linearize(values);
    gttoc_(SF_RS_LINEARIZE);
  }

  for (size_t i = 0; i < nrTests; i++) {
    SmartFactor::shared_ptr smartFactor(
        new SmartFactor(model, sharedKSimple, params));
    smartFactor->add(measurements_lmk1[0], x1);
    smartFactor->add(measurements_lmk1[1], x2);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(RS_LINEARIZE);
    smartFactor->linearize(values);
    gttoc_(RS_LINEARIZE);
  }
  tictoc_print_();
}
#endif

#include <gtsam/geometry/SphericalCamera.h>
/* ************************************************************************* */
// spherical Camera with rolling shutter effect
namespace sphericalCameraRS {
typedef SphericalCamera Camera;
typedef CameraSet<Camera> Cameras;
typedef SmartProjectionPoseFactorRollingShutter<Camera> SmartFactorRS_spherical;
Pose3 interp_pose1 = interpolate<Pose3>(level_pose, pose_right, interp_factor1);
Pose3 interp_pose2 = interpolate<Pose3>(pose_right, pose_above, interp_factor2);
Pose3 interp_pose3 = interpolate<Pose3>(pose_above, level_pose, interp_factor3);
static EmptyCal::shared_ptr emptyK(new EmptyCal());
Camera cam1(interp_pose1, emptyK);
Camera cam2(interp_pose2, emptyK);
Camera cam3(interp_pose3, emptyK);
}  // namespace sphericalCameraRS

/* *************************************************************************/
TEST(SmartProjectionPoseFactorRollingShutter,
     optimization_3poses_sphericalCameras) {
  using namespace sphericalCameraRS;
  std::vector<Unit3> measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark1,
                                   measurements_lmk1);
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark2,
                                   measurements_lmk2);
  projectToMultipleCameras<Camera>(cam1, cam2, cam3, landmark3,
                                   measurements_lmk3);

  // create inputs
  std::vector<std::pair<Key, Key>> key_pairs;
  key_pairs.push_back(std::make_pair(x1, x2));
  key_pairs.push_back(std::make_pair(x2, x3));
  key_pairs.push_back(std::make_pair(x3, x1));

  std::vector<double> interp_factors;
  interp_factors.push_back(interp_factor1);
  interp_factors.push_back(interp_factor2);
  interp_factors.push_back(interp_factor3);

  SmartProjectionParams params(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with RS factors
  params.setRankTolerance(0.1);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::Identity(), emptyK));

  SmartFactorRS_spherical::shared_ptr smartFactor1(
      new SmartFactorRS_spherical(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1, key_pairs, interp_factors);

  SmartFactorRS_spherical::shared_ptr smartFactor2(
      new SmartFactorRS_spherical(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, key_pairs, interp_factors);

  SmartFactorRS_spherical::shared_ptr smartFactor3(
      new SmartFactorRS_spherical(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, key_pairs, interp_factors);

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

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(  // check that the pose is actually noisy
      assert_equal(Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656,
                              -0.0313952598, -0.000986635786, 0.0314107591,
                              -0.999013364, -0.0313952598),
                         Point3(0.1, -0.1, 1.9)),
                   values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
