/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionRigFactor.cpp
 *  @brief Unit tests for SmartProjectionRigFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @author Frank Dellaert
 *  @date   August 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <boost/assign/std/map.hpp>
#include <iostream>

#include "smartFactorScenarios.h"
#define DISABLE_TIMING

using namespace boost::assign;
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

Key cameraId1 = 0;  // first camera
Key cameraId2 = 1;
Key cameraId3 = 2;

static Point2 measurement1(323.0, 240.0);

LevenbergMarquardtParams lmParams;
// Make more verbose like so (in tests):
// params.verbosityLM = LevenbergMarquardtParams::SUMMARY;

/* ************************************************************************* */
// default Cal3_S2 poses with rolling shutter effect
namespace vanillaRig {
using namespace vanillaPose;
SmartProjectionParams params(
    gtsam::HESSIAN,
    gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors
}  // namespace vanillaRig

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, Constructor) {
  using namespace vanillaRig;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  SmartRigFactor::shared_ptr factor1(
      new SmartRigFactor(model, cameraRig, params));
}

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, Constructor2) {
  using namespace vanillaRig;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  SmartProjectionParams params2(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors
  params2.setRankTolerance(rankTol);
  SmartRigFactor factor1(model, cameraRig, params2);
}

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, Constructor3) {
  using namespace vanillaRig;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  SmartRigFactor::shared_ptr factor1(
      new SmartRigFactor(model, cameraRig, params));
  factor1->add(measurement1, x1, cameraId1);
}

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, Constructor4) {
  using namespace vanillaRig;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  SmartProjectionParams params2(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors
  params2.setRankTolerance(rankTol);
  SmartRigFactor factor1(model, cameraRig, params2);
  factor1.add(measurement1, x1, cameraId1);
}

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, Equals) {
  using namespace vanillaRig;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));

  SmartRigFactor::shared_ptr factor1(
      new SmartRigFactor(model, cameraRig, params));
  factor1->add(measurement1, x1, cameraId1);

  SmartRigFactor::shared_ptr factor2(
      new SmartRigFactor(model, cameraRig, params));
  factor2->add(measurement1, x1, cameraId1);

  CHECK(assert_equal(*factor1, *factor2));

  SmartRigFactor::shared_ptr factor3(
      new SmartRigFactor(model, cameraRig, params));
  factor3->add(measurement1, x1);  // now use default camera ID

  CHECK(assert_equal(*factor1, *factor3));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, noiseless) {
  using namespace vanillaRig;

  // Project two landmarks into two cameras
  Point2 level_uv = level_camera.project(landmark1);
  Point2 level_uv_right = level_camera_right.project(landmark1);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));

  SmartRigFactor factor(model, cameraRig, params);
  factor.add(level_uv, x1);  // both taken from the same camera
  factor.add(level_uv_right, x2);

  Values values;  // it's a pose factor, hence these are poses
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartRigFactor::Cameras cameras = factor.cameras(values);
  double actualError2 = factor.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // Calculate expected derivative for point (easiest to check)
  std::function<Vector(Point3)> f =  //
      std::bind(&SmartRigFactor::whitenedError<Point3>, factor, cameras,
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
  SmartRigFactor::Cameras::FBlocks F;
  Matrix E;
  Vector actualErrors = factor.unwhitenedError(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  EXPECT(assert_equal(Z_4x1, actualErrors, 1e-7));

  // Calculate using computeJacobians
  Vector b;
  SmartRigFactor::FBlocks Fs;
  factor.computeJacobians(Fs, E, b, cameras, *point);
  double actualError3 = b.squaredNorm();
  EXPECT(assert_equal(expectedE, E, 1e-7));
  EXPECT_DOUBLES_EQUAL(expectedError, actualError3, 1e-6);
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, noisy) {
  using namespace vanillaRig;

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));

  // Project two landmarks into two cameras
  Point2 pixelError(0.2, 0.2);
  Point2 level_uv = level_camera.project(landmark1) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(x1, cam1.pose());
  Pose3 noise_pose =
      Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10), Point3(0.5, 0.1, 0.3));
  values.insert(x2, pose_right.compose(noise_pose));

  SmartRigFactor::shared_ptr factor(
      new SmartRigFactor(model, cameraRig, params));
  factor->add(level_uv, x1, cameraId1);
  factor->add(level_uv_right, x2, cameraId1);

  double actualError1 = factor->error(values);

  // create other factor by passing multiple measurements
  SmartRigFactor::shared_ptr factor2(
      new SmartRigFactor(model, cameraRig, params));

  Point2Vector measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  KeyVector views{x1, x2};
  FastVector<size_t> cameraIds{0, 0};

  factor2->add(measurements, views, cameraIds);
  double actualError2 = factor2->error(values);
  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, smartFactorWithSensorBodyTransform) {
  using namespace vanillaRig;

  // create arbitrary body_T_sensor (transforms from sensor to body)
  Pose3 body_T_sensor =
      Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(1, 1, 1));
  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(body_T_sensor, sharedK));

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
  KeyVector views{x1, x2, x3};
  FastVector<size_t> cameraIds{0, 0, 0};

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setDegeneracyMode(ZERO_ON_DEGENERACY);
  params.setEnableEPI(false);

  SmartRigFactor smartFactor1(model, cameraRig, params);
  smartFactor1.add(
      measurements_cam1,
      views);  // use default CameraIds since we have a single camera

  SmartRigFactor smartFactor2(model, cameraRig, params);
  smartFactor2.add(measurements_cam2, views);

  SmartRigFactor smartFactor3(model, cameraRig, params);
  smartFactor3.add(measurements_cam3, views);

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

  Pose3 noise_pose =
      Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.1, 0.1, 0.1));
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
  EXPECT(assert_equal(wTb3, result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, smartFactorWithMultipleCameras) {
  using namespace vanillaRig;

  // create arbitrary body_T_sensor (transforms from sensor to body)
  Pose3 body_T_sensor1 =
      Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(1, 1, 1));
  Pose3 body_T_sensor2 =
      Pose3(Rot3::Ypr(-M_PI / 5, 0., -M_PI / 2), Point3(0, 0, 1));
  Pose3 body_T_sensor3 = Pose3::identity();

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(body_T_sensor1, sharedK));
  cameraRig->push_back(Camera(body_T_sensor2, sharedK));
  cameraRig->push_back(Camera(body_T_sensor3, sharedK));

  // These are the poses we want to estimate, from camera measurements
  const Pose3 sensor_T_body1 = body_T_sensor1.inverse();
  const Pose3 sensor_T_body2 = body_T_sensor2.inverse();
  const Pose3 sensor_T_body3 = body_T_sensor3.inverse();
  Pose3 wTb1 = cam1.pose() * sensor_T_body1;
  Pose3 wTb2 = cam2.pose() * sensor_T_body2;
  Pose3 wTb3 = cam3.pose() * sensor_T_body3;

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2), landmark2(5, -0.5, 1.2), landmark3(5, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create smart factors
  KeyVector views{x1, x2, x3};
  FastVector<size_t> cameraIds{0, 1, 2};

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setDegeneracyMode(ZERO_ON_DEGENERACY);
  params.setEnableEPI(false);

  SmartRigFactor smartFactor1(model, cameraRig, params);
  smartFactor1.add(measurements_cam1, views, cameraIds);

  SmartRigFactor smartFactor2(model, cameraRig, params);
  smartFactor2.add(measurements_cam2, views, cameraIds);

  SmartRigFactor smartFactor3(model, cameraRig, params);
  smartFactor3.add(measurements_cam3, views, cameraIds);

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

  Pose3 noise_pose =
      Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100), Point3(0.1, 0.1, 0.1));
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
  EXPECT(assert_equal(wTb3, result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, 3poses_smart_projection_factor) {
  using namespace vanillaPose2;
  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK2));

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  KeyVector views{x1, x2, x3};
  FastVector<size_t> cameraIds{
      0, 0, 0};  // 3 measurements from the same camera in the rig

  SmartProjectionParams params(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_cam3, views, cameraIds);

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

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(assert_equal(
      Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                 -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
            Point3(0.1, -0.1, 1.9)),
      values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, Factors) {
  using namespace vanillaRig;

  // Default cameras for simple derivatives
  Rot3 R;
  static Cal3_S2::shared_ptr sharedK(new Cal3_S2(100, 100, 0, 0, 0));
  Camera cam1(Pose3(R, Point3(0, 0, 0)), sharedK),
      cam2(Pose3(R, Point3(1, 0, 0)), sharedK);

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_cam1;

  // Project 2 landmarks into 2 cameras
  measurements_cam1.push_back(cam1.project(landmark1));
  measurements_cam1.push_back(cam2.project(landmark1));

  // Create smart factors
  KeyVector views{x1, x2};
  FastVector<size_t> cameraIds{0, 0};

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));

  SmartRigFactor::shared_ptr smartFactor1 = boost::make_shared<SmartRigFactor>(
      model, cameraRig, params);
  smartFactor1->add(measurements_cam1,
                    views);  // we have a single camera so use default cameraIds

  SmartRigFactor::Cameras cameras;
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

  // After eliminating the point, A1 and A2 contain 2-rank information on
  // cameras:
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

    boost::shared_ptr<RegularHessianFactor<6>> actual =
        smartFactor1->createHessianFactor(values, 0.0);
    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
    EXPECT(assert_equal(expected, *actual, 1e-6));
    EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
    EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
  }
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, 3poses_iterative_smart_projection_factor) {
  using namespace vanillaRig;

  KeyVector views{x1, x2, x3};

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // create smart factor
  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0};
  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_cam3, views, cameraIds);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(assert_equal(Pose3(Rot3(1.11022302e-16, -0.0314107591, 0.99950656,
                                 -0.99950656, -0.0313952598, -0.000986635786,
                                 0.0314107591, -0.999013364, -0.0313952598),
                            Point3(0.1, -0.1, 1.9)),
                      values.at<Pose3>(x3)));

  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, landmarkDistance) {
  using namespace vanillaRig;

  double excludeLandmarksFutherThanDist = 2;

  KeyVector views{x1, x2, x3};

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setEnableEPI(false);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0};

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_cam3, views, cameraIds);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
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
TEST(SmartProjectionRigFactor, dynamicOutlierRejection) {
  using namespace vanillaRig;

  double excludeLandmarksFutherThanDist = 1e10;
  double dynamicOutlierRejectionThreshold =
      1;  // max 1 pixel of average reprojection error

  KeyVector views{x1, x2, x3};

  // add fourth landmark
  Point3 landmark4(5, -0.5, 1);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4;

  // Project 4 landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  measurements_cam4.at(0) =
      measurements_cam4.at(0) + Point2(10, 10);  // add outlier

  SmartProjectionParams params;
  params.setLinearizationMode(gtsam::HESSIAN);
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
  params.setDynamicOutlierRejectionThreshold(dynamicOutlierRejectionThreshold);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0};

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_cam3, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor4(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor4->add(measurements_cam4, views, cameraIds);

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
TEST(SmartProjectionRigFactor, CheckHessian) {
  KeyVector views{x1, x2, x3};

  using namespace vanillaRig;

  // Two slightly different cameras
  Pose3 pose2 =
      level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
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
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0};

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));  // HESSIAN, by default
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));  // HESSIAN, by default
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));  // HESSIAN, by default
  smartFactor3->add(measurements_cam3, views, cameraIds);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose3 * noise_pose);
  EXPECT(assert_equal(Pose3(Rot3(0.00563056869, -0.130848107, 0.991386438,
                                 -0.991390265, -0.130426831, -0.0115837907,
                                 0.130819108, -0.98278564, -0.130455917),
                            Point3(0.0897734171, -0.110201006, 0.901022872)),
                      values.at<Pose3>(x3)));

  boost::shared_ptr<GaussianFactor> factor1 = smartFactor1->linearize(values);
  boost::shared_ptr<GaussianFactor> factor2 = smartFactor2->linearize(values);
  boost::shared_ptr<GaussianFactor> factor3 = smartFactor3->linearize(values);

  Matrix CumulativeInformation =
      factor1->information() + factor2->information() + factor3->information();

  boost::shared_ptr<GaussianFactorGraph> GaussianGraph =
      graph.linearize(values);
  Matrix GraphInformation = GaussianGraph->hessian().first;

  // Check Hessian
  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-6));

  Matrix AugInformationMatrix = factor1->augmentedInformation() +
                                factor2->augmentedInformation() +
                                factor3->augmentedInformation();

  // Check Information vector
  Vector InfoVector = AugInformationMatrix.block(
      0, 18, 18, 1);  // 18x18 Hessian + information vector

  // Check Hessian
  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-6));
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, Hessian) {
  using namespace vanillaPose2;

  KeyVector views{x1, x2};

  // Project three landmarks into 2 cameras
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2Vector measurements_cam1;
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK2));
  FastVector<size_t> cameraIds{0, 0};

  SmartProjectionParams params(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  Pose3 noise_pose =
      Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10), Point3(0.5, 0.1, 0.3));
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());

  boost::shared_ptr<GaussianFactor> factor = smartFactor1->linearize(values);

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with factor.info(): the bottom right element is the squared sum of
  // the reprojection errors (normalized by the covariance) check that it is
  // correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
}

/* ************************************************************************* */
TEST(SmartProjectionRigFactor, ConstructorWithCal3Bundler) {
  using namespace bundlerPose;
  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedBundlerK));

  SmartProjectionParams params;
  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
  SmartRigFactor factor(model, cameraRig, params);
  factor.add(measurement1, x1, cameraId1);
}

/* *************************************************************************/
TEST(SmartProjectionRigFactor, Cal3Bundler) {
  using namespace bundlerPose;
  SmartProjectionParams params(
      gtsam::HESSIAN,
      gtsam::ZERO_ON_DEGENERACY);  // only config that works with rig factors

  // three landmarks ~5 meters in front of camera
  Point3 landmark3(3, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  KeyVector views{x1, x2, x3};

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedBundlerK));
  FastVector<size_t> cameraIds{0, 0, 0};

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_cam1, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_cam2, views, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_cam3, views, cameraIds);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, cam1.pose(), noisePrior);
  graph.addPrior(x2, cam2.pose(), noisePrior);

  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10),
  //  Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));  // smaller noise
  Values values;
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());
  // initialize third pose with some noise, we expect it to move back to
  // original pose_above
  values.insert(x3, pose_above * noise_pose);
  EXPECT(assert_equal(
      Pose3(Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
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
TEST(SmartProjectionRigFactor,
     hessianComparedToProjFactors_measurementsFromSamePose) {
  // in this test we make sure the fact works even if we have multiple pixel
  // measurements of the same landmark at a single pose, a setup that occurs in
  // multi-camera systems

  using namespace vanillaRig;
  Point2Vector measurements_lmk1;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);

  // create redundant measurements:
  Camera::MeasurementVector measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(
      measurements_lmk1.at(0));  // we readd the first measurement

  // create inputs
  KeyVector keys{x1, x2, x3, x1};

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0, 0};

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1_redundant, keys, cameraIds);

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

  // Use standard ProjectionFactor factor to calculate the Jacobians
  Matrix F = Matrix::Zero(2 * 4, 6 * 3);
  Matrix E = Matrix::Zero(2 * 4, 3);
  Vector b = Vector::Zero(2 * 4);

  // create projection factors rolling shutter
  TestProjectionFactor factor11(measurements_lmk1_redundant[0], model, x1, l0,
                                sharedK);
  Matrix HPoseActual, HEActual;
  // note: b is minus the reprojection error, cf the smart factor jacobian
  // computation
  b.segment<2>(0) =
      -factor11.evaluateError(pose1, *point, HPoseActual, HEActual);
  F.block<2, 6>(0, 0) = HPoseActual;
  E.block<2, 3>(0, 0) = HEActual;

  TestProjectionFactor factor12(measurements_lmk1_redundant[1], model, x2, l0,
                                sharedK);
  b.segment<2>(2) =
      -factor12.evaluateError(pose2, *point, HPoseActual, HEActual);
  F.block<2, 6>(2, 6) = HPoseActual;
  E.block<2, 3>(2, 0) = HEActual;

  TestProjectionFactor factor13(measurements_lmk1_redundant[2], model, x3, l0,
                                sharedK);
  b.segment<2>(4) =
      -factor13.evaluateError(pose3, *point, HPoseActual, HEActual);
  F.block<2, 6>(4, 12) = HPoseActual;
  E.block<2, 3>(4, 0) = HEActual;

  TestProjectionFactor factor14(measurements_lmk1_redundant[3], model, x1, l0,
                                sharedK);
  b.segment<2>(6) =
      -factor11.evaluateError(pose1, *point, HPoseActual, HEActual);
  F.block<2, 6>(6, 0) = HPoseActual;
  E.block<2, 3>(6, 0) = HEActual;

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
TEST(SmartProjectionRigFactor, optimization_3poses_measurementsFromSamePose) {
  using namespace vanillaRig;
  Point2Vector measurements_lmk1, measurements_lmk2, measurements_lmk3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_lmk1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_lmk2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_lmk3);

  // create inputs
  KeyVector keys{x1, x2, x3};
  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(Pose3::identity(), sharedK));
  FastVector<size_t> cameraIds{0, 0, 0};
  FastVector<size_t> cameraIdsRedundant{0, 0, 0, 0};

  // For first factor, we create redundant measurement (taken by the same keys
  // as factor 1, to make sure the redundancy in the keys does not create
  // problems)
  Camera::MeasurementVector& measurements_lmk1_redundant = measurements_lmk1;
  measurements_lmk1_redundant.push_back(
      measurements_lmk1.at(0));  // we readd the first measurement
  KeyVector keys_redundant = keys;
  keys_redundant.push_back(keys.at(0));  // we readd the first key

  SmartRigFactor::shared_ptr smartFactor1(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor1->add(measurements_lmk1_redundant, keys_redundant,
                    cameraIdsRedundant);

  SmartRigFactor::shared_ptr smartFactor2(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor2->add(measurements_lmk2, keys, cameraIds);

  SmartRigFactor::shared_ptr smartFactor3(
      new SmartRigFactor(model, cameraRig, params));
  smartFactor3->add(measurements_lmk3, keys, cameraIds);

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
// this factor is slightly slower (but comparable) to original
// SmartProjectionPoseFactor
//-Total: 0 CPU (0 times, 0 wall, 0.17 children, min: 0 max: 0)
//|   -SmartRigFactor LINEARIZE: 0.06 CPU
//(10000 times, 0.061226 wall, 0.06 children, min: 0 max: 0)
//|   -SmartPoseFactor LINEARIZE: 0.06 CPU
//(10000 times, 0.073037 wall, 0.06 children, min: 0 max: 0)
/* *************************************************************************/
TEST(SmartProjectionRigFactor, timing) {
  using namespace vanillaRig;

  // Default cameras for simple derivatives
  static Cal3_S2::shared_ptr sharedKSimple(new Cal3_S2(100, 100, 0, 0, 0));

  Rot3 R = Rot3::identity();
  Pose3 pose1 = Pose3(R, Point3(0, 0, 0));
  Pose3 pose2 = Pose3(R, Point3(1, 0, 0));
  Camera cam1(pose1, sharedKSimple), cam2(pose2, sharedKSimple);
  Pose3 body_P_sensorId = Pose3::identity();

  boost::shared_ptr<Cameras> cameraRig(new Cameras());  // single camera in the rig
  cameraRig->push_back(Camera(body_P_sensorId, sharedKSimple));

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  Point2Vector measurements_lmk1;

  // Project 2 landmarks into 2 cameras
  measurements_lmk1.push_back(cam1.project(landmark1));
  measurements_lmk1.push_back(cam2.project(landmark1));

  size_t nrTests = 10000;

  for (size_t i = 0; i < nrTests; i++) {
    SmartRigFactor::shared_ptr smartRigFactor(
        new SmartRigFactor(model, cameraRig, params));
    smartRigFactor->add(measurements_lmk1[0], x1, cameraId1);
    smartRigFactor->add(measurements_lmk1[1], x1, cameraId1);

    Values values;
    values.insert(x1, pose1);
    values.insert(x2, pose2);
    gttic_(SmartRigFactor_LINEARIZE);
    smartRigFactor->linearize(values);
    gttoc_(SmartRigFactor_LINEARIZE);
  }

  for (size_t i = 0; i < nrTests; i++) {
    SmartFactor::shared_ptr smartFactor(
        new SmartFactor(model, sharedKSimple, params));
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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
