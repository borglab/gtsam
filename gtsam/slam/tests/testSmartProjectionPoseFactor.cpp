/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionPoseFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @date   Sept 2013
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;

static bool isDebugTest = false;

static const double rankTol = 1.0;
static const double linThreshold = -1.0;
static const bool manageDegeneracy = true;

// Create a noise model for the pixel error
static const double sigma = 0.1;
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(2, sigma));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);

static Point2 measurement1(323.0, 240.0);
static Pose3 body_P_sensor1(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
    Point3(0.25, -0.10, 1.0));

typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
typedef SmartProjectionPoseFactor<Cal3Bundler> SmartFactorBundler;

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
  using namespace vanillaPose;
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, x1, model, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor4) {
  using namespace vanillaPose;
  SmartFactor factor1(rankTol, linThreshold);
  factor1.add(measurement1, x1, model, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, ConstructorWithTransform) {
  using namespace vanillaPose;
  bool manageDegeneracy = true;
  bool enableEPI = false;
  SmartFactor factor1(rankTol, linThreshold, manageDegeneracy, enableEPI,
      body_P_sensor1);
  factor1.add(measurement1, x1, model, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Equals ) {
  using namespace vanillaPose;
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, x1, model, sharedK);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  factor2->add(measurement1, x1, model, sharedK);

  CHECK(assert_equal(*factor1, *factor2));
}

/* *************************************************************************/
TEST_UNSAFE( SmartProjectionPoseFactor, noiseless ) {
  // cout << " ************************ SmartProjectionPoseFactor: noisy ****************************" << endl;

  using namespace vanillaPose;

  // Project two landmarks into two cameras
  Point2 level_uv = level_camera.project(landmark1);
  Point2 level_uv_right = level_camera_right.project(landmark1);

  SmartFactor factor;
  factor.add(level_uv, x1, model, sharedK);
  factor.add(level_uv_right, x2, model, sharedK);

  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartFactor::Cameras cameras = factor.cameras(values);
  double actualError2 = factor.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // Calculate expected derivative for point (easiest to check)
  boost::function<Vector(Point3)> f = //
      boost::bind(&SmartFactor::whitenedErrors, factor, cameras, _1);
  boost::optional<Point3> point = factor.point();
  CHECK(point);

  // Note ! After refactor the noiseModel is only in the factors, not these matrices
  Matrix expectedE = sigma * numericalDerivative11<Vector, Point3>(f, *point);

  // Calculate using computeEP
  Matrix actualE, PointCov;
  factor.computeEP(actualE, PointCov, values);
  EXPECT(assert_equal(expectedE, actualE, 1e-7));

  // Calculate using reprojectionErrors, note not yet divided by sigma !
  SmartFactor::Cameras::FBlocks F;
  Matrix E;
  Vector actualErrors = factor.reprojectionErrors(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  EXPECT(assert_equal(zero(4), actualErrors, 1e-7));

  // Calculate using computeJacobians
  Vector b;
  vector<SmartFactor::KeyMatrix2D> Fblocks;
  double actualError3 = factor.computeJacobians(Fblocks, E, b, cameras);
  EXPECT(assert_equal(expectedE, E, 1e-7));
  EXPECT_DOUBLES_EQUAL(expectedError, actualError3, 1e-8);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, noisy ) {
  // cout << " ************************ SmartProjectionPoseFactor: noisy ****************************" << endl;

  using namespace vanillaPose;

  // Project two landmarks into two cameras and triangulate
  Point2 pixelError(0.2, 0.2);
  Point2 level_uv = level_camera.project(landmark1) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(x1, level_pose);
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  values.insert(x2, pose_right.compose(noise_pose));

  SmartFactor::shared_ptr factor(new SmartFactor());
  factor->add(level_uv, x1, model, sharedK);
  factor->add(level_uv_right, x2, model, sharedK);

  double actualError1 = factor->error(values);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  vector<Point2> measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  vector<SharedNoiseModel> noises;
  noises.push_back(model);
  noises.push_back(model);

  vector<boost::shared_ptr<Cal3_S2> > Ks; ///< shared pointer to calibration object (one for each camera)
  Ks.push_back(sharedK);
  Ks.push_back(sharedK);

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  factor2->add(measurements, views, noises, Ks);

  double actualError2 = factor2->error(values);

  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_smart_projection_factor ) {
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks **********************" << endl;

  using namespace vanillaPose2;
  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, model, sharedK2);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model, sharedK2);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model, sharedK2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  GaussianFactorGraph::shared_ptr GFG = graph.linearize(values);
  VectorValues delta = GFG->optimize();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-8));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, smartFactorWithSensorBodyTransform ) {

  using namespace vanillaPose;

  // create arbitrary body_Pose_sensor (transforms from sensor to body)
  Pose3 sensor_to_body = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2),
      Point3(1, 1, 1)); // Pose3(); //

  // These are the poses we want to estimate, from camera measurements
  Pose3 bodyPose1 = level_pose.compose(sensor_to_body.inverse());
  Pose3 bodyPose2 = pose_right.compose(sensor_to_body.inverse());
  Pose3 bodyPose3 = pose_above.compose(sensor_to_body.inverse());

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create smart factors
  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  double rankTol = 1;
  double linThreshold = -1;
  bool manageDegeneracy = false;
  bool enableEPI = false;

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy, enableEPI,
          sensor_to_body));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy, enableEPI,
          sensor_to_body));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy, enableEPI,
          sensor_to_body));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Put all factors in factor graph, adding priors
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, bodyPose1, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, bodyPose2, noisePrior));

  // Check errors at ground truth poses
  Values gtValues;
  gtValues.insert(x1, bodyPose1);
  gtValues.insert(x2, bodyPose2);
  gtValues.insert(x3, bodyPose3);
  double actualError = graph.error(gtValues);
  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-7)

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1));
  Values values;
  values.insert(x1, bodyPose1);
  values.insert(x2, bodyPose2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, bodyPose3 * noise_pose);

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(bodyPose3, result.at<Pose3>(x3)));

  // Check derivatives

  // Calculate expected derivative for point (easiest to check)
  SmartFactor::Cameras cameras = smartFactor1->cameras(values);
  boost::function<Vector(Point3)> f = //
      boost::bind(&SmartFactor::whitenedErrors, *smartFactor1, cameras, _1);
  boost::optional<Point3> point = smartFactor1->point();
  CHECK(point);

  // Note ! After refactor the noiseModel is only in the factors, not these matrices
  Matrix expectedE = sigma * numericalDerivative11<Vector, Point3>(f, *point);

  // Calculate using computeEP
  Matrix actualE, PointCov;
  smartFactor1->computeEP(actualE, PointCov, values);
  EXPECT(assert_equal(expectedE, actualE, 1e-7));

  // Calculate using whitenedError
  Matrix E;
  SmartFactor::Cameras::FBlocks F;
  Vector actualErrors = smartFactor1->reprojectionErrors(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  // Success ! The derivatives of reprojectionErrors now agree with f !
  EXPECT(assert_equal(f(*point) * sigma, actualErrors, 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Factors ) {

  // Default cameras for simple derivatives
  Rot3 R;
  static Cal3_S2::shared_ptr sharedK(new Cal3_S2(100, 100, 0, 0, 0));
  PinholePose<Cal3_S2> cam1(Pose3(R, Point3(0, 0, 0)), sharedK), cam2(
      Pose3(R, Point3(1, 0, 0)), sharedK);

  // one landmarks 1m in front of camera
  Point3 landmark1(0, 0, 10);

  vector<Point2> measurements_cam1;

  // Project 2 landmarks into 2 cameras
  measurements_cam1.push_back(cam1.project(landmark1));
  measurements_cam1.push_back(cam2.project(landmark1));

  // Create smart factors
  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  SmartFactor::shared_ptr smartFactor1 = boost::make_shared<SmartFactor>();
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::Cameras cameras;
  cameras.push_back(cam1);
  cameras.push_back(cam2);

  // Make sure triangulation works
  LONGS_EQUAL(2, smartFactor1->triangulateSafe(cameras));
  CHECK(!smartFactor1->isDegenerate());
  CHECK(!smartFactor1->isPointBehindCamera());
  boost::optional<Point3> p = smartFactor1->point();
  CHECK(p);
  EXPECT(assert_equal(landmark1, *p));

  // After eliminating the point, A1 and A2 contain 2-rank information on cameras:
  Matrix16 A1, A2;
  A1 << -1000, 0, 0, 0, 100, 0;
  A2 << 1000, 0, 100, 0, -100, 0;
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

    boost::shared_ptr<RegularHessianFactor<6> > actual =
        smartFactor1->createHessianFactor(cameras, 0.0);
    EXPECT(assert_equal(expected.information(), actual->information(), 1e-8));
    EXPECT(assert_equal(expected, *actual, 1e-8));
  }

  {
    Matrix26 F1;
    F1.setZero();
    F1(0, 1) = -100;
    F1(0, 3) = -10;
    F1(1, 0) = 100;
    F1(1, 4) = -10;
    Matrix26 F2;
    F2.setZero();
    F2(0, 1) = -101;
    F2(0, 3) = -10;
    F2(0, 5) = -1;
    F2(1, 0) = 100;
    F2(1, 2) = 10;
    F2(1, 4) = -10;
    Matrix43 E;
    E.setZero();
    E(0, 0) = 100;
    E(1, 1) = 100;
    E(2, 0) = 100;
    E(2, 2) = 10;
    E(3, 1) = 100;
    const vector<pair<Key, Matrix26> > Fblocks = list_of<pair<Key, Matrix> > //
        (make_pair(x1, 10 * F1))(make_pair(x2, 10 * F2));
    Matrix3 P = (E.transpose() * E).inverse();
    Vector4 b;
    b.setZero();

    // createRegularImplicitSchurFactor
    RegularImplicitSchurFactor<6> expected(Fblocks, E, P, b);

    boost::shared_ptr<RegularImplicitSchurFactor<6> > actual =
        smartFactor1->createRegularImplicitSchurFactor(cameras, 0.0);
    CHECK(actual);
    CHECK(assert_equal(expected, *actual));

    // createJacobianQFactor
    JacobianFactorQ<6, 2> expectedQ(Fblocks, E, P, b);

    boost::shared_ptr<JacobianFactorQ<6, 2> > actualQ =
        smartFactor1->createJacobianQFactor(cameras, 0.0);
    CHECK(actual);
    EXPECT(assert_equal(expectedQ.information(), actualQ->information(), 1e-8));
    EXPECT(assert_equal(expectedQ, *actualQ));
  }

  {
    // createJacobianSVDFactor
    Vector1 b;
    b.setZero();
    double s = sin(M_PI_4);
    JacobianFactor expected(x1, s * A1, x2, s * A2, b);

    boost::shared_ptr<JacobianFactor> actual =
        smartFactor1->createJacobianSVDFactor(cameras, 0.0);
    CHECK(actual);
    EXPECT(assert_equal(expected.information(), actual->information(), 1e-8));
    EXPECT(assert_equal(expected, *actual));
  }
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_iterative_smart_projection_factor ) {
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks **********************" << endl;

  using namespace vanillaPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, jacobianSVD ) {

  using namespace vanillaPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above * noise_pose);

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, landmarkDistance ) {

  using namespace vanillaPose;

  double excludeLandmarksFutherThanDist = 2;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above * noise_pose);

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, dynamicOutlierRejection ) {

  using namespace vanillaPose;

  double excludeLandmarksFutherThanDist = 1e10;
  double dynamicOutlierRejectionThreshold = 1; // max 1 pixel of average reprojection error

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // add fourth landmark
  Point3 landmark4(5, -0.5, 1);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3,
      measurements_cam4;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
  measurements_cam4.at(0) = measurements_cam4.at(0) + Point2(10, 10); // add outlier

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor4(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor4->add(measurements_cam4, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3)));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, jacobianQ ) {

  using namespace vanillaPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, boost::none, JACOBIAN_Q));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above * noise_pose);

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_projection_factor ) {
  //  cout << " ************************ Normal ProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  using namespace vanillaPose2;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  NonlinearFactorGraph graph;

  // Project three landmarks into three cameras and triangulate
  graph.push_back(
      ProjectionFactor(cam1.project(landmark1), model, x1, L(1), sharedK2));
  graph.push_back(
      ProjectionFactor(cam2.project(landmark1), model, x2, L(1), sharedK2));
  graph.push_back(
      ProjectionFactor(cam3.project(landmark1), model, x3, L(1), sharedK2));

  graph.push_back(
      ProjectionFactor(cam1.project(landmark2), model, x1, L(2), sharedK2));
  graph.push_back(
      ProjectionFactor(cam2.project(landmark2), model, x2, L(2), sharedK2));
  graph.push_back(
      ProjectionFactor(cam3.project(landmark2), model, x3, L(2), sharedK2));

  graph.push_back(
      ProjectionFactor(cam1.project(landmark3), model, x1, L(3), sharedK2));
  graph.push_back(
      ProjectionFactor(cam2.project(landmark3), model, x2, L(3), sharedK2));
  graph.push_back(
      ProjectionFactor(cam3.project(landmark3), model, x3, L(3), sharedK2));

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above * noise_pose);
  values.insert(L(1), landmark1);
  values.insert(L(2), landmark2);
  values.insert(L(3), landmark3);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  if (isDebugTest)
    result.at<Pose3>(x3).print("Pose3 after optimization: ");
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, CheckHessian) {

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  using namespace vanillaPose;

  // Two slightly different cameras
  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedK);
  Camera cam3(pose3, sharedK);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  double rankTol = 10;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(rankTol));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(rankTol));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(rankTol));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

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
  // cout << AugInformationMatrix.size() << endl;
  Vector InfoVector = AugInformationMatrix.block(0, 18, 18, 1); // 18x18 Hessian + information vector

  // Check Hessian
  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-8));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_2land_rotation_only_smart_projection_factor ) {
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 2 landmarks: Rotation Only**********************" << endl;

  using namespace vanillaPose2;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // Two different cameras
  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedK2);
  Camera cam3(pose3, sharedK2);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);

  double rankTol = 50;
  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, sharedK2);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, sharedK2);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right * noise_pose);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  // EXPECT(assert_equal(pose_above,result.at<Pose3>(x3)));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, 3poses_rotation_only_smart_projection_factor ) {
  // cout << " ************************ SmartProjectionPoseFactor: 3 cams + 3 landmarks: Rotation Only**********************" << endl;

  using namespace vanillaPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // Two different cameras
  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedK);
  Camera cam3(pose3, sharedK);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  double rankTol = 10;

  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, sharedK);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model, sharedK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  // EXPECT(assert_equal(pose_above,result.at<Pose3>(x3)));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Hessian ) {
  // cout << " ************************ SmartProjectionPoseFactor: Hessian **********************" << endl;

  using namespace vanillaPose2;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  // Project three landmarks into 2 cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  vector<Point2> measurements_cam1;
  measurements_cam1.push_back(cam1_uv1);
  measurements_cam1.push_back(cam2_uv1);

  SmartFactor::shared_ptr smartFactor1(new SmartFactor());
  smartFactor1->add(measurements_cam1, views, model, sharedK2);

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor1->linearize(
      values);
  if (isDebugTest)
    hessianFactor->print("Hessian factor \n");

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with hessianFactor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, HessianWithRotation ) {
  // cout << " ************************ SmartProjectionPoseFactor: rotated Hessian **********************" << endl;

  using namespace vanillaPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);

  SmartFactor::shared_ptr smartFactorInstance(new SmartFactor());
  smartFactorInstance->add(measurements_cam1, views, model, sharedK);

  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  boost::shared_ptr<GaussianFactor> hessianFactor =
      smartFactorInstance->linearize(values);

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(level_pose));
  rotValues.insert(x2, poseDrift.compose(pose_right));
  rotValues.insert(x3, poseDrift.compose(pose_above));

  boost::shared_ptr<GaussianFactor> hessianFactorRot =
      smartFactorInstance->linearize(rotValues);

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRot->information(), 1e-7));

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(level_pose));
  tranValues.insert(x2, poseDrift2.compose(pose_right));
  tranValues.insert(x3, poseDrift2.compose(pose_above));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran =
      smartFactorInstance->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRotTran->information(), 1e-7));
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, HessianWithRotationDegenerate ) {
  // cout << " ************************ SmartProjectionPoseFactor: rotated Hessian (degenerate) **********************" << endl;

  using namespace vanillaPose2;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);

  SmartFactor::shared_ptr smartFactor(new SmartFactor());
  smartFactor->add(measurements_cam1, views, model, sharedK2);

  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);

  boost::shared_ptr<GaussianFactor> hessianFactor = smartFactor->linearize(
      values);
  if (isDebugTest)
    hessianFactor->print("Hessian factor \n");

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, poseDrift.compose(level_pose));
  rotValues.insert(x2, poseDrift.compose(pose_right));
  rotValues.insert(x3, poseDrift.compose(pose_above));

  boost::shared_ptr<GaussianFactor> hessianFactorRot = smartFactor->linearize(
      rotValues);
  if (isDebugTest)
    hessianFactorRot->print("Hessian factor \n");

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRot->information(), 1e-8));

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, poseDrift2.compose(level_pose));
  tranValues.insert(x2, poseDrift2.compose(pose_right));
  tranValues.insert(x3, poseDrift2.compose(pose_above));

  boost::shared_ptr<GaussianFactor> hessianFactorRotTran =
      smartFactor->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(
      assert_equal(hessianFactor->information(),
          hessianFactorRotTran->information(), 1e-8));
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, ConstructorWithCal3Bundler) {
  SmartFactorBundler factor(rankTol, linThreshold);
  boost::shared_ptr<Cal3Bundler> sharedBundlerK(
      new Cal3Bundler(500, 1e-3, 1e-3, 1000, 2000));
  factor.add(measurement1, x1, model, sharedBundlerK);
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Cal3Bundler ) {
  // cout << " ************************ SmartProjectionPoseFactor: Cal3Bundler **********************" << endl;

  using namespace bundlerPose;

  // three landmarks ~5 meters infront of camera
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
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

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  SmartFactorBundler::shared_ptr smartFactor1(new SmartFactorBundler());
  smartFactor1->add(measurements_cam1, views, model, sharedBundlerK);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler());
  smartFactor2->add(measurements_cam2, views, model, sharedBundlerK);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler());
  smartFactor3->add(measurements_cam3, views, model, sharedBundlerK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(PriorFactor<Pose3>(x2, pose_right, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
  if (isDebugTest)
    tictoc_print_();
}

/* *************************************************************************/
TEST( SmartProjectionPoseFactor, Cal3BundlerRotationOnly ) {

  using namespace bundlerPose;

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);
  views.push_back(x3);

  // Two different cameras
  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedBundlerK);
  Camera cam3(pose3, sharedBundlerK);

  // landmark3 at 3 meters now
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
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

  SmartFactorBundler::shared_ptr smartFactor1(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model, sharedBundlerK);

  SmartFactorBundler::shared_ptr smartFactor2(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model, sharedBundlerK);

  SmartFactorBundler::shared_ptr smartFactor3(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model, sharedBundlerK);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Pose3>(x1, level_pose, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, pose_above * noise_pose);
  if (isDebugTest)
    values.at<Pose3>(x3).print("Smart: Pose3 before optimization: ");

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_ (SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  if (isDebugTest)
    result.at<Pose3>(x3).print("Smart: Pose3 after optimization: ");
  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  // EXPECT(assert_equal(pose_above,result.at<Pose3>(x3)));
  if (isDebugTest)
    tictoc_print_();
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

