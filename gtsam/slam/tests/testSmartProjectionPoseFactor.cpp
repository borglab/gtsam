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
 *  @author Frank Dellaert
 *  @date   Sept 2013
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;

static bool isDebugTest = true;

static const double rankTol = 1.0;
static const double linThreshold = -1.0;
static const bool manageDegeneracy = true;

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
  factor1->add(measurement1, x1, model);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Constructor4) {
  using namespace vanillaPose;
  SmartFactor factor1(rankTol, linThreshold);
  factor1.add(measurement1, x1, model);
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, Equals ) {
  using namespace vanillaPose;
  SmartFactor::shared_ptr factor1(new SmartFactor());
  factor1->add(measurement1, x1, model);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  factor2->add(measurement1, x1, model);

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
  factor.add(level_uv, x1, model);
  factor.add(level_uv_right, x2, model);

  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartFactor::Cameras cameras = factor.cameras(values);
  double actualError2 = factor.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // Calculate expected derivative for point (easiest to check)
  boost::function<Vector(Point3)> f = //
      boost::bind(&SmartFactor::whitenedErrors, factor, cameras, _1);

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
  SmartFactor::Cameras::FBlocks F;
  Matrix E;
  Vector actualErrors = factor.reprojectionError(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  EXPECT(assert_equal(zero(4), actualErrors, 1e-7));

  // Calculate using computeJacobians
  Vector b;
  vector<SmartFactor::KeyMatrix2D> Fblocks;
  double actualError3 = factor.computeJacobians(Fblocks, E, b, cameras, *point);
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
  values.insert(x1, cam1);
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  values.insert(x2, Camera(pose_right.compose(noise_pose), sharedK));

  SmartFactor::shared_ptr factor(new SmartFactor());
  factor->add(level_uv, x1, model);
  factor->add(level_uv_right, x2, model);

  double actualError1 = factor->error(values);

  SmartFactor::shared_ptr factor2(new SmartFactor());
  vector<Point2> measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  vector<SharedNoiseModel> noises;
  noises.push_back(model);
  noises.push_back(model);

  vector<Key> views;
  views.push_back(x1);
  views.push_back(x2);

  factor2->add(measurements, views, noises);
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
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedK2));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)), values.at<Camera>(x3).pose()));

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

//  GaussianFactorGraph::shared_ptr GFG = graph.linearize(values);
//  VectorValues delta = GFG->optimize();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  EXPECT(assert_equal(cam3, result.at<Camera>(x3), 1e-8));
  if (isDebugTest)
    tictoc_print_();
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
  smartFactor1->add(measurements_cam1, views, model);

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

  // After eliminating the point, A1 and A2 contain 2-rank information on cameras:
  Matrix16 A1, A2;
  A1 << -10, 0, 0, 0, 1, 0;
  A2 << 10, 0, 1, 0, -1, 0;
  A1 *= 10. / sigma;
  A2 *= 10. / sigma;
  Matrix expectedInformation; // filled below
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

    boost::shared_ptr<RegularHessianFactor<6> > actual =
        smartFactor1->createHessianFactor(cameras, 0.0);
    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-8));
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
    Matrix E(4, 3);
    E.setZero();
    E(0, 0) = 10;
    E(1, 1) = 10;
    E(2, 0) = 10;
    E(2, 2) = 1;
    E(3, 1) = 10;
    vector<pair<Key, Matrix26> > Fblocks = list_of<pair<Key, Matrix> > //
        (make_pair(x1, F1))(make_pair(x2, F2));
    Vector b(4);
    b.setZero();

    // createJacobianQFactor
    SharedIsotropic n = noiseModel::Isotropic::Sigma(4, sigma);
    Matrix3 P = (E.transpose() * E).inverse();
    JacobianFactorQ<6, 2> expectedQ(Fblocks, E, P, b, n);
    EXPECT(assert_equal(expectedInformation, expectedQ.information(), 1e-8));

    boost::shared_ptr<JacobianFactorQ<6, 2> > actualQ =
        smartFactor1->createJacobianQFactor(cameras, 0.0);
    CHECK(actualQ);
    EXPECT(assert_equal(expectedInformation, actualQ->information(), 1e-8));
    EXPECT(assert_equal(expectedQ, *actualQ));

    // Whiten for RegularImplicitSchurFactor (does not have noise model)
    model->WhitenSystem(E, b);
    Matrix3 whiteP = (E.transpose() * E).inverse();
    BOOST_FOREACH(SmartFactor::KeyMatrix2D& Fblock,Fblocks)
      Fblock.second = model->Whiten(Fblock.second);

    // createRegularImplicitSchurFactor
    RegularImplicitSchurFactor<6> expected(Fblocks, E, whiteP, b);

    boost::shared_ptr<RegularImplicitSchurFactor<6> > actual =
        smartFactor1->createRegularImplicitSchurFactor(cameras, 0.0);
    CHECK(actual);
    EXPECT(assert_equal(expectedInformation, expected.information(), 1e-8));
    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-8));
    EXPECT(assert_equal(expected, *actual));
  }

  {
    // createJacobianSVDFactor
    Vector1 b;
    b.setZero();
    double s = sigma * sin(M_PI_4);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(4-3, sigma);
    JacobianFactor expected(x1, s * A1, x2, s * A2, b, n);
    EXPECT(assert_equal(expectedInformation, expected.information(), 1e-8));

    boost::shared_ptr<JacobianFactor> actual =
        smartFactor1->createJacobianSVDFactor(cameras, 0.0);
    CHECK(actual);
    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-8));
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
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor());
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor());
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(1.11022302e-16, -0.0314107591, 0.99950656, -0.99950656,
                  -0.0313952598, -0.000986635786, 0.0314107591, -0.999013364,
                  -0.0313952598), Point3(0.1, -0.1, 1.9)),
          values.at<Camera>(x3).pose()));

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  EXPECT(assert_equal(cam3, result.at<Camera>(x3), 1e-7));
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
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD));
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(cam3, result.at<Camera>(x3), 1e-8));
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
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist));
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(values.at<Camera>(x3), result.at<Camera>(x3)));
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
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor3->add(measurements_cam3, views, model);

  SmartFactor::shared_ptr smartFactor4(
      new SmartFactor(1, -1, false, false, JACOBIAN_SVD,
          excludeLandmarksFutherThanDist, dynamicOutlierRejectionThreshold));
  smartFactor4->add(measurements_cam4, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(smartFactor4);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));

  // All factors are disabled and pose should remain where it is
  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(cam3, result.at<Camera>(x3)));
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
      new SmartFactor(1, -1, false, false, JACOBIAN_Q));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(1, -1, false, false, JACOBIAN_Q));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(1, -1, false, false, JACOBIAN_Q));
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));

  LevenbergMarquardtParams params;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  EXPECT(assert_equal(cam3, result.at<Camera>(x3), 1e-8));
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

  DOUBLES_EQUAL(48406055, graph.error(values), 1);

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  DOUBLES_EQUAL(0, graph.error(result), 1e-9);

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
  Pose3 pose2 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Camera cam2(pose2, sharedK);
  Camera cam3(pose3, sharedK);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  double rankTol = 10;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(rankTol));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(new SmartFactor(rankTol));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(new SmartFactor(rankTol));
  smartFactor3->add(measurements_cam3, views, model);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                  -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          values.at<Camera>(x3).pose()));

  boost::shared_ptr<GaussianFactor> factor1 = smartFactor1->linearize(values);
  boost::shared_ptr<GaussianFactor> factor2 = smartFactor2->linearize(values);
  boost::shared_ptr<GaussianFactor> factor3 = smartFactor3->linearize(values);

  Matrix CumulativeInformation = factor1->information() + factor2->information()
      + factor3->information();

  boost::shared_ptr<GaussianFactorGraph> GaussianGraph = graph.linearize(
      values);
  Matrix GraphInformation = GaussianGraph->hessian().first;

  // Check Hessian
  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-8));

  Matrix AugInformationMatrix = factor1->augmentedInformation()
      + factor2->augmentedInformation() + factor3->augmentedInformation();

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
  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3());
  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3());
  Camera cam2(pose2, sharedK2);
  Camera cam3(pose3, sharedK2);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras and triangulate
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);

  double rankTol = 50;
  SmartFactor::shared_ptr smartFactor1(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Camera>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Camera>(x3, positionPrior, noisePriorTranslation));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, Camera(pose2 * noise_pose, sharedK2));
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose3 * noise_pose * noise_pose, sharedK2));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.154256096, -0.632754061, 0.75883289, -0.753276814,
                  -0.572308662, -0.324093872, 0.639358349, -0.521617766,
                  -0.564921063),
              Point3(0.145118171, -0.252907438, 0.819956033)),
          values.at<Camera>(x3).pose()));

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.154256096, -0.632754061, 0.75883289, -0.753276814,
                  -0.572308662, -0.324093872, 0.639358349, -0.521617766,
                  -0.564921063),
              Point3(0.145118171, -0.252907438, 0.819956033)),
          result.at<Camera>(x3).pose()));
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
  Pose3 pose2 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
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
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactor::shared_ptr smartFactor2(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactor::shared_ptr smartFactor3(
      new SmartFactor(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedK));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                  -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          values.at<Camera>(x3).pose()));

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  // result.print("results of 3 camera, 3 landmark optimization \n");
  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                  -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          result.at<Camera>(x3).pose()));
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
  smartFactor1->add(measurements_cam1, views, model);

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);

  boost::shared_ptr<GaussianFactor> factor = smartFactor1->linearize(values);
  if (isDebugTest)
    factor->print("Hessian factor \n");

  // compute triangulation from linearization point
  // compute reprojection errors (sum squared)
  // compare with factor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
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
  smartFactorInstance->add(measurements_cam1, views, model);

  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, cam3);

  boost::shared_ptr<GaussianFactor> factor = smartFactorInstance->linearize(
      values);

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, Camera(poseDrift.compose(level_pose), sharedK));
  rotValues.insert(x2, Camera(poseDrift.compose(pose_right), sharedK));
  rotValues.insert(x3, Camera(poseDrift.compose(pose_above), sharedK));

  boost::shared_ptr<GaussianFactor> factorRot = smartFactorInstance->linearize(
      rotValues);

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(assert_equal(factor->information(), factorRot->information(), 1e-7));

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, Camera(poseDrift2.compose(level_pose), sharedK));
  tranValues.insert(x2, Camera(poseDrift2.compose(pose_right), sharedK));
  tranValues.insert(x3, Camera(poseDrift2.compose(pose_above), sharedK));

  boost::shared_ptr<GaussianFactor> factorRotTran =
      smartFactorInstance->linearize(tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(
      assert_equal(factor->information(), factorRotTran->information(), 1e-7));
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
  smartFactor->add(measurements_cam1, views, model);

  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  values.insert(x3, cam3);

  boost::shared_ptr<GaussianFactor> factor = smartFactor->linearize(values);
  if (isDebugTest)
    factor->print("Hessian factor \n");

  Pose3 poseDrift = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));

  Values rotValues;
  rotValues.insert(x1, Camera(poseDrift.compose(level_pose), sharedK2));
  rotValues.insert(x2, Camera(poseDrift.compose(pose_right), sharedK2));
  rotValues.insert(x3, Camera(poseDrift.compose(pose_above), sharedK2));

  boost::shared_ptr<GaussianFactor> factorRot = smartFactor->linearize(
      rotValues);
  if (isDebugTest)
    factorRot->print("Hessian factor \n");

  // Hessian is invariant to rotations in the nondegenerate case
  EXPECT(assert_equal(factor->information(), factorRot->information(), 1e-8));

  Pose3 poseDrift2 = Pose3(Rot3::ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
      Point3(10, -4, 5));

  Values tranValues;
  tranValues.insert(x1, Camera(poseDrift2.compose(level_pose), sharedK2));
  tranValues.insert(x2, Camera(poseDrift2.compose(pose_right), sharedK2));
  tranValues.insert(x3, Camera(poseDrift2.compose(pose_above), sharedK2));

  boost::shared_ptr<GaussianFactor> factorRotTran = smartFactor->linearize(
      tranValues);

  // Hessian is invariant to rotations and translations in the nondegenerate case
  EXPECT(
      assert_equal(factor->information(), factorRotTran->information(), 1e-8));
}

/* ************************************************************************* */
TEST( SmartProjectionPoseFactor, ConstructorWithCal3Bundler) {
  SmartFactorBundler factor(rankTol, linThreshold);
  factor.add(measurement1, x1, model);
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
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactorBundler::shared_ptr smartFactor2(new SmartFactorBundler());
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactorBundler::shared_ptr smartFactor3(new SmartFactorBundler());
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(PriorFactor<Camera>(x2, cam2, noisePrior));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedBundlerK));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
              Point3(0.1, -0.1, 1.9)), values.at<Camera>(x3).pose()));
  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  EXPECT(assert_equal(cam3, result.at<Camera>(x3), 1e-6));
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
  Pose3 pose2 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
  Pose3 pose3 = level_pose
      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
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
  smartFactor1->add(measurements_cam1, views, model);

  SmartFactorBundler::shared_ptr smartFactor2(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor2->add(measurements_cam2, views, model);

  SmartFactorBundler::shared_ptr smartFactor3(
      new SmartFactorBundler(rankTol, linThreshold, manageDegeneracy));
  smartFactor3->add(measurements_cam3, views, model);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
      0.10);
  Point3 positionPrior = Point3(0, 0, 1);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.push_back(PriorFactor<Camera>(x1, cam1, noisePrior));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x2, positionPrior, noisePriorTranslation));
  graph.push_back(
      PoseTranslationPrior<Pose3>(x3, positionPrior, noisePriorTranslation));

  //  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI / 100, 0., -M_PI / 100),
      Point3(0.1, 0.1, 0.1)); // smaller noise
  Values values;
  values.insert(x1, cam1);
  values.insert(x2, cam2);
  // initialize third pose with some noise, we expect it to move back to original pose_above
  values.insert(x3, Camera(pose_above * noise_pose, sharedBundlerK));
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                  -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          values.at<Camera>(x3).pose()));

  LevenbergMarquardtParams params;
  if (isDebugTest)
    params.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
  if (isDebugTest)
    params.verbosity = NonlinearOptimizerParams::ERROR;

  Values result;
  gttic_(SmartProjectionPoseFactor);
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  result = optimizer.optimize();
  gttoc_(SmartProjectionPoseFactor);
  tictoc_finishedIteration_();

  cout
      << "TEST COMMENTED: rotation only version of smart factors has been deprecated "
      << endl;
  EXPECT(
      assert_equal(
          Pose3(
              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
                  -0.130455917),
              Point3(0.0897734171, -0.110201006, 0.901022872)),
          values.at<Camera>(x3).pose()));
  if (isDebugTest)
    tictoc_print_();
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

