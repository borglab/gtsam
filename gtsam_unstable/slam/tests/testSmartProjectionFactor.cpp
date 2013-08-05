/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testProjectionFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2009
 */

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam_unstable/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

//typedef GenericProjectionFactor<Pose3, Point3> TestProjectionFactor;


/* ************************************************************************* *
TEST( MultiProjectionFactor, noiseless ){
  cout << " ************************ MultiProjectionFactor: noiseless ****************************" << endl;
  Values theta;
  NonlinearFactorGraph graph;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
//  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2; //, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  Values value;
  value.insert(x1, level_pose);
  value.insert(x2, level_pose_right);

//  poses += level_pose, level_pose_right;
  vector<Point2> measurements;
  measurements += level_uv, level_uv_right;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2> smartFactor(measurements, noiseProjection, views, K);

  double actualError = smartFactor.error(value);
  double expectedError = 0.0;

  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}

/* ************************************************************************* *
TEST( MultiProjectionFactor, noisy ){
  cout << " ************************ MultiProjectionFactor: noisy ****************************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
//  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2; //, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera level_camera(level_pose, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera level_camera_right(level_pose_right, *K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 pixelError(0.2,0.2);
  Point2 level_uv = level_camera.project(landmark) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark);

  Values values;
  values.insert(x1, level_pose);
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  values.insert(x2, level_pose_right.compose(noise_pose));

//  poses += level_pose, level_pose_right;
  vector<Point2> measurements;
  measurements += level_uv, level_uv_right;

  SmartProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr
    smartFactor(new SmartProjectionFactor<Pose3, Point3, Cal3_S2>(measurements, noiseProjection, views, K));

  double actualError = smartFactor->error(values);
  double expectedError = sqrt(0.08);

  // we do not expect to be able to predict the error, since the error on the pixel will change
  // the triangulation of the landmark which is internal to the factor.
  //  DOUBLES_EQUAL(expectedError, actualError, 1e-7);
}


/* ************************************************************************* */
TEST( MultiProjectionFactor, 3poses ){
  cout << " ************************ MultiProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  vector<Point2> measurements_cam1, measurements_cam2, measurements_cam3;

  // 1. Project three landmarks into three cameras and triangulate
  Point2 cam1_uv1 = cam1.project(landmark1);
  Point2 cam2_uv1 = cam2.project(landmark1);
  Point2 cam3_uv1 = cam3.project(landmark1);
  measurements_cam1 += cam1_uv1, cam2_uv1, cam3_uv1;

  //
  Point2 cam1_uv2 = cam1.project(landmark2);
  Point2 cam2_uv2 = cam2.project(landmark2);
  Point2 cam3_uv2 = cam3.project(landmark2);
  measurements_cam2 += cam1_uv2, cam2_uv2, cam3_uv2;


  Point2 cam1_uv3 = cam1.project(landmark3);
  Point2 cam2_uv3 = cam2.project(landmark3);
  Point2 cam3_uv3 = cam3.project(landmark3);
  measurements_cam3 += cam1_uv3, cam2_uv3, cam3_uv3;

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;

  SmartFactor::shared_ptr smartFactor1(new SmartFactor(measurements_cam1, noiseProjection, views, K));
  SmartFactor::shared_ptr smartFactor2(new SmartFactor(measurements_cam2, noiseProjection, views, K));
  SmartFactor::shared_ptr smartFactor3(new SmartFactor(measurements_cam3, noiseProjection, views, K));

//  double actualError = smartFactor->error(values);
//  double expectedError = sqrt(0.08);

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.add(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.add(PriorFactor<Pose3>(x2, pose2, noisePrior));


  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3* noise_pose);

  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  result.print("results of 3 camera, 3 landmark optimization \n");

}


///* ************************************************************************* */
TEST( MultiProjectionFactor, 3poses_projection_factor ){
  cout << " ************************ Normal ProjectionFactor: 3 cams + 3 landmarks **********************" << endl;

  Symbol x1('X',  1);
  Symbol x2('X',  2);
  Symbol x3('X',  3);

  const SharedDiagonal noiseProjection = noiseModel::Isotropic::Sigma(2, 1);

  std::vector<Key> views;
  views += x1, x2, x3;

  Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 pose1 = Pose3(Rot3::ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
  SimpleCamera cam1(pose1, *K);

  // create second camera 1 meter to the right of first camera
  Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1,0,0));
  SimpleCamera cam2(pose2, *K);

  // create third camera 1 meter above the first camera
  Pose3 pose3 = pose1 * Pose3(Rot3(), Point3(0,-1,0));
  pose3.print("Pose3: ");
  SimpleCamera cam3(pose3, *K);

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2);
  Point3 landmark2(5, -0.5, 1.2);
  Point3 landmark3(3, 0, 3.0);

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  NonlinearFactorGraph graph;

  // 1. Project three landmarks into three cameras and triangulate
  graph.add(ProjectionFactor(cam1.project(landmark1), noiseProjection, x1, L(1), K));
  graph.add(ProjectionFactor(cam2.project(landmark1), noiseProjection, x2, L(1), K));
  graph.add(ProjectionFactor(cam3.project(landmark1), noiseProjection, x3, L(1), K));

  //
  graph.add(ProjectionFactor(cam1.project(landmark2), noiseProjection, x1, L(2), K));
  graph.add(ProjectionFactor(cam2.project(landmark2), noiseProjection, x2, L(2), K));
  graph.add(ProjectionFactor(cam3.project(landmark2), noiseProjection, x3, L(2), K));

  graph.add(ProjectionFactor(cam1.project(landmark3), noiseProjection, x1, L(3), K));
  graph.add(ProjectionFactor(cam2.project(landmark3), noiseProjection, x2, L(3), K));
  graph.add(ProjectionFactor(cam3.project(landmark3), noiseProjection, x3, L(3), K));

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
  graph.add(PriorFactor<Pose3>(x1, pose1, noisePrior));
  graph.add(PriorFactor<Pose3>(x2, pose2, noisePrior));

  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));
  Values values;
  values.insert(x1, pose1);
  values.insert(x2, pose2);
  values.insert(x3, pose3* noise_pose);
  values.insert(L(1), landmark1);
  values.insert(L(2), landmark2);
  values.insert(L(3), landmark3);

  LevenbergMarquardtParams params;
//  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
//  params.verbosity = NonlinearOptimizerParams::ERROR;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  result.print("Regular Projection Factor: results of 3 camera, 3 landmark optimization \n");

}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

