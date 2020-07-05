/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StereoVOExample.cpp
 * @brief A stereo visual odometry example
 * @date May 25, 2014
 * @author Stephen Camp
 */

/**
 * A 3D stereo visual odometry example
 *  - robot starts at origin
 *  -moves forward 1 meter
 *  -takes stereo readings on three landmarks
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // create graph object, add first pose at origin with key '1'
  NonlinearFactorGraph graph;
  Pose3 first_pose;
  graph.emplace_shared<NonlinearEquality<Pose3> >(1, Pose3());

  // create factor noise model with 3 sigmas of value 1
  const auto model = noiseModel::Isotropic::Sigma(3, 1);
  // create stereo camera calibration object with .2m between cameras
  const Cal3_S2Stereo::shared_ptr K(
      new Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));

  //create and add stereo factors between first pose (key value 1) and the three landmarks
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(520, 480, 440), model, 1, 3, K);
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(120, 80, 440), model, 1, 4, K);
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(320, 280, 140), model, 1, 5, K);

  //create and add stereo factors between second pose and the three landmarks
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(570, 520, 490), model, 2, 3, K);
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(70, 20, 490), model, 2, 4, K);
  graph.emplace_shared<GenericStereoFactor<Pose3,Point3> >(StereoPoint2(320, 270, 115), model, 2, 5, K);

  // create Values object to contain initial estimates of camera poses and
  // landmark locations
  Values initial_estimate;

  // create and add iniital estimates
  initial_estimate.insert(1, first_pose);
  initial_estimate.insert(2, Pose3(Rot3(), Point3(0.1, -0.1, 1.1)));
  initial_estimate.insert(3, Point3(1, 1, 5));
  initial_estimate.insert(4, Point3(-1, 1, 5));
  initial_estimate.insert(5, Point3(0, -0.5, 5));

  // create Levenberg-Marquardt optimizer for resulting factor graph, optimize
  LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
  Values result = optimizer.optimize();

  result.print("Final result:\n");

  return 0;
}
