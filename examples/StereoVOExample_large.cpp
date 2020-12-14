/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file StereoVOExample_large.cpp
* @brief A stereo visual odometry example
* @date May 25, 2014
* @author Stephen Camp
*/


/**
 * A 3D stereo visual odometry example
 *  - robot starts at origin
 *  -moves forward, taking periodic stereo measurements
 *  -takes stereo readings of many landmarks
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  Values initial_estimate;
  NonlinearFactorGraph graph;
  const auto model = noiseModel::Isotropic::Sigma(3, 1);

  string calibration_loc = findExampleDataFile("VO_calibration.txt");
  string pose_loc = findExampleDataFile("VO_camera_poses_large.txt");
  string factor_loc = findExampleDataFile("VO_stereo_factors_large.txt");

  // read camera calibration info from file
  // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
  double fx, fy, s, u0, v0, b;
  ifstream calibration_file(calibration_loc.c_str());
  cout << "Reading calibration info" << endl;
  calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;

  // create stereo camera calibration object
  const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, s, u0, v0, b));

  ifstream pose_file(pose_loc.c_str());
  cout << "Reading camera poses" << endl;
  int pose_id;
  MatrixRowMajor m(4, 4);
  // read camera pose parameters and use to make initial estimates of camera
  // poses
  while (pose_file >> pose_id) {
    for (int i = 0; i < 16; i++) {
      pose_file >> m.data()[i];
    }
    initial_estimate.insert(Symbol('x', pose_id), Pose3(m));
  }

  // camera and landmark keys
  size_t x, l;

  // pixel coordinates uL, uR, v (same for left/right images due to
  // rectification) landmark coordinates X, Y, Z in camera frame, resulting from
  // triangulation
  double uL, uR, v, X, Y, Z;
  ifstream factor_file(factor_loc.c_str());
  cout << "Reading stereo factors" << endl;
  // read stereo measurement details from file and use to create and add
  // GenericStereoFactor objects to the graph representation
  while (factor_file >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
    graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(
        StereoPoint2(uL, uR, v), model, Symbol('x', x), Symbol('l', l), K);
    // if the landmark variable included in this factor has not yet been added
    // to the initial variable value estimate, add it
    if (!initial_estimate.exists(Symbol('l', l))) {
      Pose3 camPose = initial_estimate.at<Pose3>(Symbol('x', x));
      // transformFrom() transforms the input Point3 from the camera pose space,
      // camPose, to the global space
      Point3 worldPoint = camPose.transformFrom(Point3(X, Y, Z));
      initial_estimate.insert(Symbol('l', l), worldPoint);
    }
  }

  Pose3 first_pose = initial_estimate.at<Pose3>(Symbol('x', 1));
  // constrain the first pose such that it cannot change from its original value
  // during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  graph.emplace_shared<NonlinearEquality<Pose3> >(Symbol('x', 1), first_pose);

  cout << "Optimizing" << endl;
  // create Levenberg-Marquardt optimizer to optimize the factor graph
  LevenbergMarquardtParams params;
  params.orderingType = Ordering::METIS;
  LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
  Values result = optimizer.optimize();

  cout << "Final result sample:" << endl;
  Values pose_values = result.filter<Pose3>();
  pose_values.print("Final camera poses:\n");

  return 0;
}
