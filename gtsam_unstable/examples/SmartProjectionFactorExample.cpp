/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file SmartProjectionFactorExample.cpp
* @brief A stereo visual odometry example
* @date May 30, 2014
* @author Stephen Camp
* @author Chris Beall
*/


/**
 * A smart projection factor example based on stereo data, throwing away the
 * measurement from the right camera
 *  -robot starts at origin
 *  -moves forward, taking periodic stereo measurements
 *  -makes monocular observations of many landmarks
 */

#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv){
  typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

  Values initial_estimate;
  NonlinearFactorGraph graph;
  const noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,1);

  string calibration_loc = findExampleDataFile("VO_calibration.txt");
  string pose_loc = findExampleDataFile("VO_camera_poses_large.txt");
  string factor_loc = findExampleDataFile("VO_stereo_factors_large.txt");

  //read camera calibration info from file
  // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
  cout << "Reading calibration info" << endl;
  ifstream calibration_file(calibration_loc.c_str());

  double fx, fy, s, u0, v0, b;
  calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;
  const Cal3_S2::shared_ptr K(new Cal3_S2(fx, fy, s, u0, v0));

  cout << "Reading camera poses" << endl;
  ifstream pose_file(pose_loc.c_str());

  int pose_index;
  MatrixRowMajor m(4,4);
  //read camera pose parameters and use to make initial estimates of camera poses
  while (pose_file >> pose_index) {
    for (int i = 0; i < 16; i++)
      pose_file >> m.data()[i];
    initial_estimate.insert(pose_index, Pose3(m));
  }

  // landmark keys
  size_t landmark_key;

  // pixel coordinates uL, uR, v (same for left/right images due to rectification)
  // landmark coordinates X, Y, Z in camera frame, resulting from triangulation
  double uL, uR, v, X, Y, Z;
  ifstream factor_file(factor_loc.c_str());
  cout << "Reading stereo factors" << endl;

  //read stereo measurements and construct smart factors

  SmartFactor::shared_ptr factor(new SmartFactor(model, K));
  size_t current_l = 3;   // hardcoded landmark ID from first measurement

  while (factor_file >> pose_index >> landmark_key >> uL >> uR >> v >> X >> Y >> Z) {

    if(current_l != landmark_key) {
      graph.push_back(factor);
      factor = SmartFactor::shared_ptr(new SmartFactor(model, K));
      current_l = landmark_key;
    }
    factor->add(Point2(uL,v), pose_index);
  }

  Pose3 firstPose = initial_estimate.at<Pose3>(1);
  //constrain the first pose such that it cannot change from its original value during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  graph.emplace_shared<NonlinearEquality<Pose3> >(1,firstPose);

  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;

  cout << "Optimizing" << endl;
  //create Levenberg-Marquardt optimizer to optimize the factor graph
  LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
  Values result = optimizer.optimize();

  cout << "Final result sample:" << endl;
  Values pose_values = result.filter<Pose3>();
  pose_values.print("Final camera poses:\n");

  return 0;
}
