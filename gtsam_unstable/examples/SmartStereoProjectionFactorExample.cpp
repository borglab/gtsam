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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv){

  typedef SmartStereoProjectionPoseFactor SmartFactor;

  bool output_poses = true;
  string poseOutput("../../../examples/data/optimized_poses.txt");
  string init_poseOutput("../../../examples/data/initial_poses.txt");
  Values initial_estimate;
  NonlinearFactorGraph graph;
  const noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(3,1);
  ofstream pose3Out, init_pose3Out;

  bool add_initial_noise = true;

  string calibration_loc = findExampleDataFile("VO_calibration.txt");
  string pose_loc = findExampleDataFile("VO_camera_poses_large.txt");
  string factor_loc = findExampleDataFile("VO_stereo_factors_large.txt");

  //read camera calibration info from file
  // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
  cout << "Reading calibration info" << endl;
  ifstream calibration_file(calibration_loc.c_str());

  double fx, fy, s, u0, v0, b;
  calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;
  const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, s, u0, v0,b));

  cout << "Reading camera poses" << endl;
  ifstream pose_file(pose_loc.c_str());

  int pose_id;
  MatrixRowMajor m(4,4);
  //read camera pose parameters and use to make initial estimates of camera poses
  while (pose_file >> pose_id) {
    for (int i = 0; i < 16; i++) {
      pose_file >> m.data()[i];
    }
    if(add_initial_noise){
      m(1,3) += (pose_id % 10)/10.0;
    }
    initial_estimate.insert(Symbol('x', pose_id), Pose3(m));
  }

  Values initial_pose_values = initial_estimate.filter<Pose3>();
  if (output_poses) {
    init_pose3Out.open(init_poseOutput.c_str(), ios::out);
    for (size_t i = 1; i <= initial_pose_values.size(); i++) {
      init_pose3Out
          << i << " "
          << initial_pose_values.at<Pose3>(Symbol('x', i)).matrix().format(
              Eigen::IOFormat(Eigen::StreamPrecision, 0, " ", " ")) << endl;
    }
  }

  // camera and landmark keys
  size_t x, l;

  // pixel coordinates uL, uR, v (same for left/right images due to rectification)
  // landmark coordinates X, Y, Z in camera frame, resulting from triangulation
  double uL, uR, v, X, Y, Z;
  ifstream factor_file(factor_loc.c_str());
  cout << "Reading stereo factors" << endl;

  //read stereo measurements and construct smart factors

  SmartFactor::shared_ptr factor(new SmartFactor(model));
  size_t current_l = 3;   // hardcoded landmark ID from first measurement

  while (factor_file >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {

    if(current_l != l) {
      graph.push_back(factor);
      factor = SmartFactor::shared_ptr(new SmartFactor(model));
      current_l = l;
    }
    factor->add(StereoPoint2(uL,uR,v), Symbol('x',x), K);
  }

  Pose3 first_pose = initial_estimate.at<Pose3>(Symbol('x',1));
  //constrain the first pose such that it cannot change from its original value during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  graph.emplace_shared<NonlinearEquality<Pose3> >(Symbol('x',1),first_pose);

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

  if(output_poses){
    pose3Out.open(poseOutput.c_str(),ios::out);
    for(size_t i = 1; i<=pose_values.size(); i++){
      pose3Out << i << " " << pose_values.at<Pose3>(Symbol('x',i)).matrix().format(Eigen::IOFormat(Eigen::StreamPrecision, 0,
        " ", " ")) << endl;
    }
    cout << "Writing output" << endl;
  }

  return 0;
}
