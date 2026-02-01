/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file ConcurrentCalibration.cpp
* @brief First step towards estimating monocular calibration in concurrent
* filter/smoother framework. To start with, just batch LM.
* @date June 11, 2014
* @author Chris Beall
*/


#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/utilities.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/dataset.h>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using symbol_shorthand::K;
using symbol_shorthand::L;
using symbol_shorthand::X;

int main(int argc, char** argv){

  Values initial_estimate;
  NonlinearFactorGraph graph;
  const auto model = noiseModel::Isotropic::Sigma(2,1);

  string calibration_loc = findExampleDataFile("VO_calibration00s.txt");
  string pose_loc = findExampleDataFile("VO_camera_poses00s.txt");
  string factor_loc = findExampleDataFile("VO_stereo_factors00s.txt");

  //read camera calibration info from file
  // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
  double fx, fy, s, u0, v0, b;
  ifstream calibration_file(calibration_loc.c_str());
  cout << "Reading calibration info" << endl;
  calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;

  //create stereo camera calibration object
  const Cal3_S2 true_K(fx,fy,s,u0,v0);
  const Cal3_S2 noisy_K(fx*1.2,fy*1.2,s,u0-10,v0+10);

  initial_estimate.insert(K(0), noisy_K);

  auto calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 1e-5, 100, 100).finished());
  graph.addPrior(K(0), noisy_K, calNoise);


  ifstream pose_file(pose_loc.c_str());
  cout << "Reading camera poses" << endl;
  int pose_id;
  MatrixRowMajor m(4,4);
  //read camera pose parameters and use to make initial estimates of camera poses
  while (pose_file >> pose_id) {
    for (int i = 0; i < 16; i++) {
      pose_file >> m.data()[i];
    }
    initial_estimate.insert(Symbol('x', pose_id), Pose3(m));
  }

  auto poseNoise = noiseModel::Isotropic::Sigma(6, 0.01);
  graph.addPrior(Symbol('x', pose_id), Pose3(m), poseNoise);

  // camera and landmark keys
  size_t x, l;

  // pixel coordinates uL, uR, v (same for left/right images due to rectification)
  // landmark coordinates X, Y, Z in camera frame, resulting from triangulation
  double uL, uR, v, _X, Y, Z;
  ifstream factor_file(factor_loc.c_str());
  cout << "Reading stereo factors" << endl;
  //read stereo measurement details from file and use to create and add GenericStereoFactor objects to the graph representation
  while (factor_file >> x >> l >> uL >> uR >> v >> _X >> Y >> Z) {
//    graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(StereoPoint2(uL, uR, v), model, X(x), L(l), K);

    graph.emplace_shared<GeneralSFMFactor2<Cal3_S2> >(Point2(uL,v), model, X(x), L(l), K(0));


    //if the landmark variable included in this factor has not yet been added to the initial variable value estimate, add it
    if (!initial_estimate.exists(L(l))) {
      Pose3 camPose = initial_estimate.at<Pose3>(X(x));
      //transformFrom() transforms the input Point3 from the camera pose space, camPose, to the global space
      Point3 worldPoint = camPose.transformFrom(Point3(_X, Y, Z));
      initial_estimate.insert(L(l), worldPoint);
    }
  }

  Pose3 first_pose = initial_estimate.at<Pose3>(Symbol('x',1));
  //constrain the first pose such that it cannot change from its original value during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  graph.emplace_shared<NonlinearEquality<Pose3> >(Symbol('x',1),first_pose);

  cout << "Optimizing" << endl;
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;

  //create Levenberg-Marquardt optimizer to optimize the factor graph
  LevenbergMarquardtOptimizer optimizer(graph, initial_estimate,params);
//  Values result = optimizer.optimize();

  string K_values_file = "K_values.txt";
  ofstream stream_K(K_values_file.c_str());

  double currentError;


  stream_K << optimizer.iterations() << " " << optimizer.values().at<Cal3_S2>(K(0)).vector().transpose() << endl;


  // Iterative loop
  do {
    // Do next iteration
    currentError = optimizer.error();
    optimizer.iterate();

    stream_K << optimizer.iterations() << " " << optimizer.values().at<Cal3_S2>(K(0)).vector().transpose() << endl;

    if(params.verbosity >= NonlinearOptimizerParams::ERROR) cout << "newError: " << optimizer.error() << endl;
  } while(optimizer.iterations() < params.maxIterations &&
      !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
            params.errorTol, currentError, optimizer.error(), params.verbosity));

  Values result = optimizer.values();

  cout << "Final result sample:" << endl;
  Values pose_values = utilities::allPose3s(result);
  pose_values.print("Final camera poses:\n");

  result.at<Cal3_S2>(K(0)).print("Final K\n");

  noisy_K.print("Initial noisy K\n");
  true_K.print("Initial correct K\n");

  return 0;
}
