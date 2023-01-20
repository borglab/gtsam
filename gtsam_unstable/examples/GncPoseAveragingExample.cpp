/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GncPoseAveragingExample.cpp
 * @brief example of GNC estimating a single pose from pose priors possibly corrupted with outliers
 * You can run this example using: ./GncPoseAveragingExample nrInliers nrOutliers
 * e.g.,: ./GncPoseAveragingExample 10 5  (if the numbers are not specified, default
 * values nrInliers = 10 and nrOutliers = 10 are used)
 * @date May 8, 2021
 * @author Luca Carlone
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GncOptimizer.h>

#include <string>
#include <fstream>
#include <iostream>
#include <random>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv){
  cout << "== Robust Pose Averaging Example === " << endl;

  // default number of inliers and outliers
  size_t nrInliers = 10;
  size_t nrOutliers = 10;

  // User can pass arbitrary number of inliers and outliers for testing
  if (argc > 1)
    nrInliers = atoi(argv[1]);
  if (argc > 2)
    nrOutliers = atoi(argv[2]);
  cout << "nrInliers " << nrInliers << " nrOutliers "<< nrOutliers << endl;

  // Seed random number generator
  random_device rd;
  mt19937 rng(rd());
  uniform_real_distribution<double> uniform(-10, 10);
  normal_distribution<double> normalInliers(0.0, 0.05);

  Values initial;
  initial.insert(0, Pose3()); // identity pose as initialization

  // create ground truth pose
  Vector6 poseGtVector;
  for(size_t i = 0; i < 6; ++i){
    poseGtVector(i) = uniform(rng);
  }
  Pose3 gtPose = Pose3::Expmap(poseGtVector); // Pose3( Rot3::Ypr(3.0, 1.5, 0.8), Point3(4,1,3) );

  NonlinearFactorGraph graph;
  const noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(6,0.05);
  // create inliers
  for(size_t i=0; i<nrInliers; i++){
    Vector6 poseNoise;
    for(size_t i = 0; i < 6; ++i){
      poseNoise(i) = normalInliers(rng);
    }
    Pose3 poseMeasurement = gtPose.retract(poseNoise);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,poseMeasurement,model));
  }

  // create outliers
  for(size_t i=0; i<nrOutliers; i++){
    Vector6 poseNoise;
    for(size_t i = 0; i < 6; ++i){
      poseNoise(i) = uniform(rng);
    }
    Pose3 poseMeasurement = gtPose.retract(poseNoise);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,poseMeasurement,model));
  }

  GncParams<LevenbergMarquardtParams> gncParams;
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(graph,
      initial,
      gncParams);

  Values estimate = gnc.optimize();
  Pose3 poseError = gtPose.between( estimate.at<Pose3>(0) );
  cout << "norm of translation error: " << poseError.translation().norm() <<
      " norm of rotation error: " << poseError.rotation().rpy().norm() << endl;
  // poseError.print("pose error: \n ");
  return 0;
}
