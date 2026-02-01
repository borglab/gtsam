/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GNCExample.cpp
 * @brief Simple example showcasing a Graduated Non-Convexity based solver
 * @author Achintya Mohan
 */

/**
 * A simple 2D pose graph optimization example
 * - The robot is initially at origin (0.0, 0.0, 0.0) 
 * - We have full odometry measurements for 2 motions
 * - The robot first moves to (1.0, 0.0, 0.1) and then to (1.0, 1.0, 0.2) 
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;

int main() {
  cout << "Graduated Non-Convexity Example\n";

  NonlinearFactorGraph graph;

  // Add a prior to the first point, set to the origin
  auto priorNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior(1, Pose2(0.0, 0.0, 0.0), priorNoise);

  // Add additional factors, noise models must be Gaussian 
  Pose2 x1(1.0, 0.0, 0.1);
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, x1, noiseModel::Isotropic::Sigma(3, 0.2));
  Pose2 x2(0.0, 1.0, 0.1);
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, x2, noiseModel::Isotropic::Sigma(3, 0.4));

  // Initial estimates
  Values initial;
  initial.insert(1, Pose2(0.2, 0.5, -0.1));
  initial.insert(2, Pose2(0.8, 0.3, 0.1));
  initial.insert(3, Pose2(0.8, 0.2, 0.3));

  // Set options for the non-minimal solver
  LevenbergMarquardtParams lmParams;
  lmParams.setMaxIterations(1000);
  lmParams.setRelativeErrorTol(1e-5);

  // Set GNC-specific options
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  gncParams.setLossType(GncLossType::TLS);

  // Optimize the graph and print results
  GncOptimizer<GncParams<LevenbergMarquardtParams>> optimizer(graph, initial, gncParams);
  Values result = optimizer.optimize();
  result.print("Final Result:");

  return 0;
}

