/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMwSPCG.cpp
 * @brief A 2D Pose SLAM example using the SimpleSPCGSolver.
 * @author Yong-Dian Jian
 * @date June 2, 2012
 */

// For an explanation of headers below, please see Pose2SLAMExample.cpp
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// In contrast to that example, however, we will use a PCG solver here
#include <gtsam/linear/SubgraphSolver.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  Pose2 prior(0.0, 0.0, 0.0);  // prior at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, prior, priorNoise);

  // 2b. Add odometry factors
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2.0, 0.0, M_PI_2), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2.0, 0.0, M_PI_2), odometryNoise);

  // 2c. Add the loop closure constraint
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 1, Pose2(0.0, 0.0, 0.0),
                                              model);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the
  // solution
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, 1.1));
  initialEstimate.insert(3, Pose2(2.1, 1.9, 2.8));
  initialEstimate.insert(4, Pose2(-.3, 2.5, 4.2));
  initialEstimate.insert(5, Pose2(0.1, -0.7, 5.8));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Single Step Optimization using Levenberg-Marquardt
  LevenbergMarquardtParams parameters;
  parameters.verbosity = NonlinearOptimizerParams::ERROR;
  parameters.verbosityLM = LevenbergMarquardtParams::LAMBDA;

  // LM is still the outer optimization loop, but by specifying "Iterative"
  // below We indicate that an iterative linear solver should be used. In
  // addition, the *type* of the iterativeParams decides on the type of
  // iterative solver, in this case the SPCG (subgraph PCG)
  parameters.linearSolverType = NonlinearOptimizerParams::Iterative;
  parameters.iterativeParams = boost::make_shared<SubgraphSolverParameters>();

  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");
  cout << "subgraph solver final error = " << graph.error(result) << endl;

  return 0;
}
