/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_graphviz.cpp
 * @brief Save factor graph as graphviz dot file
 * @date Sept 6, 2013
 * @author Frank Dellaert
 */

// For an explanation of headers below, please see Pose2SLAMExample.cpp
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, Pose2(0, 0, 0), priorNoise);

  // For simplicity, we will use the same noise model for odometry and loop
  // closures
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), model);

  // 2c. Add the loop closure constraint
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), model);

  // 3. Create the data structure to hold the initial estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initial.insert(4, Pose2(4.0, 2.0, M_PI));
  initial.insert(5, Pose2(2.1, 2.1, -M_PI_2));

  // Single Step Optimization using Levenberg-Marquardt
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  // save factor graph as graphviz dot file
  // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
  ofstream os("Pose2SLAMExample.dot");
  graph.saveGraph(os, result);

  // Also print out to console
  graph.saveGraph(cout, result);

  return 0;
}
