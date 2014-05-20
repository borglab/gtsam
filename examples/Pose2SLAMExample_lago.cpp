/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_lago.cpp
 * @brief A 2D Pose SLAM example that reads input from g2o, and solve the Pose2 problem
 * using LAGO (Linear Approximation for Graph Optimization). See class LagoInitializer.h
 * Output is written on a file, in g2o format
 * Syntax for the script is ./Pose2SLAMExample_lago input.g2o output.g2o
 * @date May 15, 2014
 * @author Luca Carlone
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LagoInitializer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <sstream>

using namespace std;
using namespace gtsam;


int main(const int argc, const char *argv[]){

  if (argc < 2)
    std::cout << "Please specify: 1st argument: input file (in g2o format) and 2nd argument: output file" << std::endl;
  const string g2oFile = argv[1];

  NonlinearFactorGraph graph;
  Values initial;
  readG2o(g2oFile, graph, initial);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = graph;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(3) << 1e-6, 1e-6, 1e-8));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  std::cout << "Computing LAGO estimate" << std::endl;
  Values estimateLago = initializeLago(graphWithPrior);
  std::cout << "done!" << std::endl;

  const string outputFile = argv[2];
  std::cout << "Writing results to file: " << outputFile << std::endl;
  writeG2o(outputFile, graph, estimateLago);
  std::cout << "done! " << std::endl;

  return 0;
}
