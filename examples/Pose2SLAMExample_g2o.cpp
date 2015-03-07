/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_g2o.cpp
 * @brief A 2D Pose SLAM example that reads input from g2o, converts it to a factor graph and does the
 * optimization. Output is written on a file, in g2o format
 * Syntax for the script is ./Pose2SLAMExample_g2o input.g2o output.g2o
 * @date May 15, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(const int argc, const char *argv[]) {

  // Read graph from file
  string g2oFile;
  if (argc < 2)
    g2oFile = "../../examples/Data/noisyToyGraph.txt";
  else
    g2oFile = argv[1];

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  boost::tie(graph, initial) = readG2o(g2oFile);

  // Add prior on the pose having index (key) = 0
  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances((Vector(3) << 1e-6, 1e-6, 1e-8));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonOptimizer optimizer(graphWithPrior, *initial);
  Values result = optimizer.optimize();
  std::cout << "Optimization complete" << std::endl;

  if (argc < 3) {
    result.print("result");
  } else {
    const string outputFile = argv[2];
    std::cout << "Writing results to file: " << outputFile << std::endl;
    writeG2o(*graph, result, outputFile);
    std::cout << "done! " << std::endl;
  }
  return 0;
}
