/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3Localization.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3Localization input.g2o output.g2o
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(const int argc, const char* argv[]) {
  // Read graph from file
  string g2oFile;
  if (argc < 2)
    g2oFile = findExampleDataFile("pose3Localizationexample.txt");
  else
    g2oFile = argv[1];

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = true;
  boost::tie(graph, initial) = readG2o(g2oFile, is3D);

  // Add prior on the first key
  auto priorModel = noiseModel::Diagonal::Variances(
      (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
  Key firstKey = 0;
  for (const Values::ConstKeyValuePair& key_value : *initial) {
    std::cout << "Adding prior to g2o file " << std::endl;
    firstKey = key_value.key;
    graph->addPrior(firstKey, Pose3(), priorModel);
    break;
  }

  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonParams params;
  params.setVerbosity("TERMINATION");  // show info about stopping conditions
  GaussNewtonOptimizer optimizer(*graph, *initial, params);
  Values result = optimizer.optimize();
  std::cout << "Optimization complete" << std::endl;

  std::cout << "initial error=" << graph->error(*initial) << std::endl;
  std::cout << "final error=" << graph->error(result) << std::endl;

  if (argc < 3) {
    result.print("result");
  } else {
    const string outputFile = argv[2];
    std::cout << "Writing results to file: " << outputFile << std::endl;
    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2oFile);
    writeG2o(*graphNoKernel, result, outputFile);
    std::cout << "done! " << std::endl;
  }

  // Calculate and print marginal covariances for all variables
  Marginals marginals(*graph, result);
  for (const auto& key_value : result) {
    auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
    if (!p) continue;
    std::cout << marginals.marginalCovariance(key_value.key) << endl;
  }
  return 0;
}
