/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3SLAMExample_initializePose3Chordal.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3SLAMExample_initializePose3Chordal input.g2o output.g2o
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(const int argc, const char* argv[]) {
  // Read graph from file
  string g2oFile;
  if (argc < 2)
    g2oFile = findExampleDataFile("pose3example.txt");
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

  std::cout << "Initializing Pose3 - chordal relaxation" << std::endl;
  Values initialization = InitializePose3::initialize(*graph);
  std::cout << "done!" << std::endl;

  if (argc < 3) {
    initialization.print("initialization");
  } else {
    const string outputFile = argv[2];
    std::cout << "Writing results to file: " << outputFile << std::endl;
    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    boost::tie(graphNoKernel, initial2) = readG2o(g2oFile);
    writeG2o(*graphNoKernel, initialization, outputFile);
    std::cout << "done! " << std::endl;
  }
  return 0;
}
