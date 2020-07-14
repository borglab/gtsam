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
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <fstream>

using namespace std;
using namespace gtsam;

// HOWTO: ./Pose2SLAMExample_g2o inputFile outputFile (maxIterations) (tukey/huber)
int main(const int argc, const char *argv[]) {
  string kernelType = "none";
  int maxIterations = 100;                                    // default
  string g2oFile = findExampleDataFile("noisyToyGraph.txt");  // default

  // Parse user's inputs
  if (argc > 1) {
    g2oFile = argv[1];  // input dataset filename
  }
  if (argc > 3) {
    maxIterations = atoi(argv[3]);  // user can specify either tukey or huber
  }
  if (argc > 4) {
    kernelType = argv[4];  // user can specify either tukey or huber
  }

  // reading file and creating factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  if (kernelType.compare("none") == 0) {
    boost::tie(graph, initial) = readG2o(g2oFile, is3D);
  }
  if (kernelType.compare("huber") == 0) {
    std::cout << "Using robust kernel: huber " << std::endl;
    boost::tie(graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeHUBER);
  }
  if (kernelType.compare("tukey") == 0) {
    std::cout << "Using robust kernel: tukey " << std::endl;
    boost::tie(graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeTUKEY);
  }

  // Add prior on the pose having index (key) = 0
  auto priorModel =  //
      noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
  graph->addPrior(0, Pose2(), priorModel);
  std::cout << "Adding prior on pose 0 " << std::endl;

  GaussNewtonParams params;
  params.setVerbosity("TERMINATION");
  if (argc > 3) {
    params.maxIterations = maxIterations;
    std::cout << "User required to perform maximum  " << params.maxIterations
              << " iterations " << std::endl;
  }

  std::cout << "Optimizing the factor graph" << std::endl;
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
  return 0;
}
