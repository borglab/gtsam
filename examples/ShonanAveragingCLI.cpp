/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information
* -------------------------------------------------------------------------- */

/**
 * @file    ShonanAveragingCLI.cpp
 * @brief   Run Shonan Rotation Averaging Algorithm on a file or example dataset
 * @author  Frank Dellaert
 * @date    August, 2020
 *
 * Example usage:
 *
 * Running without arguments will run on tiny 3D example pose3example-grid
 * ./ShonanAveragingCLI
 *
 * Read 2D dataset w10000 (in examples/data) and output to w10000-rotations.g2o
 * ./ShonanAveragingCLI -d 2 -n w10000 -o w10000-rotations.g2o
 *
 * Read 3D dataset sphere25000.txt and output to shonan.g2o (default)
 * ./ShonanAveragingCLI -i spere2500.txt
 *
 * If you prefer using a robust Huber loss, you can add the option "-h true",
 * for instance
 * ./ShonanAveragingCLI -i spere2500.txt -h true
 */

#include <gtsam/base/timing.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/slam/dataset.h>

#include <boost/program_options.hpp>

using namespace std;
using namespace gtsam;
namespace po = boost::program_options;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  string datasetName;
  string inputFile;
  string outputFile;
  int d, seed, pMin;
  bool useHuberLoss;
  po::options_description desc(
      "Shonan Rotation Averaging CLI reads a *pose* graph, extracts the "
      "rotation constraints, and runs the Shonan algorithm.");
  desc.add_options()("help", "Print help message")(
      "named_dataset,n",
      po::value<string>(&datasetName)->default_value("pose3example-grid"),
      "Find and read frome example dataset file")(
      "input_file,i", po::value<string>(&inputFile)->default_value(""),
      "Read pose constraints graph from the specified file")(
      "output_file,o",
      po::value<string>(&outputFile)->default_value("shonan.g2o"),
      "Write solution to the specified file")(
      "dimension,d", po::value<int>(&d)->default_value(3),
      "Optimize over 2D or 3D rotations")(
      "useHuberLoss,h", po::value<bool>(&useHuberLoss)->default_value(false),
      "set True to use Huber loss")("pMin,p",
                                    po::value<int>(&pMin)->default_value(3),
                                    "set to use desired rank pMin")(
      "seed,s", po::value<int>(&seed)->default_value(42),
      "Random seed for initial estimate");
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  // Get input file
  if (inputFile.empty()) {
    if (datasetName.empty()) {
      cout << "You must either specify a named dataset or an input file\n"
           << desc << endl;
      return 1;
    }
    inputFile = findExampleDataFile(datasetName);
  }

  // Seed random number generator
  static std::mt19937 rng(seed);

  NonlinearFactorGraph::shared_ptr inputGraph;
  Values::shared_ptr posesInFile;
  Values poses;
  auto lmParams = LevenbergMarquardtParams::CeresDefaults();
  if (d == 2) {
    cout << "Running Shonan averaging for SO(2) on " << inputFile << endl;
    ShonanAveraging2::Parameters parameters(lmParams);
    parameters.setUseHuber(useHuberLoss);
    ShonanAveraging2 shonan(inputFile, parameters);
    auto initial = shonan.initializeRandomly(rng);
    auto result = shonan.run(initial, pMin);

    // Parse file again to set up translation problem, adding a prior
    boost::tie(inputGraph, posesInFile) = load2D(inputFile);
    auto priorModel = noiseModel::Unit::Create(3);
    inputGraph->addPrior(0, posesInFile->at<Pose2>(0), priorModel);

    cout << "recovering 2D translations" << endl;
    auto poseGraph = initialize::buildPoseGraph<Pose2>(*inputGraph);
    poses = initialize::computePoses<Pose2>(result.first, &poseGraph);
  } else if (d == 3) {
    cout << "Running Shonan averaging for SO(3) on " << inputFile << endl;
    ShonanAveraging3::Parameters parameters(lmParams);
    parameters.setUseHuber(useHuberLoss);
    ShonanAveraging3 shonan(inputFile, parameters);
    auto initial = shonan.initializeRandomly(rng);
    auto result = shonan.run(initial, pMin);

    // Parse file again to set up translation problem, adding a prior
    boost::tie(inputGraph, posesInFile) = load3D(inputFile);
    auto priorModel = noiseModel::Unit::Create(6);
    inputGraph->addPrior(0, posesInFile->at<Pose3>(0), priorModel);

    cout << "recovering 3D translations" << endl;
    auto poseGraph = initialize::buildPoseGraph<Pose3>(*inputGraph);
    poses = initialize::computePoses<Pose3>(result.first, &poseGraph);
  } else {
    cout << "Can only run SO(2) or SO(3) averaging\n" << desc << endl;
    return 1;
  }
  cout << "Writing result to " << outputFile << endl;
  writeG2o(*inputGraph, poses, outputFile);
  return 0;
}

/* ************************************************************************* */
