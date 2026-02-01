/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ISAM2_City10000.cpp
 * @brief  Example of using ISAM2 estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <fstream>
#include <string>
#include <vector>

#include "City10000.h"

using namespace gtsam;

using symbol_shorthand::X;

// Experiment Class
class Experiment {
  /// The City10000 dataset
  City10000Dataset dataset_;

 public:
  // Parameters with default values
  size_t maxLoopCount = 2000;  // 200 //2000 //8000

  // false: run original iSAM2 without ambiguities
  // true: run original iSAM2 with ambiguities
  bool isWithAmbiguity;

 private:
  ISAM2 isam2_;
  NonlinearFactorGraph graph_;
  Values initial_;
  Values results;

 public:
  /// Construct with filename of experiment to run
  explicit Experiment(const std::string& filename, bool isWithAmbiguity = false)
      : dataset_(filename), isWithAmbiguity(isWithAmbiguity) {
    ISAM2Params parameters;
    parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0);
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = ISAM2(parameters);
  }

  /// @brief Run the main experiment with a given maxLoopCount.
  void run() {
    // Initialize local variables
    size_t index = 0;

    std::vector<std::pair<size_t, double>> smootherUpdateTimes;

    std::list<double> timeList;

    // Set up initial prior
    Pose2 priorPose(0, 0, 0);
    initial_.insert(X(0), priorPose);
    graph_.addPrior<Pose2>(X(0), priorPose, kPriorNoiseModel);

    // Initial update
    clock_t beforeUpdate = clock();
    isam2_.update(graph_, initial_);
    results = isam2_.calculateBestEstimate();
    clock_t afterUpdate = clock();
    smootherUpdateTimes.push_back(
        std::make_pair(index, afterUpdate - beforeUpdate));
    graph_.resize(0);
    initial_.clear();
    index += 1;

    // Start main loop
    size_t keyS = 0;
    size_t keyT = 0;
    clock_t startTime = clock();

    std::vector<Pose2> poseArray;
    std::pair<size_t, size_t> keys;

    while (dataset_.next(&poseArray, &keys) && index < maxLoopCount) {
      keyS = keys.first;
      keyT = keys.second;
      size_t numMeasurements = poseArray.size();

      Pose2 odomPose;
      if (isWithAmbiguity) {
        // Get wrong intentionally
        int id = index % numMeasurements;
        odomPose = Pose2(poseArray[id]);
      } else {
        odomPose = poseArray[0];
      }

      if (keyS == keyT - 1) {  // new X(key)
        initial_.insert(X(keyT), results.at<Pose2>(X(keyS)) * odomPose);
        graph_.add(
            BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose, kPoseNoiseModel));

      } else {  // loop
        int id = index % numMeasurements;
        if (isWithAmbiguity && id % 2 == 0) {
          graph_.add(BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose,
                                          kPoseNoiseModel));
        } else {
          graph_.add(BetweenFactor<Pose2>(
              X(keyS), X(keyT), odomPose,
              noiseModel::Diagonal::Sigmas(Vector3::Ones() * 10.0)));
        }
        index++;
      }

      clock_t beforeUpdate = clock();
      isam2_.update(graph_, initial_);
      results = isam2_.calculateBestEstimate();
      clock_t afterUpdate = clock();
      smootherUpdateTimes.push_back(
          std::make_pair(index, afterUpdate - beforeUpdate));
      graph_.resize(0);
      initial_.clear();
      index += 1;

      // Print loop index and time taken in processor clock ticks
      if (index % 50 == 0 && keyS != keyT - 1) {
        std::cout << "index: " << index << std::endl;
        std::cout << "accTime:  " << timeList.back() / CLOCKS_PER_SEC
                  << std::endl;
      }

      if (keyS == keyT - 1) {
        clock_t curTime = clock();
        timeList.push_back(curTime - startTime);
      }

      if (timeList.size() % 100 == 0 && (keyS == keyT - 1)) {
        std::string stepFileIdx = std::to_string(100000 + timeList.size());

        std::ofstream stepOutfile;
        std::string stepFileName = "step_files/ISAM2_City10000_S" + stepFileIdx;
        stepOutfile.open(stepFileName + ".txt");
        for (size_t i = 0; i < (keyT + 1); ++i) {
          Pose2 outPose = results.at<Pose2>(X(i));
          stepOutfile << outPose.x() << " " << outPose.y() << " "
                      << outPose.theta() << std::endl;
        }
        stepOutfile.close();
      }
    }

    clock_t endTime = clock();
    clock_t totalTime = endTime - startTime;
    std::cout << "totalTime: " << totalTime / CLOCKS_PER_SEC << std::endl;

    /// Write results to file
    writeResult(results, (keyT + 1), "ISAM2_City10000.txt");

    std::ofstream outfileTime;
    std::string timeFileName = "ISAM2_City10000_time.txt";
    outfileTime.open(timeFileName);
    for (auto accTime : timeList) {
      outfileTime << accTime << std::endl;
    }
    outfileTime.close();
    std::cout << "Written cumulative time to: " << timeFileName << " file."
              << std::endl;

    std::ofstream timingFile;
    std::string timingFileName = "ISAM2_City10000_timing.txt";
    timingFile.open(timingFileName);
    for (size_t i = 0; i < smootherUpdateTimes.size(); i++) {
      auto p = smootherUpdateTimes.at(i);
      timingFile << p.first << ", " << p.second / CLOCKS_PER_SEC << std::endl;
    }
    timingFile.close();
    std::cout << "Wrote timing information to " << timingFileName << std::endl;
  }
};

/* ************************************************************************* */
// Function to parse command-line arguments
void parseArguments(int argc, char* argv[], size_t& maxLoopCount,
                    bool& isWithAmbiguity) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--max-loop-count" && i + 1 < argc) {
      maxLoopCount = std::stoul(argv[++i]);
    } else if (arg == "--is-with-ambiguity" && i + 1 < argc) {
      isWithAmbiguity = bool(std::stoul(argv[++i]));
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  --max-loop-count <value>       Set the maximum loop "
                   "count (default: 2000)\n"
                << "  --is-with-ambiguity <value=0/1>     Set whether to use "
                   "ambiguous measurements "
                   "(default: false)\n"
                << "  --help                         Show this help message\n";
      std::exit(0);
    }
  }
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  Experiment experiment(findExampleDataFile("T1_City10000_04.txt"));
  // Experiment experiment("../data/mh_T1_City10000_04.txt"); //Type #1 only
  // Experiment experiment("../data/mh_T3b_City10000_10.txt"); //Type #3 only
  // Experiment experiment("../data/mh_T1_T3_City10000_04.txt"); //Type #1 +
  // Type #3

  // Parse command-line arguments
  parseArguments(argc, argv, experiment.maxLoopCount,
                 experiment.isWithAmbiguity);

  // Run the experiment
  experiment.run();

  return 0;
}
