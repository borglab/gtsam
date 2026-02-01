/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Hybrid_City10000.cpp
 * @brief  Example of using hybrid estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "City10000.h"

using namespace gtsam;

using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

// Experiment Class
class Experiment {
  /// The City10000 dataset
  City10000Dataset dataset_;

 public:
  // Parameters with default values
  size_t maxLoopCount = 8000;

  // 3000: {1: 62s, 2: 21s, 3: 20s, 4: 31s, 5: 39s} No DT optimizations
  // 3000: {1: 65s, 2: 20s, 3: 16s, 4: 21s, 5: 28s} With DT optimizations
  // 3000: {1: 59s, 2: 19s, 3: 18s, 4: 26s, 5: 33s} With DT optimizations +
  // merge
  size_t updateFrequency = 3;

  size_t maxNrHypotheses = 10;

  size_t reLinearizationFrequency = 10;

  double marginalThreshold = 0.9999;

 private:
  HybridSmoother smoother_;
  HybridNonlinearFactorGraph newFactors_, allFactors_;
  Values initial_;

  /**
   * @brief Create a hybrid loop closure factor where
   * 0 - loose noise model and 1 - loop noise model.
   */
  HybridNonlinearFactor hybridLoopClosureFactor(
      size_t loopCounter, size_t keyS, size_t keyT,
      const Pose2& measurement) const {
    DiscreteKey l(L(loopCounter), 2);

    auto f0 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), measurement, kOpenLoopModel);
    auto f1 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), measurement, kPoseNoiseModel);

    std::vector<NonlinearFactorValuePair> factors{{f0, kOpenLoopConstant},
                                                  {f1, kPoseNoiseConstant}};
    HybridNonlinearFactor mixtureFactor(l, factors);
    return mixtureFactor;
  }

  /// @brief Create hybrid odometry factor with discrete measurement choices.
  HybridNonlinearFactor hybridOdometryFactor(
      size_t numMeasurements, size_t keyS, size_t keyT, const DiscreteKey& m,
      const std::vector<Pose2>& poseArray) const {
    auto f0 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), poseArray[0], kPoseNoiseModel);
    auto f1 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), poseArray[1], kPoseNoiseModel);

    std::vector<NonlinearFactorValuePair> factors{{f0, kPoseNoiseConstant},
                                                  {f1, kPoseNoiseConstant}};
    HybridNonlinearFactor mixtureFactor(m, factors);
    return mixtureFactor;
  }

  /// @brief Perform smoother update and optimize the graph.
  clock_t smootherUpdate(size_t maxNrHypotheses) {
    std::cout << "Smoother update: " << newFactors_.size() << std::endl;
    gttic_(SmootherUpdate);
    clock_t beforeUpdate = clock();
    smoother_.update(newFactors_, initial_, maxNrHypotheses);
    clock_t afterUpdate = clock();
    allFactors_.push_back(newFactors_);
    newFactors_.resize(0);
    return afterUpdate - beforeUpdate;
  }

  /// @brief Re-linearize, solve ALL, and re-initialize smoother.
  clock_t reInitialize() {
    std::cout << "================= Re-Initialize: " << allFactors_.size()
              << std::endl;
    clock_t beforeUpdate = clock();
    allFactors_ = allFactors_.restrict(smoother_.fixedValues());
    auto linearized = allFactors_.linearize(initial_);
    auto bayesNet = linearized->eliminateSequential();
    HybridValues delta = bayesNet->optimize();
    initial_ = initial_.retract(delta.continuous());
    smoother_.reInitialize(std::move(*bayesNet));
    clock_t afterUpdate = clock();
    std::cout << "Took " << (afterUpdate - beforeUpdate) / CLOCKS_PER_SEC
              << " seconds." << std::endl;
    return afterUpdate - beforeUpdate;
  }

 public:
  /// Construct with filename of experiment to run
  explicit Experiment(const std::string& filename)
      : dataset_(filename), smoother_(marginalThreshold) {}

  /// @brief Run the main experiment with a given maxLoopCount.
  void run() {
    // Initialize local variables
    size_t discreteCount = 0, index = 0, loopCount = 0, updateCount = 0;

    std::list<double> timeList;

    // Set up initial prior
    Pose2 priorPose(0, 0, 0);
    initial_.insert(X(0), priorPose);
    newFactors_.push_back(
        PriorFactor<Pose2>(X(0), priorPose, kPriorNoiseModel));

    // Initial update
    auto time = smootherUpdate(maxNrHypotheses);
    std::vector<std::pair<size_t, double>> smootherUpdateTimes;
    smootherUpdateTimes.push_back({index, time});

    // Flag to decide whether to run smoother update
    size_t numberOfHybridFactors = 0;

    // Start main loop
    Values result;
    size_t keyS = 0, keyT = 0;
    clock_t startTime = clock();

    std::vector<Pose2> poseArray;
    std::pair<size_t, size_t> keys;

    while (dataset_.next(&poseArray, &keys) && index < maxLoopCount) {
      keyS = keys.first;
      keyT = keys.second;
      size_t numMeasurements = poseArray.size();

      // Take the first one as the initial estimate
      Pose2 odomPose = poseArray[0];
      if (keyS == keyT - 1) {
        // Odometry factor
        if (numMeasurements > 1) {
          // Add hybrid factor
          DiscreteKey m(M(discreteCount), numMeasurements);
          HybridNonlinearFactor mixtureFactor =
              hybridOdometryFactor(numMeasurements, keyS, keyT, m, poseArray);
          newFactors_.push_back(mixtureFactor);
          discreteCount++;
          numberOfHybridFactors += 1;
          std::cout << "mixtureFactor: " << keyS << " " << keyT << std::endl;
        } else {
          newFactors_.add(BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose,
                                               kPoseNoiseModel));
        }
        // Insert next pose initial guess
        initial_.insert(X(keyT), initial_.at<Pose2>(X(keyS)) * odomPose);
      } else {
        // Loop closure
        HybridNonlinearFactor loopFactor =
            hybridLoopClosureFactor(loopCount, keyS, keyT, odomPose);
        // print loop closure event keys:
        std::cout << "Loop closure: " << keyS << " " << keyT << std::endl;
        newFactors_.add(loopFactor);
        numberOfHybridFactors += 1;
        loopCount++;
      }

      if (numberOfHybridFactors >= updateFrequency) {
        auto time = smootherUpdate(maxNrHypotheses);
        smootherUpdateTimes.push_back({index, time});
        numberOfHybridFactors = 0;
        updateCount++;

        if (updateCount % reLinearizationFrequency == 0) {
          reInitialize();
        }
      }

      // Record timing for odometry edges only
      if (keyS == keyT - 1) {
        clock_t curTime = clock();
        timeList.push_back(curTime - startTime);
      }

      // Print some status every 100 steps
      if (index % 100 == 0) {
        std::cout << "Index: " << index << std::endl;
        if (!timeList.empty()) {
          std::cout << "Acc_time: " << timeList.back() / CLOCKS_PER_SEC
                    << " seconds" << std::endl;
          // delta.discrete().print("The Discrete Assignment");
          tictoc_finishedIteration_();
          tictoc_print_();
        }
      }

      index++;
    }

    // Final update
    time = smootherUpdate(maxNrHypotheses);
    smootherUpdateTimes.push_back({index, time});

    // Final optimize
    gttic_(HybridSmootherOptimize);
    HybridValues delta = smoother_.optimize();
    gttoc_(HybridSmootherOptimize);

    result.insert_or_assign(initial_.retract(delta.continuous()));

    std::cout << "Final error: " << smoother_.hybridBayesNet().error(delta)
              << std::endl;

    clock_t endTime = clock();
    clock_t totalTime = endTime - startTime;
    std::cout << "Total time: " << totalTime / CLOCKS_PER_SEC << " seconds"
              << std::endl;

    // Write results to file
    writeResult(result, keyT + 1, "Hybrid_City10000.txt");

    // Write timing info to file
    std::ofstream outfileTime;
    std::string timeFileName = "Hybrid_City10000_time.txt";
    outfileTime.open(timeFileName);
    for (auto accTime : timeList) {
      outfileTime << accTime / CLOCKS_PER_SEC << std::endl;
    }
    outfileTime.close();
    std::cout << "Output " << timeFileName << " file." << std::endl;

    std::ofstream timingFile;
    std::string timingFileName = "Hybrid_City10000_timing.txt";
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
                    size_t& updateFrequency, size_t& maxNrHypotheses) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--max-loop-count" && i + 1 < argc) {
      maxLoopCount = std::stoul(argv[++i]);
    } else if (arg == "--update-frequency" && i + 1 < argc) {
      updateFrequency = std::stoul(argv[++i]);
    } else if (arg == "--max-nr-hypotheses" && i + 1 < argc) {
      maxNrHypotheses = std::stoul(argv[++i]);
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  --max-loop-count <value>       Set the maximum loop "
                   "count (default: 3000)\n"
                << "  --update-frequency <value>     Set the update frequency "
                   "(default: 3)\n"
                << "  --max-nr-hypotheses <value>    Set the maximum number of "
                   "hypotheses (default: 10)\n"
                << "  --help                         Show this help message\n";
      std::exit(0);
    }
  }
}

/* ************************************************************************* */
// Main function
int main(int argc, char* argv[]) {
  Experiment experiment(findExampleDataFile("T1_city10000_04.txt"));
  // Experiment experiment("../data/mh_T1_city10000_04.txt"); //Type #1 only
  // Experiment experiment("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  // Experiment experiment("../data/mh_T1_T3_city10000_04.txt"); //Type #1 +
  // Type #3

  // Parse command-line arguments
  parseArguments(argc, argv, experiment.maxLoopCount,
                 experiment.updateFrequency, experiment.maxNrHypotheses);

  // Run the experiment
  experiment.run();

  return 0;
}
