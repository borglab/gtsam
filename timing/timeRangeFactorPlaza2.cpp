/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeRangeFactorPlaza2.cpp
 * @brief   Benchmark the range-only Plaza2 incremental SLAM workload.
 * @date    April 2026
 * @author  Codex 5.4, prompted by Frank Dellaert
 */

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "internal/TimingUtils.h"

using gtsam::Point2;
using gtsam::Pose2;
using gtsam::Symbol;
using gtsam::Vector;
using gtsam::Vector3;
using std::cout;
using std::endl;
using std::optional;
using std::pair;
using std::set;
using std::size_t;
using std::string;
using std::tuple;
using std::vector;

namespace NM = gtsam::noiseModel;

namespace {

using TimedOdometry = pair<double, Pose2>;
using RangeTriple = tuple<double, size_t, double>;

struct Dataset {
  vector<TimedOdometry> odometry;
  vector<RangeTriple> triples;
};

struct BenchmarkOptions {
  size_t warmupRuns = 1;
  size_t measuredRuns = 5;
  size_t minRanges = 150;
  size_t incrementRanges = 25;
  bool robust = true;
  uint64_t seed = 42;
  optional<string> outputPath;
};

struct RunResult {
  size_t runIndex = 0;
  double solveSeconds = 0.0;
  double totalSeconds = 0.0;
  double batchInitializationSeconds = 0.0;
  double updateSeconds = 0.0;
  double calculateEstimateSeconds = 0.0;
  double finalEstimateSeconds = 0.0;
  size_t odometryEntries = 0;
  size_t rangeTriples = 0;
  size_t rangeFactorsAdded = 0;
  size_t updateCount = 0;
  size_t initializedLandmarks = 0;
  size_t finalVariableCount = 0;
};

vector<TimedOdometry> readOdometry() {
  vector<TimedOdometry> odometry;
  const string dataFile = gtsam::findExampleDataFile("Plaza2_DR.txt");
  std::ifstream input(dataFile.c_str());
  if (!input) {
    throw std::runtime_error("Plaza2_DR.txt file not found");
  }

  while (input) {
    double time = 0.0;
    double distanceTraveled = 0.0;
    double deltaHeading = 0.0;
    input >> time >> distanceTraveled >> deltaHeading;
    if (!input) {
      break;
    }
    odometry.emplace_back(time, Pose2(distanceTraveled, 0.0, deltaHeading));
  }
  return odometry;
}

vector<RangeTriple> readTriples() {
  vector<RangeTriple> triples;
  const string dataFile = gtsam::findExampleDataFile("Plaza2_TD.txt");
  std::ifstream input(dataFile.c_str());
  if (!input) {
    throw std::runtime_error("Plaza2_TD.txt file not found");
  }

  while (input) {
    double time = 0.0;
    double sender = 0.0;
    double receiver = 0.0;
    double range = 0.0;
    input >> time >> sender >> receiver >> range;
    if (!input) {
      break;
    }
    triples.emplace_back(time, static_cast<size_t>(receiver), range);
  }
  return triples;
}

Dataset loadDataset() { return Dataset{readOdometry(), readTriples()}; }

RunResult runOnce(const Dataset& dataset, const BenchmarkOptions& options,
                  size_t runIndex) {
  RunResult result;
  result.runIndex = runIndex;
  result.odometryEntries = dataset.odometry.size();
  result.rangeTriples = dataset.triples.size();

  Vector priorSigmas = Vector3(1.0, 1.0, M_PI);
  Vector odoSigmas = Vector3(0.05, 0.01, 0.1);
  const double sigmaR = 100.0;
  const NM::Base::shared_ptr priorNoise = NM::Diagonal::Sigmas(priorSigmas);
  const NM::Base::shared_ptr looseNoise = NM::Isotropic::Sigma(2, 1000.0);
  const NM::Base::shared_ptr odoNoise = NM::Diagonal::Sigmas(odoSigmas);
  const NM::Base::shared_ptr gaussian = NM::Isotropic::Sigma(1, sigmaR);
  const NM::Base::shared_ptr tukey =
      NM::Robust::Create(NM::mEstimator::Tukey::Create(15), gaussian);
  const NM::Base::shared_ptr rangeNoise = options.robust ? tukey : gaussian;

  gtsam::ISAM2 isam;
  Pose2 pose0(-34.2086489999201, 45.3007639991120, M_PI - 2.021089);
  gtsam::NonlinearFactorGraph newFactors;
  newFactors.addPrior(0, pose0, priorNoise);
  gtsam::Values initial;
  initial.insert(0, pose0);

  std::mt19937_64 rng(options.seed);
  std::normal_distribution<double> normal(0.0, 100.0);
  set<Symbol> initializedLandmarks;

  size_t i = 1;
  size_t k = 0;
  bool initialized = false;
  Pose2 lastPose = pose0;
  size_t countK = 0;

  gtsam::Values finalResult;
  result.totalSeconds = gtsam::timing::measureSeconds([&] {
    result.solveSeconds = gtsam::timing::measureSeconds([&] {
      for (const TimedOdometry& timedOdometry : dataset.odometry) {
        double time = 0.0;
        Pose2 odometry;
        std::tie(time, odometry) = timedOdometry;

        newFactors.emplace_shared<gtsam::BetweenFactor<Pose2>>(
            i - 1, i, odometry, odoNoise);

        Pose2 predictedPose = lastPose.compose(odometry);
        lastPose = predictedPose;
        initial.insert(i, predictedPose);

        while (k < dataset.triples.size() &&
               time >= std::get<0>(dataset.triples[k])) {
          const size_t landmarkId = std::get<1>(dataset.triples[k]);
          const Symbol landmarkKey('L', landmarkId);
          const double range = std::get<2>(dataset.triples[k]);

          newFactors.emplace_shared<gtsam::RangeFactor<Pose2, Point2>>(
              i, landmarkKey, range, rangeNoise);
          ++result.rangeFactorsAdded;

          if (initializedLandmarks.count(landmarkKey) == 0) {
            initial.insert(landmarkKey, Point2(normal(rng), normal(rng)));
            initializedLandmarks.insert(landmarkKey);
            newFactors.emplace_shared<gtsam::PriorFactor<Point2>>(
                landmarkKey, Point2(0.0, 0.0), looseNoise);
          }

          ++k;
          ++countK;
        }

        if (k > options.minRanges && countK > options.incrementRanges) {
          if (!initialized) {
            result.batchInitializationSeconds +=
                gtsam::timing::measureSeconds([&] {
                  gtsam::LevenbergMarquardtOptimizer batchOptimizer(newFactors,
                                                                    initial);
                  initial = batchOptimizer.optimize();
                });
            initialized = true;
          }

          result.updateSeconds += gtsam::timing::measureSeconds(
              [&] { isam.update(newFactors, initial); });

          gtsam::Values estimate;
          result.calculateEstimateSeconds += gtsam::timing::measureSeconds(
              [&] { estimate = isam.calculateEstimate(); });
          lastPose = estimate.at<Pose2>(i);

          newFactors = gtsam::NonlinearFactorGraph();
          initial = gtsam::Values();
          countK = 0;
          ++result.updateCount;
        }

        ++i;
      }
    });
    result.finalEstimateSeconds = gtsam::timing::measureSeconds(
        [&] { finalResult = isam.calculateEstimate(); });
  });

  result.initializedLandmarks = initializedLandmarks.size();
  result.finalVariableCount = finalResult.size();
  return result;
}

void writeCsv(const vector<RunResult>& results, const string& outputPath) {
  std::ofstream output = gtsam::timing::openOutputFile(outputPath);

  output
      << "run_index,solve_seconds,total_seconds,batch_initialization_seconds,"
         "update_seconds,calculate_estimate_seconds,final_estimate_seconds,"
         "odometry_entries,range_triples,range_factors_added,update_count,"
         "initialized_landmarks,final_variable_count\n";
  output << std::fixed << std::setprecision(6);
  for (const RunResult& result : results) {
    output << result.runIndex << ',' << result.solveSeconds << ','
           << result.totalSeconds << ',' << result.batchInitializationSeconds
           << ',' << result.updateSeconds << ','
           << result.calculateEstimateSeconds << ','
           << result.finalEstimateSeconds << ',' << result.odometryEntries
           << ',' << result.rangeTriples << ',' << result.rangeFactorsAdded
           << ',' << result.updateCount << ',' << result.initializedLandmarks
           << ',' << result.finalVariableCount << '\n';
  }
}

void printSummary(const BenchmarkOptions& options, const Dataset& dataset,
                  const vector<RunResult>& measuredResults) {
  vector<double> solveSeconds;
  vector<double> totalSeconds;
  vector<double> batchSeconds;
  vector<double> updateSeconds;
  vector<double> estimateSeconds;
  for (const RunResult& result : measuredResults) {
    solveSeconds.push_back(result.solveSeconds);
    totalSeconds.push_back(result.totalSeconds);
    batchSeconds.push_back(result.batchInitializationSeconds);
    updateSeconds.push_back(result.updateSeconds);
    estimateSeconds.push_back(result.calculateEstimateSeconds);
  }

  const RunResult& shape = measuredResults.front();
  cout << std::fixed << std::setprecision(6);
  cout << "RangeFactor Plaza2 benchmark\n";
  cout << "dataset=Plaza2"
       << " warmup_runs=" << options.warmupRuns
       << " measured_runs=" << options.measuredRuns
       << " robust=" << (options.robust ? "true" : "false")
       << " seed=" << options.seed << '\n';
  cout << "odometry_entries=" << dataset.odometry.size()
       << " range_triples=" << dataset.triples.size()
       << " range_factors_added=" << shape.rangeFactorsAdded
       << " updates=" << shape.updateCount
       << " initialized_landmarks=" << shape.initializedLandmarks << '\n';
  const auto solveSummary = gtsam::timing::summarizeSamples(
      solveSeconds, gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto totalSummary = gtsam::timing::summarizeSamples(
      totalSeconds, gtsam::timing::MedianPolicy::kAverageMiddle);
  cout << "solve_seconds_mean=" << solveSummary.mean
       << " solve_seconds_median=" << solveSummary.median << '\n';
  cout << "total_seconds_mean=" << totalSummary.mean
       << " total_seconds_median=" << totalSummary.median << '\n';
  cout << "batch_initialization_seconds_mean="
       << gtsam::timing::summarizeSamples(
              batchSeconds, gtsam::timing::MedianPolicy::kAverageMiddle)
              .mean
       << '\n';
  cout << "update_seconds_mean="
       << gtsam::timing::summarizeSamples(
              updateSeconds, gtsam::timing::MedianPolicy::kAverageMiddle)
              .mean
       << '\n';
  cout << "calculate_estimate_seconds_mean="
       << gtsam::timing::summarizeSamples(
              estimateSeconds, gtsam::timing::MedianPolicy::kAverageMiddle)
              .mean
       << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  const BenchmarkOptions options{
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 5),
      arguments.sizeValue("--min-ranges", 150),
      arguments.sizeValue("--inc-ranges", 25),
      !arguments.flag("--no-robust"),
      arguments.uint64Value("--seed", 42),
      [&]() -> optional<string> {
        const string output = arguments.stringValue("--output", "");
        return output.empty() ? optional<string>() : optional<string>(output);
      }(),
  };
  arguments.validateAllConsumed();

  if (options.measuredRuns == 0) {
    throw std::invalid_argument("--repeats must be at least 1");
  }

  const Dataset dataset = loadDataset();

  for (size_t warmupIndex = 0; warmupIndex < options.warmupRuns;
       ++warmupIndex) {
    static_cast<void>(runOnce(dataset, options, warmupIndex));
  }

  vector<RunResult> measuredResults;
  measuredResults.reserve(options.measuredRuns);
  for (size_t runIndex = 0; runIndex < options.measuredRuns; ++runIndex) {
    measuredResults.push_back(runOnce(dataset, options, runIndex));
  }

  if (options.outputPath) {
    writeCsv(measuredResults, *options.outputPath);
  }
  printSummary(options, dataset, measuredResults);
  return 0;
}
