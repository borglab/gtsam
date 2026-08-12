/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeBinaryFactorPoseGraph.cpp
 * @brief   Compare fixed-size binary and generic pose-graph linearization.
 * @date    August 2026
 * @author  Codex, prompted by Frank Dellaert
 */

#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Pose2;
using gtsam::Values;
using std::size_t;
using std::string;
using std::vector;

struct BenchmarkOptions {
  string dataset = "w10000.graph";
  size_t warmupRuns = 1;
  size_t measuredRuns = 7;
  size_t linearizeRepeats = 3;
  size_t optimizerIterations = 10;
  std::optional<string> outputPath;
};

struct TrialResult {
  size_t trial = 0;
  bool generic = false;
  double linearizeMilliseconds = 0.0;
  double optimizeMilliseconds = 0.0;
  double finalError = 0.0;
  Values result;
};

/** Force the pre-specialization NoiseModelFactor linearization path. */
template <class VALUE>
class GenericBetweenFactor : public gtsam::BetweenFactor<VALUE> {
 public:
  using gtsam::BetweenFactor<VALUE>::BetweenFactor;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<GenericBetweenFactor>(*this);
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(
      const Values& values) const override {
    return this->gtsam::NoiseModelFactor::linearize(values);
  }
};

NonlinearFactorGraph makeGenericGraph(const NonlinearFactorGraph& source,
                                      size_t* replacedFactorCount) {
  NonlinearFactorGraph result;
  result.reserve(source.size());
  *replacedFactorCount = 0;
  for (const auto& factor : source) {
    const auto between =
        std::dynamic_pointer_cast<gtsam::BetweenFactor<Pose2>>(factor);
    if (between) {
      result.emplace_shared<GenericBetweenFactor<Pose2>>(
          between->key1(), between->key2(), between->measured(),
          between->noiseModel());
      ++*replacedFactorCount;
    } else {
      result.push_back(factor);
    }
  }
  return result;
}

size_t countBinaryJacobians(const gtsam::GaussianFactorGraph& graph) {
  size_t count = 0;
  for (const auto& factor : graph) {
    if (std::dynamic_pointer_cast<gtsam::BinaryJacobianFactor<3, 3, 3>>(
            factor)) {
      ++count;
    }
  }
  return count;
}

TrialResult runTrial(size_t trial, bool generic,
                     const NonlinearFactorGraph& graph, const Values& initial,
                     const Ordering& ordering,
                     const BenchmarkOptions& options) {
  size_t accumulatedFactorCount = 0;
  const vector<double> linearizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        const auto linear = graph.linearize(initial);
        accumulatedFactorCount += linear->size();
      },
      0, options.linearizeRepeats);
  if (accumulatedFactorCount == 0) {
    throw std::runtime_error("Linearization produced an empty factor graph");
  }

  Values optimized;
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  const vector<double> optimizeSamples = gtsam::timing::measureMilliseconds(
      [&] {
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, ordering,
                                                     params);
        for (size_t iteration = 0; iteration < options.optimizerIterations;
             ++iteration) {
          optimizer.iterate();
        }
        optimized = optimizer.values();
      },
      0, 1);

  const auto linearizeSummary = gtsam::timing::summarizeSamples(
      linearizeSamples, gtsam::timing::MedianPolicy::kAverageMiddle);
  return {trial,
          generic,
          linearizeSummary.mean,
          optimizeSamples.front(),
          graph.error(optimized),
          std::move(optimized)};
}

double maxPoseDifference(const Values& expected, const Values& actual) {
  if (expected.size() != actual.size()) {
    throw std::runtime_error("Generic and binary results have different sizes");
  }

  double maximum = 0.0;
  for (gtsam::Key key : expected.keys()) {
    const Pose2& expectedPose = expected.at<Pose2>(key);
    const Pose2& actualPose = actual.at<Pose2>(key);
    maximum = std::max(
        maximum, gtsam::traits<Pose2>::Local(expectedPose, actualPose).norm());
  }
  return maximum;
}

vector<double> samplesFor(const vector<TrialResult>& results, bool generic,
                          bool linearize) {
  vector<double> samples;
  for (const TrialResult& result : results) {
    if (result.generic != generic) continue;
    samples.push_back(linearize ? result.linearizeMilliseconds
                                : result.optimizeMilliseconds);
  }
  return samples;
}

vector<double> pairedPercentChanges(const vector<TrialResult>& results,
                                    bool linearize) {
  vector<double> generic;
  vector<double> binary;
  for (const TrialResult& result : results) {
    vector<double>& destination = result.generic ? generic : binary;
    destination.push_back(linearize ? result.linearizeMilliseconds
                                    : result.optimizeMilliseconds);
  }
  if (generic.size() != binary.size()) {
    throw std::runtime_error("Unpaired benchmark results");
  }

  vector<double> changes;
  changes.reserve(generic.size());
  for (size_t i = 0; i < generic.size(); ++i) {
    changes.push_back(100.0 * (binary[i] / generic[i] - 1.0));
  }
  return changes;
}

void writeCsv(const vector<TrialResult>& results, const string& outputPath) {
  std::ofstream output = gtsam::timing::openOutputFile(outputPath);
  output << "trial,mode,linearize_ms,optimize_ms,final_error\n";
  output << std::fixed << std::setprecision(9);
  for (const TrialResult& result : results) {
    output << result.trial << ',' << (result.generic ? "generic" : "binary")
           << ',' << result.linearizeMilliseconds << ','
           << result.optimizeMilliseconds << ',' << result.finalError << '\n';
  }
}

void printSummary(const BenchmarkOptions& options, size_t variableCount,
                  size_t factorCount, size_t betweenFactorCount,
                  const vector<TrialResult>& results,
                  double maxResultDifference) {
  const auto summarize = [](const vector<double>& samples) {
    return gtsam::timing::summarizeSamples(
        samples, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  const auto genericLinearize = summarize(samplesFor(results, true, true));
  const auto binaryLinearize = summarize(samplesFor(results, false, true));
  const auto genericOptimize = summarize(samplesFor(results, true, false));
  const auto binaryOptimize = summarize(samplesFor(results, false, false));
  const auto pairedLinearize = summarize(pairedPercentChanges(results, true));
  const auto pairedOptimize = summarize(pairedPercentChanges(results, false));

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Binary factor pose-graph benchmark\n";
  std::cout << "dataset=" << options.dataset
            << " warmup_runs=" << options.warmupRuns
            << " measured_runs=" << options.measuredRuns
            << " linearize_repeats=" << options.linearizeRepeats
            << " optimizer_iterations=" << options.optimizerIterations << '\n';
  std::cout << "variables=" << variableCount << " factors=" << factorCount
            << " between_factors=" << betweenFactorCount << '\n';
  std::cout << "generic_linearize_ms_median=" << genericLinearize.median
            << " binary_linearize_ms_median=" << binaryLinearize.median
            << " paired_linearize_change_percent_median="
            << pairedLinearize.median << '\n';
  std::cout << "generic_optimize_ms_median=" << genericOptimize.median
            << " binary_optimize_ms_median=" << binaryOptimize.median
            << " paired_optimize_change_percent_median="
            << pairedOptimize.median << '\n';
  std::cout << "final_error=" << results.back().finalError
            << " max_pose_local_difference=" << maxResultDifference << '\n';
}

void printUsage() {
  std::cout
      << "Usage: timeBinaryFactorPoseGraph [options]\n"
      << "  --dataset NAME              Example dataset (default: "
         "w10000.graph)\n"
      << "  --warmup N                 Alternating warmup pairs (default: 1)\n"
      << "  --repeats N                Alternating measured pairs (default: "
         "7)\n"
      << "  --linearize-repeats N      Linearizations per sample (default: 3)\n"
      << "  --iterations N             LM iterations per sample (default: 10)\n"
      << "  --output PATH              Optional per-trial CSV output\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }

  const BenchmarkOptions options{
      arguments.stringValue("--dataset", "w10000.graph"),
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 7),
      arguments.sizeValue("--linearize-repeats", 3),
      arguments.sizeValue("--iterations", 10),
      arguments.optionalString("--output"),
  };
  arguments.validateAllConsumed();
  if (options.measuredRuns == 0 || options.linearizeRepeats == 0 ||
      options.optimizerIterations == 0) {
    throw std::invalid_argument(
        "--repeats, --linearize-repeats, and --iterations must be positive");
  }

  const auto [loadedGraph, initial] =
      gtsam::load2D(gtsam::findExampleDataFile(options.dataset));
  if (initial->empty()) throw std::runtime_error("Dataset has no variables");

  NonlinearFactorGraph binaryGraph = *loadedGraph;
  const gtsam::Key anchorKey = initial->keys().front();
  binaryGraph.addPrior(anchorKey, initial->at<Pose2>(anchorKey),
                       gtsam::noiseModel::Unit::Create(3));
  size_t betweenFactorCount = 0;
  const NonlinearFactorGraph genericGraph =
      makeGenericGraph(binaryGraph, &betweenFactorCount);
  if (betweenFactorCount == 0) {
    throw std::runtime_error("Dataset has no BetweenFactor<Pose2> factors");
  }
  const Ordering ordering = Ordering::Create(Ordering::COLAMD, binaryGraph);
  const auto binaryLinear = binaryGraph.linearize(*initial);
  const auto genericLinear = genericGraph.linearize(*initial);
  if (countBinaryJacobians(*binaryLinear) != betweenFactorCount ||
      countBinaryJacobians(*genericLinear) != 0) {
    throw std::runtime_error(
        "Graphs do not use the expected binary and generic linear factors");
  }

  for (size_t warmup = 0; warmup < options.warmupRuns; ++warmup) {
    const bool genericFirst = warmup % 2 == 0;
    for (size_t pass = 0; pass < 2; ++pass) {
      const bool generic = genericFirst ? pass == 0 : pass == 1;
      const NonlinearFactorGraph& graph = generic ? genericGraph : binaryGraph;
      static_cast<void>(
          runTrial(warmup, generic, graph, *initial, ordering, options));
    }
  }

  vector<TrialResult> results;
  results.reserve(2 * options.measuredRuns);
  Values lastGenericResult, lastBinaryResult;
  double lastGenericError = 0.0, lastBinaryError = 0.0;
  for (size_t trial = 0; trial < options.measuredRuns; ++trial) {
    const bool genericFirst = trial % 2 == 0;
    for (size_t pass = 0; pass < 2; ++pass) {
      const bool generic = genericFirst ? pass == 0 : pass == 1;
      const NonlinearFactorGraph& graph = generic ? genericGraph : binaryGraph;
      TrialResult result =
          runTrial(trial + 1, generic, graph, *initial, ordering, options);
      if (generic) {
        lastGenericError = result.finalError;
        lastGenericResult = result.result;
      } else {
        lastBinaryError = result.finalError;
        lastBinaryResult = result.result;
      }
      results.push_back(std::move(result));
    }
  }

  const double maxResultDifference =
      maxPoseDifference(lastGenericResult, lastBinaryResult);
  const double errorScale = std::max(1.0, std::abs(lastGenericError));
  if (std::abs(lastGenericError - lastBinaryError) > 1e-12 * errorScale ||
      maxResultDifference > 1e-9) {
    throw std::runtime_error(
        "Generic and binary optimization results are not equivalent");
  }

  if (options.outputPath) writeCsv(results, *options.outputPath);
  printSummary(options, initial->size(), binaryGraph.size(), betweenFactorCount,
               results, maxResultDifference);
  return 0;
}
