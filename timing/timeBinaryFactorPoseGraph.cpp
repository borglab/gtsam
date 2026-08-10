/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeBinaryFactorPoseGraph.cpp
 * @brief   Compare generic, binary, and between pose-graph linearization.
 * @date    August 2026
 * @author  Codex, prompted by Frank Dellaert
 */

#include <gtsam/linear/BetweenJacobianFactor.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <array>
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
  bool unitNoise = false;
  std::optional<string> outputPath;
};

enum class Mode { Generic, Binary, Between };

constexpr size_t modeIndex(Mode mode) { return static_cast<size_t>(mode); }

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::Generic:
      return "generic";
    case Mode::Binary:
      return "binary";
    case Mode::Between:
      return "between";
  }
  throw std::logic_error("Unknown benchmark mode");
}

struct TrialResult {
  size_t trial = 0;
  Mode mode = Mode::Generic;
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

/** Select the ordinary BinaryJacobianFactor as the structured-factor control.
 */
class BinaryBetweenFactor : public gtsam::BetweenFactor<Pose2> {
 public:
  using gtsam::BetweenFactor<Pose2>::BetweenFactor;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BinaryBetweenFactor>(*this);
  }

 protected:
  std::shared_ptr<gtsam::GaussianFactor> createBinaryJacobianFactor(
      const gtsam::Matrix& A1, const gtsam::Matrix& A2, const gtsam::Vector& b,
      const gtsam::SharedDiagonal& model) const override {
    const gtsam::Matrix3 fixedA1 = A1;
    const gtsam::Matrix3 fixedA2 = A2;
    const gtsam::Vector3 fixedB = b;
    return std::make_shared<gtsam::BinaryJacobianFactor<3, 3, 3>>(
        keys()[0], fixedA1, keys()[1], fixedA2, fixedB, model);
  }
};

template <class REPLACEMENT>
NonlinearFactorGraph replaceBetweenFactors(const NonlinearFactorGraph& source,
                                           bool unitNoise,
                                           size_t* replacedFactorCount) {
  NonlinearFactorGraph result;
  result.reserve(source.size());
  *replacedFactorCount = 0;
  for (const auto& factor : source) {
    const auto between =
        std::dynamic_pointer_cast<gtsam::BetweenFactor<Pose2>>(factor);
    if (between) {
      const gtsam::SharedNoiseModel model =
          unitNoise ? gtsam::noiseModel::Unit::Create(3)
                    : between->noiseModel();
      result.emplace_shared<REPLACEMENT>(between->key1(), between->key2(),
                                         between->measured(), model);
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
            factor) &&
        !std::dynamic_pointer_cast<gtsam::BetweenJacobianFactor<3>>(factor)) {
      ++count;
    }
  }
  return count;
}

size_t countBetweenJacobians(const gtsam::GaussianFactorGraph& graph) {
  size_t count = 0;
  for (const auto& factor : graph) {
    if (std::dynamic_pointer_cast<gtsam::BetweenJacobianFactor<3>>(factor)) {
      ++count;
    }
  }
  return count;
}

TrialResult runTrial(size_t trial, Mode mode, const NonlinearFactorGraph& graph,
                     const Values& initial, const Ordering& ordering,
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
          mode,
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

vector<double> samplesFor(const vector<TrialResult>& results, Mode mode,
                          bool linearize) {
  vector<double> samples;
  for (const TrialResult& result : results) {
    if (result.mode != mode) continue;
    samples.push_back(linearize ? result.linearizeMilliseconds
                                : result.optimizeMilliseconds);
  }
  return samples;
}

vector<double> pairedPercentChanges(const vector<TrialResult>& results,
                                    Mode baseline, Mode candidate,
                                    bool linearize) {
  vector<double> baselineSamples;
  vector<double> candidateSamples;
  for (const TrialResult& result : results) {
    if (result.mode != baseline && result.mode != candidate) continue;
    vector<double>& destination =
        result.mode == baseline ? baselineSamples : candidateSamples;
    destination.push_back(linearize ? result.linearizeMilliseconds
                                    : result.optimizeMilliseconds);
  }
  if (baselineSamples.size() != candidateSamples.size()) {
    throw std::runtime_error("Unpaired benchmark results");
  }

  vector<double> changes;
  changes.reserve(baselineSamples.size());
  for (size_t i = 0; i < baselineSamples.size(); ++i) {
    changes.push_back(100.0 * (candidateSamples[i] / baselineSamples[i] - 1.0));
  }
  return changes;
}

void writeCsv(const vector<TrialResult>& results, const string& outputPath,
              bool unitNoise) {
  std::ofstream output = gtsam::timing::openOutputFile(outputPath);
  output << "trial,noise_models,mode,linearize_ms,optimize_ms,final_error\n";
  output << std::fixed << std::setprecision(9);
  for (const TrialResult& result : results) {
    output << result.trial << ',' << (unitNoise ? "unit" : "dataset") << ','
           << modeName(result.mode) << ',' << result.linearizeMilliseconds
           << ',' << result.optimizeMilliseconds << ',' << result.finalError
           << '\n';
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
  const auto genericLinearize =
      summarize(samplesFor(results, Mode::Generic, true));
  const auto binaryLinearize =
      summarize(samplesFor(results, Mode::Binary, true));
  const auto betweenLinearize =
      summarize(samplesFor(results, Mode::Between, true));
  const auto genericOptimize =
      summarize(samplesFor(results, Mode::Generic, false));
  const auto binaryOptimize =
      summarize(samplesFor(results, Mode::Binary, false));
  const auto betweenOptimize =
      summarize(samplesFor(results, Mode::Between, false));
  const auto pairedBinaryLinearize = summarize(
      pairedPercentChanges(results, Mode::Generic, Mode::Binary, true));
  const auto pairedBetweenLinearize = summarize(
      pairedPercentChanges(results, Mode::Binary, Mode::Between, true));
  const auto pairedBinaryOptimize = summarize(
      pairedPercentChanges(results, Mode::Generic, Mode::Binary, false));
  const auto pairedBetweenOptimize = summarize(
      pairedPercentChanges(results, Mode::Binary, Mode::Between, false));

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Between Jacobian pose-graph benchmark\n";
  std::cout << "dataset=" << options.dataset
            << " warmup_runs=" << options.warmupRuns
            << " measured_runs=" << options.measuredRuns
            << " linearize_repeats=" << options.linearizeRepeats
            << " optimizer_iterations=" << options.optimizerIterations
            << " noise_models=" << (options.unitNoise ? "unit" : "dataset")
            << '\n';
  std::cout << "variables=" << variableCount << " factors=" << factorCount
            << " between_factors=" << betweenFactorCount << '\n';
  std::cout << "generic_linearize_ms_median=" << genericLinearize.median
            << " binary_linearize_ms_median=" << binaryLinearize.median
            << " between_linearize_ms_median=" << betweenLinearize.median
            << " binary_vs_generic_linearize_change_percent_median="
            << pairedBinaryLinearize.median
            << " between_vs_binary_linearize_change_percent_median="
            << pairedBetweenLinearize.median << '\n';
  std::cout << "generic_optimize_ms_median=" << genericOptimize.median
            << " binary_optimize_ms_median=" << binaryOptimize.median
            << " between_optimize_ms_median=" << betweenOptimize.median
            << " binary_vs_generic_optimize_change_percent_median="
            << pairedBinaryOptimize.median
            << " between_vs_binary_optimize_change_percent_median="
            << pairedBetweenOptimize.median << '\n';
  std::cout << "final_error=" << results.back().finalError
            << " max_pose_local_difference=" << maxResultDifference << '\n';
}

void printUsage() {
  std::cout
      << "Usage: timeBinaryFactorPoseGraph [options]\n"
      << "  --dataset NAME              Example dataset (default: "
         "w10000.graph)\n"
      << "  --warmup N                 Alternating warmup runs (default: 1)\n"
      << "  --repeats N                Alternating measured runs (default: "
         "7)\n"
      << "  --linearize-repeats N      Linearizations per sample (default: 3)\n"
      << "  --iterations N             LM iterations per sample (default: 10)\n"
      << "  --unit-noise               Replace all between-factor models with "
         "Unit(3)\n"
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
      arguments.flag("--unit-noise"),
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

  NonlinearFactorGraph sourceGraph = *loadedGraph;
  const gtsam::Key anchorKey = initial->keys().front();
  sourceGraph.addPrior(anchorKey, initial->at<Pose2>(anchorKey),
                       gtsam::noiseModel::Unit::Create(3));
  size_t genericFactorCount = 0, binaryFactorCount = 0,
         specializedFactorCount = 0;
  const NonlinearFactorGraph genericGraph =
      replaceBetweenFactors<GenericBetweenFactor<Pose2>>(
          sourceGraph, options.unitNoise, &genericFactorCount);
  const NonlinearFactorGraph binaryGraph =
      replaceBetweenFactors<BinaryBetweenFactor>(sourceGraph, options.unitNoise,
                                                 &binaryFactorCount);
  const NonlinearFactorGraph betweenGraph =
      replaceBetweenFactors<gtsam::BetweenFactor<Pose2>>(
          sourceGraph, options.unitNoise, &specializedFactorCount);
  const size_t betweenFactorCount = genericFactorCount;
  if (betweenFactorCount == 0) {
    throw std::runtime_error("Dataset has no BetweenFactor<Pose2> factors");
  }
  if (binaryFactorCount != betweenFactorCount ||
      specializedFactorCount != betweenFactorCount) {
    throw std::runtime_error("Benchmark graphs replaced different factors");
  }
  const Ordering ordering = Ordering::Create(Ordering::COLAMD, betweenGraph);
  const auto binaryLinear = binaryGraph.linearize(*initial);
  const auto genericLinear = genericGraph.linearize(*initial);
  const auto betweenLinear = betweenGraph.linearize(*initial);
  if (countBinaryJacobians(*binaryLinear) != betweenFactorCount ||
      countBinaryJacobians(*genericLinear) != 0 ||
      countBetweenJacobians(*betweenLinear) != betweenFactorCount ||
      countBetweenJacobians(*binaryLinear) != 0 ||
      countBetweenJacobians(*genericLinear) != 0) {
    throw std::runtime_error(
        "Graphs do not use the expected generic, binary, and between factors");
  }

  const auto graphForMode = [&](Mode mode) -> const NonlinearFactorGraph& {
    switch (mode) {
      case Mode::Generic:
        return genericGraph;
      case Mode::Binary:
        return binaryGraph;
      case Mode::Between:
        return betweenGraph;
    }
    throw std::logic_error("Unknown benchmark mode");
  };
  constexpr std::array<Mode, 3> modes{Mode::Generic, Mode::Binary,
                                      Mode::Between};

  for (size_t warmup = 0; warmup < options.warmupRuns; ++warmup) {
    for (size_t pass = 0; pass < modes.size(); ++pass) {
      const Mode mode = modes[(warmup + pass) % modes.size()];
      static_cast<void>(runTrial(warmup, mode, graphForMode(mode), *initial,
                                 ordering, options));
    }
  }

  vector<TrialResult> results;
  results.reserve(modes.size() * options.measuredRuns);
  std::array<Values, 3> lastResults;
  std::array<double, 3> lastErrors{};
  for (size_t trial = 0; trial < options.measuredRuns; ++trial) {
    for (size_t pass = 0; pass < modes.size(); ++pass) {
      const Mode mode = modes[(trial + pass) % modes.size()];
      TrialResult result = runTrial(trial + 1, mode, graphForMode(mode),
                                    *initial, ordering, options);
      lastErrors[modeIndex(mode)] = result.finalError;
      lastResults[modeIndex(mode)] = result.result;
      results.push_back(std::move(result));
    }
  }

  const double maxResultDifference =
      std::max(maxPoseDifference(lastResults[modeIndex(Mode::Generic)],
                                 lastResults[modeIndex(Mode::Binary)]),
               maxPoseDifference(lastResults[modeIndex(Mode::Generic)],
                                 lastResults[modeIndex(Mode::Between)]));
  const double genericError = lastErrors[modeIndex(Mode::Generic)];
  const double errorScale = std::max(1.0, std::abs(genericError));
  if (std::abs(genericError - lastErrors[modeIndex(Mode::Binary)]) >
          1e-12 * errorScale ||
      std::abs(genericError - lastErrors[modeIndex(Mode::Between)]) >
          1e-12 * errorScale ||
      maxResultDifference > 1e-9) {
    throw std::runtime_error(
        "Generic, binary, and between optimization results are not equivalent");
  }

  if (options.outputPath) {
    writeCsv(results, *options.outputPath, options.unitNoise);
  }
  printSummary(options, initial->size(), betweenGraph.size(),
               betweenFactorCount, results, maxResultDifference);
  return 0;
}
