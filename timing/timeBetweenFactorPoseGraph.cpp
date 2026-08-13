/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeBetweenFactorPoseGraph.cpp
 * @brief Benchmark corrected factor Jacobians in full pose-graph optimization.
 * @date August 2026
 * @author Codex 5.6, prompted by Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/config.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/dataset.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::BetweenFactor;
using gtsam::Key;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::Matrix;
using gtsam::NoiseModelFactorN;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose2;
using gtsam::SharedNoiseModel;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;

struct BenchmarkOptions {
  std::string dataset = "w10000.graph";
  size_t warmups = 1;
  size_t repetitions = 5;
  size_t maxIterations = 100;
  bool useFastSync = true;
  std::optional<std::string> outputPath;
};

struct Graphs {
  NonlinearFactorGraph current;
  NonlinearFactorGraph legacy;
  Values initial;
  size_t betweenFactors = 0;
};

struct RunResult {
  double seconds = 0.0;
  size_t iterations = 0;
  size_t innerIterations = 0;
  double finalError = 0.0;
  double finalLambda = 0.0;
};

struct ComparisonResults {
  std::vector<RunResult> current;
  std::vector<RunResult> legacy;
};

class LegacyBetweenFactorPose2 : public NoiseModelFactorN<Pose2, Pose2> {
  using Base = NoiseModelFactorN<Pose2, Pose2>;

  Pose2 measured_;

 public:
  LegacyBetweenFactorPose2(Key key1, Key key2, const Pose2& measured,
                           const SharedNoiseModel& model)
      : Base(gtsam::noiseModel::validOrDefault(measured, model), key1, key2),
        measured_(measured) {}

  Vector evaluateError(const Pose2& p1, const Pose2& p2,
                       gtsam::OptionalMatrixType h1,
                       gtsam::OptionalMatrixType h2) const override {
    const Pose2 hx = gtsam::traits<Pose2>::Between(p1, p2, h1, h2);
    return gtsam::traits<Pose2>::Local(measured_, hx);
  }
};

class LegacyPriorFactorPose2 : public NoiseModelFactorN<Pose2> {
  using Base = NoiseModelFactorN<Pose2>;

  Pose2 prior_;

 public:
  LegacyPriorFactorPose2(Key key, const Pose2& prior,
                         const SharedNoiseModel& model)
      : Base(gtsam::noiseModel::validOrDefault(prior, model), key),
        prior_(prior) {}

  Vector evaluateError(const Pose2& value,
                       gtsam::OptionalMatrixType h) const override {
    if (h) *h = Matrix::Identity(3, 3);
    return -gtsam::traits<Pose2>::Local(value, prior_);
  }
};

Graphs loadGraphs(const std::string& dataset) {
  const auto [graph, initial] =
      gtsam::load2D(gtsam::findExampleDataFile(dataset));

  Graphs graphs;
  graphs.current = *graph;
  graphs.initial = *initial;
  graphs.legacy.reserve(graph->size() + 1);

  for (const auto& factor : *graph) {
    const auto between =
        std::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor);
    if (!between) {
      throw std::runtime_error(
          "Expected w10000 to contain only BetweenFactor<Pose2> factors");
    }
    graphs.legacy.emplace_shared<LegacyBetweenFactorPose2>(
        between->key<1>(), between->key<2>(), between->measured(),
        between->noiseModel());
    ++graphs.betweenFactors;
  }

  const Key anchorKey = 0;
  const Pose2 anchor = graphs.initial.at<Pose2>(anchorKey);
  const auto priorModel =
      gtsam::noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
  graphs.current.emplace_shared<gtsam::PriorFactor<Pose2>>(anchorKey, anchor,
                                                           priorModel);
  graphs.legacy.emplace_shared<LegacyPriorFactorPose2>(anchorKey, anchor,
                                                       priorModel);
  return graphs;
}

RunResult runOnce(const NonlinearFactorGraph& graph, const Values& initial,
                  const LevenbergMarquardtParams& params) {
  std::unique_ptr<LevenbergMarquardtOptimizer> optimizer;
  const double seconds = gtsam::timing::measureSeconds([&] {
    optimizer =
        std::make_unique<LevenbergMarquardtOptimizer>(graph, initial, params);
    optimizer->optimize();
  });
  return {seconds, optimizer->iterations(),
          static_cast<size_t>(optimizer->getInnerIterations()),
          optimizer->error(), optimizer->lambda()};
}

ComparisonResults runComparison(const Graphs& graphs,
                                const BenchmarkOptions& options) {
  LevenbergMarquardtParams params;
  params.maxIterations = options.maxIterations;

  for (size_t i = 0; i < options.warmups; ++i) {
    static_cast<void>(runOnce(graphs.current, graphs.initial, params));
    static_cast<void>(runOnce(graphs.legacy, graphs.initial, params));
  }

  ComparisonResults results;
  results.current.reserve(options.repetitions);
  results.legacy.reserve(options.repetitions);
  for (size_t i = 0; i < options.repetitions; ++i) {
    if (i % 2 == 0) {
      results.current.push_back(
          runOnce(graphs.current, graphs.initial, params));
      results.legacy.push_back(runOnce(graphs.legacy, graphs.initial, params));
    } else {
      results.legacy.push_back(runOnce(graphs.legacy, graphs.initial, params));
      results.current.push_back(
          runOnce(graphs.current, graphs.initial, params));
    }
  }
  return results;
}

std::vector<double> values(const std::vector<RunResult>& results,
                           double RunResult::* member) {
  std::vector<double> output;
  output.reserve(results.size());
  for (const RunResult& result : results) output.push_back(result.*member);
  return output;
}

std::vector<double> counts(const std::vector<RunResult>& results,
                           size_t RunResult::* member) {
  std::vector<double> output;
  output.reserve(results.size());
  for (const RunResult& result : results) {
    output.push_back(static_cast<double>(result.*member));
  }
  return output;
}

void printRuns(const std::string& name, const std::vector<RunResult>& results) {
  for (size_t i = 0; i < results.size(); ++i) {
    const RunResult& result = results[i];
    std::cout << name << "_run=" << i << " seconds=" << result.seconds
              << " iterations=" << result.iterations
              << " inner_iterations=" << result.innerIterations
              << " final_error=" << result.finalError
              << " final_lambda=" << result.finalLambda << '\n';
  }
}

void printUsage() {
  std::cout << "Usage: timeBetweenFactorPoseGraph [--dataset NAME] "
               "[--warmup N] [--repeats N] [--max-iterations N] "
               "[--raw-initial] [--output FILE]\n";
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
      arguments.sizeValue("--repeats", 5),
      arguments.sizeValue("--max-iterations", 100),
      !arguments.flag("--raw-initial"),
      [&] {
        const std::string path = arguments.stringValue("--output", "");
        return path.empty() ? std::optional<std::string>()
                            : std::optional<std::string>(path);
      }(),
  };
  arguments.validateAllConsumed();
  if (options.repetitions == 0 || options.maxIterations == 0) {
    throw std::invalid_argument(
        "--repeats and --max-iterations must both be at least 1");
  }

  Graphs graphs = loadGraphs(options.dataset);
  const double rawInitialError = graphs.current.error(graphs.initial);
  double initializationSeconds = 0.0;
  if (options.useFastSync) {
    initializationSeconds = gtsam::timing::measureSeconds(
        [&] { graphs.initial = gtsam::fastSync<Pose2>(graphs.current); });
  }
  const double initialError = graphs.current.error(graphs.initial);
  const ComparisonResults results = runComparison(graphs, options);

  const auto currentTime = gtsam::timing::summarizeSamples(
      values(results.current, &RunResult::seconds),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacyTime = gtsam::timing::summarizeSamples(
      values(results.legacy, &RunResult::seconds),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto currentIterations = gtsam::timing::summarizeSamples(
      counts(results.current, &RunResult::iterations),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacyIterations = gtsam::timing::summarizeSamples(
      counts(results.legacy, &RunResult::iterations),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto currentInnerIterations = gtsam::timing::summarizeSamples(
      counts(results.current, &RunResult::innerIterations),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacyInnerIterations = gtsam::timing::summarizeSamples(
      counts(results.legacy, &RunResult::innerIterations),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto currentError = gtsam::timing::summarizeSamples(
      values(results.current, &RunResult::finalError),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacyError = gtsam::timing::summarizeSamples(
      values(results.legacy, &RunResult::finalError),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto currentLambda = gtsam::timing::summarizeSamples(
      values(results.current, &RunResult::finalLambda),
      gtsam::timing::MedianPolicy::kAverageMiddle);
  const auto legacyLambda = gtsam::timing::summarizeSamples(
      values(results.legacy, &RunResult::finalLambda),
      gtsam::timing::MedianPolicy::kAverageMiddle);

#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
  constexpr const char* mode = "local_jacobians";
#else
  constexpr const char* mode = "legacy";
#endif

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Pose2 pose-graph optimization benchmark\n";
  std::cout << "dataset=" << options.dataset << " mode=" << mode
            << " initialization=" << (options.useFastSync ? "fast_sync" : "raw")
            << " poses=" << graphs.initial.size()
            << " between_factors=" << graphs.betweenFactors
            << " warmups=" << options.warmups
            << " repetitions=" << options.repetitions
            << " raw_initial_error=" << rawInitialError
            << " initial_error=" << initialError
            << " initialization_seconds=" << initializationSeconds << '\n';
  printRuns("current", results.current);
  printRuns("legacy", results.legacy);
  std::cout << "current_median_seconds=" << currentTime.median
            << " median_iterations=" << currentIterations.median
            << " median_inner_iterations=" << currentInnerIterations.median
            << " median_final_error=" << currentError.median
            << " median_final_lambda=" << currentLambda.median << '\n';
  std::cout << "legacy_median_seconds=" << legacyTime.median
            << " median_iterations=" << legacyIterations.median
            << " median_inner_iterations=" << legacyInnerIterations.median
            << " median_final_error=" << legacyError.median
            << " median_final_lambda=" << legacyLambda.median << '\n';
  std::cout << "current_over_legacy_time_ratio="
            << currentTime.median / legacyTime.median
            << " iteration_difference="
            << currentIterations.median - legacyIterations.median << '\n';

  if (options.outputPath) {
    gtsam::timing::writeBenchmarkActionMetrics(
        *options.outputPath,
        {{"PoseGraph/current", "s", currentTime.median},
         {"PoseGraph/legacy", "s", legacyTime.median},
         {"PoseGraph/current/iterations", "count", currentIterations.median},
         {"PoseGraph/legacy/iterations", "count", legacyIterations.median},
         {"PoseGraph/current/inner_iterations", "count",
          currentInnerIterations.median},
         {"PoseGraph/legacy/inner_iterations", "count",
          legacyInnerIterations.median},
         {"PoseGraph/current/final_error", "error", currentError.median},
         {"PoseGraph/legacy/final_error", "error", legacyError.median},
         {"PoseGraph/current/final_lambda", "lambda", currentLambda.median},
         {"PoseGraph/legacy/final_lambda", "lambda", legacyLambda.median},
         {"PoseGraph/initialization", "s", initializationSeconds},
         {"PoseGraph/initial_error", "error", initialError}});
  }
  return 0;
}
