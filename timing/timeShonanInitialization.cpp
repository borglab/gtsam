/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeShonanInitialization.cpp
 * @brief   Compare chordal and FAST-Sync initialization for Shonan averaging.
 * @date    August 2026
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>

#include <filesystem>
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

using gtsam::BetweenFactor;
using gtsam::BinaryMeasurement;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::ShonanAveraging3;
using gtsam::Values;
using std::optional;
using std::size_t;
using std::string;
using std::vector;

const string kChordal = "Chordal";
const string kFastMetis = "FAST-Sync/METIS";
const string kFastColamd = "FAST-Sync/COLAMD";

struct Options {
  string dataset = "pose3example-grid.txt";
  string optimizerMethod = "JACOBI";
  size_t warmups = 0;
  size_t repetitions = 1;
  size_t minimumRank = 3;
  size_t maximumRank = 10;
  double isotropicSigma = 0.0;
  optional<string> csvPath;
  optional<string> benchmarkActionJsonPath;
  bool help = false;
};

struct BenchmarkContext {
  string datasetPath;
  ShonanAveraging3::Measurements measurements;
  NonlinearFactorGraph chordalGraph;
  NonlinearFactorGraph fastSyncGraph;
  std::unique_ptr<ShonanAveraging3> shonan;
};

struct TrialResult {
  string initializer;
  size_t runIndex = 0;
  double initializationSeconds = 0.0;
  double solveSeconds = 0.0;
  double initialCost = 0.0;
  double finalCost = 0.0;
  double minimumEigenvalue = 0.0;

  double totalSeconds() const { return initializationSeconds + solveSeconds; }
};

struct MethodSummary {
  gtsam::timing::TimingSummary initialization;
  gtsam::timing::TimingSummary solve;
  gtsam::timing::TimingSummary total;
  gtsam::timing::TimingSummary initialCost;
  gtsam::timing::TimingSummary finalCost;
};

string usage() {
  return "Usage: timeShonanInitialization [options] [dataset]\n"
         "  --dataset FILE_OR_EXAMPLE       Input pose graph\n"
         "  --method NAME                   Shonan linear solver (default: "
         "JACOBI)\n"
         "  --warmup N                      Untimed runs per initializer\n"
         "  --repeats N                     Measured runs per initializer\n"
         "  --p-min N                       Minimum Shonan rank (default: 3)\n"
         "  --p-max N                       Maximum Shonan rank (default: 10)\n"
         "  --isotropic-sigma VALUE         Replace all rotation noise; zero "
         "preserves input\n"
         "  --csv FILE                      Write individual trials\n"
         "  --benchmark-action-json FILE    Write mean timing metrics\n"
         "  --help                          Show this message\n";
}

optional<string> nonemptyPath(const string& path) {
  return path.empty() ? optional<string>() : optional<string>(path);
}

Options parseOptions(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  Options options;
  options.help = arguments.helpRequested();
  const optional<string> datasetOption = arguments.optionalString("--dataset");
  options.optimizerMethod = arguments.stringValue("--method", "JACOBI");
  options.warmups = arguments.sizeValue("--warmup", 0);
  options.repetitions = arguments.sizeValue("--repeats", 1);
  options.minimumRank = arguments.sizeValue("--p-min", 3);
  options.maximumRank = arguments.sizeValue("--p-max", 10);
  options.isotropicSigma = arguments.doubleValue("--isotropic-sigma", 0.0);
  options.csvPath = nonemptyPath(arguments.stringValue("--csv", ""));
  options.benchmarkActionJsonPath =
      nonemptyPath(arguments.stringValue("--benchmark-action-json", ""));
  const vector<string> positionals = arguments.positionals();
  arguments.validateAllConsumed();

  if (datasetOption && !positionals.empty()) {
    throw std::invalid_argument(
        "Specify the dataset either positionally or with --dataset, not both");
  }
  if (positionals.size() > 1) throw std::invalid_argument(usage());
  if (datasetOption) {
    options.dataset = *datasetOption;
  } else if (!positionals.empty()) {
    options.dataset = positionals.front();
  }

  if (options.help) return options;
  if (options.repetitions == 0) {
    throw std::invalid_argument("--repeats must be at least 1");
  }
  if (options.minimumRank < 3) {
    throw std::invalid_argument("--p-min must be at least 3 for Rot3");
  }
  if (options.maximumRank < options.minimumRank) {
    throw std::invalid_argument("--p-max must be at least --p-min");
  }
  if (options.isotropicSigma < 0.0) {
    throw std::invalid_argument("--isotropic-sigma must be non-negative");
  }
  return options;
}

string resolveDataset(const string& dataset) {
  if (std::filesystem::exists(dataset)) return dataset;
  return gtsam::findExampleDataFile(dataset);
}

gtsam::noiseModel::Isotropic::shared_ptr isotropicModel(
    const BinaryMeasurement<Rot3>& measurement, double sigmaOverride) {
  if (sigmaOverride > 0.0) {
    return gtsam::noiseModel::Isotropic::Sigma(3, sigmaOverride);
  }
  const auto model = std::dynamic_pointer_cast<gtsam::noiseModel::Isotropic>(
      measurement.noiseModel());
  if (!model) {
    throw std::invalid_argument(
        "Rotation measurements must have isotropic noise for Shonan and "
        "FAST-Sync; use --isotropic-sigma to replace anisotropic noise");
  }
  return model;
}

BenchmarkContext buildContext(const Options& options) {
  BenchmarkContext context;
  context.datasetPath = resolveDataset(options.dataset);
  const ShonanAveraging3::Measurements parsedMeasurements =
      gtsam::parseMeasurements<Rot3>(context.datasetPath);
  if (parsedMeasurements.empty()) {
    throw std::invalid_argument("Dataset contains no Rot3 measurements");
  }

  context.measurements.reserve(parsedMeasurements.size());
  for (const auto& measurement : parsedMeasurements) {
    const auto rotationModel =
        isotropicModel(measurement, options.isotropicSigma);
    context.measurements.emplace_back(measurement.key1(), measurement.key2(),
                                      measurement.measured(), rotationModel);
    context.fastSyncGraph.emplace_shared<BetweenFactor<Rot3>>(
        measurement.key1(), measurement.key2(), measurement.measured(),
        rotationModel);

    const auto poseModel =
        gtsam::noiseModel::Isotropic::Sigma(6, rotationModel->sigma());
    context.chordalGraph.emplace_shared<BetweenFactor<Pose3>>(
        measurement.key1(), measurement.key2(),
        Pose3(measurement.measured(), gtsam::Point3::Zero()), poseModel);
  }

  const gtsam::Key anchorKey = context.measurements.front().key1();
  context.fastSyncGraph.addPrior(anchorKey, Rot3(),
                                 gtsam::noiseModel::Unit::Create(3));
  context.chordalGraph.addPrior(anchorKey, Pose3(),
                                gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));

  const gtsam::LevenbergMarquardtParams lmParameters =
      gtsam::LevenbergMarquardtParams::CeresDefaults();
  const ShonanAveraging3::Parameters shonanParameters(lmParameters,
                                                      options.optimizerMethod);
  context.shonan = std::make_unique<ShonanAveraging3>(context.measurements,
                                                      shonanParameters);
  return context;
}

TrialResult runTrial(const string& initializer, size_t runIndex,
                     const Options& options, const BenchmarkContext& context) {
  TrialResult result;
  result.initializer = initializer;
  result.runIndex = runIndex;

  Values initial;
  result.initializationSeconds = gtsam::timing::measureSeconds([&] {
    if (initializer == kChordal) {
      initial =
          gtsam::InitializePose3::initializeOrientations(context.chordalGraph);
    } else if (initializer == kFastMetis) {
      initial =
          gtsam::fastSync<Rot3>(context.fastSyncGraph, gtsam::Ordering::METIS);
    } else {
      initial =
          gtsam::fastSync<Rot3>(context.fastSyncGraph, gtsam::Ordering::COLAMD);
    }
  });
  result.initialCost = context.shonan->cost(initial);

  std::pair<Values, double> solution;
  result.solveSeconds = gtsam::timing::measureSeconds([&] {
    solution =
        context.shonan->run(initial, options.minimumRank, options.maximumRank);
  });
  result.finalCost = context.shonan->cost(solution.first);
  result.minimumEigenvalue = solution.second;
  return result;
}

void runMethods(size_t runIndex, const Options& options,
                const BenchmarkContext& context, vector<TrialResult>* results) {
  const vector<string> initializers{kChordal, kFastMetis, kFastColamd};
  for (size_t offset = 0; offset < initializers.size(); ++offset) {
    const string& initializer =
        initializers[(runIndex + offset) % initializers.size()];
    results->push_back(runTrial(initializer, runIndex, options, context));
  }
}

MethodSummary summarizeMethod(const vector<TrialResult>& results,
                              const string& initializer) {
  vector<double> initialization;
  vector<double> solve;
  vector<double> total;
  vector<double> initialCost;
  vector<double> finalCost;
  for (const TrialResult& result : results) {
    if (result.initializer != initializer) continue;
    initialization.push_back(result.initializationSeconds);
    solve.push_back(result.solveSeconds);
    total.push_back(result.totalSeconds());
    initialCost.push_back(result.initialCost);
    finalCost.push_back(result.finalCost);
  }
  const auto summarize = [](const vector<double>& samples) {
    return gtsam::timing::summarizeSamples(
        samples, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  return {summarize(initialization), summarize(solve), summarize(total),
          summarize(initialCost), summarize(finalCost)};
}

void writeCsv(const vector<TrialResult>& results, const string& path) {
  std::ofstream output = gtsam::timing::openOutputFile(path);
  output << "initializer,run_index,initialization_seconds,solve_seconds,"
            "total_seconds,initial_cost,final_cost,minimum_eigenvalue\n";
  output << std::fixed << std::setprecision(9);
  for (const TrialResult& result : results) {
    output << result.initializer << ',' << result.runIndex << ','
           << result.initializationSeconds << ',' << result.solveSeconds << ','
           << result.totalSeconds() << ',' << result.initialCost << ','
           << result.finalCost << ',' << result.minimumEigenvalue << '\n';
  }
}

void writeBenchmarkActionJson(const MethodSummary& chordal,
                              const MethodSummary& fastMetis,
                              const MethodSummary& fastColamd,
                              const string& dataset, const string& path) {
  const string prefix = "timeShonanInitialization/" +
                        std::filesystem::path(dataset).filename().string() +
                        "/";
  const vector<gtsam::timing::BenchmarkMetric> metrics{
      {prefix + "Chordal/initialization", "s", chordal.initialization.mean},
      {prefix + "Chordal/solve", "s", chordal.solve.mean},
      {prefix + "Chordal/total", "s", chordal.total.mean},
      {prefix + "FAST-Sync/METIS/initialization", "s",
       fastMetis.initialization.mean},
      {prefix + "FAST-Sync/METIS/solve", "s", fastMetis.solve.mean},
      {prefix + "FAST-Sync/METIS/total", "s", fastMetis.total.mean},
      {prefix + "FAST-Sync/COLAMD/initialization", "s",
       fastColamd.initialization.mean},
      {prefix + "FAST-Sync/COLAMD/solve", "s", fastColamd.solve.mean},
      {prefix + "FAST-Sync/COLAMD/total", "s", fastColamd.total.mean},
  };
  gtsam::timing::writeBenchmarkActionMetrics(path, metrics);
}

void printSummary(const Options& options, const BenchmarkContext& context,
                  const MethodSummary& chordal, const MethodSummary& fastMetis,
                  const MethodSummary& fastColamd) {
  std::cout << "Shonan initialization benchmark\n";
  std::cout << "dataset=" << context.datasetPath
            << " rotations=" << context.shonan->nrUnknowns()
            << " measurements=" << context.measurements.size()
            << " optimizer=" << options.optimizerMethod
            << " p_min=" << options.minimumRank
            << " p_max=" << options.maximumRank
            << " warmups=" << options.warmups
            << " repeats=" << options.repetitions;
  if (options.isotropicSigma > 0.0) {
    std::cout << " isotropic_sigma=" << options.isotropicSigma;
  }
  std::cout << '\n';

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "initializer        init_mean_s  init_median_s  solve_mean_s  "
               "solve_median_s  total_mean_s  initial_cost  final_cost\n";
  const auto printRow = [](const string& name, const MethodSummary& summary) {
    std::cout << std::left << std::setw(18) << name << std::right
              << std::setw(13) << summary.initialization.mean << std::setw(15)
              << summary.initialization.median << std::setw(14)
              << summary.solve.mean << std::setw(16) << summary.solve.median
              << std::setw(14) << summary.total.mean << std::setw(14)
              << summary.initialCost.mean << std::setw(12)
              << summary.finalCost.mean << '\n';
  };
  printRow(kChordal, chordal);
  printRow(kFastMetis, fastMetis);
  printRow(kFastColamd, fastColamd);
  const auto printSpeedup = [&](const string& name,
                                const MethodSummary& fastSummary) {
    std::cout << "speedup_chordal_over_" << name << " init="
              << chordal.initialization.mean / fastSummary.initialization.mean
              << " solve=" << chordal.solve.mean / fastSummary.solve.mean
              << " total=" << chordal.total.mean / fastSummary.total.mean
              << '\n';
  };
  printSpeedup("fast_metis", fastMetis);
  printSpeedup("fast_colamd", fastColamd);
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  if (options.help) {
    std::cout << usage();
    return 0;
  }

  const BenchmarkContext context = buildContext(options);
  vector<TrialResult> discardedWarmups;
  for (size_t runIndex = 0; runIndex < options.warmups; ++runIndex) {
    runMethods(runIndex, options, context, &discardedWarmups);
  }

  vector<TrialResult> results;
  results.reserve(3 * options.repetitions);
  for (size_t runIndex = 0; runIndex < options.repetitions; ++runIndex) {
    runMethods(runIndex, options, context, &results);
  }

  const MethodSummary chordal = summarizeMethod(results, kChordal);
  const MethodSummary fastMetis = summarizeMethod(results, kFastMetis);
  const MethodSummary fastColamd = summarizeMethod(results, kFastColamd);
  if (options.csvPath) writeCsv(results, *options.csvPath);
  if (options.benchmarkActionJsonPath) {
    writeBenchmarkActionJson(chordal, fastMetis, fastColamd,
                             context.datasetPath,
                             *options.benchmarkActionJsonPath);
  }
  printSummary(options, context, chordal, fastMetis, fastColamd);
  return 0;
}
