/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeShonanInitialization.cpp
 * @brief   Compare Shonan before and after FAST-Sync initialization with PCG
 *          and LM.
 * @date    August 2026
 * @author  Frank Dellaert
 *
 * The before path uses chordal initialization and the default PCG solver. The
 * after paths use FAST-Sync with the default PCG solver or direct LM solves.
 * Run with --help for all command-line arguments and an example.
 */

#include <gtsam/base/DSFMap.h>
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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
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

constexpr size_t kMinimumRank = 3;
constexpr size_t kMaximumRank = 10;
constexpr double kOptimalityThreshold = -1e-4;
const string kBefore = "Before";
const string kAfterPcg = "After/PCG";
const string kAfterLm = "After/LM";

const vector<string>& benchmarkMethods() {
  static const vector<string> methods{kBefore, kAfterPcg, kAfterLm};
  return methods;
}

struct Options {
  string dataset = "pose3example-grid.txt";
  optional<string> g2oDirectory;
  size_t largestFiles = 0;
  size_t warmups = 0;
  size_t repetitions = 1;
  double isotropicSigma = 0.0;
  optional<string> csvPath;
  optional<string> benchmarkActionJsonPath;
  bool help = false;
};

struct BenchmarkContext {
  string datasetPath;
  size_t rawRotations = 0;
  size_t rawMeasurements = 0;
  size_t components = 0;
  ShonanAveraging3::Measurements measurements;
  NonlinearFactorGraph chordalGraph;
  NonlinearFactorGraph fastSyncGraph;
  gtsam::Ordering fastSyncOrdering;
  std::unique_ptr<ShonanAveraging3> beforeShonan;
  std::unique_ptr<ShonanAveraging3> afterPcgShonan;
  std::unique_ptr<ShonanAveraging3> afterLmShonan;
};

struct TrialResult {
  string dataset;
  string initializer;
  size_t rotations = 0;
  size_t measurements = 0;
  size_t runIndex = 0;
  size_t rankReached = 0;
  double initializationSeconds = 0.0;
  double solveSeconds = 0.0;
  double initialCost = std::numeric_limits<double>::quiet_NaN();
  double finalCost = std::numeric_limits<double>::quiet_NaN();
  double minimumEigenvalue = std::numeric_limits<double>::quiet_NaN();
  bool success = false;
  string error;

  double totalSeconds() const { return initializationSeconds + solveSeconds; }
};

struct MethodSummary {
  gtsam::timing::TimingSummary initialization;
  gtsam::timing::TimingSummary solve;
  gtsam::timing::TimingSummary total;
  gtsam::timing::TimingSummary initialCost;
  gtsam::timing::TimingSummary finalCost;
  size_t minimumRank = 0;
  size_t maximumRank = 0;
  size_t successes = 0;
  size_t failures = 0;
};

struct DatasetBenchmark {
  string path;
  size_t rawRotations = 0;
  size_t rawMeasurements = 0;
  size_t components = 0;
  size_t rotations = 0;
  size_t measurements = 0;
  vector<TrialResult> results;
};

struct StaircaseResult {
  Values values;
  double minimumEigenvalue = 0.0;
  size_t rankReached = 0;
};

struct PreparedMeasurements {
  ShonanAveraging3::Measurements measurements;
  size_t rawRotations = 0;
  size_t rawMeasurements = 0;
  size_t components = 0;
};

string usage() {
  return "Usage: timeShonanInitialization [options] [dataset]\n"
         "  dataset                         Positional input pose graph\n"
         "  --dataset FILE_OR_EXAMPLE       Input pose graph (default: "
         "pose3example-grid.txt)\n"
         "  --g2o-directory DIR             Benchmark each .g2o file directly "
         "in DIR\n"
         "  --largest N                    With --g2o-directory, benchmark the "
         "N largest .g2o files by size\n"
         "  --warmup N                      Untimed runs per initializer "
         "(default: 0)\n"
         "  --repeats N                     Measured runs per initializer "
         "(default: 1)\n"
         "  --isotropic-sigma VALUE         Replace all rotation noise; zero "
         "preserves input (default: 0)\n"
         "  --csv FILE                      Write individual trials in "
         "seconds\n"
         "  --benchmark-action-json FILE    Write mean metrics for Benchmark "
         "Action\n"
         "  --help                          Show this message\n"
         "\nSpecify at most one of dataset, --dataset, and --g2o-directory. "
         "The console table reports rounded integer milliseconds.\n"
         "The Shonan staircase always runs from p=3 through p=10. "
         "Disconnected inputs are reduced to their largest component.\n"
         "\nBefore: chordal initialization with PCG.\n"
         "After/PCG: FAST-Sync initialization with PCG.\n"
         "After/LM:  FAST-Sync initialization with direct LM solves.\n"
         "\nExample:\n"
         "  timeShonanInitialization --g2o-directory datasets/yfcc2 "
         "--largest 5 --repeats 1 --csv shonan.csv\n";
}

optional<string> nonemptyPath(const string& path) {
  return path.empty() ? optional<string>() : optional<string>(path);
}

Options parseOptions(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  Options options;
  options.help = arguments.helpRequested();
  const optional<string> datasetOption = arguments.optionalString("--dataset");
  options.g2oDirectory = arguments.optionalString("--g2o-directory");
  options.largestFiles = arguments.sizeValue("--largest", 0);
  options.warmups = arguments.sizeValue("--warmup", 0);
  options.repetitions = arguments.sizeValue("--repeats", 1);
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
  if (options.g2oDirectory && (datasetOption || !positionals.empty())) {
    throw std::invalid_argument(
        "--g2o-directory cannot be combined with a single dataset");
  }
  if (options.largestFiles > 0 && !options.g2oDirectory) {
    throw std::invalid_argument("--largest requires --g2o-directory");
  }
  if (datasetOption) {
    options.dataset = *datasetOption;
  } else if (!positionals.empty()) {
    options.dataset = positionals.front();
  }

  if (options.help) return options;
  if (options.repetitions == 0) {
    throw std::invalid_argument("--repeats must be at least 1");
  }
  if (options.isotropicSigma < 0.0) {
    throw std::invalid_argument("--isotropic-sigma must be non-negative");
  }
  return options;
}

string resolveDataset(const string& dataset) {
  if (std::filesystem::is_regular_file(dataset)) return dataset;
  return gtsam::findExampleDataFile(dataset);
}

vector<string> resolveDatasets(const Options& options) {
  if (!options.g2oDirectory) return {resolveDataset(options.dataset)};

  const std::filesystem::path directory(*options.g2oDirectory);
  if (!std::filesystem::is_directory(directory)) {
    throw std::invalid_argument("Not a directory: " + directory.string());
  }

  vector<string> datasets;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".g2o") {
      datasets.push_back(entry.path().string());
    }
  }
  if (datasets.empty()) {
    throw std::invalid_argument("No .g2o files found in " + directory.string());
  }
  std::sort(datasets.begin(), datasets.end(),
            [](const string& left, const string& right) {
              const auto leftSize = std::filesystem::file_size(left);
              const auto rightSize = std::filesystem::file_size(right);
              if (leftSize != rightSize) return leftSize > rightSize;
              return left < right;
            });
  if (options.largestFiles > 0 && options.largestFiles < datasets.size()) {
    datasets.resize(options.largestFiles);
  }
  return datasets;
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

PreparedMeasurements prepareMeasurements(
    const ShonanAveraging3::Measurements& parsedMeasurements) {
  gtsam::DSFMap<gtsam::Key> components;
  for (const auto& measurement : parsedMeasurements) {
    components.merge(measurement.key1(), measurement.key2());
  }

  const std::map<gtsam::Key, std::set<gtsam::Key>> componentSets =
      components.sets();
  if (componentSets.empty()) {
    throw std::invalid_argument("Dataset contains no Rot3 measurements");
  }

  const std::set<gtsam::Key>* largestComponent = nullptr;
  size_t rawRotations = 0;
  for (const auto& [representative, keys] : componentSets) {
    (void)representative;
    rawRotations += keys.size();
    if (!largestComponent || keys.size() > largestComponent->size()) {
      largestComponent = &keys;
    }
  }

  std::map<gtsam::Key, gtsam::Key> remapping;
  gtsam::Key nextKey = 0;
  for (const gtsam::Key key : *largestComponent) {
    remapping.emplace(key, nextKey++);
  }

  ShonanAveraging3::Measurements measurements;
  measurements.reserve(parsedMeasurements.size());
  for (const auto& measurement : parsedMeasurements) {
    const auto key1 = remapping.find(measurement.key1());
    const auto key2 = remapping.find(measurement.key2());
    if (key1 != remapping.end() && key2 != remapping.end()) {
      measurements.emplace_back(key1->second, key2->second,
                                measurement.measured(),
                                measurement.noiseModel());
    }
  }
  return {std::move(measurements), rawRotations, parsedMeasurements.size(),
          componentSets.size()};
}

BenchmarkContext buildContext(const Options& options,
                              const string& datasetPath) {
  BenchmarkContext context;
  context.datasetPath = datasetPath;
  const ShonanAveraging3::Measurements parsedMeasurements =
      gtsam::parseMeasurements<Rot3>(context.datasetPath);
  if (parsedMeasurements.empty()) {
    throw std::invalid_argument("Dataset contains no Rot3 measurements");
  }

  PreparedMeasurements prepared = prepareMeasurements(parsedMeasurements);
  context.rawRotations = prepared.rawRotations;
  context.rawMeasurements = prepared.rawMeasurements;
  context.components = prepared.components;

  context.measurements.reserve(prepared.measurements.size());
  for (const auto& measurement : prepared.measurements) {
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
  context.chordalGraph.addPrior(anchorKey, Pose3(),
                                gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));

  gtsam::LevenbergMarquardtParams lmParameters =
      gtsam::LevenbergMarquardtParams::CeresDefaults();
  lmParameters.setOrderingType("COLAMD");
  const ShonanAveraging3::Parameters beforeParameters(lmParameters, "JACOBI",
                                                      kOptimalityThreshold);
  const ShonanAveraging3::Parameters afterPcgParameters(lmParameters, "JACOBI",
                                                        kOptimalityThreshold);
  ShonanAveraging3::Parameters afterLmParameters(lmParameters, "CHOLESKY",
                                                 kOptimalityThreshold);
  context.fastSyncOrdering =
      gtsam::Ordering::Create(gtsam::Ordering::METIS, context.fastSyncGraph);
  if (afterLmParameters.lm.requiresOrdering()) {
    afterLmParameters.lm.setOrdering(context.fastSyncOrdering);
  }
  context.beforeShonan = std::make_unique<ShonanAveraging3>(
      context.measurements, beforeParameters);
  context.afterPcgShonan = std::make_unique<ShonanAveraging3>(
      context.measurements, afterPcgParameters);
  context.afterLmShonan = std::make_unique<ShonanAveraging3>(
      context.measurements, afterLmParameters);
  return context;
}

StaircaseResult runStaircase(const ShonanAveraging3& shonan,
                             const Values& initial, size_t* rankReached) {
  Values initialAtRank = ShonanAveraging3::LiftTo<Rot3>(kMinimumRank, initial);
  for (size_t rank = kMinimumRank; rank <= kMaximumRank; ++rank) {
    *rankReached = rank;
    const Values optimum = shonan.tryOptimizingAt(rank, initialAtRank);
    gtsam::Vector minimumEigenvector;
    const double minimumEigenvalue =
        shonan.computeMinEigenValue(optimum, &minimumEigenvector);
    if (minimumEigenvalue > kOptimalityThreshold) {
      return {shonan.roundSolution(optimum), minimumEigenvalue, rank};
    }
    if (rank < kMaximumRank) {
      initialAtRank = shonan.initializeWithDescent(
          rank + 1, optimum, minimumEigenvector, minimumEigenvalue);
    }
  }
  throw std::runtime_error("Shonan did not converge by p=10");
}

double elapsedSeconds(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

TrialResult runTrial(const string& initializer, size_t runIndex,
                     const BenchmarkContext& context) {
  TrialResult result;
  result.dataset = context.datasetPath;
  result.initializer = initializer;
  result.rotations = context.beforeShonan->nrUnknowns();
  result.measurements = context.measurements.size();
  result.runIndex = runIndex;

  Values initial;
  auto start = std::chrono::steady_clock::now();
  try {
    if (initializer == kBefore) {
      initial =
          gtsam::InitializePose3::initializeOrientations(context.chordalGraph);
    } else {
      initial = gtsam::fastSync<Rot3>(context.fastSyncGraph,
                                      context.fastSyncOrdering);
    }
  } catch (const std::exception& exception) {
    result.initializationSeconds = elapsedSeconds(start);
    result.error = string("initialization: ") + exception.what();
    return result;
  }
  result.initializationSeconds = elapsedSeconds(start);
  const ShonanAveraging3* shonan = nullptr;
  if (initializer == kBefore) {
    shonan = context.beforeShonan.get();
  } else if (initializer == kAfterPcg) {
    shonan = context.afterPcgShonan.get();
  } else if (initializer == kAfterLm) {
    shonan = context.afterLmShonan.get();
  } else {
    throw std::invalid_argument("Unknown benchmark method: " + initializer);
  }
  result.initialCost = shonan->cost(initial);

  start = std::chrono::steady_clock::now();
  try {
    const StaircaseResult solution =
        runStaircase(*shonan, initial, &result.rankReached);
    result.solveSeconds = elapsedSeconds(start);
    result.finalCost = shonan->cost(solution.values);
    result.minimumEigenvalue = solution.minimumEigenvalue;
    result.rankReached = solution.rankReached;
    result.success = true;
  } catch (const std::exception& exception) {
    result.solveSeconds = elapsedSeconds(start);
    result.error = string("solve: ") + exception.what();
  }
  return result;
}

void runMethods(size_t runIndex, const BenchmarkContext& context,
                vector<TrialResult>* results) {
  for (size_t offset = 0; offset < benchmarkMethods().size(); ++offset) {
    const string& initializer =
        benchmarkMethods()[(runIndex + offset) % benchmarkMethods().size()];
    results->push_back(runTrial(initializer, runIndex, context));
  }
}

optional<MethodSummary> summarizeMethod(const vector<TrialResult>& results,
                                        const string& initializer) {
  vector<double> initialization;
  vector<double> solve;
  vector<double> total;
  vector<double> initialCost;
  vector<double> finalCost;
  vector<size_t> ranks;
  size_t failures = 0;
  for (const TrialResult& result : results) {
    if (result.initializer != initializer) continue;
    if (!result.success) {
      ++failures;
      continue;
    }
    initialization.push_back(result.initializationSeconds);
    solve.push_back(result.solveSeconds);
    total.push_back(result.totalSeconds());
    initialCost.push_back(result.initialCost);
    finalCost.push_back(result.finalCost);
    ranks.push_back(result.rankReached);
  }
  if (ranks.empty()) return std::nullopt;
  const auto summarize = [](const vector<double>& samples) {
    return gtsam::timing::summarizeSamples(
        samples, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  return MethodSummary{summarize(initialization),
                       summarize(solve),
                       summarize(total),
                       summarize(initialCost),
                       summarize(finalCost),
                       *std::min_element(ranks.begin(), ranks.end()),
                       *std::max_element(ranks.begin(), ranks.end()),
                       ranks.size(),
                       failures};
}

DatasetBenchmark runDataset(const Options& options, const string& datasetPath) {
  const BenchmarkContext context = buildContext(options, datasetPath);
  DatasetBenchmark benchmark;
  benchmark.path = datasetPath;
  benchmark.rawRotations = context.rawRotations;
  benchmark.rawMeasurements = context.rawMeasurements;
  benchmark.components = context.components;
  benchmark.rotations = context.beforeShonan->nrUnknowns();
  benchmark.measurements = context.measurements.size();

  vector<TrialResult> discardedWarmups;
  for (size_t runIndex = 0; runIndex < options.warmups; ++runIndex) {
    runMethods(runIndex, context, &discardedWarmups);
  }

  benchmark.results.reserve(benchmarkMethods().size() * options.repetitions);
  for (size_t runIndex = 0; runIndex < options.repetitions; ++runIndex) {
    runMethods(runIndex, context, &benchmark.results);
  }
  return benchmark;
}

void writeCsv(const vector<DatasetBenchmark>& benchmarks, const string& path) {
  std::ofstream output = gtsam::timing::openOutputFile(path);
  output << "dataset,initializer,raw_rotations,raw_measurements,components,"
            "rotations,measurements,run_index,success,p_reached,"
            "initialization_seconds,solve_seconds,total_seconds,initial_cost,"
            "final_cost,minimum_eigenvalue,error\n";
  output << std::fixed << std::setprecision(9);
  for (const DatasetBenchmark& benchmark : benchmarks) {
    for (const TrialResult& result : benchmark.results) {
      output << std::quoted(result.dataset) << ','
             << std::quoted(result.initializer) << ',' << benchmark.rawRotations
             << ',' << benchmark.rawMeasurements << ',' << benchmark.components
             << ',' << result.rotations << ',' << result.measurements << ','
             << result.runIndex << ',' << (result.success ? "true" : "false")
             << ',' << result.rankReached << ',' << result.initializationSeconds
             << ',' << result.solveSeconds << ',' << result.totalSeconds()
             << ',' << result.initialCost << ',' << result.finalCost << ','
             << result.minimumEigenvalue << ',' << std::quoted(result.error)
             << '\n';
    }
  }
}

void writeBenchmarkActionJson(const vector<DatasetBenchmark>& benchmarks,
                              const string& path) {
  vector<gtsam::timing::BenchmarkMetric> metrics;
  for (const DatasetBenchmark& benchmark : benchmarks) {
    const string prefix =
        "timeShonanInitialization/" +
        std::filesystem::path(benchmark.path).filename().string() + "/";
    for (const string& initializer : benchmarkMethods()) {
      const auto summary = summarizeMethod(benchmark.results, initializer);
      if (!summary) continue;
      metrics.push_back({prefix + initializer + "/initialization", "s",
                         summary->initialization.mean});
      metrics.push_back(
          {prefix + initializer + "/solve", "s", summary->solve.mean});
      metrics.push_back(
          {prefix + initializer + "/total", "s", summary->total.mean});
      metrics.push_back({prefix + initializer + "/rank_reached", "rank",
                         static_cast<double>(summary->maximumRank)});
    }
  }
  gtsam::timing::writeBenchmarkActionMetrics(path, metrics);
}

string rankText(const MethodSummary& summary) {
  if (summary.minimumRank == summary.maximumRank) {
    return std::to_string(summary.minimumRank);
  }
  return std::to_string(summary.minimumRank) + "-" +
         std::to_string(summary.maximumRank);
}

long long milliseconds(double seconds) {
  return std::llround(1000.0 * seconds);
}

string compactError(const string& error) {
  std::ostringstream compact;
  bool previousWasSpace = false;
  for (const char character : error) {
    if (std::isspace(static_cast<unsigned char>(character))) {
      if (!previousWasSpace) compact << ' ';
      previousWasSpace = true;
    } else {
      compact << (character == '|' ? '/' : character);
      previousWasSpace = false;
    }
    if (compact.tellp() >= 120) {
      compact << "...";
      break;
    }
  }
  return compact.str();
}

void printSummary(const Options& options,
                  const vector<DatasetBenchmark>& benchmarks) {
  std::cout << "Shonan initialization benchmark\n"
            << "p_min=" << kMinimumRank << " p_max=" << kMaximumRank
            << " warmups=" << options.warmups
            << " repeats=" << options.repetitions;
  if (options.isotropicSigma > 0.0) {
    std::cout << " isotropic_sigma=" << options.isotropicSigma;
  }
  std::cout << "\n\n";
  std::cout << "Times are init/solve/total milliseconds; each cell ends with "
               "p reached and success count.\n\n"
            << "| Dataset | Before | After/PCG | After/LM |\n"
            << "|---|---|---|---|\n";
  for (const DatasetBenchmark& benchmark : benchmarks) {
    const string dataset =
        std::filesystem::path(benchmark.path).filename().string();
    std::cout << "| " << dataset;
    for (const string& initializer : benchmarkMethods()) {
      const auto summary = summarizeMethod(benchmark.results, initializer);
      std::cout << " | ";
      if (!summary) {
        size_t rankReached = 0;
        string error = "failed";
        for (const TrialResult& result : benchmark.results) {
          if (result.initializer == initializer) {
            rankReached = std::max(rankReached, result.rankReached);
            if (!result.error.empty()) error = result.error;
          }
        }
        std::cout << "- (p=" << rankReached << ", " << compactError(error)
                  << ")";
      } else {
        std::cout << milliseconds(summary->initialization.mean) << "/"
                  << milliseconds(summary->solve.mean) << "/"
                  << milliseconds(summary->total.mean)
                  << " ms, p=" << rankText(*summary) << ", "
                  << summary->successes << "/"
                  << summary->successes + summary->failures;
      }
    }
    std::cout << " |\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  if (options.help) {
    std::cout << usage();
    return 0;
  }

  const vector<string> datasetPaths = resolveDatasets(options);
  vector<DatasetBenchmark> benchmarks;
  benchmarks.reserve(datasetPaths.size());
  for (size_t index = 0; index < datasetPaths.size(); ++index) {
    const string& datasetPath = datasetPaths[index];
    std::cerr << "[" << index + 1 << "/" << datasetPaths.size() << "] "
              << datasetPath << '\n';
    try {
      benchmarks.push_back(runDataset(options, datasetPath));
    } catch (const std::exception& exception) {
      std::cerr << "Skipping " << datasetPath << ": " << exception.what()
                << '\n';
    }
  }
  if (benchmarks.empty()) {
    throw std::runtime_error("No datasets could be benchmarked");
  }

  if (options.csvPath) writeCsv(benchmarks, *options.csvPath);
  if (options.benchmarkActionJsonPath) {
    writeBenchmarkActionJson(benchmarks, *options.benchmarkActionJsonPath);
  }
  printSummary(options, benchmarks);
  return 0;
}
