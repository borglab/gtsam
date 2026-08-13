/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeShonanInitialization.cpp
 * @brief   Compare Shonan with the general QCQP/BM staircase.
 * @date    August 2026
 * @author  Frank Dellaert
 *
 * The before path uses chordal initialization and the default PCG solver. The
 * after paths use FAST-Sync with the default PCG solver or direct LM solves.
 * The BM paths convert the same FAST-Sync estimate into matrix QCQP values and
 * use the generic augmented-Lagrangian staircase and certificate.
 */

#include <gtsam/base/DSFMap.h>
#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>
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
#include <numeric>
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
const string kBmDefault = "BM/ALM default";
const string kBmTuned = "BM/ALM tuned";
const string kBmAggressive = "BM/ALM Aggressive";
const string kBmBcl = "BM/ALM BCL";

const vector<string>& allBenchmarkMethods() {
  static const vector<string> methods{kBefore,       kAfterPcg, kAfterLm,
                                      kBmAggressive, kBmTuned,  kBmBcl};
  return methods;
}

const std::map<string, string>& methodTokens() {
  static const std::map<string, string> tokens{
      {"before", kBefore},        {"shonan-pcg", kAfterPcg},
      {"shonan-lm", kAfterLm},    {"bm-alm-default", kBmDefault},
      {"bm-alm-tuned", kBmTuned}, {"bm-alm-aggressive", kBmAggressive},
      {"bm-alm-bcl", kBmBcl},
  };
  return tokens;
}

struct Options {
  string dataset = "pose3example-grid.txt";
  optional<string> g2oDirectory;
  size_t largestFiles = 0;
  size_t warmups = 0;
  size_t repetitions = 1;
  double isotropicSigma = 0.0;
  vector<string> methods = allBenchmarkMethods();
  double bmInitialMu = 1.0;
  double bmMuIncrease = 2.0;
  size_t bmMaxIterations = 50;
  double bmViolationTolerance = 1e-8;
  double bmCostTolerance = 1e-10;
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
  NonlinearFactorGraph bmGraph;
  gtsam::Ordering fastSyncOrdering;
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
  double qcqpBuildSeconds = std::numeric_limits<double>::quiet_NaN();
  double localSolveSeconds = std::numeric_limits<double>::quiet_NaN();
  double certificateSeconds = std::numeric_limits<double>::quiet_NaN();
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
  optional<gtsam::timing::TimingSummary> qcqpBuild;
  optional<gtsam::timing::TimingSummary> localSolve;
  optional<gtsam::timing::TimingSummary> certificate;
  optional<gtsam::timing::TimingSummary> initialCost;
  optional<gtsam::timing::TimingSummary> finalCost;
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
         "  --methods LIST                  Comma-separated: before, "
         "shonan-pcg, shonan-lm, bm-alm-default, bm-alm-tuned, "
         "bm-alm-aggressive, bm-alm-bcl\n"
         "  --bm-initial-mu VALUE           Tuned ALM initial equality penalty "
         "(default: 1)\n"
         "  --bm-mu-increase VALUE          Tuned ALM penalty growth "
         "(default: 2)\n"
         "  --bm-max-iterations N           Tuned ALM outer iteration cap "
         "(default: 50)\n"
         "  --bm-violation-tol VALUE        Tuned ALM absolute/relative "
         "violation tolerance (default: 1e-8)\n"
         "  --bm-cost-tol VALUE             Tuned ALM absolute/relative cost "
         "tolerance (default: 1e-10)\n"
         "  --csv FILE                      Write individual trials in "
         "seconds\n"
         "  --benchmark-action-json FILE    Write mean metrics for Benchmark "
         "Action\n"
         "  --help                          Show this message\n"
         "\nSpecify at most one of dataset, --dataset, and --g2o-directory. "
         "The console table reports rounded integer milliseconds.\n"
         "Both staircases run from p=3 through p=10. "
         "Disconnected inputs are reduced to their largest component.\n"
         "\nBefore: chordal initialization with PCG.\n"
         "After/PCG: FAST-Sync initialization with PCG.\n"
         "After/LM:  FAST-Sync initialization with direct LM solves.\n"
         "BM/ALM default: FAST-Sync, QCQP conversion, and default ALM.\n"
         "BM/ALM tuned: FAST-Sync, QCQP conversion, and configured "
         "Aggressive ALM.\n"
         "BM/ALM Aggressive: FAST-Sync, QCQP conversion, and explicit "
         "Aggressive ALM defaults.\n"
         "BM/ALM BCL: FAST-Sync, QCQP conversion, and explicit BCL "
         "defaults.\n"
         "\nExample:\n"
         "  timeShonanInitialization --g2o-directory datasets/yfcc2 "
         "--largest 5 --repeats 1 --csv shonan.csv\n";
}

optional<string> nonemptyPath(const string& path) {
  return path.empty() ? optional<string>() : optional<string>(path);
}

vector<string> parseMethodList(const string& list) {
  if (list.empty()) return allBenchmarkMethods();
  vector<string> methods;
  std::istringstream stream(list);
  string token;
  while (std::getline(stream, token, ',')) {
    const auto method = methodTokens().find(token);
    if (method == methodTokens().end()) {
      throw std::invalid_argument("Unknown --methods token: " + token);
    }
    if (std::find(methods.begin(), methods.end(), method->second) ==
        methods.end()) {
      methods.push_back(method->second);
    }
  }
  if (methods.empty()) throw std::invalid_argument("--methods is empty");
  return methods;
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
  options.methods = parseMethodList(arguments.stringValue("--methods", ""));
  options.bmInitialMu = arguments.doubleValue("--bm-initial-mu", 1.0);
  options.bmMuIncrease = arguments.doubleValue("--bm-mu-increase", 2.0);
  options.bmMaxIterations = arguments.sizeValue("--bm-max-iterations", 50);
  options.bmViolationTolerance =
      arguments.doubleValue("--bm-violation-tol", 1e-8);
  options.bmCostTolerance = arguments.doubleValue("--bm-cost-tol", 1e-10);
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
  if (options.bmInitialMu <= 0.0 || options.bmMuIncrease <= 1.0 ||
      options.bmMaxIterations == 0 || options.bmViolationTolerance <= 0.0 ||
      options.bmCostTolerance <= 0.0) {
    throw std::invalid_argument(
        "BM penalties, iteration count, and tolerances must be positive; "
        "--bm-mu-increase must be greater than 1");
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
    context.bmGraph.emplace_shared<gtsam::FrobeniusBetweenFactor<Rot3>>(
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

  // Precompute FastSync's default METIS ordering so ordering construction is
  // consistently excluded from every measured initialization.
  context.fastSyncOrdering =
      gtsam::Ordering::Create(gtsam::Ordering::METIS, context.bmGraph);
  return context;
}

bool isBmMethod(const string& method) {
  return method == kBmDefault || method == kBmTuned ||
         method == kBmAggressive || method == kBmBcl;
}

gtsam::RiemannianStaircaseParams bmParameters(const string& method,
                                              const Options& options) {
  gtsam::RiemannianStaircaseParams parameters;
  parameters.pMin = kMinimumRank;
  parameters.pMax = kMaximumRank;
  parameters.eta = -kOptimalityThreshold;
  if (method == kBmAggressive || method == kBmTuned) {
    parameters.almParams->updatePolicy =
        gtsam::AugmentedLagrangianUpdatePolicy::Aggressive;
  } else if (method == kBmBcl) {
    parameters.almParams->updatePolicy =
        gtsam::AugmentedLagrangianUpdatePolicy::BCL;
  }
  if (method == kBmTuned) {
    parameters.almParams->initialMuEq = options.bmInitialMu;
    parameters.almParams->muEqIncreaseRate = options.bmMuIncrease;
    parameters.almParams->maxIterations = options.bmMaxIterations;
    parameters.almParams->absoluteViolationTolerance =
        options.bmViolationTolerance;
    parameters.almParams->relativeViolationTolerance =
        options.bmViolationTolerance;
    parameters.almParams->absoluteCostTolerance = options.bmCostTolerance;
    parameters.almParams->relativeCostTolerance = options.bmCostTolerance;
  }
  return parameters;
}

ShonanAveraging3::Parameters shonanParameters(const string& method,
                                              const BenchmarkContext& context) {
  gtsam::LevenbergMarquardtParams lmParameters =
      gtsam::LevenbergMarquardtParams::CeresDefaults();
  lmParameters.setOrderingType("COLAMD");
  const string linearSolver = method == kAfterLm ? "CHOLESKY" : "JACOBI";
  ShonanAveraging3::Parameters parameters(lmParameters, linearSolver,
                                          kOptimalityThreshold);
  if (method == kAfterLm && parameters.lm.requiresOrdering()) {
    parameters.lm.setOrdering(context.fastSyncOrdering);
  }
  return parameters;
}

Values qcqpInitialValues(const Values& rotations) {
  Values initial;
  for (const auto& [key, rotation] : rotations.extract<Rot3>()) {
    gtsam::InsertQcqpValue<Rot3, 3>(key, rotation, &initial);
  }
  return initial;
}

void alignBlockDeterminants(Values* values) {
  size_t negative = 0, blocks = 0;
  for (const auto& [key, matrix] : values->extract<gtsam::Matrix>()) {
    (void)key;
    if (matrix.rows() != 3 || matrix.cols() != 3) continue;
    ++blocks;
    if (matrix.determinant() < 0.0) ++negative;
  }
  if (blocks == 0 || negative <= blocks / 2) return;
  for (const auto& [key, matrix] : values->extract<gtsam::Matrix>()) {
    if (matrix.rows() != 3 || matrix.cols() != 3) continue;
    gtsam::Matrix aligned = matrix;
    aligned.col(2) *= -1.0;
    values->update(key, aligned);
  }
}

Values extractRoundedRotations(const gtsam::RiemannianStaircaseResult& result) {
  if (!result.hasRoundedSolution()) return Values();
  Values rounded = result.roundedValues();
  alignBlockDeterminants(&rounded);
  Values rotations;
  for (const auto& [key, rotation] :
       gtsam::ExtractQcqpValues<Rot3, 3>(rounded)) {
    rotations.insert(key, rotation);
  }
  return rotations;
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
                     const BenchmarkContext& context, const Options& options) {
  TrialResult result;
  result.dataset = context.datasetPath;
  result.initializer = initializer;
  result.rotations = context.bmGraph.keys().size();
  result.measurements = context.measurements.size();
  result.runIndex = runIndex;

  Values initialRotations, initialQcqp;
  auto start = std::chrono::steady_clock::now();
  try {
    if (initializer == kBefore) {
      initialRotations =
          gtsam::InitializePose3::initializeOrientations(context.chordalGraph);
    } else {
      initialRotations =
          gtsam::fastSync<Rot3>(context.bmGraph, context.fastSyncOrdering);
    }
    if (isBmMethod(initializer)) {
      initialQcqp = qcqpInitialValues(initialRotations);
    }
  } catch (const std::exception& exception) {
    result.initializationSeconds = elapsedSeconds(start);
    result.error = string("initialization: ") + exception.what();
    return result;
  }
  result.initializationSeconds = elapsedSeconds(start);
  result.initialCost = context.bmGraph.error(initialRotations);

  if (isBmMethod(initializer)) {
    start = std::chrono::steady_clock::now();
    try {
      const gtsam::RiemannianStaircaseParams parameters =
          bmParameters(initializer, options);
      const gtsam::RiemannianStaircaseOptimizer optimizer(
          context.bmGraph, initialQcqp, parameters);
      const gtsam::RiemannianStaircaseResult bmResult = optimizer.optimize();
      result.minimumEigenvalue = bmResult.minEigenvalue;
      result.rankReached = bmResult.finalRank;
      result.qcqpBuildSeconds =
          std::accumulate(bmResult.qcqpBuildTimePerLevel.begin(),
                          bmResult.qcqpBuildTimePerLevel.end(), 0.0);
      result.localSolveSeconds =
          std::accumulate(bmResult.nlpTimePerLevel.begin(),
                          bmResult.nlpTimePerLevel.end(), 0.0);
      result.certificateSeconds =
          std::accumulate(bmResult.verifyTimePerLevel.begin(),
                          bmResult.verifyTimePerLevel.end(), 0.0);

      const Values rotations = extractRoundedRotations(bmResult);
      if (!rotations.empty()) {
        result.finalCost = context.bmGraph.error(rotations);
      }
      result.success = bmResult.certified &&
                       rotations.size() == result.rotations &&
                       std::isfinite(result.finalCost);
      if (!bmResult.certified) {
        result.error = "BM did not certify";
      } else if (rotations.size() != result.rotations) {
        result.error = "BM rounded solution did not contain every rotation";
      } else if (!std::isfinite(result.finalCost)) {
        result.error = "BM rounded objective is not finite";
      }
      result.solveSeconds = elapsedSeconds(start);
    } catch (const std::exception& exception) {
      result.solveSeconds = elapsedSeconds(start);
      result.error = string("BM solve: ") + exception.what();
    }
    return result;
  }

  start = std::chrono::steady_clock::now();
  try {
    const ShonanAveraging3 shonan(context.measurements,
                                  shonanParameters(initializer, context));
    const StaircaseResult solution =
        runStaircase(shonan, initialRotations, &result.rankReached);
    result.finalCost = context.bmGraph.error(solution.values);
    result.minimumEigenvalue = solution.minimumEigenvalue;
    result.rankReached = solution.rankReached;
    result.success = true;
    result.solveSeconds = elapsedSeconds(start);
  } catch (const std::exception& exception) {
    result.solveSeconds = elapsedSeconds(start);
    result.error = string("solve: ") + exception.what();
  }
  return result;
}

void runMethods(size_t runIndex, const BenchmarkContext& context,
                const Options& options, vector<TrialResult>* results) {
  for (size_t offset = 0; offset < options.methods.size(); ++offset) {
    const string& initializer =
        options.methods[(runIndex + offset) % options.methods.size()];
    results->push_back(runTrial(initializer, runIndex, context, options));
  }
}

optional<MethodSummary> summarizeMethod(const vector<TrialResult>& results,
                                        const string& initializer) {
  vector<double> initialization;
  vector<double> solve;
  vector<double> total;
  vector<double> qcqpBuild;
  vector<double> localSolve;
  vector<double> certificate;
  vector<double> initialCost;
  vector<double> finalCost;
  vector<size_t> ranks;
  size_t successes = 0;
  size_t failures = 0;
  for (const TrialResult& result : results) {
    if (result.initializer != initializer) continue;
    initialization.push_back(result.initializationSeconds);
    solve.push_back(result.solveSeconds);
    total.push_back(result.totalSeconds());
    if (std::isfinite(result.qcqpBuildSeconds)) {
      qcqpBuild.push_back(result.qcqpBuildSeconds);
    }
    if (std::isfinite(result.localSolveSeconds)) {
      localSolve.push_back(result.localSolveSeconds);
    }
    if (std::isfinite(result.certificateSeconds)) {
      certificate.push_back(result.certificateSeconds);
    }
    if (std::isfinite(result.initialCost)) {
      initialCost.push_back(result.initialCost);
    }
    if (std::isfinite(result.finalCost)) finalCost.push_back(result.finalCost);
    if (result.rankReached > 0) ranks.push_back(result.rankReached);
    result.success ? ++successes : ++failures;
  }
  if (initialization.empty()) return std::nullopt;
  const auto summarize = [](const vector<double>& samples) {
    return gtsam::timing::summarizeSamples(
        samples, gtsam::timing::MedianPolicy::kAverageMiddle);
  };
  const auto optionalSummary = [&summarize](const vector<double>& samples)
      -> optional<gtsam::timing::TimingSummary> {
    return samples.empty()
               ? std::nullopt
               : optional<gtsam::timing::TimingSummary>(summarize(samples));
  };
  return MethodSummary{
      summarize(initialization),
      summarize(solve),
      summarize(total),
      optionalSummary(qcqpBuild),
      optionalSummary(localSolve),
      optionalSummary(certificate),
      optionalSummary(initialCost),
      optionalSummary(finalCost),
      ranks.empty() ? 0 : *std::min_element(ranks.begin(), ranks.end()),
      ranks.empty() ? 0 : *std::max_element(ranks.begin(), ranks.end()),
      successes,
      failures};
}

DatasetBenchmark runDataset(const Options& options, const string& datasetPath) {
  const BenchmarkContext context = buildContext(options, datasetPath);
  DatasetBenchmark benchmark;
  benchmark.path = datasetPath;
  benchmark.rawRotations = context.rawRotations;
  benchmark.rawMeasurements = context.rawMeasurements;
  benchmark.components = context.components;
  benchmark.rotations = context.bmGraph.keys().size();
  benchmark.measurements = context.measurements.size();

  vector<TrialResult> discardedWarmups;
  for (size_t runIndex = 0; runIndex < options.warmups; ++runIndex) {
    runMethods(runIndex, context, options, &discardedWarmups);
  }

  benchmark.results.reserve(options.methods.size() * options.repetitions);
  for (size_t runIndex = 0; runIndex < options.repetitions; ++runIndex) {
    runMethods(runIndex, context, options, &benchmark.results);
  }
  return benchmark;
}

void writeCsv(const vector<DatasetBenchmark>& benchmarks, const string& path) {
  std::ofstream output = gtsam::timing::openOutputFile(path);
  output << "dataset,initializer,raw_rotations,raw_measurements,components,"
            "rotations,measurements,run_index,success,p_reached,"
            "initialization_seconds,solve_seconds,total_seconds,"
            "qcqp_build_seconds,local_solve_seconds,certificate_seconds,"
            "initial_cost,final_cost,minimum_eigenvalue,error\n";
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
             << ',' << result.qcqpBuildSeconds << ','
             << result.localSolveSeconds << ',' << result.certificateSeconds
             << ',' << result.initialCost << ',' << result.finalCost << ','
             << result.minimumEigenvalue << ',' << std::quoted(result.error)
             << '\n';
    }
  }
}

void writeBenchmarkActionJson(const vector<DatasetBenchmark>& benchmarks,
                              const Options& options, const string& path) {
  vector<gtsam::timing::BenchmarkMetric> metrics;
  for (const DatasetBenchmark& benchmark : benchmarks) {
    const string prefix =
        "timeShonanInitialization/" +
        std::filesystem::path(benchmark.path).filename().string() + "/";
    for (const string& initializer : options.methods) {
      const auto summary = summarizeMethod(benchmark.results, initializer);
      if (!summary) continue;
      metrics.push_back({prefix + initializer + "/initialization", "s",
                         summary->initialization.mean});
      metrics.push_back(
          {prefix + initializer + "/solve", "s", summary->solve.mean});
      metrics.push_back(
          {prefix + initializer + "/total", "s", summary->total.mean});
      if (summary->qcqpBuild) {
        metrics.push_back({prefix + initializer + "/qcqp_build", "s",
                           summary->qcqpBuild->mean});
      }
      if (summary->localSolve) {
        metrics.push_back({prefix + initializer + "/local_solve", "s",
                           summary->localSolve->mean});
      }
      if (summary->certificate) {
        metrics.push_back({prefix + initializer + "/certificate", "s",
                           summary->certificate->mean});
      }
      metrics.push_back({prefix + initializer + "/rank_reached", "rank",
                         static_cast<double>(summary->maximumRank)});
      metrics.push_back(
          {prefix + initializer + "/successes", "trials",
           static_cast<double>(summary->successes)});
      if (summary->finalCost) {
        metrics.push_back({prefix + initializer + "/final_cost", "cost",
                           summary->finalCost->mean});
      }
    }
  }
  gtsam::timing::writeBenchmarkActionMetrics(path, metrics);
}

string rankText(const MethodSummary& summary) {
  if (summary.maximumRank == 0) return "?";
  if (summary.minimumRank == summary.maximumRank) {
    return std::to_string(summary.minimumRank);
  }
  return std::to_string(summary.minimumRank) + "-" +
         std::to_string(summary.maximumRank);
}

string costText(double cost) {
  std::ostringstream stream;
  stream << std::scientific << std::setprecision(3) << cost;
  return stream.str();
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
  std::cout << "Shonan versus generic QCQP/BM benchmark\n"
            << "p_min=" << kMinimumRank << " p_max=" << kMaximumRank
            << " warmups=" << options.warmups
            << " repeats=" << options.repetitions;
  if (options.isotropicSigma > 0.0) {
    std::cout << " isotropic_sigma=" << options.isotropicSigma;
  }
  if (std::find(options.methods.begin(), options.methods.end(), kBmTuned) !=
      options.methods.end()) {
    std::cout << " bm_tuned={initial_mu=" << options.bmInitialMu
              << ",mu_increase=" << options.bmMuIncrease
              << ",max_iterations=" << options.bmMaxIterations
              << ",violation_tol=" << options.bmViolationTolerance
              << ",cost_tol=" << options.bmCostTolerance << "}";
  }
  std::cout << "\n\n"
            << "Times are init/solve/total milliseconds over every attempt, "
               "including failures. Dataset parsing, graph construction, and "
               "the shared FAST-Sync ordering are excluded.\n\n"
            << "| Dataset";
  for (const string& method : options.methods) std::cout << " | " << method;
  std::cout << " |\n|---";
  for (size_t i = 0; i < options.methods.size(); ++i) std::cout << "|---";
  std::cout << "|\n";
  for (const DatasetBenchmark& benchmark : benchmarks) {
    const string dataset =
        std::filesystem::path(benchmark.path).filename().string();
    std::cout << "| " << dataset;
    for (const string& initializer : options.methods) {
      const auto summary = summarizeMethod(benchmark.results, initializer);
      std::cout << " | ";
      if (!summary) {
        std::cout << "not run";
      } else {
        std::cout << milliseconds(summary->initialization.mean) << "/"
                  << milliseconds(summary->solve.mean) << "/"
                  << milliseconds(summary->total.mean)
                  << " ms, p=" << rankText(*summary) << ", "
                  << summary->successes << "/"
                  << summary->successes + summary->failures;
        if (summary->finalCost) {
          std::cout << ", cost=" << costText(summary->finalCost->mean);
        }
        if (summary->failures > 0) {
          for (const TrialResult& result : benchmark.results) {
            if (result.initializer == initializer && !result.success &&
                !result.error.empty()) {
              std::cout << ", error=" << compactError(result.error);
              break;
            }
          }
        }
      }
    }
    std::cout << " |\n";
  }

  bool printedBreakdownHeader = false;
  for (const DatasetBenchmark& benchmark : benchmarks) {
    const string dataset =
        std::filesystem::path(benchmark.path).filename().string();
    for (const string& method : options.methods) {
      if (!isBmMethod(method)) continue;
      const auto summary = summarizeMethod(benchmark.results, method);
      if (!summary || !summary->qcqpBuild || !summary->localSolve ||
          !summary->certificate) {
        continue;
      }
      if (!printedBreakdownHeader) {
        std::cout << "\nBM phase times are QCQP build/local solve/certificate "
                     "milliseconds.\n\n"
                  << "| Dataset | Method | Build/Local/Certificate |\n"
                  << "|---|---|---|\n";
        printedBreakdownHeader = true;
      }
      std::cout << "| " << dataset << " | " << method << " | "
                << milliseconds(summary->qcqpBuild->mean) << "/"
                << milliseconds(summary->localSolve->mean) << "/"
                << milliseconds(summary->certificate->mean) << " ms |\n";
    }
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
    writeBenchmarkActionJson(benchmarks, options,
                             *options.benchmarkActionJsonPath);
  }
  printSummary(options, benchmarks);
  return 0;
}
