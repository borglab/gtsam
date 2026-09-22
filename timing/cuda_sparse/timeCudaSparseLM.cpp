/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCudaSparseLM.cpp
 * @brief   time the general CUDA sparse Levenberg-Marquardt optimizer
 * @author  Ruogu Li
 * @date    Jul 25, 2026
 */

#include "../timeSFMBAL.h"

#include <cuda_runtime_api.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
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

#ifndef GTSAM_BENCHMARK_BUILD_TYPE
#define GTSAM_BENCHMARK_BUILD_TYPE "unknown"
#endif

namespace {

namespace bal = gtsam::timing::bal;

using Clock = std::chrono::steady_clock;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Pose2;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::cuda::SparseLevenbergMarquardtOptimizer;
using gtsam::cuda::SparseLevenbergMarquardtParams;
using gtsam::cuda::SparseLevenbergMarquardtResult;
using gtsam::cuda::SparseLevenbergMarquardtAttemptRecord;
using gtsam::cuda::SparseLevenbergMarquardtBackend;
using gtsam::cuda::SparseLevenbergMarquardtStageTimings;
using gtsam::cuda::SparseLevenbergMarquardtSystemSize;
using gtsam::cuda::SparseLevenbergMarquardtTerminationReason;
using gtsam::cuda::SparseLevenbergMarquardtTransferCounts;
using gtsam::cuda::LinearSolveStats;

constexpr double kObjectiveTolerance = 1e-8;

#define FOR_EACH_TIMING_FIELD(FIELD)               \
  FIELD(totalWall)                                 \
  FIELD(initialError)                              \
  FIELD(plan)                                      \
  FIELD(persistentSetupWall)                       \
  FIELD(deviceInitializeWall)                      \
  FIELD(patternH2d)                                \
  FIELD(structureSetup)                            \
  FIELD(setupD2h)                                  \
  FIELD(hostZero)                                  \
  FIELD(factorLinearizationAndPackingWall)         \
  FIELD(factorLinearizationCpuSum)                 \
  FIELD(csrPackingCpuSum)                          \
  FIELD(numericH2d)                                \
  FIELD(transposeUpdate)                           \
  FIELD(normalJtJ)                                 \
  FIELD(normalJtb)                                 \
  FIELD(diagonalExtraction)                        \
  FIELD(oldModelError)                             \
  FIELD(dampingPreparation)                        \
  FIELD(dampingApplication)                        \
  FIELD(cudssAnalysis)                             \
  FIELD(cudssFactorAndSolve)                       \
  FIELD(cudssDataInfoBoundaryWall)                 \
  FIELD(pcgPreconditionerBuild)                    \
  FIELD(pcgSolve)                                  \
  FIELD(pcgD2h)                                    \
  FIELD(newModelError)                             \
  FIELD(attemptD2h)                                \
  FIELD(attemptHostBuild)                          \
  FIELD(retract)                                   \
  FIELD(nonlinearTrialError)

struct RunOptions {
  size_t warmups = 1;
  size_t repeats = 5;
  std::vector<std::string> datasets{"bal16", "bal135", "pose2", "pose3"};
  std::string jsonPath = "cuda-sparse-lm-benchmark.json";
  std::string csvPath = "cuda-sparse-lm-benchmark.csv";
  std::string dataDirectory;
#if GTSAM_ENABLE_CUDSS
  std::string gpuSolver = "cudss";
  std::string configuration = "cudss-auto";
#else
  std::string gpuSolver = "pcg";
  std::string configuration = "pcg";
#endif
  std::string ordering = "auto";
  std::string outputFormat = "text";
  bool listConfigurations = false;
  bool dryRun = false;
  bool allCudaConfigurations = false;
  double pcgTolerance = 0.0;   // 0 keeps the library default
  size_t pcgMaxIterations = 0; // 0 keeps the library default
  bool pose2FastSync = false;
  bool pose2UpstreamSettings = false;
  // Relative CPU/GPU final-objective tolerance. The strict default assumes a
  // direct solver; inexact PCG takes a different LM trajectory and needs an
  // explicit, recorded loosening.
  double objectiveTolerance = kObjectiveTolerance;
  bool help = false;
};

struct SummaryStatistics {
  double median = 0.0;
  double mean = 0.0;
  double standardDeviation = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
};

void ApplyMatrixConfigurationDefaults(RunOptions* options) {
  if (!options || options->gpuSolver != "pcg") return;
  if (options->pcgTolerance == 0.0) options->pcgTolerance = 1e-6;
  if (options->pcgMaxIterations == 0) options->pcgMaxIterations = 5000;
  if (options->objectiveTolerance == kObjectiveTolerance) {
    options->objectiveTolerance = 1e-3;
  }
}

enum class WorkloadKind { Bal, Pose2, Pose3, Stereo };

struct WorkloadSpec {
  std::string name;
  WorkloadKind kind;
  std::string filename;
  std::string exampleDataName;
  std::string poseFilename;
  std::string calibrationFilename;
};

struct LoadedWorkload {
  WorkloadSpec spec;
  std::string path;
  NonlinearFactorGraph graph;
  Values initial;
  std::optional<Ordering> cpuOrdering;
  bool pose2UpstreamSettings = false;
  double initializationWall = 0.0;
  double initialError = 0.0;
};

struct StereoWorkloadData {
  NonlinearFactorGraph graph;
  Values initial;
  Ordering ordering;
};

StereoWorkloadData BuildStereoWorkload(std::istream& calibrationInput,
                                       std::istream& poseInput,
                                       std::istream& factorInput);

struct RawRun {
  std::string backend;
  size_t repetition = 0;
  double externalWall = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  size_t outerLinearizations = 0;
  size_t lambdaAttempts = 0;
  size_t acceptedSteps = 0;
  size_t cudssAnalyses = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgSolves = 0;
  size_t pcgMaxIterationHits = 0;
  std::string termination;
  double finalLambda = 0.0;
  SparseLevenbergMarquardtSystemSize systemSize;
  SparseLevenbergMarquardtTransferCounts transfers;
  SparseLevenbergMarquardtStageTimings timings;
  LinearSolveStats linearSolveStats;
  std::vector<SparseLevenbergMarquardtAttemptRecord> attempts;
};

void WriteRawRuns(std::ostream& output, const std::vector<RawRun>& runs);

struct TimingField {
  const char* name;
  double SparseLevenbergMarquardtStageTimings::*member;
};

struct HardwareMetadata {
  std::string gpuName;
  size_t gpuMemoryBytes = 0;
  int computeMajor = 0;
  int computeMinor = 0;
  int cudaRuntimeVersion = 0;
  int cudaDriverApiVersion = 0;
  int compiledCudaVersion = CUDART_VERSION;
  std::string nvidiaDriverVersion;
  std::string gitCommit;
  bool gitDirty = false;
  std::string utcTimestamp;
};

struct WorkloadResult {
  std::string name;
  std::string path;
  size_t factors = 0;
  size_t values = 0;
  double initializationWall = 0.0;
  double initialError = 0.0;
  double referenceObjective = 0.0;
  double maximumObjectiveDifference = 0.0;
  std::vector<RawRun> cpuRuns;
  std::vector<RawRun> gpuRuns;
};

struct LegacyToroPose3Edge {
  size_t key1 = 0;
  size_t key2 = 0;
  Pose3 measured;
};

const std::vector<TimingField>& TimingFields() {
  static const std::vector<TimingField> fields{
#define MAKE_TIMING_FIELD(name) {#name, &SparseLevenbergMarquardtStageTimings::name},
      FOR_EACH_TIMING_FIELD(MAKE_TIMING_FIELD)
#undef MAKE_TIMING_FIELD
  };
  return fields;
}

std::string Usage() {
  return
      "Usage: timeCudaSparseLM [--warmups N] [--repeats N]\n"
      "  [--datasets bal16,bal135,pose2,pose3,stereo26,stereo77,stereo135]\n"
      "  [--data-dir DIR]\n"
      "  [--json FILE] [--csv FILE]\n"
      "  [--configuration cudss-auto|cudss-gtsam|pcg]\n"
      "  [--cuda-linear-solver cudss|pcg] [--ordering auto|gtsam]\n"
      "  [--output-format text|csv|json] [--list-configurations] [--dry-run]\n"
      "  [--all-cuda-configurations]\n"
      "  [--gpu-solver cudss|pcg] [--pcg-tol X] [--pcg-max-iters N]\n"
      "  [--pose2-fast-sync] [--pose2-upstream-settings]\n"
      "  [--objective-tol X] [--help]";
}

size_t ParseSize(const std::string& value, const char* option,
                 bool requirePositive) {
  if (value.empty() ||
      !std::all_of(value.begin(), value.end(),
                   [](const char character) {
                     return character >= '0' && character <= '9';
                   })) {
    throw std::invalid_argument(std::string(option) +
                                " requires an unsigned integer");
  }
  size_t parsedCharacters = 0;
  unsigned long long parsed = 0;
  try {
    parsed = std::stoull(value, &parsedCharacters);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) +
                                " requires an unsigned integer");
  }
  if (parsedCharacters != value.size() || (requirePositive && parsed == 0) ||
      parsed > std::numeric_limits<size_t>::max()) {
    throw std::invalid_argument(std::string(option) +
                                (requirePositive
                                     ? " requires a positive integer"
                                     : " requires an unsigned integer"));
  }
  return static_cast<size_t>(parsed);
}

std::vector<std::string> SplitDatasets(const std::string& value) {
  static const std::set<std::string> supported{
      "bal16",   "bal135",  "pose2",     "pose3",
      "stereo26", "stereo77", "stereo135"};
  if (value.empty() || value.back() == ',') {
    throw std::invalid_argument("--datasets contains an empty dataset");
  }
  std::vector<std::string> datasets;
  std::set<std::string> seen;
  std::stringstream stream(value);
  std::string dataset;
  while (std::getline(stream, dataset, ',')) {
    if (dataset.empty() || !supported.count(dataset)) {
      throw std::invalid_argument("unknown dataset: " + dataset);
    }
    if (!seen.insert(dataset).second) {
      throw std::invalid_argument("duplicate dataset: " + dataset);
    }
    datasets.push_back(dataset);
  }
  if (datasets.empty()) {
    throw std::invalid_argument("--datasets requires at least one dataset");
  }
  return datasets;
}

RunOptions ParseOptions(int argc, char** argv) {
  RunOptions options;
  const auto requireValue = [&](int* index, const char* option) {
    if (*index + 1 >= argc) {
      throw std::invalid_argument(std::string("missing value for ") + option);
    }
    return std::string(argv[++*index]);
  };

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--list-configurations") {
      options.listConfigurations = true;
    } else if (argument == "--dry-run") {
      options.dryRun = true;
    } else if (argument == "--all-cuda-configurations") {
      options.allCudaConfigurations = true;
    } else if (argument == "--help" || argument == "-h") {
      options.help = true;
    } else if (argument == "--warmups") {
      options.warmups =
          ParseSize(requireValue(&index, "--warmups"), "--warmups", false);
    } else if (argument == "--repeats") {
      options.repeats =
          ParseSize(requireValue(&index, "--repeats"), "--repeats", true);
    } else if (argument == "--datasets") {
      options.datasets = SplitDatasets(requireValue(&index, "--datasets"));
    } else if (argument == "--json") {
      options.jsonPath = requireValue(&index, "--json");
    } else if (argument == "--csv") {
      options.csvPath = requireValue(&index, "--csv");
    } else if (argument == "--data-dir") {
      options.dataDirectory = requireValue(&index, "--data-dir");
    } else if (argument == "--configuration") {
      options.configuration = requireValue(&index, "--configuration");
      if (options.configuration == "cudss-auto") {
        options.gpuSolver = "cudss";
        options.ordering = "auto";
      } else if (options.configuration == "cudss-gtsam") {
        options.gpuSolver = "cudss";
        options.ordering = "gtsam";
      } else if (options.configuration == "pcg") {
        options.gpuSolver = "pcg";
        options.ordering = "auto";
      } else {
        throw std::invalid_argument("unknown CUDA configuration: " +
                                    options.configuration);
      }
    } else if (argument == "--cuda-linear-solver" ||
               argument == "--gpu-solver") {
      options.gpuSolver = requireValue(&index, "--gpu-solver");
      if (options.gpuSolver != "cudss" && options.gpuSolver != "pcg") {
        throw std::invalid_argument("--gpu-solver requires cudss or pcg");
      }
    } else if (argument == "--ordering") {
      options.ordering = requireValue(&index, "--ordering");
      if (options.ordering != "auto" && options.ordering != "gtsam") {
        throw std::invalid_argument("--ordering requires auto or gtsam");
      }
    } else if (argument == "--output-format") {
      options.outputFormat = requireValue(&index, "--output-format");
      if (options.outputFormat != "text" && options.outputFormat != "csv" &&
          options.outputFormat != "json") {
        throw std::invalid_argument(
            "--output-format requires text, csv, or json");
      }
    } else if (argument == "--pcg-tol") {
      const std::string value = requireValue(&index, "--pcg-tol");
      size_t parsedCharacters = 0;
      try {
        options.pcgTolerance = std::stod(value, &parsedCharacters);
      } catch (const std::exception&) {
        throw std::invalid_argument("--pcg-tol requires a positive number");
      }
      if (parsedCharacters != value.size() || options.pcgTolerance <= 0.0 ||
          !std::isfinite(options.pcgTolerance)) {
        throw std::invalid_argument("--pcg-tol requires a positive number");
      }
    } else if (argument == "--pcg-max-iters") {
      options.pcgMaxIterations = ParseSize(
          requireValue(&index, "--pcg-max-iters"), "--pcg-max-iters", true);
    } else if (argument == "--pose2-fast-sync") {
      options.pose2FastSync = true;
    } else if (argument == "--pose2-upstream-settings") {
      options.pose2UpstreamSettings = true;
    } else if (argument == "--objective-tol") {
      const std::string value = requireValue(&index, "--objective-tol");
      size_t parsedCharacters = 0;
      try {
        options.objectiveTolerance = std::stod(value, &parsedCharacters);
      } catch (const std::exception&) {
        throw std::invalid_argument(
            "--objective-tol requires a positive number");
      }
      if (parsedCharacters != value.size() ||
          options.objectiveTolerance <= 0.0 ||
          !std::isfinite(options.objectiveTolerance)) {
        throw std::invalid_argument(
            "--objective-tol requires a positive number");
      }
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.gpuSolver == "pcg" && options.ordering != "auto") {
    throw std::invalid_argument(
        "GTSAM ordering is supported only by the cuDSS backend");
  }
  options.configuration =
      options.gpuSolver == "pcg"
          ? "pcg"
          : options.ordering == "gtsam" ? "cudss-gtsam" : "cudss-auto";
  ApplyMatrixConfigurationDefaults(&options);
  return options;
}

SummaryStatistics Summarize(const std::vector<double>& samples) {
  if (samples.empty()) {
    throw std::invalid_argument("cannot summarize an empty sample set");
  }
  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());

  SummaryStatistics result;
  const size_t middle = sorted.size() / 2;
  result.median = sorted.size() % 2 == 0
                      ? 0.5 * (sorted[middle - 1] + sorted[middle])
                      : sorted[middle];
  result.mean =
      std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
  double squaredDifference = 0.0;
  for (const double sample : sorted) {
    const double difference = sample - result.mean;
    squaredDifference += difference * difference;
  }
  result.standardDeviation =
      std::sqrt(squaredDifference / static_cast<double>(sorted.size()));
  result.minimum = sorted.front();
  result.maximum = sorted.back();
  return result;
}

void ValidateObjective(double reference, double candidate, double tolerance) {
  if (!std::isfinite(reference) || !std::isfinite(candidate) ||
      !std::isfinite(tolerance) || tolerance < 0.0) {
    throw std::invalid_argument(
        "objective values and tolerance must be finite and tolerance must be "
        "nonnegative");
  }
  const double allowed =
      tolerance * std::max({1.0, std::abs(reference), std::abs(candidate)});
  if (std::abs(reference - candidate) > allowed) {
    std::ostringstream message;
    message << std::setprecision(17)
            << "objective mismatch: reference=" << reference
            << ", candidate=" << candidate << ", difference="
            << std::abs(reference - candidate) << ", allowed=" << allowed;
    throw std::runtime_error(message.str());
  }
}

std::string CsvEscape(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
  std::string escaped{"\""};
  for (const char character : value) {
    if (character == '"') escaped.push_back('"');
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

std::string JsonEscape(const std::string& value) {
  std::string escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (character < 0x20) {
          const char digits[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped.push_back(digits[character >> 4]);
          escaped.push_back(digits[character & 0x0f]);
        } else {
          escaped.push_back(static_cast<char>(character));
        }
    }
  }
  return escaped;
}

std::optional<LegacyToroPose3Edge> ParseLegacyToroPose3Edge(
    const std::string& line) {
  std::istringstream stream(line);
  std::string tag;
  stream >> tag;
  if (tag != "EDGE3") return std::nullopt;

  LegacyToroPose3Edge edge;
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  if (!(stream >> edge.key1 >> edge.key2 >> x >> y >> z >> roll >> pitch >>
        yaw)) {
    throw std::runtime_error("malformed legacy TORO EDGE3 record");
  }
  std::string extra;
  if (stream >> extra) {
    throw std::runtime_error(
        "legacy TORO EDGE3 record unexpectedly contains a noise matrix");
  }
  edge.measured =
      Pose3(gtsam::Rot3::Ypr(yaw, pitch, roll), gtsam::Point3(x, y, z));
  return edge;
}


void ValidatePcgBenchmarkRun(const RawRun& run) {
  if (run.linearSolveStats.pcgMaxIterationHits != 0) {
    throw std::runtime_error(
        "PCG benchmark reached its iteration cap in one or more solves");
  }
  if (run.linearSolveStats.pcgBreakdownCount != 0 ||
      run.linearSolveStats.lastPcgBreakdown) {
    throw std::runtime_error("PCG benchmark encountered a solver breakdown");
  }
  if (run.linearSolveStats.solveCount == 0 ||
      !run.linearSolveStats.lastPcgConverged) {
    throw std::runtime_error(
        "PCG benchmark did not finish with a converged solve");
  }
}


const WorkloadSpec& LookupWorkload(const std::string& name) {
  static const std::map<std::string, WorkloadSpec> specifications{
      {"bal16",
       {"bal16", WorkloadKind::Bal, "dubrovnik-16-22106-pre.txt",
        "dubrovnik-16-22106-pre", "", ""}},
      {"bal135",
       {"bal135", WorkloadKind::Bal, "dubrovnik-135-90642-pre.txt",
        "dubrovnik-135-90642-pre", "", ""}},
      {"pose2",
       {"pose2", WorkloadKind::Pose2, "w10000.graph", "w10000", "", ""}},
      {"pose3",
       {"pose3", WorkloadKind::Pose3, "sphere_smallnoise.graph",
        "sphere_smallnoise", "", ""}},
      {"stereo26",
       {"stereo26", WorkloadKind::Stereo, "VO_stereo_factors_large.txt",
        "VO_stereo_factors_large", "VO_camera_poses_large.txt",
        "VO_calibration.txt"}},
      {"stereo77",
       {"stereo77", WorkloadKind::Stereo, "VO_stereo_factors00s.txt",
        "VO_stereo_factors00s", "VO_camera_poses00s.txt",
        "VO_calibration00s.txt"}},
      {"stereo135",
       {"stereo135", WorkloadKind::Stereo, "VO_stereo_factors00.txt",
        "VO_stereo_factors00", "VO_camera_poses00.txt",
        "VO_calibration00.txt"}},
  };
  return specifications.at(name);
}

std::string ResolvePath(const WorkloadSpec& specification,
                        const RunOptions& options) {
  if (!options.dataDirectory.empty()) {
    return (std::filesystem::path(options.dataDirectory) /
            specification.filename)
        .string();
  }
  return gtsam::findExampleDataFile(specification.exampleDataName);
}

std::string ResolveStereoAuxiliaryPath(const std::string& filename,
                                       const RunOptions& options) {
  if (!options.dataDirectory.empty()) {
    return (std::filesystem::path(options.dataDirectory) / filename).string();
  }
  return gtsam::findExampleDataFile(
      std::filesystem::path(filename).stem().string());
}

StereoWorkloadData BuildStereoWorkload(std::istream& calibrationInput,
                                       std::istream& poseInput,
                                       std::istream& factorInput) {
  double fx = 0.0;
  double fy = 0.0;
  double skew = 0.0;
  double principalX = 0.0;
  double principalY = 0.0;
  double baseline = 0.0;
  if (!(calibrationInput >> fx >> fy >> skew >> principalX >> principalY >>
        baseline)) {
    throw std::runtime_error("stereo calibration requires six scalars");
  }
  std::string trailingToken;
  if (calibrationInput >> trailingToken) {
    throw std::runtime_error("stereo calibration has trailing data");
  }
  for (const double value :
       {fx, fy, skew, principalX, principalY, baseline}) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("stereo calibration contains non-finite data");
    }
  }
  const auto calibration = std::make_shared<gtsam::Cal3_S2Stereo>(
      fx, fy, skew, principalX, principalY, baseline);

  StereoWorkloadData result;
  std::set<gtsam::Key> poseKeys;
  std::string line;
  size_t lineNumber = 0;
  while (std::getline(poseInput, line)) {
    ++lineNumber;
    std::istringstream row(line);
    long long poseIndex = -1;
    if (!(row >> poseIndex)) {
      if (row.eof()) continue;
      throw std::runtime_error("malformed stereo pose row " +
                               std::to_string(lineNumber));
    }
    if (poseIndex < 0) {
      throw std::runtime_error("negative stereo pose index at row " +
                               std::to_string(lineNumber));
    }
    gtsam::MatrixRowMajor matrix(4, 4);
    for (int index = 0; index < 16; ++index) {
      if (!(row >> matrix.data()[index]) ||
          !std::isfinite(matrix.data()[index])) {
        throw std::runtime_error("malformed stereo pose matrix at row " +
                                 std::to_string(lineNumber));
      }
    }
    if (row >> trailingToken) {
      throw std::runtime_error("stereo pose row has trailing data at row " +
                               std::to_string(lineNumber));
    }
    const gtsam::Key poseKey =
        gtsam::Symbol('x', static_cast<uint64_t>(poseIndex));
    if (!poseKeys.insert(poseKey).second) {
      throw std::runtime_error("duplicate stereo pose at row " +
                               std::to_string(lineNumber));
    }
    result.initial.insert(poseKey, Pose3(matrix));
  }
  if (poseInput.bad()) {
    throw std::runtime_error("failed while reading stereo poses");
  }
  if (poseKeys.empty()) {
    throw std::runtime_error("stereo pose input is empty");
  }

  const auto measurementModel = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  std::set<gtsam::Key> landmarkKeys;
  lineNumber = 0;
  while (std::getline(factorInput, line)) {
    ++lineNumber;
    std::istringstream row(line);
    long long poseIndex = -1;
    long long landmarkIndex = -1;
    double uLeft = 0.0;
    double uRight = 0.0;
    double v = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(row >> poseIndex)) {
      if (row.eof()) continue;
      throw std::runtime_error("malformed stereo factor row " +
                               std::to_string(lineNumber));
    }
    if (!(row >> landmarkIndex >> uLeft >> uRight >> v >> x >> y >> z) ||
        poseIndex < 0 || landmarkIndex < 0) {
      throw std::runtime_error("malformed stereo factor row " +
                               std::to_string(lineNumber));
    }
    for (const double value : {uLeft, uRight, v, x, y, z}) {
      if (!std::isfinite(value)) {
        throw std::runtime_error(
            "stereo factor contains non-finite data at row " +
            std::to_string(lineNumber));
      }
    }
    if (row >> trailingToken) {
      throw std::runtime_error("stereo factor row has trailing data at row " +
                               std::to_string(lineNumber));
    }

    const gtsam::Key poseKey =
        gtsam::Symbol('x', static_cast<uint64_t>(poseIndex));
    if (!result.initial.exists(poseKey)) {
      throw std::runtime_error("stereo factor references missing pose at row " +
                               std::to_string(lineNumber));
    }
    const gtsam::Key landmarkKey =
        gtsam::Symbol('l', static_cast<uint64_t>(landmarkIndex));
    result.graph.emplace_shared<
        gtsam::GenericStereoFactor<Pose3, gtsam::Point3>>(
        gtsam::StereoPoint2(uLeft, uRight, v), measurementModel, poseKey,
        landmarkKey, calibration);
    if (landmarkKeys.insert(landmarkKey).second) {
      const Pose3& cameraPose = result.initial.at<Pose3>(poseKey);
      result.initial.insert(
          landmarkKey,
          cameraPose.transformFrom(gtsam::Point3(x, y, z)));
    }
  }
  if (factorInput.bad()) {
    throw std::runtime_error("failed while reading stereo factors");
  }
  if (result.graph.empty() || landmarkKeys.empty()) {
    throw std::runtime_error("stereo factor input is empty");
  }

  const gtsam::Key firstPoseKey = *poseKeys.begin();
  result.graph.emplace_shared<gtsam::PriorFactor<Pose3>>(
      firstPoseKey, result.initial.at<Pose3>(firstPoseKey),
      gtsam::noiseModel::Unit::Create(6));
  result.ordering.reserve(landmarkKeys.size() + poseKeys.size());
  result.ordering.insert(result.ordering.end(), landmarkKeys.begin(),
                         landmarkKeys.end());
  result.ordering.insert(result.ordering.end(), poseKeys.begin(),
                         poseKeys.end());
  return result;
}

std::pair<NonlinearFactorGraph, Values> LoadLegacyToroPose3(
    const std::string& path) {
  Values initial;
  const std::map<size_t, Pose3> poses = gtsam::parseVariables<Pose3>(path);
  for (const auto& [key, pose] : poses) initial.insert(key, pose);

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open Pose3 dataset: " + path);
  }
  NonlinearFactorGraph graph;
  const auto model = gtsam::noiseModel::Unit::Create(6);
  std::string line;
  size_t lineNumber = 0;
  while (std::getline(input, line)) {
    ++lineNumber;
    try {
      const auto edge = ParseLegacyToroPose3Edge(line);
      if (!edge) continue;
      if (!initial.exists(edge->key1) || !initial.exists(edge->key2)) {
        throw std::runtime_error("edge references a missing vertex");
      }
      graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
          edge->key1, edge->key2, edge->measured, model);
    } catch (const std::exception& error) {
      throw std::runtime_error(path + ":" + std::to_string(lineNumber) +
                               ": " + error.what());
    }
  }
  if (poses.empty() || graph.empty()) {
    throw std::runtime_error("legacy TORO Pose3 dataset is empty: " + path);
  }
  return {std::move(graph), std::move(initial)};
}

LoadedWorkload LoadWorkload(const WorkloadSpec& specification,
                            const RunOptions& options) {
  LoadedWorkload workload;
  workload.spec = specification;
  workload.path = ResolvePath(specification, options);
  if (!std::filesystem::is_regular_file(workload.path)) {
    throw std::runtime_error("dataset does not exist: " + workload.path);
  }

  if (specification.kind == WorkloadKind::Bal) {
    const gtsam::SfmData data = gtsam::SfmData::FromBalFile(workload.path);
    const bal::BalBenchmarkConfig config;
    workload.graph = bal::buildGeneralSfmGraph(data, config);
    workload.initial = bal::buildGeneralSfmInitial(data);
    workload.cpuOrdering = bal::createSchurOrdering(data, false);
  } else if (specification.kind == WorkloadKind::Pose2) {
    const auto [graph, initial] = gtsam::load2D(workload.path);
    if (!graph || !initial || !initial->exists(0)) {
      throw std::runtime_error("Pose2 dataset is missing pose 0");
    }
    workload.graph = *graph;
    workload.initial = *initial;
    workload.pose2UpstreamSettings = options.pose2UpstreamSettings;
    gtsam::SharedNoiseModel priorModel;
    if (options.pose2UpstreamSettings) {
      priorModel = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(1e-6, 1e-6, 1e-8));
    } else {
      priorModel = gtsam::noiseModel::Unit::Create(3);
    }
    workload.graph.emplace_shared<gtsam::PriorFactor<Pose2>>(
        0, workload.initial.at<Pose2>(0), priorModel);
    if (options.pose2FastSync) {
      const auto start = Clock::now();
      workload.initial = gtsam::fastSync<Pose2>(workload.graph);
      workload.initializationWall =
          std::chrono::duration<double>(Clock::now() - start).count();
    }
  } else if (specification.kind == WorkloadKind::Pose3) {
    auto [graph, initial] = LoadLegacyToroPose3(workload.path);
    if (!initial.exists(0)) {
      throw std::runtime_error("Pose3 dataset is missing pose 0");
    }
    workload.graph = std::move(graph);
    workload.initial = std::move(initial);
    workload.graph.emplace_shared<gtsam::PriorFactor<Pose3>>(
        0, workload.initial.at<Pose3>(0), gtsam::noiseModel::Unit::Create(6));
  } else {
    const std::string posePath =
        ResolveStereoAuxiliaryPath(specification.poseFilename, options);
    const std::string calibrationPath = ResolveStereoAuxiliaryPath(
        specification.calibrationFilename, options);
    if (!std::filesystem::is_regular_file(posePath)) {
      throw std::runtime_error("stereo pose dataset does not exist: " +
                               posePath);
    }
    if (!std::filesystem::is_regular_file(calibrationPath)) {
      throw std::runtime_error("stereo calibration dataset does not exist: " +
                               calibrationPath);
    }
    std::ifstream calibrationInput(calibrationPath);
    std::ifstream poseInput(posePath);
    std::ifstream factorInput(workload.path);
    if (!calibrationInput || !poseInput || !factorInput) {
      throw std::runtime_error("cannot open stereo workload inputs: " +
                               specification.name);
    }
    StereoWorkloadData stereo =
        BuildStereoWorkload(calibrationInput, poseInput, factorInput);
    workload.graph = std::move(stereo.graph);
    workload.initial = std::move(stereo.initial);
    workload.cpuOrdering = std::move(stereo.ordering);
  }

  if (workload.graph.empty() || workload.initial.empty()) {
    throw std::runtime_error("loaded workload is empty: " + specification.name);
  }
  workload.initialError = workload.graph.error(workload.initial);
  if (!std::isfinite(workload.initialError)) {
    throw std::runtime_error("initial error is non-finite: " +
                             specification.name);
  }
  return workload;
}

SparseLevenbergMarquardtParams MakeGpuParams(
    const LoadedWorkload& workload, const RunOptions& options) {
  SparseLevenbergMarquardtParams params;
  if (workload.pose2UpstreamSettings) {
    params.maxIterations = 100;
  } else {
    LevenbergMarquardtParams::SetCeresDefaults(&params);
  }
  params.setVerbosityLM("SILENT");
  if (workload.spec.kind == WorkloadKind::Bal) {
    params.maxIterations = 20;
    params.relativeErrorTol = 0.01;
  }
  params.fallbackOnUnsupported = false;
  params.collectTiming = true;
  params.collectAttemptTrace = true;
  if (options.gpuSolver == "pcg") {
    params.linear.backend = gtsam::cuda::LinearSolverType::Pcg;
    if (options.pcgTolerance > 0.0) {
      params.pcg.relativeTolerance = options.pcgTolerance;
    }
    if (options.pcgMaxIterations > 0) {
      params.pcg.maxIterations = static_cast<int>(options.pcgMaxIterations);
    }
  } else if (options.ordering == "gtsam") {
    if (workload.cpuOrdering) {
      params.setOrdering(*workload.cpuOrdering);
    } else {
      params.setOrdering(
          Ordering::Create(Ordering::COLAMD, workload.graph));
    }
  }
  return params;
}

const char* TerminationName(SparseLevenbergMarquardtTerminationReason termination) {
  switch (termination) {
    case SparseLevenbergMarquardtTerminationReason::None:
      return "none";
    case SparseLevenbergMarquardtTerminationReason::ErrorThreshold:
      return "error_threshold";
    case SparseLevenbergMarquardtTerminationReason::Converged:
      return "converged";
    case SparseLevenbergMarquardtTerminationReason::MaxIterations:
      return "max_iterations";
    case SparseLevenbergMarquardtTerminationReason::SmallCostChange:
      return "small_cost_change";
    case SparseLevenbergMarquardtTerminationReason::LambdaUpperBound:
      return "lambda_upper_bound";
  }
  throw std::runtime_error("unknown CUDA sparse LM termination reason");
}

LevenbergMarquardtParams MakeCpuParams(const LoadedWorkload& workload) {
  // The CPU baseline is independent of the GPU solver selection.
  LevenbergMarquardtParams params = MakeGpuParams(workload, RunOptions{});
  if (workload.cpuOrdering) params.setOrdering(*workload.cpuOrdering);
  return params;
}

void ValidateRawRun(const RawRun& run) {
  if (!std::isfinite(run.externalWall) || run.externalWall < 0.0 ||
      !std::isfinite(run.initialError) || !std::isfinite(run.finalError)) {
    throw std::runtime_error(run.backend + " run produced non-finite results");
  }
  if (run.backend == "gpu") {
    if (!std::isfinite(run.finalLambda) || run.finalLambda < 0.0) {
      throw std::runtime_error("GPU run produced an invalid final lambda");
    }
    for (const TimingField& field : TimingFields()) {
      const double seconds = run.timings.*(field.member);
      if (!std::isfinite(seconds) || seconds < 0.0) {
        throw std::runtime_error(std::string("invalid GPU timing: ") +
                                 field.name);
      }
    }
    for (const SparseLevenbergMarquardtAttemptRecord& attempt : run.attempts) {
      if (!std::isfinite(attempt.lambda) ||
          !std::isfinite(attempt.linearizedChange) ||
          !std::isfinite(attempt.nonlinearChange) ||
          !std::isfinite(attempt.modelFidelity)) {
        throw std::runtime_error("GPU attempt trace contains non-finite data");
      }
    }
  }
}

RawRun RunCpu(const LoadedWorkload& workload, size_t repetition) {
  const LevenbergMarquardtParams params = MakeCpuParams(workload);
  const auto begin = Clock::now();
  LevenbergMarquardtOptimizer optimizer(workload.graph, workload.initial,
                                        params);
  (void)optimizer.optimize();
  const auto end = Clock::now();

  RawRun run;
  run.backend = "cpu";
  run.repetition = repetition;
  run.externalWall = std::chrono::duration<double>(end - begin).count();
  run.initialError = workload.initialError;
  run.finalError = optimizer.error();
  run.iterations = optimizer.iterations();
  ValidateRawRun(run);
  return run;
}

RawRun RunGpu(const LoadedWorkload& workload, size_t repetition,
              const RunOptions& options) {
  const SparseLevenbergMarquardtParams params =
      MakeGpuParams(workload, options);
  const auto begin = Clock::now();
  SparseLevenbergMarquardtOptimizer optimizer(workload.graph,
                                                  workload.initial, params);
  (void)optimizer.optimize();
  const auto end = Clock::now();
  const SparseLevenbergMarquardtResult& result = optimizer.result();

  if (result.backend != SparseLevenbergMarquardtBackend::Device) {
    throw std::runtime_error("GPU benchmark fell back to CPU: " +
                             result.fallbackDetail);
  }
  if (result.termination == SparseLevenbergMarquardtTerminationReason::None) {
    throw std::runtime_error("GPU benchmark has no termination reason");
  }

  RawRun run;
  run.backend = "gpu";
  run.repetition = repetition;
  run.externalWall = std::chrono::duration<double>(end - begin).count();
  run.initialError = result.initialError;
  run.finalError = result.finalError;
  run.iterations = result.iterations;
  run.outerLinearizations = result.outerLinearizations;
  run.lambdaAttempts = result.lambdaAttempts;
  run.acceptedSteps = result.acceptedSteps;
  run.cudssAnalyses = result.cudssAnalyses;
  run.pcgIterationsTotal = result.pcgIterationsTotal;
  run.pcgSolves = result.pcgSolves;
  run.pcgMaxIterationHits = result.pcgMaxIterationHits;
  run.termination = TerminationName(result.termination);
  run.finalLambda = result.finalLambda;
  run.systemSize = result.systemSize;
  run.transfers = result.transfers;
  run.timings = result.timings;
  run.linearSolveStats = result.linearSolveStats;
  run.attempts = result.attemptTrace;
  ValidateRawRun(run);
  if (options.gpuSolver == "pcg") ValidatePcgBenchmarkRun(run);
  return run;
}

std::vector<double> ExternalSamples(const std::vector<RawRun>& runs) {
  std::vector<double> samples;
  samples.reserve(runs.size());
  for (const RawRun& run : runs) samples.push_back(run.externalWall);
  return samples;
}

std::vector<double> TimingSamples(
    const std::vector<RawRun>& runs,
    double SparseLevenbergMarquardtStageTimings::*member) {
  std::vector<double> samples;
  samples.reserve(runs.size());
  for (const RawRun& run : runs) samples.push_back(run.timings.*member);
  return samples;
}

std::vector<double> TransferSamples(const std::vector<RawRun>& runs,
                                    size_t SparseLevenbergMarquardtTransferCounts::*member) {
  std::vector<double> samples;
  samples.reserve(runs.size());
  for (const RawRun& run : runs) {
    samples.push_back(static_cast<double>(run.transfers.*member));
  }
  return samples;
}

WorkloadResult BenchmarkWorkload(const LoadedWorkload& workload,
                                 const RunOptions& options, bool humanOutput) {
  if (humanOutput) {
    std::cout << "\nLoading complete: " << workload.spec.name << "\n"
            << "  path: " << workload.path << "\n"
            << "  factors: " << workload.graph.size()
            << ", values: " << workload.initial.size()
            << ", initialization: "
            << (options.pose2FastSync &&
                        workload.spec.kind == WorkloadKind::Pose2
                    ? "fast_sync"
                    : "raw")
            << " (" << workload.initializationWall << " s)"
            << ", initial error: " << std::setprecision(15)
              << workload.initialError << std::setprecision(9) << "\n";
  }

  for (size_t warmup = 0; warmup < options.warmups; ++warmup) {
    if (humanOutput) {
      std::cout << "  warm-up " << warmup << ": CPU" << std::flush;
    }
    (void)RunCpu(workload, warmup);
    if (humanOutput) std::cout << ", GPU" << std::flush;
    (void)RunGpu(workload, warmup, options);
    if (humanOutput) std::cout << " complete\n";
  }

  WorkloadResult result;
  result.name = workload.spec.name;
  result.path = workload.path;
  result.factors = workload.graph.size();
  result.values = workload.initial.size();
  result.initializationWall = workload.initializationWall;
  result.initialError = workload.initialError;
  result.cpuRuns.reserve(options.repeats);
  result.gpuRuns.reserve(options.repeats);

  bool haveReference = false;
  for (size_t repetition = 0; repetition < options.repeats; ++repetition) {
    RawRun cpu;
    RawRun gpu;
    if (repetition % 2 == 0) {
      cpu = RunCpu(workload, repetition);
      if (!haveReference) {
        result.referenceObjective = cpu.finalError;
        haveReference = true;
      }
      gpu = RunGpu(workload, repetition, options);
    } else {
      gpu = RunGpu(workload, repetition, options);
      cpu = RunCpu(workload, repetition);
    }

    // CPU repeatability keeps the strict tolerance; GPU parity uses the
    // configured tolerance so inexact PCG runs can be validated at a stated,
    // recorded bound.
    ValidateObjective(result.referenceObjective, cpu.finalError,
                      kObjectiveTolerance);
    ValidateObjective(result.referenceObjective, gpu.finalError,
                      options.objectiveTolerance);
    cpu.systemSize = gpu.systemSize;
    result.maximumObjectiveDifference =
        std::max({result.maximumObjectiveDifference,
                  std::abs(result.referenceObjective - cpu.finalError),
                  std::abs(result.referenceObjective - gpu.finalError)});
    result.cpuRuns.push_back(std::move(cpu));
    result.gpuRuns.push_back(std::move(gpu));
    if (humanOutput) {
      std::cout << "  repetition " << repetition << ": CPU "
                << result.cpuRuns.back().externalWall << " s, GPU "
                << result.gpuRuns.back().externalWall << " s\n";
    }
  }
  return result;
}

std::string RunCommand(const char* command) {
  std::string output;
  FILE* pipe = popen(command, "r");
  if (!pipe) return output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe)) output += buffer;
  (void)pclose(pipe);
  while (!output.empty() &&
         (output.back() == '\n' || output.back() == '\r')) {
    output.pop_back();
  }
  return output;
}

std::string UtcTimestamp() {
  const std::time_t current = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&current, &utc);
  char buffer[32];
  if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc) == 0) {
    throw std::runtime_error("failed to format UTC timestamp");
  }
  return buffer;
}

HardwareMetadata QueryHardware() {
  HardwareMetadata metadata;
  int device = 0;
  if (cudaGetDevice(&device) != cudaSuccess) {
    throw std::runtime_error("cudaGetDevice failed");
  }
  cudaDeviceProp properties{};
  if (cudaGetDeviceProperties(&properties, device) != cudaSuccess) {
    throw std::runtime_error("cudaGetDeviceProperties failed");
  }
  metadata.gpuName = properties.name;
  metadata.gpuMemoryBytes = properties.totalGlobalMem;
  metadata.computeMajor = properties.major;
  metadata.computeMinor = properties.minor;
  if (cudaRuntimeGetVersion(&metadata.cudaRuntimeVersion) != cudaSuccess ||
      cudaDriverGetVersion(&metadata.cudaDriverApiVersion) != cudaSuccess) {
    throw std::runtime_error("CUDA version query failed");
  }
  metadata.nvidiaDriverVersion = RunCommand(
      "nvidia-smi --query-gpu=driver_version --format=csv,noheader "
      "2>/dev/null");
  metadata.gitCommit = RunCommand("git rev-parse HEAD 2>/dev/null");
  metadata.gitDirty = !RunCommand("git status --porcelain 2>/dev/null").empty();
  metadata.utcTimestamp = UtcTimestamp();
  return metadata;
}

void WriteStatistics(std::ostream& output,
                     const SummaryStatistics& statistics) {
  output << "{\"median\":" << statistics.median
         << ",\"mean\":" << statistics.mean
         << ",\"standard_deviation\":" << statistics.standardDeviation
         << ",\"minimum\":" << statistics.minimum
         << ",\"maximum\":" << statistics.maximum << "}";
}

void WriteSystemSize(std::ostream& output,
                     const SparseLevenbergMarquardtSystemSize& size) {
  output << "{\"factors\":" << size.factors
         << ",\"jacobian_rows\":" << size.jacobianRows
         << ",\"jacobian_columns\":" << size.jacobianColumns
         << ",\"jacobian_nonzeros\":" << size.jacobianNonzeros
         << ",\"normal_nonzeros\":" << size.normalNonzeros << "}";
}

void WriteTransfers(std::ostream& output,
                    const SparseLevenbergMarquardtTransferCounts& transfers) {
  output << "{\"pattern_h2d_bytes\":" << transfers.patternH2dBytes
         << ",\"numeric_h2d_bytes\":" << transfers.numericH2dBytes
         << ",\"setup_d2h_bytes\":" << transfers.setupD2hBytes
         << ",\"attempt_d2h_bytes\":" << transfers.attemptD2hBytes
         << ",\"pcg_d2h_bytes\":" << transfers.pcgD2hBytes
         << ",\"total_h2d_bytes\":" << transfers.totalH2dBytes()
         << ",\"total_d2h_bytes\":" << transfers.totalD2hBytes() << "}";
}

void WriteTimings(std::ostream& output,
                  const SparseLevenbergMarquardtStageTimings& timings) {
  output << "{";
  bool first = true;
  for (const TimingField& field : TimingFields()) {
    if (!first) output << ",";
    first = false;
    output << "\"" << field.name << "\":" << timings.*(field.member);
  }
  output << "}";
}

void WriteAttempts(std::ostream& output,
                   const std::vector<SparseLevenbergMarquardtAttemptRecord>& attempts) {
  output << "[";
  for (size_t index = 0; index < attempts.size(); ++index) {
    if (index) output << ",";
    const SparseLevenbergMarquardtAttemptRecord& attempt = attempts[index];
    output << "{\"accepted_iterations_before\":"
           << attempt.acceptedIterationsBeforeAttempt
           << ",\"attempt\":" << attempt.attempt
           << ",\"lambda\":" << attempt.lambda
           << ",\"linearized_change\":" << attempt.linearizedChange
           << ",\"nonlinear_change\":" << attempt.nonlinearChange
           << ",\"model_fidelity\":" << attempt.modelFidelity
           << ",\"accepted\":" << (attempt.accepted ? "true" : "false")
           << "}";
  }
  output << "]";
}

void WriteRawRuns(std::ostream& output, const std::vector<RawRun>& runs) {
  output << "[";
  for (size_t index = 0; index < runs.size(); ++index) {
    if (index) output << ",";
    const RawRun& run = runs[index];
    output << "{\"backend\":\"" << run.backend
           << "\",\"repetition\":" << run.repetition
           << ",\"external_wall\":" << run.externalWall
           << ",\"initial_error\":" << run.initialError
           << ",\"final_error\":" << run.finalError
           << ",\"iterations\":" << run.iterations
           << ",\"outer_linearizations\":" << run.outerLinearizations
           << ",\"lambda_attempts\":" << run.lambdaAttempts
           << ",\"accepted_steps\":" << run.acceptedSteps
           << ",\"cudss_analyses\":" << run.cudssAnalyses
           << ",\"pcg_iterations_total\":" << run.pcgIterationsTotal
           << ",\"pcg_solves\":" << run.pcgSolves
           << ",\"pcg_max_iteration_hits\":" << run.pcgMaxIterationHits
           << ",\"termination\":\"" << run.termination
           << "\",\"final_lambda\":" << run.finalLambda
           << ",\"system_size\":";
    WriteSystemSize(output, run.systemSize);
    output << ",\"transfers\":";
    WriteTransfers(output, run.transfers);
    output << ",\"timings\":";
    WriteTimings(output, run.timings);
    output << ",\"linear_solve_stats\":{"
           << "\"user_ordering_applied\":"
           << (run.linearSolveStats.userOrderingApplied ? "true" : "false")
           << ",\"analysis_count\":" << run.linearSolveStats.analysisCount
           << ",\"factorization_count\":"
           << run.linearSolveStats.factorizationCount
           << ",\"solve_count\":" << run.linearSolveStats.solveCount
           << ",\"pcg_iterations_total\":"
           << run.linearSolveStats.pcgIterationsTotal
           << ",\"pcg_max_iteration_hits\":"
           << run.linearSolveStats.pcgMaxIterationHits
           << ",\"pcg_breakdown_count\":"
           << run.linearSolveStats.pcgBreakdownCount
           << ",\"last_pcg_iterations\":"
           << run.linearSolveStats.lastPcgIterations
           << ",\"pcg_host_convergence_checks\":"
           << run.linearSolveStats.pcgHostConvergenceChecks
           << ",\"pcg_d2h_bytes\":" << run.linearSolveStats.pcgD2hBytes
           << ",\"last_pcg_converged\":"
           << (run.linearSolveStats.lastPcgConverged ? "true" : "false")
           << ",\"last_pcg_breakdown\":"
           << (run.linearSolveStats.lastPcgBreakdown ? "true" : "false")
           << ",\"analysis_seconds\":"
           << run.linearSolveStats.analysisSeconds
           << ",\"factorization_seconds\":"
           << run.linearSolveStats.factorizationSeconds
           << ",\"solve_seconds\":" << run.linearSolveStats.solveSeconds
           << ",\"preconditioner_seconds\":"
           << run.linearSolveStats.preconditionerSeconds
           << ",\"pcg_d2h_seconds\":"
           << run.linearSolveStats.pcgD2hSeconds << "}";
    output << ",\"attempts\":";
    WriteAttempts(output, run.attempts);
    output << "}";
  }
  output << "]";
}

void WriteWorkloadJson(std::ostream& output, const WorkloadResult& result,
                       double objectiveTolerance) {
  const SummaryStatistics cpu = Summarize(ExternalSamples(result.cpuRuns));
  const SummaryStatistics gpu = Summarize(ExternalSamples(result.gpuRuns));
  output << "{\"name\":\"" << JsonEscape(result.name) << "\",\"path\":\""
         << JsonEscape(result.path) << "\",\"factors\":" << result.factors
         << ",\"values\":" << result.values
         << ",\"initialization_seconds\":" << result.initializationWall
         << ",\"initial_error\":" << result.initialError
         << ",\"reference_objective\":" << result.referenceObjective
         << ",\"maximum_objective_difference\":"
         << result.maximumObjectiveDifference
         << ",\"objective_relative_tolerance\":" << objectiveTolerance
         << ",\"cpu_external_wall\":";
  WriteStatistics(output, cpu);
  output << ",\"gpu_external_wall\":";
  WriteStatistics(output, gpu);
  output << ",\"median_cpu_over_gpu_speedup\":" << cpu.median / gpu.median
         << ",\"gpu_stage_summaries\":{";
  bool first = true;
  for (const TimingField& field : TimingFields()) {
    if (!first) output << ",";
    first = false;
    output << "\"" << field.name << "\":";
    WriteStatistics(output,
                    Summarize(TimingSamples(result.gpuRuns, field.member)));
  }
  output << "},\"cpu_runs\":";
  WriteRawRuns(output, result.cpuRuns);
  output << ",\"gpu_runs\":";
  WriteRawRuns(output, result.gpuRuns);
  output << "}";
}

std::string MakeJson(const RunOptions& options,
                     const HardwareMetadata& metadata,
                     const std::vector<WorkloadResult>& results) {
  std::ostringstream output;
  output << std::setprecision(17);
  output << "{\"schema_version\":1,\"generated_utc\":\""
         << metadata.utcTimestamp << "\",\"build\":{\"type\":\""
         << JsonEscape(GTSAM_BENCHMARK_BUILD_TYPE) << "\",\"git_commit\":\""
         << JsonEscape(metadata.gitCommit)
         << "\",\"git_dirty\":" << (metadata.gitDirty ? "true" : "false")
         << "},\"hardware\":{\"gpu\":\"" << JsonEscape(metadata.gpuName)
         << "\",\"gpu_memory_bytes\":" << metadata.gpuMemoryBytes
         << ",\"compute_capability\":\"" << metadata.computeMajor << "."
         << metadata.computeMinor
         << "\",\"cuda_runtime_version\":" << metadata.cudaRuntimeVersion
         << ",\"cuda_driver_api_version\":" << metadata.cudaDriverApiVersion
         << ",\"nvidia_driver_version\":\""
         << JsonEscape(metadata.nvidiaDriverVersion) << "\""
         << ",\"compiled_cuda_version\":" << metadata.compiledCudaVersion
         << "},\"configuration\":{\"warmups\":" << options.warmups
         << ",\"repeats\":" << options.repeats
         << ",\"name\":\"" << options.configuration << "\""
         << ",\"gpu_solver\":\"" << options.gpuSolver << "\""
         << ",\"ordering\":\"" << options.ordering << "\""
         << ",\"pcg_tolerance\":" << options.pcgTolerance
         << ",\"pcg_max_iterations\":" << options.pcgMaxIterations
         << ",\"pcg_preconditioner\":\"block-jacobi\""
         << ",\"pcg_warm_start\":true"
         << ",\"pose2_fast_sync\":"
         << (options.pose2FastSync ? "true" : "false")
         << ",\"pose2_upstream_settings\":"
         << (options.pose2UpstreamSettings ? "true" : "false")
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
         << ",\"correct_between_factor_jacobians\":true"
#else
         << ",\"correct_between_factor_jacobians\":false"
#endif
         << ",\"objective_relative_tolerance\":" << kObjectiveTolerance
         << ",\"gpu_objective_relative_tolerance\":"
         << options.objectiveTolerance
         << ",\"paired_order\":\"alternating_cpu_gpu\""
         << ",\"timing_note\":\"GPU stage timers overlap and must not be "
            "summed as exclusive wall time\"},\"workloads\":[";
  for (size_t index = 0; index < results.size(); ++index) {
    if (index) output << ",";
    WriteWorkloadJson(output, results[index], options.objectiveTolerance);
  }
  output << "]}\n";
  return output.str();
}

void EnsureOutputParent(const std::string& path) {
  const std::filesystem::path parent =
      std::filesystem::path(path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
}

void WriteAtomically(const std::string& path, const std::string& contents) {
  EnsureOutputParent(path);
  const std::string temporary = path + ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output: " + temporary);
    output << contents;
    output.flush();
    if (!output) throw std::runtime_error("cannot write output: " + temporary);
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(path, error);
    error.clear();
    std::filesystem::rename(temporary, path, error);
  }
  if (error) {
    throw std::runtime_error("cannot publish output " + path + ": " +
                             error.message());
  }
}

std::string MakeCsv(const RunOptions& options,
                    const std::vector<WorkloadResult>& results) {
  std::ostringstream output;
  output << std::setprecision(17);
  output
      << "configuration,ordering,dataset,backend,factors,values,"
         "jacobian_rows,jacobian_columns,"
         "jacobian_nonzeros,normal_nonzeros,warmups,repeats,external_median,"
         "external_mean,external_standard_deviation,external_minimum,"
         "external_maximum,initial_error,final_error,iterations,"
         "outer_linearizations,lambda_attempts,accepted_steps,cudss_analyses,"
         "user_ordering_applied,analysis_count,factorization_count,solve_count,"
         "pcg_iterations_total,"
         "pcg_max_iteration_hits,pcg_breakdown_count,last_pcg_iterations,"
         "last_pcg_converged,pcg_host_convergence_checks,pcg_d2h_bytes,"
         "last_pcg_breakdown,analysis_seconds,factorization_seconds,"
         "solve_seconds,preconditioner_seconds,pcg_d2h_seconds,"
         "termination,final_lambda,cpu_over_gpu_speedup,objective_difference,"
         "pattern_h2d_bytes,"
         "numeric_h2d_bytes,setup_d2h_bytes,attempt_d2h_bytes,pcg_d2h_bytes";
  for (const TimingField& field : TimingFields()) {
    output << "," << field.name << "_median";
  }
  output << "\n";

  for (const WorkloadResult& result : results) {
    const SummaryStatistics cpu = Summarize(ExternalSamples(result.cpuRuns));
    const SummaryStatistics gpu = Summarize(ExternalSamples(result.gpuRuns));
    const double speedup = cpu.median / gpu.median;
    for (const std::string backend : {"cpu", "gpu"}) {
      const std::vector<RawRun>& runs =
          backend == "cpu" ? result.cpuRuns : result.gpuRuns;
      const SummaryStatistics external = Summarize(ExternalSamples(runs));
      const RawRun& representative = runs.front();
      const SparseLevenbergMarquardtSystemSize& systemSize =
          result.gpuRuns.front().systemSize;
      output << options.configuration << "," << options.ordering << ","
             << CsvEscape(result.name) << "," << backend << ","
             << result.factors << "," << result.values << ","
             << systemSize.jacobianRows << "," << systemSize.jacobianColumns
             << "," << systemSize.jacobianNonzeros << ","
             << systemSize.normalNonzeros << ","
             << options.warmups << "," << options.repeats << ","
             << external.median << "," << external.mean << ","
             << external.standardDeviation << "," << external.minimum << ","
             << external.maximum << "," << representative.initialError << ","
             << representative.finalError << "," << representative.iterations
             << "," << representative.outerLinearizations << ","
             << representative.lambdaAttempts << ","
             << representative.acceptedSteps << ","
             << representative.cudssAnalyses << ","
             << representative.linearSolveStats.userOrderingApplied << ","
             << representative.linearSolveStats.analysisCount << ","
             << representative.linearSolveStats.factorizationCount << ","
             << representative.linearSolveStats.solveCount << ","
             << representative.linearSolveStats.pcgIterationsTotal << ","
             << representative.linearSolveStats.pcgMaxIterationHits << ","
             << representative.linearSolveStats.pcgBreakdownCount << ","
             << representative.linearSolveStats.lastPcgIterations << ","
             << representative.linearSolveStats.lastPcgConverged << ","
             << representative.linearSolveStats.pcgHostConvergenceChecks << ","
             << representative.linearSolveStats.pcgD2hBytes << ","
             << representative.linearSolveStats.lastPcgBreakdown << ","
             << representative.linearSolveStats.analysisSeconds << ","
             << representative.linearSolveStats.factorizationSeconds << ","
             << representative.linearSolveStats.solveSeconds << ","
             << representative.linearSolveStats.preconditionerSeconds << ","
             << representative.linearSolveStats.pcgD2hSeconds << ","
             << representative.termination << ","
             << representative.finalLambda << "," << speedup << ","
             << result.maximumObjectiveDifference << ",";
      if (backend == "gpu") {
        output << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &SparseLevenbergMarquardtTransferCounts::
                                              patternH2dBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &SparseLevenbergMarquardtTransferCounts::
                                              numericH2dBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs,
                                    &SparseLevenbergMarquardtTransferCounts::setupD2hBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &SparseLevenbergMarquardtTransferCounts::
                                              attemptD2hBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs,
                                    &SparseLevenbergMarquardtTransferCounts::pcgD2hBytes))
                          .median);
      } else {
        output << ",,,,";
      }
      for (const TimingField& field : TimingFields()) {
        output << ",";
        if (backend == "gpu") {
          output << Summarize(TimingSamples(runs, field.member)).median;
        }
      }
      output << "\n";
    }
  }
  return output.str();
}

void PrintStatistics(const char* label,
                     const SummaryStatistics& statistics) {
  std::cout << "  " << std::left << std::setw(35) << label << std::right
            << " median " << statistics.median << " s, mean "
            << statistics.mean << " s, stddev "
            << statistics.standardDeviation << " s, range ["
            << statistics.minimum << ", " << statistics.maximum << "] s\n";
}

void PrintResult(const WorkloadResult& result) {
  const SummaryStatistics cpu = Summarize(ExternalSamples(result.cpuRuns));
  const SummaryStatistics gpu = Summarize(ExternalSamples(result.gpuRuns));
  const RawRun& representative = result.gpuRuns.front();
  std::cout << "\nResult: " << result.name << "\n"
            << "  factors " << result.factors << ", values " << result.values
            << ", residual rows " << representative.systemSize.jacobianRows
            << ", columns " << representative.systemSize.jacobianColumns
            << ", J.nnz " << representative.systemSize.jacobianNonzeros
            << ", H.nnz " << representative.systemSize.normalNonzeros << "\n"
            << std::setprecision(15) << "  objective "
            << result.referenceObjective << ", maximum CPU/GPU difference "
            << result.maximumObjectiveDifference << std::setprecision(9)
            << "\n";
  PrintStatistics("CPU external wall", cpu);
  PrintStatistics("GPU external wall", gpu);
  std::cout << "  median CPU/GPU speedup: " << cpu.median / gpu.median
            << "x\n"
            << "  GPU iterations " << representative.iterations
            << ", outer linearizations " << representative.outerLinearizations
            << ", lambda attempts " << representative.lambdaAttempts
            << ", analyses " << representative.cudssAnalyses
            << ", termination " << representative.termination << "\n"
            << "  GPU detailed timing medians (overlapping fields):\n";
  for (const TimingField& field : TimingFields()) {
    const SummaryStatistics statistics =
        Summarize(TimingSamples(result.gpuRuns, field.member));
    std::cout << "    " << std::left << std::setw(38) << field.name << std::right
              << statistics.median << " s\n";
  }
  std::cout << "  GPU median transfer bytes:\n"
            << "    pattern H2D: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &SparseLevenbergMarquardtTransferCounts::patternH2dBytes))
                       .median)
            << "\n    numeric H2D: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &SparseLevenbergMarquardtTransferCounts::numericH2dBytes))
                       .median)
            << "\n    setup D2H: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &SparseLevenbergMarquardtTransferCounts::setupD2hBytes))
                       .median)
            << "\n    attempt D2H: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &SparseLevenbergMarquardtTransferCounts::attemptD2hBytes))
                       .median)
            << "\n    PCG convergence D2H: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &SparseLevenbergMarquardtTransferCounts::pcgD2hBytes))
                       .median)
            << "\n";
}

int RunBenchmark(const RunOptions& options) {
  const HardwareMetadata metadata = QueryHardware();
  const bool humanOutput = options.outputFormat == "text";
  if (humanOutput) {
    std::cout << std::setprecision(9)
              << "CUDA direct sparse LM benchmark\n"
            << "GPU: " << metadata.gpuName << " (sm_" << metadata.computeMajor
            << metadata.computeMinor << ", " << metadata.gpuMemoryBytes
            << " bytes)\n"
            << "NVIDIA driver: " << metadata.nvidiaDriverVersion
            << ", CUDA driver API: " << metadata.cudaDriverApiVersion << "\n"
            << "CUDA runtime: " << metadata.cudaRuntimeVersion
            << ", compiled: " << metadata.compiledCudaVersion << "\n"
            << "Build: " << GTSAM_BENCHMARK_BUILD_TYPE
            << ", git: " << metadata.gitCommit
            << (metadata.gitDirty ? " (dirty)" : " (clean)") << "\n"
            << "Warm-ups: " << options.warmups
            << ", measured repeats: " << options.repeats << "\n"
              << "GPU timing fields overlap; they are not an exclusive sum.\n";
  }

  std::vector<WorkloadResult> results;
  results.reserve(options.datasets.size());
  for (const std::string& dataset : options.datasets) {
    const LoadedWorkload workload =
        LoadWorkload(LookupWorkload(dataset), options);
    results.push_back(BenchmarkWorkload(workload, options, humanOutput));
    if (humanOutput) PrintResult(results.back());
    WriteAtomically(options.jsonPath, MakeJson(options, metadata, results));
    WriteAtomically(options.csvPath, MakeCsv(options, results));
    if (humanOutput) {
      std::cout << "  published " << options.jsonPath << " and "
                << options.csvPath << "\n";
    }
  }
  if (options.outputFormat == "json") {
    std::cout << MakeJson(options, metadata, results);
  } else if (options.outputFormat == "csv") {
    std::cout << MakeCsv(options, results);
  }
  return 0;
}

#if GTSAM_ENABLE_CUDSS
constexpr std::array<const char*, 3> kCudaConfigurations{
    "cudss-auto", "cudss-gtsam", "pcg"};
#else
constexpr std::array<const char*, 1> kCudaConfigurations{"pcg"};
#endif

void PrintConfigurations() {
  for (const char* configuration : kCudaConfigurations) {
    std::cout << configuration << "\n";
  }
}

void PrintDryRun(const RunOptions& options) {
  const auto printJsonRecord = [](const char* configuration) {
    const bool pcg = std::string(configuration) == "pcg";
    const bool ordered = std::string(configuration) == "cudss-gtsam";
    std::cout
        << "{\"configuration\":\"" << configuration
        << "\",\"formulation\":\"general-normal\",\"backend\":\""
        << (pcg ? "pcg" : "cudss") << "\",\"ordering\":\""
        << (ordered ? "gtsam" : "auto")
        << "\",\"dimension\":0,\"nnz\":0,\"analysis_count\":0,"
           "\"factorization_count\":0,\"solve_count\":0,"
           "\"pcg_iterations\":0,\"pcg_converged\":false,"
           "\"initial_objective\":0,\"final_objective\":0,"
           "\"h2d_bytes\":0,\"d2h_bytes\":0,\"frontend_wall_seconds\":0,"
           "\"analysis_seconds\":0,\"factorization_seconds\":0,"
           "\"solve_seconds\":0,\"accepted_iterations\":0,"
           "\"lambda_attempts\":0,\"dry_run\":true}";
  };
  if (options.outputFormat == "json") {
    std::cout << "[";
    for (size_t i = 0; i < kCudaConfigurations.size(); ++i) {
      if (i != 0) std::cout << ",";
      printJsonRecord(kCudaConfigurations[i]);
    }
    std::cout << "]\n";
    return;
  }
  if (options.outputFormat == "csv") {
    std::cout << "configuration,formulation,backend,ordering,dimension,nnz,"
                 "analysis_count,factorization_count,solve_count,"
                 "pcg_iterations,pcg_converged,initial_objective,"
                 "final_objective,h2d_bytes,d2h_bytes,frontend_wall_seconds,"
                 "analysis_seconds,factorization_seconds,solve_seconds,"
                 "accepted_iterations,lambda_attempts,dry_run\n";
    for (const char* configuration : kCudaConfigurations) {
      const bool pcg = std::string(configuration) == "pcg";
      const bool ordered = std::string(configuration) == "cudss-gtsam";
      std::cout << configuration << ",general-normal,"
                << (pcg ? "pcg" : "cudss") << ","
                << (ordered ? "gtsam" : "auto")
                << ",0,0,0,0,0,0,false,0,0,0,0,0,0,0,0,0,0,true\n";
    }
    return;
  }
  for (const char* configuration : kCudaConfigurations) {
    std::cout << "configuration=" << configuration
              << " formulation=general-normal backend="
              << (std::string(configuration) == "pcg" ? "pcg" : "cudss")
              << " ordering="
              << (std::string(configuration) == "cudss-gtsam" ? "gtsam"
                                                               : "auto")
              << " dry_run=true\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const RunOptions options = ParseOptions(argc, argv);
    if (options.help) {
      std::cout << Usage() << "\n";
      return 0;
    }
    if (options.listConfigurations) {
      PrintConfigurations();
      return 0;
    }
    if (options.dryRun) {
      PrintDryRun(options);
      return 0;
    }
    if (options.allCudaConfigurations) {
      for (const char* configuration : kCudaConfigurations) {
        RunOptions row = options;
        row.allCudaConfigurations = false;
        row.configuration = configuration;
        row.gpuSolver =
            std::string(configuration) == "pcg" ? "pcg" : "cudss";
        row.ordering = std::string(configuration) == "cudss-gtsam"
                           ? "gtsam"
                           : "auto";
        ApplyMatrixConfigurationDefaults(&row);
        row.jsonPath += "." + row.configuration + ".json";
        row.csvPath += "." + row.configuration + ".csv";
        const int status = RunBenchmark(row);
        if (status != 0) return status;
      }
      return 0;
    }
    return RunBenchmark(options);
  } catch (const std::exception& error) {
    std::cerr << "timeCudaSparseLM: " << error.what() << "\n";
    return 1;
  }
}
