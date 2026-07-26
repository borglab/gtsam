#include "../timeSFMBAL.h"

#include <cuda_runtime_api.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
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

using Clock = std::chrono::steady_clock;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Pose2;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::cuda::CudaSparseLevenbergMarquardtOptimizer;
using gtsam::cuda::CudaSparseLevenbergMarquardtParams;
using gtsam::cuda::CudaSparseLevenbergMarquardtResult;
using gtsam::cuda::CudaSparseLmAttemptRecord;
using gtsam::cuda::CudaSparseLmBackend;
using gtsam::cuda::CudaSparseLmStageTimings;
using gtsam::cuda::CudaSparseLmSystemSize;
using gtsam::cuda::CudaSparseLmTerminationReason;
using gtsam::cuda::CudaSparseLmTransferCounts;

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
  std::string gpuSolver = "cudss";
  double pcgTolerance = 0.0;   // 0 keeps the library default
  size_t pcgMaxIterations = 0; // 0 keeps the library default
  // Relative CPU/GPU final-objective tolerance. The strict default assumes a
  // direct solver; inexact PCG takes a different LM trajectory and needs an
  // explicit, recorded loosening.
  double objectiveTolerance = kObjectiveTolerance;
  bool selfTest = false;
  bool help = false;
};

struct SummaryStatistics {
  double median = 0.0;
  double mean = 0.0;
  double standardDeviation = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
};

enum class WorkloadKind { Bal, Pose2, Pose3 };

struct WorkloadSpec {
  std::string name;
  WorkloadKind kind;
  std::string filename;
  std::string exampleDataName;
};

struct LoadedWorkload {
  WorkloadSpec spec;
  std::string path;
  NonlinearFactorGraph graph;
  Values initial;
  std::optional<Ordering> cpuOrdering;
  double initialError = 0.0;
};

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
  CudaSparseLmSystemSize systemSize;
  CudaSparseLmTransferCounts transfers;
  CudaSparseLmStageTimings timings;
  std::vector<CudaSparseLmAttemptRecord> attempts;
};

struct TimingField {
  const char* name;
  double CudaSparseLmStageTimings::*member;
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
#define MAKE_TIMING_FIELD(name) {#name, &CudaSparseLmStageTimings::name},
      FOR_EACH_TIMING_FIELD(MAKE_TIMING_FIELD)
#undef MAKE_TIMING_FIELD
  };
  return fields;
}

std::string Usage() {
  return
      "Usage: timeCudaSparseLM [--warmups N] [--repeats N]\n"
      "  [--datasets bal16,bal135,pose2,pose3] [--data-dir DIR]\n"
      "  [--json FILE] [--csv FILE]\n"
      "  [--gpu-solver cudss|pcg] [--pcg-tol X] [--pcg-max-iters N]\n"
      "  [--objective-tol X] [--self-test] [--help]";
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
  static const std::set<std::string> supported{"bal16", "bal135", "pose2",
                                                "pose3"};
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
    if (argument == "--self-test") {
      options.selfTest = true;
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
    } else if (argument == "--gpu-solver") {
      options.gpuSolver = requireValue(&index, "--gpu-solver");
      if (options.gpuSolver != "cudss" && options.gpuSolver != "pcg") {
        throw std::invalid_argument("--gpu-solver requires cudss or pcg");
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

template <class Function>
bool Throws(Function&& function) {
  try {
    function();
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

int RunSelfTest() {
  const SummaryStatistics odd = Summarize({5.0, 1.0, 4.0, 2.0, 3.0});
  if (odd.median != 3.0 || odd.mean != 3.0 ||
      std::abs(odd.standardDeviation - std::sqrt(2.0)) > 1e-12 ||
      odd.minimum != 1.0 || odd.maximum != 5.0) {
    throw std::runtime_error("odd statistics self-test failed");
  }
  const SummaryStatistics even = Summarize({4.0, 1.0, 3.0, 2.0});
  if (even.median != 2.5) {
    throw std::runtime_error("even median self-test failed");
  }
  if (!Throws([] { (void)Summarize({}); })) {
    throw std::runtime_error("empty statistics self-test failed");
  }

  ValidateObjective(100.0, 100.0 + 5e-7, 1e-8);
  if (!Throws([] { ValidateObjective(100.0, 101.0, 1e-8); })) {
    throw std::runtime_error("objective mismatch self-test failed");
  }
  if (CsvEscape("a,\"b\"") != "\"a,\"\"b\"\"\"" ||
      JsonEscape("a\n\"b\"") != "a\\n\\\"b\\\"") {
    throw std::runtime_error("escaping self-test failed");
  }
  const auto edge =
      ParseLegacyToroPose3Edge("EDGE3 4 9 1 2 3 0.1 0.2 0.3");
  if (!edge || edge->key1 != 4 || edge->key2 != 9 ||
      (edge->measured.translation() - gtsam::Point3(1, 2, 3)).norm() >
          1e-12 ||
      ParseLegacyToroPose3Edge("VERTEX3 4 1 2 3 0.1 0.2 0.3")) {
    throw std::runtime_error("legacy TORO Pose3 parser self-test failed");
  }
  if (!Throws([] {
        (void)ParseLegacyToroPose3Edge("EDGE3 4 9 1 2");
      }) ||
      !Throws([] {
        (void)ParseLegacyToroPose3Edge(
            "EDGE3 4 9 1 2 3 0.1 0.2 0.3 1.0");
      })) {
    throw std::runtime_error(
        "legacy TORO Pose3 parser rejection self-test failed");
  }

  const char* arguments[] = {
      "timeCudaSparseLM", "--warmups", "1", "--repeats", "5",
      "--datasets",       "bal16,bal135,pose2,pose3",
      "--json",           "results.json", "--csv", "results.csv"};
  RunOptions options =
      ParseOptions(static_cast<int>(std::size(arguments)),
                   const_cast<char**>(arguments));
  if (options.warmups != 1 || options.repeats != 5 ||
      options.datasets.size() != 4 || options.jsonPath != "results.json" ||
      options.csvPath != "results.csv") {
    throw std::runtime_error("option parsing self-test failed");
  }
  if (!Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--repeats", "0"};
        (void)ParseOptions(3, const_cast<char**>(args));
      }) ||
      !Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--datasets", "pose2,pose2"};
        (void)ParseOptions(3, const_cast<char**>(args));
      }) ||
      !Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--warmups", "-1"};
        (void)ParseOptions(3, const_cast<char**>(args));
      }) ||
      !Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--warmups", "+1"};
        (void)ParseOptions(3, const_cast<char**>(args));
      }) ||
      !Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--datasets", "pose2,"};
        (void)ParseOptions(3, const_cast<char**>(args));
      }) ||
      !Throws([] {
        const char* args[] = {"timeCudaSparseLM", "--unknown"};
        (void)ParseOptions(2, const_cast<char**>(args));
      })) {
    throw std::runtime_error("option rejection self-test failed");
  }
  std::cout << "timeCudaSparseLM self-test passed\n";
  return 0;
}

const WorkloadSpec& LookupWorkload(const std::string& name) {
  static const std::map<std::string, WorkloadSpec> specifications{
      {"bal16",
       {"bal16", WorkloadKind::Bal, "dubrovnik-16-22106-pre.txt",
        "dubrovnik-16-22106-pre"}},
      {"bal135",
       {"bal135", WorkloadKind::Bal, "dubrovnik-135-90642-pre.txt",
        "dubrovnik-135-90642-pre"}},
      {"pose2",
       {"pose2", WorkloadKind::Pose2, "w10000.graph", "w10000"}},
      {"pose3",
       {"pose3", WorkloadKind::Pose3, "sphere_smallnoise.graph",
        "sphere_smallnoise"}},
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
    workload.graph = buildGeneralSfmGraph(data);
    workload.initial = buildGeneralSfmInitial(data);
    workload.cpuOrdering = createSchurOrdering(data, false);
  } else if (specification.kind == WorkloadKind::Pose2) {
    const auto [graph, initial] = gtsam::load2D(workload.path);
    if (!graph || !initial || !initial->exists(0)) {
      throw std::runtime_error("Pose2 dataset is missing pose 0");
    }
    workload.graph = *graph;
    workload.initial = *initial;
    workload.graph.emplace_shared<gtsam::PriorFactor<Pose2>>(
        0, workload.initial.at<Pose2>(0), gtsam::noiseModel::Unit::Create(3));
  } else {
    auto [graph, initial] = LoadLegacyToroPose3(workload.path);
    if (!initial.exists(0)) {
      throw std::runtime_error("Pose3 dataset is missing pose 0");
    }
    workload.graph = std::move(graph);
    workload.initial = std::move(initial);
    workload.graph.emplace_shared<gtsam::PriorFactor<Pose3>>(
        0, workload.initial.at<Pose3>(0), gtsam::noiseModel::Unit::Create(6));
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

CudaSparseLevenbergMarquardtParams MakeGpuParams(
    const LoadedWorkload& workload, const RunOptions& options) {
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM("SILENT");
  if (workload.spec.kind == WorkloadKind::Bal) {
    params.maxIterations = 20;
    params.relativeErrorTol = 0.01;
  }
  params.fallbackOnUnsupported = false;
  params.collectTiming = true;
  params.collectAttemptTrace = true;
  if (options.gpuSolver == "pcg") {
    params.linearSolver = gtsam::cuda::CudaSparseLmLinearSolver::Pcg;
    if (options.pcgTolerance > 0.0) {
      params.pcg.relativeTolerance = options.pcgTolerance;
    }
    if (options.pcgMaxIterations > 0) {
      params.pcg.maxIterations = static_cast<int>(options.pcgMaxIterations);
    }
  }
  return params;
}

const char* TerminationName(CudaSparseLmTerminationReason termination) {
  switch (termination) {
    case CudaSparseLmTerminationReason::None:
      return "none";
    case CudaSparseLmTerminationReason::ErrorThreshold:
      return "error_threshold";
    case CudaSparseLmTerminationReason::Converged:
      return "converged";
    case CudaSparseLmTerminationReason::MaxIterations:
      return "max_iterations";
    case CudaSparseLmTerminationReason::SmallCostChange:
      return "small_cost_change";
    case CudaSparseLmTerminationReason::LambdaUpperBound:
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
    for (const CudaSparseLmAttemptRecord& attempt : run.attempts) {
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
  const CudaSparseLevenbergMarquardtParams params =
      MakeGpuParams(workload, options);
  const auto begin = Clock::now();
  CudaSparseLevenbergMarquardtOptimizer optimizer(workload.graph,
                                                  workload.initial, params);
  (void)optimizer.optimize();
  const auto end = Clock::now();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  if (result.backend != CudaSparseLmBackend::Cuda) {
    throw std::runtime_error("GPU benchmark fell back to CPU: " +
                             result.fallbackDetail);
  }
  if (result.termination == CudaSparseLmTerminationReason::None) {
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
  run.attempts = result.attemptTrace;
  ValidateRawRun(run);
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
    double CudaSparseLmStageTimings::*member) {
  std::vector<double> samples;
  samples.reserve(runs.size());
  for (const RawRun& run : runs) samples.push_back(run.timings.*member);
  return samples;
}

std::vector<double> TransferSamples(const std::vector<RawRun>& runs,
                                    size_t CudaSparseLmTransferCounts::*member) {
  std::vector<double> samples;
  samples.reserve(runs.size());
  for (const RawRun& run : runs) {
    samples.push_back(static_cast<double>(run.transfers.*member));
  }
  return samples;
}

WorkloadResult BenchmarkWorkload(const LoadedWorkload& workload,
                                 const RunOptions& options) {
  std::cout << "\nLoading complete: " << workload.spec.name << "\n"
            << "  path: " << workload.path << "\n"
            << "  factors: " << workload.graph.size()
            << ", values: " << workload.initial.size()
            << ", initial error: " << std::setprecision(15)
            << workload.initialError << std::setprecision(9) << "\n";

  for (size_t warmup = 0; warmup < options.warmups; ++warmup) {
    std::cout << "  warm-up " << warmup << ": CPU" << std::flush;
    (void)RunCpu(workload, warmup);
    std::cout << ", GPU" << std::flush;
    (void)RunGpu(workload, warmup, options);
    std::cout << " complete\n";
  }

  WorkloadResult result;
  result.name = workload.spec.name;
  result.path = workload.path;
  result.factors = workload.graph.size();
  result.values = workload.initial.size();
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
    std::cout << "  repetition " << repetition << ": CPU "
              << result.cpuRuns.back().externalWall << " s, GPU "
              << result.gpuRuns.back().externalWall << " s\n";
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
                     const CudaSparseLmSystemSize& size) {
  output << "{\"factors\":" << size.factors
         << ",\"jacobian_rows\":" << size.jacobianRows
         << ",\"jacobian_columns\":" << size.jacobianColumns
         << ",\"jacobian_nonzeros\":" << size.jacobianNonzeros
         << ",\"normal_nonzeros\":" << size.normalNonzeros << "}";
}

void WriteTransfers(std::ostream& output,
                    const CudaSparseLmTransferCounts& transfers) {
  output << "{\"pattern_h2d_bytes\":" << transfers.patternH2dBytes
         << ",\"numeric_h2d_bytes\":" << transfers.numericH2dBytes
         << ",\"setup_d2h_bytes\":" << transfers.setupD2hBytes
         << ",\"attempt_d2h_bytes\":" << transfers.attemptD2hBytes
         << ",\"total_h2d_bytes\":" << transfers.totalH2dBytes()
         << ",\"total_d2h_bytes\":" << transfers.totalD2hBytes() << "}";
}

void WriteTimings(std::ostream& output,
                  const CudaSparseLmStageTimings& timings) {
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
                   const std::vector<CudaSparseLmAttemptRecord>& attempts) {
  output << "[";
  for (size_t index = 0; index < attempts.size(); ++index) {
    if (index) output << ",";
    const CudaSparseLmAttemptRecord& attempt = attempts[index];
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
    output << ",\"attempts\":";
    WriteAttempts(output, run.attempts);
    output << "}";
  }
  output << "]";
}

void WriteWorkloadJson(std::ostream& output, const WorkloadResult& result) {
  const SummaryStatistics cpu = Summarize(ExternalSamples(result.cpuRuns));
  const SummaryStatistics gpu = Summarize(ExternalSamples(result.gpuRuns));
  output << "{\"name\":\"" << JsonEscape(result.name) << "\",\"path\":\""
         << JsonEscape(result.path) << "\",\"factors\":" << result.factors
         << ",\"values\":" << result.values
         << ",\"initial_error\":" << result.initialError
         << ",\"reference_objective\":" << result.referenceObjective
         << ",\"maximum_objective_difference\":"
         << result.maximumObjectiveDifference
         << ",\"objective_relative_tolerance\":" << kObjectiveTolerance
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
         << ",\"gpu_solver\":\"" << options.gpuSolver << "\""
         << ",\"pcg_tolerance\":" << options.pcgTolerance
         << ",\"pcg_max_iterations\":" << options.pcgMaxIterations
         << ",\"objective_relative_tolerance\":" << kObjectiveTolerance
         << ",\"gpu_objective_relative_tolerance\":"
         << options.objectiveTolerance
         << ",\"paired_order\":\"alternating_cpu_gpu\""
         << ",\"timing_note\":\"GPU stage timers overlap and must not be "
            "summed as exclusive wall time\"},\"workloads\":[";
  for (size_t index = 0; index < results.size(); ++index) {
    if (index) output << ",";
    WriteWorkloadJson(output, results[index]);
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
      << "dataset,backend,factors,values,jacobian_rows,jacobian_columns,"
         "jacobian_nonzeros,normal_nonzeros,warmups,repeats,external_median,"
         "external_mean,external_standard_deviation,external_minimum,"
         "external_maximum,initial_error,final_error,iterations,"
         "outer_linearizations,lambda_attempts,accepted_steps,cudss_analyses,"
         "termination,final_lambda,cpu_over_gpu_speedup,objective_difference,"
         "pattern_h2d_bytes,"
         "numeric_h2d_bytes,setup_d2h_bytes,attempt_d2h_bytes";
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
      const CudaSparseLmSystemSize& systemSize =
          result.gpuRuns.front().systemSize;
      output << CsvEscape(result.name) << "," << backend << ","
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
             << representative.termination << ","
             << representative.finalLambda << "," << speedup << ","
             << result.maximumObjectiveDifference << ",";
      if (backend == "gpu") {
        output << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &CudaSparseLmTransferCounts::
                                              patternH2dBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &CudaSparseLmTransferCounts::
                                              numericH2dBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs,
                                    &CudaSparseLmTransferCounts::setupD2hBytes))
                          .median)
               << ","
               << static_cast<size_t>(
                      Summarize(TransferSamples(
                                    runs, &CudaSparseLmTransferCounts::
                                              attemptD2hBytes))
                          .median);
      } else {
        output << ",,,";
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
                                 &CudaSparseLmTransferCounts::patternH2dBytes))
                       .median)
            << "\n    numeric H2D: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &CudaSparseLmTransferCounts::numericH2dBytes))
                       .median)
            << "\n    setup D2H: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &CudaSparseLmTransferCounts::setupD2hBytes))
                       .median)
            << "\n    attempt D2H: "
            << static_cast<size_t>(
                   Summarize(TransferSamples(
                                 result.gpuRuns,
                                 &CudaSparseLmTransferCounts::attemptD2hBytes))
                       .median)
            << "\n";
}

int RunBenchmark(const RunOptions& options) {
  const HardwareMetadata metadata = QueryHardware();
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

  std::vector<WorkloadResult> results;
  results.reserve(options.datasets.size());
  for (const std::string& dataset : options.datasets) {
    const LoadedWorkload workload =
        LoadWorkload(LookupWorkload(dataset), options);
    results.push_back(BenchmarkWorkload(workload, options));
    PrintResult(results.back());
    WriteAtomically(options.jsonPath, MakeJson(options, metadata, results));
    WriteAtomically(options.csvPath, MakeCsv(options, results));
    std::cout << "  published " << options.jsonPath << " and "
              << options.csvPath << "\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const RunOptions options = ParseOptions(argc, argv);
    if (options.help) {
      std::cout << Usage() << "\n";
      return 0;
    }
    if (options.selfTest) return RunSelfTest();
    return RunBenchmark(options);
  } catch (const std::exception& error) {
    std::cerr << "timeCudaSparseLM: " << error.what() << "\n";
    return 1;
  }
}
