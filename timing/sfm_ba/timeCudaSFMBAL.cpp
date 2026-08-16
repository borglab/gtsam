/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCudaSFMBAL.cpp
 * @brief   time CUDA SFM/BA with BAL files
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>

#include "../internal/TimingUtils.h"
#include "../timeSFMBAL.h"
#include "GncOutlierSampling.h"

#include <gtsam/config.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#if GTSAM_ENABLE_CUDA
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>
#include <gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmReducedCsrPlan.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#endif

#include <gtsam/nonlinear/GncOptimizer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>

namespace {
using namespace gtsam;
using namespace std;
using symbol_shorthand::C;
using symbol_shorthand::P;
namespace bal = gtsam::timing::bal;

std::string usage() {
  return "Usage: timeCudaSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
         "[--cuda-lm] [--cuda-lm-graph] "
         "[--cuda-sparse-lm] [--cuda-sparse-pack-only] "
         "[--gaussian-graph-pack-diagnostic] "
         "[--linearization-benchmark] [--linearization-repeats N] "
         "[--configuration NAME] [--formulation schur|full-normal] "
         "[--ordering auto|gtsam] [--output-format text|csv|json] "
         "[--list-configurations] [--dry-run] "
         "[--cuda-linear-solver dense-schur|cudss-schur|pcg-schur|"
         "cudss-full-normal|pcg-full-normal] "
         "[--cuda-lm-graph-kind raw|point-batch|camera-batch] "
         "[--batch-chunk-size N] "
         "[--cuda-warmup-file FILE] "
         "[--projection-noise unit|huber|tukey] "
         "[--gnc cpu|cuda] [--gnc-loss tls|gm] "
         "[--gnc-outlier-fraction F] [--gnc-outlier-pixels P] "
         "[--gnc-seed N] [--gnc-max-outer N] "
         "[--benchmark-action-json FILE] [BALfile ...]";
}

struct TimingRow {
  std::string dataset;
  double legacy = 0.0;
  double newer = 0.0;
};

enum class CudaLinearSolverOption {
  DenseSchur,
  CudssSchur,
  PcgSchur,
  CudssFullNormal,
  PcgFullNormal,
};

enum class CudaGraphKind {
  Raw,
  PointBatch,
  CameraBatch,
};

enum class GncBackend {
  None,
  Cpu,
  Cuda,
};

struct GncRunOptions {
  GncBackend backend = GncBackend::None;
  GncLossType lossType = GncLossType::TLS;
  double outlierFraction = 0.1;
  double outlierPixels = 12.0;
  unsigned int seed = 42;
  size_t maxOuterIterations = 100;
};

struct RunOptions {
  bal::BalBenchmarkConfig config;
  bool profile = false;
  bool cudaStructureOnly = false;
  bool cudaLm = false;
  bool cudaLmGraph = false;
  bool cudaSparseLm = false;
  bool cudaSparsePackOnly = false;
  bool gaussianGraphPackDiagnostic = false;
  bool linearizationBenchmark = false;
  bool linearizationRepeatsSpecified = false;
  size_t linearizationRepeats = 10;
  bool cudaLinearSolverSpecified = false;
  CudaLinearSolverOption cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
  bool cudaGraphKindSpecified = false;
  CudaGraphKind cudaGraphKind = CudaGraphKind::Raw;
  bool batchChunkSizeSpecified = false;
  size_t batchChunkSize = 0;
  bool cudaWarmupFileSpecified = false;
  std::string cudaWarmupFile;
  bool benchmarkActionJson = false;
  std::string benchmarkActionJsonPath;
  GncRunOptions gnc;
  std::vector<std::string> filenames;
  bool help = false;
  bool listConfigurations = false;
  bool dryRun = false;
  std::string matrixConfiguration = "schur-dense";
  std::string ordering = "auto";
  std::string outputFormat = "text";
};

const char* cudaLinearSolverName(CudaLinearSolverOption solver) {
  switch (solver) {
    case CudaLinearSolverOption::DenseSchur:
      return "dense-schur";
    case CudaLinearSolverOption::CudssSchur:
      return "cudss-schur";
    case CudaLinearSolverOption::PcgSchur:
      return "pcg-schur";
    case CudaLinearSolverOption::CudssFullNormal:
      return "cudss-full-normal";
    case CudaLinearSolverOption::PcgFullNormal:
      return "pcg-full-normal";
  }
  return "unknown";
}

const char* cudaGraphKindName(CudaGraphKind kind) {
  switch (kind) {
    case CudaGraphKind::Raw:
      return "raw";
    case CudaGraphKind::PointBatch:
      return "point-batch";
    case CudaGraphKind::CameraBatch:
      return "camera-batch";
  }
  return "unknown";
}

void setProjectionNoiseModel(bal::BalBenchmarkConfig* config,
                             const std::string& name) {
  if (name == "unit") {
    config->projectionNoise = noiseModel::Unit::Create(2);
  } else if (name == "huber") {
    config->projectionNoise =
        noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),
                                   noiseModel::Unit::Create(2));
  } else if (name == "tukey") {
    config->projectionNoise = noiseModel::Robust::Create(
        noiseModel::mEstimator::Tukey::Create(4.6851),
        noiseModel::Unit::Create(2));
  } else {
    throw runtime_error(usage());
  }
}

template <typename Params>
void applyBalBenchmarkLmSettings(Params& params) {
  params.maxIterations = 20;
  params.relativeErrorTol = 0.01;
}

LevenbergMarquardtParams makeBalLevenbergMarquardtParams(
    const bal::BalBenchmarkConfig& config) {
  LevenbergMarquardtParams params =
      bal::makeLevenbergMarquardtParams(config, nullptr, "SILENT");
  applyBalBenchmarkLmSettings(params);
  return params;
}

void printProfileRow(const std::string& indent, const std::string& label,
                     double elapsed, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << "\n";
}

bool gtsamBuiltWithTbb() {
#ifdef GTSAM_USE_TBB
  return true;
#else
  return false;
#endif
}

struct TimingSamples {
  std::vector<double> seconds;

  double mean() const {
    double total = 0.0;
    for (const double sample : seconds) total += sample;
    return total / static_cast<double>(seconds.size());
  }

  double minimum() const {
    return *std::min_element(seconds.begin(), seconds.end());
  }

  double maximum() const {
    return *std::max_element(seconds.begin(), seconds.end());
  }
};

struct CpuLinearizationBenchmark {
  TimingSamples timing;
  size_t linearizedFactors = 0;
};

struct CpuStateBenchmark {
  TimingSamples retract;
  TimingSamples trialError;
  size_t retractedValues = 0;
  double error = 0.0;
};

VectorValues makeBenchmarkDelta(const Values& initial) {
  VectorValues delta = initial.zeroVectors();
  for (Key key : initial.keys()) {
    Vector& value = delta.at(key);
    value.setConstant(1e-6);
  }
  return delta;
}

CpuLinearizationBenchmark benchmarkCpuLinearization(
    const NonlinearFactorGraph& graph, const Values& initial, size_t repeats) {
  GaussianFactorGraph::shared_ptr warmup = graph.linearize(initial);
  warmup.reset();

  CpuLinearizationBenchmark benchmark;
  benchmark.timing.seconds.reserve(repeats);
  for (size_t repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    GaussianFactorGraph::shared_ptr linear = graph.linearize(initial);
    const auto end = std::chrono::steady_clock::now();
    benchmark.timing.seconds.push_back(
        std::chrono::duration<double>(end - start).count());
    benchmark.linearizedFactors = linear->size();
  }
  return benchmark;
}

CpuStateBenchmark benchmarkCpuState(const NonlinearFactorGraph& graph,
                                    const Values& initial,
                                    const VectorValues& delta, size_t repeats) {
  initial.retract(delta);

  CpuStateBenchmark benchmark;
  benchmark.retract.seconds.reserve(repeats);
  benchmark.trialError.seconds.reserve(repeats);
  for (size_t repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    {
      const Values retracted = initial.retract(delta);
      const auto end = std::chrono::steady_clock::now();
      benchmark.retract.seconds.push_back(
          std::chrono::duration<double>(end - start).count());
      benchmark.retractedValues = retracted.size();
    }
  }

  const Values trial = initial.retract(delta);
  graph.error(trial);
  for (size_t repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    benchmark.error = graph.error(trial);
    const auto end = std::chrono::steady_clock::now();
    benchmark.trialError.seconds.push_back(
        std::chrono::duration<double>(end - start).count());
  }
  if (!std::isfinite(benchmark.error)) {
    throw std::runtime_error("CPU trial graph.error() is not finite");
  }
  return benchmark;
}

void printTimingSamples(const std::string& label, const TimingSamples& timing,
                        size_t workItems, const std::string& workItemLabel) {
  std::cout << "  " << label << "\n";
  for (size_t repeat = 0; repeat < timing.seconds.size(); ++repeat) {
    std::cout << "    repeat " << repeat << ": " << timing.seconds[repeat]
              << " s\n";
  }
  std::cout << "    mean: " << timing.mean() << " s\n";
  std::cout << "    min:  " << timing.minimum() << " s\n";
  std::cout << "    max:  " << timing.maximum() << " s\n";
  if (timing.mean() > 0.0) {
    std::cout << "    throughput: "
              << static_cast<double>(workItems) / timing.mean() << " "
              << workItemLabel << "/s\n";
  }
}

#if GTSAM_ENABLE_CUDA
enum class SparsePackingPath { Streaming, GaussianGraph };

struct SparsePackingContext {
  gtsam::cuda::SparseJacobianColumnLayout columns;
  gtsam::cuda::SparseJacobianPlan plan;
  gtsam::cuda::HostSparseJacobian host;
  gtsam::cuda::StreamingSparseJacobianLinearizer linearizer;

  SparsePackingContext(const NonlinearFactorGraph& graph, const Values& values)
      : columns(values), plan(graph, columns), host(plan) {}
};

struct SparsePackingBenchmark {
  TimingSamples hostClear;
  TimingSamples combinedWall;
  TimingSamples streamingWall;
  TimingSamples factorLinearizationCpuSum;
  TimingSamples csrPackingCpuSum;
  TimingSamples gaussianGraphLinearization;
  TimingSamples gaussianGraphPacking;
  size_t factors = 0;
  int residualRows = 0;
  int scalarColumns = 0;
  int jacobianNonzeros = 0;
  gtsam::cuda::StreamingLinearizationStats streamingStats;
};

void requireDirectJacobianSuccess(
    const char* operation, const gtsam::cuda::DirectJacobianStatus& status) {
  if (status.ok()) return;

  std::string detail = std::string(operation) + " failed";
  if (status.factorIndex != std::numeric_limits<size_t>::max()) {
    detail += " at factor " + std::to_string(status.factorIndex);
  }
  if (!status.detail.empty()) detail += ": " + status.detail;
  throw std::runtime_error(detail);
}

SparsePackingBenchmark benchmarkSparsePacking(const NonlinearFactorGraph& graph,
                                              const Values& values,
                                              size_t repeats,
                                              SparsePackingPath path) {
  SparsePackingContext context(graph, values);
  SparsePackingBenchmark benchmark;
  benchmark.factors = graph.size();
  benchmark.residualRows = context.plan.rows();
  benchmark.scalarColumns = context.plan.columns();
  benchmark.jacobianNonzeros = context.plan.nonzeros();
  benchmark.hostClear.seconds.reserve(repeats);
  benchmark.combinedWall.seconds.reserve(repeats);
  if (path == SparsePackingPath::Streaming) {
    benchmark.streamingWall.seconds.reserve(repeats);
    benchmark.factorLinearizationCpuSum.seconds.reserve(repeats);
    benchmark.csrPackingCpuSum.seconds.reserve(repeats);
  } else {
    benchmark.gaussianGraphLinearization.seconds.reserve(repeats);
    benchmark.gaussianGraphPacking.seconds.reserve(repeats);
  }

  const auto runOnce = [&](bool measured) {
    const auto clearStart = std::chrono::steady_clock::now();
    context.host.clear();
    const auto clearEnd = std::chrono::steady_clock::now();
    if (measured) {
      benchmark.hostClear.seconds.push_back(
          std::chrono::duration<double>(clearEnd - clearStart).count());
    }

    const auto combinedStart = std::chrono::steady_clock::now();
    if (path == SparsePackingPath::Streaming) {
      gtsam::cuda::StreamingLinearizationStats stats;
      gtsam::cuda::StreamingLinearizationProfile profile;
      const auto streamingStart = std::chrono::steady_clock::now();
      const gtsam::cuda::DirectJacobianStatus status =
          context.linearizer.linearize(graph, values, context.columns,
                                       context.plan, &context.host, &stats,
                                       false, &profile);
      const auto streamingEnd = std::chrono::steady_clock::now();
      requireDirectJacobianSuccess("streaming sparse Jacobian", status);
      if (measured) {
        benchmark.streamingStats = stats;
        benchmark.streamingWall.seconds.push_back(
            std::chrono::duration<double>(streamingEnd - streamingStart)
                .count());
        benchmark.combinedWall.seconds.push_back(
            std::chrono::duration<double>(streamingEnd - combinedStart)
                .count());
        benchmark.factorLinearizationCpuSum.seconds.push_back(
            profile.factorLinearizationCpuSum);
        benchmark.csrPackingCpuSum.seconds.push_back(profile.csrPackingCpuSum);
      }
    } else {
      // Keep each temporary Gaussian graph alive through its corresponding
      // pack, then destroy it before the next repeat.
      const auto linearizationStart = std::chrono::steady_clock::now();
      GaussianFactorGraph::shared_ptr linear = graph.linearize(values);
      const auto linearizationEnd = std::chrono::steady_clock::now();
      const auto packingStart = std::chrono::steady_clock::now();
      const gtsam::cuda::DirectJacobianStatus status =
          context.linearizer.packGaussianFactorGraph(*linear, context.plan,
                                                     &context.host);
      const auto packingEnd = std::chrono::steady_clock::now();
      requireDirectJacobianSuccess("GaussianFactorGraph CSR packing", status);
      if (measured) {
        benchmark.gaussianGraphLinearization.seconds.push_back(
            std::chrono::duration<double>(linearizationEnd - linearizationStart)
                .count());
        benchmark.gaussianGraphPacking.seconds.push_back(
            std::chrono::duration<double>(packingEnd - packingStart).count());
        benchmark.combinedWall.seconds.push_back(
            std::chrono::duration<double>(packingEnd - combinedStart).count());
      }
    }
  };

  runOnce(false);  // Untimed warm-up using the same plan and host storage.
  for (size_t repeat = 0; repeat < repeats; ++repeat) runOnce(true);
  return benchmark;
}

void printSparsePackingBenchmark(const SparsePackingBenchmark& benchmark,
                                 SparsePackingPath path, size_t repeats) {
  std::cout << std::setprecision(9);
  std::cout << "  generic sparse-Jacobian packing benchmark\n";
  std::cout << "  path: "
            << (path == SparsePackingPath::Streaming
                    ? "streaming temporary-factor pack"
                    : "GaussianFactorGraph construction plus identical CSR "
                      "pack")
            << "\n";
  std::cout << "  GTSAM_USE_TBB: " << (gtsamBuiltWithTbb() ? "yes" : "no")
            << "\n";
  std::cout << "  factors: " << benchmark.factors
            << ", residual rows: " << benchmark.residualRows
            << ", scalar columns: " << benchmark.scalarColumns
            << ", J.nnz: " << benchmark.jacobianNonzeros << "\n";
  std::cout << "  warm-up calls: 1 (not timed)\n";
  std::cout << "  measured repeats: " << repeats << "\n";
  printTimingSamples("host buffer clear", benchmark.hostClear,
                     static_cast<size_t>(benchmark.jacobianNonzeros) +
                         static_cast<size_t>(benchmark.residualRows),
                     "scalars");
  if (path == SparsePackingPath::Streaming) {
    std::cout << "  scheduling: " << benchmark.streamingStats.sendableFactors
              << " sendable, " << benchmark.streamingStats.nonSendableFactors
              << " non-sendable factors\n";
    printTimingSamples("streaming factor linearization + CSR pack wall",
                       benchmark.streamingWall, benchmark.factors, "factors");
    printTimingSamples("sum of per-factor linearize CPU time",
                       benchmark.factorLinearizationCpuSum, benchmark.factors,
                       "factors");
    printTimingSamples("sum of per-factor CSR scatter CPU time",
                       benchmark.csrPackingCpuSum, benchmark.factors,
                       "factors");
  } else {
    printTimingSamples("GaussianFactorGraph graph.linearize() wall",
                       benchmark.gaussianGraphLinearization, benchmark.factors,
                       "factors");
    printTimingSamples("GaussianFactorGraph-to-CSR pack wall",
                       benchmark.gaussianGraphPacking, benchmark.factors,
                       "factors");
  }
  printTimingSamples("combined path wall (excluding host clear)",
                     benchmark.combinedWall, benchmark.factors, "factors");
  std::cout << std::setprecision(6);
}
#endif

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
enum class CudaLmDefaults {
  Backend,
  Graph,
};

gtsam::cuda::CudaSfmLevenbergMarquardtParams makeBalCudaLmParams(
    CudaLinearSolverOption solverOption, CudaLmDefaults defaults,
    bool enableDetailedProfiling) {
  gtsam::cuda::CudaSfmLevenbergMarquardtParams params =
      defaults == CudaLmDefaults::Graph
          ? gtsam::cuda::CudaSfmLevenbergMarquardtParams::CeresDefaults()
          : gtsam::cuda::CudaSfmLevenbergMarquardtParams();
  applyBalBenchmarkLmSettings(params);
  params.setLinearSolver(cudaLinearSolverName(solverOption));
  if (solverOption == CudaLinearSolverOption::PcgSchur ||
      solverOption == CudaLinearSolverOption::PcgFullNormal) {
    params.pcg.relativeTolerance = 1e-10;
    params.pcg.maxIterations = 1000;
  }
  params.enableDetailedProfiling = enableDetailedProfiling;
  return params;
}

void applyCudaMatrixOrdering(
    const SfmData& data, const RunOptions& options,
    gtsam::cuda::CudaSfmLevenbergMarquardtParams* params) {
  if (!params || options.ordering != "gtsam") return;
  if (options.cudaLinearSolver != CudaLinearSolverOption::CudssSchur &&
      options.cudaLinearSolver != CudaLinearSolverOption::CudssFullNormal) {
    throw std::invalid_argument(
        "GTSAM ordering is supported only by the cuDSS backend");
  }
  if (options.cudaLinearSolver == CudaLinearSolverOption::CudssSchur) {
    std::vector<Key> cameraKeys;
    cameraKeys.reserve(data.numberCameras());
    for (size_t index = 0; index < data.numberCameras(); ++index) {
      cameraKeys.push_back(C(index));
    }
    params->setOrdering(
        gtsam::cuda::CudaSfmReducedCsrPlan(data, cameraKeys).colamdOrdering());
  } else {
    const NonlinearFactorGraph graph =
        bal::buildGeneralSfmGraph(data, options.config);
    params->setOrdering(Ordering::Create(Ordering::COLAMD, graph));
  }
}

struct GpuLinearizationBenchmark {
  TimingSamples projectionLinearization;
  TimingSamples hessianDiagonal;
  TimingSamples denseSchurCombined;
  size_t observations = 0;
  size_t deltaDimension = 0;
};

GpuLinearizationBenchmark benchmarkGpuLinearization(const SfmData& db,
                                                    size_t repeats) {
  gtsam::cuda::CudaContext context;
  const auto values = gtsam::cuda::PackSfmValues(db, context.stream());
  const auto batch =
      gtsam::cuda::CudaSfmProjectionBatch::FromSfmData(db, context.stream());
  context.synchronize();

  GpuLinearizationBenchmark benchmark;
  benchmark.observations = batch.numObservations();
  benchmark.projectionLinearization.seconds.reserve(repeats);
  benchmark.hessianDiagonal.seconds.reserve(repeats);
  benchmark.denseSchurCombined.seconds.reserve(repeats);

  gtsam::cuda::CudaSfmProjectionLinearization linearization;
  gtsam::cuda::LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                               context.stream());
  context.synchronize();
  for (size_t repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    gtsam::cuda::LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                                 context.stream());
    context.synchronize();
    const auto end = std::chrono::steady_clock::now();
    benchmark.projectionLinearization.seconds.push_back(
        std::chrono::duration<double>(end - start).count());
  }

  const auto params = makeBalCudaLmParams(CudaLinearSolverOption::DenseSchur,
                                          CudaLmDefaults::Graph, false);
  gtsam::cuda::CudaDeviceArray<double> dampingDiagonal;
  if (params.dampingParams.diagonalDamping) {
    const auto computeHessianDiagonal = [&]() {
      gtsam::cuda::ComputeCudaSfmHessianDiagonal(
          values, batch, static_cast<int>(db.numberCameras()),
          params.dampingParams.minDiagonal, params.dampingParams.maxDiagonal,
          &dampingDiagonal,
          context.stream());
    };
    computeHessianDiagonal();
    context.synchronize();
    for (size_t repeat = 0; repeat < repeats; ++repeat) {
      const auto start = std::chrono::steady_clock::now();
      computeHessianDiagonal();
      context.synchronize();
      const auto end = std::chrono::steady_clock::now();
      benchmark.hessianDiagonal.seconds.push_back(
          std::chrono::duration<double>(end - start).count());
    }
  }

  gtsam::cuda::CudaSfmDenseSchurSolver solver;
  gtsam::cuda::CudaDeviceArray<double> delta;
  const auto solveOnce = [&]() {
    if (params.dampingParams.diagonalDamping) {
      solver.solve(values, batch, static_cast<int>(db.numberCameras()),
                   params.lambdaInitial, dampingDiagonal, &delta,
                   context.stream());
    } else {
      solver.solve(values, batch, static_cast<int>(db.numberCameras()),
                   params.lambdaInitial, &delta, context.stream());
    }
  };

  solveOnce();
  context.synchronize();
  for (size_t repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    solveOnce();
    context.synchronize();
    const auto end = std::chrono::steady_clock::now();
    benchmark.denseSchurCombined.seconds.push_back(
        std::chrono::duration<double>(end - start).count());
  }
  benchmark.deltaDimension = delta.size();
  return benchmark;
}

struct CudaBackendLmRun {
  double elapsed = 0.0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult result;
};

CudaBackendLmRun runCudaBackendLm(
    const SfmData& db,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params) {
  CudaBackendLmRun run;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    run.result = gtsam::cuda::OptimizeCudaSfmWithoutValueDownload(db, params);
  });
  return run;
}

std::string jsonEscape(const std::string& value) {
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        output << character;
    }
  }
  return output.str();
}

void printCudaMatrixResult(const RunOptions& options,
                           const std::string& dataset,
                           const CudaBackendLmRun& run,
                           bool printCsvHeader) {
  const auto& result = run.result;
  const auto& stats = result.linearSolveStats;
  const char* formulation =
      result.formulation == gtsam::cuda::CudaSfmSystemFormulation::Schur
          ? "schur"
          : "full-normal";
  const char* backend = "dense-cholesky";
  if (result.linearBackend == gtsam::cuda::CudaLinearSolverType::Cudss) {
    backend = "cudss";
  } else if (result.linearBackend ==
             gtsam::cuda::CudaLinearSolverType::Pcg) {
    backend = "pcg";
  }

  if (options.outputFormat == "json") {
    std::cout << std::setprecision(17)
              << "{\"configuration\":\""
              << jsonEscape(options.matrixConfiguration)
              << "\",\"dataset\":\"" << jsonEscape(dataset)
              << "\",\"formulation\":\"" << formulation
              << "\",\"backend\":\"" << backend
              << "\",\"ordering\":\"" << options.ordering
              << "\",\"dimension\":" << result.linearSystemDimension
              << ",\"nnz\":" << result.linearSystemNonzeros
              << ",\"matrix_free\":"
              << (result.linearBackend == gtsam::cuda::CudaLinearSolverType::Pcg
                      ? "true"
                      : "false")
              << ",\"analysis_count\":" << stats.analysisCount
              << ",\"factorization_count\":" << stats.factorizationCount
              << ",\"solve_count\":" << stats.solveCount
              << ",\"pcg_iterations\":" << stats.pcgIterationsTotal
              << ",\"pcg_host_convergence_checks\":"
              << stats.pcgHostConvergenceChecks
              << ",\"pcg_converged\":"
              << (stats.lastPcgConverged ? "true" : "false")
              << ",\"initial_objective\":" << result.initialError
              << ",\"final_objective\":" << result.finalError
              << ",\"h2d_bytes\":" << result.totalH2dBytes
              << ",\"d2h_bytes\":" << result.totalD2hBytes
              << ",\"frontend_wall_seconds\":" << run.elapsed
              << ",\"analysis_seconds\":" << stats.analysisSeconds
              << ",\"factorization_seconds\":"
              << stats.factorizationSeconds << ",\"solve_seconds\":"
              << stats.solveSeconds << ",\"accepted_iterations\":"
              << result.acceptedSteps << ",\"lambda_attempts\":"
              << result.innerIterations << "}\n";
    return;
  }
  if (options.outputFormat == "csv") {
    if (printCsvHeader) {
      std::cout
          << "configuration,dataset,formulation,backend,ordering,dimension,"
             "nnz,matrix_free,analysis_count,factorization_count,solve_count,"
             "pcg_iterations,pcg_host_convergence_checks,pcg_converged,"
             "initial_objective,final_objective,"
             "h2d_bytes,d2h_bytes,frontend_wall_seconds,analysis_seconds,"
             "factorization_seconds,solve_seconds,accepted_iterations,"
             "lambda_attempts\n";
    }
    std::cout << std::setprecision(17) << options.matrixConfiguration << ","
              << dataset << "," << formulation << "," << backend << ","
              << options.ordering << "," << result.linearSystemDimension << ","
              << result.linearSystemNonzeros << ","
              << (result.linearBackend == gtsam::cuda::CudaLinearSolverType::Pcg)
              << "," << stats.analysisCount << "," << stats.factorizationCount
              << "," << stats.solveCount << "," << stats.pcgIterationsTotal
              << "," << stats.pcgHostConvergenceChecks << ","
              << stats.lastPcgConverged << "," << result.initialError
              << "," << result.finalError << "," << result.totalH2dBytes << ","
              << result.totalD2hBytes << "," << run.elapsed << ","
              << stats.analysisSeconds << "," << stats.factorizationSeconds
              << "," << stats.solveSeconds << "," << result.acceptedSteps
              << "," << result.innerIterations << "\n";
    return;
  }
}

const char* yesNo(bool value) { return value ? "yes" : "no"; }

void printTransferRow(const std::string& indent, const std::string& label,
                      double elapsed, size_t bytes, double total) {
  std::cout << indent << std::left << std::setw(30) << label << std::right
            << elapsed << " s";
  if (total > 0.0) {
    std::cout << " (" << (100.0 * elapsed / total) << "%)";
  }
  std::cout << " [" << bytes << " B";
  if (elapsed > 0.0 && bytes > 0) {
    const double gib = static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
    std::cout << ", " << (gib / elapsed) << " GiB/s";
  }
  std::cout << "]\n";
}

void printCudaLmSetupBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  std::cout << prefix << "stage breakdown:\n";
  std::cout << "    context: " << result.contextElapsed << " s\n";
  std::cout << "    pack values: " << result.packValuesElapsed << " s\n";
  std::cout << "    allocate trial values: " << result.allocateTrialElapsed
            << " s\n";
  std::cout << "    projection batch: " << result.projectionBatchElapsed
            << " s\n";
  std::cout << "    initial error: " << result.initialErrorElapsed << " s\n";
  std::cout << "    cuDSS solver construction: "
            << result.cudssSolverConstructionElapsed << " s\n";
  std::cout << "    dense Schur solver construction: "
            << result.denseSchurSolverConstructionElapsed << " s\n";
  std::cout << "    CSR structure: " << result.csrStructureElapsed << " s\n";
  std::cout << "    upload pattern: " << result.uploadPatternElapsed << " s\n";
  std::cout << "    first cuDSS analyze (solve loop): "
            << result.firstCudssAnalyzeElapsed << " s\n";
  std::cout << "    download values (post solve): " << result.downloadElapsed
            << " s\n";
}

void printCudaLmTransferBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  const double total = result.totalMeasuredElapsed;
  std::cout << prefix << "pure transfer breakdown:\n";
  printProfileRow("    ", "pack values host build",
                  result.packValuesHostBuildElapsed, total);
  printProfileRow("    ", "pack values device alloc",
                  result.packValuesDeviceAllocElapsed, total);
  printTransferRow("    ", "pack values H2D memcpy",
                   result.packValuesH2dCopyElapsed, result.packValuesH2dBytes,
                   total);
  printProfileRow("    ", "projection host build",
                  result.projectionBatchHostBuildElapsed, total);
  printProfileRow("    ", "projection device alloc",
                  result.projectionBatchDeviceAllocElapsed, total);
  printTransferRow("    ", "projection H2D memcpy",
                   result.projectionBatchH2dCopyElapsed,
                   result.projectionBatchH2dBytes, total);
  printProfileRow("    ", "CSR pattern device alloc",
                  result.uploadPatternDeviceAllocElapsed, total);
  printTransferRow("    ", "CSR pattern H2D memcpy",
                   result.uploadPatternH2dCopyElapsed,
                   result.uploadPatternH2dBytes, total);
  printTransferRow("    ", "total H2D memcpy", result.totalH2dCopyElapsed,
                   result.totalH2dBytes, total);
  printProfileRow("    ", "download host alloc",
                  result.downloadHostAllocElapsed, total);
  printTransferRow("    ", "download D2H memcpy", result.downloadD2hCopyElapsed,
                   result.downloadD2hBytes, total);
  printProfileRow("    ", "download Values rebuild",
                  result.downloadValuesBuildElapsed, total);
  printTransferRow("    ", "total D2H memcpy", result.totalD2hCopyElapsed,
                   result.totalD2hBytes, total);
}

void printCudaLmDetailedBreakdown(
    const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result,
    const std::string& prefix) {
  const double solveLoop = result.solveLoopElapsed;
  const double accounted =
      result.dampingDiagonalElapsed + result.denseSchurSolveElapsed +
      result.normalEquationsElapsed + result.addDampingElapsed +
      result.cudssAnalyzeElapsed + result.cudssSolveElapsed +
      result.linearizedErrorElapsed + result.applyDeltaElapsed +
      result.trialErrorElapsed + result.acceptTrialElapsed +
      result.lambdaUpdateElapsed;

  std::cout << prefix << "solve-loop aggregate breakdown:\n";
  printProfileRow("    ", "damping diagonal", result.dampingDiagonalElapsed,
                  solveLoop);
  printProfileRow("    ", "dense Schur solve", result.denseSchurSolveElapsed,
                  solveLoop);
  printProfileRow("    ", "normal equations", result.normalEquationsElapsed,
                  solveLoop);
  printProfileRow("    ", "add diagonal damping", result.addDampingElapsed,
                  solveLoop);
  printProfileRow("    ", "cuDSS analyze", result.cudssAnalyzeElapsed,
                  solveLoop);
  printProfileRow("    ", "cuDSS solve", result.cudssSolveElapsed, solveLoop);
  printProfileRow("    ", "linearized error change",
                  result.linearizedErrorElapsed, solveLoop);
  printProfileRow("    ", "apply delta", result.applyDeltaElapsed, solveLoop);
  printProfileRow("    ", "trial projection error", result.trialErrorElapsed,
                  solveLoop);
  printProfileRow("    ", "accept trial copy", result.acceptTrialElapsed,
                  solveLoop);
  printProfileRow("    ", "lambda/convergence update",
                  result.lambdaUpdateElapsed, solveLoop);
  printProfileRow("    ", "profiled subtotal", accounted, solveLoop);
  printProfileRow("    ", "unprofiled loop time", solveLoop - accounted,
                  solveLoop);

  std::cout << prefix << "per-iteration breakdown:\n";
  if (result.iterationProfiles.empty()) {
    std::cout << "    no LM iterations recorded\n";
    return;
  }
  for (const auto& iteration : result.iterationProfiles) {
    std::cout << "    iteration " << iteration.iteration << ": total "
              << iteration.totalElapsed << " s, attempts "
              << iteration.attemptProfiles.size()
              << ", accepted: " << yesNo(iteration.acceptedStep)
              << ", terminated: " << yesNo(iteration.terminated) << "\n";
    std::cout << "      error: " << std::setprecision(15)
              << iteration.startError << " -> " << iteration.endError
              << std::setprecision(6) << "\n";
    std::cout << "      lambda: " << iteration.startLambda << " -> "
              << iteration.endLambda << "\n";
    printProfileRow("      ", "damping diagonal",
                    iteration.dampingDiagonalElapsed, iteration.totalElapsed);
    printProfileRow("      ", "accept trial copy", iteration.acceptTrialElapsed,
                    iteration.totalElapsed);

    for (const auto& attempt : iteration.attemptProfiles) {
      std::cout << "      attempt " << attempt.attempt << ": total "
                << attempt.totalElapsed << " s, lambda " << attempt.lambda
                << ", accepted: " << yesNo(attempt.accepted)
                << ", step successful: " << yesNo(attempt.stepSuccessful)
                << ", attempted trial: " << yesNo(attempt.attemptedTrial)
                << ", terminated: " << yesNo(attempt.terminated) << "\n";
      std::cout << "        costs: linearized change "
                << attempt.linearizedCostChange << ", actual change "
                << attempt.costChange << ", fidelity " << attempt.modelFidelity
                << ", trial error " << std::setprecision(15)
                << attempt.trialError << std::setprecision(6) << "\n";
      std::cout << "        flags: stop lambda search "
                << yesNo(attempt.stopSearchingLambda) << ", lambda upper bound "
                << yesNo(attempt.lambdaUpperBoundReached) << "\n";
      printProfileRow("        ", "dense Schur solve",
                      attempt.denseSchurSolveElapsed, attempt.totalElapsed);
      printProfileRow("        ", "normal equations",
                      attempt.normalEquationsElapsed, attempt.totalElapsed);
      printProfileRow("        ", "add diagonal damping",
                      attempt.addDampingElapsed, attempt.totalElapsed);
      printProfileRow("        ", "cuDSS analyze", attempt.cudssAnalyzeElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "cuDSS solve", attempt.cudssSolveElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "linearized error change",
                      attempt.linearizedErrorElapsed, attempt.totalElapsed);
      printProfileRow("        ", "apply delta", attempt.applyDeltaElapsed,
                      attempt.totalElapsed);
      printProfileRow("        ", "trial projection error",
                      attempt.trialErrorElapsed, attempt.totalElapsed);
      printProfileRow("        ", "lambda/convergence update",
                      attempt.lambdaUpdateElapsed, attempt.totalElapsed);
    }
  }
}

void printCudaBackendLmRun(const CudaBackendLmRun& run,
                           CudaLinearSolverOption solverOption,
                           bool detailedProfiling) {
  const auto& result = run.result;
  std::cout << "  CUDA LM: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM linear solver: " << cudaLinearSolverName(solverOption)
            << "\n";
  std::cout << "  CUDA LM solve loop: " << result.solveLoopElapsed << " s\n";
  std::cout << "  CUDA LM measured total: " << result.totalMeasuredElapsed
            << " s\n";
  std::cout << "  CUDA LM setup before solve loop: " << result.setupElapsed
            << " s\n";
  if (detailedProfiling) {
    printCudaLmSetupBreakdown(result, "  CUDA LM ");
    printCudaLmTransferBreakdown(result, "  CUDA LM ");
    printCudaLmDetailedBreakdown(result, "  CUDA LM ");
  } else {
    std::cout << "  CUDA LM detailed profiling: disabled (use --profile to "
                 "enable)\n";
  }
  std::cout << "Initial error: " << std::setprecision(15) << result.initialError
            << "\n";
  std::cout << "Final error: " << result.finalError
            << ", iterations: " << result.iterations
            << ", accepted: " << result.acceptedSteps << std::setprecision(6)
            << "\n";
}

struct CudaGraphLmRun {
  double elapsed = 0.0;
  double optimizerConstructionElapsed = 0.0;
  double optimizeElapsed = 0.0;
  double resultQueryElapsed = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  gtsam::cuda::CudaSfmLevenbergMarquardtResult backend;
};

CudaGraphLmRun runCudaGraphLm(
    const NonlinearFactorGraph& graph, const Values& initial,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params) {
  CudaGraphLmRun run;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    std::optional<gtsam::cuda::CudaSfmLevenbergMarquardtOptimizer> optimizer;
    run.optimizerConstructionElapsed = gtsam::timing::measureSeconds(
        [&] { optimizer.emplace(graph, initial, params); });
    run.optimizeElapsed = gtsam::timing::measureSeconds([&] {
      const Values& optimized = optimizer->optimize();
      (void)optimized;
    });
    run.resultQueryElapsed = gtsam::timing::measureSeconds([&] {
      run.initialError = optimizer->result().initialError;
      run.finalError = optimizer->error();
      run.iterations = optimizer->iterations();
      run.backend = optimizer->result();
    });
  });
  return run;
}

void printCudaGraphLmRun(const CudaGraphLmRun& run,
                         CudaLinearSolverOption solverOption,
                         CudaGraphKind graphKind, bool detailedProfiling) {
  const double graphBackendCallOverhead =
      run.backend.graphBackendCallElapsed - run.backend.totalMeasuredElapsed;
  const double graphApiOtherOverhead =
      run.optimizeElapsed - run.backend.totalMeasuredElapsed -
      run.backend.graphConversionElapsed - run.backend.graphValueMergeElapsed;
  const double graphApiRemainingOptimizeOverhead =
      run.optimizeElapsed - run.backend.graphConversionElapsed -
      run.backend.graphBackendCallElapsed - run.backend.graphValueMergeElapsed -
      run.backend.graphConvertedDataDestructionElapsed;

  std::cout << "  CUDA LM graph API: " << run.elapsed << " s\n";
  std::cout << "  CUDA LM graph linear solver: "
            << cudaLinearSolverName(solverOption) << "\n";
  std::cout << "  CUDA LM graph kind: " << cudaGraphKindName(graphKind) << "\n";
  std::cout << "  CUDA LM graph API overhead over backend: "
            << run.elapsed - run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM graph API breakdown:\n";
  printProfileRow("    ", "optimizer construction",
                  run.optimizerConstructionElapsed, run.elapsed);
  printProfileRow("    ", "optimize call", run.optimizeElapsed, run.elapsed);
  std::cout << "    result/error queries (excluded from measured total): "
            << run.resultQueryElapsed << " s\n";
  printProfileRow("    ", "graph conversion",
                  run.backend.graphConversionElapsed, run.elapsed);
  printProfileRow("    ", "backend measured total",
                  run.backend.totalMeasuredElapsed, run.elapsed);
  printProfileRow("    ", "backend return/assignment", graphBackendCallOverhead,
                  run.elapsed);
  printProfileRow("    ", "value merge/state update",
                  run.backend.graphValueMergeElapsed, run.elapsed);
  printProfileRow("    ", "converted data destruction",
                  run.backend.graphConvertedDataDestructionElapsed,
                  run.elapsed);
  printProfileRow("    ", "other graph optimize overhead",
                  graphApiOtherOverhead, run.elapsed);
  printProfileRow("    ", "remaining optimize overhead",
                  graphApiRemainingOptimizeOverhead, run.elapsed);
  std::cout << "  CUDA LM backend solve loop: " << run.backend.solveLoopElapsed
            << " s\n";
  std::cout << "  CUDA LM backend measured total: "
            << run.backend.totalMeasuredElapsed << " s\n";
  std::cout << "  CUDA LM backend setup before solve loop: "
            << run.backend.setupElapsed << " s\n";
  if (detailedProfiling) {
    printCudaLmSetupBreakdown(run.backend, "  CUDA LM backend ");
    printCudaLmTransferBreakdown(run.backend, "  CUDA LM backend ");
    printCudaLmDetailedBreakdown(run.backend, "  CUDA LM backend ");
  } else {
    std::cout << "  CUDA LM backend detailed profiling: disabled (use "
                 "--profile to enable)\n";
  }
  std::cout << "Initial error: " << std::setprecision(15) << run.initialError
            << "\n";
  std::cout << "Final error: " << run.finalError
            << ", iterations: " << run.iterations
            << ", accepted: " << run.backend.acceptedSteps
            << ", inner: " << run.backend.innerIterations
            << std::setprecision(6) << "\n";
}

struct OrdinaryLmComparisonRun {
  double elapsed = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
};

OrdinaryLmComparisonRun runOrdinaryLmComparison(
    const NonlinearFactorGraph& graph, const Values& initial,
    const LevenbergMarquardtParams& params) {
  const auto start = std::chrono::steady_clock::now();
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  (void)optimizer.optimize();
  const auto end = std::chrono::steady_clock::now();

  OrdinaryLmComparisonRun run;
  run.elapsed = std::chrono::duration<double>(end - start).count();
  run.initialError = graph.error(initial);
  run.finalError = optimizer.error();
  run.iterations = optimizer.iterations();
  return run;
}

struct GenericSparseLmRun {
  double elapsed = 0.0;
  gtsam::cuda::CudaSparseLevenbergMarquardtResult result;
};

GenericSparseLmRun runGenericSparseLm(
    const NonlinearFactorGraph& graph, const Values& initial,
    const gtsam::cuda::CudaSparseLevenbergMarquardtParams& params) {
  const auto start = std::chrono::steady_clock::now();
  gtsam::cuda::CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial,
                                                               params);
  (void)optimizer.optimize();
  const auto end = std::chrono::steady_clock::now();

  GenericSparseLmRun run;
  run.elapsed = std::chrono::duration<double>(end - start).count();
  run.result = optimizer.result();
  if (run.result.backend != gtsam::cuda::CudaSparseLmBackend::Cuda) {
    throw std::runtime_error(
        "generic sparse-Jacobian LM did not use the CUDA backend: " +
        run.result.fallbackDetail);
  }
  return run;
}

const char* sparseTerminationName(
    gtsam::cuda::CudaSparseLmTerminationReason reason) {
  using Reason = gtsam::cuda::CudaSparseLmTerminationReason;
  switch (reason) {
    case Reason::None:
      return "none";
    case Reason::ErrorThreshold:
      return "error-threshold";
    case Reason::Converged:
      return "converged";
    case Reason::MaxIterations:
      return "max-iterations";
    case Reason::SmallCostChange:
      return "small-cost-change";
    case Reason::LambdaUpperBound:
      return "lambda-upper-bound";
  }
  return "unknown";
}

// Keep generic optimizer profile formatting isolated here so newly exposed
// device counters can be added without changing benchmark control flow.
void printGenericSparseLmProfile(const GenericSparseLmRun& run) {
  const auto& result = run.result;
  const auto& timing = result.timings;
  const auto& transfers = result.transfers;
  const double total = timing.totalWall;
  std::cout << "  generic sparse-Jacobian CUDA LM result:\n";
  std::cout << "    termination: " << sparseTerminationName(result.termination)
            << "\n";
  std::cout << "    outer linearizations: " << result.outerLinearizations
            << ", iterations: " << result.iterations
            << ", accepted steps: " << result.acceptedSteps
            << ", lambda attempts: " << result.lambdaAttempts
            << ", cuDSS analyses: " << result.cudssAnalyses << "\n";
  std::cout << "    final lambda: " << result.finalLambda << "\n";
  std::cout
      << "    timing note: total wall contains the CUDA prefix; persistent "
         "setup contains device initialization; cuDSS DATA_INFO overlaps "
         "factor + solve; worker CPU sums can exceed wall time. Compatibility "
         "aggregates are omitted below.\n";
  std::cout << "    stage breakdown:\n";
  printProfileRow("      ", "optimizer internal total wall", timing.totalWall,
                  total);
  printProfileRow("      ", "initial nonlinear error", timing.initialError,
                  total);
  printProfileRow("      ", "symbolic plan", timing.plan, total);
  printProfileRow("      ", "persistent setup wall", timing.persistentSetupWall,
                  total);
  printProfileRow("      ", "device initialize wall",
                  timing.deviceInitializeWall, total);
  printProfileRow("      ", "pattern H2D", timing.patternH2d, total);
  printProfileRow("      ", "transpose/H structure setup",
                  timing.structureSetup, total);
  printProfileRow("      ", "setup D2H", timing.setupD2h, total);
  printProfileRow("      ", "host zero", timing.hostZero, total);
  printProfileRow("      ", "factor linearize + pack wall",
                  timing.factorLinearizationAndPackingWall, total);
  printProfileRow("      ", "factor linearize CPU sum",
                  timing.factorLinearizationCpuSum, total);
  printProfileRow("      ", "CSR packing CPU sum", timing.csrPackingCpuSum,
                  total);
  printProfileRow("      ", "numeric H2D", timing.numericH2d, total);
  printProfileRow("      ", "transpose update", timing.transposeUpdate, total);
  printProfileRow("      ", "J^T J", timing.normalJtJ, total);
  printProfileRow("      ", "J^T b", timing.normalJtb, total);
  printProfileRow("      ", "diagonal extraction", timing.diagonalExtraction,
                  total);
  printProfileRow("      ", "old model error", timing.oldModelError, total);
  printProfileRow("      ", "damping preparation", timing.dampingPreparation,
                  total);
  printProfileRow("      ", "cuDSS analysis", timing.cudssAnalysis, total);
  printProfileRow("      ", "damping application", timing.dampingApplication,
                  total);
  printProfileRow("      ", "cuDSS factor + solve", timing.cudssFactorAndSolve,
                  total);
  printProfileRow("      ", "cuDSS DATA_INFO boundary",
                  timing.cudssDataInfoBoundaryWall, total);
  printProfileRow("      ", "new model error", timing.newModelError, total);
  printProfileRow("      ", "attempt D2H", timing.attemptD2h, total);
  printProfileRow("      ", "attempt host result build",
                  timing.attemptHostBuild, total);
  printProfileRow("      ", "Values retract", timing.retract, total);
  printProfileRow("      ", "nonlinear trial error", timing.nonlinearTrialError,
                  total);
  std::cout << "    transfer breakdown:\n";
  printTransferRow("      ", "pattern H2D", timing.patternH2d,
                   transfers.patternH2dBytes, total);
  printTransferRow("      ", "numeric H2D", timing.numericH2d,
                   transfers.numericH2dBytes, total);
  printTransferRow("      ", "setup D2H", timing.setupD2h,
                   transfers.setupD2hBytes, total);
  printTransferRow("      ", "attempt D2H", timing.attemptD2h,
                   transfers.attemptD2hBytes, total);
  std::cout << "      total H2D: " << transfers.totalH2dBytes() << " B\n";
  std::cout << "      total D2H: " << transfers.totalD2hBytes() << " B\n";
}

void printSparseLmComparison(const OrdinaryLmComparisonRun& ordinary,
                             const CudaGraphLmRun& specialized,
                             const GenericSparseLmRun& generic) {
  const double genericObjective = generic.result.finalError;
  const double specializedObjective = specialized.finalError;
  if (!std::isfinite(ordinary.finalError) ||
      !std::isfinite(specializedObjective) ||
      !std::isfinite(genericObjective)) {
    throw std::runtime_error("LM comparison produced a non-finite objective");
  }

  const double objectiveTolerance =
      1e-8 *
      std::max({1.0, std::abs(ordinary.finalError),
                std::abs(specializedObjective), std::abs(genericObjective)});
  const double genericObjectiveDifference =
      std::abs(ordinary.finalError - genericObjective);
  const double specializedObjectiveDifference =
      std::abs(ordinary.finalError - specializedObjective);
  if (genericObjectiveDifference > objectiveTolerance) {
    throw std::runtime_error(
        "generic sparse-Jacobian LM final objective differs from ordinary "
        "GTSAM LM");
  }
  if (specializedObjectiveDifference > objectiveTolerance) {
    throw std::runtime_error(
        "specialized CUDA SFM LM final objective differs from ordinary "
        "GTSAM LM");
  }

  const auto& size = generic.result.systemSize;
  std::cout << std::setprecision(9);
  std::cout << "  sparse-Jacobian CUDA LM comparison\n";
  std::cout
      << "  untimed warm-ups: 1 ordinary CPU, 1 specialized CUDA, 1 generic "
         "sparse CUDA\n";
  std::cout << "  GTSAM_USE_TBB: " << (gtsamBuiltWithTbb() ? "yes" : "no")
            << "\n";
  std::cout << "  factors: " << size.factors
            << ", residual rows: " << size.jacobianRows
            << ", scalar columns: " << size.jacobianColumns
            << ", J.nnz: " << size.jacobianNonzeros
            << ", H.nnz: " << size.normalNonzeros << "\n";
  std::cout << "  ordinary GTSAM LM total: " << ordinary.elapsed << " s"
            << " (error " << ordinary.finalError << ", iterations "
            << ordinary.iterations << ")\n";
  std::cout << "  existing CUDA SFM specialized LM total: "
            << specialized.elapsed << " s (error " << specialized.finalError
            << ", iterations " << specialized.iterations << ")\n";
  std::cout << "  generic sparse-Jacobian CUDA LM total: " << generic.elapsed
            << " s (error " << genericObjective << ", accepted "
            << generic.result.acceptedSteps << ", attempts "
            << generic.result.lambdaAttempts << ")\n";
  std::cout << "  specialized/ordinary objective difference: "
            << specializedObjectiveDifference << "\n";
  std::cout << "  generic/ordinary objective difference: "
            << genericObjectiveDifference << " (shared tolerance "
            << objectiveTolerance << ")\n";
  printGenericSparseLmProfile(generic);
  std::cout << std::setprecision(6);
}
#endif

void writeBenchmarkActionJson(const std::vector<TimingRow>& rows,
                              const std::string& outputPath) {
  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset + "/";
    metrics.push_back({prefix + "MultifrontalCholesky", "s", row.legacy});
    metrics.push_back({prefix + "MultifrontalSolver", "s", row.newer});
  }
  gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
}
RunOptions parseBalFiles(int argc, char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  RunOptions options;
  options.config.useSchur = !arguments.flag("--colamd");
  options.profile = arguments.flag("--profile");
  options.cudaStructureOnly = arguments.flag("--cuda-structure-only");
  options.cudaLm = arguments.flag("--cuda-lm");
  options.cudaLmGraph = arguments.flag("--cuda-lm-graph");
  options.cudaSparseLm = arguments.flag("--cuda-sparse-lm");
  options.cudaSparsePackOnly = arguments.flag("--cuda-sparse-pack-only");
  options.gaussianGraphPackDiagnostic =
      arguments.flag("--gaussian-graph-pack-diagnostic");
  options.linearizationBenchmark =
      arguments.flag("--linearization-benchmark");
  options.listConfigurations = arguments.flag("--list-configurations");
  options.dryRun = arguments.flag("--dry-run");
  options.help = arguments.helpRequested();

  const auto linearizationRepeats =
      arguments.optionalString("--linearization-repeats");
  options.linearizationRepeatsSpecified = linearizationRepeats.has_value();
  if (linearizationRepeats) {
    gtsam::timing::Arguments valueArguments(
        {"--value", *linearizationRepeats});
    options.linearizationRepeats = valueArguments.sizeValue(
        "--value", options.linearizationRepeats);
  }

  const auto configuration = arguments.optionalString("--configuration");
  const auto formulation = arguments.optionalString("--formulation");
  const auto linearSolver = arguments.optionalString("--cuda-linear-solver");
  const auto orderingMode = arguments.optionalString("--ordering");
  const auto outputFormat = arguments.optionalString("--output-format");
  options.cudaLinearSolverSpecified =
      configuration.has_value() || formulation.has_value() ||
      linearSolver.has_value() || orderingMode.has_value();

  if (orderingMode) options.ordering = *orderingMode;
  if (options.ordering != "auto" && options.ordering != "gtsam") {
    throw std::runtime_error("--ordering requires auto or gtsam");
  }
  if (outputFormat) options.outputFormat = *outputFormat;
  if (options.outputFormat != "text" && options.outputFormat != "csv" &&
      options.outputFormat != "json") {
    throw std::runtime_error("--output-format requires text, csv, or json");
  }

  auto selectConfiguration = [&](const std::string& name) {
    options.matrixConfiguration = name;
    if (name == "schur-dense") {
      options.cudaLinearSolver = CudaLinearSolverOption::DenseSchur;
      options.ordering = "auto";
    } else if (name == "schur-cudss-auto") {
      options.cudaLinearSolver = CudaLinearSolverOption::CudssSchur;
      options.ordering = "auto";
    } else if (name == "schur-cudss-gtsam") {
      options.cudaLinearSolver = CudaLinearSolverOption::CudssSchur;
      options.ordering = "gtsam";
    } else if (name == "schur-pcg") {
      options.cudaLinearSolver = CudaLinearSolverOption::PcgSchur;
      options.ordering = "auto";
    } else if (name == "full-normal-cudss-auto") {
      options.cudaLinearSolver = CudaLinearSolverOption::CudssFullNormal;
      options.ordering = "auto";
    } else if (name == "full-normal-cudss-gtsam") {
      options.cudaLinearSolver = CudaLinearSolverOption::CudssFullNormal;
      options.ordering = "gtsam";
    } else if (name == "full-normal-pcg") {
      options.cudaLinearSolver = CudaLinearSolverOption::PcgFullNormal;
      options.ordering = "auto";
    } else {
      throw std::runtime_error("unknown CUDA SFM configuration: " + name);
    }
  };

  if (configuration) {
    selectConfiguration(*configuration);
    if (orderingMode && options.ordering != *orderingMode) {
      throw std::runtime_error(
          "--configuration and --ordering select different modes");
    }
  } else {
    std::string formulationName = formulation.value_or("schur");
    if (formulationName == "full_normal") formulationName = "full-normal";
    if (formulationName != "schur" && formulationName != "full-normal") {
      throw std::runtime_error(
          "--formulation requires schur or full-normal");
    }
    const std::string backend = linearSolver.value_or("dense-cholesky");
    if (backend == "dense-schur" || backend == "cudss-schur" ||
        backend == "pcg-schur" || backend == "cudss-full-normal" ||
        backend == "pcg-full-normal") {
      if (formulation || orderingMode) {
        throw std::runtime_error(
            "combined --cuda-linear-solver names cannot be mixed with "
            "--formulation or --ordering; use backend-only names");
      }
      if (backend == "dense-schur") selectConfiguration("schur-dense");
      if (backend == "cudss-schur")
        selectConfiguration("schur-cudss-auto");
      if (backend == "pcg-schur") selectConfiguration("schur-pcg");
      if (backend == "cudss-full-normal")
        selectConfiguration("full-normal-cudss-auto");
      if (backend == "pcg-full-normal")
        selectConfiguration("full-normal-pcg");
    } else if (backend == "dense" || backend == "dense-cholesky") {
      if (formulationName != "schur") {
        throw std::runtime_error(
            "dense Cholesky does not support full-normal SFM systems");
      }
      selectConfiguration("schur-dense");
    } else if (backend == "cudss") {
      selectConfiguration(formulationName == "schur"
                              ? (options.ordering == "gtsam"
                                     ? "schur-cudss-gtsam"
                                     : "schur-cudss-auto")
                              : (options.ordering == "gtsam"
                                     ? "full-normal-cudss-gtsam"
                                     : "full-normal-cudss-auto"));
    } else if (backend == "pcg") {
      if (options.ordering != "auto") {
        throw std::runtime_error(
            "GTSAM ordering is supported only by the cuDSS backend");
      }
      selectConfiguration(formulationName == "schur" ? "schur-pcg"
                                                       : "full-normal-pcg");
    } else if (linearSolver) {
      throw std::runtime_error(usage());
    }
  }

  const auto graphKind = arguments.optionalString("--cuda-lm-graph-kind");
  options.cudaGraphKindSpecified = graphKind.has_value();
  if (graphKind == "raw") {
    options.cudaGraphKind = CudaGraphKind::Raw;
  } else if (graphKind == "point-batch") {
    options.cudaGraphKind = CudaGraphKind::PointBatch;
  } else if (graphKind == "camera-batch") {
    options.cudaGraphKind = CudaGraphKind::CameraBatch;
  } else if (graphKind) {
    throw std::runtime_error(usage());
  }

  const auto batchChunkSize = arguments.optionalString("--batch-chunk-size");
  options.batchChunkSizeSpecified = batchChunkSize.has_value();
  if (batchChunkSize) {
    gtsam::timing::Arguments valueArguments({"--value", *batchChunkSize});
    options.batchChunkSize = valueArguments.sizeValue("--value", 0);
  }

  const auto warmupFile = arguments.optionalString("--cuda-warmup-file");
  options.cudaWarmupFileSpecified = warmupFile.has_value();
  options.cudaWarmupFile = warmupFile.value_or("");

  const auto projectionNoise = arguments.optionalString("--projection-noise");
  const bool projectionNoiseSpecified = projectionNoise.has_value();
  if (projectionNoise) {
    setProjectionNoiseModel(&options.config, *projectionNoise);
  }

  const auto gncBackend = arguments.optionalString("--gnc");
  if (gncBackend == "cpu") {
    options.gnc.backend = GncBackend::Cpu;
  } else if (gncBackend == "cuda") {
    options.gnc.backend = GncBackend::Cuda;
  } else if (gncBackend) {
    throw std::runtime_error(usage());
  }

  bool gncOptionSpecified = false;
  const auto gncLoss = arguments.optionalString("--gnc-loss");
  gncOptionSpecified |= gncLoss.has_value();
  if (gncLoss == "tls") {
    options.gnc.lossType = GncLossType::TLS;
  } else if (gncLoss == "gm") {
    options.gnc.lossType = GncLossType::GM;
  } else if (gncLoss) {
    throw std::runtime_error(usage());
  }

  const auto outlierFraction =
      arguments.optionalString("--gnc-outlier-fraction");
  gncOptionSpecified |= outlierFraction.has_value();
  if (outlierFraction) {
    gtsam::timing::Arguments valueArguments({"--value", *outlierFraction});
    options.gnc.outlierFraction =
        valueArguments.doubleValue("--value", options.gnc.outlierFraction);
  }
  if (options.gnc.outlierFraction < 0.0 || options.gnc.outlierFraction >= 1.0) {
    throw std::runtime_error("--gnc-outlier-fraction must be in [0, 1)");
  }

  const auto outlierPixels = arguments.optionalString("--gnc-outlier-pixels");
  gncOptionSpecified |= outlierPixels.has_value();
  if (outlierPixels) {
    gtsam::timing::Arguments valueArguments({"--value", *outlierPixels});
    options.gnc.outlierPixels =
        valueArguments.doubleValue("--value", options.gnc.outlierPixels);
  }

  const auto gncSeed = arguments.optionalString("--gnc-seed");
  gncOptionSpecified |= gncSeed.has_value();
  if (gncSeed) {
    gtsam::timing::Arguments valueArguments({"--value", *gncSeed});
    options.gnc.seed = static_cast<unsigned int>(
        valueArguments.uint64Value("--value", options.gnc.seed));
  }

  const auto maxOuter = arguments.optionalString("--gnc-max-outer");
  gncOptionSpecified |= maxOuter.has_value();
  if (maxOuter) {
    gtsam::timing::Arguments valueArguments({"--value", *maxOuter});
    options.gnc.maxOuterIterations =
        valueArguments.sizeValue("--value", options.gnc.maxOuterIterations);
  }

  const auto jsonPath = arguments.optionalString("--benchmark-action-json");
  options.benchmarkActionJson = jsonPath.has_value();
  options.benchmarkActionJsonPath = jsonPath.value_or("");
  options.filenames = arguments.positionals();
  arguments.validateAllConsumed();
  if (options.help) return options;

  const bool gnc = options.gnc.backend != GncBackend::None;
  const size_t sparseModeCount =
      static_cast<size_t>(options.cudaSparseLm) +
      static_cast<size_t>(options.cudaSparsePackOnly) +
      static_cast<size_t>(options.gaussianGraphPackDiagnostic);
  if (options.listConfigurations || options.dryRun) {
    if (options.profile || options.cudaStructureOnly || options.cudaLm ||
        options.cudaLmGraph || sparseModeCount != 0 ||
        options.linearizationBenchmark ||
        options.gnc.backend != GncBackend::None) {
      throw std::runtime_error(
          "--list-configurations and --dry-run cannot be combined with a "
          "benchmark execution mode");
    }
    options.filenames.clear();
    return options;
  }
  const bool sparseMode = sparseModeCount != 0;
  if (sparseModeCount > 1) {
    throw runtime_error(
        "--cuda-sparse-lm, --cuda-sparse-pack-only, and "
        "--gaussian-graph-pack-diagnostic are mutually exclusive");
  }
  if (sparseMode &&
      (options.profile || options.cudaStructureOnly || options.cudaLm ||
       options.cudaLmGraph || options.linearizationBenchmark ||
       options.benchmarkActionJson || gnc)) {
    throw runtime_error(
        "sparse-Jacobian modes cannot be combined with another run mode");
  }
  if (options.profile && !options.filenames.empty()) {
    throw std::runtime_error(usage());
  }
  if ((options.profile || options.cudaStructureOnly || options.cudaLm ||
       options.cudaLmGraph) &&
      options.benchmarkActionJson) {
    throw std::runtime_error(usage());
  }
  if ((options.cudaLm && options.cudaStructureOnly) ||
      (options.cudaLmGraph && options.cudaStructureOnly) ||
      (options.cudaLm && options.cudaLmGraph)) {
    throw std::runtime_error(usage());
  }
  if (options.linearizationBenchmark &&
      (options.profile || options.cudaStructureOnly || options.cudaLm ||
       options.cudaLmGraph || options.benchmarkActionJson || gnc)) {
    throw std::runtime_error(
        "--linearization-benchmark cannot be combined with another run mode");
  }
  const bool repeatedLinearizationMode = options.linearizationBenchmark ||
                                         options.cudaSparsePackOnly ||
                                         options.gaussianGraphPackDiagnostic;
  if (options.linearizationRepeatsSpecified && !repeatedLinearizationMode) {
    throw std::runtime_error(
        "--linearization-repeats requires --linearization-benchmark, "
        "--cuda-sparse-pack-only, or --gaussian-graph-pack-diagnostic");
  }
  if (repeatedLinearizationMode && options.linearizationRepeats == 0) {
    throw std::runtime_error("--linearization-repeats must be positive");
  }
  if (gnc &&
      (options.cudaLm || options.cudaLmGraph || options.cudaStructureOnly ||
       options.profile || options.benchmarkActionJson)) {
    throw std::runtime_error("--gnc cannot be combined with other run modes");
  }
  if (gncOptionSpecified && !gnc) {
    throw std::runtime_error("--gnc-* options require --gnc cpu|cuda");
  }
  if (gnc && projectionNoiseSpecified) {
    throw std::runtime_error(
        "--projection-noise cannot be combined with --gnc: GNC replaces "
        "robust noise with its own weighting");
  }
  if (options.linearizationBenchmark && projectionNoiseSpecified) {
    throw std::runtime_error(
        "--projection-noise cannot be combined with "
        "--linearization-benchmark");
  }
  if (projectionNoiseSpecified && options.cudaLm) {
    throw std::runtime_error(
        "--projection-noise only applies to factor-graph runs; use "
        "--cuda-lm-graph for CUDA robust-noise benchmarking");
  }
  if (options.cudaLinearSolverSpecified && !options.cudaLm &&
      !options.cudaLmGraph && options.gnc.backend != GncBackend::Cuda) {
    throw std::runtime_error(usage());
  }
  if (options.cudaWarmupFileSpecified && !options.cudaLm &&
      !options.cudaLmGraph && !gnc) {
    throw std::runtime_error(usage());
  }
  if (options.cudaGraphKindSpecified && !options.cudaLmGraph) {
    throw std::runtime_error("--cuda-lm-graph-kind requires --cuda-lm-graph");
  }
  if (options.batchChunkSizeSpecified &&
      (!options.cudaLmGraph ||
       options.cudaGraphKind != CudaGraphKind::PointBatch)) {
    throw std::runtime_error(
        "--batch-chunk-size only applies to --cuda-lm-graph-kind point-batch");
  }

  if (options.filenames.empty()) {
    if (options.profile || options.linearizationBenchmark ||
        options.cudaSparsePackOnly || options.gaussianGraphPackDiagnostic) {
      options.filenames = {bal::profileDataset()};
    } else if (options.benchmarkActionJson || gnc) {
      options.filenames = {bal::defaultDataset()};
    } else {
      options.filenames = bal::standardDatasets();
    }
  }
  return options;
}
double runSolver(const NonlinearFactorGraph& graph, const Values& initial,
                 const Ordering& ordering,
                 NonlinearOptimizerParams::LinearSolverType solverType,
                 const std::string& label,
                 const bal::BalBenchmarkConfig& config) {
  LevenbergMarquardtParams params =
      bal::makeLevenbergMarquardtParams(config, &ordering);
  params.maxIterations = 20;
  params.linearSolverType = solverType;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  }

  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const double elapsed =
      gtsam::timing::measureSeconds([&] { optimizer.optimize(); });
  std::cout << "  " << label << ": " << elapsed << " s\n";
  // SUMMARY rounds costs, so retain this precise machine-readable line.
  std::cout << "Final error: " << std::setprecision(15) << optimizer.error()
            << ", iterations: " << optimizer.iterations()
            << std::setprecision(6) << "\n";
  return elapsed;
}

NonlinearFactorGraph buildCudaGraphSfmGraph(
    const SfmData& data, CudaGraphKind graphKind, size_t batchChunkSize,
    const bal::BalBenchmarkConfig& config) {
  switch (graphKind) {
    case CudaGraphKind::Raw:
      return bal::buildGeneralSfmGraph(data, config);
    case CudaGraphKind::PointBatch:
      return bal::buildBatchSfmGraph(data, config, false, batchChunkSize);
    case CudaGraphKind::CameraBatch:
      return bal::buildCameraBatchSfmGraph(data, config);
  }
  return bal::buildGeneralSfmGraph(data, config);
}
/* ************************************************************************* */
// GNC benchmarking

const char* gncBackendName(GncBackend backend) {
  switch (backend) {
    case GncBackend::Cpu:
      return "cpu";
    case GncBackend::Cuda:
      return "cuda";
    case GncBackend::None:
      break;
  }
  return "none";
}

const char* gncLossName(GncLossType lossType) {
  return lossType == GncLossType::TLS ? "tls" : "gm";
}

// A BAL problem with a fraction of measurements corrupted by a fixed-magnitude
// offset in a random direction. Factor slot i of the graph corresponds to
// isOutlier[i].
struct CorruptedBalProblem {
  SfmData data;  // corrupted copy
  std::vector<bool> isOutlier;
  size_t outlierCount = 0;
};

// Corrupt a deterministic random subset of measurements. Only tracks with
// > 2 measurements are eligible so that every corrupted track keeps at least
// two clean views and remains constrained when GNC removes the outlier.
CorruptedBalProblem corruptBalMeasurements(const SfmData& db,
                                           double outlierFraction,
                                           double outlierPixels,
                                           unsigned int seed) {
  CorruptedBalProblem problem;
  problem.data = db;

  // Global factor slots follow buildGeneralSfmGraph order: tracks with < 2
  // measurements are skipped entirely.
  std::vector<size_t> trackSizes(db.numberTracks(), 0);
  size_t numFactors = 0;
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const size_t n = db.tracks[j].measurements.size();
    trackSizes[j] = n;
    if (n < 2) continue;
    numFactors += n;
  }
  problem.isOutlier.assign(numFactors, false);

  const size_t requested = static_cast<size_t>(
      std::llround(outlierFraction * static_cast<double>(numFactors)));
  const auto selected = gtsam::timing::SelectConstrainedOutlierMeasurements(
      trackSizes, requested, seed);

  std::mt19937 angleRng(seed);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);

  // Corrupt the chosen measurements in the SfmData copy.
  std::vector<std::vector<bool>> corruptByTrack(db.numberTracks());
  for (const auto& [trackIndex, measIndex] : selected) {
    SfmMeasurement& measurement =
        problem.data.tracks[trackIndex].measurements[measIndex];
    const double a = angle(angleRng);
    measurement.second +=
        Point2(outlierPixels * std::cos(a), outlierPixels * std::sin(a));
    if (corruptByTrack[trackIndex].empty()) {
      corruptByTrack[trackIndex].assign(
          db.tracks[trackIndex].measurements.size(), false);
    }
    corruptByTrack[trackIndex][measIndex] = true;
  }
  problem.outlierCount = selected.size();

  // Map (track, measurement) corruption flags to global factor slots.
  size_t slot = 0;
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    const size_t n = db.tracks[j].measurements.size();
    if (n < 2) continue;
    for (size_t m = 0; m < n; ++m, ++slot) {
      if (!corruptByTrack[j].empty() && corruptByTrack[j][m]) {
        problem.isOutlier[slot] = true;
      }
    }
  }
  return problem;
}

struct GncClassificationMetrics {
  size_t truePositives = 0;   // outlier classified as outlier
  size_t falsePositives = 0;  // inlier classified as outlier
  size_t falseNegatives = 0;  // outlier classified as inlier
  double minInlierWeight = 1.0;
  double maxOutlierWeight = 0.0;

  double precision() const {
    const size_t denominator = truePositives + falsePositives;
    return denominator == 0 ? 1.0
                            : static_cast<double>(truePositives) / denominator;
  }
  double recall() const {
    const size_t denominator = truePositives + falseNegatives;
    return denominator == 0 ? 1.0
                            : static_cast<double>(truePositives) / denominator;
  }
};

GncClassificationMetrics classifyGncWeights(
    const Vector& weights, const std::vector<bool>& isOutlier) {
  GncClassificationMetrics metrics;
  for (size_t i = 0; i < isOutlier.size(); ++i) {
    const double w = weights[i];
    const bool classifiedOutlier = w < 0.5;
    if (isOutlier[i]) {
      metrics.maxOutlierWeight = std::max(metrics.maxOutlierWeight, w);
      if (classifiedOutlier) {
        ++metrics.truePositives;
      } else {
        ++metrics.falseNegatives;
      }
    } else {
      metrics.minInlierWeight = std::min(metrics.minInlierWeight, w);
      if (classifiedOutlier) {
        ++metrics.falsePositives;
      }
    }
  }
  return metrics;
}

// Inlier-only RMS reprojection error (pixels) of the solution.
double inlierRmsReprojectionError(const NonlinearFactorGraph& graph,
                                  const std::vector<bool>& isOutlier,
                                  const Values& solution) {
  double sumSquared = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < graph.size(); ++i) {
    if (isOutlier[i]) continue;
    // error() is 0.5 * ||r||^2 for the unit-noise BAL factors.
    sumSquared += 2.0 * graph.at(i)->error(solution);
    ++count;
  }
  return count == 0 ? 0.0 : std::sqrt(sumSquared / static_cast<double>(count));
}

struct GncBenchmarkRun {
  double elapsed = 0.0;
  GncTiming timing;
  Vector weights;
  Values solution;
  size_t outerIterations = 0;
};

template <typename BaseParams>
GncBenchmarkRun runGncBenchmark(const NonlinearFactorGraph& graph,
                                const Values& initial,
                                const BaseParams& baseParams,
                                const GncRunOptions& gncOptions) {
  GncParams<BaseParams> gncParams(baseParams);
  gncParams.setLossType(gncOptions.lossType);
  gncParams.enableTiming = true;
  if (gncOptions.maxOuterIterations != gncParams.maxIterations) {
    gncParams.maxIterations = gncOptions.maxOuterIterations;
  }

  GncBenchmarkRun run;
  std::optional<GncOptimizer<GncParams<BaseParams>>> optimizer;
  run.elapsed = gtsam::timing::measureSeconds([&] {
    optimizer.emplace(graph, initial, gncParams);
    run.solution = optimizer->optimize();
  });
  run.timing = optimizer->getTiming();
  run.weights = optimizer->getWeights();
  run.outerIterations = run.timing.iterations.size();
  return run;
}

void printGncRun(const GncBenchmarkRun& run, const CorruptedBalProblem& problem,
                 const NonlinearFactorGraph& graph,
                 const GncRunOptions& gncOptions) {
  const GncTiming& timing = run.timing;
  std::cout << "  GNC (" << gncBackendName(gncOptions.backend) << ", "
            << gncLossName(gncOptions.lossType) << "): " << run.elapsed
            << " s\n";
  std::cout << "  GNC outer iterations: " << run.outerIterations
            << " (+1 initial optimize)\n";
  std::cout << "  GNC timing breakdown:\n";
  printProfileRow("    ", "initial optimize", timing.initialOptimizeElapsed,
                  timing.totalElapsed);
  printProfileRow("    ", "weights update (factor errors)",
                  timing.sumWeightsUpdate(), timing.totalElapsed);
  printProfileRow("    ", "weighted graph rebuild", timing.sumMakeGraph(),
                  timing.totalElapsed);
  printProfileRow("    ", "base optimize calls", timing.sumBaseOptimize(),
                  timing.totalElapsed);
  printProfileRow("    ", "cost evaluation", timing.sumCostEvaluation(),
                  timing.totalElapsed);
  std::cout << "  GNC per-outer-iteration timings:\n";
  for (size_t i = 0; i < timing.iterations.size(); ++i) {
    const GncIterationTiming& it = timing.iterations[i];
    std::cout << "    outer " << i << ": total " << it.totalElapsed
              << " s (weights " << it.weightsUpdateElapsed << ", graph "
              << it.makeGraphElapsed << ", optimize " << it.baseOptimizeElapsed
              << ", cost " << it.costEvaluationElapsed << ")\n";
  }

  const GncClassificationMetrics metrics =
      classifyGncWeights(run.weights, problem.isOutlier);
  std::cout << "  GNC injected outliers: " << problem.outlierCount << " / "
            << problem.isOutlier.size() << " measurements ("
            << gncOptions.outlierPixels << " px, seed " << gncOptions.seed
            << ")\n";
  std::cout << "  GNC classification: precision " << metrics.precision()
            << ", recall " << metrics.recall() << " (TP "
            << metrics.truePositives << ", FP " << metrics.falsePositives
            << ", FN " << metrics.falseNegatives << ")\n";
  std::cout << "  GNC weight separation: min inlier weight "
            << metrics.minInlierWeight << ", max outlier weight "
            << metrics.maxOutlierWeight << "\n";
  std::cout << "Inlier RMS reprojection error: " << std::setprecision(15)
            << inlierRmsReprojectionError(graph, problem.isOutlier,
                                          run.solution)
            << " px" << std::setprecision(6) << "\n";
}

struct MatrixConfigurationRecord {
  const char* name;
  const char* formulation;
  const char* backend;
  const char* ordering;
};

constexpr std::array<MatrixConfigurationRecord, 7> kMatrixConfigurations{{
    {"schur-dense", "schur", "dense-cholesky", "auto"},
    {"schur-cudss-auto", "schur", "cudss", "auto"},
    {"schur-cudss-gtsam", "schur", "cudss", "gtsam"},
    {"schur-pcg", "schur", "pcg", "auto"},
    {"full-normal-cudss-auto", "full-normal", "cudss", "auto"},
    {"full-normal-cudss-gtsam", "full-normal", "cudss", "gtsam"},
    {"full-normal-pcg", "full-normal", "pcg", "auto"},
}};

void printMatrixConfigurations() {
  for (const MatrixConfigurationRecord& record : kMatrixConfigurations) {
    std::cout << record.name << "\n";
  }
}

void printMatrixDryRun(const RunOptions& options) {
  const auto jsonRecord = [](const MatrixConfigurationRecord& record) {
    std::cout
        << "{\"configuration\":\"" << record.name
        << "\",\"formulation\":\"" << record.formulation
        << "\",\"backend\":\"" << record.backend
        << "\",\"ordering\":\"" << record.ordering
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
    for (size_t index = 0; index < kMatrixConfigurations.size(); ++index) {
      if (index != 0) std::cout << ",";
      jsonRecord(kMatrixConfigurations[index]);
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
    for (const MatrixConfigurationRecord& record : kMatrixConfigurations) {
      std::cout << record.name << "," << record.formulation << ","
                << record.backend << "," << record.ordering
                << ",0,0,0,0,0,0,false,0,0,0,0,0,0,0,0,0,0,true\n";
    }
    return;
  }
  for (const MatrixConfigurationRecord& record : kMatrixConfigurations) {
    std::cout << "configuration=" << record.name
              << " formulation=" << record.formulation
              << " backend=" << record.backend
              << " ordering=" << record.ordering << " dry_run=true\n";
  }
}
}  // namespace

int RunMain(int argc, char* argv[]) {
  const auto options = parseBalFiles(argc, argv);
  if (options.help) {
    std::cout << usage() << '\n';
    return 0;
  }
  if (options.listConfigurations) {
    printMatrixConfigurations();
    return 0;
  }
  if (options.dryRun) {
    printMatrixDryRun(options);
    return 0;
  }
  std::vector<TimingRow> rows;
#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
  bool cudaWarmupDone = false;
  bool cudaMatrixCsvHeaderPending = true;
#endif

  for (const auto& filename : options.filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    if (options.outputFormat == "text") {
      std::cout << "\nProcessing BAL file: " << filename << std::endl;
    }
    const SfmData db = bal::loadDataset(filename);

    if (options.gnc.backend != GncBackend::None) {
      const CorruptedBalProblem problem =
          corruptBalMeasurements(db, options.gnc.outlierFraction,
                                 options.gnc.outlierPixels, options.gnc.seed);
      const NonlinearFactorGraph graph =
          bal::buildGeneralSfmGraph(problem.data, options.config);
      const Values initial = bal::buildGeneralSfmInitial(problem.data);
      std::cout << "  GNC problem: " << graph.size() << " factors, "
                << problem.outlierCount << " corrupted\n";

      if (options.gnc.backend == GncBackend::Cpu) {
        const GncBenchmarkRun run = runGncBenchmark(
            graph, initial, makeBalLevenbergMarquardtParams(options.config),
            options.gnc);
        printGncRun(run, problem, graph, options.gnc);
        continue;
      }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
      auto cudaParams = makeBalCudaLmParams(options.cudaLinearSolver,
                                            CudaLmDefaults::Graph, false);
      applyCudaMatrixOrdering(problem.data, options, &cudaParams);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA GNC warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
        auto warmupParams = makeBalCudaLmParams(
            options.cudaLinearSolver, CudaLmDefaults::Graph, false);
        applyCudaMatrixOrdering(warmupDb, options, &warmupParams);
        const CudaBackendLmRun warmupRun =
            runCudaBackendLm(warmupDb, warmupParams);
        std::cout << "  CUDA GNC warmup: " << warmupRun.elapsed
                  << " s ignored\n";
        cudaWarmupDone = true;
      }

      // Reference single CUDA LM solve on the corrupted problem: the gap
      // between (outer iterations x this) and the GNC total is the per-outer
      // re-conversion/re-upload overhead that a device-resident GNC removes.
      const CudaGraphLmRun referenceLm =
          runCudaGraphLm(graph, initial, cudaParams);
      std::cout << "  CUDA LM single solve (reference): " << referenceLm.elapsed
                << " s (backend measured "
                << referenceLm.backend.totalMeasuredElapsed << " s, setup "
                << referenceLm.backend.setupElapsed << " s, solve loop "
                << referenceLm.backend.solveLoopElapsed << " s)\n";

      const GncBenchmarkRun run =
          runGncBenchmark(graph, initial, cudaParams, options.gnc);
      printGncRun(run, problem, graph, options.gnc);
      continue;
#else
      throw std::runtime_error(
          "--gnc cuda requires configuring with GTSAM_ENABLE_CUDA=ON and "
          "GTSAM_ENABLE_CUDSS=ON");
#endif
    }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaLm) {
      auto cudaParams = makeBalCudaLmParams(
          options.cudaLinearSolver, CudaLmDefaults::Backend,
          options.profile || options.outputFormat != "text");
      applyCudaMatrixOrdering(db, options, &cudaParams);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        if (options.outputFormat == "text") {
          std::cout << "  CUDA warmup file: " << options.cudaWarmupFile
                    << " (timing ignored)\n";
        }
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
        auto warmupParams = makeBalCudaLmParams(
            options.cudaLinearSolver, CudaLmDefaults::Backend,
            options.profile || options.outputFormat != "text");
        applyCudaMatrixOrdering(warmupDb, options, &warmupParams);
        const CudaBackendLmRun warmupRun =
            runCudaBackendLm(warmupDb, warmupParams);
        if (options.outputFormat == "text") {
          std::cout << "  CUDA warmup: " << warmupRun.elapsed
                    << " s ignored, final error: " << std::setprecision(15)
                    << warmupRun.result.finalError
                    << ", iterations: " << warmupRun.result.iterations
                    << std::setprecision(6) << "\n";
        }
        cudaWarmupDone = true;
      }

      const CudaBackendLmRun run = runCudaBackendLm(db, cudaParams);
      if (options.outputFormat == "text") {
        printCudaBackendLmRun(run, options.cudaLinearSolver, options.profile);
      } else {
        printCudaMatrixResult(options, dataset, run,
                              cudaMatrixCsvHeaderPending);
        cudaMatrixCsvHeaderPending = false;
      }
      continue;
    }
#elif GTSAM_ENABLE_CUDA
    if (options.cudaLm || options.cudaLmGraph) {
      throw std::runtime_error(
          "--cuda-lm and --cuda-lm-graph require configuring with "
          "GTSAM_ENABLE_CUDSS=ON");
    }
#else
    if (options.cudaLm || options.cudaLmGraph) {
      throw std::runtime_error(
          "--cuda-lm and --cuda-lm-graph require configuring with "
          "GTSAM_ENABLE_CUDA=ON and GTSAM_ENABLE_CUDSS=ON");
    }
#endif

#if GTSAM_ENABLE_CUDA
    if (options.cudaStructureOnly) {
      gtsam::cuda::CudaContext context;
      const auto values = gtsam::cuda::PackSfmValues(db, context.stream());
      const auto batch = gtsam::cuda::CudaSfmProjectionBatch::FromSfmData(
          db, context.stream());
      const auto csr = gtsam::cuda::CudaBalCsrStructure::FromSfmData(db);
      context.synchronize();

      std::cout << "CUDA BAL structure: cameras=" << batch.numCameras()
                << " points=" << batch.numPoints()
                << " observations=" << batch.numObservations()
                << " packed_values=" << values.index().size()
                << " dimension=" << csr.dimension()
                << " csr_nnz=" << csr.colIndices().size() << std::endl;
      continue;
    }
#else
    if (options.cudaStructureOnly) {
      throw std::runtime_error(
          "--cuda-structure-only requires configuring with "
          "GTSAM_ENABLE_CUDA=ON");
    }
#endif

    NonlinearFactorGraph graph = buildCudaGraphSfmGraph(
        db, options.cudaGraphKind, options.batchChunkSize, options.config);
    Values initial = bal::buildGeneralSfmInitial(db);

#if GTSAM_ENABLE_CUDA
    if (options.cudaSparsePackOnly || options.gaussianGraphPackDiagnostic) {
      const SparsePackingPath path = options.cudaSparsePackOnly
                                         ? SparsePackingPath::Streaming
                                         : SparsePackingPath::GaussianGraph;
      const SparsePackingBenchmark benchmark = benchmarkSparsePacking(
          graph, initial, options.linearizationRepeats, path);
      printSparsePackingBenchmark(benchmark, path,
                                  options.linearizationRepeats);
      continue;
    }
#else
    if (options.cudaSparsePackOnly || options.gaussianGraphPackDiagnostic) {
      throw std::runtime_error(
          "sparse-Jacobian packing modes require configuring with "
          "GTSAM_ENABLE_CUDA=ON");
    }
#endif

    if (options.linearizationBenchmark) {
      if (!gtsamBuiltWithTbb()) {
        throw std::runtime_error(
            "--linearization-benchmark requires GTSAM_USE_TBB");
      }
      std::cout << std::setprecision(9);
      std::cout << "  SFM linearization feasibility benchmark\n";
      std::cout << "  graph representation: raw GeneralSFMFactor\n";
      std::cout << "  GTSAM_USE_TBB: " << (gtsamBuiltWithTbb() ? "yes" : "no")
                << "\n";
      std::cout << "  cameras: " << db.numberCameras()
                << ", points: " << db.numberTracks()
                << ", observations/factors: " << graph.size()
                << ", values: " << initial.size() << "\n";
      std::cout << "  warm-up calls per path: 1 (not timed)\n";
      std::cout << "  measured repeats: " << options.linearizationRepeats
                << "\n";

      const CpuLinearizationBenchmark cpu = benchmarkCpuLinearization(
          graph, initial, options.linearizationRepeats);
      std::cout << "  CPU output linear factors: " << cpu.linearizedFactors
                << "\n";
      printTimingSamples("CPU/TBB graph.linearize()", cpu.timing, graph.size(),
                         "factors");
      const VectorValues delta = makeBenchmarkDelta(initial);
      const CpuStateBenchmark cpuState = benchmarkCpuState(
          graph, initial, delta, options.linearizationRepeats);
      std::cout << "  CPU state validation: retracted values: "
                << cpuState.retractedValues
                << ", trial error: " << cpuState.error << "\n";
      printTimingSamples("CPU Values retract()", cpuState.retract,
                         initial.size(), "values");
      printTimingSamples("CPU trial graph.error()", cpuState.trialError,
                         graph.size(), "factors");

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
      const GpuLinearizationBenchmark gpu =
          benchmarkGpuLinearization(db, options.linearizationRepeats);
      std::cout << "  GPU output observations: " << gpu.observations
                << ", delta dimension: " << gpu.deltaDimension << "\n";
      printTimingSamples("GPU projection residual/Jacobian generation",
                         gpu.projectionLinearization, gpu.observations,
                         "observations");
      if (!gpu.hessianDiagonal.seconds.empty()) {
        printTimingSamples("GPU Hessian diagonal generation",
                           gpu.hessianDiagonal, gpu.observations,
                           "observations");
      }
      printTimingSamples(
          "GPU dense Schur combined (linearize + Schur/Hessian + solve + "
          "recover; precomputed damping diagonal)",
          gpu.denseSchurCombined, gpu.observations, "observations");
      std::cout << "  comparison\n";
      std::cout << "    CPU linearize / GPU projection linearize: "
                << cpu.timing.mean() / gpu.projectionLinearization.mean()
                << "x\n";
      if (!gpu.hessianDiagonal.seconds.empty()) {
        std::cout << "    CPU linearize / GPU Hessian diagonal: "
                  << cpu.timing.mean() / gpu.hessianDiagonal.mean() << "x\n";
      }
      std::cout << "    CPU linearize / GPU dense Schur combined: "
                << cpu.timing.mean() / gpu.denseSchurCombined.mean() << "x\n";
#else
      throw std::runtime_error(
          "--linearization-benchmark requires configuring with "
          "GTSAM_ENABLE_CUDA=ON and GTSAM_ENABLE_CUDSS=ON");
#endif
      std::cout << std::setprecision(6);
      continue;
    }

    Ordering ordering;
    if (options.config.useSchur) {
      ordering = bal::createSchurOrdering(db, false);
    }

#if GTSAM_ENABLE_CUDA && GTSAM_ENABLE_CUDSS
    if (options.cudaSparseLm) {
      gtsam::cuda::CudaSparseLevenbergMarquardtParams sparseParams;
      LevenbergMarquardtParams::SetCeresDefaults(&sparseParams);
      applyBalBenchmarkLmSettings(sparseParams);
      sparseParams.fallbackOnUnsupported = false;
      sparseParams.collectTiming = true;
      sparseParams.setVerbosityLM("SILENT");
      if (options.config.useSchur) sparseParams.setOrdering(ordering);

      const LevenbergMarquardtParams ordinaryParams = sparseParams;
      const auto specializedParams = makeBalCudaLmParams(
          CudaLinearSolverOption::DenseSchur, CudaLmDefaults::Graph, true);

      // Warm every implementation once before measuring so library, allocator,
      // and GPU first-use costs do not land asymmetrically in one CUDA row.
      (void)runOrdinaryLmComparison(graph, initial, ordinaryParams);
      (void)runCudaGraphLm(graph, initial, specializedParams);
      (void)runGenericSparseLm(graph, initial, sparseParams);

      const OrdinaryLmComparisonRun ordinary =
          runOrdinaryLmComparison(graph, initial, ordinaryParams);
      const CudaGraphLmRun specialized =
          runCudaGraphLm(graph, initial, specializedParams);
      const GenericSparseLmRun generic =
          runGenericSparseLm(graph, initial, sparseParams);
      printSparseLmComparison(ordinary, specialized, generic);
      continue;
    }

    if (options.cudaLmGraph) {
      auto cudaParams = makeBalCudaLmParams(
          options.cudaLinearSolver, CudaLmDefaults::Graph,
          options.profile || options.outputFormat != "text");
      applyCudaMatrixOrdering(db, options, &cudaParams);

      if (options.cudaWarmupFileSpecified && !cudaWarmupDone) {
        std::cout << "  CUDA graph warmup file: " << options.cudaWarmupFile
                  << " (timing ignored)\n";
        const SfmData warmupDb = bal::loadDataset(options.cudaWarmupFile);
        auto warmupParams = makeBalCudaLmParams(
            options.cudaLinearSolver, CudaLmDefaults::Graph,
            options.profile || options.outputFormat != "text");
        applyCudaMatrixOrdering(warmupDb, options, &warmupParams);
        const NonlinearFactorGraph warmupGraph =
            buildCudaGraphSfmGraph(warmupDb, options.cudaGraphKind,
                                   options.batchChunkSize, options.config);
        const Values warmupInitial = bal::buildGeneralSfmInitial(warmupDb);
        const CudaGraphLmRun warmupRun =
            runCudaGraphLm(warmupGraph, warmupInitial, warmupParams);
        std::cout << "  CUDA graph warmup: " << warmupRun.elapsed
                  << " s ignored, final error: " << std::setprecision(15)
                  << warmupRun.finalError
                  << ", iterations: " << warmupRun.iterations
                  << std::setprecision(6) << "\n";
        cudaWarmupDone = true;
      }

      const CudaGraphLmRun run = runCudaGraphLm(graph, initial, cudaParams);
      if (options.outputFormat == "text") {
        printCudaGraphLmRun(run, options.cudaLinearSolver,
                            options.cudaGraphKind, options.profile);
      } else {
        const CudaBackendLmRun backendRun{run.elapsed, run.backend};
        printCudaMatrixResult(options, dataset, backendRun,
                              cudaMatrixCsvHeaderPending);
        cudaMatrixCsvHeaderPending = false;
      }
      continue;
    }
#else
    if (options.cudaSparseLm) {
      throw std::runtime_error(
          "--cuda-sparse-lm requires configuring with GTSAM_ENABLE_CUDA=ON "
          "and GTSAM_ENABLE_CUDSS=ON");
    }
#endif

    const double newTime = runSolver(
        graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
        "MultifrontalSolver", options.config);
    double legacyTime = 0.0;
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky", options.config);
    }

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, newTime});
    }
  }
  if (!options.profile && !options.cudaStructureOnly && !options.cudaLm &&
      !options.cudaLmGraph && !options.cudaSparseLm &&
      !options.cudaSparsePackOnly && !options.gaussianGraphPackDiagnostic &&
      !options.linearizationBenchmark &&
      options.gnc.backend == GncBackend::None) {
    std::cout
        << "\n| Dataset | Legacy (Cholesky) s | New (Solver) s | Speedup |\n";
    std::cout << "| --- | --- | --- | --- |\n";
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : rows) {
      const double speedup = row.newer > 0.0 ? (row.legacy / row.newer) : 0.0;
      std::cout << "| " << row.dataset << " | " << row.legacy << " | "
                << row.newer << " | " << speedup << "x |\n";
    }
  }

  if (options.benchmarkActionJson && !options.cudaStructureOnly &&
      !options.cudaLm && !options.cudaLmGraph && !options.cudaSparseLm &&
      !options.cudaSparsePackOnly && !options.gaussianGraphPackDiagnostic &&
      !options.linearizationBenchmark) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}

int main(int argc, char* argv[]) {
  try {
    return RunMain(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "timeCudaSFMBAL: " << error.what() << "\n";
    return 1;
  }
}
